// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver.h"
#include "contact_solver.h"
#include "world.h"
#include "shape.h"
#include "box2d/math_functions.h"


// Continuous collision uses a local frame centered on the fast body's previous center.
struct b2ContinuousContext
{
    b2World* world;
    b2BodySim* fastBodySim;
    b2Shape* fastShape;
    b2Vec2 centroid1,centroid2;
    b2Sweep sweep;
    b2Pos base;
    float fraction;
};
#define B2_CORE_FRACTION 0.25f

static bool b2ContinuousQueryCallback(int proxyId,uint64_t userData,void* context)
{
    (void)proxyId;
    b2ContinuousContext* continuousContext=static_cast<b2ContinuousContext*>(context);
    b2World* world=continuousContext->world;
    b2Shape* fastShape=continuousContext->fastShape;
    b2BodySim* fastBodySim=continuousContext->fastBodySim;
    b2Shape* shape=world->shapes.data+(int)userData;
    if(shape->id==fastShape->id || shape->bodyId==fastBodySim->bodyId || !b2ShouldShapesCollide(fastShape->filter,shape->filter)) { return true; }
    b2Body* body=world->bodies.data+shape->bodyId;
    B2_ASSERT(body->type==b2_staticBody || (fastBodySim->flags&b2_isBullet));
    if(body->flags&b2_isBullet) { return true; }
    b2BodySim* bodySim=b2GetBodySim(world,body);
	// 沿链条形状快速平行运动时提前退出。
	if ( shape->type == b2_chainSegmentShape )
	{
		b2Transform transform = b2Transform{b2Sub(bodySim->transform.p,continuousContext->base),bodySim->transform.q};
		b2Vec2 p1 = b2TransformPoint( transform, shape->chainSegment.segment.point1 );
		b2Vec2 p2 = b2TransformPoint( transform, shape->chainSegment.segment.point2 );
		b2Vec2 e = b2Sub( p2, p1 );
		float length;
		e = b2GetLengthAndNormalize( &length, e );
		if ( length > B2_LINEAR_SLOP )
		{
			b2Vec2 c1 = continuousContext->centroid1;
			float separation1 = b2Cross( b2Sub( c1, p1 ), e );
			b2Vec2 c2 = continuousContext->centroid2;
			float separation2 = b2Cross( b2Sub( c2, p1 ), e );

			float coreDistance = B2_CORE_FRACTION * fastBodySim->minExtent;

			if ( separation1 < 0.0f || ( separation1 - separation2 < coreDistance && separation2 > coreDistance ) )
			{
				// 最小裁剪
				return true;
			}
		}
	}

    b2TOIInput input;
    input.proxyA=b2MakeShapeDistanceProxy(shape);input.proxyB=b2MakeShapeDistanceProxy(fastShape);
    input.sweepA=b2MakeRelativeSweep(bodySim,continuousContext->base);input.sweepB=continuousContext->sweep;
    input.maxFraction=continuousContext->fraction;
    b2TOIOutput output=b2ComputeTimeOfImpact(&input);
    float hitFraction=continuousContext->fraction;bool didHit=false;
    if(0.0f<output.fraction && output.fraction<continuousContext->fraction)
    { hitFraction=output.fraction;didHit=true; }
    else if(output.fraction==0.0f)
    {
        // The reference fallback uses a small core circle at the fast shape's centroid.
        b2Vec2 centroid=b2GetShapeCentroid(fastShape);b2ShapeExtent extent=b2ComputeShapeExtent(fastShape,centroid);
        input.proxyB=b2MakeProxy(&centroid,1,B2_CORE_FRACTION*extent.minExtent);
        output=b2ComputeTimeOfImpact(&input);
        if(0.0f<output.fraction && output.fraction<continuousContext->fraction)
        { hitFraction=output.fraction;didHit=true; }
    }
    if(didHit) { continuousContext->fraction=hitFraction; }
    return true;
}

static void b2SolveContinuous(b2World* world,int bodySimIndex,float dt)
{
    b2SolverSet* awake=world->solverSets.data+b2_awakeSet;
    b2BodySim* fastBodySim=awake->bodySims.data+bodySimIndex;
    b2Pos base=fastBodySim->center0;b2Sweep sweep=b2MakeRelativeSweep(fastBodySim,base);
    b2Transform xf1={b2Sub(sweep.c1,b2RotateVector(sweep.q1,sweep.localCenter)),sweep.q1};
    b2Transform xf2={b2Sub(sweep.c2,b2RotateVector(sweep.q2,sweep.localCenter)),sweep.q2};
    b2Body* fastBody=world->bodies.data+fastBodySim->bodyId;
    b2ContinuousContext context={};context.world=world;context.sweep=sweep;context.base=base;
    context.fastBodySim=fastBodySim;context.fraction=1.0f;
    for(int shapeId=fastBody->headShapeId;shapeId!=B2_NULL_INDEX;shapeId=world->shapes.data[shapeId].nextShapeId)
    {
        b2Shape* shape=world->shapes.data+shapeId;context.fastShape=shape;
        b2Vec2 centroid=b2GetShapeCentroid(shape);
        context.centroid1=b2TransformPoint(xf1,centroid);context.centroid2=b2TransformPoint(xf2,centroid);
        b2AABB box1=shape->aabb,box2=b2ComputeShapeAABB(shape,xf2);
        box2.lowerBound=b2Add(box2.lowerBound,base);box2.upperBound=b2Add(box2.upperBound,base);
        shape->aabb=box2;
        b2AABB sweptBox=b2AABB_Union(box1,box2);
        b2DynamicTree_OverlapAABB(world->broadPhase.trees+b2_staticBody,sweptBox,UINT64_MAX,b2ContinuousQueryCallback,&context);
        if(fastBodySim->flags&b2_isBullet)
        {
            b2DynamicTree_OverlapAABB(world->broadPhase.trees+b2_kinematicBody,sweptBox,UINT64_MAX,b2ContinuousQueryCallback,&context);
            b2DynamicTree_OverlapAABB(world->broadPhase.trees+b2_dynamicBody,sweptBox,UINT64_MAX,b2ContinuousQueryCallback,&context);
        }
    }
    if(context.fraction<1.0f)
    {
        b2Rot q=b2NLerp(sweep.q1,sweep.q2,context.fraction);b2Vec2 c=b2Lerp(sweep.c1,sweep.c2,context.fraction);
        fastBodySim->transform.q=q;fastBodySim->transform.p=b2Add(base,b2Sub(c,b2RotateVector(q,sweep.localCenter)));
        fastBodySim->center=b2Add(base,c);fastBodySim->flags|=b2_hadTimeOfImpact;
        b2BodyState* state=awake->bodyStates.data+bodySimIndex;
        float timeLoss=(1.0f-context.fraction)*dt;
        b2Vec2 dv=b2MulSV(-timeLoss*fastBodySim->gravityScale,world->gravity);
        dv.x=(state->flags&b2_lockLinearX)?0.0f:dv.x;dv.y=(state->flags&b2_lockLinearY)?0.0f:dv.y;
        state->linearVelocity=b2Add(state->linearVelocity,dv);
    }
    fastBodySim->center0=fastBodySim->center;fastBodySim->rotation0=fastBodySim->transform.q;
    // The existing proxy synchronization below recomputes bounds at the clipped transform.
}

static void b2IntegrateBodyVelocity( b2World* world, b2BodySim* sim, b2BodyState* state, float h )
{
    float gravityScale=sim->invMass>0.0f?sim->gravityScale:0.0f;
    b2Vec2 delta=b2Add(b2MulSV(h*sim->invMass,sim->force),b2MulSV(h*gravityScale,world->gravity));
    float linearDamping=1.0f/(1.0f+h*sim->linearDamping);
    float angularDamping=1.0f/(1.0f+h*sim->angularDamping);
    state->linearVelocity=b2MulAdd(delta,linearDamping,state->linearVelocity);
    state->angularVelocity=h*sim->invInertia*sim->torque+angularDamping*state->angularVelocity;
}

void b2SolveFreeBodies( b2World* world, float timeStep, int subStepCount )
{
    float h=timeStep/(float)subStepCount;
    b2SolverSet* set=world->solverSets.data+b2_awakeSet;
    B2_ASSERT(set->bodySims.count==set->bodyStates.count);
    b2StepContext context={world,&world->constraintGraph,set->bodyStates.data,h,1.0f/h};
    b2PrepareContactConstraints(&context);
    for(int step=0;step<subStepCount;++step)
    {
        for(int i=0;i<set->bodySims.count;++i)
        { b2IntegrateBodyVelocity(world,set->bodySims.data+i,set->bodyStates.data+i,h); }
        b2WarmStartContacts(&context);
        b2PushNormalContacts(&context);
        for(int i=0;i<set->bodySims.count;++i)
        {
            b2BodyState* state=set->bodyStates.data+i;
            // Motion locks act as the final constraint before position integration.
            state->linearVelocity.x=(state->flags&b2_lockLinearX)?0.0f:state->linearVelocity.x;
            state->linearVelocity.y=(state->flags&b2_lockLinearY)?0.0f:state->linearVelocity.y;
            state->angularVelocity=(state->flags&b2_lockAngularZ)?0.0f:state->angularVelocity;
            float maxSpeed=world->maxLinearSpeed;
            if(b2LengthSquared(state->linearVelocity)>maxSpeed*maxSpeed)
            {
                float ratio=maxSpeed/b2Length(state->linearVelocity);
                state->linearVelocity=b2MulSV(ratio,state->linearVelocity);state->flags|=b2_isSpeedCapped;
            }
            float maxAngularSpeed=B2_MAX_ROTATION/timeStep;
            if(state->angularVelocity*state->angularVelocity>maxAngularSpeed*maxAngularSpeed && (state->flags&b2_allowFastRotation)==0)
            {
                float ratio=maxAngularSpeed/b2AbsFloat(state->angularVelocity);
                state->angularVelocity*=ratio;state->flags|=b2_isSpeedCapped;
            }
            state->deltaPosition=b2MulAdd(state->deltaPosition,h,state->linearVelocity);
            state->deltaRotation=b2IntegrateRotation(state->deltaRotation,h*state->angularVelocity);
        }
        b2SolveNormalContacts(&context);
    }
    b2ApplyContactRestitution(&context);
    b2StoreContactImpulses(&context);
    b2ReleaseContactConstraints(&context);
    int bulletCount=0;int* bulletIndices=NULL;
    if(world->enableContinuous && set->bodySims.count>0)
    { bulletIndices=static_cast<int*>(b2StackAlloc(&world->stack,set->bodySims.count*sizeof(int),"bullets")); }
    for(int i=0;i<set->bodySims.count;++i)
    {
        b2BodySim* sim=set->bodySims.data+i;
        b2BodyState* state=set->bodyStates.data+i;
        sim->center=b2Add(sim->center,state->deltaPosition);
        sim->transform.q=b2NormalizeRot(b2MulRot(state->deltaRotation,sim->transform.q));
        sim->transform.p=b2Sub(sim->center,b2RotateVector(sim->transform.q,sim->localCenter));
        float maxVelocity=b2Length(state->linearVelocity)+b2AbsFloat(state->angularVelocity)*sim->maxExtent;
        float maxDeltaPosition=b2Length(state->deltaPosition)+b2AbsFloat(state->deltaRotation.s)*sim->maxExtent;
        float sleepVelocity=b2MaxFloat(maxVelocity,0.5f/timeStep*maxDeltaPosition);
        b2Body* body=world->bodies.data+sim->bodyId;
        if(!world->enableSleep || (body->flags&b2_enableSleep)==0 || sleepVelocity>body->sleepThreshold) { body->sleepTime=0; }
        else { body->sleepTime+=timeStep; }
        body->flags&=~b2_bodyTransientFlags;body->flags|=state->flags&b2_isSpeedCapped;
        state->flags&=~b2_bodyTransientFlags;
        sim->flags&=~(b2_isFast|b2_bodyTransientFlags);
        float maxMotion=b2MaxFloat(maxDeltaPosition,maxVelocity*timeStep);
        bool deferBullet=false;
        if(body->type==b2_dynamicBody && world->enableContinuous && maxMotion>body->safetyFactor*sim->minExtent)
        {
            sim->flags|=b2_isFast;deferBullet=(sim->flags&b2_isBullet)!=0;
            if(deferBullet) { bulletIndices[bulletCount++]=i; }
            else { b2SolveContinuous(world,i,timeStep); }
        }
        else { sim->center0=sim->center;sim->rotation0=sim->transform.q; }
        state->deltaPosition=b2Vec2_zero;state->deltaRotation=b2Rot_identity;
        sim->force=b2Vec2_zero;sim->torque=0.0f;
        if(!deferBullet)
        {
            for(int shapeId=world->bodies.data[sim->bodyId].headShapeId;shapeId!=B2_NULL_INDEX;shapeId=world->shapes.data[shapeId].nextShapeId)
            { b2MoveShapeProxy(world,world->shapes.data+shapeId,sim->transform); }
        }
    }
    for(int i=0;i<bulletCount;++i)
    {
        int index=bulletIndices[i];b2SolveContinuous(world,index,timeStep);
        b2BodySim* sim=set->bodySims.data+index;
        for(int shapeId=world->bodies.data[sim->bodyId].headShapeId;shapeId!=B2_NULL_INDEX;shapeId=world->shapes.data[shapeId].nextShapeId)
        { b2MoveShapeProxy(world,world->shapes.data+shapeId,sim->transform); }
    }
    if(bulletIndices!=NULL) { b2StackFree(&world->stack,bulletIndices); }
    if(world->enableSleep)
    {
        // Reverse order is required because sleeping swap-removes island descriptors.
        int count=world->solverSets.data[b2_awakeSet].islandSims.count;
        for(int i=count-1;i>=0;--i)
        {
            int islandId=world->solverSets.data[b2_awakeSet].islandSims.data[i].islandId;
            b2Island* island=world->islands.data+islandId;bool ready=true;
            for(int j=0;j<island->bodies.count;++j)
            { if(world->bodies.data[island->bodies.data[j]].sleepTime<B2_TIME_TO_SLEEP) { ready=false;break; } }
            if(ready) { b2TrySleepIsland(world,islandId); }
        }
    }

}

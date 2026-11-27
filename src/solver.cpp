// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver.h"
#include "contact_solver.h"
#include "world.h"
#include "shape.h"
#include "platform.h"
#include "simd.h"
#include "aabb.h"
#include "box2d/math_functions.h"

#include <bit>


// Continuous collision uses a local frame centered on the fast body's previous center.
struct b2ContinuousContext
{
    b2SensorHit sensorHits[8];
    float sensorFractions[8];
    int sensorCount;
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
    bool isSensor=shape->sensorIndex!=B2_NULL_INDEX;
    if(isSensor && (!shape->enableSensorEvents || !fastShape->enableSensorEvents)) { return true; }
    b2Body* body=world->bodies.data+shape->bodyId;
    B2_ASSERT(body->type==b2_staticBody || (fastBodySim->flags&b2_isBullet));
    if(body->flags&b2_isBullet) { return true; }
    if(!b2ShouldBodiesCollide(world,world->bodies.data+fastBodySim->bodyId,body)) { return true; }
    b2BodySim* bodySim=b2GetBodySim(world,body);
    if((shape->enableCustomFiltering || fastShape->enableCustomFiltering) && world->customFilterFcn!=NULL)
    {
        b2ShapeId idA={shape->id+1,world->worldId,shape->generation},idB={fastShape->id+1,world->worldId,fastShape->generation};
        if(!world->customFilterFcn(idA,idB,world->customFilterContext)) { return true; }
    }
	// 沿链条形状快速平行运动时提前退出。
	if ( shape->type == b2_chainSegmentShape )
	{
		b2Transform transform = b2Transform{b2SubPos(bodySim->transform.p,continuousContext->base),bodySim->transform.q};
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
    if(isSensor)
    {
        if(output.fraction<=continuousContext->fraction && continuousContext->sensorCount<8)
        {
            int index=continuousContext->sensorCount++;continuousContext->sensorHits[index]={shape->id,fastShape->id};
            continuousContext->sensorFractions[index]=output.fraction;
        }
        return true;
    }
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
    if(didHit && (shape->enablePreSolveEvents || fastShape->enablePreSolveEvents) && world->preContinuousFcn!=NULL)
    {
        b2ShapeId idA={shape->id+1,world->worldId,shape->generation},idB={fastShape->id+1,world->worldId,fastShape->generation};
        didHit=world->preContinuousFcn(idA,idB,b2OffsetPos(continuousContext->base,output.point),output.normal,world->preSolveContext);
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
        b2AABB box1=shape->aabb,box2=b2ComputeShapeAABB(shape,b2MakeWorldTransform(xf2));
        box2=b2OffsetAABB(box2,base);
        shape->aabb=box2;
        if(shape->sensorIndex!=B2_NULL_INDEX) { continue; }
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
        fastBodySim->transform.q=q;fastBodySim->transform.p=b2OffsetPos(base,b2Sub(c,b2RotateVector(q,sweep.localCenter)));
        fastBodySim->center=b2OffsetPos(base,c);fastBodySim->flags|=b2_hadTimeOfImpact;
        world->bodyMoveEvents.data[bodySimIndex].transform=fastBodySim->transform;
        b2BodyState* state=awake->bodyStates.data+bodySimIndex;
        float timeLoss=(1.0f-context.fraction)*dt;
        b2Vec2 dv=b2MulSV(-timeLoss*fastBodySim->gravityScale,world->gravity);
        dv.x=(state->flags&b2_lockLinearX)?0.0f:dv.x;dv.y=(state->flags&b2_lockLinearY)?0.0f:dv.y;
        state->linearVelocity=b2Add(state->linearVelocity,dv);
    }
    fastBodySim->center0=fastBodySim->center;fastBodySim->rotation0=fastBodySim->transform.q;
    for(int i=0;i<context.sensorCount;++i)
    {
        if(context.sensorFractions[i]<context.fraction)
        {
            b2SensorHit hit=context.sensorHits[i];b2Shape* sensorShape=world->shapes.data+hit.sensorId;
            b2Shape* visitor=world->shapes.data+hit.visitorId;
            b2Array_Push(world->sensors.data[sensorShape->sensorIndex].hits,b2Visitor{visitor->id,visitor->generation});
        }
    }
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

static void b2ReportHitEvents(b2World* world)
{
    for(int i=0;i<world->contacts.count;++i)
    {
        b2Contact* contact=world->contacts.data+i;
        if(contact->contactId==B2_NULL_INDEX || contact->setIndex!=b2_awakeSet || contact->colorIndex==B2_NULL_INDEX) { continue; }
        b2ContactSim* sim=b2GetContactSim(world,contact);if((sim->simFlags&b2_simEnableHitEvent)==0) { continue; }
        b2ContactHitEvent event={};event.approachSpeed=world->hitEventThreshold;
        b2ManifoldPoint* best=NULL;
        for(int p=0;p<sim->manifold.pointCount;++p)
        {
            b2ManifoldPoint* point=sim->manifold.points+p;float speed=-point->normalVelocity;
            if(speed>event.approachSpeed && point->totalNormalImpulse>0) { event.approachSpeed=speed;best=point; }
        }
        if(best==NULL) { continue; }
        b2Shape* a=world->shapes.data+contact->shapeIdA;b2Shape* b=world->shapes.data+contact->shapeIdB;
        b2Body* bodyA=world->bodies.data+a->bodyId;b2Body* bodyB=world->bodies.data+b->bodyId;
        event.normal=sim->manifold.normal;
        if(bodyA->type!=b2_staticBody && bodyB->type==b2_staticBody) { event.point=b2OffsetPos(b2GetBodySim(world,bodyB)->center,best->anchorB); }
        else { event.point=b2OffsetPos(b2GetBodySim(world,bodyA)->center,best->anchorA); }
        event.shapeIdA={a->id+1,world->worldId,a->generation};event.shapeIdB={b->id+1,world->worldId,b->generation};
        event.contactId={contact->contactId+1,world->worldId,0,contact->generation};b2Array_Push(world->contactHitEvents,event);
    }
}

static void b2ReportJointEvents(b2World* world)
{
 uint32_t wordCount=world->jointStateBitSet.blockCount;
 uint64_t* bits=world->jointStateBitSet.bits;
 for(uint32_t k=0;k<wordCount;++k)
 {
  uint64_t word=bits[k];
  while(word!=0)
  {
   uint32_t ctz=std::countr_zero(word);
   int jointId=(int)(64*k+ctz);
   b2Joint* joint=world->joints.data+jointId;
   B2_ASSERT(joint->setIndex==b2_awakeSet);
   b2JointEvent event={};
   event.jointId={jointId+1,world->worldId,joint->generation};
   event.userData=joint->userData;
   b2Array_Push(world->jointEvents,event);
   word=word&(word-1);
  }
 }
}

static void b2SolverRangeTask(int startIndex,int endIndex,int workerIndex,void* taskContext)
{
 b2StepContext* context=static_cast<b2StepContext*>(taskContext);
 b2World* world=context->world;
 b2SolverSet* set=world->solverSets.data+b2_awakeSet;
 if(context->solverTaskType==0)
 {
  for(int i=startIndex;i<endIndex;++i) { b2IntegrateBodyVelocity(world,set->bodySims.data+i,set->bodyStates.data+i,context->h); }
  return;
 }
 if(context->solverTaskType==3)
 {
  float h=context->h,timeStep=context->timeStep;
  for(int i=startIndex;i<endIndex;++i)
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
  return;
 }
 int colorIndex=context->activeColorIndex;
 b2GraphColor* color=context->graph->colors+colorIndex;
 int jointCount=color->jointSims.count;
 bool warm=context->solverTaskType==1,useBias=context->solverTaskType==2;
 b2BitSet* bits=&world->taskContexts.data[workerIndex].jointStateBitSet;
 for(int i=startIndex;i<b2MinInt(endIndex,jointCount);++i)
 {
  b2JointSim* joint=color->jointSims.data+i;
  if(warm) { b2WarmStartJoint(joint,context);continue; }
  b2SolveJoint(joint,context,useBias);
  if(colorIndex!=B2_OVERFLOW_INDEX && useBias && (joint->forceThreshold<FLT_MAX || joint->torqueThreshold<FLT_MAX) && !b2GetBit(bits,joint->jointId))
  {
   float force,torque;b2GetJointReaction(joint,context->inv_h,&force,&torque);
   if(force>=joint->forceThreshold || torque>=joint->torqueThreshold) { b2SetBit(bits,joint->jointId); }
  }
 }
 int contactStart=b2MaxInt(0,startIndex-jointCount),contactEnd=b2MaxInt(0,endIndex-jointCount);
 if(context->simdWidth>0 && colorIndex!=B2_OVERFLOW_INDEX)
 {
  b2SolverBlock block={contactStart,(uint16_t)(contactEnd-contactStart),0,(uint8_t)colorIndex};
  if(warm) { context->warmStartContacts(block,context); }
  else if(useBias) { context->pushContacts(block,context); }
  else { context->solveContacts(block,context); }
 }
 else if(warm) { b2WarmStartContactRange(context,colorIndex,contactStart,contactEnd); }
 else { b2SolveNormalRange(context,colorIndex,useBias,contactStart,contactEnd); }
}

void b2BeginSolve(b2World* world,b2StepContext* context)
{
 uint64_t ticks=b2GetTicks();
 b2SolverSet* set=world->solverSets.data+b2_awakeSet;
 B2_ASSERT(set->bodySims.count==set->bodyStates.count);
 context->graph=&world->constraintGraph;context->states=set->bodyStates.data;
 context->h=context->timeStep/(float)context->subStepCount;context->inv_h=1.0f/context->h;
 b2SelectContactFunctions(context,b2GetSIMDWidth());
 context->solverStage=0;context->solverColor=0;context->solverStep=0;
 context->solverTimingActive=false;
 b2Array_Resize(world->bodyMoveEvents,set->bodySims.count);
 b2SetBitCountAndClear(&world->jointStateBitSet,b2GetIdCapacity(&world->jointIdPool));
 for(int i=0;i<world->workerCount;++i) { b2SetBitCountAndClear(&world->taskContexts.data[i].jointStateBitSet,b2GetIdCapacity(&world->jointIdPool)); }
 world->profile.solverSetup=b2GetMillisecondsAndReset(&ticks);
 for(int c=0;c<B2_GRAPH_COLOR_COUNT;++c)
 {
  b2GraphColor* color=context->graph->colors+c;
  for(int i=0;i<color->jointSims.count;++i) { b2PrepareJoint(color->jointSims.data+i,context); }
 }
 b2PrepareContactConstraints(context);world->profile.prepareConstraints=b2GetMillisecondsAndReset(&ticks);
 context->constraintTicks=b2GetTicks();
}

bool b2ContinueSolve(b2StepContext* context)
{
 b2World* world=context->world;
 if(context->solverTimingActive)
 {
  float elapsed=b2GetMilliseconds(context->solverTaskTicks);
  switch(context->solverTaskType)
  {
   case 0:world->profile.integrateVelocities+=elapsed;break;
   case 1:world->profile.warmStart+=elapsed;break;
   case 2:world->profile.solveImpulses+=elapsed;break;
   case 3:world->profile.integratePositions+=elapsed;break;
   case 4:world->profile.relaxImpulses+=elapsed;break;
  }
  context->solverTimingActive=false;
 }
 for(;;)
 {
  b2ResetTaskBatch(context);
  int stage=context->solverStage;context->solverTaskType=stage;context->solverTaskTicks=b2GetTicks();
  if(stage==0 || stage==3)
  {
   context->solverStage+=1;context->solverColor=0;
   b2StageParallelFor(world,b2_taskSolver,b2SolverRangeTask,world->solverSets.data[b2_awakeSet].bodySims.count,32,context);
  }
  else if(stage==1 || stage==2 || stage==4)
  {
   if(context->solverColor==B2_GRAPH_COLOR_COUNT) { context->solverStage+=1;context->solverColor=0;continue; }
   int order=context->solverColor++;
   int colorIndex=order==0?B2_OVERFLOW_INDEX:order-1;context->activeColorIndex=colorIndex;
   b2GraphColor* color=context->graph->colors+colorIndex;
   int contactCount=context->simdWidth>0 && colorIndex!=B2_OVERFLOW_INDEX?color->wideConstraintCount:color->contactSims.count;
   int count=color->jointSims.count+contactCount;
   if(colorIndex==B2_OVERFLOW_INDEX)
   {
    if(count>0) { b2SolverRangeTask(0,count,0,context); }
   }
   else { b2StageParallelFor(world,b2_taskSolver,b2SolverRangeTask,count,32,context); }
  }
  else
  {
   context->solverStep+=1;
   if(context->solverStep==context->subStepCount) { return false; }
   context->solverStage=0;continue;
  }
  if(context->batchTaskCount>0) { context->solverTimingActive=true;return true; }
 }
}

void b2EndSolve(b2StepContext* context)
{
 b2World* world=context->world;float timeStep=context->timeStep;
 b2SolverSet* set=world->solverSets.data+b2_awakeSet;
 uint64_t ticks=b2GetTicks();world->profile.constraints=b2GetMilliseconds(context->constraintTicks);
 if(b2AtomicLoadInt(&context->anyRestitution) && world->restitutionIterations>0)
 { b2ApplyContactRestitution(context);world->profile.restitution=b2GetMillisecondsAndReset(&ticks); }
 b2StoreContactImpulses(context);b2ReleaseContactConstraints(context);
 world->profile.storeImpulses=b2GetMillisecondsAndReset(&ticks);
    int bulletCount=0;int* bulletIndices=NULL;
    if(world->enableContinuous && set->bodySims.count>0)
    { bulletIndices=static_cast<int*>(b2StackAlloc(&world->stack,set->bodySims.count*sizeof(int),"bullets")); }
    for(int i=0;i<set->bodySims.count;++i)
    {
        b2BodySim* sim=set->bodySims.data+i;
        b2BodyState* state=set->bodyStates.data+i;
        sim->center=b2OffsetPos(sim->center,state->deltaPosition);
        sim->transform.q=b2NormalizeRot(b2MulRot(state->deltaRotation,sim->transform.q));
        sim->transform.p=b2OffsetPos(sim->center,b2Neg(b2RotateVector(sim->transform.q,sim->localCenter)));
        float maxVelocity=b2Length(state->linearVelocity)+b2AbsFloat(state->angularVelocity)*sim->maxExtent;
        float maxDeltaPosition=b2Length(state->deltaPosition)+b2AbsFloat(state->deltaRotation.s)*sim->maxExtent;
        float sleepVelocity=b2MaxFloat(maxVelocity,0.5f/timeStep*maxDeltaPosition);
        b2Body* body=world->bodies.data+sim->bodyId;
        body->bodyMoveIndex=i;
        world->bodyMoveEvents.data[i]=b2BodyMoveEvent{body->userData,sim->transform,b2BodyId{body->id+1,world->worldId,body->generation},false};
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
    world->profile.transforms=b2GetMillisecondsAndReset(&ticks);
    for(int i=0;i<world->workerCount;++i) { b2InPlaceUnion(&world->jointStateBitSet,&world->taskContexts.data[i].jointStateBitSet); }
    b2ReportJointEvents(world);world->profile.jointEvents=b2GetMillisecondsAndReset(&ticks);
    b2ReportHitEvents(world);world->profile.hitEvents=b2GetMillisecondsAndReset(&ticks);
    for(int i=0;i<bulletCount;++i)
    {
        int index=bulletIndices[i];b2SolveContinuous(world,index,timeStep);
        b2BodySim* sim=set->bodySims.data+index;
        for(int shapeId=world->bodies.data[sim->bodyId].headShapeId;shapeId!=B2_NULL_INDEX;shapeId=world->shapes.data[shapeId].nextShapeId)
        { b2MoveShapeProxy(world,world->shapes.data+shapeId,sim->transform); }
    }
    if(bulletIndices!=NULL) { b2StackFree(&world->stack,bulletIndices); }
    if(bulletCount>0) { world->profile.bullets=b2GetMilliseconds(ticks); }
    ticks=b2GetTicks();
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

    if(world->enableSleep) { world->profile.sleepIslands=b2GetMilliseconds(ticks); }

}

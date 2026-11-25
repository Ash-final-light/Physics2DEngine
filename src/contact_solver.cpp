// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
#include "contact_solver.h"
#include "world.h"

void b2PrepareContactConstraints(b2StepContext* context)
{
    b2World* world=context->world;
    float warmStartScale=world->enableWarmStarting?1.0f:0.0f;
    float contactHertz=b2MinFloat(world->contactHertz,0.125f*context->inv_h);
    b2Softness contactSoftness=b2MakeSoft(contactHertz,world->contactDampingRatio,context->h);
    b2Softness staticSoftness=b2MakeSoft(2.0f*contactHertz,world->contactDampingRatio,context->h);
    for(int colorIndex=0;colorIndex<B2_GRAPH_COLOR_COUNT;++colorIndex)
    {
        b2GraphColor* color=context->graph->colors+colorIndex;
        B2_ASSERT(color->overflowConstraints==NULL);
        int count=color->contactSims.count;
        if(count==0) { continue; }
        color->overflowConstraints=static_cast<b2ContactConstraint*>(b2StackAlloc(&world->stack,count*sizeof(b2ContactConstraint),"contact constraints"));
        for(int i=0;i<count;++i)
        {
            b2ContactSim* sim=color->contactSims.data+i;
            b2Contact* contact=world->contacts.data+sim->contactId;
            b2Body* bodyA=world->bodies.data+contact->edges[0].bodyId;
            b2Body* bodyB=world->bodies.data+contact->edges[1].bodyId;
            B2_ASSERT(sim->encodedBodySimA==b2EncodeBodySimIndex(bodyA));
            B2_ASSERT(sim->encodedBodySimB==b2EncodeBodySimIndex(bodyB));
            b2BodySim* bodySimA=b2GetBodySim(world,bodyA);
            b2BodySim* bodySimB=b2GetBodySim(world,bodyB);
            // A mass edit can occur while the persistent contact remains in its color.
            sim->invMassA=bodySimA->invMass;sim->invIA=bodySimA->invInertia;
            sim->invMassB=bodySimB->invMass;sim->invIB=bodySimB->invInertia;
            b2Manifold* manifold=&sim->manifold;
            B2_ASSERT(0<manifold->pointCount && manifold->pointCount<=2);
            b2ContactConstraint* constraint=color->overflowConstraints+i;
            *constraint=b2ContactConstraint{};
            constraint->indexA=b2DecodeAwakeIndex(sim->encodedBodySimA)+1;
            constraint->indexB=b2DecodeAwakeIndex(sim->encodedBodySimB)+1;
            constraint->normal=manifold->normal;
            constraint->friction=sim->friction;
            constraint->restitution=sim->restitution;
            constraint->tangentSpeed=sim->tangentSpeed;
            constraint->rollingResistance=sim->rollingResistance;
            constraint->rollingImpulse=warmStartScale*manifold->rollingImpulse;
            constraint->softness=(constraint->indexA==0 || constraint->indexB==0)?staticSoftness:contactSoftness;
            constraint->pointCount=manifold->pointCount;
            float mA=sim->invMassA,iA=sim->invIA,mB=sim->invMassB,iB=sim->invIB;
            constraint->invMassA=mA;constraint->invIA=iA;
            constraint->invMassB=mB;constraint->invIB=iB;
            constraint->rollingMass=iA+iB>0.0f?1.0f/(iA+iB):0.0f;
            for(int j=0;j<manifold->pointCount;++j)
            {
                b2ManifoldPoint* mp=manifold->points+j;
                b2ContactConstraintPoint* cp=constraint->points+j;
                b2Vec2 rA=mp->anchorA,rB=mp->anchorB;
                cp->anchorA=rA;cp->anchorB=rB;
                cp->normalImpulse=warmStartScale*mp->normalImpulse;
                cp->tangentImpulse=warmStartScale*mp->tangentImpulse;
                cp->baseSeparation=mp->separation-b2Dot(b2Sub(rB,rA),constraint->normal);
                float rnA=b2Cross(rA,constraint->normal),rnB=b2Cross(rB,constraint->normal);
                float kNormal=mA+mB+iA*rnA*rnA+iB*rnB*rnB;
                cp->normalMass=kNormal>0.0f?1.0f/kNormal:0.0f;
                b2Vec2 tangent=b2RightPerp(constraint->normal);
                float rtA=b2Cross(rA,tangent),rtB=b2Cross(rB,tangent);
                float kTangent=mA+mB+iA*rtA*rtA+iB*rtB*rtB;
                cp->tangentMass=kTangent>0.0f?1.0f/kTangent:0.0f;
                if(sim->restitution>0.0f)
                {
                    b2BodyState dummy=b2_identityBodyState;
                    const b2BodyState* stateA=constraint->indexA==0?&dummy:context->states+constraint->indexA-1;
                    const b2BodyState* stateB=constraint->indexB==0?&dummy:context->states+constraint->indexB-1;
                    b2Vec2 vrA=b2Add(stateA->linearVelocity,b2CrossSV(stateA->angularVelocity,rA));
                    b2Vec2 vrB=b2Add(stateB->linearVelocity,b2CrossSV(stateB->angularVelocity,rB));
                    cp->relativeVelocity=b2Dot(constraint->normal,b2Sub(vrB,vrA));
                }
                mp->normalVelocity=cp->relativeVelocity;
            }
        }
    }
}

void b2ReleaseContactConstraints(b2StepContext* context)
{
    for(int i=B2_GRAPH_COLOR_COUNT-1;i>=0;--i)
    {
        b2GraphColor* color=context->graph->colors+i;
        if(color->overflowConstraints==NULL) { continue; }
        b2StackFree(&context->world->stack,color->overflowConstraints);
        color->overflowConstraints=NULL;
    }
}

static void b2SolveNormalColor(b2StepContext* context,int colorIndex,bool useBias)
{
    b2GraphColor* color=context->graph->colors+colorIndex;
    b2BodyState* states=context->states;
    b2BodyState dummyState=b2_identityBodyState;
    for(int i=0;i<color->contactSims.count;++i)
    {
        b2ContactConstraint* constraint=color->overflowConstraints+i;
        float mA=constraint->invMassA,iA=constraint->invIA,mB=constraint->invMassB,iB=constraint->invIB;
        int indexA=constraint->indexA-1,indexB=constraint->indexB-1;
        b2BodyState* stateA=indexA==B2_NULL_INDEX?&dummyState:states+indexA;
        b2BodyState* stateB=indexB==B2_NULL_INDEX?&dummyState:states+indexB;
        b2Vec2 vA=stateA->linearVelocity,vB=stateB->linearVelocity;
        float wA=stateA->angularVelocity,wB=stateB->angularVelocity;
        b2Vec2 dp=b2Sub(stateB->deltaPosition,stateA->deltaPosition);
        b2Vec2 normal=constraint->normal;
        float vnc=b2Dot(b2Sub(vB,vA),normal);
        b2Vec2 tangent=b2RightPerp(normal);
        float vtc=b2Dot(b2Sub(vB,vA),tangent)-constraint->tangentSpeed;
        float tangentImpulses[2]={0,0},rtA[2],rtB[2];
        float impulses[2]={0,0},rnA[2],rnB[2];
        float normalImpulse=0,angularA=0,angularB=0;
        for(int j=0;j<constraint->pointCount;++j)
        {
            b2ContactConstraintPoint* cp=constraint->points+j;
            b2Vec2 rA=cp->anchorA,rB=cp->anchorB;
            rnA[j]=b2Cross(rA,normal);rnB[j]=b2Cross(rB,normal);
            rtA[j]=b2Cross(rA,tangent);rtB[j]=b2Cross(rB,tangent);
            b2Vec2 ds=b2Add(dp,b2Sub(b2RotateVector(stateB->deltaRotation,rB),b2RotateVector(stateA->deltaRotation,rA)));
            float s=cp->baseSeparation+b2Dot(ds,normal);
            float velocityBias=s>0.0f?s*context->inv_h:0.0f;
            float massScale=1.0f,impulseScale=0.0f;
            if(useBias && s<=0.0f)
            {
                const b2Softness softness=constraint->softness;
                velocityBias=b2MaxFloat(softness.massScale*softness.biasRate*s,-context->world->contactSpeed);
                massScale=softness.massScale;impulseScale=softness.impulseScale;
            }
            float vn=vnc+wB*rnB[j]-wA*rnA[j];
            for(int k=0;k<j;++k)
            {
                vn+=(mA+mB+iA*rnA[k]*rnA[j]+iB*rnB[k]*rnB[j])*impulses[k];
            }
            float impulse=b2MaxFloat(-cp->normalMass*(massScale*vn+velocityBias)-impulseScale*cp->normalImpulse,-cp->normalImpulse);
            cp->normalImpulse+=impulse;cp->totalNormalImpulse+=impulse;
            impulses[j]=impulse;
            normalImpulse+=impulse;angularA+=impulse*rnA[j];angularB+=impulse*rnB[j];
        }
        float tangentImpulse=0.0f;
        float rollingImpulse=0.0f;
        if(!useBias)
        {
            float Cdot=wB-wA,totalNormalImpulse=0.0f;
            for(int j=0;j<constraint->pointCount;++j)
            {
                Cdot+=(iA*rnA[j]+iB*rnB[j])*impulses[j];
                totalNormalImpulse+=constraint->points[j].normalImpulse;
            }
            float lambda=constraint->rollingImpulse;
            float maxLambda=constraint->rollingResistance*totalNormalImpulse;
            constraint->rollingImpulse=b2ClampFloat(lambda-constraint->rollingMass*Cdot,-maxLambda,maxLambda);
            rollingImpulse=constraint->rollingImpulse-lambda;
            angularA+=rollingImpulse;angularB+=rollingImpulse;
        }
        if(!useBias)
        {
            for(int j=0;j<constraint->pointCount;++j)
            {
                b2ContactConstraintPoint* cp=constraint->points+j;
                float vt=vtc+wB*rtB[j]-wA*rtA[j]+(iA*rtA[j]+iB*rtB[j])*rollingImpulse;
                for(int k=0;k<constraint->pointCount;++k)
                { vt+=(iA*rtA[j]*rnA[k]+iB*rtB[j]*rnB[k])*impulses[k]; }
                for(int k=0;k<j;++k)
                { vt+=(mA+mB+iA*rtA[k]*rtA[j]+iB*rtB[k]*rtB[j])*tangentImpulses[k]; }
                float maxFriction=constraint->friction*cp->normalImpulse;
                float impulse=b2ClampFloat(-cp->tangentMass*vt,-maxFriction-cp->tangentImpulse,maxFriction-cp->tangentImpulse);
                cp->tangentImpulse+=impulse;tangentImpulses[j]=impulse;
                tangentImpulse+=impulse;angularA+=impulse*rtA[j];angularB+=impulse*rtB[j];
            }
        }
        b2Vec2 P=b2Add(b2MulSV(normalImpulse,normal),b2MulSV(tangentImpulse,tangent));
        vA=b2MulSub(vA,mA,P);wA-=iA*angularA;
        vB=b2MulAdd(vB,mB,P);wB+=iB*angularB;
        if(stateA->flags&b2_dynamicFlag) { stateA->linearVelocity=vA;stateA->angularVelocity=wA; }
        if(stateB->flags&b2_dynamicFlag) { stateB->linearVelocity=vB;stateB->angularVelocity=wB; }
    }
}

void b2SolveNormalContacts(b2StepContext* context)
{
    // Overflow constraints run first, as in the reference stage driver.
    b2SolveNormalColor(context,B2_OVERFLOW_INDEX,false);
    for(int i=0;i<B2_OVERFLOW_INDEX;++i) { b2SolveNormalColor(context,i,false); }
}

void b2StoreContactImpulses(b2StepContext* context)
{
    for(int c=0;c<B2_GRAPH_COLOR_COUNT;++c)
    {
        b2GraphColor* color=context->graph->colors+c;
        for(int i=0;i<color->contactSims.count;++i)
        {
            b2Manifold* manifold=&color->contactSims.data[i].manifold;
            const b2ContactConstraint* constraint=color->overflowConstraints+i;
            B2_ASSERT(manifold->pointCount==constraint->pointCount);
            manifold->rollingImpulse=constraint->rollingImpulse;
            for(int j=0;j<manifold->pointCount;++j)
            {
                manifold->points[j].normalImpulse=constraint->points[j].normalImpulse;
                manifold->points[j].tangentImpulse=constraint->points[j].tangentImpulse;
                manifold->points[j].totalNormalImpulse=constraint->points[j].totalNormalImpulse;
            }
        }
    }
}

void b2PushNormalContacts(b2StepContext* context)
{
    b2SolveNormalColor(context,B2_OVERFLOW_INDEX,true);
    for(int i=0;i<B2_OVERFLOW_INDEX;++i) { b2SolveNormalColor(context,i,true); }
}

void b2WarmStartContacts(b2StepContext* context)
{
    b2BodyState dummyState=b2_identityBodyState;
    for(int order=0;order<B2_GRAPH_COLOR_COUNT;++order)
    {
        int c=order==0?B2_OVERFLOW_INDEX:order-1;
        b2GraphColor* color=context->graph->colors+c;
        for(int i=0;i<color->contactSims.count;++i)
        {
            b2ContactConstraint* constraint=color->overflowConstraints+i;
            int indexA=constraint->indexA-1,indexB=constraint->indexB-1;
            b2BodyState* stateA=indexA==B2_NULL_INDEX?&dummyState:context->states+indexA;
            b2BodyState* stateB=indexB==B2_NULL_INDEX?&dummyState:context->states+indexB;
            b2Vec2 vA=stateA->linearVelocity,vB=stateB->linearVelocity;
            float wA=stateA->angularVelocity,wB=stateB->angularVelocity;
            for(int j=0;j<constraint->pointCount;++j)
            {
                b2ContactConstraintPoint* cp=constraint->points+j;
                b2Vec2 P=b2Add(b2MulSV(cp->normalImpulse,constraint->normal),b2MulSV(cp->tangentImpulse,b2RightPerp(constraint->normal)));
                cp->totalNormalImpulse+=cp->normalImpulse;
                wA-=constraint->invIA*b2Cross(cp->anchorA,P);vA=b2MulAdd(vA,-constraint->invMassA,P);
                wB+=constraint->invIB*b2Cross(cp->anchorB,P);vB=b2MulAdd(vB,constraint->invMassB,P);
            }
            wA-=constraint->invIA*constraint->rollingImpulse;wB+=constraint->invIB*constraint->rollingImpulse;
            if(stateA->flags&b2_dynamicFlag) { stateA->linearVelocity=vA;stateA->angularVelocity=wA; }
            if(stateB->flags&b2_dynamicFlag) { stateB->linearVelocity=vB;stateB->angularVelocity=wB; }
        }
    }
}

static void b2ApplyRestitutionColor( b2StepContext* context,int colorIndex )
{

	b2ConstraintGraph* graph = context->graph;
	b2GraphColor* color = graph->colors + colorIndex;
	b2ContactConstraint* constraints = color->overflowConstraints;
	int contactCount = color->contactSims.count;
	b2World* world = context->world;
	b2SolverSet* awakeSet = b2Array_Get( world->solverSets, b2_awakeSet );
	b2BodyState* states = awakeSet->bodyStates.data;

	float threshold = world->restitutionThreshold;
	float inv_h = context->inv_h;
	bool propagate = false;

	b2BodyState dummyState = b2_identityBodyState;

	for ( int i = 0; i < contactCount; ++i )
	{
		b2ContactConstraint* constraint = constraints + i;
		if ( propagate == false && constraint->restitution == 0.0f )
		{
			continue;
		}

		float mA = constraint->invMassA;
		float iA = constraint->invIA;
		float mB = constraint->invMassB;
		float iB = constraint->invIB;

		int indexA = constraint->indexA - 1;
		int indexB = constraint->indexB - 1;

		b2BodyState* stateA = indexA == B2_NULL_INDEX ? &dummyState : states + indexA;
		b2Vec2 vA = stateA->linearVelocity;
		float wA = stateA->angularVelocity;
		b2Rot dqA = stateA->deltaRotation;

		b2BodyState* stateB = indexB == B2_NULL_INDEX ? &dummyState : states + indexB;
		b2Vec2 vB = stateB->linearVelocity;
		float wB = stateB->angularVelocity;
		b2Rot dqB = stateB->deltaRotation;

		b2Vec2 dp = b2Sub( stateB->deltaPosition, stateA->deltaPosition );

		b2Vec2 normal = constraint->normal;
		float restitution = constraint->restitution;
		int pointCount = constraint->pointCount;

		for ( int j = 0; j < pointCount; ++j )
		{
			b2ContactConstraintPoint* cp = constraint->points + j;

			b2Vec2 rA = cp->anchorA;
			b2Vec2 rB = cp->anchorB;

			// 预测性接触点的总法向冲量为 0。
			float compressionImpulse = cp->totalNormalImpulse - cp->restitutionImpulse;
			bool armed = restitution > 0.0f && cp->relativeVelocity < -threshold && compressionImpulse > 0.0f;

			float velocityBias;
			if ( armed )
			{
				velocityBias = restitution * cp->relativeVelocity;
			}
			else
			{
				b2Vec2 ds = b2Add( dp, b2Sub( b2RotateVector( dqB, rB ), b2RotateVector( dqA, rA ) ) );
				float s = cp->baseSeparation + b2Dot( ds, normal );

				velocityBias = s > 0.0f ? s * inv_h : 0.0f;
			}

			b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
			b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
			float vn = b2Dot( b2Sub( vrB, vrA ), normal );

			float impulse = -cp->normalMass * ( vn + velocityBias );

			float newImpulse = b2MaxFloat( cp->normalImpulse + impulse, 0.0f );
			impulse = newImpulse - cp->normalImpulse;

			float approachImpulse = b2MinFloat( b2MaxFloat( -cp->normalMass * vn, 0.0f ), b2MaxFloat( impulse, 0.0f ) );

			if ( armed )
			{
				// Poisson 动能恢复保证不增能。
				float allowance = restitution * ( compressionImpulse + approachImpulse ) - cp->restitutionImpulse;
				impulse = b2MinFloat( impulse, approachImpulse + b2MaxFloat( allowance, 0.0f ) );
			}

			cp->normalImpulse += impulse;
			cp->restitutionImpulse += impulse - approachImpulse;
			cp->totalNormalImpulse += impulse;

			b2Vec2 P = b2MulSV( impulse, normal );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );

			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}

		if ( stateA->flags & b2_dynamicFlag )
		{
			stateA->linearVelocity = vA;
			stateA->angularVelocity = wA;
		}

		if ( stateB->flags & b2_dynamicFlag )
		{
			stateB->linearVelocity = vB;
			stateB->angularVelocity = wB;
		}
	}

}

void b2ApplyContactRestitution(b2StepContext* context)
{
    b2ApplyRestitutionColor(context,B2_OVERFLOW_INDEX);
    for(int i=0;i<B2_OVERFLOW_INDEX;++i) { b2ApplyRestitutionColor(context,i); }
}

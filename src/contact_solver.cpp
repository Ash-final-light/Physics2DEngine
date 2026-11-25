// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
#include "contact_solver.h"
#include "world.h"

void b2PrepareContactConstraints(b2StepContext* context)
{
    b2World* world=context->world;
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
            constraint->pointCount=manifold->pointCount;
            float mA=sim->invMassA,iA=sim->invIA,mB=sim->invMassB,iB=sim->invIB;
            constraint->invMassA=mA;constraint->invIA=iA;
            constraint->invMassB=mB;constraint->invIB=iB;
            for(int j=0;j<manifold->pointCount;++j)
            {
                const b2ManifoldPoint* mp=manifold->points+j;
                b2ContactConstraintPoint* cp=constraint->points+j;
                b2Vec2 rA=mp->anchorA,rB=mp->anchorB;
                cp->anchorA=rA;cp->anchorB=rB;
                cp->baseSeparation=mp->separation-b2Dot(b2Sub(rB,rA),constraint->normal);
                float rnA=b2Cross(rA,constraint->normal),rnB=b2Cross(rB,constraint->normal);
                float kNormal=mA+mB+iA*rnA*rnA+iB*rnB*rnB;
                cp->normalMass=kNormal>0.0f?1.0f/kNormal:0.0f;
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

static void b2SolveNormalColor(b2StepContext* context,int colorIndex)
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
        float impulses[2]={0,0},rnA[2],rnB[2];
        float normalImpulse=0,angularA=0,angularB=0;
        for(int j=0;j<constraint->pointCount;++j)
        {
            b2ContactConstraintPoint* cp=constraint->points+j;
            b2Vec2 rA=cp->anchorA,rB=cp->anchorB;
            rnA[j]=b2Cross(rA,normal);rnB[j]=b2Cross(rB,normal);
            b2Vec2 ds=b2Add(dp,b2Sub(b2RotateVector(stateB->deltaRotation,rB),b2RotateVector(stateA->deltaRotation,rA)));
            float s=cp->baseSeparation+b2Dot(ds,normal);
            float velocityBias=s>0.0f?s*context->inv_h:0.0f;
            float vn=vnc+wB*rnB[j]-wA*rnA[j];
            for(int k=0;k<j;++k)
            {
                vn+=(mA+mB+iA*rnA[k]*rnA[j]+iB*rnB[k]*rnB[j])*impulses[k];
            }
            float impulse=b2MaxFloat(-cp->normalMass*(vn+velocityBias),-cp->normalImpulse);
            cp->normalImpulse+=impulse;cp->totalNormalImpulse+=impulse;
            impulses[j]=impulse;
            normalImpulse+=impulse;angularA+=impulse*rnA[j];angularB+=impulse*rnB[j];
        }
        b2Vec2 P=b2MulSV(normalImpulse,normal);
        vA=b2MulSub(vA,mA,P);wA-=iA*angularA;
        vB=b2MulAdd(vB,mB,P);wB+=iB*angularB;
        if(stateA->flags&b2_dynamicFlag) { stateA->linearVelocity=vA;stateA->angularVelocity=wA; }
        if(stateB->flags&b2_dynamicFlag) { stateB->linearVelocity=vB;stateB->angularVelocity=wB; }
    }
}

void b2SolveNormalContacts(b2StepContext* context)
{
    // Overflow constraints run first, as in the reference stage driver.
    b2SolveNormalColor(context,B2_OVERFLOW_INDEX);
    for(int i=0;i<B2_OVERFLOW_INDEX;++i) { b2SolveNormalColor(context,i); }
}

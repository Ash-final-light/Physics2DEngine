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

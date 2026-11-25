// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver.h"
#include "contact_solver.h"
#include "world.h"
#include "shape.h"
#include "box2d/math_functions.h"

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
            state->deltaPosition=b2MulAdd(state->deltaPosition,h,state->linearVelocity);
            state->deltaRotation=b2IntegrateRotation(state->deltaRotation,h*state->angularVelocity);
        }
        b2SolveNormalContacts(&context);
    }
    b2ApplyContactRestitution(&context);
    b2StoreContactImpulses(&context);
    b2ReleaseContactConstraints(&context);
    for(int i=0;i<set->bodySims.count;++i)
    {
        b2BodySim* sim=set->bodySims.data+i;
        b2BodyState* state=set->bodyStates.data+i;
        sim->center=b2Add(sim->center,state->deltaPosition);
        sim->transform.q=b2NormalizeRot(b2MulRot(state->deltaRotation,sim->transform.q));
        sim->transform.p=b2Sub(sim->center,b2RotateVector(sim->transform.q,sim->localCenter));
        state->deltaPosition=b2Vec2_zero;state->deltaRotation=b2Rot_identity;
        sim->force=b2Vec2_zero;sim->torque=0.0f;
        for(int shapeId=world->bodies.data[sim->bodyId].headShapeId;shapeId!=B2_NULL_INDEX;shapeId=world->shapes.data[shapeId].nextShapeId)
        { b2MoveShapeProxy(world,world->shapes.data+shapeId,sim->transform); }
    }
}

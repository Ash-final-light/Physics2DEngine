// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "test_macros.h"
#include "contact_solver.h"
#include "world.h"

int ContactPrepareTest(void)
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();
    b2Polygon box=b2MakeBox(1,0.5f);b2CreatePolygonShape(body,&sd,&box);
    bd.type=b2_staticBody;bd.position={0,-0.9f};b2BodyId ground=b2CreateBody(id,&bd);
    b2Polygon floor=b2MakeBox(5,0.5f);b2CreatePolygonShape(ground,&sd,&floor);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    b2Contact* contact=world->contacts.data;b2ContactSim* sim=b2GetContactSim(world,contact);
    ENSURE(sim->manifold.pointCount==2);
    // Edit mass after graph insertion, exercising current mass rather than stale cache.
    b2MassData mass={2,{0,0},4};b2Body_SetMassData(body,mass);
    b2Body_SetLinearVelocity(body,{1,-2});
    sim->manifold.points[0].normalImpulse=7;
    b2StepContext context={world,&world->constraintGraph,world->solverSets.data[b2_awakeSet].bodyStates.data,0.1f,10.0f};
    b2PrepareContactConstraints(&context);
    b2ContactConstraint* constraint=world->constraintGraph.colors[contact->colorIndex].overflowConstraints+contact->localIndex;
    ENSURE(constraint->indexA==world->bodies.data[body.index1-1].localIndex+1);
    ENSURE(constraint->indexB==0 && constraint->invMassB==0 && constraint->invIB==0);
    ENSURE(constraint->pointCount==2);
    for(int j=0;j<2;++j)
    {
        b2ContactConstraintPoint* cp=constraint->points+j;
        ENSURE_SMALL(cp->normalMass-4.0f/3.0f,0.00001f);
        float separation=cp->baseSeparation+b2Dot(b2Sub(cp->anchorB,cp->anchorA),constraint->normal);
        ENSURE_SMALL(separation-sim->manifold.points[j].separation,0.00001f);
        ENSURE(cp->normalImpulse==0 && cp->totalNormalImpulse==0);
    }
    ENSURE(sim->manifold.points[0].normalImpulse==7);
    ENSURE(b2Body_GetLinearVelocity(body).x==1 && b2Body_GetLinearVelocity(body).y==-2);
    b2ReleaseContactConstraints(&context);
    for(int i=0;i<B2_GRAPH_COLOR_COUNT;++i) { ENSURE(world->constraintGraph.colors[i].overflowConstraints==NULL); }
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int ContactNormalSolveTest(void)
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId id=b2CreateWorld(&wd);
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={3,2};
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};
    b2BodyId a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.type=b2_staticBody;bd.position={2,0};b2BodyId b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    b2World_Step(id,0.01f,1);
    ENSURE_SMALL(b2Body_GetLinearVelocity(a).x,0.00001f);
    ENSURE(b2Body_GetLinearVelocity(a).y==2);
    ENSURE_SMALL(b2Body_GetPosition(a).x,0.00001f);ENSURE(b2Body_GetPosition(b).x==2);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // Unequal masses retain linear momentum in an inelastic collision.
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={3,0};
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.position={2,0};bd.linearVelocity={0,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    b2Body_SetMassData(a,b2MassData{2,{0,0},1});b2Body_SetMassData(b,b2MassData{1,{0,0},1});
    b2World_Step(id,0.01f,1);
    ENSURE_SMALL(b2Body_GetLinearVelocity(a).x-2,0.00001f);
    ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-2,0.00001f);
    ENSURE_SMALL(2*b2Body_GetLinearVelocity(a).x+b2Body_GetLinearVelocity(b).x-6,0.00001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={3,0};
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.position={2,0};bd.linearVelocity={-3,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    b2World_Step(id,0.01f,1);
    ENSURE_SMALL(b2Body_GetLinearVelocity(a).x,0.00001f);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x,0.00001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // A moving kinematic obstacle transfers normal velocity without being slowed.
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.type=b2_kinematicBody;bd.position={2,0};bd.linearVelocity={-1,0};
    b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    b2World_Step(id,0.01f,1);
    ENSURE_SMALL(b2Body_GetLinearVelocity(a).x+1,0.00001f);ENSURE(b2Body_GetLinearVelocity(b).x==-1);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // The only blocking normal is assigned to overflow after the regular static colors fill.
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={-3,0};
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.type=b2_staticBody;bd.linearVelocity={0,0};b=b2CreateBody(id,&bd);
    for(int i=0;i<B2_GRAPH_COLOR_COUNT;++i)
    {
        c.center={i==B2_GRAPH_COLOR_COUNT-1?-2.0f:2.0f,0};b2CreateCircleShape(b,&sd,&c);
    }
    b2World_Step(id,0.01f,1);
    ENSURE(b2_worlds[id.index1-1].constraintGraph.colors[B2_OVERFLOW_INDEX].contactSims.count>0);
    ENSURE_SMALL(b2Body_GetLinearVelocity(a).x,0.00001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);c.center={0,0};
    // Sustained gravity support is exercised over many frames and substeps.
    wd.gravity={0,-10};id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.type=b2_staticBody;bd.position={0,-2};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60.0f,4); }
    ENSURE_SMALL(b2Body_GetPosition(a).y,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(a).y,0.0001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

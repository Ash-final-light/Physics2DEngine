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
    b2World_EnableWarmStarting(id,false);
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

static void SolveHardNormal(b2WorldId id)
{
    b2World* world=b2_worlds+id.index1-1;
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    b2StepContext context={world,&world->constraintGraph,world->solverSets.data[b2_awakeSet].bodyStates.data,0.01f,100.0f};
    b2PrepareContactConstraints(&context);b2WarmStartContacts(&context);b2SolveNormalContacts(&context);
    b2StoreContactImpulses(&context);b2ReleaseContactConstraints(&context);
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
    SolveHardNormal(id);
    ENSURE_SMALL(b2Body_GetLinearVelocity(a).x,0.00001f);
    ENSURE(b2Body_GetLinearVelocity(a).y==2);
    ENSURE_SMALL(b2Body_GetPosition(a).x,0.00001f);ENSURE(b2Body_GetPosition(b).x==2);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // Unequal masses retain linear momentum in an inelastic collision.
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={3,0};
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.position={2,0};bd.linearVelocity={0,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    b2Body_SetMassData(a,b2MassData{2,{0,0},1});b2Body_SetMassData(b,b2MassData{1,{0,0},1});
    SolveHardNormal(id);
    ENSURE_SMALL(b2Body_GetLinearVelocity(a).x-2,0.00001f);
    ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-2,0.00001f);
    ENSURE_SMALL(2*b2Body_GetLinearVelocity(a).x+b2Body_GetLinearVelocity(b).x-6,0.00001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={3,0};
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.position={2,0};bd.linearVelocity={-3,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    SolveHardNormal(id);
    ENSURE_SMALL(b2Body_GetLinearVelocity(a).x,0.00001f);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x,0.00001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // A moving kinematic obstacle transfers normal velocity without being slowed.
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.type=b2_kinematicBody;bd.position={2,0};bd.linearVelocity={-1,0};
    b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    SolveHardNormal(id);
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
    SolveHardNormal(id);
    ENSURE(b2_worlds[id.index1-1].constraintGraph.colors[B2_OVERFLOW_INDEX].contactSims.count>0);
    ENSURE_SMALL(b2Body_GetLinearVelocity(a).x,0.00001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);c.center={0,0};
    // Sustained normal support balances repeated gravity impulses.
    wd.gravity={0,-10};id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.type=b2_staticBody;bd.position={0,-2};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    for(int i=0;i<120;++i)
    {
        b2Body_ApplyLinearImpulseToCenter(a,{0,-b2Body_GetMass(a)/6.0f},true);SolveHardNormal(id);
    }
    ENSURE_SMALL(b2Body_GetPosition(a).y,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(a).y,0.0001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int ContactImpulseStoreTest(void)
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={0,-2};
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};
    b2BodyId body=b2CreateBody(id,&bd);b2CreateCircleShape(body,&sd,&c);
    b2Body_SetMassData(body,b2MassData{2,{0,0},1});
    bd.type=b2_staticBody;bd.position={0,-2};b2BodyId ground=b2CreateBody(id,&bd);b2CreateCircleShape(ground,&sd,&c);
    b2World_Step(id,0.01f,1);
    b2Contact* contact=world->contacts.data;b2ContactSim* sim=b2GetContactSim(world,contact);
    ENSURE_SMALL(sim->manifold.points[0].normalImpulse-4,0.00001f);
    ENSURE_SMALL(sim->manifold.points[0].totalNormalImpulse-4,0.00001f);
    b2ContactData data[1];ENSURE(b2Body_GetContactData(body,data,1)==1);
    ENSURE_SMALL(data[0].manifold.points[0].normalImpulse-4,0.00001f);
    b2UpdateContacts(world);sim=b2GetContactSim(world,contact);
    ENSURE(sim->manifold.points[0].persisted);
    ENSURE_SMALL(sim->manifold.points[0].normalImpulse-4,0.00001f);
    ENSURE(sim->manifold.points[0].totalNormalImpulse==0);
    b2World_Step(id,0.01f,1);sim=b2GetContactSim(world,contact);
    ENSURE_SMALL(sim->manifold.points[0].normalImpulse,0.00001f);
    ENSURE_SMALL(sim->manifold.points[0].totalNormalImpulse,0.00001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int ContactSubstepCorrectionTest(void)
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;wd.contactSpeed=0.5f;
    b2WorldId id=b2CreateWorld(&wd);
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{0,0},1};
    b2BodyId body=b2CreateBody(id,&bd);b2CreateCircleShape(body,&sd,&circle);
    bd.type=b2_staticBody;bd.position={0,-1.5f};b2BodyId ground=b2CreateBody(id,&bd);b2CreateCircleShape(ground,&sd,&circle);
    b2World_Step(id,0.01f,1);
    ENSURE(b2Body_GetPosition(body).y>0 && b2Body_GetPosition(body).y<=0.005001f);
    ENSURE_SMALL(b2Body_GetLinearVelocity(body).y,0.00001f);
    for(int i=0;i<240;++i) { b2World_Step(id,1.0f/60.0f,4); }
    ENSURE_SMALL(b2Body_GetPosition(body).y-0.5f,0.0002f);
    ENSURE_SMALL(b2Body_GetLinearVelocity(body).y,0.0001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // A rotated box must settle under gravity using both contact points and angular correction.
    wd=b2DefaultWorldDef();id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();
    bd.type=b2_staticBody;bd.position={0,-1};ground=b2CreateBody(id,&bd);
    b2Polygon floor=b2MakeBox(20,0.5f);b2CreatePolygonShape(ground,&sd,&floor);
    bd.type=b2_dynamicBody;bd.position={0,0.1f};bd.rotation=b2MakeRot(0.15f);
    body=b2CreateBody(id,&bd);b2Polygon box=b2MakeBox(0.5f,0.5f);b2CreatePolygonShape(body,&sd,&box);
    for(int i=0;i<360;++i) { b2World_Step(id,1.0f/60.0f,4); }
    ENSURE(b2Body_GetPosition(body).y>-0.01f && b2Body_GetPosition(body).y<0.02f);
    ENSURE_SMALL(b2Body_GetRotation(body).s,0.005f);
    ENSURE_SMALL(b2Body_GetLinearVelocity(body).y,0.01f);
    ENSURE_SMALL(b2Body_GetAngularVelocity(body),0.01f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // Smaller substeps reduce gravity-induced penetration for the same frame interval.
    float penetration[2];
    for(int run=0;run<2;++run)
    {
        id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
        body=b2CreateBody(id,&bd);b2CreateCircleShape(body,&sd,&circle);
        bd.type=b2_staticBody;bd.position={0,-2};ground=b2CreateBody(id,&bd);b2CreateCircleShape(ground,&sd,&circle);
        for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60.0f,run==0?1:4); }
        penetration[run]=-b2Body_GetPosition(body).y;
        ENSURE(penetration[run]>=0 && penetration[run]<0.01f);
        b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    ENSURE(penetration[1]<penetration[0]);return 0;
}

int ContactWarmStartTest(void)
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
    ENSURE(b2World_IsWarmStartingEnabled(id));
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();
    b2Polygon box=b2MakeBox(1,0.5f);b2CreatePolygonShape(body,&sd,&box);
    bd.type=b2_staticBody;bd.position={0,-0.9f};b2BodyId ground=b2CreateBody(id,&bd);
    b2Polygon floor=b2MakeBox(5,0.5f);b2CreatePolygonShape(ground,&sd,&floor);
    b2Body_SetMassData(body,b2MassData{2,{0,0},4});
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    b2Contact* contact=world->contacts.data;b2ContactSim* sim=b2GetContactSim(world,contact);
    sim->manifold.points[0].normalImpulse=4;
    for(int enabled=0;enabled<2;++enabled)
    {
        b2World_EnableWarmStarting(id,enabled!=0);ENSURE(b2World_IsWarmStartingEnabled(id)==(enabled!=0));
        world->locked=true;b2World_EnableWarmStarting(id,enabled==0);world->locked=false;
        ENSURE(b2World_IsWarmStartingEnabled(id)==(enabled!=0));
        b2Body_SetLinearVelocity(body,{0,0});b2Body_SetAngularVelocity(body,0);
        b2StepContext context={world,&world->constraintGraph,world->solverSets.data[b2_awakeSet].bodyStates.data,0.01f,100};
        b2PrepareContactConstraints(&context);
        b2ContactConstraint* constraint=world->constraintGraph.colors[contact->colorIndex].overflowConstraints+contact->localIndex;
        ENSURE(constraint->points[0].normalImpulse==(enabled?4.0f:0.0f));
        b2Vec2 P=b2MulSV(enabled?4.0f:0.0f,sim->manifold.normal);
        b2Vec2 expected=b2MulSV(-0.5f,P);
        float expectedW=-0.25f*b2Cross(sim->manifold.points[0].anchorA,P);
        b2WarmStartContacts(&context);
        ENSURE_SMALL(b2Body_GetLinearVelocity(body).x-expected.x,0.00001f);
        ENSURE_SMALL(b2Body_GetLinearVelocity(body).y-expected.y,0.00001f);
        ENSURE_SMALL(b2Body_GetAngularVelocity(body)-expectedW,0.00001f);
        ENSURE(constraint->points[0].totalNormalImpulse==(enabled?4.0f:0.0f));
        ENSURE(b2Body_GetPosition(ground).y==-0.9f && b2Body_GetLinearVelocity(ground).y==0);
        b2ReleaseContactConstraints(&context);
    }
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // Persistent warm starting must keep a small stack bounded over many frames.
    wd=b2DefaultWorldDef();id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_staticBody;bd.position={0,-1};
    ground=b2CreateBody(id,&bd);b2CreatePolygonShape(ground,&sd,&floor);
    b2BodyId stack[3];box=b2MakeBox(0.5f,0.5f);
    for(int i=0;i<3;++i)
    {
        bd.type=b2_dynamicBody;bd.position={0,1.0f*i};stack[i]=b2CreateBody(id,&bd);b2CreatePolygonShape(stack[i],&sd,&box);
    }
    for(int i=0;i<360;++i) { b2World_Step(id,1.0f/60.0f,4); }
    for(int i=0;i<3;++i)
    {
        ENSURE_SMALL(b2Body_GetPosition(stack[i]).y-i,0.03f);
        ENSURE_SMALL(b2Body_GetLinearVelocity(stack[i]).y,0.02f);
    }
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int ContactTuningTest(void)
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId id=b2CreateWorld(&wd);b2World* w=b2_worlds+id.index1-1;
    float hz=-1,zeta=-1,speed=-1;
    b2World_GetContactTuning(id,&hz,&zeta,&speed);
    ENSURE(hz==wd.contactHertz && zeta==wd.contactDampingRatio && speed==wd.contactSpeed);
    b2World_SetContactTuning(id,12,2,0.25f);b2World_GetContactTuning(id,&hz,&zeta,&speed);
    ENSURE(hz==12 && zeta==2 && speed==0.25f);
    w->locked=true;b2World_SetContactTuning(id,1,1,1);w->locked=false;
    b2World_SetContactTuning(id,NAN,1,1);b2World_SetContactTuning(id,1,INFINITY,1);
    b2World_GetContactTuning(id,&hz,&zeta,&speed);ENSURE(hz==12 && zeta==2 && speed==0.25f);
    b2World_SetContactTuning(id,-1,-2,-3);b2World_GetContactTuning(id,&hz,&zeta,&speed);
    ENSURE(hz==0 && zeta==0 && speed==0);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    float displacement[4];
    const float hertz[4]={0,10,10,10},damping[4]={1,1,10,1},push[4]={10,10,10,0.01f};
    for(int run=0;run<4;++run)
    {
        id=b2CreateWorld(&wd);w=b2_worlds+id.index1-1;b2World_SetContactTuning(id,hertz[run],damping[run],push[run]);
        b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
        b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};b2CreateCircleShape(body,&sd,&c);
        bd.type=b2_staticBody;bd.position={0,-1.5f};b2BodyId ground=b2CreateBody(id,&bd);b2CreateCircleShape(ground,&sd,&c);
        b2World_Step(id,0.01f,1);displacement[run]=b2Body_GetPosition(body).y;
        b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    ENSURE(displacement[0]==0 && displacement[1]>0);
    ENSURE(displacement[2]<displacement[1] && displacement[3]<=0.0001001f);
    ENSURE(displacement[3]>0);
    // Frequency clipping is applied before doubling the static contact frequency.
    id=b2CreateWorld(&wd);w=b2_worlds+id.index1-1;b2World_SetContactTuning(id,10000,1,10);
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;b2BodyId body=b2CreateBody(id,&bd);
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};b2CreateCircleShape(body,&sd,&c);
    bd.type=b2_staticBody;bd.position={0,-1.5f};b2BodyId ground=b2CreateBody(id,&bd);b2CreateCircleShape(ground,&sd,&c);
    b2UpdateBroadPhaseContacts(w);b2UpdateContacts(w);
    b2StepContext context={w,&w->constraintGraph,w->solverSets.data[b2_awakeSet].bodyStates.data,0.01f,100};
    b2PrepareContactConstraints(&context);
    b2Contact* contact=w->contacts.data;
    const b2Softness actual=w->constraintGraph.colors[contact->colorIndex].overflowConstraints[contact->localIndex].softness;
    const b2Softness expected=b2MakeSoft(25,1,0.01f);
    ENSURE_SMALL(actual.biasRate-expected.biasRate,0.00001f);
    ENSURE_SMALL(actual.massScale-expected.massScale,0.00001f);
    ENSURE_SMALL(actual.impulseScale-expected.impulseScale,0.00001f);
    b2ReleaseContactConstraints(&context);b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

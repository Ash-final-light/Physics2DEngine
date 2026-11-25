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
    b2ShapeDef sd=b2DefaultShapeDef();sd.material.friction=0;b2Circle c={{0,0},1};
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
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableContactRecycling=false;bd.linearVelocity={0,-2};
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

int ContactFrictionTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    for(int run=0;run<2;++run)
    {
        b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
        b2World_SetContactRecycleDistance(id,0);
        b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={10,-2};
        b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.material.friction=run?0.25f:0;
        b2Circle c={{0,0},1};b2ShapeId shape=b2CreateCircleShape(body,&sd,&c);b2Body_SetMassData(body,b2MassData{1,{0,0},0});
        bd.type=b2_staticBody;bd.position={0,-2};sd.material.friction=1;
        b2BodyId ground=b2CreateBody(id,&bd);b2ShapeId fixed=b2CreateCircleShape(ground,&sd,&c);
        SolveHardNormal(id);
        b2Contact* contact=world->contacts.data;b2ContactSim* sim=b2GetContactSim(world,contact);
        ENSURE(sim->friction==(run?0.5f:0.0f));
        ENSURE_SMALL(b2Body_GetLinearVelocity(body).x-(run?9.0f:10.0f),0.00001f);
        ENSURE_SMALL(b2Body_GetLinearVelocity(body).y,0.00001f);
        ENSURE_SMALL(b2AbsFloat(sim->manifold.points[0].tangentImpulse)-(run?1.0f:0.0f),0.00001f);
        ENSURE(b2AbsFloat(sim->manifold.points[0].tangentImpulse)<=sim->friction*sim->manifold.points[0].normalImpulse+0.00001f);
        b2Shape_SetFriction(shape,0.36f);b2Shape_SetFriction(shape,-1);b2Shape_SetFriction(shape,NAN);
        ENSURE(b2Shape_GetFriction(shape)==0.36f);world->locked=true;b2Shape_SetFriction(shape,2);world->locked=false;
        ENSURE(b2Shape_GetFriction(shape)==0.36f);
        b2UpdateContacts(world);sim=b2GetContactSim(world,contact);ENSURE_SMALL(sim->friction-0.6f,0.00001f);
        ENSURE(b2Shape_GetFriction(fixed)==1);
        // Cached tangent impulse is loaded and applied by warm starting.
        b2Body_SetLinearVelocity(body,{0,0});b2StepContext context={world,&world->constraintGraph,world->solverSets.data[b2_awakeSet].bodyStates.data,0.01f,100};
        b2PrepareContactConstraints(&context);b2WarmStartContacts(&context);
        ENSURE_SMALL(b2AbsFloat(b2Body_GetLinearVelocity(body).x)-(run?1.0f:0.0f),0.00001f);
        b2ReleaseContactConstraints(&context);b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    // On flat ground, mu=0.5 produces approximately 5 units/s deceleration in one second.
    wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={10,0};bd.enableContactRecycling=false;
    b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.material.friction=0.25f;
    b2Circle c={{0,0},1};b2CreateCircleShape(body,&sd,&c);b2Body_SetMassData(body,b2MassData{1,{0,0},0});
    bd.type=b2_staticBody;bd.position={0,-1.5f};b2BodyId ground=b2CreateBody(id,&bd);
    sd.material.friction=1;b2Polygon floor=b2MakeBox(100,0.5f);b2CreatePolygonShape(ground,&sd,&floor);
    for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60.0f,4); }
    ENSURE_SMALL(b2Body_GetLinearVelocity(body).x-5,0.05f);
    ENSURE_SMALL(b2Body_GetAngularVelocity(body),0.00001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int ContactRollingResistanceTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    for(int run=0;run<3;++run)
    {
        b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
        b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={0,-2};bd.angularVelocity=10;
        b2BodyId a=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.material.friction=0;
        sd.material.rollingResistance=run?0.1f:0;b2Circle c={{0,0},2};b2CreateCircleShape(a,&sd,&c);
        b2Body_SetMassData(a,b2MassData{1,{0,0},1});
        bd.type=run==2?b2_dynamicBody:b2_staticBody;bd.position={0,-3};bd.linearVelocity={0,0};bd.angularVelocity=-10;
        b2BodyId b=b2CreateBody(id,&bd);sd.material.rollingResistance=run?0.2f:0;c.radius=1;b2CreateCircleShape(b,&sd,&c);
        if(run==2) { b2Body_SetMassData(b,b2MassData{1,{0,0},1}); }
        SolveHardNormal(id);b2Contact* contact=world->contacts.data;b2ContactSim* sim=b2GetContactSim(world,contact);
        float expected=run==0?0.0f:run==1?0.8f:0.4f;
        ENSURE_SMALL(sim->rollingResistance-(run?0.4f:0.0f),0.00001f);
        ENSURE_SMALL(sim->manifold.rollingImpulse-expected,0.00001f);
        ENSURE_SMALL(b2Body_GetAngularVelocity(a)-(10-expected),0.00001f);
        if(run==2)
        {
            ENSURE_SMALL(b2Body_GetAngularVelocity(b)-(-10+expected),0.00001f);
            ENSURE_SMALL(b2Body_GetAngularVelocity(a)+b2Body_GetAngularVelocity(b),0.00001f);
        }
        b2Body_SetAngularVelocity(a,0);b2Body_SetLinearVelocity(a,{0,0});
        b2StepContext context={world,&world->constraintGraph,world->solverSets.data[b2_awakeSet].bodyStates.data,0.01f,100};
        b2PrepareContactConstraints(&context);b2WarmStartContacts(&context);
        ENSURE_SMALL(b2Body_GetAngularVelocity(a)+expected,0.00001f);
        b2ReleaseContactConstraints(&context);b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    return 0;
}

int ContactTangentSpeedTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    for(int run=0;run<2;++run)
    {
        b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;b2World_SetContactRecycleDistance(id,0);
        b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={0,-2};
        b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.material.friction=run?10.0f:0.0f;sd.material.tangentSpeed=1;
        b2Circle c={{0,0},1};b2ShapeId shape=b2CreateCircleShape(body,&sd,&c);b2Body_SetMassData(body,b2MassData{1,{0,0},0});
        bd.type=b2_staticBody;bd.position={0,-1.5f};b2BodyId ground=b2CreateBody(id,&bd);sd.material.tangentSpeed=2;
        b2Polygon floor=b2MakeBox(100,0.5f);b2ShapeId fixed=b2CreatePolygonShape(ground,&sd,&floor);
        SolveHardNormal(id);b2ContactSim* sim=b2GetContactSim(world,world->contacts.data);
        ENSURE(sim->tangentSpeed==3);ENSURE_SMALL(b2Body_GetLinearVelocity(body).x-(run?3.0f:0.0f),0.00001f);
        b2SurfaceMaterial material=b2Shape_GetSurfaceMaterial(fixed);ENSURE(material.tangentSpeed==2);
        material.tangentSpeed=-4;b2Shape_SetSurfaceMaterial(fixed,&material);
        ENSURE(b2Shape_GetSurfaceMaterial(fixed).tangentSpeed==-4);
        material.tangentSpeed=NAN;b2Shape_SetSurfaceMaterial(fixed,&material);ENSURE(b2Shape_GetSurfaceMaterial(fixed).tangentSpeed==-4);
        b2UpdateContacts(world);ENSURE(b2GetContactSim(world,world->contacts.data)->tangentSpeed==-3);
        ENSURE(b2Shape_GetSurfaceMaterial(shape).tangentSpeed==1);
        b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);b2World_SetContactRecycleDistance(id,0);
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;b2BodyId body=b2CreateBody(id,&bd);
    b2ShapeDef sd=b2DefaultShapeDef();sd.material.friction=1;b2Circle c={{0,0},1};b2CreateCircleShape(body,&sd,&c);
    b2Body_SetMassData(body,b2MassData{1,{0,0},0});bd.type=b2_staticBody;bd.position={0,-1.5f};
    b2BodyId ground=b2CreateBody(id,&bd);sd.material.tangentSpeed=2;b2Polygon floor=b2MakeBox(100,0.5f);
    b2ShapeId fixed=b2CreatePolygonShape(ground,&sd,&floor);
    for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60.0f,4); }
    ENSURE_SMALL(b2Body_GetLinearVelocity(body).x-2,0.0001f);
    b2SurfaceMaterial material=b2Shape_GetSurfaceMaterial(fixed);material.tangentSpeed=-1;b2Shape_SetSurfaceMaterial(fixed,&material);
    for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60.0f,4); }
    ENSURE_SMALL(b2Body_GetLinearVelocity(body).x+1,0.0001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int ContactRestitutionTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    const float restitution[4]={0,0.5f,1,0.75f};
    for(int run=0;run<4;++run)
    {
        b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
        b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={0,-4};
        b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.material.friction=0;sd.material.restitution=run==3?0.25f:restitution[run];
        b2Circle c={{0,0},1};b2ShapeId shape=b2CreateCircleShape(body,&sd,&c);
        bd.type=b2_staticBody;bd.position={0,-2};sd.material.restitution=run==3?0.75f:0;
        b2BodyId ground=b2CreateBody(id,&bd);b2CreateCircleShape(ground,&sd,&c);
        b2World_Step(id,0.01f,4);
        b2ContactSim* sim=b2GetContactSim(world,world->contacts.data);
        ENSURE(sim->restitution==restitution[run]);
        ENSURE_SMALL(b2Body_GetLinearVelocity(body).y-4*restitution[run],0.0001f);
        ENSURE(b2Body_GetLinearVelocity(body).y<=4.0001f);
        b2Shape_SetRestitution(shape,0.2f);b2Shape_SetRestitution(shape,-1);b2Shape_SetRestitution(shape,NAN);
        ENSURE(b2Shape_GetRestitution(shape)==0.2f);
        b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={4,0};
    b2ShapeDef sd=b2DefaultShapeDef();sd.material.restitution=1;sd.material.friction=0;b2Circle c={{0,0},1};
    b2BodyId a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.position={2,0};bd.linearVelocity={-4,0};b2BodyId b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    b2World_Step(id,0.01f,4);
    float va=b2Body_GetLinearVelocity(a).x,vb=b2Body_GetLinearVelocity(b).x;
    ENSURE_SMALL(va+4,0.0001f);ENSURE_SMALL(vb-4,0.0001f);ENSURE_SMALL(va+vb,0.0001f);
    ENSURE(va*va+vb*vb<=32.0001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // A separating contact does not gain another restitution impulse.
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={0,4};
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);bd.type=b2_staticBody;bd.position={0,-2};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetLinearVelocity(a).y-4,0.00001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // An approaching speculative point with no compression impulse must not bounce.
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={0,-4};
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);bd.type=b2_staticBody;bd.position={0,-2.01f};
    b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    b2World_Step(id,0.001f,1);ENSURE_SMALL(b2Body_GetLinearVelocity(a).y+4,0.00001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int ContactRestitutionThresholdTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    const float incoming[4]={0.5f,1,2,0.5f};
    for(int run=0;run<4;++run)
    {
        b2WorldId id=b2CreateWorld(&wd);b2World* w=b2_worlds+id.index1-1;
        b2World_SetRestitutionThreshold(id,run==3?0.0f:1.0f);
        float expectedThreshold=run==3?0.0f:1.0f;
        b2World_SetRestitutionThreshold(id,NAN);w->locked=true;b2World_SetRestitutionThreshold(id,10);w->locked=false;
        ENSURE(b2World_GetRestitutionThreshold(id)==expectedThreshold);
        b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={0,-incoming[run]};
        b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.material.restitution=1;b2Circle c={{0,0},1};b2CreateCircleShape(body,&sd,&c);
        bd.type=b2_staticBody;bd.position={0,-2};b2BodyId ground=b2CreateBody(id,&bd);b2CreateCircleShape(ground,&sd,&c);
        b2World_Step(id,0.01f,1);
        ENSURE_SMALL(b2Body_GetLinearVelocity(body).y-(run>=2?incoming[run]:0.0f),0.0001f);
        b2World_SetRestitutionThreshold(id,-1);ENSURE(b2World_GetRestitutionThreshold(id)==0);
        b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    return 0;
}

int ContactRestitutionControlTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    for(int run=0;run<2;++run)
    {
        b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
        ENSURE(b2World_GetRestitutionIterations(id)==wd.restitutionIterations);
        ENSURE(!b2World_IsRestitutionPropagationEnabled(id));
        b2World_SetRestitutionIterations(id,1000);ENSURE(b2World_GetRestitutionIterations(id)==B2_MAX_RESTITUTION_ITERATIONS);
        b2World_SetRestitutionIterations(id,-1);ENSURE(b2World_GetRestitutionIterations(id)==B2_MAX_RESTITUTION_ITERATIONS);
        b2World_SetRestitutionIterations(id,run?2:0);
        world->locked=true;b2World_SetRestitutionIterations(id,3);b2World_EnableRestitutionPropagation(id,true);world->locked=false;
        ENSURE(b2World_GetRestitutionIterations(id)==(run?2:0));ENSURE(!b2World_IsRestitutionPropagationEnabled(id));
        b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={0,-4};
        b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.material.restitution=1;b2Circle c={{0,0},1};b2CreateCircleShape(body,&sd,&c);
        bd.type=b2_staticBody;bd.position={0,-2};b2BodyId ground=b2CreateBody(id,&bd);b2CreateCircleShape(ground,&sd,&c);
        b2World_Step(id,0.01f,4);ENSURE_SMALL(b2Body_GetLinearVelocity(body).y-(run?4.0f:0.0f),0.0001f);
        b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    float finalVelocity[2][4];
    for(int propagate=0;propagate<2;++propagate)
    {
        b2WorldId id=b2CreateWorld(&wd);b2World_EnableRestitutionPropagation(id,propagate!=0);
        b2World_SetRestitutionIterations(id,16);b2BodyId bodies[4];
        for(int i=0;i<4;++i)
        {
            b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.position={2.0f*i,0};bd.linearVelocity={i==0?8.0f:0.0f,0};
            bodies[i]=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.material.restitution=i==0?1.0f:0.0f;
            b2Circle c={{0,0},1};b2CreateCircleShape(bodies[i],&sd,&c);b2Body_SetMassData(bodies[i],b2MassData{1,{0,0},1});
        }
        b2World_Step(id,0.001f,1);
        float momentum=0,energy=0;
        for(int i=0;i<4;++i)
        { float v=b2Body_GetLinearVelocity(bodies[i]).x;finalVelocity[propagate][i]=v;momentum+=v;energy+=v*v; }
        ENSURE_SMALL(momentum-8,0.0001f);ENSURE(energy<=64.0001f);
        b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    float difference=0;for(int i=0;i<4;++i) { difference+=b2AbsFloat(finalVelocity[0][i]-finalVelocity[1][i]); }
    ENSURE(difference>0.1f);
    // Propagation resolves the approach at the last zero-restitution contact.
    ENSURE(finalVelocity[1][3]-finalVelocity[1][2]>-0.001f);
    return 0;
}

static uint64_t expectedMaterialA;
static bool materialInputsValid;
static int frictionCalls,restitutionCalls;
static float MixTestFriction(float a,uint64_t idA,float b,uint64_t idB)
{
    bool forward=idA==expectedMaterialA && idB==29 && a==0.04f && b==0.81f;
    bool reverse=idB==expectedMaterialA && idA==29 && b==0.04f && a==0.81f;
    materialInputsValid&=forward||reverse;++frictionCalls;return 0.25f;
}
static float MixTestRestitution(float a,uint64_t idA,float b,uint64_t idB)
{
    bool forward=idA==expectedMaterialA && idB==29 && a==0.2f && b==0.3f;
    bool reverse=idB==expectedMaterialA && idA==29 && b==0.2f && a==0.3f;
    materialInputsValid&=forward||reverse;++restitutionCalls;return 0.5f;
}

int ContactMaterialCallbackTest(void)
{
    size_t before=b2GetByteCount();expectedMaterialA=17;materialInputsValid=true;frictionCalls=0;restitutionCalls=0;
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;wd.frictionCallback=MixTestFriction;wd.restitutionCallback=MixTestRestitution;
    b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;b2World_SetContactRecycleDistance(id,0);
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.linearVelocity={10,-4};
    b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.material.friction=0.04f;sd.material.restitution=0.2f;sd.material.userMaterialId=17;
    b2Circle c={{0,0},1};b2ShapeId shape=b2CreateCircleShape(body,&sd,&c);b2Body_SetMassData(body,b2MassData{1,{0,0},0});
    bd.type=b2_staticBody;bd.position={0,-1.5f};b2BodyId ground=b2CreateBody(id,&bd);
    sd.material.friction=0.81f;sd.material.restitution=0.3f;sd.material.userMaterialId=29;
    b2Polygon floor=b2MakeBox(100,0.5f);b2CreatePolygonShape(ground,&sd,&floor);
    world->locked=true;b2World_SetFrictionCallback(id,NULL);b2World_SetRestitutionCallback(id,NULL);world->locked=false;
    b2World_Step(id,0.01f,1);
    ENSURE(materialInputsValid && frictionCalls==1 && restitutionCalls==1);
    b2ContactSim* sim=b2GetContactSim(world,world->contacts.data);ENSURE(sim->friction==0.25f && sim->restitution==0.5f);
    ENSURE_SMALL(b2Body_GetLinearVelocity(body).x-9,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(body).y-2,0.0001f);
    expectedMaterialA=(uint64_t(1)<<45)+17;b2Shape_SetUserMaterial(shape,expectedMaterialA);
    ENSURE(b2Shape_GetUserMaterial(shape)==expectedMaterialA);
    b2UpdateContacts(world);ENSURE(materialInputsValid && frictionCalls==2 && restitutionCalls==2);
    b2World_SetFrictionCallback(id,NULL);b2World_SetRestitutionCallback(id,NULL);b2UpdateContacts(world);
    sim=b2GetContactSim(world,world->contacts.data);ENSURE_SMALL(sim->friction-0.18f,0.00001f);ENSURE(sim->restitution==0.3f);
    ENSURE(frictionCalls==2 && restitutionCalls==2);
    b2World_SetFrictionCallback(id,MixTestFriction);b2World_SetRestitutionCallback(id,MixTestRestitution);b2UpdateContacts(world);
    ENSURE(materialInputsValid && frictionCalls==3 && restitutionCalls==3);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int SpeculativeContactTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;wd.enableContinuous=false;
    for(int run=0;run<4;++run)
    {
        b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
        b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
        bd.linearVelocity=run==0?b2Vec2{0.25f,0}:run==2?b2Vec2{0,4}:b2Vec2{4,0};
        b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.material.friction=1;
        b2Circle c={{0,0},1};b2CreateCircleShape(body,&sd,&c);
        float gap=run==3?1.5f*B2_SPECULATIVE_DISTANCE:0.5f*B2_SPECULATIVE_DISTANCE;
        bd.type=b2_staticBody;bd.position={2+gap,0};b2BodyId fixed=b2CreateBody(id,&bd);b2CreateCircleShape(fixed,&sd,&c);
        b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);ENSURE(world->broadPhase.pairSet.count==1);
        b2ContactSim* sim=b2GetContactSim(world,world->contacts.data);
        ENSURE(sim->manifold.pointCount==(run==3?0:1));
        if(run!=3) { ENSURE_SMALL(sim->manifold.points[0].separation-gap,0.00001f); }
        b2World_Step(id,run==3?0.001f:0.01f,1);
        b2Vec2 velocity=b2Body_GetLinearVelocity(body);b2Pos position=b2Body_GetPosition(body);
        if(run==0)
        { ENSURE_SMALL(velocity.x-0.25f,0.00001f);ENSURE_SMALL(position.x-0.0025f,0.00001f); }
        else if(run==1)
        { ENSURE(position.x<=gap+0.00001f);ENSURE(velocity.x<4 && position.x>0); }
        else if(run==2)
        { ENSURE_SMALL(velocity.x,0.00001f);ENSURE_SMALL(velocity.y-4,0.00001f); }
        else
        { ENSURE_SMALL(velocity.x-4,0.00001f);ENSURE_SMALL(position.x-0.004f,0.00001f); }
        b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    return 0;
}

int ContinuousCollisionTest(void)
{
    size_t before=b2GetByteCount();
    for(int run=0;run<5;++run)
    {
        b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;wd.enableContinuous=run!=3;
        b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
        b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.position={-2,0};bd.linearVelocity={100,0};
        b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();
        if(run==1) { b2Capsule capsule={{0,-0.3f},{0,0.3f},0.1f};b2CreateCapsuleShape(body,&sd,&capsule); }
        else if(run==2) { b2Polygon box=b2MakeBox(0.1f,0.3f);b2CreatePolygonShape(body,&sd,&box); }
        else { b2Circle circle={{0,0},0.1f};b2CreateCircleShape(body,&sd,&circle); }
        bd.type=b2_staticBody;bd.position={0,0};bd.linearVelocity={0,0};b2BodyId obstacle=b2CreateBody(id,&bd);
        if(run==4) { sd.filter.maskBits=0; }
        b2Polygon wall=b2MakeBox(0.05f,10);b2CreatePolygonShape(obstacle,&sd,&wall);
        b2World_Step(id,0.05f,1);
        if(run<3)
        {
            ENSURE(b2Body_GetPosition(body).x<0 && b2Body_GetPosition(body).x>-0.2f);
            b2Body* cold=world->bodies.data+body.index1-1;ENSURE((b2GetBodySim(world,cold)->flags&b2_hadTimeOfImpact)!=0);
            for(int i=0;i<30;++i) { b2World_Step(id,1.0f/60.0f,4); }
            ENSURE(b2Body_GetPosition(body).x<0);
        }
        else { ENSURE_SMALL(b2Body_GetPosition(body).x-3,0.00001f); }
        b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    // Teleporting updates the sweep origin and must not sweep through an intervening wall.
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.position={-2,0};bd.linearVelocity={1,0};
    b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{0,0},0.1f};b2CreateCircleShape(body,&sd,&circle);
    bd.type=b2_staticBody;bd.position={0,0};b2BodyId obstacle=b2CreateBody(id,&bd);b2Polygon wall=b2MakeBox(0.05f,10);b2CreatePolygonShape(obstacle,&sd,&wall);
    b2Body_SetTransform(body,{2,0},b2Rot_identity);b2World_Step(id,0.2f,1);
    ENSURE_SMALL(b2Body_GetPosition(body).x-2.2f,0.00001f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // One-sided chain edges retain the reference orientation test during a fast sweep.
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_staticBody;obstacle=b2CreateBody(id,&bd);
    b2Vec2 vertices[4]={{0,-10},{10,-10},{10,10},{0,10}};
    b2SurfaceMaterial material=b2DefaultSurfaceMaterial();b2ChainDef chain=b2DefaultChainDef();
    chain.points=vertices;chain.pointCount=4;chain.isLoop=true;chain.materials=&material;chain.materialCount=1;
    b2CreateChain(obstacle,&chain);bd.type=b2_dynamicBody;bd.position={-2,0};bd.linearVelocity={100,0};
    body=b2CreateBody(id,&bd);b2CreateCircleShape(body,&sd,&circle);b2World_Step(id,0.05f,1);
    ENSURE(b2Body_GetPosition(body).x<0 && b2Body_GetPosition(body).x>-0.15f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // Recentered TOI also works away from the world origin.
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.position={998,0};bd.linearVelocity={100,0};
    body=b2CreateBody(id,&bd);b2CreateCircleShape(body,&sd,&circle);
    bd.type=b2_staticBody;bd.position={1000,0};obstacle=b2CreateBody(id,&bd);b2CreatePolygonShape(obstacle,&sd,&wall);
    b2World_Step(id,0.05f,1);ENSURE(b2Body_GetPosition(body).x<1000 && b2Body_GetPosition(body).x>999.8f);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int BulletCollisionTest(void)
{
    size_t before=b2GetByteCount();
    for(int run=0;run<5;++run)
    {
        b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);
        b2World* world=b2_worlds+id.index1-1;
        b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.position={-2,0};bd.linearVelocity={100,0};bd.isBullet=run!=0;
        b2BodyId bullet=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},0.1f};b2CreateCircleShape(bullet,&sd,&c);
        ENSURE(b2Body_IsBullet(bullet)==(run!=0));
        bd.type=run==2?b2_kinematicBody:b2_dynamicBody;bd.position={0,0};bd.linearVelocity=run==2?b2Vec2{-10,0}:b2Vec2{0,0};bd.isBullet=run==3;
        b2BodyId target=b2CreateBody(id,&bd);b2Polygon wall=b2MakeBox(0.05f,10);
        if(run==4) { sd.filter.maskBits=0; }
        b2CreatePolygonShape(target,&sd,&wall);
        b2World_Step(id,0.05f,1);
        if(run==1 || run==2)
        {
            ENSURE(b2Body_GetPosition(bullet).x<b2Body_GetPosition(target).x);
            ENSURE(b2Body_GetPosition(bullet).x>b2Body_GetPosition(target).x-0.2f);
            b2Body* cold=world->bodies.data+bullet.index1-1;ENSURE((b2GetBodySim(world,cold)->flags&b2_hadTimeOfImpact)!=0);
        }
        else { ENSURE_SMALL(b2Body_GetPosition(bullet).x-3,0.00001f); }
        world->locked=true;b2Body_SetBullet(bullet,run==0);world->locked=false;ENSURE(b2Body_IsBullet(bullet)==(run!=0));
        b2Body_SetBullet(bullet,false);ENSURE(!b2Body_IsBullet(bullet));b2Body_SetBullet(bullet,true);ENSURE(b2Body_IsBullet(bullet));
        b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    return 0;
}

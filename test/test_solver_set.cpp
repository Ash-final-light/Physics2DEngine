// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "world.h"
#include "box2d/types.h"

int SolverSetOrganizationTest( void )
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId worldId=b2CreateWorld(&wd);
    b2World* world=b2_worlds+worldId.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2BodyId first=b2CreateBody(worldId,&bd);
    bd.position={10.0f,0.0f};b2BodyId moving=b2CreateBody(worldId,&bd);
    bd.type=b2_kinematicBody;bd.position={20.0f,0.0f};b2BodyId kinematic=b2CreateBody(worldId,&bd);
    bd.type=b2_staticBody;bd.position={10.0f,1.5f};b2BodyId fixed=b2CreateBody(worldId,&bd);
    bd.type=b2_dynamicBody;bd.isEnabled=false;b2BodyId disabled=b2CreateBody(worldId,&bd);
    bd.isEnabled=true;bd.isAwake=false;bd.position={30.0f,0.0f};b2BodyId sleeping=b2CreateBody(worldId,&bd);
    b2Body* sleepBody=world->bodies.data+sleeping.index1-1;
    int sleepingSet=sleepBody->setIndex;
    ENSURE(sleepingSet>=b2_firstSleepingSet);
    ENSURE(world->solverSets.data[b2_awakeSet].bodySims.count==3);
    ENSURE(world->solverSets.data[b2_awakeSet].bodyStates.count==3);
    ENSURE(world->solverSets.data[b2_staticSet].bodyStates.count==0);
    ENSURE(world->solverSets.data[b2_disabledSet].bodyStates.count==0);
    ENSURE(world->bodies.data[disabled.index1-1].setIndex==b2_disabledSet);
    ENSURE(b2GetBodyState(world,sleepBody)==NULL);
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{0.0f,0.0f},1.0f};
    b2CreateCircleShape(moving,&sd,&circle);b2CreateCircleShape(fixed,&sd,&circle);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    ENSURE(world->broadPhase.pairSet.count==1);
    b2Contact* contact=world->contacts.data;
    b2Body* body=world->bodies.data+moving.index1-1;
    ENSURE(b2GetContactSim(world,contact)->encodedBodySimA==body->localIndex);
    b2DestroyBody(first);
    for(int i=0;i<world->solverSets.data[b2_awakeSet].bodySims.count;++i)
    {
        int id=world->solverSets.data[b2_awakeSet].bodySims.data[i].bodyId;
        ENSURE(world->bodies.data[id].localIndex==i);
        ENSURE(world->bodies.data[id].setIndex==b2_awakeSet);
    }
    ENSURE(b2GetContactSim(world,contact)->encodedBodySimA==world->bodies.data[moving.index1-1].localIndex);
    ENSURE(b2Body_GetPosition(kinematic).x==20.0f && b2Body_GetPosition(moving).x==10.0f);
    b2Body_SetLinearVelocity(sleeping,{3.0f,0.0f});
    ENSURE(world->bodies.data[sleeping.index1-1].setIndex==b2_awakeSet);
    ENSURE(world->solverSets.data[sleepingSet].setIndex==B2_NULL_INDEX);
    ENSURE(b2Body_GetLinearVelocity(sleeping).x==3.0f);
    ENSURE(world->solverSets.data[b2_awakeSet].bodySims.count==3);
    b2World_Step(worldId,0.1f,1);
    ENSURE_SMALL(b2Body_GetPosition(sleeping).x-30.3f,0.00001f);
    b2DestroyWorld(worldId);ENSURE(b2GetByteCount()==before);return 0;
}

int IslandSleepTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;b2BodyId a=b2CreateBody(id,&bd);
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};b2CreateCircleShape(a,&sd,&c);
    bd.position={2,0};b2BodyId b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    for(int i=0;i<20;++i) { b2World_Step(id,1.0f/60.0f,4); }
    ENSURE(b2Body_IsAwake(a) && b2Body_IsAwake(b));
    for(int i=0;i<20;++i) { b2World_Step(id,1.0f/60.0f,4); }
    ENSURE(!b2Body_IsAwake(a) && !b2Body_IsAwake(b));
    int sleepSet=world->bodies.data[a.index1-1].setIndex;
    ENSURE(sleepSet>=b2_firstSleepingSet && world->bodies.data[b.index1-1].setIndex==sleepSet);
    ENSURE(world->solverSets.data[sleepSet].bodyStates.count==0 && world->solverSets.data[sleepSet].contactSims.count==1);
    ENSURE(world->solverSets.data[b2_awakeSet].bodySims.count==0);
    b2Contact* contact=world->contacts.data;ENSURE(contact->colorIndex==B2_NULL_INDEX && contact->setIndex==sleepSet);
    b2Body_SetLinearVelocity(a,{0,1});ENSURE(b2Body_IsAwake(a) && b2Body_IsAwake(b));
    ENSURE(world->solverSets.data[sleepSet].setIndex==B2_NULL_INDEX);
    ENSURE(contact->colorIndex!=B2_NULL_INDEX && contact->setIndex==b2_awakeSet);
    b2World_Step(id,0.01f,1);ENSURE(b2Body_GetPosition(a).y>0);
    b2Body_SetAwake(a,false);ENSURE(!b2Body_IsAwake(a) && !b2Body_IsAwake(b));
    b2Body_SetAwake(b,true);ENSURE(b2Body_IsAwake(a) && b2Body_IsAwake(b));
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // Any active member keeps an island awake; a non-touching neighbor may sleep independently.
    id=b2CreateWorld(&wd);world=b2_worlds+id.index1-1;bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableSleep=false;
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);bd.enableSleep=true;bd.position={2,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    bd.position={20,0};b2BodyId independent=b2CreateBody(id,&bd);b2CreateCircleShape(independent,&sd,&c);
    for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60.0f,4); }
    ENSURE(b2Body_IsAwake(a) && b2Body_IsAwake(b) && !b2Body_IsAwake(independent));
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // Non-touching pairs between sleeping islands belong to the disabled set.
    id=b2CreateWorld(&wd);world=b2_worlds+id.index1-1;bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);bd.position={2.04f,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    for(int i=0;i<40;++i) { b2World_Step(id,1.0f/60.0f,1); }
    contact=world->contacts.data;ENSURE(!b2Body_IsAwake(a) && !b2Body_IsAwake(b));
    ENSURE(contact->setIndex==b2_disabledSet && b2GetContactSim(world,contact)->manifold.pointCount==0);
    b2Body_SetAwake(a,true);ENSURE(contact->setIndex==b2_awakeSet && !b2Body_IsAwake(b));
    b2Body_SetTransform(a,{0.05f,0},b2Rot_identity);b2UpdateContacts(world);
    ENSURE(b2Body_IsAwake(b) && contact->colorIndex!=B2_NULL_INDEX);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // Many independent sleep sets force solver-set storage to grow during finalization.
    id=b2CreateWorld(&wd);world=b2_worlds+id.index1-1;bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2BodyId many[24];
    for(int i=0;i<24;++i)
    { bd.position={10.0f*i,0};many[i]=b2CreateBody(id,&bd);b2CreateCircleShape(many[i],&sd,&c); }
    for(int i=0;i<40;++i) { b2World_Step(id,1.0f/60.0f,1); }
    for(int i=0;i<24;++i)
    {
        ENSURE(!b2Body_IsAwake(many[i]));b2Body_SetAwake(many[i],true);ENSURE(b2Body_IsAwake(many[i]));
        b2Body* cold=world->bodies.data+many[i].index1-1;
        ENSURE(world->solverSets.data[b2_awakeSet].bodySims.data[cold->localIndex].bodyId==cold->id);
    }
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int IslandWakeTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);
    b2World* world=b2_worlds+id.index1-1;b2BodyDef bd=b2DefaultBodyDef();bd.position={0,-1};
    b2BodyId ground=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Polygon floor=b2MakeBox(20,0.5f);
    b2ShapeId support=b2CreatePolygonShape(ground,&sd,&floor);
    bd.type=b2_dynamicBody;bd.position={-3,0};b2Circle c={{0,0},0.5f};
    b2BodyId a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.position={3,0};b2BodyId b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60.0f,4); }
    ENSURE(!b2Body_IsAwake(a) && !b2Body_IsAwake(b));
    ENSURE(world->bodies.data[a.index1-1].islandId!=world->bodies.data[b.index1-1].islandId);
    world->locked=true;b2Body_WakeTouching(ground);world->locked=false;
    ENSURE(!b2Body_IsAwake(a) && !b2Body_IsAwake(b));
    b2Body_WakeTouching(ground);ENSURE(b2Body_IsAwake(a) && b2Body_IsAwake(b) && !b2Body_IsAwake(ground));
    b2Body_SetAwake(a,false);b2Body_SetAwake(b,false);
    b2DestroyShape(support,false);ENSURE(b2Body_IsAwake(a) && b2Body_IsAwake(b));
    ENSURE(world->broadPhase.pairSet.count==0);
    b2World_Step(id,0.1f,4);ENSURE(b2Body_GetLinearVelocity(a).y<0 && b2Body_GetLinearVelocity(b).y<0);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    wd.gravity=b2Vec2_zero;id=b2CreateWorld(&wd);world=b2_worlds+id.index1-1;bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);bd.position={1,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);b2Body_SetAwake(a,false);
    b2Body_ApplyLinearImpulseToCenter(a,{1,0},false);ENSURE(!b2Body_IsAwake(a) && !b2Body_IsAwake(b));
    b2Body_ApplyLinearImpulseToCenter(a,{1,0},true);ENSURE(b2Body_IsAwake(a) && b2Body_IsAwake(b));
    b2Contact* contact=world->contacts.data;ENSURE(contact->colorIndex!=B2_NULL_INDEX);
    b2ContactSim* sim=b2GetContactSim(world,contact);
    ENSURE(sim->encodedBodySimA==b2EncodeBodySimIndex(world->bodies.data+contact->edges[0].bodyId));
    ENSURE(sim->encodedBodySimB==b2EncodeBodySimIndex(world->bodies.data+contact->edges[1].bodyId));
    b2Body_SetAwake(a,false);b2DestroyBody(a);ENSURE(b2Body_IsAwake(b));
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

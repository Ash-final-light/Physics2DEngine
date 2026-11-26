// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "world.h"
#include "box2d/types.h"

int ContactLifecycleTest( void )
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId worldId=b2CreateWorld(&wd);
    b2World* world=b2_worlds+worldId.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2BodyId a=b2CreateBody(worldId,&bd);
    bd.type=b2_staticBody;bd.position={1.0f,0.0f};
    b2BodyId b=b2CreateBody(worldId,&bd);
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{0.0f,0.0f},1.0f};
    b2ShapeId sa=b2CreateCircleShape(a,&sd,&circle);
    b2ShapeId sb=b2CreateCircleShape(b,&sd,&circle);
    b2World_Step(worldId,0.01f,1);
    ENSURE(world->broadPhase.pairSet.count==1);
    ENSURE(world->bodies.data[a.index1-1].contactCount==1);
    b2Contact* contact=world->contacts.data;
    b2ContactId old={contact->contactId+1,world->worldId,0,contact->generation};
    ENSURE(b2Contact_IsValid(old));
    b2World_Step(worldId,0.01f,1);
    ENSURE(world->broadPhase.pairSet.count==1);
    b2Body_SetTransform(b,{20.0f,0.0f},b2Rot_identity);
    b2World_Step(worldId,0.01f,1);
    ENSURE(world->broadPhase.pairSet.count==0 && b2Contact_IsValid(old)==false);
    b2Body_SetTransform(b,{1.0f,0.0f},b2Rot_identity);
    b2World_Step(worldId,0.01f,1);
    ENSURE(world->broadPhase.pairSet.count==1 && b2Contact_IsValid(old)==false);
    b2Body_SetType(a,b2_kinematicBody);
    ENSURE(world->broadPhase.pairSet.count==0);
    b2World_Step(worldId,0.01f,1);
    ENSURE(world->broadPhase.pairSet.count==0);
    b2Body_SetType(a,b2_dynamicBody);
    b2World_Step(worldId,0.01f,1);
    ENSURE(world->broadPhase.pairSet.count==1);
    b2DestroyShape(sb,true);
    ENSURE(world->broadPhase.pairSet.count==0 && world->bodies.data[a.index1-1].contactCount==0);
    b2CreateCircleShape(b,&sd,&circle);
    b2World_Step(worldId,0.01f,1);
    b2Filter filter=b2DefaultFilter();filter.maskBits=0;
    b2Shape_SetFilter(sa,filter);
    ENSURE(world->broadPhase.pairSet.count==0);
    b2World_Step(worldId,0.01f,1);
    ENSURE(world->broadPhase.pairSet.count==0);
    b2Shape_SetFilter(sa,b2DefaultFilter());
    b2World_Step(worldId,0.01f,1);
    ENSURE(world->broadPhase.pairSet.count==1);
    b2DestroyBody(a);
    ENSURE(world->broadPhase.pairSet.count==0 && world->bodies.data[b.index1-1].contactCount==0);
    b2DestroyWorld(worldId);ENSURE(b2GetByteCount()==before);return 0;
}


int ContactManifoldUpdateTest( void )
{
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId worldId=b2CreateWorld(&wd);
    b2World* world=b2_worlds+worldId.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableContactRecycling=false;
    bd.position={100.0f,200.0f};bd.rotation={0.0f,1.0f};
    b2BodyId a=b2CreateBody(worldId,&bd);
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{1.0f,0.0f},1.0f};
    b2CreateCircleShape(a,&sd,&circle);
    bd.type=b2_staticBody;bd.position={100.0f,202.5f};bd.rotation=b2Rot_identity;
    b2BodyId b=b2CreateBody(worldId,&bd);
    circle.center={0.0f,0.0f};b2CreateCircleShape(b,&sd,&circle);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    b2Contact* contact=world->contacts.data;
    b2ContactSim* sim=b2GetContactSim(world,contact);
    ENSURE(sim->manifold.pointCount==1 && (contact->flags&b2_contactTouchingFlag)!=0);
    ENSURE_SMALL(sim->manifold.normal.x,0.00001f);
    ENSURE_SMALL(sim->manifold.normal.y-1.0f,0.00001f);
    ENSURE_SMALL(sim->manifold.points[0].separation+0.5f,0.00001f);
    ENSURE_SMALL(sim->manifold.points[0].anchorA.y-0.75f,0.00001f);
    ENSURE_SMALL(sim->manifold.points[0].anchorB.y+0.75f,0.00001f);
    b2Body_SetTransform(b,{100.0f,203.03f},b2Rot_identity);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    sim=b2GetContactSim(world,contact);
    ENSURE(world->broadPhase.pairSet.count==1);
    ENSURE(sim->manifold.pointCount==0 && (contact->flags&b2_contactTouchingFlag)==0);
    b2Body_SetTransform(b,{100.0f,202.5f},b2Rot_identity);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    sim=b2GetContactSim(world,contact);
    ENSURE(sim->manifold.pointCount==1);
    b2DestroyWorld(worldId);return 0;
}


int ContactPersistenceTest( void )
{
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId worldId=b2CreateWorld(&wd);
    b2World* world=b2_worlds+worldId.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableContactRecycling=false;
    b2BodyId a=b2CreateBody(worldId,&bd);
    bd.type=b2_staticBody;bd.position={1.5f,0.0f};
    b2BodyId b=b2CreateBody(worldId,&bd);
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{0.0f,0.0f},1.0f};
    b2CreateCircleShape(a,&sd,&circle);b2CreateCircleShape(b,&sd,&circle);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    b2ContactSim* sim=b2GetContactSim(world,world->contacts.data);
    ENSURE(sim->manifold.pointCount==1 && sim->manifold.points[0].persisted==false);
    uint16_t id=sim->manifold.points[0].id;
    sim->manifold.points[0].normalImpulse=3.0f;
    sim->manifold.points[0].tangentImpulse=0.5f;
    sim->manifold.points[0].totalNormalImpulse=9.0f;
    sim->manifold.points[0].normalVelocity=-10.0f;
    sim->manifold.rollingImpulse=0.25f;
    b2UpdateContacts(world);
    sim=b2GetContactSim(world,world->contacts.data);
    ENSURE(sim->manifold.points[0].persisted && sim->manifold.points[0].id==id);
    ENSURE(sim->manifold.points[0].normalImpulse==3.0f && sim->manifold.points[0].tangentImpulse==0.5f);
    ENSURE(sim->manifold.rollingImpulse==0.25f);
    ENSURE(sim->manifold.points[0].totalNormalImpulse==0.0f && sim->manifold.points[0].normalVelocity==0.0f);
    b2Body_SetTransform(b,{2.03f,0.0f},b2Rot_identity);b2UpdateContacts(world);
    sim=b2GetContactSim(world,world->contacts.data);
    ENSURE(sim->manifold.pointCount==0);
    b2Body_SetTransform(b,{1.5f,0.0f},b2Rot_identity);b2UpdateContacts(world);
    sim=b2GetContactSim(world,world->contacts.data);
    ENSURE(sim->manifold.pointCount==1 && sim->manifold.points[0].persisted==false);
    ENSURE(sim->manifold.points[0].normalImpulse==0.0f && sim->manifold.rollingImpulse==0.0f);
    b2DestroyWorld(worldId);return 0;
}


int ContactDataQueryTest( void )
{
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId worldId=b2CreateWorld(&wd);
    b2World* world=b2_worlds+worldId.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableContactRecycling=false;
    b2BodyId body=b2CreateBody(worldId,&bd);
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{0.0f,2.0f},1.0f};
    b2ShapeId upper=b2CreateCircleShape(body,&sd,&circle);
    circle.center.y=-2.0f;b2CreateCircleShape(body,&sd,&circle);
    bd.type=b2_staticBody;bd.position={1.5f,2.0f};
    b2BodyId other=b2CreateBody(worldId,&bd);
    circle.center=b2Vec2_zero;b2CreateCircleShape(other,&sd,&circle);
    bd.position.y=-2.0f;b2CreateCircleShape(b2CreateBody(worldId,&bd),&sd,&circle);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    ENSURE(b2Body_GetContactCapacity(body)==2);
    b2ContactData data[3]={};data[2].contactId.index1=12345;
    ENSURE(b2Body_GetContactData(body,data,1)==1 && data[1].contactId.index1==0);
    ENSURE(b2Body_GetContactData(body,data,3)==2 && data[2].contactId.index1==12345);
    ENSURE(b2Contact_IsValid(data[0].contactId));
    b2ContactData single=b2Contact_GetData(data[0].contactId);
    ENSURE(single.shapeIdA.index1==data[0].shapeIdA.index1 && single.manifold.pointCount==1);
    ENSURE(b2Shape_GetContactCapacity(upper)==2);
    ENSURE(b2Shape_GetContactData(upper,data,3)==1);
    ENSURE(data[0].shapeIdA.index1==upper.index1 || data[0].shapeIdB.index1==upper.index1);
    ENSURE(b2Body_GetContactData(body,NULL,0)==0);
    b2Body_SetTransform(other,{2.03f,2.0f},b2Rot_identity);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    ENSURE(b2Body_GetContactCapacity(body)==2);
    ENSURE(b2Body_GetContactData(body,data,3)==1);
    ENSURE(b2Shape_GetContactData(upper,data,3)==0);
    b2DestroyWorld(worldId);return 0;
}

int ContactRecycleTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
    ENSURE(b2World_GetContactRecycleDistance(id)==B2_CONTACT_RECYCLE_DISTANCE);
    b2World_SetContactRecycleDistance(id,0.05f);
    b2World_SetContactRecycleDistance(id,NAN);ENSURE(b2World_GetContactRecycleDistance(id)==0.05f);
    world->locked=true;b2World_SetContactRecycleDistance(id,1);world->locked=false;
    ENSURE(b2World_GetContactRecycleDistance(id)==0.05f);
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2BodyId a=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};
    b2ShapeId sa=b2CreateCircleShape(a,&sd,&c);
    bd.type=b2_staticBody;bd.position={1.5f,0};b2BodyId b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    ENSURE(b2Body_IsContactRecyclingEnabled(a));b2Body_EnableContactRecycling(a,false);ENSURE(!b2Body_IsContactRecyclingEnabled(a));
    b2Body_EnableContactRecycling(a,true);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    b2Contact* contact=world->contacts.data;b2ContactSim* sim=b2GetContactSim(world,contact);
    ENSURE((sim->simFlags&b2_contactRecycleFlag)!=0);
    float cached=sim->cachedRelativePose.p.x;
    float oldAnchor=sim->manifold.points[0].anchorA.x;
    b2Body_SetTransform(b,{1.51f,0},b2Rot_identity);b2UpdateContacts(world);sim=b2GetContactSim(world,contact);
    ENSURE(sim->cachedRelativePose.p.x==cached && sim->manifold.points[0].anchorA.x==oldAnchor);
    ENSURE_SMALL(sim->manifold.points[0].separation+0.49f,0.00001f);ENSURE(sim->manifold.points[0].persisted);
    b2Body_SetTransform(b,{1.56f,0},b2Rot_identity);b2UpdateContacts(world);sim=b2GetContactSim(world,contact);
    ENSURE_SMALL(sim->cachedRelativePose.p.x-1.56f,0.00001f);
    ENSURE_SMALL(sim->manifold.points[0].separation+0.44f,0.00001f);
    b2Body_SetTransform(a,{0,0},b2MakeRot(0.5f));b2UpdateContacts(world);sim=b2GetContactSim(world,contact);
    ENSURE_SMALL(sim->cachedRotationA.s-b2MakeRot(0.5f).s,0.00001f);
    b2World_SetContactRecycleDistance(id,0);ENSURE(b2World_GetContactRecycleDistance(id)==0);
    b2Body_SetTransform(b,{1.561f,0},b2Rot_identity);b2UpdateContacts(world);sim=b2GetContactSim(world,contact);
    ENSURE_SMALL(b2Length(sim->cachedRelativePose.p)-1.561f,0.00001f);
    b2ContactId old={contact->contactId+1,id.index1-1,0,contact->generation};
    b2Filter filter=b2DefaultFilter();filter.maskBits=0;b2Shape_SetFilter(sa,filter);
    ENSURE(!b2Contact_IsValid(old));ENSURE(world->broadPhase.pairSet.count==0);
    b2Shape_SetFilter(sa,b2DefaultFilter());b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    contact=world->contacts.data;sim=b2GetContactSim(world,contact);ENSURE(sim->manifold.pointCount==1);
    c.radius=0.1f;b2Shape_SetCircle(sa,&c);ENSURE(world->broadPhase.pairSet.count==0);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // A previously non-touching pair must use the tighter threshold to discover collision.
    id=b2CreateWorld(&wd);world=b2_worlds+id.index1-1;bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;c.radius=1;
    a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.type=b2_staticBody;bd.position={2.04f,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    contact=world->contacts.data;ENSURE(b2GetContactSim(world,contact)->manifold.pointCount==0);
    b2Body_SetTransform(b,{2.01f,0},b2Rot_identity);b2UpdateContacts(world);
    ENSURE(b2GetContactSim(world,contact)->manifold.pointCount==1);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int ContactTouchEventsTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);
    b2World* world=b2_worlds+id.index1-1;b2World_SetContactRecycleDistance(id,0);
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;b2BodyId a=b2CreateBody(id,&bd);
    b2ShapeDef sd=b2DefaultShapeDef();ENSURE(!sd.enableContactEvents);sd.enableContactEvents=true;b2Circle c={{0,0},1};b2ShapeId sa=b2CreateCircleShape(a,&sd,&c);
    bd.type=b2_staticBody;bd.position={2,0};b2BodyId b=b2CreateBody(id,&bd);sd.enableContactEvents=false;b2ShapeId sb=b2CreateCircleShape(b,&sd,&c);
    b2World_Step(id,0.001f,1);b2ContactEvents events=b2World_GetContactEvents(id);ENSURE(events.beginCount==1 && events.endCount==0);
    b2ContactId contact=events.beginEvents[0].contactId;ENSURE(b2Contact_IsValid(contact));
    ENSURE(events.beginEvents[0].shapeIdA.index1==sa.index1 && events.beginEvents[0].shapeIdB.index1==sb.index1);
    world->locked=true;ENSURE(b2World_GetContactEvents(id).beginCount==0);world->locked=false;
    b2World_Step(id,0.001f,1);events=b2World_GetContactEvents(id);ENSURE(events.beginCount==0 && events.endCount==0);
    b2Body_SetTransform(b,{2.03f,0},b2Rot_identity);b2World_Step(id,0.001f,1);events=b2World_GetContactEvents(id);
    ENSURE(events.beginCount==0 && events.endCount==1 && b2Contact_IsValid(contact));ENSURE(events.endEvents[0].contactId.generation==contact.generation);
    b2World_Step(id,0.001f,1);events=b2World_GetContactEvents(id);ENSURE(events.beginCount==0 && events.endCount==0);
    b2Body_SetTransform(b,{2,0},b2Rot_identity);b2World_Step(id,0.001f,1);ENSURE(b2World_GetContactEvents(id).beginCount==1);
    b2DestroyShape(sa,true);ENSURE(!b2Contact_IsValid(contact));b2World_Step(id,0.001f,1);events=b2World_GetContactEvents(id);
    ENSURE(events.endCount==1 && events.beginCount==0);ENSURE(!b2Shape_IsValid(events.endEvents[0].shapeIdA));
    b2World_Step(id,0.001f,1);ENSURE(b2World_GetContactEvents(id).endCount==0);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // Enabling applies to shapes and new contacts; default contacts remain silent.
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;a=b2CreateBody(id,&bd);sd=b2DefaultShapeDef();sa=b2CreateCircleShape(a,&sd,&c);
    bd.type=b2_staticBody;bd.position={2,0};b=b2CreateBody(id,&bd);sb=b2CreateCircleShape(b,&sd,&c);
    b2World_Step(id,0.001f,1);ENSURE(b2World_GetContactEvents(id).beginCount==0);
    b2Body_EnableContactEvents(a,true);ENSURE(b2Shape_AreContactEventsEnabled(sa));
    b2Shape_EnableContactEvents(sb,true);ENSURE(b2Shape_AreContactEventsEnabled(sb));
    b2Filter filter=b2DefaultFilter();filter.maskBits=0;b2Shape_SetFilter(sa,filter);b2Shape_SetFilter(sa,b2DefaultFilter());
    b2World_Step(id,0.001f,1);ENSURE(b2World_GetContactEvents(id).beginCount==1);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

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

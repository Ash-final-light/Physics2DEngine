// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "test_macros.h"
#include "world.h"

int SensorOverlapEventsTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();b2BodyId fixed=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();
    sd.isSensor=true;sd.enableSensorEvents=true;b2Circle c={{0,0},2};b2ShapeId sensor=b2CreateCircleShape(fixed,&sd,&c);ENSURE(b2Shape_IsSensor(sensor));
    c.center={20,0};b2ShapeId secondSensor=b2CreateCircleShape(fixed,&sd,&c);c.center={0,0};
    bd.type=b2_dynamicBody;bd.position={-4,0};bd.linearVelocity={1,0};b2BodyId visitor=b2CreateBody(id,&bd);sd.isSensor=false;c.radius=0.5f;b2ShapeId shape=b2CreateCircleShape(visitor,&sd,&c);
    b2World_Step(id,0.1f,1);ENSURE(b2World_GetSensorEvents(id).beginCount==0);
    b2Body_SetTransform(visitor,{-1,0},b2Rot_identity);b2World_Step(id,0.1f,1);b2SensorEvents events=b2World_GetSensorEvents(id);
    ENSURE(events.beginCount==1 && events.endCount==0);ENSURE(events.beginEvents[0].sensorShapeId.index1==sensor.index1 && events.beginEvents[0].visitorShapeId.index1==shape.index1);
    ENSURE(world->broadPhase.pairSet.count==0 && b2Body_GetLinearVelocity(visitor).x==1);
    b2World_Step(id,0.1f,1);ENSURE(b2World_GetSensorEvents(id).beginCount==0);
    b2Body_SetTransform(visitor,{5,0},b2Rot_identity);b2World_Step(id,0.1f,1);ENSURE(b2World_GetSensorEvents(id).endCount==1);
    b2Body_SetTransform(visitor,{0,0},b2Rot_identity);b2World_Step(id,0.01f,1);ENSURE(b2World_GetSensorEvents(id).beginCount==1);
    b2DestroyShape(shape,true);b2ShapeId replacement=b2CreateCircleShape(visitor,&sd,&c);ENSURE(replacement.index1==shape.index1 && replacement.generation!=shape.generation);
    b2World_Step(id,0.01f,1);events=b2World_GetSensorEvents(id);ENSURE(events.beginCount==1 && events.endCount==1);
    ENSURE(!b2Shape_IsValid(events.endEvents[0].visitorShapeId) && b2Shape_IsValid(events.beginEvents[0].visitorShapeId));
    b2DestroyShape(sensor,true);ENSURE(world->sensors.count==1 && world->shapes.data[secondSensor.index1-1].sensorIndex==0);b2World_Step(id,0.01f,1);events=b2World_GetSensorEvents(id);ENSURE(events.endCount==1 && !b2Shape_IsValid(events.endEvents[0].sensorShapeId));
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // A fast visitor is sensed even when it passes completely through in one frame.
    id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();fixed=b2CreateBody(id,&bd);sd=b2DefaultShapeDef();sd.isSensor=true;sd.enableSensorEvents=true;
    b2Polygon region=b2MakeBox(0.1f,3);sensor=b2CreatePolygonShape(fixed,&sd,&region);
    bd.type=b2_dynamicBody;bd.position={-2,0};bd.linearVelocity={100,0};visitor=b2CreateBody(id,&bd);sd.isSensor=false;c.radius=0.1f;b2CreateCircleShape(visitor,&sd,&c);
    b2World_Step(id,0.05f,1);events=b2World_GetSensorEvents(id);ENSURE(events.beginCount==1);ENSURE_SMALL(b2Body_GetPosition(visitor).x-3,0.00001f);
    b2World_Step(id,0.01f,1);ENSURE(b2World_GetSensorEvents(id).endCount==1);
    b2Body_SetTransform(visitor,{-2,0},b2Rot_identity);b2Body_SetLinearVelocity(visitor,{100,0});
    bd.type=b2_staticBody;bd.position={-1,0};b2BodyId blocker=b2CreateBody(id,&bd);b2CreatePolygonShape(blocker,&sd,&region);
    b2World_Step(id,0.05f,1);ENSURE(b2World_GetSensorEvents(id).beginCount==0 && b2Body_GetPosition(visitor).x<-1);

    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // Both participants must opt in, and category filtering also applies to sensors.
    for(int run=0;run<3;++run)
    {
        id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();fixed=b2CreateBody(id,&bd);
        sd=b2DefaultShapeDef();sd.isSensor=true;sd.enableSensorEvents=run!=0;c.radius=2;
        if(run==2) { sd.filter.maskBits=0; }
        b2CreateCircleShape(fixed,&sd,&c);bd.type=b2_dynamicBody;visitor=b2CreateBody(id,&bd);
        sd=b2DefaultShapeDef();sd.enableSensorEvents=run!=1;c.radius=0.5f;b2CreateCircleShape(visitor,&sd,&c);
        b2World_Step(id,0.01f,1);ENSURE(b2World_GetSensorEvents(id).beginCount==0);
        b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    }
    return 0;
}

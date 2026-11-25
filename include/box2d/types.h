// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "collision.h"
#include "id.h"
#include "math_types.h"

typedef float b2FrictionCallbackFcn( float frictionA, uint64_t userMaterialIdA, float frictionB, uint64_t userMaterialIdB );
typedef float b2RestitutionCallbackFcn( float restitutionA, uint64_t userMaterialIdA, float restitutionB, uint64_t userMaterialIdB );

typedef struct b2Capacity
{
	int staticShapeCount;
	int dynamicShapeCount;
	int staticBodyCount;
	int dynamicBodyCount;
	int contactCount;
	int arenaByteCount;
} b2Capacity;

typedef struct b2WorldDef
{
	b2Vec2 gravity;
	float restitutionThreshold;
	int restitutionIterations;
	bool enableRestitutionPropagation;
	float hitEventThreshold;
	float contactHertz;
	float contactDampingRatio;
	float contactSpeed;
	float maximumLinearSpeed;
	b2FrictionCallbackFcn* frictionCallback;
	b2RestitutionCallbackFcn* restitutionCallback;
	bool enableSleep;
	bool enableContinuous;
	int workerCount;
	void* userData;
	b2Capacity capacity;
	int internalValue;
} b2WorldDef;

B2_API b2WorldDef b2DefaultWorldDef( void );
B2_API b2WorldId b2CreateWorld( const b2WorldDef* def );
B2_API void b2DestroyWorld( b2WorldId worldId );
B2_API bool b2World_IsValid( b2WorldId id );
B2_API void b2World_SetUserData( b2WorldId worldId, void* userData );
B2_API void* b2World_GetUserData( b2WorldId worldId );

typedef enum b2BodyType
{
	b2_staticBody = 0,
	b2_kinematicBody = 1,
	b2_dynamicBody = 2,
	b2_bodyTypeCount
} b2BodyType;

typedef struct b2MotionLocks
{
	bool linearX;
	bool linearY;
	bool angularZ;
} b2MotionLocks;

typedef struct b2BodyDef
{
	b2BodyType type;
	b2Pos position;
	b2Rot rotation;
	b2Vec2 linearVelocity;
	float angularVelocity;
	float linearDamping;
	float angularDamping;
	float gravityScale;
	float sleepThreshold;
	float safetyFactor;
	const char* name;
	void* userData;
	b2MotionLocks motionLocks;
	bool enableSleep;
	bool isAwake;
	bool isBullet;
	bool isEnabled;
	bool allowFastRotation;
	bool enableContactRecycling;
	int internalValue;
} b2BodyDef;

B2_API b2BodyDef b2DefaultBodyDef( void );
B2_API b2BodyId b2CreateBody( b2WorldId worldId, const b2BodyDef* def );
B2_API void b2DestroyBody( b2BodyId bodyId );
B2_API bool b2Body_IsValid( b2BodyId id );
B2_API void b2Body_SetName( b2BodyId bodyId, const char* name );
B2_API const char* b2Body_GetName( b2BodyId bodyId );
B2_API void b2Body_SetUserData( b2BodyId bodyId, void* userData );
B2_API void* b2Body_GetUserData( b2BodyId bodyId );

B2_API b2Pos b2Body_GetPosition( b2BodyId bodyId );
B2_API b2Rot b2Body_GetRotation( b2BodyId bodyId );
B2_API b2WorldTransform b2Body_GetTransform( b2BodyId bodyId );
B2_API b2Vec2 b2Body_GetLocalPoint( b2BodyId bodyId, b2Pos worldPoint );
B2_API b2Pos b2Body_GetWorldPoint( b2BodyId bodyId, b2Vec2 localPoint );
B2_API b2Vec2 b2Body_GetLocalVector( b2BodyId bodyId, b2Vec2 worldVector );
B2_API b2Vec2 b2Body_GetWorldVector( b2BodyId bodyId, b2Vec2 localVector );
B2_API void b2Body_SetTransform( b2BodyId bodyId, b2Pos position, b2Rot rotation );
B2_API b2AABB b2Body_ComputeBounds( b2BodyId bodyId );


typedef struct b2Filter
{
    uint64_t categoryBits;
    uint64_t maskBits;
    int groupIndex;
} b2Filter;

typedef enum b2ShapeType
{
    b2_circleShape,
    b2_capsuleShape,
    b2_segmentShape,
    b2_polygonShape,
    b2_chainSegmentShape,
    b2_shapeTypeCount
} b2ShapeType;

typedef struct b2SurfaceMaterial
{
    float friction;
    float restitution;
    float rollingResistance;
    float tangentSpeed;
    uint64_t userMaterialId;
    uint32_t customColor;
} b2SurfaceMaterial;

typedef struct b2ShapeDef
{
    void* userData;
    b2SurfaceMaterial material;
    float density;
    b2Filter filter;
    bool updateBodyMass;
    int internalValue;
} b2ShapeDef;

B2_API b2Filter b2DefaultFilter( void );
B2_API b2SurfaceMaterial b2DefaultSurfaceMaterial( void );
B2_API b2ShapeDef b2DefaultShapeDef( void );
B2_API b2ShapeId b2CreateCircleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle );
B2_API void b2DestroyShape( b2ShapeId shapeId, bool updateBodyMass );
B2_API bool b2Shape_IsValid( b2ShapeId shapeId );

B2_API b2ShapeId b2CreateCapsuleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule );

B2_API b2ShapeId b2CreatePolygonShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon );

B2_API b2ShapeId b2CreateSegmentShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment );

B2_API void b2Shape_SetUserData( b2ShapeId shapeId, void* userData );
B2_API void* b2Shape_GetUserData( b2ShapeId shapeId );

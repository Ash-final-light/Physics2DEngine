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
    bool invokeContactCreation;
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

B2_API b2Circle b2Shape_GetCircle( b2ShapeId shapeId );
B2_API void b2Shape_SetCircle( b2ShapeId shapeId, const b2Circle* circle );

B2_API b2Capsule b2Shape_GetCapsule( b2ShapeId shapeId );
B2_API void b2Shape_SetCapsule( b2ShapeId shapeId, const b2Capsule* capsule );

B2_API b2Polygon b2Shape_GetPolygon( b2ShapeId shapeId );
B2_API void b2Shape_SetPolygon( b2ShapeId shapeId, const b2Polygon* polygon );

B2_API b2Segment b2Shape_GetSegment( b2ShapeId shapeId );
B2_API void b2Shape_SetSegment( b2ShapeId shapeId, const b2Segment* segment );

B2_API b2ShapeType b2Shape_GetType( b2ShapeId shapeId );


B2_API b2MassData b2Body_GetMassData( b2BodyId bodyId );
B2_API float b2Body_GetMass( b2BodyId bodyId );
B2_API float b2Body_GetRotationalInertia( b2BodyId bodyId );
B2_API b2Vec2 b2Body_GetLocalCenter( b2BodyId bodyId );
B2_API b2Pos b2Body_GetWorldCenter( b2BodyId bodyId );
B2_API void b2Body_UpdateMassFromShapes( b2BodyId bodyId );
B2_API void b2Shape_SetDensity( b2ShapeId shapeId, float density, bool updateBodyMass );
B2_API float b2Shape_GetDensity( b2ShapeId shapeId );
B2_API b2MassData b2Shape_ComputeMassData( b2ShapeId shapeId );

B2_API void b2Body_SetMassData( b2BodyId bodyId, b2MassData massData );


typedef struct b2ChainDef
{
    void* userData;
    const b2Vec2* points;
    int pointCount;
    b2Vec2 ghost1;
    b2Vec2 ghost2;
    const b2SurfaceMaterial* materials;
    int materialCount;
    b2Filter filter;
    bool isLoop;
    int internalValue;
} b2ChainDef;

B2_API b2ChainDef b2DefaultChainDef( void );
B2_API b2ChainId b2CreateChain( b2BodyId bodyId, const b2ChainDef* def );
B2_API void b2DestroyChain( b2ChainId chainId );
B2_API bool b2Chain_IsValid( b2ChainId chainId );


B2_API b2WorldId b2Chain_GetWorld( b2ChainId chainId );
B2_API int b2Chain_GetSegmentCount( b2ChainId chainId );
B2_API int b2Chain_GetSegments( b2ChainId chainId, b2ShapeId* segmentArray, int capacity );
B2_API b2ChainSegment b2Shape_GetChainSegment( b2ShapeId shapeId );
B2_API b2ChainId b2Shape_GetParentChain( b2ShapeId shapeId );


B2_API void b2Chain_SetSurfaceMaterial( b2ChainId chainId, const b2SurfaceMaterial* material, int segmentIndex );
B2_API void b2Chain_SetAllSurfaceMaterials( b2ChainId chainId, const b2SurfaceMaterial* material );
B2_API b2SurfaceMaterial b2Chain_GetSurfaceMaterial( b2ChainId chainId, int segmentIndex );


B2_API b2WorldId b2Body_GetWorld( b2BodyId bodyId );
B2_API int b2Body_GetShapeCount( b2BodyId bodyId );
B2_API int b2Body_GetShapes( b2BodyId bodyId, b2ShapeId* shapeArray, int capacity );
B2_API b2BodyId b2Shape_GetBody( b2ShapeId shapeId );
B2_API b2WorldId b2Shape_GetWorld( b2ShapeId shapeId );

B2_API b2Filter b2Shape_GetFilter( b2ShapeId shapeId );
B2_API void b2Shape_SetFilter( b2ShapeId shapeId, b2Filter filter );

B2_API void b2World_Step( b2WorldId worldId, float timeStep, int subStepCount );


B2_API b2Vec2 b2Body_GetLinearVelocity( b2BodyId bodyId );
B2_API float b2Body_GetAngularVelocity( b2BodyId bodyId );
B2_API void b2Body_SetLinearVelocity( b2BodyId bodyId, b2Vec2 linearVelocity );
B2_API void b2Body_SetAngularVelocity( b2BodyId bodyId, float angularVelocity );
B2_API b2Vec2 b2Body_GetLocalPointVelocity( b2BodyId bodyId, b2Vec2 localPoint );
B2_API b2Vec2 b2Body_GetWorldPointVelocity( b2BodyId bodyId, b2Pos worldPoint );

B2_API void b2Body_SetGravityScale( b2BodyId bodyId, float gravityScale );
B2_API float b2Body_GetGravityScale( b2BodyId bodyId );


B2_API void b2Body_SetLinearDamping( b2BodyId bodyId, float linearDamping );
B2_API float b2Body_GetLinearDamping( b2BodyId bodyId );
B2_API void b2Body_SetAngularDamping( b2BodyId bodyId, float angularDamping );
B2_API float b2Body_GetAngularDamping( b2BodyId bodyId );


B2_API void b2Body_ApplyForce( b2BodyId bodyId, b2Vec2 force, b2Pos point, bool wake );
B2_API void b2Body_ApplyForceToCenter( b2BodyId bodyId, b2Vec2 force, bool wake );
B2_API void b2Body_ClearForces( b2BodyId bodyId );

B2_API void b2Body_ApplyTorque( b2BodyId bodyId, float torque, bool wake );


B2_API void b2Body_ApplyLinearImpulse( b2BodyId bodyId, b2Vec2 impulse, b2Pos point, bool wake );
B2_API void b2Body_ApplyLinearImpulseToCenter( b2BodyId bodyId, b2Vec2 impulse, bool wake );

B2_API void b2Body_ApplyAngularImpulse( b2BodyId bodyId, float impulse, bool wake );

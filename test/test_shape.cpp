// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "box2d/types.h"
#include "box2d/math_functions.h"
#include "box2d/constants.h"
#include <math.h>

int CircleShapeLifecycleTest( void )
{
    size_t before = b2GetByteCount();
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef();
    bd.position = { 10.0f, 20.0f };
    b2BodyId body = b2CreateBody( world, &bd );
    b2ShapeDef sd = b2DefaultShapeDef();
    ENSURE( sd.density == 1.0f && sd.material.friction == 0.6f );
    b2Circle circle = { { 2.0f, 0.0f }, 1.0f };
    b2ShapeId first = b2CreateCircleShape( body, &sd, &circle );
    ENSURE( b2Shape_IsValid( first ) );
    b2AABB bounds = b2Body_ComputeBounds( body );
    ENSURE_SMALL( bounds.lowerBound.x - (11.0f - B2_SPECULATIVE_DISTANCE), 0.00001f );
    ENSURE_SMALL( bounds.upperBound.y - (21.0f + B2_SPECULATIVE_DISTANCE), 0.00001f );
    circle.center = { -4.0f, 0.0f };
    b2ShapeId second = b2CreateCircleShape( body, &sd, &circle );
    circle.center = { 0.0f, 5.0f };
    b2ShapeId third = b2CreateCircleShape( body, &sd, &circle );
    bounds = b2Body_ComputeBounds( body );
    ENSURE( bounds.lowerBound.x < 5.0f && bounds.upperBound.y > 26.0f );
    b2DestroyShape( second, true );
    ENSURE( b2Shape_IsValid( second ) == false );
    ENSURE( b2Shape_IsValid( first ) && b2Shape_IsValid( third ) );
    bounds = b2Body_ComputeBounds( body );
    ENSURE( bounds.lowerBound.x > 8.0f );
    b2ShapeId reused = b2CreateCircleShape( body, &sd, &circle );
    ENSURE( reused.index1 == second.index1 && reused.generation != second.generation );
    b2DestroyShape( second, true );
    ENSURE( b2Shape_IsValid( reused ) );
    b2Body_SetTransform( body, { 0.0f, 0.0f }, { 0.0f, 1.0f } );
    bounds = b2Body_ComputeBounds( body );
    ENSURE( bounds.lowerBound.x < -6.0f && bounds.upperBound.y > 3.0f );
    b2DestroyBody( body );
    ENSURE( b2Shape_IsValid( first ) == false );
    ENSURE( b2Shape_IsValid( third ) == false );
    ENSURE( b2Shape_IsValid( reused ) == false );
    b2BodyId replacement = b2CreateBody( world, &bd );
    bounds = b2Body_ComputeBounds( replacement );
    ENSURE( bounds.lowerBound.x == bd.position.x && bounds.upperBound.y == bd.position.y );
    circle.radius = -1.0f;
    ENSURE( b2CreateCircleShape( replacement, &sd, &circle ).index1 == 0 );
    circle.radius = NAN;
    ENSURE( b2CreateCircleShape( replacement, &sd, &circle ).index1 == 0 );
    circle.radius = 0.0f;
    sd.density = -1.0f;
    ENSURE( b2CreateCircleShape( replacement, &sd, &circle ).index1 == 0 );
    sd = b2DefaultShapeDef();
    b2ShapeId zero = b2CreateCircleShape( replacement, &sd, &circle );
    ENSURE( b2Shape_IsValid( zero ) );
    b2DestroyWorld( world );
    ENSURE( b2Shape_IsValid( zero ) == false );
    ENSURE( b2GetByteCount() == before );
    return 0;
}


int CapsuleShapeLifecycleTest( void )
{
    size_t before = b2GetByteCount();
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef();
    b2BodyId body = b2CreateBody( world, &bd );
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Capsule capsule = { { -2.0f, 0.0f }, { 2.0f, 0.0f }, 0.5f };
    b2ShapeId shape = b2CreateCapsuleShape( body, &sd, &capsule );
    ENSURE( b2Shape_IsValid( shape ) );
    b2AABB bounds = b2Body_ComputeBounds( body );
    ENSURE_SMALL( bounds.upperBound.x - (2.5f + B2_SPECULATIVE_DISTANCE), 0.00001f );
    b2Body_SetTransform( body, { 5.0f, 3.0f }, { 0.0f, 1.0f } );
    bounds = b2Body_ComputeBounds( body );
    ENSURE_SMALL( bounds.upperBound.y - (5.5f + B2_SPECULATIVE_DISTANCE), 0.00001f );
    b2DestroyShape( shape, true );
    ENSURE( b2Shape_IsValid( shape ) == false );
    capsule.center2 = capsule.center1;
    ENSURE( b2CreateCapsuleShape( body, &sd, &capsule ).index1 == 0 );
    capsule.center2 = { 2.0f, 0.0f };
    capsule.radius = -1.0f;
    ENSURE( b2CreateCapsuleShape( body, &sd, &capsule ).index1 == 0 );
    capsule.radius = 0.0f;
    ENSURE( b2Shape_IsValid( b2CreateCapsuleShape( body, &sd, &capsule ) ) );
    b2DestroyWorld( world );
    ENSURE( b2GetByteCount() == before );
    return 0;
}


int PolygonShapeLifecycleTest( void )
{
    size_t before = b2GetByteCount();
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef();
    b2BodyId body = b2CreateBody( world, &bd );
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Polygon box = b2MakeBox( 2.0f, 1.0f );
    b2ShapeId shape = b2CreatePolygonShape( body, &sd, &box );
    ENSURE( b2Shape_IsValid( shape ) );
    b2AABB bounds = b2Body_ComputeBounds( body );
    ENSURE_SMALL( bounds.upperBound.x - (2.0f + B2_SPECULATIVE_DISTANCE), 0.00001f );
    b2Body_SetTransform( body, { 5.0f, 3.0f }, { 0.0f, 1.0f } );
    bounds = b2Body_ComputeBounds( body );
    ENSURE_SMALL( bounds.upperBound.y - (5.0f + B2_SPECULATIVE_DISTANCE), 0.00001f );
    b2Polygon bad = box;
    bad.count = 0;
    ENSURE( b2CreatePolygonShape( body, &sd, &bad ).index1 == 0 );
    bad = box;
    bad.normals[0].x = NAN;
    ENSURE( b2CreatePolygonShape( body, &sd, &bad ).index1 == 0 );
    bad = box;
    bad.radius = -1.0f;
    ENSURE( b2CreatePolygonShape( body, &sd, &bad ).index1 == 0 );
    b2DestroyShape( shape, true );
    ENSURE( b2Shape_IsValid( shape ) == false );
    b2DestroyWorld( world );
    ENSURE( b2GetByteCount() == before );
    return 0;
}


int SegmentShapeLifecycleTest( void )
{
    size_t before = b2GetByteCount();
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef();
    b2BodyId body = b2CreateBody( world, &bd );
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Segment segment = { { -2.0f, 0.0f }, { 3.0f, 0.0f } };
    b2ShapeId shape = b2CreateSegmentShape( body, &sd, &segment );
    ENSURE( b2Shape_IsValid( shape ) );
    b2AABB bounds = b2Body_ComputeBounds( body );
    ENSURE_SMALL( bounds.upperBound.x - (3.0f + B2_SPECULATIVE_DISTANCE), 0.00001f );
    b2Body_SetTransform( body, { 5.0f, 3.0f }, { 0.0f, 1.0f } );
    bounds = b2Body_ComputeBounds( body );
    ENSURE_SMALL( bounds.upperBound.y - (6.0f + B2_SPECULATIVE_DISTANCE), 0.00001f );
    segment.point2 = segment.point1;
    ENSURE( b2CreateSegmentShape( body, &sd, &segment ).index1 == 0 );
    segment.point2.x = INFINITY;
    ENSURE( b2CreateSegmentShape( body, &sd, &segment ).index1 == 0 );
    b2DestroyBody( body );
    ENSURE( b2Shape_IsValid( shape ) == false );
    b2DestroyWorld( world );
    ENSURE( b2GetByteCount() == before );
    return 0;
}


int ShapeUserDataTest( void )
{
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef();
    b2BodyId body = b2CreateBody( world, &bd );
    b2ShapeDef sd = b2DefaultShapeDef();
    int first = 1, second = 2;
    sd.userData = &first;
    b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
    b2ShapeId a = b2CreateCircleShape( body, &sd, &circle );
    b2ShapeId b = b2CreateCircleShape( body, &sd, &circle );
    ENSURE( b2Shape_GetUserData( a ) == &first );
    b2Shape_SetUserData( a, &second );
    ENSURE( b2Shape_GetUserData( a ) == &second );
    ENSURE( b2Shape_GetUserData( b ) == &first );
    b2Shape_SetUserData( a, NULL );
    ENSURE( b2Shape_GetUserData( a ) == NULL );
    b2DestroyShape( a, false );
    b2ShapeId reused = b2CreateCircleShape( body, &sd, &circle );
    b2Shape_SetUserData( a, &second );
    ENSURE( b2Shape_GetUserData( a ) == NULL );
    ENSURE( b2Shape_GetUserData( reused ) == &first );
    b2DestroyWorld( world );
    return 0;
}


int ShapeGeometryEditTest( void )
{
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef();
    b2BodyId body = b2CreateBody( world, &bd );
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
    b2ShapeId shape = b2CreateCircleShape( body, &sd, &circle );
    ENSURE( b2Shape_GetType( shape ) == b2_circleShape );
    circle.center.x = 3.0f;
    circle.radius = 2.0f;
    b2Shape_SetCircle( shape, &circle );
    ENSURE( b2Shape_GetCircle( shape ).center.x == 3.0f );
    ENSURE( b2Body_ComputeBounds( body ).upperBound.x > 5.0f );
    b2Circle bad = circle;
    bad.radius = -1.0f;
    b2Shape_SetCircle( shape, &bad );
    ENSURE( b2Shape_GetCircle( shape ).radius == 2.0f );
    b2Polygon box = b2MakeBox( 2.0f, 3.0f );
    b2Shape_SetPolygon( shape, &box );
    ENSURE( b2Shape_GetType( shape ) == b2_polygonShape );
    ENSURE( b2Shape_GetPolygon( shape ).count == 4 );
    ENSURE( b2Body_ComputeBounds( body ).upperBound.y > 3.0f );
    b2Capsule capsule = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 0.5f };
    b2Shape_SetCapsule( shape, &capsule );
    ENSURE( b2Shape_GetType( shape ) == b2_capsuleShape );
    ENSURE( b2Shape_GetCapsule( shape ).radius == 0.5f );
    capsule.center2 = capsule.center1;
    b2Shape_SetCapsule( shape, &capsule );
    ENSURE( b2Shape_GetCapsule( shape ).center2.x == 1.0f );
    b2Segment segment = { { -5.0f, 0.0f }, { 5.0f, 0.0f } };
    b2Shape_SetSegment( shape, &segment );
    ENSURE( b2Shape_GetType( shape ) == b2_segmentShape );
    ENSURE( b2Shape_GetSegment( shape ).point2.x == 5.0f );
    b2Body_SetTransform( body, { 10.0f, 0.0f }, { 0.0f, 1.0f } );
    b2AABB bounds = b2Body_ComputeBounds( body );
    ENSURE( bounds.lowerBound.y < -5.0f && bounds.upperBound.y > 5.0f );
    ENSURE( b2Shape_IsValid( shape ) );
    b2DestroyWorld( world );
    return 0;
}


int BodyMassFromDensityTest( void )
{
    size_t before = b2GetByteCount();
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    bd.position = { 10.0f, 20.0f };
    b2BodyId body = b2CreateBody( world, &bd );
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Circle circle = { { -2.0f, 0.0f }, 1.0f };
    b2ShapeId left = b2CreateCircleShape( body, &sd, &circle );
    ENSURE_SMALL( b2Body_GetMass( body ) - B2_PI, 0.0001f );
    ENSURE( b2Body_GetLocalCenter( body ).x == -2.0f );
    ENSURE( b2Body_GetWorldCenter( body ).x == 8.0f );
    circle.center.x = 2.0f;
    b2ShapeId right = b2CreateCircleShape( body, &sd, &circle );
    b2MassData data = b2Body_GetMassData( body );
    ENSURE_SMALL( data.mass - 2.0f * B2_PI, 0.0001f );
    ENSURE_SMALL( data.rotationalInertia - 9.0f * B2_PI, 0.0001f );
    ENSURE_SMALL( data.center.x, 0.00001f );
    b2Shape_SetDensity( left, 0.0f, true );
    ENSURE( b2Shape_GetDensity( left ) == 0.0f );
    ENSURE_SMALL( b2Body_GetMass( body ) - B2_PI, 0.0001f );
    ENSURE( b2Body_GetLocalCenter( body ).x == 2.0f );
    ENSURE_SMALL( b2Body_GetRotationalInertia( body ) - 0.5f * B2_PI, 0.0001f );
    b2Shape_SetDensity( right, -1.0f, true );
    ENSURE( b2Shape_GetDensity( right ) == 1.0f );
    b2Shape_SetDensity( right, 2.0f, false );
    ENSURE_SMALL( b2Body_GetMass( body ) - B2_PI, 0.0001f );
    b2Body_UpdateMassFromShapes( body );
    ENSURE_SMALL( b2Body_GetMass( body ) - 2.0f * B2_PI, 0.0001f );
    b2Body_SetTransform( body, { 1.0f, 2.0f }, { 0.0f, 1.0f } );
    ENSURE( b2Body_GetWorldCenter( body ).x == 1.0f && b2Body_GetWorldCenter( body ).y == 4.0f );
    b2DestroyShape( right, true );
    ENSURE( b2Body_GetMass( body ) == 0.0f );
    bd.type = b2_staticBody;
    b2BodyId fixed = b2CreateBody( world, &bd );
    b2ShapeId shape = b2CreateCircleShape( fixed, &sd, &circle );
    ENSURE( b2Body_GetMass( fixed ) == 0.0f );
    ENSURE_SMALL( b2Shape_ComputeMassData( shape ).mass - B2_PI, 0.0001f );
    bd.type = b2_dynamicBody;
    bd.motionLocks.angularZ = true;
    b2BodyId locked = b2CreateBody( world, &bd );
    b2CreateCircleShape( locked, &sd, &circle );
    ENSURE( b2Body_GetRotationalInertia( locked ) == 0.0f );
    b2DestroyWorld( world );
    ENSURE( b2GetByteCount() == before );
    return 0;
}

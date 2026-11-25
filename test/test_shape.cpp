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

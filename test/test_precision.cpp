// SPDX-License-Identifier: MIT
#include "test_macros.h"
#include "box2d/box2d.h"
#include <vector>
#include <cmath>

int PrecisionConfigurationTest( void )
{
#if defined( BOX2D_DOUBLE_PRECISION )
    static_assert( sizeof( b2Pos ) == 2 * sizeof( double ) ); ENSURE( b2IsDoublePrecision() );
    double base = 1.0e9;
#else
    static_assert( sizeof( b2Pos ) == sizeof( b2Vec2 ) ); ENSURE( !b2IsDoublePrecision() );
    float base = 1000.0f;
#endif
    int before = b2GetByteCount(); b2WorldDef wd = b2DefaultWorldDef(); wd.gravity = { 0, 0 };
    b2WorldId world = b2CreateWorld( &wd ); b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody; bd.position = { base + 0.25f, base + 0.5f }; bd.linearVelocity = { 0.125f, 0 };
    b2BodyId body = b2CreateBody( world, &bd ); b2ShapeDef sd = b2DefaultShapeDef(); b2Circle circle = { { 0, 0 }, 0.5f };
    b2ShapeId shape = b2CreateCircleShape( body, &sd, &circle ); ENSURE( b2Shape_IsValid( shape ) );
    ENSURE( b2Body_GetPosition( body ).x == base + 0.25 );
    b2Pos point = b2Body_GetWorldPoint( body, ( b2Vec2{ 0.125f, 0.25f } ) );
    ENSURE( point.x == base + 0.375 && point.y == base + 0.75 );
    b2Vec2 local = b2Body_GetLocalPoint( body, point ); ENSURE( local.x == 0.125f && local.y == 0.25f );
    b2AABB bounds = b2Shape_GetBounds( shape );
    ENSURE( (double)bounds.lowerBound.x <= base - 0.25 && (double)bounds.upperBound.x >= base + 0.75 );
    b2Recording* rec = b2CreateRecording( 1 ); b2World_StartRecording( world, rec );
    for ( int i = 0; i < 8; ++i ) b2World_Step( world, 0.125f, 1 );
    b2World_StopRecording( world ); ENSURE( b2Body_GetPosition( body ).x > base + 0.25 );
    int size = b2World_GetSnapshot( world, nullptr, 0 ); std::vector<unsigned char> bytes( size );
    ENSURE( b2World_GetSnapshot( world, bytes.data(), size ) == size ); uint64_t hash = b2World_GetStateHash( world );
    ENSURE( b2World_RestoreFromSnapshot( world, bytes.data(), size ) && b2World_GetStateHash( world ) == hash );
    b2DestroyWorld( world ); ENSURE( b2ValidateRecording( b2Recording_GetData( rec ), b2Recording_GetSize( rec ), 4 ) );
    b2DestroyRecording( rec ); ENSURE( b2GetByteCount() == before ); return 0;
}

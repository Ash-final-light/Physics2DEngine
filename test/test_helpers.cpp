// SPDX-License-Identifier: MIT
#include "test_macros.h"
#include "utils.h"
#include "box2d/box2d.h"


int SharedUtilitiesTest( void )
{
    uint32_t saved = g_randomSeed; g_randomSeed = RAND_SEED;
    const int expected[] = { 29818, 2479, 4000, 16874 };
    for ( int value : expected ) ENSURE( RandomInt() == value );
    ENSURE( GetNumberOfCores() >= 1 );
    for ( int i = 0; i < 100; ++i ) {
        int k = RandomIntRange( -3, 7 ); ENSURE( k >= -3 && k <= 7 );
        float f = RandomFloat(); ENSURE( f >= -1 && f <= 1 );
        f = RandomFloatRange( 2, 5 ); ENSURE( f >= 2 && f <= 5 );
        b2Vec2 v = RandomVec2( -4, 4 ); ENSURE( b2IsValidVec2( v ) && v.x >= -4 && v.x <= 4 && v.y >= -4 && v.y <= 4 );
        b2Pos p = RandomPos( -8, 8 ); ENSURE( b2IsValidPos( p ) && p.x >= -8 && p.x <= 8 && p.y >= -8 && p.y <= 8 );
        ENSURE( b2IsValidRot( RandomRot() ) ); b2Polygon poly = RandomPolygon( 2 );
        ENSURE( poly.count >= 3 && poly.count <= B2_MAX_POLYGON_VERTICES );
        ENSURE( b2ComputePolygonMass( &poly, 1 ).mass > 0 );
    }
    g_randomSeed = saved; return 0;
}

int AwakeBodyCountTest( void )
{
    int before = b2GetByteCount(); b2WorldDef wd = b2DefaultWorldDef(); wd.gravity = { 0, 0 };
    b2WorldId world = b2CreateWorld( &wd ); ENSURE( b2World_GetAwakeBodyCount( world ) == 0 );
    b2BodyDef bd = b2DefaultBodyDef(); b2CreateBody( world, &bd );
    ENSURE( b2World_GetAwakeBodyCount( world ) == 0 );
    bd.type = b2_dynamicBody; b2BodyId a = b2CreateBody( world, &bd );
    bd.isAwake = false; b2BodyId b = b2CreateBody( world, &bd );
    ENSURE( b2World_GetAwakeBodyCount( world ) == 1 );
    b2Body_SetAwake( b, true ); ENSURE( b2World_GetAwakeBodyCount( world ) == 2 );
    b2Body_SetAwake( a, false ); ENSURE( b2World_GetAwakeBodyCount( world ) == 1 );
    b2Body_Disable( b ); ENSURE( b2World_GetAwakeBodyCount( world ) == 0 );
    b2Body_Enable( b ); ENSURE( b2World_GetAwakeBodyCount( world ) == 1 );
    b2DestroyBody( b ); ENSURE( b2World_GetAwakeBodyCount( world ) == 0 );
    b2DestroyWorld( world ); ENSURE( b2GetByteCount() == before ); return 0;
}

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

#include "determinism.h"
int SharedDeterminismTest( void )
{
    const uint8_t text[] = { 'a', 'b', 'c' };
    ENSURE( b2Hash( B2_HASH_INIT, text, 3 ) == 193485963u );
    ENSURE( b2Hash( b2Hash( B2_HASH_INIT, text, 1 ), text + 1, 2 ) == 193485963u );
    int before = b2GetByteCount(); int workers[] = { 1, 4 };
    for ( int worker : workers ) {
        b2WorldDef wd = b2DefaultWorldDef(); wd.workerCount = worker; b2WorldId world = b2CreateWorld( &wd );
        FallingHingeData data = CreateFallingHinges( world );
        ENSURE( data.bodyCount == 80 && b2World_GetCounters( world ).bodyCount == 81 && b2World_GetCounters( world ).jointCount == 40 );
        for ( int i = 0; i < 1000; ++i ) {
            b2World_Step( world, 1.0f / 60.0f, 4 ); if ( UpdateFallingHinges( world, &data ) ) break;
        }
        printf( "Falling hinges workers=%d sleepStep=%d hash=0x%08X\n", worker, data.sleepStep, data.hash );
        b2DestroyWorld( world ); int sleepStep = data.sleepStep; uint32_t hash = data.hash;
        DestroyFallingHinges( &data ); ENSURE( data.bodyIds == nullptr );
        ENSURE( b2GetByteCount() == before ); ENSURE( sleepStep == EXPECTED_SLEEP_STEP ); ENSURE( hash == EXPECTED_HASH );
    }
    return 0;
}

#include "human.h"
int SharedHumanTest( void )
{
    int before = b2GetByteCount(); uint32_t seed = g_randomSeed;
    b2WorldDef wd = b2DefaultWorldDef(); b2WorldId world = b2CreateWorld( &wd ); Human human = {};
    int tag = 42; CreateHuman( &human, world, ( b2Pos{ 0, 10 } ), 1, 0.5f, 2, 0.7f, 1, &tag, true );
    ENSURE( human.isSpawned && b2World_GetCounters( world ).bodyCount == bone_count );
    ENSURE( b2World_GetCounters( world ).jointCount == bone_count - 1 );
    for ( int i = 0; i < bone_count; ++i ) {
        ENSURE( b2Body_IsValid( human.bones[i].bodyId ) && b2Body_GetUserData( human.bones[i].bodyId ) == &tag );
        if ( i > 0 ) ENSURE( b2Joint_IsValid( human.bones[i].jointId ) );
    }
    Human_SetVelocity( &human, ( b2Vec2{ 2, 3 } ) );
    for ( const Bone& bone : human.bones ) ENSURE( b2Body_GetLinearVelocity( bone.bodyId ).x == 2 );
    Human_ApplyRandomAngularImpulse( &human, 1 );
    ENSURE( b2Body_GetAngularVelocity( human.bones[bone_torso].bodyId ) != 0 );
    Human_SetJointFrictionTorque( &human, 0 ); Human_SetJointSpringHertz( &human, 0 );
    for ( int i = 1; i < bone_count; ++i ) {
        ENSURE( !b2RevoluteJoint_IsMotorEnabled( human.bones[i].jointId ) );
        ENSURE( !b2RevoluteJoint_IsSpringEnabled( human.bones[i].jointId ) );
    }
    Human_SetJointFrictionTorque( &human, 2 ); Human_SetJointSpringHertz( &human, 3 ); Human_SetJointDampingRatio( &human, 0.5f );
    for ( int i = 1; i < bone_count; ++i ) {
        ENSURE( b2RevoluteJoint_IsMotorEnabled( human.bones[i].jointId ) );
        ENSURE( b2RevoluteJoint_GetSpringHertz( human.bones[i].jointId ) == 3 );
        ENSURE( b2RevoluteJoint_GetSpringDampingRatio( human.bones[i].jointId ) == 0.5f );
    }
    Human_EnableSensorEvents( &human, true ); b2ShapeId torsoShape;
    ENSURE( b2Body_GetShapes( human.bones[bone_torso].bodyId, &torsoShape, 1 ) == 1 && b2Shape_AreSensorEventsEnabled( torsoShape ) );
    Human_EnableSensorEvents( &human, false ); ENSURE( !b2Shape_AreSensorEventsEnabled( torsoShape ) );
    float mass = b2Body_GetMass( human.bones[bone_torso].bodyId );
    Human_SetScale( &human, 1.5f ); ENSURE( human.scale == 1.5f && b2Body_GetMass( human.bones[bone_torso].bodyId ) > mass );
    for ( int i = 0; i < 5; ++i ) b2World_Step( world, 1.0f / 60.0f, 4 );
    DestroyHuman( &human ); ENSURE( !human.isSpawned && b2World_GetCounters( world ).bodyCount == 0 && b2World_GetCounters( world ).jointCount == 0 );
    for ( const Bone& bone : human.bones ) ENSURE( bone.bodyId.index1 == 0 && bone.jointId.index1 == 0 );
    b2DestroyWorld( world ); g_randomSeed = seed; ENSURE( b2GetByteCount() == before ); return 0;
}

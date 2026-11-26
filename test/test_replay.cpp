// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "replay.h"
#include "state_hash.h"
#include "world.h"
#include <string.h>
#include <vector>

int ReplaySequentialTest( void )
{
	int before = b2GetByteCount();
	b2WorldDef wd = b2DefaultWorldDef(); b2WorldId world = b2CreateWorld( &wd );
	b2BodyDef bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody; bd.position = { 0, 10 }; bd.linearVelocity = { 1, 0 };
	b2BodyId body = b2CreateBody( world, &bd ); b2ShapeDef sd = b2DefaultShapeDef(); b2Circle circle = { { 0, 0 }, 1 };
	b2CreateCircleShape( body, &sd, &circle );
	b2Recording* rec = b2CreateRecording( 1 ); b2World_StartRecording( world, rec );
	std::vector<uint64_t> hashes;
	for ( int i = 0; i < 20; ++i )
	{
		b2World_Step( world, 1.0f / 60.0f, 4 );
		if ( i == 8 ) b2Body_SetLinearVelocity( body, ( b2Vec2{ -1, 2 } ) );
		hashes.push_back( b2World_GetStateHash( world ) );
	}
	b2World_StopRecording( world );
	std::vector<uint8_t> bytes( b2Recording_GetData( rec ), b2Recording_GetData( rec ) + b2Recording_GetSize( rec ) );
	b2DestroyRecording( rec ); b2DestroyWorld( world );
	b2Replay* player = b2CreateReplay( bytes.data(), (int)bytes.size(), 1 );
	ENSURE( player != nullptr );
	memset( bytes.data(), 0, bytes.size() );
	b2ReplayInfo info = b2Replay_GetInfo( player );
	ENSURE( info.frameCount == 20 && info.timeStep == 1.0f / 60.0f && info.subStepCount == 4 );
	for ( int i = 0; i < 20; ++i )
	{
		ENSURE( b2Replay_StepFrame( player ) && b2Replay_GetFrame( player ) == i + 1 );
		ENSURE( player->rdr.ok && !player->rdr.diverged );
		ENSURE( b2World_GetStateHash( b2Replay_GetWorldId( player ) ) == hashes[i] );
	}
	ENSURE( !b2Replay_StepFrame( player ) && b2Replay_IsAtEnd( player ) );
	b2WorldId playbackWorld = b2Replay_GetWorldId( player );
	ENSURE( b2World_IsValid( playbackWorld ) );
	b2DestroyReplay( player );
	ENSURE( b2GetByteCount() == before ); return 0;
}

static bool ReplayOverlap( b2ShapeId, void* context ) { *(int*)context += 1; return true; }
static float ReplayCast( b2ShapeId, b2Pos, b2Vec2, float, void* context ) { *(int*)context += 1; return 1; }

int ReplayMutationTest( void )
{
	int before = b2GetByteCount(); b2WorldDef wd = b2DefaultWorldDef();
	b2WorldId world = b2CreateWorld( &wd ); b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( world, rec );
	b2BodyDef bd = b2DefaultBodyDef(); b2BodyId a = b2CreateBody( world, &bd );
	bd.type = b2_dynamicBody; bd.position = { 0, 3 }; b2BodyId b = b2CreateBody( world, &bd );
	b2ShapeDef sd = b2DefaultShapeDef(); b2Circle circle = { { 0, 0 }, 0.5f };
	b2ShapeId shape = b2CreateCircleShape( b, &sd, &circle );
	b2Shape_SetFriction( shape, 0.7f ); b2Body_SetName( b, "replayed" );
	b2Vec2 points[] = { { -5, -2 }, { 5, -2 }, { 0, -4 } };
	b2SurfaceMaterial material = b2DefaultSurfaceMaterial(); b2ChainDef chainDef = b2DefaultChainDef();
	chainDef.points = points; chainDef.pointCount = 3; chainDef.materials = &material; chainDef.materialCount = 1; chainDef.isLoop = true;
	b2ChainId chain = b2CreateChain( a, &chainDef );
	b2JointId joints[9]; int j = 0;
#define MAKE_REPLAY_JOINT( Type ) \
	do { b2##Type##JointDef def = b2Default##Type##JointDef(); def.base.bodyIdA = a; def.base.bodyIdB = b; \
		joints[j++] = b2Create##Type##Joint( world, &def ); } while ( 0 )
	MAKE_REPLAY_JOINT( Distance ); MAKE_REPLAY_JOINT( Filter ); MAKE_REPLAY_JOINT( Motor );
	MAKE_REPLAY_JOINT( Mover ); MAKE_REPLAY_JOINT( Pogo ); MAKE_REPLAY_JOINT( Prismatic );
	MAKE_REPLAY_JOINT( Revolute ); MAKE_REPLAY_JOINT( Weld ); MAKE_REPLAY_JOINT( Wheel );
#undef MAKE_REPLAY_JOINT
	b2DistanceJoint_SetSpringHertz( joints[0], 3 );
	b2Joint_SetForceThreshold( joints[0], 100 );
	for ( int i = 0; i < 4; ++i )
	{
		b2World_Step( world, 1.0f / 60.0f, 4 );
		int hits = 0;
		b2World_OverlapAABB( world, b2Vec2_zero, ( b2AABB{ { -10, -10 }, { 10, 10 } } ), b2DefaultQueryFilter(), ReplayOverlap, &hits );
		b2World_CastRay( world, ( b2Pos{ -10, 3 } ), ( b2Vec2{ 20, 0 } ), b2DefaultQueryFilter(), ReplayCast, &hits );
	}
	b2DestroyJoint( joints[1] ); b2DestroyChain( chain );
	b2World_Step( world, 1.0f / 60.0f, 4 );
	uint64_t expected = b2World_GetStateHash( world );
	b2World_StopRecording( world );
	b2Replay* player = b2CreateReplay( b2Recording_GetData( rec ), b2Recording_GetSize( rec ), 1 );
	ENSURE( player != nullptr );
	int frames = 0; while ( b2Replay_StepFrame( player ) ) { frames += 1; ENSURE( frames <= 5 ); }
	ENSURE( frames == 5 && player->rdr.ok && !player->rdr.diverged );
	ENSURE( b2World_GetStateHash( b2Replay_GetWorldId( player ) ) == expected );
	b2DestroyReplay( player ); b2DestroyWorld( world ); b2DestroyRecording( rec );
	ENSURE( b2GetByteCount() == before ); return 0;
}

int ReplayCreateInvalidTest( void )
{
	int before = b2GetByteCount();
	ENSURE( b2CreateReplay( nullptr, 0, 1 ) == nullptr );
	b2WorldDef wd = b2DefaultWorldDef(); b2WorldId world = b2CreateWorld( &wd );
	b2Recording* rec = b2CreateRecording( 0 ); b2World_StartRecording( world, rec ); b2World_StopRecording( world );
	std::vector<uint8_t> good( b2Recording_GetData( rec ), b2Recording_GetData( rec ) + b2Recording_GetSize( rec ) );
	for ( int i = 0; i < 4; ++i )
	{
		auto bytes = good;
		if ( i == 0 ) bytes[0] = 0;
		if ( i == 1 ) bytes[4] += 1;
		if ( i == 2 ) bytes[17] = 1;
		if ( i == 3 ) bytes[32] = 0;
		int allocated = b2GetByteCount();
		ENSURE( b2CreateReplay( bytes.data(), (int)bytes.size(), 1 ) == nullptr );
		ENSURE( b2GetByteCount() == allocated );
	}
	b2DestroyRecording( rec ); b2DestroyWorld( world );
	ENSURE( b2GetByteCount() == before ); return 0;
}

static std::vector<uint8_t> MakeReplayFixture( int frames, std::vector<uint64_t>* hashes = nullptr )
{
    b2WorldDef wd = b2DefaultWorldDef(); b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody; bd.position = { 0, 10 };
    b2BodyId body = b2CreateBody( world, &bd ); b2ShapeDef sd = b2DefaultShapeDef(); b2Circle circle = { { 0, 0 }, 1 };
    b2CreateCircleShape( body, &sd, &circle );
    b2Recording* rec = b2CreateRecording( 1 ); b2World_StartRecording( world, rec );
    for ( int i = 0; i < frames; ++i ) {
        b2World_Step( world, 1.0f / 60.0f, 4 );
        if ( hashes ) hashes->push_back( b2World_GetStateHash( world ) );
    }
    b2World_StopRecording( world );
    std::vector<uint8_t> bytes( b2Recording_GetData( rec ), b2Recording_GetData( rec ) + b2Recording_GetSize( rec ) );
    b2DestroyRecording( rec ); b2DestroyWorld( world ); return bytes;
}

int ReplayRestartTest( void )
{
    int before = b2GetByteCount(); std::vector<uint64_t> hashes;
    auto bytes = MakeReplayFixture( 12, &hashes );
    b2Replay* player = b2CreateReplay( bytes.data(), (int)bytes.size(), 1 ); ENSURE( player != nullptr );
    b2WorldId id = b2Replay_GetWorldId( player );
    for ( int pass = 0; pass < 3; ++pass ) {
        int steps = pass == 0 ? 5 : 12;
        for ( int i = 0; i < steps; ++i ) {
            ENSURE( b2Replay_StepFrame( player ) ); ENSURE( b2World_GetStateHash( id ) == hashes[i] );
        }
        b2Replay_Restart( player );
        b2WorldId restored = b2Replay_GetWorldId( player );
        ENSURE( restored.index1 == id.index1 && restored.generation == id.generation );
        ENSURE( b2Replay_GetFrame( player ) == 0 && !b2Replay_IsAtEnd( player ) && player->rdr.ok );
    }
    b2DestroyReplay( player ); ENSURE( b2GetByteCount() == before ); return 0;
}

int ReplayLifetimeTest( void )
{
    int before = b2GetByteCount(); float original = b2GetLengthUnitsPerMeter();
    b2SetLengthUnitsPerMeter( 2 ); auto bytes = MakeReplayFixture( 3 ); b2SetLengthUnitsPerMeter( 3 );
    b2DestroyReplay( nullptr );
    for ( int i = 0; i < 10; ++i ) {
        b2Replay* player = b2CreateReplay( bytes.data(), (int)bytes.size(), 1 ); ENSURE( player != nullptr );
        ENSURE( b2GetLengthUnitsPerMeter() == 2 );
        ENSURE( b2Replay_StepFrame( player ) ); b2Replay_Restart( player );
        b2DestroyReplay( player );
        ENSURE( b2GetLengthUnitsPerMeter() == 3 && b2GetByteCount() == before );
    }
    b2SetLengthUnitsPerMeter( original ); return 0;
}

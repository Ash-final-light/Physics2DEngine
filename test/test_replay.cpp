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
		b2World_OverlapAABB( world, b2Pos_zero, ( b2AABB{ { -10, -10 }, { 10, 10 } } ), b2DefaultQueryFilter(), ReplayOverlap, &hits );
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

int ReplayDivergenceTest( void )
{
    int before = b2GetByteCount(); auto good = MakeReplayFixture( 5 );
    ENSURE( !b2Replay_HasDiverged( nullptr ) && b2Replay_GetDivergeFrame( nullptr ) == -1 );
    for ( int which = 0; which < 2; ++which ) {
        auto bytes = good; b2RecHeader header; memcpy( &header, bytes.data(), sizeof( header ) );
        int offset = 32 + (int)header.snapshotSize, seen = 0, target = which == 0 ? 1 : 3;
        bool changed = false;
        while ( offset + 4 <= (int)bytes.size() ) {
            int length = bytes[offset+1] | (int)bytes[offset+2] << 8 | (int)bytes[offset+3] << 16;
            if ( bytes[offset] == 0xF1 && ++seen == target ) { bytes[offset + 8] ^= 1; changed = true; break; }
            offset += 4 + length;
        }
        ENSURE( changed );
        b2Replay* player = b2CreateReplay( bytes.data(), (int)bytes.size(), 1 ); ENSURE( player != nullptr );
        int frames = 0; while ( b2Replay_StepFrame( player ) ) { frames++; }
        ENSURE( frames == 5 && player->rdr.ok && b2Replay_HasDiverged( player ) );
        ENSURE( b2Replay_GetDivergeFrame( player ) == ( which == 0 ? 0 : 2 ) );
        b2Replay_Restart( player );
        ENSURE( !b2Replay_HasDiverged( player ) && b2Replay_GetDivergeFrame( player ) == -1 );
        while ( b2Replay_StepFrame( player ) ) {}
        ENSURE( b2Replay_GetDivergeFrame( player ) == ( which == 0 ? 0 : 2 ) );
        b2DestroyReplay( player );
    }
    ENSURE( b2GetByteCount() == before ); return 0;
}

int ReplayExternalChangeTest( void )
{
    int before = b2GetByteCount(); auto bytes = MakeReplayFixture( 5 );
    b2Replay* player = b2CreateReplay( bytes.data(), (int)bytes.size(), 1 ); ENSURE( player != nullptr );
    ENSURE( b2Replay_StepFrame( player ) && !b2Replay_HasDiverged( player ) );
    b2WorldId id = b2Replay_GetWorldId( player ); b2World* world = b2_worlds + id.index1 - 1;
    b2BodyId body = { 1, (uint16_t)( id.index1 - 1 ), world->bodies.data[0].generation };
    b2Body_SetLinearVelocity( body, ( b2Vec2{ 10, 20 } ) );
    ENSURE( b2Replay_StepFrame( player ) && b2Replay_HasDiverged( player ) );
    ENSURE( b2Replay_GetDivergeFrame( player ) == 2 );
    while ( b2Replay_StepFrame( player ) ) {}
    ENSURE( b2Replay_GetFrame( player ) == 5 && b2Replay_GetDivergeFrame( player ) == 2 );
    b2DestroyReplay( player ); ENSURE( b2GetByteCount() == before ); return 0;
}

int ReplayKeyframeTest( void )
{
    int before = b2GetByteCount(); auto bytes = MakeReplayFixture( 80 );
    b2Replay* player = b2CreateReplay( bytes.data(), (int)bytes.size(), 1 ); ENSURE( player != nullptr );
    ENSURE( b2Replay_GetKeyframeMinInterval( player ) == 16 && b2Replay_GetKeyframeBytes( player ) == 0 );
    while ( b2Replay_StepFrame( player ) ) {}
    ENSURE( player->keyframeCount == 4 && player->lastKeyframeFrame == 64 );
    size_t sum = 0;
    for ( int i = 0; i < player->keyframeCount; ++i ) {
        b2RecKeyframe* frame = player->keyframes + i;
        ENSURE( frame->frame == ( i + 1 ) * 16 && frame->imageSize > 0 && frame->imageCapacity >= frame->imageSize );
        ENSURE( frame->cursor < player->size && player->data[frame->cursor] == 0x80 );
        sum += frame->imageCapacity + frame->bodyIdCount * sizeof( b2BodyId );
    }
    ENSURE( sum == b2Replay_GetKeyframeBytes( player ) );
    b2Replay_Restart( player ); while ( b2Replay_StepFrame( player ) ) {}
    ENSURE( player->keyframeCount == 4 && b2Replay_GetKeyframeBytes( player ) == sum );
    b2DestroyReplay( player ); ENSURE( b2GetByteCount() == before ); return 0;
}

int ReplayKeyframeBudgetTest( void )
{
    int before = b2GetByteCount(); auto bytes = MakeReplayFixture( 100 );
    b2Replay* player = b2CreateReplay( bytes.data(), (int)bytes.size(), 1 ); ENSURE( player != nullptr );
    b2Replay_SetKeyframePolicy( player, 1, 4 );
    while ( b2Replay_StepFrame( player ) ) {}
    ENSURE( player->keyframeCount == 1 && b2Replay_GetKeyframeInterval( player ) > 4 );
    ENSURE( b2Replay_GetKeyframeBudget( player ) == 1 && b2Replay_GetKeyframeBytes( player ) > 1 );
    // One snapshot may exceed the soft budget; the next capture evicts older snapshots.
    b2Replay_SetKeyframePolicy( player, 0, 0 );
    ENSURE( player->keyframeCount == 0 && b2Replay_GetKeyframeBytes( player ) == 0 );
    ENSURE( b2Replay_GetKeyframeBudget( player ) == 1 && b2Replay_GetKeyframeMinInterval( player ) == 4 );
    b2Replay_SetKeyframePolicy( player, 1024 * 1024, 8 ); b2Replay_Restart( player );
    while ( b2Replay_StepFrame( player ) ) {}
    ENSURE( player->keyframeCount > 1 && b2Replay_GetKeyframeInterval( player ) == 8 );
    b2DestroyReplay( player ); ENSURE( b2GetByteCount() == before ); return 0;
}

int ReplaySeekTest( void )
{
    int before = b2GetByteCount(); std::vector<uint64_t> hashes; auto bytes = MakeReplayFixture( 50, &hashes );
    int workers[] = { 1, 4 };
    for ( int worker : workers ) {
        b2Replay* player = b2CreateReplay( bytes.data(), (int)bytes.size(), worker ); ENSURE( player != nullptr );
        b2WorldId id = b2Replay_GetWorldId( player ); uint64_t initial = b2World_GetStateHash( id );
        b2Replay_SetKeyframePolicy( player, 1024 * 1024, 4 );
        while ( b2Replay_StepFrame( player ) ) {}
        int targets[] = { 0, 30, 5, 45, 16, 49, 1, 50, 100, -5, 27, 8, 50 };
        for ( int target : targets ) {
            b2Replay_SeekFrame( player, target ); int expected = target < 0 ? 0 : target > 50 ? 50 : target;
            ENSURE( b2Replay_GetFrame( player ) == expected && player->rdr.ok && !b2Replay_HasDiverged( player ) );
            b2WorldId restored = b2Replay_GetWorldId( player );
            ENSURE( restored.index1 == id.index1 && restored.generation == id.generation );
            ENSURE( b2World_GetStateHash( id ) == ( expected == 0 ? initial : hashes[expected - 1] ) );
        }
        b2DestroyReplay( player );
    }
    ENSURE( b2GetByteCount() == before ); return 0;
}

static std::vector<uint8_t> MakeMutableReplayFixture( std::vector<uint64_t>* hashes )
{
    b2WorldDef wd = b2DefaultWorldDef(); b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody; bd.position = { 0, 10 };
    b2ShapeDef sd = b2DefaultShapeDef(); b2Circle circle = { { 0, 0 }, 1 };
    b2CreateCircleShape( b2CreateBody( world, &bd ), &sd, &circle );
    b2Recording* rec = b2CreateRecording( 1 ); b2World_StartRecording( world, rec ); b2BodyId extra = {};
    for ( int i = 0; i < 50; ++i ) {
        b2World_Step( world, 1.0f / 60.0f, 4 );
        if ( i == 10 ) { bd.position = { 10, 10 }; extra = b2CreateBody( world, &bd ); b2CreateCircleShape( extra, &sd, &circle ); }
        if ( i == 30 ) b2DestroyBody( extra );
        hashes->push_back( b2World_GetStateHash( world ) );
    }
    b2World_StopRecording( world );
    std::vector<uint8_t> bytes( b2Recording_GetData( rec ), b2Recording_GetData( rec ) + b2Recording_GetSize( rec ) );
    b2DestroyWorld( world ); b2DestroyRecording( rec ); return bytes;
}

int ReplaySeekMutationTest( void )
{
    int before = b2GetByteCount(); std::vector<uint64_t> hashes; auto bytes = MakeMutableReplayFixture( &hashes );
    size_t budgets[] = { 1, 1024 * 1024 };
    for ( size_t budget : budgets ) {
        b2Replay* player = b2CreateReplay( bytes.data(), (int)bytes.size(), 1 ); ENSURE( player != nullptr );
        b2Replay_SetKeyframePolicy( player, budget, 4 ); while ( b2Replay_StepFrame( player ) ) {}
        int targets[] = { 8, 15, 35, 11, 30, 31, 50, 10, 20, 32 };
        for ( int target : targets ) {
            b2Replay_SeekFrame( player, target );
            ENSURE( player->rdr.ok && !b2Replay_HasDiverged( player ) && b2Replay_GetFrame( player ) == target );
            ENSURE( b2World_GetStateHash( b2Replay_GetWorldId( player ) ) == hashes[target - 1] );
        }
        b2DestroyReplay( player );
    }
    ENSURE( b2GetByteCount() == before ); return 0;
}

static bool ReplayPlane( b2ShapeId, const b2PlaneResult*, void* context ) { *(int*)context += 1; return true; }

static std::vector<uint8_t> MakeQueryReplayFixture( void )
{
    b2WorldDef wd = b2DefaultWorldDef(); b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef(); b2ShapeDef sd = b2DefaultShapeDef(); b2Circle circle = { { 0, 0 }, 0.75f };
    for ( int i = 0; i < 2; ++i ) { bd.position = { 3.0f * i, 0 }; b2CreateCircleShape( b2CreateBody( world, &bd ), &sd, &circle ); }
    b2Recording* rec = b2CreateRecording( 1 ); b2World_StartRecording( world, rec );
    for ( int i = 0; i < 3; ++i ) {
        b2World_Step( world, 1.0f / 60.0f, 4 ); int hits = 0; b2QueryFilter filter = b2DefaultQueryFilter();
        b2Vec2 center = {}; b2ShapeProxy proxy = b2MakeProxy( &center, 1, 5 );
        b2World_OverlapAABB( world, b2Pos_zero, ( b2AABB{ { -2, -2 }, { 5, 2 } } ), filter, ReplayOverlap, &hits );
        b2World_OverlapShape( world, b2Pos_zero, &proxy, filter, ReplayOverlap, &hits );
        b2World_CastRay( world, ( b2Pos{ -2, 0 } ), ( b2Vec2{ 7, 0 } ), filter, ReplayCast, &hits );
        proxy.radius = 0.25f;
        b2World_CastShape( world, ( b2Pos{ -2, 0 } ), &proxy, ( b2Vec2{ 7, 0 } ), filter, ReplayCast, &hits );
        b2Capsule mover = { { 0, -0.2f }, { 0, 0.2f }, 0.3f };
        b2World_CollideMover( world, ( b2Pos{ 0, 0.8f } ), &mover, filter, ReplayPlane, &hits );
        b2World_CastRayClosest( world, ( b2Pos{ -2, 0 } ), ( b2Vec2{ 7, 0 } ), filter );
        b2World_CastMover( world, ( b2Pos{ -2, 0 } ), &mover, ( b2Vec2{ 7, 0 } ), filter );
    }
    b2World_StopRecording( world );
    std::vector<uint8_t> bytes( b2Recording_GetData( rec ), b2Recording_GetData( rec ) + b2Recording_GetSize( rec ) );
    b2DestroyWorld( world ); b2DestroyRecording( rec ); return bytes;
}

int ReplayQueryInspectionTest( void )
{
    int before = b2GetByteCount(); auto bytes = MakeQueryReplayFixture();
    b2Replay* player = b2CreateReplay( bytes.data(), (int)bytes.size(), 1 ); ENSURE( player != nullptr );
    b2Replay_SetKeyframePolicy( player, 1024 * 1024, 1 );
    int expectedHits[] = { 2, 2, 2, 2, 1, 1, 0 };
    for ( int frame = 0; frame < 3; ++frame ) {
        ENSURE( b2Replay_StepFrame( player ) && !b2Replay_HasDiverged( player ) );
        ENSURE( b2Replay_GetFrameQueryCount( player ) == 7 );
        for ( int i = 0; i < 7; ++i ) {
            b2ReplayQueryInfo info = b2Replay_GetFrameQuery( player, i );
            ENSURE( (int)info.type == i && info.hitCount == expectedHits[i] );
            if ( info.hitCount > 0 ) {
                b2ReplayQueryHit hit = b2Replay_GetFrameQueryHit( player, i, 0 );
                ENSURE( b2Shape_IsValid( hit.shapeId ) );
                ENSURE( hit.shapeId.world0 == b2Replay_GetWorldId( player ).index1 - 1 );
            }
        }
    }
    b2Replay_SeekFrame( player, 1 ); ENSURE( b2Replay_GetFrameQueryCount( player ) == 7 );
    b2Replay_SeekFrame( player, 0 ); ENSURE( b2Replay_GetFrameQueryCount( player ) == 0 );
    ENSURE( b2Replay_GetFrameQuery( player, 99 ).hitCount == 0 );
    ENSURE( b2Replay_GetFrameQueryHit( player, 99, 0 ).shapeId.index1 == 0 );
    b2DestroyReplay( player ); ENSURE( b2GetByteCount() == before ); return 0;
}

struct ReplayDrawCounts { int polygons, lines, circles, capsules, points; b2Vec2 lower, upper; };
static void ReplayDrawPolygon( b2WorldTransform, const b2Vec2* vertices, int count, b2HexColor color, void* context )
{
    auto* counts = (ReplayDrawCounts*)context; counts->polygons++;
    if ( color == b2_colorMagenta && count == 4 ) { counts->lower = vertices[0]; counts->upper = vertices[2]; }
}
static void ReplayDrawLine( b2Pos, b2Pos, b2HexColor, void* context ) { ((ReplayDrawCounts*)context)->lines++; }
static void ReplayDrawCircle( b2Pos, float, b2HexColor, void* context ) { ((ReplayDrawCounts*)context)->circles++; }
static void ReplayDrawCapsule( b2Pos, b2Pos, float, b2HexColor, void* context ) { ((ReplayDrawCounts*)context)->capsules++; }
static void ReplayDrawPoint( b2Pos, float, b2HexColor, void* context ) { ((ReplayDrawCounts*)context)->points++; }

int ReplayQueryDrawTest( void )
{
    int before = b2GetByteCount(); auto bytes = MakeQueryReplayFixture();
    b2Replay* player = b2CreateReplay( bytes.data(), (int)bytes.size(), 1 ); ENSURE( player != nullptr );
    ENSURE( b2Replay_StepFrame( player ) ); ReplayDrawCounts counts = {}; b2DebugDraw draw = {};
    draw.context = &counts; draw.DrawPolygonFcn = ReplayDrawPolygon; draw.DrawLineFcn = ReplayDrawLine;
    draw.DrawCircleFcn = ReplayDrawCircle; draw.DrawSolidCapsuleFcn = ReplayDrawCapsule; draw.DrawPointFcn = ReplayDrawPoint;
    b2Replay_DrawFrameQueries( player, &draw, -1 );
    ENSURE( counts.polygons > 0 && counts.lines > 0 && counts.circles > 0 && counts.capsules == 2 && counts.points > 0 );
    counts = {}; b2Replay_DrawFrameQueries( player, &draw, 0 );
    ENSURE( counts.polygons == 3 );
    b2ShapeId lastHit = b2Replay_GetFrameQueryHit( player, 0, 1 ).shapeId;
    b2AABB bounds = b2Shape_GetBounds( lastHit );
    b2Pos center = b2Body_GetPosition( b2Shape_GetBody( lastHit ) );
    ENSURE( bounds.lowerBound.x < center.x && bounds.upperBound.x > center.x );
    ENSURE( bounds.lowerBound.y < center.y && bounds.upperBound.y > center.y );
    ENSURE( counts.lower.x == bounds.lowerBound.x && counts.lower.y == bounds.lowerBound.y );
    ENSURE( counts.upper.x == bounds.upperBound.x && counts.upper.y == bounds.upperBound.y );
    counts = {}; b2Replay_DrawFrameQueries( player, &draw, 2 );
    ENSURE( counts.lines >= 2 && counts.points == 2 && counts.polygons == 0 && counts.capsules == 0 );
    counts = {}; b2Replay_DrawFrameQueries( player, &draw, 99 ); ENSURE( counts.lines == 0 && counts.polygons == 0 );
    b2DebugDraw empty = {}; b2Replay_DrawFrameQueries( player, &empty, -1 );
    b2Replay_DrawFrameQueries( nullptr, &draw, -1 ); b2Replay_DrawFrameQueries( player, nullptr, -1 );
    b2DestroyReplay( player ); ENSURE( b2GetByteCount() == before ); return 0;
}

int ReplayQueryDivergenceTest( void )
{
    int before = b2GetByteCount(); auto bytes = MakeQueryReplayFixture(); b2RecHeader header;
    memcpy( &header, bytes.data(), sizeof( header ) ); int offset = 32 + (int)header.snapshotSize; bool changed = false;
    while ( offset + 4 <= (int)bytes.size() ) {
        int size = bytes[offset+1] | (int)bytes[offset+2] << 8 | (int)bytes[offset+3] << 16;
        if ( bytes[offset] == 0xE2 ) { bytes[offset + 48 + sizeof(b2Pos) - 8] ^= 1; changed = true; break; }
        offset += 4 + size;
    }
    ENSURE( changed ); b2Replay* player = b2CreateReplay( bytes.data(), (int)bytes.size(), 1 ); ENSURE( player != nullptr );
    ENSURE( b2Replay_StepFrame( player ) && b2Replay_HasDiverged( player ) );
    ENSURE( b2Replay_GetDivergeFrame( player ) == 1 && b2Replay_GetFrameQueryCount( player ) == 7 );
    b2DestroyReplay( player ); ENSURE( b2GetByteCount() == before ); return 0;
}

int ReplayBodyEnumerationTest( void )
{
    int before = b2GetByteCount();
    b2WorldDef wd = b2DefaultWorldDef(); b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef(); bd.name = "seed"; b2BodyId seed = b2CreateBody( world, &bd );
    b2Recording* rec = b2CreateRecording( 1 ); b2World_StartRecording( world, rec );
    b2World_Step( world, 1.0f / 60.0f, 4 ); bd.name = "first"; b2BodyId first = b2CreateBody( world, &bd );
    b2World_Step( world, 1.0f / 60.0f, 4 ); b2DestroyBody( first ); bd.name = "replacement";
    b2BodyId replacement = b2CreateBody( world, &bd );
    ENSURE( replacement.index1 == first.index1 && replacement.generation != first.generation );
    b2World_Step( world, 1.0f / 60.0f, 4 ); b2DestroyBody( seed );
    b2World_Step( world, 1.0f / 60.0f, 4 ); b2World_StopRecording( world ); b2DestroyWorld( world );
    int workers[] = { 1, 4 };
    for ( int worker : workers ) {
        b2Replay* player = b2CreateReplay( b2Recording_GetData( rec ), b2Recording_GetSize( rec ), worker );
        ENSURE( player != nullptr ); b2Replay_SetKeyframePolicy( player, 1024 * 1024, 1 );
        ENSURE( b2Replay_GetBodyCount( player ) == 1 );
        b2BodyId replaySeed = b2Replay_GetBodyId( player, 0 );
        ENSURE( b2Body_IsValid( replaySeed ) && strcmp( b2Body_GetName( replaySeed ), "seed" ) == 0 );
        ENSURE( b2Replay_StepFrame( player ) && b2Replay_GetBodyCount( player ) == 2 );
        b2BodyId replayFirst = b2Replay_GetBodyId( player, 1 );
        ENSURE( b2Body_IsValid( replayFirst ) && strcmp( b2Body_GetName( replayFirst ), "first" ) == 0 );
        ENSURE( b2Replay_StepFrame( player ) && b2Replay_GetBodyCount( player ) == 3 );
        ENSURE( b2Replay_GetBodyId( player, 1 ).index1 == 0 && !b2Body_IsValid( replayFirst ) );
        b2BodyId replayReplacement = b2Replay_GetBodyId( player, 2 );
        ENSURE( b2Body_IsValid( replayReplacement ) && replayReplacement.index1 == replayFirst.index1 );
        ENSURE( replayReplacement.generation != replayFirst.generation );
        ENSURE( strncmp( b2Body_GetName( replayReplacement ), "replacement", B2_NAME_LENGTH ) == 0 );
        ENSURE( strlen( b2Body_GetName( replayReplacement ) ) == B2_NAME_LENGTH );
        while ( b2Replay_StepFrame( player ) ) {}
        ENSURE( b2Replay_GetBodyId( player, 0 ).index1 == 0 && b2Replay_GetBodyCount( player ) == 3 );
        int targets[] = { 1, 4, 2, 0, 3, 1, 2 };
        for ( int target : targets ) {
            b2Replay_SeekFrame( player, target ); ENSURE( !b2Replay_HasDiverged( player ) );
            ENSURE( b2Replay_GetBodyCount( player ) == ( target == 0 ? 1 : target == 1 ? 2 : 3 ) );
            ENSURE( ( b2Replay_GetBodyId( player, 0 ).index1 == 0 ) == ( target >= 3 ) );
            if ( target >= 2 ) { ENSURE( b2Replay_GetBodyId( player, 1 ).index1 == 0 ); ENSURE( B2_ID_EQUALS( b2Replay_GetBodyId( player, 2 ), replayReplacement ) ); }
            else if ( target == 1 ) { ENSURE( B2_ID_EQUALS( b2Replay_GetBodyId( player, 1 ), replayFirst ) ); }
        }
        ENSURE( b2Replay_GetBodyId( player, -1 ).index1 == 0 && b2Replay_GetBodyId( player, 99 ).index1 == 0 );
        b2Replay_Restart( player ); ENSURE( b2Replay_GetBodyCount( player ) == 1 );
        ENSURE( B2_ID_EQUALS( b2Replay_GetBodyId( player, 0 ), replaySeed ) ); b2DestroyReplay( player );
    }
    ENSURE( b2Replay_GetBodyCount( nullptr ) == 0 && b2Replay_GetBodyId( nullptr, 0 ).index1 == 0 );
    b2DestroyRecording( rec ); ENSURE( b2GetByteCount() == before ); return 0;
}

int RecordingIntegrityTest( void )
{
    int before = b2GetByteCount(); float original = b2GetLengthUnitsPerMeter();
    b2SetLengthUnitsPerMeter( 2 );
    auto empty = MakeReplayFixture( 0 ); auto normal = MakeReplayFixture( 40 ); auto queries = MakeQueryReplayFixture();
    std::vector<uint64_t> hashes; auto mutations = MakeMutableReplayFixture( &hashes );
    b2SetLengthUnitsPerMeter( 3 ); int allocated = b2GetByteCount();
    int workers[] = { 0, 1, 4 };
    for ( int worker : workers ) {
        for ( const auto* bytes : { &empty, &normal, &queries, &mutations } ) {
            ENSURE( b2ValidateRecording( bytes->data(), (int)bytes->size(), worker ) );
            ENSURE( b2GetByteCount() == allocated && b2GetLengthUnitsPerMeter() == 3 );
        }
    }
    b2SetLengthUnitsPerMeter( original ); ENSURE( b2GetByteCount() == before ); return 0;
}

int RecordingIntegrityInvalidTest( void )
{
    int before = b2GetByteCount(); auto good = MakeReplayFixture( 5 ); auto queries = MakeQueryReplayFixture();
    ENSURE( !b2ValidateRecording( nullptr, 0, 0 ) );
    for ( int which = 0; which < 5; ++which ) {
        auto bytes = which == 4 ? queries : good;
        if ( which == 0 ) bytes[0] = 0;
        if ( which == 1 ) bytes[4]++;
        if ( which == 2 ) bytes.pop_back();
        if ( which >= 3 ) {
            b2RecHeader header; memcpy( &header, bytes.data(), sizeof( header ) );
            int offset = 32 + (int)header.snapshotSize; bool changed = false;
            while ( offset + 4 <= (int)bytes.size() ) {
                int size = bytes[offset+1] | (int)bytes[offset+2] << 8 | (int)bytes[offset+3] << 16;
                if ( which == 3 && bytes[offset] == 0xF1 ) { bytes[offset + 8] ^= 1; changed = true; break; }
                if ( which == 4 && bytes[offset] == 0xE2 ) { bytes[offset + 48 + sizeof(b2Pos) - 8] ^= 1; changed = true; break; }
                offset += 4 + size;
            }
            ENSURE( changed );
        }
        ENSURE( !b2ValidateRecording( bytes.data(), (int)bytes.size(), 4 ) );
        ENSURE( b2GetByteCount() == before );
    }
    return 0;
}

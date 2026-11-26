// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "recording.h"
#include "state_hash.h"
#include "world.h"

#include <string.h>
#include <vector>

int RecordingBufferTest( void )
{
	int baseline = b2GetByteCount();
	b2RecBuffer buf = {};
	b2RecBufAppend( &buf, NULL, 0 );
	b2RecBufAppend( &buf, NULL, -1 );
	ENSURE( buf.data == NULL && buf.size == 0 && buf.capacity == 0 );
	for ( int i = 0; i < 5000; ++i )
	{
		uint8_t value = (uint8_t)( i % 251 );
		b2RecBufAppend( &buf, &value, 1 );
	}
	ENSURE( buf.size == 5000 && buf.capacity >= buf.size );
	for ( int i = 0; i < buf.size; ++i )
	{
		ENSURE( buf.data[i] == (uint8_t)( i % 251 ) );
	}
	uint8_t tail[12000];
	memset( tail, 0xA6, sizeof( tail ) );
	b2RecBufAppend( &buf, tail, sizeof( tail ) );
	ENSURE( buf.size == 17000 && buf.capacity >= buf.size );
	ENSURE( memcmp( buf.data + 5000, tail, sizeof( tail ) ) == 0 );
	b2RecBufFree( &buf );
	ENSURE( buf.data == NULL && buf.size == 0 && buf.capacity == 0 );
	b2RecBufFree( &buf );
	b2RecBuffer count = {};
	count.countOnly = true;
	b2RecBufAppend( &count, NULL, 17000 );
	ENSURE( count.size == 17000 && count.data == NULL && count.capacity == 0 );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

int RecordingPrimitiveEncodingTest( void )
{
	b2RecBuffer buf = {};
	b2RecW_U8( &buf, 0xA5 );
	b2RecW_U16( &buf, 0x1234 );
	b2RecW_U32( &buf, 0x89ABCDEFu );
	b2RecW_U64( &buf, 0x0123456789ABCDEFull );
	b2RecW_I32( &buf, -2 );
	b2RecW_F32( &buf, 1.0f );
	b2RecW_F64( &buf, -2.0 );
	b2RecW_BOOL( &buf, false );
	b2RecW_BOOL( &buf, true );
	const uint8_t expected[] = {
		0xA5, 0x34, 0x12, 0xEF, 0xCD, 0xAB, 0x89,
		0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01,
		0xFE, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x80, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x01
	};
	ENSURE( buf.size == sizeof( expected ) );
	ENSURE( memcmp( buf.data, expected, sizeof( expected ) ) == 0 );
	buf.size = 0;
	b2WorldTransform xf = { { 1.0f, -2.0f }, { 0.0f, 1.0f } };
	b2RecW_WORLDXF( &buf, xf );
	const uint8_t transformBytes[] = {
		0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0xC0,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F
	};
	ENSURE( buf.size == sizeof( transformBytes ) );
	ENSURE( memcmp( buf.data, transformBytes, sizeof( transformBytes ) ) == 0 );
	buf.size = 0;
	b2RecW_WORLDID( &buf, b2WorldId{ 0x1234, 0x5678 } );
	b2RecW_BODYID( &buf, b2BodyId{ 0x12345678, 0x9ABC, 0xDEF0 } );
	const uint8_t idBytes[] = { 0x78, 0x56, 0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12 };
	ENSURE( buf.size == sizeof( idBytes ) );
	ENSURE( memcmp( buf.data, idBytes, sizeof( idBytes ) ) == 0 );
	b2RecBufFree( &buf );
	return 0;
}

int RecordingStringEncodingTest( void )
{
	b2RecBuffer buf = {};
	b2RecW_STR( &buf, NULL );
	b2RecW_STR( &buf, "" );
	b2RecW_STR( &buf, "box" );
	const uint8_t expected[] = { 0xFF, 0xFF, 0, 0, 3, 0, 'b', 'o', 'x' };
	ENSURE( buf.size == sizeof( expected ) );
	ENSURE( memcmp( buf.data, expected, sizeof( expected ) ) == 0 );
	std::vector<char> longName( 70001, 'x' );
	longName.back() = 0;
	buf.size = 0;
	b2RecW_STR( &buf, longName.data() );
	ENSURE( buf.size == 65536 );
	ENSURE( buf.data[0] == 0xFE && buf.data[1] == 0xFF );
	for ( int i = 2; i < buf.size; ++i )
	{
		ENSURE( buf.data[i] == 'x' );
	}
	b2RecBuffer count = {};
	count.countOnly = true;
	b2RecW_STR( &count, longName.data() );
	ENSURE( count.size == buf.size && count.data == NULL );
	b2RecBufFree( &buf );
	return 0;
}

int RecordingDefinitionEncodingTest( void )
{
	int baseline = b2GetByteCount();
	b2RecBuffer a = {}, b = {}, count = {};
	count.countOnly = true;
	int tag;
	b2ShapeDef shape = b2DefaultShapeDef();
	shape.userData = &tag;
	b2RecW_SHAPEDEF( &a, shape );
	shape.userData = NULL;
	shape.internalValue = 0;
	b2RecW_SHAPEDEF( &b, shape );
	ENSURE( a.size == b.size && memcmp( a.data, b.data, a.size ) == 0 );
	for ( int i = 0; i < 8; ++i ) ENSURE( a.data[i] == 0 );
	a.size = b.size = 0;
	b2BodyDef body = b2DefaultBodyDef();
	body.name = "recorded";
	body.userData = &tag;
	b2RecW_BODYDEF( &a, body );
	body.userData = NULL;
	body.internalValue = 0;
	b2RecW_BODYDEF( &b, body );
	b2RecW_BODYDEF( &count, body );
	ENSURE( a.size == b.size && count.size == a.size );
	ENSURE( memcmp( a.data, b.data, a.size ) == 0 && count.data == NULL );
	b2Vec2 points[] = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };
	b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
	b2ChainDef chain = b2DefaultChainDef();
	chain.points = points;
	chain.pointCount = 4;
	chain.materials = &material;
	chain.materialCount = 1;
	a.size = count.size = 0;
	b2RecW_CHAINDEF( &a, chain );
	b2RecW_CHAINDEF( &count, chain );
	ENSURE( a.size == count.size && count.data == NULL );
	ENSURE( a.data[8] == 4 && a.data[9] == 0 && a.data[10] == 0 && a.data[11] == 0 );
	// Definitions embed the values, so editing source arrays cannot change existing bytes.
	std::vector<uint8_t> saved( a.data, a.data + a.size );
	points[0].x = 17.0f;
	ENSURE( memcmp( a.data, saved.data(), a.size ) == 0 );
	b2RecBufFree( &a );
	b2RecBufFree( &b );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

int RecordingLifetimeTest( void )
{
	int baseline = b2GetByteCount();
	b2DestroyRecording( NULL );
	for ( int i = 0; i < 20; ++i )
	{
		b2Recording* rec = b2CreateRecording( i % 2 == 0 ? 0 : -1 );
		ENSURE( rec != NULL && b2Recording_GetData( rec ) != NULL );
		ENSURE( b2Recording_GetSize( rec ) == 0 );
		ENSURE( rec->buffer.capacity == 65536 && rec->lock != NULL );
		b2LockMutex( rec->lock );
		b2UnlockMutex( rec->lock );
		b2DestroyRecording( rec );
		ENSURE( b2GetByteCount() == baseline );
	}
	return 0;
}

int RecordingCapacityTest( void )
{
	int baseline = b2GetByteCount();
	int capacities[] = { 1, 17, 4096 };
	for ( int capacity : capacities )
	{
		b2Recording* rec = b2CreateRecording( capacity );
		ENSURE( rec->buffer.capacity == capacity );
		for ( int i = 0; i < capacity + 100; ++i ) b2RecW_U8( &rec->buffer, (uint8_t)i );
		ENSURE( b2Recording_GetSize( rec ) == capacity + 100 );
		ENSURE( rec->buffer.capacity >= capacity + 100 );
		const uint8_t* bytes = b2Recording_GetData( rec );
		for ( int i = 0; i < capacity + 100; ++i ) ENSURE( bytes[i] == (uint8_t)i );
		b2DestroyRecording( rec );
	}
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

static uint32_t ReadRecordingU32( const uint8_t* data )
{
	return (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}

static int RecordingPayloadSize( const uint8_t* record )
{
	return record[1] | (int)record[2] << 8 | (int)record[3] << 16;
}

int RecordingSessionTest( void )
{
	int baseline = b2GetByteCount();
	b2WorldDef wd = b2DefaultWorldDef();
	b2WorldId id = b2CreateWorld( &wd );
	b2BodyDef bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.position = { 2, 4 };
	b2BodyId body = b2CreateBody( id, &bd );
	b2ShapeDef sd = b2DefaultShapeDef();
	b2Circle circle = { { 0, 0 }, 1.0f };
	b2CreateCircleShape( body, &sd, &circle );
	b2Recording* rec = b2CreateRecording( 1 );
	b2World_StartRecording( id, rec );
	ENSURE( b2_worlds[id.index1 - 1].recording == rec );
	b2RecHeader header;
	memcpy( &header, b2Recording_GetData( rec ), sizeof( header ) );
	ENSURE( sizeof( header ) == 32 && header.magic == B2_REC_MAGIC );
	ENSURE( header.versionMajor == 3 && header.versionMinor == 13 );
	ENSURE( header.pointerWidth == sizeof( void* ) && header.bigEndian == 0 );
	ENSURE( header.lengthScale == b2GetLengthUnitsPerMeter() && header.snapshotSize > 0 );
	int snapshotSize = b2World_GetSnapshot( id, NULL, 0 );
	ENSURE( header.snapshotSize == snapshotSize );
	std::vector<uint8_t> snapshot( snapshotSize );
	b2World_GetSnapshot( id, snapshot.data(), snapshotSize );
	ENSURE( memcmp( b2Recording_GetData( rec ) + sizeof( header ), snapshot.data(), snapshotSize ) == 0 );
	b2WorldId loaded = b2CreateWorldFromSnapshot( snapshot.data(), snapshotSize, 1 );
	ENSURE( b2World_IsValid( loaded ) );
	ENSURE( b2World_GetStateHash( loaded ) == b2World_GetStateHash( id ) );
	b2DestroyWorld( loaded );
	int offset = (int)sizeof( header ) + snapshotSize;
	const uint8_t* bytes = b2Recording_GetData( rec );
	ENSURE( bytes[offset] == 0xF1 && RecordingPayloadSize( bytes + offset ) == 12 );
	ENSURE( ReadRecordingU32( bytes + offset + 4 ) == b2StoreWorldId( id ) );
	uint64_t hash = b2HashWorldState( b2_worlds + id.index1 - 1 );
	ENSURE( ReadRecordingU32( bytes + offset + 8 ) == (uint32_t)hash );
	ENSURE( ReadRecordingU32( bytes + offset + 12 ) == (uint32_t)( hash >> 32 ) );
	offset += 16;
	ENSURE( b2Recording_GetSize( rec ) == offset );
	b2World_StopRecording( id );
	ENSURE( b2_worlds[id.index1 - 1].recording == NULL );
	bytes = b2Recording_GetData( rec );
	ENSURE( bytes[offset] == 0xF2 && RecordingPayloadSize( bytes + offset ) == 16 );
	offset += 20;
	ENSURE( bytes[offset] == 0x01 && RecordingPayloadSize( bytes + offset ) == 4 );
	ENSURE( ReadRecordingU32( bytes + offset + 4 ) == b2StoreWorldId( id ) );
	ENSURE( b2Recording_GetSize( rec ) == offset + 8 );
	std::vector<uint8_t> session( bytes, bytes + b2Recording_GetSize( rec ) );
	b2World_StopRecording( id );
	ENSURE( b2Recording_GetSize( rec ) == session.size() );
	b2World_StartRecording( id, rec );
	b2World_StopRecording( id );
	ENSURE( b2Recording_GetSize( rec ) == session.size() );
	ENSURE( memcmp( session.data(), b2Recording_GetData( rec ), session.size() ) == 0 );
	b2DestroyWorld( id );
	b2DestroyRecording( rec );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

static bool RecordingFilter( b2ShapeId, b2ShapeId, void* ) { return true; }

int RecordingSessionGuardTest( void )
{
	int baseline = b2GetByteCount();
	b2WorldDef wd = b2DefaultWorldDef();
	b2WorldId id = b2CreateWorld( &wd );
	b2World* world = b2_worlds + id.index1 - 1;
	b2Recording* rec = b2CreateRecording( 0 );
	b2Recording* unused = b2CreateRecording( 0 );
	b2World_StartRecording( id, NULL );
	ENSURE( world->recording == NULL );
	world->locked = true;
	b2World_StartRecording( id, rec );
	ENSURE( world->recording == NULL && b2Recording_GetSize( rec ) == 0 );
	world->locked = false;
	b2World_SetCustomFilterCallback( id, RecordingFilter, NULL );
	b2World_StartRecording( id, rec );
	ENSURE( world->recording == NULL && b2Recording_GetSize( rec ) == 0 );
	b2World_SetCustomFilterCallback( id, NULL, NULL );
	b2World_StartRecording( id, rec );
	int size = b2Recording_GetSize( rec );
	b2World_StartRecording( id, unused );
	ENSURE( world->recording == rec && b2Recording_GetSize( unused ) == 0 );
	world->locked = true;
	b2World_StopRecording( id );
	ENSURE( world->recording == rec && b2Recording_GetSize( rec ) == size );
	world->locked = false;
	b2World_StopRecording( id );
	ENSURE( world->recording == NULL );
	b2DestroyRecording( unused );
	b2DestroyRecording( rec );
	b2DestroyWorld( id );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

int RecordingWorldLifetimeTest( void )
{
	int baseline = b2GetByteCount();
	b2WorldDef wd = b2DefaultWorldDef();
	b2WorldId id = b2CreateWorld( &wd );
	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( id, rec );
	int before = b2Recording_GetSize( rec );
	b2DestroyWorld( id );
	ENSURE( b2Recording_GetSize( rec ) == before + 28 );
	const uint8_t* bytes = b2Recording_GetData( rec );
	ENSURE( bytes[before] == 0xF2 && bytes[before + 20] == 0x01 );
	b2WorldId next = b2CreateWorld( &wd );
	ENSURE( b2_worlds[next.index1 - 1].recording == NULL );
	b2DestroyWorld( next );
	b2DestroyRecording( rec );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

// Each public operation must append one complete frame with the stable opcode.
#define CHECK_RECORD( opcode, ... ) \
	do { \
		int previousSize = b2Recording_GetSize( rec ); \
		__VA_ARGS__; \
		int currentSize = b2Recording_GetSize( rec ); \
		ENSURE( currentSize >= previousSize + 4 ); \
		const uint8_t* frame = b2Recording_GetData( rec ) + previousSize; \
		ENSURE( frame[0] == opcode ); \
		ENSURE( RecordingPayloadSize( frame ) + 4 == currentSize - previousSize ); \
	} while ( 0 )

int RecordingWorldOperationsTest( void )
{
	int baseline = b2GetByteCount();
	b2WorldDef wd = b2DefaultWorldDef();
	b2WorldId id = b2CreateWorld( &wd );
	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( id, rec );
	CHECK_RECORD( 0x02, b2World_EnableSleeping( id, false ) );
	CHECK_RECORD( 0x03, b2World_EnableContinuous( id, false ) );
	CHECK_RECORD( 0x04, b2World_SetRestitutionThreshold( id, 2.0f ) );
	CHECK_RECORD( 0x05, b2World_SetHitEventThreshold( id, 3.0f ) );
	int gravityOffset = b2Recording_GetSize( rec );
	CHECK_RECORD( 0x06, b2World_SetGravity( id, ( b2Vec2{ 1.0f, -2.0f } ) ) );
	const uint8_t* gravity = b2Recording_GetData( rec ) + gravityOffset;
	ENSURE( RecordingPayloadSize( gravity ) == 12 );
	ENSURE( ReadRecordingU32( gravity + 4 ) == b2StoreWorldId( id ) );
	ENSURE( ReadRecordingU32( gravity + 8 ) == 0x3F800000u );
	ENSURE( ReadRecordingU32( gravity + 12 ) == 0xC0000000u );
	b2ExplosionDef explosion = b2DefaultExplosionDef();
	CHECK_RECORD( 0x07, b2World_Explode( id, &explosion ) );
	CHECK_RECORD( 0x08, b2World_SetContactTuning( id, 20, 0.8f, 2 ) );
	CHECK_RECORD( 0x09, b2World_SetContactRecycleDistance( id, 0.02f ) );
	CHECK_RECORD( 0x0A, b2World_SetMaxLinearSpeed( id, 100 ) );
	CHECK_RECORD( 0x0B, b2World_EnableWarmStarting( id, false ) );
	CHECK_RECORD( 0x0C, b2World_RebuildStaticTree( id ) );
	CHECK_RECORD( 0x0D, b2World_SetRestitutionIterations( id, 2 ) );
	CHECK_RECORD( 0x0E, b2World_EnableRestitutionPropagation( id, false ) );
	b2World_StopRecording( id );
	int stoppedSize = b2Recording_GetSize( rec );
	b2World_SetGravity( id, b2Vec2_zero );
	ENSURE( b2Recording_GetSize( rec ) == stoppedSize );
	b2DestroyWorld( id );
	b2DestroyRecording( rec );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

int RecordingBodyOperationsTest( void )
{
	int baseline = b2GetByteCount();
	b2WorldDef wd = b2DefaultWorldDef();
	b2WorldId id = b2CreateWorld( &wd );
	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( id, rec );
	b2BodyDef bd = b2DefaultBodyDef();
	b2BodyId body;
	CHECK_RECORD( 0x10, body = b2CreateBody( id, &bd ) );
	CHECK_RECORD( 0x20, b2Body_SetTransform( body, ( b2Pos{ 3, 5 } ), b2Rot_identity ) );
	CHECK_RECORD( 0x22, b2Body_SetType( body, b2_dynamicBody ) );
	CHECK_RECORD( 0x21, b2Body_SetLinearVelocity( body, ( b2Vec2{ 1, 2 } ) ) );
	CHECK_RECORD( 0x23, b2Body_SetName( body, "moving" ) );
	CHECK_RECORD( 0x24, b2Body_SetAngularVelocity( body, 0.5f ) );
	b2WorldTransform target = { { 4, 6 }, b2Rot_identity };
	CHECK_RECORD( 0x25, b2Body_SetTargetTransform( body, target, 0.5f, true ) );
	CHECK_RECORD( 0x26, b2Body_ApplyForce( body, ( b2Vec2{ 1, 0 } ), ( b2Pos{ 3, 5 } ), true ) );
	CHECK_RECORD( 0x27, b2Body_ApplyForceToCenter( body, ( b2Vec2{ 0, 1 } ), true ) );
	CHECK_RECORD( 0x28, b2Body_ApplyTorque( body, 1, true ) );
	CHECK_RECORD( 0x29, b2Body_ClearForces( body ) );
	CHECK_RECORD( 0x2A, b2Body_ApplyLinearImpulse( body, ( b2Vec2{ 1, 0 } ), ( b2Pos{ 3, 5 } ), true ) );
	CHECK_RECORD( 0x2B, b2Body_ApplyLinearImpulseToCenter( body, ( b2Vec2{ 0, 1 } ), true ) );
	CHECK_RECORD( 0x2C, b2Body_ApplyAngularImpulse( body, 1, true ) );
	b2MassData mass = { 2, { 0, 0 }, 1 };
	CHECK_RECORD( 0x2D, b2Body_SetMassData( body, mass ) );
	CHECK_RECORD( 0x2E, b2Body_UpdateMassFromShapes( body ) );
	CHECK_RECORD( 0x2F, b2Body_SetLinearDamping( body, 0.1f ) );
	CHECK_RECORD( 0x30, b2Body_SetAngularDamping( body, 0.2f ) );
	CHECK_RECORD( 0x31, b2Body_SetGravityScale( body, 2.0f ) );
	CHECK_RECORD( 0x32, b2Body_SetAwake( body, false ) );
	CHECK_RECORD( 0x33, b2Body_WakeTouching( body ) );
	CHECK_RECORD( 0x34, b2Body_EnableSleeping( body, false ) );
	CHECK_RECORD( 0x35, b2Body_SetSleepThreshold( body, 0.05f ) );
	CHECK_RECORD( 0x36, b2Body_Disable( body ) );
	CHECK_RECORD( 0x37, b2Body_Enable( body ) );
	CHECK_RECORD( 0x38, b2Body_SetMotionLocks( body, ( b2MotionLocks{ true, false, false } ) ) );
	CHECK_RECORD( 0x39, b2Body_SetBullet( body, true ) );
	CHECK_RECORD( 0x3A, b2Body_EnableContactRecycling( body, false ) );
	CHECK_RECORD( 0x3B, b2Body_EnableContactEvents( body, true ) );
	CHECK_RECORD( 0x3C, b2Body_EnableHitEvents( body, true ) );
	CHECK_RECORD( 0x3D, b2Body_SetSafetyFactor( body, 0.8f ) );
	CHECK_RECORD( 0x11, b2DestroyBody( body ) );
	b2World_StopRecording( id );
	b2DestroyRecording( rec );
	b2DestroyWorld( id );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

int RecordingBodyPayloadTest( void )
{
	b2WorldDef wd = b2DefaultWorldDef();
	b2WorldId id = b2CreateWorld( &wd );
	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( id, rec );
	b2BodyDef bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.name = "payload";
	int start = b2Recording_GetSize( rec );
	b2BodyId body = b2CreateBody( id, &bd );
	const uint8_t* frame = b2Recording_GetData( rec ) + start;
	int payloadSize = RecordingPayloadSize( frame );
	ENSURE( frame[0] == 0x10 && ReadRecordingU32( frame + 4 ) == b2StoreWorldId( id ) );
	ENSURE( ReadRecordingU32( frame + 8 ) == b2_dynamicBody );
	uint64_t stored = b2StoreBodyId( body );
	ENSURE( ReadRecordingU32( frame + payloadSize - 4 ) == (uint32_t)stored );
	ENSURE( ReadRecordingU32( frame + payloadSize ) == (uint32_t)( stored >> 32 ) );
	start = b2Recording_GetSize( rec );
	b2Body_SetName( body, "abc" );
	frame = b2Recording_GetData( rec ) + start;
	ENSURE( frame[0] == 0x23 && RecordingPayloadSize( frame ) == 13 );
	ENSURE( frame[12] == 3 && frame[13] == 0 && memcmp( frame + 14, "abc", 3 ) == 0 );
	b2World_StopRecording( id );
	b2DestroyWorld( id );
	b2DestroyRecording( rec );
	return 0;
}

int RecordingShapeOperationsTest( void )
{
	int baseline = b2GetByteCount();
	b2WorldDef wd = b2DefaultWorldDef();
	b2WorldId id = b2CreateWorld( &wd );
	b2BodyDef bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	b2BodyId body = b2CreateBody( id, &bd );
	b2Recording* rec = b2CreateRecording( 1 );
	b2World_StartRecording( id, rec );
	b2ShapeDef sd = b2DefaultShapeDef();
	b2Circle circle = { { 0, 0 }, 1 };
	b2Capsule capsule = { { -1, 0 }, { 1, 0 }, 0.5f };
	b2Segment segment = { { -1, 0 }, { 1, 0 } };
	b2Polygon polygon = b2MakeBox( 1, 1 );
	b2ChainSegment chainSegment = { { -2, 0 }, { { -1, 0 }, { 1, 0 } }, { 2, 0 }, 23 };
	b2ShapeId shapes[5];
	int start = b2Recording_GetSize( rec );
	CHECK_RECORD( 0x40, shapes[0] = b2CreateCircleShape( body, &sd, &circle ) );
	const uint8_t* frame = b2Recording_GetData( rec ) + start;
	int payload = RecordingPayloadSize( frame );
	uint64_t stored = b2StoreShapeId( shapes[0] );
	ENSURE( ReadRecordingU32( frame + payload - 4 ) == (uint32_t)stored );
	ENSURE( ReadRecordingU32( frame + payload ) == (uint32_t)( stored >> 32 ) );
	CHECK_RECORD( 0x41, shapes[1] = b2CreateCapsuleShape( body, &sd, &capsule ) );
	CHECK_RECORD( 0x42, shapes[2] = b2CreateSegmentShape( body, &sd, &segment ) );
	CHECK_RECORD( 0x43, shapes[3] = b2CreatePolygonShape( body, &sd, &polygon ) );
	CHECK_RECORD( 0x44, shapes[4] = b2CreateChainSegmentShape( body, &sd, &chainSegment ) );
	b2ShapeId shape = shapes[0];
	CHECK_RECORD( 0x50, b2Shape_SetDensity( shape, 2.0f, true ) );
	CHECK_RECORD( 0x51, b2Shape_SetFriction( shape, 0.25f ) );
	CHECK_RECORD( 0x52, b2Shape_SetRestitution( shape, 0.5f ) );
	CHECK_RECORD( 0x53, b2Shape_SetUserMaterial( shape, 0x123456789ABCDEFull ) );
	b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
	material.friction = 0.4f;
	CHECK_RECORD( 0x54, b2Shape_SetSurfaceMaterial( shape, &material ) );
	b2Filter filter = b2DefaultFilter();
	filter.categoryBits = 4;
	CHECK_RECORD( 0x55, b2Shape_SetFilter( shape, filter ) );
	CHECK_RECORD( 0x56, b2Shape_EnableSensorEvents( shape, true ) );
	CHECK_RECORD( 0x57, b2Shape_EnableContactEvents( shape, true ) );
	CHECK_RECORD( 0x58, b2Shape_EnablePreSolveEvents( shape, true ) );
	CHECK_RECORD( 0x59, b2Shape_EnableHitEvents( shape, true ) );
	CHECK_RECORD( 0x5F, b2Shape_ApplyWind( shape, ( b2Vec2{ 2, 0 } ), 0.2f, 0.1f, true ) );
	CHECK_RECORD( 0x5A, b2Shape_SetCircle( shape, &circle ) );
	CHECK_RECORD( 0x5B, b2Shape_SetCapsule( shape, &capsule ) );
	CHECK_RECORD( 0x5C, b2Shape_SetSegment( shape, &segment ) );
	CHECK_RECORD( 0x5D, b2Shape_SetPolygon( shape, &polygon ) );
	CHECK_RECORD( 0x5E, b2Shape_SetChainSegment( shape, &chainSegment ) );
	ENSURE( b2Shape_GetChainSegment( shape ).chainId == B2_NULL_INDEX );
	CHECK_RECORD( 0x45, b2DestroyShape( shape, true ) );
	b2World_StopRecording( id );
	int stopped = b2Recording_GetSize( rec );
	b2Shape_SetFriction( shapes[1], 0.7f );
	ENSURE( b2Recording_GetSize( rec ) == stopped );
	b2DestroyWorld( id );
	b2DestroyRecording( rec );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

int RecordingChainOperationsTest( void )
{
	int baseline = b2GetByteCount();
	b2WorldDef wd = b2DefaultWorldDef();
	b2WorldId id = b2CreateWorld( &wd );
	b2BodyDef bd = b2DefaultBodyDef();
	b2BodyId body = b2CreateBody( id, &bd );
	b2Recording* rec = b2CreateRecording( 1 );
	b2World_StartRecording( id, rec );
	b2Vec2 points[] = { { 0, 0 }, { 2, 0 }, { 2, 2 }, { 0, 2 } };
	b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
	b2ChainDef cd = b2DefaultChainDef();
	cd.points = points;
	cd.pointCount = 4;
	cd.materials = &material;
	cd.materialCount = 1;
	cd.isLoop = true;
	b2ChainId chain;
	int start = b2Recording_GetSize( rec );
	CHECK_RECORD( 0x70, chain = b2CreateChain( body, &cd ) );
	const uint8_t* frame = b2Recording_GetData( rec ) + start;
	ENSURE( ReadRecordingU32( frame + 20 ) == 4 );
	ENSURE( ReadRecordingU32( frame + 32 ) == 0x40000000u );
	int payload = RecordingPayloadSize( frame );
	uint64_t stored = b2StoreChainId( chain );
	ENSURE( ReadRecordingU32( frame + payload - 4 ) == (uint32_t)stored );
	ENSURE( ReadRecordingU32( frame + payload ) == (uint32_t)( stored >> 32 ) );
	std::vector<uint8_t> saved( frame, frame + 4 + payload );
	points[1].x = 17;
	material.friction = 0.9f;
	ENSURE( memcmp( saved.data(), b2Recording_GetData( rec ) + start, saved.size() ) == 0 );
	CHECK_RECORD( 0x72, b2Chain_SetSurfaceMaterial( chain, &material, 1 ) );
	CHECK_RECORD( 0x73, b2Chain_SetAllSurfaceMaterials( chain, &material ) );
	b2ShapeId segments[4];
	ENSURE( b2Chain_GetSegments( chain, segments, 4 ) == 4 );
	CHECK_RECORD( 0x71, b2DestroyChain( chain ) );
	for ( b2ShapeId segmentId : segments ) ENSURE( !b2Shape_IsValid( segmentId ) );
	b2World_StopRecording( id );
	b2DestroyRecording( rec );
	b2DestroyWorld( id );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

int RecordingCascadeDestroyTest( void )
{
	int baseline = b2GetByteCount();
	b2WorldDef wd = b2DefaultWorldDef();
	b2WorldId id = b2CreateWorld( &wd );
	b2BodyDef bd = b2DefaultBodyDef();
	b2BodyId body = b2CreateBody( id, &bd );
	b2ShapeDef sd = b2DefaultShapeDef();
	b2Circle circle = { { 0, 0 }, 1 };
	b2ShapeId ordinary = b2CreateCircleShape( body, &sd, &circle );
	sd.isSensor = true;
	b2ShapeId sensor = b2CreateCircleShape( body, &sd, &circle );
	b2Vec2 points[] = { { 0, 0 }, { 2, 0 }, { 1, 2 } };
	b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
	b2ChainDef cd = b2DefaultChainDef();
	cd.points = points;
	cd.pointCount = 3;
	cd.materials = &material;
	cd.materialCount = 1;
	cd.isLoop = true;
	b2ChainId chain = b2CreateChain( body, &cd );
	b2ShapeId segments[3];
	ENSURE( b2Chain_GetSegments( chain, segments, 3 ) == 3 );
	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( id, rec );
	CHECK_RECORD( 0x11, b2DestroyBody( body ) );
	ENSURE( !b2Body_IsValid( body ) && !b2Chain_IsValid( chain ) );
	ENSURE( !b2Shape_IsValid( ordinary ) && !b2Shape_IsValid( sensor ) );
	for ( b2ShapeId segment : segments ) ENSURE( !b2Shape_IsValid( segment ) );
	ENSURE( b2_worlds[id.index1 - 1].sensors.count == 0 );
	// Reusing the freed slots must retain valid generations and linked ownership.
	b2BodyId reused = b2CreateBody( id, &bd );
	b2ShapeId reusedShape = b2CreateCircleShape( reused, &sd, &circle );
	ENSURE( b2Shape_IsValid( reusedShape ) && !b2Shape_IsValid( sensor ) );
	b2World_StopRecording( id );
	b2DestroyWorld( id );
	b2DestroyRecording( rec );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

static bool CountRecordingShape( b2ShapeId, void* context )
{
	*(int*)context += 1;
	return true;
}

int RecordingChainSegmentTest( void )
{
	int baseline = b2GetByteCount();
	b2WorldDef wd = b2DefaultWorldDef();
	b2WorldId id = b2CreateWorld( &wd );
	b2BodyDef bd = b2DefaultBodyDef();
	b2BodyId body = b2CreateBody( id, &bd );
	b2ShapeDef sd = b2DefaultShapeDef();
	b2Recording* rec = b2CreateRecording( 1 );
	b2World_StartRecording( id, rec );
	b2ChainSegment geometry = { { 0, 0 }, { { 0, 0 }, { 2, 0 } }, { 2, 0 }, 99 };
	b2ShapeId shape;
	CHECK_RECORD( 0x44, shape = b2CreateChainSegmentShape( body, &sd, &geometry ) );
	b2ChainSegment stored = b2Shape_GetChainSegment( shape );
	ENSURE( stored.chainId == B2_NULL_INDEX );
	ENSURE( stored.ghost1.x == 0 && stored.ghost1.y == 1 );
	ENSURE( stored.ghost2.x == 2 && stored.ghost2.y == 1 );
	geometry = { { 10, 0 }, { { 10, 0 }, { 12, 0 } }, { 12, 0 }, 88 };
	CHECK_RECORD( 0x5E, b2Shape_SetChainSegment( shape, &geometry ) );
	stored = b2Shape_GetChainSegment( shape );
	ENSURE( stored.chainId == B2_NULL_INDEX && stored.ghost1.x == 10 && stored.ghost1.y == 1 );
	b2World* world = b2_worlds + id.index1 - 1;
	b2Shape* internal = world->shapes.data + shape.index1 - 1;
	ENSURE( internal->aabb.lowerBound.x > 9 && world->fatAABBs.data[internal->id].lowerBound.x > 9 );
	int oldHits = 0, newHits = 0;
	b2World_OverlapAABB( id, b2Vec2_zero, ( b2AABB{ { -1, -1 }, { 3, 1 } } ), b2DefaultQueryFilter(), CountRecordingShape, &oldHits );
	b2World_OverlapAABB( id, b2Vec2_zero, ( b2AABB{ { 9, -1 }, { 13, 1 } } ), b2DefaultQueryFilter(), CountRecordingShape, &newHits );
	ENSURE( oldHits == 0 && newHits == 1 );
	int size = b2Recording_GetSize( rec );
	geometry.segment.point2 = geometry.segment.point1;
	b2Shape_SetChainSegment( shape, &geometry );
	ENSURE( b2Recording_GetSize( rec ) == size );
	ENSURE( b2CreateChainSegmentShape( body, &sd, &geometry ).index1 == 0 );
	ENSURE( b2CreateChainSegmentShape( body, &sd, NULL ).index1 == 0 );
	ENSURE( b2Recording_GetSize( rec ) == size );
	geometry.segment.point2 = { 12, 0 };
	world->locked = true;
	b2Shape_SetChainSegment( shape, &geometry );
	ENSURE( b2Recording_GetSize( rec ) == size );
	world->locked = false;
	b2Vec2 points[] = { { 0, 0 }, { 2, 0 }, { 1, 2 } };
	b2SurfaceMaterial material = b2DefaultSurfaceMaterial();
	b2ChainDef cd = b2DefaultChainDef();
	cd.points = points; cd.pointCount = 3; cd.materials = &material; cd.materialCount = 1; cd.isLoop = true;
	b2ChainId chain = b2CreateChain( body, &cd );
	b2ShapeId owned;
	ENSURE( b2Chain_GetSegments( chain, &owned, 1 ) == 1 );
	size = b2Recording_GetSize( rec );
	b2Shape_SetChainSegment( owned, &geometry );
	b2DestroyShape( owned, false );
	ENSURE( b2Recording_GetSize( rec ) == size && b2Shape_IsValid( owned ) );
	ENSURE( b2Shape_GetParentChain( owned ).index1 == chain.index1 );
	b2World_StopRecording( id );
	b2DestroyWorld( id );
	b2DestroyRecording( rec );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

static b2JointId CreateRecordingJoint( int type, b2WorldId world, b2BodyId a, b2BodyId b )
{
	// Every definition carries body handles through its common base.
#define CREATE_RECORDING_JOINT( Type ) \
	do { \
		b2##Type##JointDef def = b2Default##Type##JointDef(); \
		def.base.bodyIdA = a; def.base.bodyIdB = b; \
		return b2Create##Type##Joint( world, &def ); \
	} while ( 0 )
	switch ( type )
	{
		case 0: CREATE_RECORDING_JOINT( Distance );
		case 1: CREATE_RECORDING_JOINT( Filter );
		case 2: CREATE_RECORDING_JOINT( Motor );
		case 3: CREATE_RECORDING_JOINT( Mover );
		case 4: CREATE_RECORDING_JOINT( Pogo );
		case 5: CREATE_RECORDING_JOINT( Prismatic );
		case 6: CREATE_RECORDING_JOINT( Revolute );
		case 7: CREATE_RECORDING_JOINT( Weld );
		case 8: CREATE_RECORDING_JOINT( Wheel );
		default: return b2_nullJointId;
	}
#undef CREATE_RECORDING_JOINT
}

static b2WorldId CreateRecordingJointScene( b2BodyId* a, b2BodyId* b )
{
	b2WorldDef wd = b2DefaultWorldDef();
	b2WorldId world = b2CreateWorld( &wd );
	b2BodyDef bd = b2DefaultBodyDef();
	*a = b2CreateBody( world, &bd );
	bd.type = b2_dynamicBody;
	bd.position = { 0, 2 };
	*b = b2CreateBody( world, &bd );
	b2ShapeDef sd = b2DefaultShapeDef();
	b2Circle circle = { { 0, 0 }, 0.5f };
	b2CreateCircleShape( *b, &sd, &circle );
	return world;
}

int RecordingJointCreationTest( void )
{
	int baseline = b2GetByteCount();
	b2BodyId a, b;
	b2WorldId world = CreateRecordingJointScene( &a, &b );
	b2Recording* rec = b2CreateRecording( 1 );
	b2World_StartRecording( world, rec );
	uint8_t opcodes[] = { 0x98, 0xA3, 0xA4, 0xAF, 0xB2, 0xB6, 0xC0, 0xCA, 0xCF };
	b2JointId joints[9];
	for ( int i = 0; i < 9; ++i )
	{
		int start = b2Recording_GetSize( rec );
		CHECK_RECORD( opcodes[i], joints[i] = CreateRecordingJoint( i, world, a, b ) );
		ENSURE( b2Joint_IsValid( joints[i] ) );
		const uint8_t* frame = b2Recording_GetData( rec ) + start;
		int payload = RecordingPayloadSize( frame );
		ENSURE( ReadRecordingU32( frame + 4 ) == b2StoreWorldId( world ) );
		for ( int j = 8; j < 16; ++j ) ENSURE( frame[j] == 0 );
		uint64_t bodyA = b2StoreBodyId( a ), bodyB = b2StoreBodyId( b );
		ENSURE( ReadRecordingU32( frame + 16 ) == (uint32_t)bodyA );
		ENSURE( ReadRecordingU32( frame + 20 ) == (uint32_t)( bodyA >> 32 ) );
		ENSURE( ReadRecordingU32( frame + 24 ) == (uint32_t)bodyB );
		ENSURE( ReadRecordingU32( frame + 28 ) == (uint32_t)( bodyB >> 32 ) );
		uint64_t stored = b2StoreJointId( joints[i] );
		ENSURE( ReadRecordingU32( frame + payload - 4 ) == (uint32_t)stored );
		ENSURE( ReadRecordingU32( frame + payload ) == (uint32_t)( stored >> 32 ) );
	}
	for ( b2JointId joint : joints ) { CHECK_RECORD( 0x90, b2DestroyJoint( joint ) ); }
	b2World_StopRecording( world );
	b2DestroyWorld( world );
	b2DestroyRecording( rec );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

int RecordingJointCommonTest( void )
{
	int baseline = b2GetByteCount();
	b2BodyId a, b;
	b2WorldId world = CreateRecordingJointScene( &a, &b );
	b2JointId joint = CreateRecordingJoint( 0, world, a, b );
	b2Body_SetAwake( b, false );
	ENSURE( !b2Body_IsAwake( b ) );
	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( world, rec );
	b2Transform frame = { { 1, 0 }, b2Rot_identity };
	CHECK_RECORD( 0x91, b2Joint_SetLocalFrameA( joint, frame ) );
	frame.p = { 0, 1 };
	CHECK_RECORD( 0x92, b2Joint_SetLocalFrameB( joint, frame ) );
	CHECK_RECORD( 0x93, b2Joint_SetCollideConnected( joint, true ) );
	CHECK_RECORD( 0x94, b2Joint_WakeBodies( joint ) );
	ENSURE( b2Body_IsAwake( b ) );
	CHECK_RECORD( 0x95, b2Joint_SetConstraintTuning( joint, 30, 0.8f ) );
	int start = b2Recording_GetSize( rec );
	CHECK_RECORD( 0x96, b2Joint_SetForceThreshold( joint, 2.0f ) );
	const uint8_t* bytes = b2Recording_GetData( rec ) + start;
	ENSURE( RecordingPayloadSize( bytes ) == 12 && ReadRecordingU32( bytes + 12 ) == 0x40000000u );
	CHECK_RECORD( 0x97, b2Joint_SetTorqueThreshold( joint, 3.0f ) );
	int size = b2Recording_GetSize( rec );
	b2Joint_WakeBodies( b2_nullJointId );
	b2World* internalWorld = b2_worlds + world.index1 - 1;
	internalWorld->locked = true;
	b2Joint_WakeBodies( joint );
	ENSURE( b2Recording_GetSize( rec ) == size );
	internalWorld->locked = false;
	b2World_StopRecording( world );
	size = b2Recording_GetSize( rec );
	b2Joint_SetForceThreshold( joint, 4.0f );
	ENSURE( b2Recording_GetSize( rec ) == size );
	b2DestroyWorld( world );
	b2DestroyRecording( rec );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

int RecordingJointTypeOperationsTest( void )
{
	int baseline = b2GetByteCount();
	b2BodyId a, b;
	b2WorldId world = CreateRecordingJointScene( &a, &b );
	b2JointId joints[9];
	for ( int i = 0; i < 9; ++i ) joints[i] = CreateRecordingJoint( i, world, a, b );
	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( world, rec );
	b2JointId d = joints[0];
	CHECK_RECORD( 0x99, b2DistanceJoint_SetRestLength( d, 2 ) );
	CHECK_RECORD( 0x9A, b2DistanceJoint_EnableSpring( d, true ) );
	CHECK_RECORD( 0x9B, b2DistanceJoint_SetSpringForceRange( d, -10, 20 ) );
	CHECK_RECORD( 0x9C, b2DistanceJoint_SetSpringHertz( d, 4 ) );
	CHECK_RECORD( 0x9D, b2DistanceJoint_SetSpringDampingRatio( d, 0.6f ) );
	CHECK_RECORD( 0x9E, b2DistanceJoint_EnableLimit( d, true ) );
	CHECK_RECORD( 0x9F, b2DistanceJoint_SetLimits( d, 1, 3 ) );
	CHECK_RECORD( 0xA0, b2DistanceJoint_EnableMotor( d, true ) );
	CHECK_RECORD( 0xA1, b2DistanceJoint_SetTargetMotorSpeed( d, 0.5f ) );
	CHECK_RECORD( 0xA2, b2DistanceJoint_SetMaxMotorForce( d, 10 ) );
	b2JointId m = joints[2];
	CHECK_RECORD( 0xA5, b2MotorJoint_SetLinearVelocity( m, ( b2Vec2{ 1, 2 } ) ) );
	CHECK_RECORD( 0xA6, b2MotorJoint_SetAngularVelocity( m, 0.5f ) );
	CHECK_RECORD( 0xA7, b2MotorJoint_SetMaxVelocityForce( m, 10 ) );
	CHECK_RECORD( 0xA8, b2MotorJoint_SetMaxVelocityTorque( m, 10 ) );
	CHECK_RECORD( 0xA9, b2MotorJoint_SetLinearHertz( m, 4 ) );
	CHECK_RECORD( 0xAA, b2MotorJoint_SetLinearDampingRatio( m, 0.6f ) );
	CHECK_RECORD( 0xAB, b2MotorJoint_SetAngularHertz( m, 4 ) );
	CHECK_RECORD( 0xAC, b2MotorJoint_SetAngularDampingRatio( m, 0.6f ) );
	CHECK_RECORD( 0xAD, b2MotorJoint_SetMaxSpringForce( m, 10 ) );
	CHECK_RECORD( 0xAE, b2MotorJoint_SetMaxSpringTorque( m, 10 ) );
	m = joints[3];
	CHECK_RECORD( 0xB0, b2MoverJoint_SetLinearVelocity( m, ( b2Vec2{ 1, 2 } ) ) );
	CHECK_RECORD( 0xB1, b2MoverJoint_SetMaxVelocityForce( m, ( b2Vec2{ 10, 20 } ) ) );
	m = joints[4];
	CHECK_RECORD( 0xB3, b2PogoJoint_SetRestLength( m, 2 ) );
	CHECK_RECORD( 0xB4, b2PogoJoint_SetSpringHertz( m, 4 ) );
	CHECK_RECORD( 0xB5, b2PogoJoint_SetSpringDampingRatio( m, 0.6f ) );
	m = joints[5];
	CHECK_RECORD( 0xB7, b2PrismaticJoint_EnableSpring( m, true ) );
	CHECK_RECORD( 0xB8, b2PrismaticJoint_SetSpringHertz( m, 4 ) );
	CHECK_RECORD( 0xB9, b2PrismaticJoint_SetSpringDampingRatio( m, 0.6f ) );
	CHECK_RECORD( 0xBA, b2PrismaticJoint_SetTargetTranslation( m, 1 ) );
	CHECK_RECORD( 0xBB, b2PrismaticJoint_EnableLimit( m, true ) );
	CHECK_RECORD( 0xBC, b2PrismaticJoint_SetLimits( m, -1, 2 ) );
	CHECK_RECORD( 0xBD, b2PrismaticJoint_EnableMotor( m, true ) );
	CHECK_RECORD( 0xBE, b2PrismaticJoint_SetTargetMotorSpeed( m, 0.5f ) );
	CHECK_RECORD( 0xBF, b2PrismaticJoint_SetMaxMotorForce( m, 10 ) );
	m = joints[6];
	CHECK_RECORD( 0xC1, b2RevoluteJoint_EnableSpring( m, true ) );
	CHECK_RECORD( 0xC2, b2RevoluteJoint_SetSpringHertz( m, 4 ) );
	CHECK_RECORD( 0xC3, b2RevoluteJoint_SetSpringDampingRatio( m, 0.6f ) );
	CHECK_RECORD( 0xC4, b2RevoluteJoint_SetTargetAngle( m, 0.2f ) );
	CHECK_RECORD( 0xC5, b2RevoluteJoint_EnableLimit( m, true ) );
	CHECK_RECORD( 0xC6, b2RevoluteJoint_SetLimits( m, -1, 1 ) );
	CHECK_RECORD( 0xC7, b2RevoluteJoint_EnableMotor( m, true ) );
	CHECK_RECORD( 0xC8, b2RevoluteJoint_SetTargetMotorSpeed( m, 0.5f ) );
	CHECK_RECORD( 0xC9, b2RevoluteJoint_SetMaxMotorTorque( m, 10 ) );
	m = joints[7];
	CHECK_RECORD( 0xCB, b2WeldJoint_SetLinearHertz( m, 4 ) );
	CHECK_RECORD( 0xCC, b2WeldJoint_SetLinearDampingRatio( m, 0.6f ) );
	CHECK_RECORD( 0xCD, b2WeldJoint_SetAngularHertz( m, 4 ) );
	CHECK_RECORD( 0xCE, b2WeldJoint_SetAngularDampingRatio( m, 0.6f ) );
	m = joints[8];
	CHECK_RECORD( 0xD0, b2WheelJoint_EnableSpring( m, true ) );
	CHECK_RECORD( 0xD1, b2WheelJoint_SetSpringHertz( m, 4 ) );
	CHECK_RECORD( 0xD2, b2WheelJoint_SetSpringDampingRatio( m, 0.6f ) );
	CHECK_RECORD( 0xD3, b2WheelJoint_EnableLimit( m, true ) );
	CHECK_RECORD( 0xD4, b2WheelJoint_SetLimits( m, -1, 2 ) );
	CHECK_RECORD( 0xD5, b2WheelJoint_EnableMotor( m, true ) );
	CHECK_RECORD( 0xD6, b2WheelJoint_SetTargetMotorSpeed( m, 0.5f ) );
	CHECK_RECORD( 0xD7, b2WheelJoint_SetMaxMotorTorque( m, 10 ) );
	b2World_StopRecording( world );
	b2DestroyWorld( world );
	b2DestroyRecording( rec );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

int RecordingJointCascadeTest( void )
{
	int baseline = b2GetByteCount();
	b2BodyId a, b;
	b2WorldId world = CreateRecordingJointScene( &a, &b );
	b2JointId joints[9];
	for ( int i = 0; i < 9; ++i ) joints[i] = CreateRecordingJoint( i, world, a, b );
	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( world, rec );
	CHECK_RECORD( 0x11, b2DestroyBody( b ) );
	for ( b2JointId joint : joints ) ENSURE( !b2Joint_IsValid( joint ) );
	b2World_StopRecording( world );
	b2DestroyWorld( world );
	b2DestroyRecording( rec );
	ENSURE( b2GetByteCount() == baseline );
	return 0;
}

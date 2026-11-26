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

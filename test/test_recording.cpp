// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "recording.h"

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

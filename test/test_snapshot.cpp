// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "world.h"

#include "box2d/types.h"

#include <stdlib.h>
#include <string.h>

typedef struct SnapshotImageHeader
{
	uint32_t magic;
	uint32_t version;
	uint32_t layoutHash;
	uint32_t flags;
} SnapshotImageHeader;

#define SNAPSHOT_MAGIC 0x32534E42u
#define SNAPSHOT_VERSION 13u

int WorldSnapshotWriteTest( void )
{
	b2WorldDef def = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &def );
	ENSURE( b2World_IsValid( worldId ) );

	int size = b2World_GetSnapshot( worldId, NULL, 0 );
	ENSURE( size > (int)sizeof( SnapshotImageHeader ) );

	uint8_t* image = (uint8_t*)malloc( (size_t)size );
	ENSURE( image != NULL );
	memset( image, 0xA5, (size_t)size );

	int undersized = b2World_GetSnapshot( worldId, image, size - 1 );
	ENSURE( undersized == size );
	for ( int i = 0; i < size; ++i )
	{
		ENSURE( image[i] == 0xA5 );
	}

	int written = b2World_GetSnapshot( worldId, image, size );
	ENSURE( written == size );

	SnapshotImageHeader header;
	memcpy( &header, image, sizeof( header ) );
	ENSURE( header.magic == SNAPSHOT_MAGIC );
	ENSURE( header.version == SNAPSHOT_VERSION );
	ENSURE( header.layoutHash != 0 );

	uint8_t* again = (uint8_t*)malloc( (size_t)size );
	ENSURE( again != NULL );
	ENSURE( b2World_GetSnapshot( worldId, again, size ) == size );
	ENSURE( memcmp( image, again, (size_t)size ) == 0 );

	free( again );
	free( image );
	b2DestroyWorld( worldId );
	return 0;
}

static b2WorldId BuildSnapshotWriteScene( void* userData )
{
	b2WorldDef def = b2DefaultWorldDef();
	def.workerCount = 1;
	b2WorldId worldId = b2CreateWorld( &def );
	b2World_SetUserData( worldId, userData );

	b2ShapeDef shapeDef = b2DefaultShapeDef();

	b2BodyDef groundDef = b2DefaultBodyDef();
	groundDef.position = b2Vec2{ 0.0f, -1.0f };
	b2BodyId groundId = b2CreateBody( worldId, &groundDef );
	b2Body_SetUserData( groundId, userData );
	b2Polygon groundBox = b2MakeBox( 10.0f, 1.0f );
	b2ShapeId groundShape = b2CreatePolygonShape( groundId, &shapeDef, &groundBox );
	b2Shape_SetUserData( groundShape, userData );

	b2BodyId previous = b2_nullBodyId;
	for ( int i = 0; i < 3; ++i )
	{
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.type = b2_dynamicBody;
		bodyDef.position = b2Vec2{ 0.0f, 0.5f + (float)i * 1.1f };
		b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
		b2Body_SetUserData( bodyId, userData );
		b2Polygon box = b2MakeBox( 0.4f, 0.4f );
		b2ShapeId shapeId = b2CreatePolygonShape( bodyId, &shapeDef, &box );
		b2Shape_SetUserData( shapeId, userData );
		if ( i == 0 )
		{
			previous = bodyId;
		}
	}

	b2BodyDef jointBodyDef = b2DefaultBodyDef();
	jointBodyDef.type = b2_dynamicBody;
	jointBodyDef.position = b2Vec2{ 4.0f, 2.0f };
	b2BodyId jointBodyA = b2CreateBody( worldId, &jointBodyDef );
	jointBodyDef.position = b2Vec2{ 6.0f, 2.0f };
	b2BodyId jointBodyB = b2CreateBody( worldId, &jointBodyDef );

	b2DistanceJointDef jointDef = b2DefaultDistanceJointDef();
	jointDef.base.bodyIdA = jointBodyA;
	jointDef.base.bodyIdB = jointBodyB;
	jointDef.base.userData = userData;
	jointDef.length = 2.0f;
	b2CreateDistanceJoint( worldId, &jointDef );

	b2BodyDef chainBodyDef = b2DefaultBodyDef();
	chainBodyDef.position = b2Vec2{ -8.0f, 0.0f };
	b2BodyId chainBodyId = b2CreateBody( worldId, &chainBodyDef );
	b2Vec2 points[3] = { b2Vec2{ -1.0f, 0.0f }, b2Vec2{ 0.0f, 0.0f }, b2Vec2{ 1.0f, 0.0f } };
	b2ChainDef chainDef = b2DefaultChainDef();
	chainDef.userData = userData;
	chainDef.points = points;
	chainDef.pointCount = 3;
	b2CreateChain( chainBodyId, &chainDef );

	b2BodyDef sensorBodyDef = b2DefaultBodyDef();
	sensorBodyDef.position = b2Vec2{ 0.0f, 5.0f };
	b2BodyId sensorBodyId = b2CreateBody( worldId, &sensorBodyDef );
	b2ShapeDef sensorDef = b2DefaultShapeDef();
	sensorDef.isSensor = true;
	sensorDef.enableSensorEvents = true;
	b2Polygon sensorBox = b2MakeBox( 2.0f, 1.0f );
	b2CreatePolygonShape( sensorBodyId, &sensorDef, &sensorBox );

	if ( b2Body_IsValid( previous ) == false )
	{
		return b2WorldId{};
	}
	for ( int i = 0; i < 20; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	return worldId;
}

int WorldSnapshotStateWriteTest( void )
{
	int markerA = 11;
	int markerB = 22;

	b2WorldId first = BuildSnapshotWriteScene( &markerA );
	b2WorldId second = BuildSnapshotWriteScene( &markerB );

	b2WorldDef plainDef = b2DefaultWorldDef();
	b2WorldId emptyWorld = b2CreateWorld( &plainDef );
	int emptySize = b2World_GetSnapshot( emptyWorld, NULL, 0 );
	int sceneSize = b2World_GetSnapshot( second, NULL, 0 );
	ENSURE( sceneSize > emptySize );
	b2DestroyWorld( emptyWorld );

	uint8_t* firstImage = (uint8_t*)malloc( (size_t)sceneSize );
	uint8_t* secondImage = (uint8_t*)malloc( (size_t)sceneSize );
	ENSURE( firstImage != NULL && secondImage != NULL );

	ENSURE( b2World_GetSnapshot( first, firstImage, sceneSize ) == sceneSize );
	ENSURE( b2World_GetSnapshot( second, secondImage, sceneSize ) == sceneSize );
	ENSURE( memcmp( firstImage, secondImage, (size_t)sceneSize ) == 0 );

	free( secondImage );
	free( firstImage );
	b2DestroyWorld( second );
	b2DestroyWorld( first );
	return 0;
}

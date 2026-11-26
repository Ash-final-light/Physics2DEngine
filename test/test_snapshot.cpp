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

typedef struct SnapshotIds
{
	b2BodyId body;
	b2ShapeId shape;
	b2JointId joint;
	b2ChainId chain;
} SnapshotIds;

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

static b2WorldId BuildSnapshotWriteScene( void* userData, SnapshotIds* outIds = NULL )
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
	b2JointId jointId = b2CreateDistanceJoint( worldId, &jointDef );

	b2BodyDef chainBodyDef = b2DefaultBodyDef();
	chainBodyDef.position = b2Vec2{ -8.0f, 0.0f };
	b2BodyId chainBodyId = b2CreateBody( worldId, &chainBodyDef );
	b2Vec2 points[3] = { b2Vec2{ -1.0f, 0.0f }, b2Vec2{ 0.0f, 0.0f }, b2Vec2{ 1.0f, 0.0f } };
	b2SurfaceMaterial chainMaterial = b2DefaultSurfaceMaterial();
	b2ChainDef chainDef = b2DefaultChainDef();
	chainDef.userData = userData;
	chainDef.points = points;
	chainDef.pointCount = 3;
	chainDef.materials = &chainMaterial;
	chainDef.materialCount = 1;
	chainDef.ghost1 = b2Vec2{ -2.0f, 0.0f };
	chainDef.ghost2 = b2Vec2{ 2.0f, 0.0f };
	b2ChainId chainId = b2CreateChain( chainBodyId, &chainDef );

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

	if ( outIds != NULL )
	{
		outIds->body = previous;
		outIds->shape = groundShape;
		outIds->joint = jointId;
		outIds->chain = chainId;
	}

	return worldId;
}

int WorldSnapshotStateWriteTest( void )
{
	int markerA = 11;
	int markerB = 22;
	SnapshotIds ids = {};
	b2WorldId worldId = BuildSnapshotWriteScene( &markerA, &ids );
	ENSURE( b2World_IsValid( worldId ) );

	b2WorldDef plainDef = b2DefaultWorldDef();
	b2WorldId emptyWorld = b2CreateWorld( &plainDef );
	int emptySize = b2World_GetSnapshot( emptyWorld, NULL, 0 );
	int sceneSize = b2World_GetSnapshot( worldId, NULL, 0 );
	ENSURE( sceneSize > emptySize );
	b2DestroyWorld( emptyWorld );

	uint8_t* firstImage = (uint8_t*)malloc( (size_t)sceneSize );
	uint8_t* secondImage = (uint8_t*)malloc( (size_t)sceneSize );
	ENSURE( firstImage != NULL && secondImage != NULL );

	ENSURE( b2World_GetSnapshot( worldId, firstImage, sceneSize ) == sceneSize );

	b2World_SetUserData( worldId, &markerB );
	b2Body_SetUserData( ids.body, &markerB );
	b2Shape_SetUserData( ids.shape, &markerB );
	b2Joint_SetUserData( ids.joint, &markerB );

	ENSURE( b2World_GetSnapshot( worldId, secondImage, sceneSize ) == sceneSize );
	ENSURE( memcmp( firstImage, secondImage, (size_t)sceneSize ) == 0 );

	free( secondImage );
	free( firstImage );
	b2DestroyWorld( worldId );
	return 0;
}

int WorldSnapshotCreateTest( void )
{
	int marker = 7;
	SnapshotIds ids = {};
	b2WorldId source = BuildSnapshotWriteScene( &marker, &ids );
	ENSURE( b2World_IsValid( source ) );

	int size = b2World_GetSnapshot( source, NULL, 0 );
	uint8_t* image = (uint8_t*)malloc( (size_t)size );
	ENSURE( image != NULL );
	ENSURE( b2World_GetSnapshot( source, image, size ) == size );
	uint64_t sourceHash = b2World_GetStateHash( source );

	b2WorldId loaded = b2CreateWorldFromSnapshot( image, size, 1 );
	ENSURE( b2World_IsValid( loaded ) );
	ENSURE( b2World_GetWorkerCount( loaded ) == 1 );

	b2BodyId loadedBody = ids.body;
	b2ShapeId loadedShape = ids.shape;
	b2JointId loadedJoint = ids.joint;
	b2ChainId loadedChain = ids.chain;
	loadedBody.world0 = (uint16_t)( loaded.index1 - 1 );
	loadedShape.world0 = (uint16_t)( loaded.index1 - 1 );
	loadedJoint.world0 = (uint16_t)( loaded.index1 - 1 );
	loadedChain.world0 = (uint16_t)( loaded.index1 - 1 );

	ENSURE( b2Body_IsValid( loadedBody ) );
	ENSURE( b2Shape_IsValid( loadedShape ) );
	ENSURE( b2Joint_IsValid( loadedJoint ) );
	ENSURE( b2Chain_IsValid( loadedChain ) );
	ENSURE( b2Body_GetWorld( loadedBody ).index1 == loaded.index1 );
	ENSURE( b2Shape_GetWorld( loadedShape ).index1 == loaded.index1 );
	ENSURE( b2Joint_GetWorld( loadedJoint ).index1 == loaded.index1 );
	ENSURE( b2Chain_GetWorld( loadedChain ).index1 == loaded.index1 );

	ENSURE( b2World_GetUserData( loaded ) == NULL );
	ENSURE( b2Body_GetUserData( loadedBody ) == NULL );
	ENSURE( b2Shape_GetUserData( loadedShape ) == NULL );
	ENSURE( b2Joint_GetUserData( loadedJoint ) == NULL );
	ENSURE( b2World_GetStateHash( loaded ) == sourceHash );

	b2WorldId truncated = b2CreateWorldFromSnapshot( image, size - 1, 1 );
	ENSURE( b2World_IsValid( truncated ) == false );

	image[0] ^= 0xFFu;
	b2WorldId badMagic = b2CreateWorldFromSnapshot( image, size, 1 );
	ENSURE( b2World_IsValid( badMagic ) == false );

	free( image );
	b2DestroyWorld( loaded );
	b2DestroyWorld( source );
	return 0;
}

int WorldSnapshotContinueTest( void )
{
	int marker = 9;
	b2WorldId source = BuildSnapshotWriteScene( &marker, NULL );
	ENSURE( b2World_IsValid( source ) );

	int size = b2World_GetSnapshot( source, NULL, 0 );
	uint8_t* image = (uint8_t*)malloc( (size_t)size );
	ENSURE( image != NULL );
	ENSURE( b2World_GetSnapshot( source, image, size ) == size );

	b2WorldId loaded = b2CreateWorldFromSnapshot( image, size, 1 );
	ENSURE( b2World_IsValid( loaded ) );

	for ( int i = 0; i < 12; ++i )
	{
		b2World_Step( source, 1.0f / 60.0f, 4 );
		b2World_Step( loaded, 1.0f / 60.0f, 4 );
		ENSURE( b2World_GetStateHash( source ) == b2World_GetStateHash( loaded ) );
	}

	free( image );
	b2DestroyWorld( loaded );
	b2DestroyWorld( source );
	return 0;
}

static bool SnapshotFilter(b2ShapeId,b2ShapeId,void*) { return true; }
static void SnapshotPreSolve(b2ShapeId,b2ShapeId,b2Manifold*,void*) {}
int WorldSnapshotRestoreTest(void)
{
 size_t before=b2GetByteCount();int marker=17;SnapshotIds ids={};
 b2WorldId id=BuildSnapshotWriteScene(&marker,&ids);b2World_SetWorkerCount(id,4);
 b2World* w=b2_worlds+id.index1-1;
 b2World_SetCustomFilterCallback(id,SnapshotFilter,&marker);
 b2World_SetPreSolveCallback(id,SnapshotPreSolve,nullptr,&marker);
 int size=b2World_GetSnapshot(id,nullptr,0);auto image=(uint8_t*)malloc(size);ENSURE(image!=nullptr);
 ENSURE(b2World_GetSnapshot(id,image,size)==size);uint64_t hash=b2World_GetStateHash(id);
 b2WorldId expected=b2CreateWorldFromSnapshot(image,size,1);ENSURE(b2World_IsValid(expected));
 for(int pass=0;pass<3;++pass)
 {
  b2World_Step(id,1.0f/60,4);b2BodyDef bd=b2DefaultBodyDef();b2BodyId extra=b2CreateBody(id,&bd);
  ENSURE(b2World_RestoreFromSnapshot(id,image,size));
  ENSURE(b2World_GetStateHash(id)==hash && b2World_IsValid(id));
  ENSURE(b2Body_IsValid(ids.body) && b2Shape_IsValid(ids.shape) && b2Joint_IsValid(ids.joint) && b2Chain_IsValid(ids.chain));
  ENSURE(!b2Body_IsValid(extra));ENSURE(b2World_GetUserData(id)==&marker && b2World_GetWorkerCount(id)==4);
  ENSURE(w->customFilterFcn==SnapshotFilter && w->customFilterContext==&marker);
  ENSURE(w->preSolveFcn==SnapshotPreSolve && w->preSolveContext==&marker);
  ENSURE(b2Body_GetUserData(ids.body)==nullptr && b2Shape_GetUserData(ids.shape)==nullptr && b2Joint_GetUserData(ids.joint)==nullptr);
 }
 for(int i=0;i<8;++i)
 {
  b2World_Step(id,1.0f/60,4);b2World_Step(expected,1.0f/60,4);
  ENSURE(b2World_GetStateHash(id)==b2World_GetStateHash(expected));
 }
 free(image);b2DestroyWorld(id);b2DestroyWorld(expected);ENSURE(b2GetByteCount()==before);return 0;
}
int WorldSnapshotRestoreInvalidTest(void)
{
 size_t before=b2GetByteCount();b2WorldId id=BuildSnapshotWriteScene(nullptr);
 int size=b2World_GetSnapshot(id,nullptr,0);auto image=(uint8_t*)malloc(size);ENSURE(image!=nullptr);
 b2World_GetSnapshot(id,image,size);uint64_t hash=b2World_GetStateHash(id);
 ENSURE(!b2World_RestoreFromSnapshot(id,nullptr,0));
 image[0]^=255;ENSURE(!b2World_RestoreFromSnapshot(id,image,size));image[0]^=255;
 b2World* w=b2_worlds+id.index1-1;w->locked=true;
 ENSURE(!b2World_RestoreFromSnapshot(id,image,size));w->locked=false;
 ENSURE(b2World_GetStateHash(id)==hash);
 free(image);b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

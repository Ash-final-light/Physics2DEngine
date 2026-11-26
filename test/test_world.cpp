// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "world.h"
#include <stdio.h>
#include <string.h>

#include "box2d/base.h"
#include "box2d/math_functions.h"
#include "box2d/types.h"

#include <math.h>
#include <string.h>

int WorldCreateTest( void )
{
	size_t before = b2GetByteCount();
	b2WorldDef def = b2DefaultWorldDef();
	ENSURE( def.gravity.y == -10.0f );
	b2WorldId worldId = b2CreateWorld( &def );
	ENSURE( worldId.index1 == 1 );
	ENSURE( b2GetByteCount() > before );
	b2DestroyWorld( worldId );
	ENSURE( b2GetByteCount() == before );

	def.gravity.y = NAN;
	b2WorldId invalid = b2CreateWorld( &def );
	ENSURE( invalid.index1 == 0 );
	ENSURE( b2GetByteCount() == before );

	def = b2DefaultWorldDef();
	b2WorldId first = b2CreateWorld( &def );
	b2WorldId second = b2CreateWorld( &def );
	ENSURE( first.index1 != 0 && second.index1 != 0 && first.index1 != second.index1 );
	b2DestroyWorld( second );
	b2DestroyWorld( first );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

int WorldIdTest( void )
{
	b2WorldId empty = {};
	ENSURE( b2World_IsValid( empty ) == false );
	b2WorldId tooFar = { 200, 0 };
	ENSURE( b2World_IsValid( tooFar ) == false );

	b2WorldDef def = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &def );
	ENSURE( b2World_IsValid( worldId ) );
	uint16_t generation = worldId.generation;
	b2DestroyWorld( worldId );
	ENSURE( b2World_IsValid( worldId ) == false );

	b2WorldId reused = b2CreateWorld( &def );
	ENSURE( reused.index1 == worldId.index1 );
	ENSURE( reused.generation != generation );
	ENSURE( b2World_IsValid( reused ) );
	ENSURE( b2World_IsValid( worldId ) == false );
	b2WorldId wrongGeneration = reused;
	wrongGeneration.generation = (uint16_t)( reused.generation + 1 );
	ENSURE( b2World_IsValid( wrongGeneration ) == false );
	b2DestroyWorld( worldId );
	ENSURE( b2World_IsValid( reused ) );
	b2DestroyWorld( reused );
	ENSURE( b2World_IsValid( reused ) == false );
	return 0;
}

int WorldUserDataTest( void )
{
	int marker = 7;
	int other = 9;
	b2WorldDef def = b2DefaultWorldDef();
	def.userData = &marker;
	b2WorldId worldId = b2CreateWorld( &def );
	ENSURE( b2World_GetUserData( worldId ) == &marker );
	b2World_SetUserData( worldId, &other );
	ENSURE( b2World_GetUserData( worldId ) == &other );

	b2WorldDef plain = b2DefaultWorldDef();
	b2WorldId second = b2CreateWorld( &plain );
	ENSURE( b2World_GetUserData( second ) == NULL );
	b2DestroyWorld( worldId );
	ENSURE( b2World_GetUserData( worldId ) == NULL );
	ENSURE( b2World_GetUserData( second ) == NULL );
	b2DestroyWorld( second );
	return 0;
}

int BodyCreateTest( void )
{
	size_t before = b2GetByteCount();
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	ENSURE( bodyDef.type == b2_staticBody );
	ENSURE( bodyDef.gravityScale == 1.0f );
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	ENSURE( bodyId.index1 == 1 );
	ENSURE( bodyId.world0 == 0 );
	ENSURE( bodyId.generation == 1 );

	b2BodyDef bad = bodyDef;
	bad.position.x = NAN;
	b2BodyId invalid = b2CreateBody( worldId, &bad );
	ENSURE( invalid.index1 == 0 );

	b2BodyId second = b2CreateBody( worldId, &bodyDef );
	ENSURE( second.index1 == 2 );
	b2DestroyBody( bodyId );
	b2DestroyBody( second );
	b2DestroyWorld( worldId );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

int BodyIdTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsValid( bodyId ) );
	ENSURE( b2Body_IsValid( b2_nullBodyId ) == false );
	uint16_t generation = bodyId.generation;
	b2DestroyBody( bodyId );
	ENSURE( b2Body_IsValid( bodyId ) == false );

	b2BodyId reused = b2CreateBody( worldId, &bodyDef );
	ENSURE( reused.index1 == bodyId.index1 );
	ENSURE( reused.generation != generation );
	ENSURE( b2Body_IsValid( reused ) );
	ENSURE( b2Body_IsValid( bodyId ) == false );
	b2DestroyBody( bodyId );
	ENSURE( b2Body_IsValid( reused ) );
	b2DestroyWorld( worldId );
	ENSURE( b2Body_IsValid( reused ) == false );
	return 0;
}

int BodyNameTest( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.name = "box";
	int marker = 3;
	bodyDef.userData = &marker;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	ENSURE( strcmp( b2Body_GetName( bodyId ), "box" ) == 0 );
	ENSURE( b2Body_GetUserData( bodyId ) == &marker );

	b2Body_SetName( bodyId, "0123456789ABC" );
	ENSURE( strcmp( b2Body_GetName( bodyId ), "0123456789" ) == 0 );
	b2Body_SetName( bodyId, NULL );
	ENSURE( strcmp( b2Body_GetName( bodyId ), "" ) == 0 );

	int other = 4;
	b2BodyDef plain = b2DefaultBodyDef();
	b2BodyId second = b2CreateBody( worldId, &plain );
	b2Body_SetUserData( bodyId, &other );
	ENSURE( b2Body_GetUserData( bodyId ) == &other );
	ENSURE( b2Body_GetUserData( second ) == NULL );
	b2DestroyBody( bodyId );
	ENSURE( b2Body_GetUserData( bodyId ) == NULL );
	ENSURE( strcmp( b2Body_GetName( bodyId ), "" ) == 0 );
	b2DestroyBody( second );
	b2DestroyWorld( worldId );
	return 0;
}

int BodyTransformQueryTest( void )
{
	size_t before = b2GetByteCount();
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2BodyDef def = b2DefaultBodyDef();
	def.position = { 5.0f, -3.0f };
	def.rotation = { 0.0f, 1.0f };
	b2BodyId body = b2CreateBody( worldId, &def );
	b2Pos position = b2Body_GetPosition( body );
	b2Rot rotation = b2Body_GetRotation( body );
	b2WorldTransform transform = b2Body_GetTransform( body );
	ENSURE( position.x == 5.0f && position.y == -3.0f );
	ENSURE( rotation.c == 0.0f && rotation.s == 1.0f );
	ENSURE( transform.p.x == position.x && transform.p.y == position.y );
	ENSURE( transform.q.c == rotation.c && transform.q.s == rotation.s );

	b2Vec2 localPoint = { 2.0f, 4.0f };
	b2Pos worldPoint = b2Body_GetWorldPoint( body, localPoint );
	ENSURE_SMALL( worldPoint.x - 1.0f, 0.00001f );
	ENSURE_SMALL( worldPoint.y + 1.0f, 0.00001f );
	b2Vec2 restored = b2Body_GetLocalPoint( body, worldPoint );
	ENSURE_SMALL( restored.x - localPoint.x, 0.00001f );
	ENSURE_SMALL( restored.y - localPoint.y, 0.00001f );

	b2Vec2 worldVector = b2Body_GetWorldVector( body, localPoint );
	ENSURE( worldVector.x == -4.0f && worldVector.y == 2.0f );
	b2Vec2 localVector = b2Body_GetLocalVector( body, worldVector );
	ENSURE( localVector.x == localPoint.x && localVector.y == localPoint.y );

	def.position = { -100.0f, 40.0f };
	b2BodyId other = b2CreateBody( worldId, &def );
	b2Vec2 otherVector = b2Body_GetWorldVector( other, localPoint );
	ENSURE( otherVector.x == worldVector.x && otherVector.y == worldVector.y );
	ENSURE( b2Body_GetPosition( body ).x == 5.0f );
	ENSURE( b2Body_GetPosition( other ).x == -100.0f );

	b2BodyDef identity = b2DefaultBodyDef();
	b2BodyId origin = b2CreateBody( worldId, &identity );
	worldPoint = b2Body_GetWorldPoint( origin, localPoint );
	ENSURE( worldPoint.x == localPoint.x && worldPoint.y == localPoint.y );
	b2DestroyWorld( worldId );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

int BodySetTransformTest( void )
{
	size_t before = b2GetByteCount();
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId world = b2CreateWorld( &worldDef );
	b2BodyDef def = b2DefaultBodyDef();
	b2BodyId body = b2CreateBody( world, &def );
	b2BodyId other = b2CreateBody( world, &def );
	b2Body_SetTransform( body, { 7.0f, -2.0f }, { 0.0f, 1.0f } );
	b2WorldTransform transform = b2Body_GetTransform( body );
	ENSURE( transform.p.x == 7.0f && transform.p.y == -2.0f );
	ENSURE( transform.q.c == 0.0f && transform.q.s == 1.0f );
	b2Pos point = b2Body_GetWorldPoint( body, { 3.0f, 1.0f } );
	ENSURE( point.x == 6.0f && point.y == 1.0f );
	ENSURE( b2Body_GetPosition( other ).x == 0.0f );

	b2Body_SetTransform( body, { NAN, 4.0f }, b2Rot_identity );
	b2Body_SetTransform( body, { 1.0f, 2.0f }, { 0.0f, 0.0f } );
	b2Body_SetTransform( body, { 1.0f, 2.0f }, { INFINITY, 0.0f } );
	ENSURE( b2Body_GetPosition( body ).x == 7.0f );
	ENSURE( b2Body_GetRotation( body ).s == 1.0f );

	b2DestroyBody( body );
	b2BodyId replacement = b2CreateBody( world, &def );
	b2Body_SetTransform( body, { 9.0f, 9.0f }, b2Rot_identity );
	ENSURE( b2Body_GetPosition( replacement ).x == 0.0f );
	b2Body_SetTransform( b2_nullBodyId, { 0.0f, 0.0f }, b2Rot_identity );
	b2DestroyWorld( world );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

int BodyBoundsTest( void )
{
	size_t before = b2GetByteCount();
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId world = b2CreateWorld( &worldDef );
	b2BodyDef def = b2DefaultBodyDef();
	b2BodyId body = b2CreateBody( world, &def );
	b2AABB bounds = b2Body_ComputeBounds( body );
	ENSURE( bounds.lowerBound.x == 0.0f && bounds.upperBound.y == 0.0f );
	b2Body_SetTransform( body, { 25.5f, -17.25f }, { 0.0f, 1.0f } );
	bounds = b2Body_ComputeBounds( body );
	ENSURE( bounds.lowerBound.x == 25.5f && bounds.upperBound.x == 25.5f );
	ENSURE( bounds.lowerBound.y == -17.25f && bounds.upperBound.y == -17.25f );
	b2Body_SetTransform( body, { -250000.0f, 300000.0f }, b2Rot_identity );
	bounds = b2Body_ComputeBounds( body );
	ENSURE( bounds.lowerBound.x == -250000.0f && bounds.upperBound.y == 300000.0f );
	b2DestroyBody( body );
	bounds = b2Body_ComputeBounds( body );
	ENSURE( bounds.lowerBound.x == 0.0f && bounds.upperBound.x == 0.0f );
	b2DestroyWorld( world );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

int WorldCountersTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId id=b2CreateWorld(&wd);b2Counters counts=b2World_GetCounters(id);
    ENSURE(counts.bodyCount==0 && counts.shapeCount==0 && counts.contactCount==0 && counts.jointCount==0 && counts.islandCount==0);
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};
    b2BodyId a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);bd.position={20,0};b2BodyId b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    bd.type=b2_kinematicBody;bd.position={40,0};b2BodyId k=b2CreateBody(id,&bd);b2CreateCircleShape(k,&sd,&c);
    bd.type=b2_dynamicBody;bd.isEnabled=false;bd.position={60,0};b2BodyId disabled=b2CreateBody(id,&bd);b2CreateCircleShape(disabled,&sd,&c);
    bd.type=b2_staticBody;bd.isEnabled=true;bd.position={0,-2};b2BodyId ground=b2CreateBody(id,&bd);b2CreateCircleShape(ground,&sd,&c);
    b2World_Step(id,0.001f,1);b2World_Step(id,0.001f,1);counts=b2World_GetCounters(id);
    ENSURE(counts.bodyCount==5 && counts.shapeCount==5 && counts.contactCount==1 && counts.islandCount==3);
    ENSURE(counts.awakeContactCount==1 && counts.recycledContactCount==1 && counts.taskCount==0 && counts.enqueueCount==0);
    ENSURE(counts.byteCount==b2GetByteCount() && counts.stackUsed>0);
    int colors=0;for(int i=0;i<24;++i) { colors+=counts.colorCounts[i]; }ENSURE(colors==1);
    b2Capacity capacity=b2World_GetMaxCapacity(id);
    ENSURE(capacity.staticBodyCount==1 && capacity.dynamicBodyCount==4 && capacity.staticShapeCount==1 && capacity.dynamicShapeCount==2);
    ENSURE(capacity.contactCount==1 && capacity.arenaByteCount==counts.stackUsed);
    b2Body_SetAwake(a,false);counts=b2World_GetCounters(id);colors=0;for(int i=0;i<24;++i) { colors+=counts.colorCounts[i]; }
    ENSURE(counts.contactCount==1 && counts.awakeContactCount==0 && colors==0);
    b2DestroyBody(b);b2World_Step(id,0.001f,1);counts=b2World_GetCounters(id);ENSURE(counts.bodyCount==4 && counts.shapeCount==4);
    capacity=b2World_GetMaxCapacity(id);ENSURE(capacity.dynamicBodyCount==4 && capacity.dynamicShapeCount==2 && capacity.contactCount==1);
    b2DestroyBody(ground);counts=b2World_GetCounters(id);ENSURE(counts.bodyCount==3 && counts.shapeCount==3 && counts.contactCount==0);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int WorldProfileMemoryTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;b2Profile initial=b2World_GetProfile(id);ENSURE(initial.step==0);
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};
    b2BodyId a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);bd.type=b2_staticBody;bd.position={0,-2};b2BodyId b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    b2World_Step(id,1.0f/60.0f,4);b2Profile profile=b2World_GetProfile(id);
    ENSURE(b2IsValidFloat(profile.step) && profile.step>=0);
    ENSURE(profile.pairs>=0 && profile.collide>=0 && profile.solve>=0 && profile.prepareConstraints>=0);
    ENSURE(profile.integrateVelocities>=0 && profile.warmStart>=0 && profile.solveImpulses>=0 && profile.integratePositions>=0 && profile.relaxImpulses>=0);
    ENSURE(profile.step+0.001f>=profile.pairs+profile.collide+profile.solve);
    ENSURE(profile.sensors==0 && profile.jointEvents==0 && profile.hitEvents==0 && profile.bullets==0 && profile.restitution==0);
    world->profile.sensorHits=123;b2World_Step(id,1.0f/60.0f,1);ENSURE(b2World_GetProfile(id).sensorHits==0);
    size_t bytes=b2GetByteCount();b2World_DumpMemoryStats(id);ENSURE(b2GetByteCount()==bytes);
    FILE* file=fopen("box2d_memory.txt","r");ENSURE(file!=NULL);
    char content[8192];size_t length=fread(content,1,sizeof(content)-1,file);content[length]=0;fclose(file);
    ENSURE(strstr(content,"id pools")!=NULL && strstr(content,"broad-phase")!=NULL && strstr(content,"constraint graph")!=NULL);
    char expected[80];snprintf(expected,sizeof(expected),"bodies: %d",b2Array_ByteCount(world->bodies));ENSURE(strstr(content,expected)!=NULL);
    char* total=strstr(content,"total: ");ENSURE(total!=NULL);int reported=0;int parsed=sscanf(total,"total: %d",&reported);ENSURE(parsed==1);
    ENSURE(reported>0 && size_t(reported)<=bytes-before);
    ENSURE(remove("box2d_memory.txt")==0);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

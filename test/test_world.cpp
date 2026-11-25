// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/base.h"
#include "box2d/types.h"

#include <math.h>

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

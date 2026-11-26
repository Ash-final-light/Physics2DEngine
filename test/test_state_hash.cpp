// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "state_hash.h"
#include "world.h"

#include "box2d/types.h"

int WorldStateHashTest( void )
{
	b2WorldDef def = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &def );
	ENSURE( b2World_IsValid( worldId ) );
	b2World* world = b2_worlds + worldId.index1 - 1;

	uint64_t empty = b2HashWorldState( world );
	ENSURE( empty == B2_SNAP_FNV_INIT );
	ENSURE( b2HashWorldState( world ) == empty );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = b2Vec2{ 1.0f, 2.0f };
	bodyDef.gravityScale = 0.0f;
	b2BodyId bodyId = b2CreateBody( worldId, &bodyDef );
	ENSURE( b2Body_IsValid( bodyId ) );
	b2Body_SetLinearVelocity( bodyId, b2Vec2{ 3.0f, 4.0f } );
	b2Body_SetAngularVelocity( bodyId, 0.5f );

	uint64_t base = b2HashWorldState( world );
	ENSURE( base != empty );
	ENSURE( b2HashWorldState( world ) == base );

	b2Body_SetTransform( bodyId, b2Vec2{ 5.0f, 2.0f }, b2Rot_identity );
	uint64_t moved = b2HashWorldState( world );
	ENSURE( moved != base );

	b2Body_SetTransform( bodyId, b2Vec2{ 1.0f, 2.0f }, b2Rot{ 0.96891242f, 0.24740396f } );
	uint64_t rotated = b2HashWorldState( world );
	ENSURE( rotated != base );
	ENSURE( rotated != moved );

	b2Body_SetTransform( bodyId, b2Vec2{ 1.0f, 2.0f }, b2Rot_identity );
	b2Body_SetLinearVelocity( bodyId, b2Vec2{ 3.0f, 5.0f } );
	uint64_t linear = b2HashWorldState( world );
	ENSURE( linear != base );

	b2Body_SetLinearVelocity( bodyId, b2Vec2{ 3.0f, 4.0f } );
	b2Body_SetAngularVelocity( bodyId, 0.75f );
	uint64_t angular = b2HashWorldState( world );
	ENSURE( angular != base );

	b2DestroyWorld( worldId );
	return 0;
}

int WorldStateHashFreeSlotTest( void )
{
	b2WorldDef def = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &def );

	b2BodyDef staticDef = b2DefaultBodyDef();
	staticDef.type = b2_staticBody;
	staticDef.position = b2Vec2{ 0.0f, 0.0f };
	b2BodyId staticId = b2CreateBody( worldId, &staticDef );
	ENSURE( b2Body_IsValid( staticId ) );

	b2BodyDef dynamicDef = b2DefaultBodyDef();
	dynamicDef.type = b2_dynamicBody;
	dynamicDef.position = b2Vec2{ 1.0f, 2.0f };
	dynamicDef.gravityScale = 0.0f;
	b2BodyId dynamicId = b2CreateBody( worldId, &dynamicDef );
	ENSURE( b2Body_IsValid( dynamicId ) );
	b2Body_SetLinearVelocity( dynamicId, b2Vec2{ 3.0f, 4.0f } );
	b2Body_SetAngularVelocity( dynamicId, 0.5f );

	uint64_t both = b2HashWorldState( b2_worlds + worldId.index1 - 1 );
	b2DestroyBody( dynamicId );
	uint64_t afterFree = b2HashWorldState( b2_worlds + worldId.index1 - 1 );
	ENSURE( afterFree != both );
	b2DestroyWorld( worldId );

	b2WorldId onlyId = b2CreateWorld( &def );
	b2BodyId onlyStatic = b2CreateBody( onlyId, &staticDef );
	ENSURE( b2Body_IsValid( onlyStatic ) );
	uint64_t only = b2HashWorldState( b2_worlds + onlyId.index1 - 1 );
	ENSURE( afterFree == only );

	b2DestroyWorld( onlyId );
	return 0;
}

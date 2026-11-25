// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"

#include "core.h"
#include "world.h"

#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <string.h>

static b2Body* b2GetBody( b2World* world, b2BodyId bodyId )
{
	if ( world == NULL || bodyId.index1 < 1 || bodyId.index1 > world->bodies.count )
	{
		return NULL;
	}
	b2Body* body = world->bodies.data + ( bodyId.index1 - 1 );
	if ( body->id != bodyId.index1 - 1 || body->generation != bodyId.generation )
	{
		return NULL;
	}
	return body;
}

b2BodyId b2CreateBody( b2WorldId worldId, const b2BodyDef* def )
{
	B2_CHECK_DEF( def );
	if ( b2IsValidVec2( def->position ) == false || b2IsValidRot( def->rotation ) == false ||
		 b2IsValidVec2( def->linearVelocity ) == false || b2IsValidFloat( def->angularVelocity ) == false ||
		 b2IsValidFloat( def->linearDamping ) == false || def->linearDamping < 0.0f ||
		 b2IsValidFloat( def->angularDamping ) == false || def->angularDamping < 0.0f ||
		 b2IsValidFloat( def->sleepThreshold ) == false || def->sleepThreshold < 0.0f ||
		 b2IsValidFloat( def->safetyFactor ) == false || def->safetyFactor < 0.0f || b2IsValidFloat( def->gravityScale ) == false )
	{
		return b2_nullBodyId;
	}

	if ( b2World_IsValid( worldId ) == false )
	{
		return b2_nullBodyId;
	}

	b2World* world = b2_worlds + ( worldId.index1 - 1 );
	if ( world->locked )
	{
		return b2_nullBodyId;
	}

	int bodyId = b2AllocId( &world->bodyIdPool );
	if ( bodyId == world->bodies.count )
	{
		b2Array_Push( world->bodies, b2Body{} );
	}

	b2Body* body = world->bodies.data + bodyId;
	uint16_t generation = (uint16_t)( body->generation + 1 );
	*body = b2Body{};
	body->generation = generation;
	body->id = bodyId;
	body->type = def->type;
	body->transform.p = def->position;
	body->transform.q = def->rotation;
	body->linearVelocity = def->linearVelocity;
	body->angularVelocity = def->angularVelocity;
	body->linearDamping = def->linearDamping;
	body->angularDamping = def->angularDamping;
	body->gravityScale = def->gravityScale;
	body->sleepThreshold = def->sleepThreshold;
	body->safetyFactor = def->safetyFactor;
	body->userData = def->userData;
	body->motionLocks = def->motionLocks;
	body->enableSleep = def->enableSleep;
	body->isAwake = def->isAwake;
	body->isEnabled = def->isEnabled;
	body->isBullet = def->isBullet;
	body->allowFastRotation = def->allowFastRotation;
	body->enableContactRecycling = def->enableContactRecycling;
	if ( def->name != NULL )
	{
		strncpy_s( body->name, sizeof( body->name ), def->name, B2_NAME_LENGTH );
	}

	return b2BodyId{ bodyId + 1, world->worldId, body->generation };
}

void b2DestroyBody( b2BodyId bodyId )
{
	if ( bodyId.world0 >= B2_MAX_WORLDS || b2_worlds[bodyId.world0].inUse == false )
	{
		return;
	}

	b2World* world = b2_worlds + bodyId.world0;
	b2Body* body = b2GetBody( world, bodyId );
	if ( body == NULL )
	{
		return;
	}

	b2FreeId( &world->bodyIdPool, body->id );
	uint16_t generation = body->generation;
	*body = b2Body{};
	body->generation = generation;
	body->id = B2_NULL_INDEX;
}

bool b2Body_IsValid( b2BodyId id )
{
	if ( id.world0 >= B2_MAX_WORLDS )
	{
		return false;
	}
	b2World* world = b2_worlds + id.world0;
	if ( world->inUse == false || world->worldId != id.world0 )
	{
		return false;
	}
	if ( id.index1 < 1 || world->bodies.count < id.index1 )
	{
		return false;
	}
	b2Body* body = world->bodies.data + ( id.index1 - 1 );
	if ( body->id == B2_NULL_INDEX )
	{
		return false;
	}
	return body->generation == id.generation;
}

void b2Body_SetName( b2BodyId bodyId, const char* name )
{
	if ( bodyId.world0 >= B2_MAX_WORLDS || b2_worlds[bodyId.world0].inUse == false )
	{
		return;
	}
	b2Body* body = b2GetBody( b2_worlds + bodyId.world0, bodyId );
	if ( body == NULL )
	{
		return;
	}
	if ( name != NULL )
	{
		strncpy_s( body->name, sizeof( body->name ), name, B2_NAME_LENGTH );
	}
	else
	{
		memset( body->name, 0, sizeof( body->name ) );
	}
}

const char* b2Body_GetName( b2BodyId bodyId )
{
	static const char empty[] = "";
	if ( bodyId.world0 >= B2_MAX_WORLDS || b2_worlds[bodyId.world0].inUse == false )
	{
		return empty;
	}
	b2Body* body = b2GetBody( b2_worlds + bodyId.world0, bodyId );
	if ( body == NULL )
	{
		return empty;
	}
	return body->name;
}

void b2Body_SetUserData( b2BodyId bodyId, void* userData )
{
	if ( bodyId.world0 >= B2_MAX_WORLDS || b2_worlds[bodyId.world0].inUse == false )
	{
		return;
	}
	b2Body* body = b2GetBody( b2_worlds + bodyId.world0, bodyId );
	if ( body == NULL )
	{
		return;
	}
	body->userData = userData;
}

void* b2Body_GetUserData( b2BodyId bodyId )
{
	if ( bodyId.world0 >= B2_MAX_WORLDS || b2_worlds[bodyId.world0].inUse == false )
	{
		return NULL;
	}
	b2Body* body = b2GetBody( b2_worlds + bodyId.world0, bodyId );
	if ( body == NULL )
	{
		return NULL;
	}
	return body->userData;
}

static b2WorldTransform b2GetBodyTransformFromId( b2BodyId bodyId )
{
	B2_ASSERT( b2Body_IsValid( bodyId ) );
	b2Body* body = b2GetBody( b2_worlds + bodyId.world0, bodyId );
	return body->transform;
}

b2Pos b2Body_GetPosition( b2BodyId bodyId )
{
	return b2GetBodyTransformFromId( bodyId ).p;
}

b2Rot b2Body_GetRotation( b2BodyId bodyId )
{
	return b2GetBodyTransformFromId( bodyId ).q;
}

b2WorldTransform b2Body_GetTransform( b2BodyId bodyId )
{
	return b2GetBodyTransformFromId( bodyId );
}

b2Vec2 b2Body_GetLocalPoint( b2BodyId bodyId, b2Pos worldPoint )
{
	return b2InvTransformPoint( b2GetBodyTransformFromId( bodyId ), worldPoint );
}

b2Pos b2Body_GetWorldPoint( b2BodyId bodyId, b2Vec2 localPoint )
{
	return b2TransformPoint( b2GetBodyTransformFromId( bodyId ), localPoint );
}

b2Vec2 b2Body_GetLocalVector( b2BodyId bodyId, b2Vec2 worldVector )
{
	return b2InvRotateVector( b2GetBodyTransformFromId( bodyId ).q, worldVector );
}

b2Vec2 b2Body_GetWorldVector( b2BodyId bodyId, b2Vec2 localVector )
{
	return b2RotateVector( b2GetBodyTransformFromId( bodyId ).q, localVector );
}

void b2Body_SetTransform( b2BodyId bodyId, b2Pos position, b2Rot rotation )
{
	if ( b2IsValidVec2( position ) == false || b2IsValidRot( rotation ) == false ||
		 b2Body_IsValid( bodyId ) == false )
	{
		return;
	}
	b2World* world = b2_worlds + bodyId.world0;
	if ( world->locked )
	{
		return;
	}
	b2Body* body = b2GetBody( world, bodyId );
	body->transform.p = position;
	body->transform.q = rotation;
}

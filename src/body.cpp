// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"

#include "core.h"
#include "world.h"
#include "shape.h"

#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <string.h>

b2Body* b2GetBodyFullId( b2World* world, b2BodyId bodyId )
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
	body->headShapeId = B2_NULL_INDEX;
	body->type = def->type;
	body->transform.p = def->position;
	body->transform.q = def->rotation;
	body->center = def->position;
	body->minExtent = B2_HUGE;
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
	b2Body* body = b2GetBodyFullId( world, bodyId );
	if ( body == NULL )
	{
		return;
	}


    while ( body->headShapeId != B2_NULL_INDEX )
    {
        b2Shape* shape = world->shapes.data + body->headShapeId;
        b2DestroyShape( b2ShapeId{ shape->id + 1, world->worldId, shape->generation }, false );
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
	b2Body* body = b2GetBodyFullId( b2_worlds + bodyId.world0, bodyId );
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
	b2Body* body = b2GetBodyFullId( b2_worlds + bodyId.world0, bodyId );
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
	b2Body* body = b2GetBodyFullId( b2_worlds + bodyId.world0, bodyId );
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
	b2Body* body = b2GetBodyFullId( b2_worlds + bodyId.world0, bodyId );
	if ( body == NULL )
	{
		return NULL;
	}
	return body->userData;
}

static b2WorldTransform b2GetBodyTransformFromId( b2BodyId bodyId )
{
	B2_ASSERT( b2Body_IsValid( bodyId ) );
	b2Body* body = b2GetBodyFullId( b2_worlds + bodyId.world0, bodyId );
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
	b2Body* body = b2GetBodyFullId( world, bodyId );
	body->transform.p = position;
	body->transform.q = rotation;
	body->center = b2TransformPoint( body->transform, body->localCenter );
    for ( int id = body->headShapeId; id != B2_NULL_INDEX; id = world->shapes.data[id].nextShapeId )
    {
        b2UpdateShapeAABB( world->shapes.data + id, body->transform );
    }
}

b2AABB b2Body_ComputeBounds( b2BodyId bodyId )
{
	if ( b2Body_IsValid( bodyId ) == false || b2_worlds[bodyId.world0].locked )
	{
		return b2AABB{};
	}
    b2World* world = b2_worlds + bodyId.world0;
    b2Body* body = b2GetBodyFullId( world, bodyId );
    if ( body->headShapeId == B2_NULL_INDEX )
    {
        b2Pos position = body->transform.p;
        return b2AABB{ position, position };
    }
    b2Shape* shape = world->shapes.data + body->headShapeId;
    b2AABB bounds = shape->aabb;
    while ( shape->nextShapeId != B2_NULL_INDEX )
    {
        shape = world->shapes.data + shape->nextShapeId;
        bounds = b2AABB_Union( bounds, shape->aabb );
    }
    return bounds;
}


void b2UpdateBodyMassData( b2World* world, b2Body* body )
{
    body->dirtyMass = false;
    body->mass = 0.0f;
    body->inertia = 0.0f;
    body->invMass = 0.0f;
    body->invInertia = 0.0f;
    body->localCenter = b2Vec2_zero;
    body->minExtent = B2_HUGE;
    body->maxExtent = 0.0f;
    if ( body->type != b2_dynamicBody )
    {
        body->center = body->transform.p;
        if ( body->type == b2_kinematicBody )
        {
            for ( int id = body->headShapeId; id != B2_NULL_INDEX; id = world->shapes.data[id].nextShapeId )
            {
                b2ShapeExtent extent = b2ComputeShapeExtent( world->shapes.data + id, b2Vec2_zero );
                body->minExtent = b2MinFloat( body->minExtent, extent.minExtent );
                body->maxExtent = b2MaxFloat( body->maxExtent, extent.maxExtent );
            }
        }
        return;
    }
    b2MassData* masses = static_cast<b2MassData*>( b2StackAlloc( &world->stack, body->shapeCount * sizeof( b2MassData ), "mass data" ) );
    b2Vec2 localCenter = b2Vec2_zero;
    int index = 0;
    for ( int id = body->headShapeId; id != B2_NULL_INDEX; id = world->shapes.data[id].nextShapeId )
    {
        const b2Shape* shape = world->shapes.data + id;
        if ( shape->density == 0.0f )
        {
            masses[index++] = b2MassData{};
            continue;
        }
        b2MassData data = b2ComputeShapeMass( shape );
        body->mass += data.mass;
        localCenter = b2MulAdd( localCenter, data.mass, data.center );
        masses[index++] = data;
    }
    if ( body->mass > 0.0f )
    {
        body->invMass = 1.0f / body->mass;
        localCenter = b2MulSV( body->invMass, localCenter );
    }
    for ( int i = 0; i < body->shapeCount; ++i )
    {
        b2MassData data = masses[i];
        if ( data.mass == 0.0f )
        {
            continue;
        }
        b2Vec2 offset = b2Sub( localCenter, data.center );
        body->inertia += data.rotationalInertia + data.mass * b2Dot( offset, offset );
    }
    b2StackFree( &world->stack, masses );
    if ( body->inertia > 0.0f && body->motionLocks.angularZ == false )
    {
        body->invInertia = 1.0f / body->inertia;
    }
    else
    {
        body->inertia = 0.0f;
    }
    b2Pos oldCenter = body->center;
    body->localCenter = localCenter;
    body->center = b2TransformPoint( body->transform, localCenter );
    if ( body->isAwake && body->isEnabled )
    {
        b2Vec2 delta = b2CrossSV( body->angularVelocity, b2Sub( body->center, oldCenter ) );
        body->linearVelocity = b2Add( body->linearVelocity, delta );
    }
    for ( int id = body->headShapeId; id != B2_NULL_INDEX; id = world->shapes.data[id].nextShapeId )
    {
        b2ShapeExtent extent = b2ComputeShapeExtent( world->shapes.data + id, localCenter );
        body->minExtent = b2MinFloat( body->minExtent, extent.minExtent );
        body->maxExtent = b2MaxFloat( body->maxExtent, extent.maxExtent );
    }
}

b2MassData b2Body_GetMassData( b2BodyId id )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    b2Body* body = b2GetBodyFullId( b2_worlds + id.world0, id );
    return b2MassData{ body->mass, body->localCenter, body->inertia };
}

float b2Body_GetMass( b2BodyId id )
{
    return b2Body_GetMassData( id ).mass;
}

float b2Body_GetRotationalInertia( b2BodyId id )
{
    return b2Body_GetMassData( id ).rotationalInertia;
}

b2Vec2 b2Body_GetLocalCenter( b2BodyId id )
{
    return b2Body_GetMassData( id ).center;
}

b2Pos b2Body_GetWorldCenter( b2BodyId id )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    return b2GetBodyFullId( b2_worlds + id.world0, id )->center;
}

void b2Body_UpdateMassFromShapes( b2BodyId id )
{
    if ( b2Body_IsValid( id ) && b2_worlds[id.world0].locked == false )
    {
        b2UpdateBodyMassData( b2_worlds + id.world0, b2GetBodyFullId( b2_worlds + id.world0, id ) );
    }
}


void b2Body_SetMassData( b2BodyId id, b2MassData data )
{
    if ( b2IsValidFloat( data.mass ) == false || data.mass < 0.0f ||
         b2IsValidFloat( data.rotationalInertia ) == false || data.rotationalInertia < 0.0f ||
         b2IsValidVec2( data.center ) == false || b2Body_IsValid( id ) == false || b2_worlds[id.world0].locked )
    {
        return;
    }
    b2World* world = b2_worlds + id.world0;
    b2Body* body = b2GetBodyFullId( world, id );
    body->mass = data.mass;
    body->inertia = data.rotationalInertia;
    body->localCenter = data.center;
    b2Pos oldCenter = body->center;
    body->center = b2TransformPoint( body->transform, data.center );
    if ( body->type != b2_staticBody && body->isAwake && body->isEnabled )
    {
        b2Vec2 delta = b2CrossSV( body->angularVelocity, b2Sub( body->center, oldCenter ) );
        body->linearVelocity = b2Add( body->linearVelocity, delta );
    }
    body->invMass = body->mass > 0.0f ? 1.0f / body->mass : 0.0f;
    body->invInertia = body->inertia > 0.0f ? 1.0f / body->inertia : 0.0f;
    body->minExtent = B2_HUGE;
    body->maxExtent = 0.0f;
    for ( int shapeId = body->headShapeId; shapeId != B2_NULL_INDEX; shapeId = world->shapes.data[shapeId].nextShapeId )
    {
        b2ShapeExtent extent = b2ComputeShapeExtent( world->shapes.data + shapeId, data.center );
        body->minExtent = b2MinFloat( body->minExtent, extent.minExtent );
        body->maxExtent = b2MaxFloat( body->maxExtent, extent.maxExtent );
    }
    if ( body->motionLocks.angularZ )
    {
        body->inertia = 0.0f;
        body->invInertia = 0.0f;
    }
    body->dirtyMass = false;
}

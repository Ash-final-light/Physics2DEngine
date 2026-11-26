// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "state_hash.h"

#include "core.h"

#include "body.h"
#include "contact.h"
#include "id_pool.h"
#include "joint.h"
#include "world.h"

uint64_t b2HashWorldState( b2World* world )
{
	uint64_t hash = B2_SNAP_FNV_INIT;
	const uint64_t prime = B2_SNAP_FNV_PRIME;

	int bodyCount = world->bodies.count;
	for ( int i = 0; i < bodyCount; ++i )
	{
		b2Body* body = world->bodies.data + i;
		if ( body->id != i )
		{
			// 空闲或从未使用的槽
			continue;
		}

		b2BodySim* sim = b2GetBodySim( world, body );
		uint32_t bits;

#define B2_HASH_FLOAT( f )                                                                                                       \
	memcpy( &bits, &( f ), 4 );                                                                                                  \
	hash = ( hash ^ (uint64_t)bits ) * prime;

		hash = b2FnvMixPosition( hash, sim->transform.p );
		B2_HASH_FLOAT( sim->transform.q.c )
		B2_HASH_FLOAT( sim->transform.q.s )

		b2BodyState* state = b2GetBodyState( world, body );
		if ( state != NULL )
		{
			B2_HASH_FLOAT( state->linearVelocity.x )
			B2_HASH_FLOAT( state->linearVelocity.y )
			B2_HASH_FLOAT( state->angularVelocity )
		}

#undef B2_HASH_FLOAT
	}

	return hash;
}

static uint64_t b2FnvMixBytes( uint64_t hash, const void* data, int n )
{
	const uint8_t* p = (const uint8_t*)data;
	for ( int i = 0; i < n; ++i )
	{
		hash = ( hash ^ (uint64_t)p[i] ) * B2_SNAP_FNV_PRIME;
	}
	return hash;
}

static uint64_t b2FnvMixFloat( uint64_t hash, float f )
{
	uint32_t bits;
	memcpy( &bits, &f, 4 );
	return ( hash ^ (uint64_t)bits ) * B2_SNAP_FNV_PRIME;
}

static uint64_t b2FnvMixInt( uint64_t hash, int v )
{
	return ( hash ^ (uint64_t)(uint32_t)v ) * B2_SNAP_FNV_PRIME;
}

uint64_t b2HashWorldStateDeep( b2World* world )
{
	uint64_t hash = B2_SNAP_FNV_INIT;

	int bodyCount = world->bodies.count;
	for ( int i = 0; i < bodyCount; ++i )
	{
		b2Body* body = world->bodies.data + i;
		if ( body->id != i )
		{
			continue;
		}

		b2BodySim* sim = b2GetBodySim( world, body );

		hash = b2FnvMixPosition( hash, sim->transform.p );
		hash = b2FnvMixFloat( hash, sim->transform.q.c );
		hash = b2FnvMixFloat( hash, sim->transform.q.s );

		b2BodyState* state = b2GetBodyState( world, body );
		if ( state != NULL )
		{
			hash = b2FnvMixFloat( hash, state->linearVelocity.x );
			hash = b2FnvMixFloat( hash, state->linearVelocity.y );
			hash = b2FnvMixFloat( hash, state->angularVelocity );
		}

		hash = b2FnvMixInt( hash, body->setIndex );
		hash = b2FnvMixInt( hash, body->localIndex );
	}

	int contactCount = world->contacts.count;
	for ( int i = 0; i < contactCount; ++i )
	{
		b2Contact* contact = world->contacts.data + i;
		if ( contact->contactId != i )
		{
			continue;
		}

		hash = b2FnvMixInt( hash, contact->setIndex );
		hash = b2FnvMixInt( hash, contact->colorIndex );
		hash = b2FnvMixInt( hash, contact->localIndex );

		b2ContactSim* sim = b2GetContactSim( world, contact );
		if ( sim != NULL )
		{
			b2Manifold* m = &sim->manifold;
			hash = b2FnvMixInt( hash, m->pointCount );
			for ( int p = 0; p < m->pointCount; ++p )
			{
				hash = b2FnvMixFloat( hash, m->points[p].normalImpulse );
				hash = b2FnvMixFloat( hash, m->points[p].tangentImpulse );
				hash = b2FnvMixFloat( hash, m->points[p].totalNormalImpulse );
			}
		}
	}

	int jointCount = world->joints.count;
	for ( int i = 0; i < jointCount; ++i )
	{
		b2Joint* joint = world->joints.data + i;
		if ( joint->jointId != i )
		{
			continue;
		}

		hash = b2FnvMixInt( hash, joint->setIndex );
		hash = b2FnvMixInt( hash, joint->colorIndex );
		hash = b2FnvMixInt( hash, joint->localIndex );

		b2JointSim* sim = b2GetJointSim( world, joint );
		if ( sim != NULL )
		{
			switch ( sim->type )
			{
				case b2_distanceJoint:
					hash = b2FnvMixFloat( hash, sim->distanceJoint.impulse );
					hash = b2FnvMixFloat( hash, sim->distanceJoint.lowerImpulse );
					hash = b2FnvMixFloat( hash, sim->distanceJoint.upperImpulse );
					hash = b2FnvMixFloat( hash, sim->distanceJoint.motorImpulse );
					break;
				case b2_moverJoint:
					hash = b2FnvMixBytes( hash, &sim->moverJoint.linearVelocityImpulse, sizeof( b2Vec2 ) );
					break;
				case b2_motorJoint:
					hash = b2FnvMixBytes( hash, &sim->motorJoint.linearVelocityImpulse, sizeof( b2Vec2 ) );
					hash = b2FnvMixFloat( hash, sim->motorJoint.angularVelocityImpulse );
					hash = b2FnvMixBytes( hash, &sim->motorJoint.linearSpringImpulse, sizeof( b2Vec2 ) );
					hash = b2FnvMixFloat( hash, sim->motorJoint.angularSpringImpulse );
					break;
				case b2_pogoJoint:
					hash = b2FnvMixFloat( hash, sim->pogoJoint.impulse );
					hash = b2FnvMixFloat( hash, sim->pogoJoint.velocity );
					break;
				case b2_prismaticJoint:
					hash = b2FnvMixBytes( hash, &sim->prismaticJoint.impulse, sizeof( b2Vec2 ) );
					hash = b2FnvMixFloat( hash, sim->prismaticJoint.springImpulse );
					hash = b2FnvMixFloat( hash, sim->prismaticJoint.motorImpulse );
					hash = b2FnvMixFloat( hash, sim->prismaticJoint.lowerImpulse );
					hash = b2FnvMixFloat( hash, sim->prismaticJoint.upperImpulse );
					break;
				case b2_revoluteJoint:
					hash = b2FnvMixBytes( hash, &sim->revoluteJoint.linearImpulse, sizeof( b2Vec2 ) );
					hash = b2FnvMixFloat( hash, sim->revoluteJoint.springImpulse );
					hash = b2FnvMixFloat( hash, sim->revoluteJoint.motorImpulse );
					hash = b2FnvMixFloat( hash, sim->revoluteJoint.lowerImpulse );
					hash = b2FnvMixFloat( hash, sim->revoluteJoint.upperImpulse );
					break;
				case b2_weldJoint:
					hash = b2FnvMixBytes( hash, &sim->weldJoint.linearImpulse, sizeof( b2Vec2 ) );
					hash = b2FnvMixFloat( hash, sim->weldJoint.angularImpulse );
					break;
				case b2_wheelJoint:
					hash = b2FnvMixFloat( hash, sim->wheelJoint.perpImpulse );
					hash = b2FnvMixFloat( hash, sim->wheelJoint.motorImpulse );
					hash = b2FnvMixFloat( hash, sim->wheelJoint.springImpulse );
					hash = b2FnvMixFloat( hash, sim->wheelJoint.lowerImpulse );
					hash = b2FnvMixFloat( hash, sim->wheelJoint.upperImpulse );
					break;
				default:
					break;
			}
		}
	}

	hash = b2FnvMixInt( hash, world->bodyIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->bodyIdPool ) );
	hash = b2FnvMixInt( hash, world->shapeIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->shapeIdPool ) );
	hash = b2FnvMixInt( hash, world->chainIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->chainIdPool ) );
	hash = b2FnvMixInt( hash, world->contactIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->contactIdPool ) );
	hash = b2FnvMixInt( hash, world->jointIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->jointIdPool ) );
	hash = b2FnvMixInt( hash, world->islandIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->islandIdPool ) );
	hash = b2FnvMixInt( hash, world->solverSetIdPool.nextIndex );
	hash = b2FnvMixInt( hash, b2GetIdCount( &world->solverSetIdPool ) );

	hash = b2FnvMixInt( hash, world->solverSets.count );

	return hash;
}

uint64_t b2World_GetStateHash( b2WorldId worldId )
{
	B2_ASSERT( 1 <= worldId.index1 && worldId.index1 <= B2_MAX_WORLDS );
	b2World* world = b2_worlds + ( worldId.index1 - 1 );
	B2_ASSERT( worldId.index1 == world->worldId + 1 );
	B2_ASSERT( worldId.generation == world->generation );
	return b2HashWorldStateDeep( world );
}

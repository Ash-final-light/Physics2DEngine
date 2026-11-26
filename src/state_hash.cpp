// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "state_hash.h"

#include "body.h"
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

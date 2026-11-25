// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "world.h"

#include "core.h"

#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <string.h>

static_assert( B2_MAX_WORLDS > 0, "must be 1 or more" );
static_assert( B2_MAX_WORLDS < UINT16_MAX, "B2_MAX_WORLDS limit exceeded" );
static b2World b2_worlds[B2_MAX_WORLDS];

b2WorldId b2CreateWorld( const b2WorldDef* def )
{
	B2_CHECK_DEF( def );
	if ( b2IsValidVec2( def->gravity ) == false || b2IsValidFloat( def->restitutionThreshold ) == false ||
		 def->restitutionIterations < 0 || b2IsValidFloat( def->hitEventThreshold ) == false ||
		 b2IsValidFloat( def->contactHertz ) == false || b2IsValidFloat( def->contactDampingRatio ) == false ||
		 b2IsValidFloat( def->contactSpeed ) == false || b2IsValidFloat( def->maximumLinearSpeed ) == false ||
		 def->maximumLinearSpeed <= 0.0f )
	{
		return b2WorldId{};
	}

	int worldId = B2_NULL_INDEX;
	for ( int i = 0; i < B2_MAX_WORLDS; ++i )
	{
		if ( b2_worlds[i].inUse == false )
		{
			worldId = i;
			break;
		}
	}
	if ( worldId == B2_NULL_INDEX )
	{
		return b2WorldId{};
	}

	b2World* world = b2_worlds + worldId;
	uint16_t generation = world->generation;
	*world = b2World{};
	world->worldId = (uint16_t)worldId;
	world->generation = generation;
	world->inUse = true;
	world->stack = b2CreateStack( b2MaxInt( 2048, def->capacity.arenaByteCount ) );
	world->gravity = def->gravity;
	world->hitEventThreshold = b2ClampFloat( def->hitEventThreshold, 0.0f, FLT_MAX );
	world->restitutionThreshold = b2ClampFloat( def->restitutionThreshold, 0.0f, FLT_MAX );
	world->restitutionIterations = b2ClampInt( def->restitutionIterations, 0, B2_MAX_RESTITUTION_ITERATIONS );
	world->enableRestitutionPropagation = def->enableRestitutionPropagation;
	world->maxLinearSpeed = def->maximumLinearSpeed;
	world->contactSpeed = b2ClampFloat( def->contactSpeed, 0.0f, FLT_MAX );
	world->contactHertz = b2ClampFloat( def->contactHertz, 0.0f, FLT_MAX );
	world->contactDampingRatio = b2ClampFloat( def->contactDampingRatio, 0.0f, FLT_MAX );
	world->frictionCallback = def->frictionCallback;
	world->restitutionCallback = def->restitutionCallback;
	world->enableSleep = def->enableSleep;
	world->enableContinuous = def->enableContinuous;
	world->workerCount = 1;
	world->userData = def->userData;
	return b2WorldId{ (uint16_t)( worldId + 1 ), world->generation };
}

void b2DestroyWorld( b2WorldId worldId )
{
	if ( worldId.index1 < 1 || worldId.index1 > B2_MAX_WORLDS )
	{
		return;
	}

	b2World* world = b2_worlds + ( worldId.index1 - 1 );
	if ( world->inUse == false || world->generation != worldId.generation )
	{
		return;
	}

	b2DestroyStack( &world->stack );
	uint16_t generation = world->generation;
	*world = b2World{};
	world->generation = generation + 1;
}

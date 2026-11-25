// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "world.h"

#include "core.h"
#include "solver.h"

#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <string.h>

static_assert( B2_MAX_WORLDS > 0, "must be 1 or more" );
static_assert( B2_MAX_WORLDS < UINT16_MAX, "B2_MAX_WORLDS limit exceeded" );
b2World b2_worlds[B2_MAX_WORLDS];

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
	world->contactRecycleDistance=B2_CONTACT_RECYCLE_DISTANCE;
	world->enableWarmStarting=true;
	world->enableSleep = def->enableSleep;
	world->enableContinuous = def->enableContinuous;
	world->workerCount = 1;
	world->userData = def->userData;
	world->bodyIdPool = b2CreateIdPool();
	b2Array_Create( world->bodies );
	world->shapeIdPool = b2CreateIdPool();
	b2Array_Create( world->shapes );
	b2Array_Create( world->fatAABBs );
	world->chainIdPool = b2CreateIdPool();
	b2Array_Create( world->chainShapes );
	b2CreateBroadPhase( &world->broadPhase, &def->capacity );
	world->contactIdPool = b2CreateIdPool();
	b2Array_Create( world->contacts );
	world->islandIdPool=b2CreateIdPool();
	b2Array_Create(world->islands);
	b2CreateSolverSets( world );
	b2CreateGraph(&world->constraintGraph,&def->capacity);
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

	b2DestroyGraph(&world->constraintGraph);
	b2DestroyBroadPhase( &world->broadPhase );
	b2Array_Destroy( world->contacts );
	for(int i=0;i<world->islands.count;++i)
	{ if(world->islands.data[i].setIndex!=B2_NULL_INDEX) { b2DestroyIsland(world,i); } }
	b2Array_Destroy(world->islands);
	b2DestroyIdPool(&world->islandIdPool);
	b2DestroySolverSets( world );
	b2DestroyIdPool( &world->contactIdPool );
	b2DestroyStack( &world->stack );
	b2Array_Destroy( world->bodies );
    for ( int i = 0; i < world->chainShapes.count; ++i )
    {
        b2ChainShape* chain = world->chainShapes.data + i;
        b2Free( chain->shapeIndices, chain->segmentCount * sizeof( int ) );
    }
    b2Array_Destroy( world->chainShapes );
    b2DestroyIdPool( &world->chainIdPool );
	b2Array_Destroy( world->shapes );
	b2Array_Destroy( world->fatAABBs );
	b2DestroyIdPool( &world->shapeIdPool );
	b2DestroyIdPool( &world->bodyIdPool );
	uint16_t generation = world->generation;
	*world = b2World{};
	world->generation = generation + 1;
}

bool b2World_IsValid( b2WorldId id )
{
	if ( id.index1 < 1 || B2_MAX_WORLDS < id.index1 )
	{
		return false;
	}

	b2World* world = b2_worlds + ( id.index1 - 1 );
	if ( world->worldId != id.index1 - 1 )
	{
		return false;
	}
	return id.generation == world->generation;
}

static b2World* b2GetWorldFromId( b2WorldId id )
{
	if ( b2World_IsValid( id ) == false )
	{
		return NULL;
	}
	return b2_worlds + ( id.index1 - 1 );
}

void b2World_SetUserData( b2WorldId worldId, void* userData )
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( world == NULL )
	{
		return;
	}
	world->userData = userData;
}

void* b2World_GetUserData( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( world == NULL )
	{
		return NULL;
	}
	return world->userData;
}


void b2World_Step( b2WorldId id, float timeStep, int subStepCount )
{
    if ( b2World_IsValid( id ) == false || b2IsValidFloat( timeStep ) == false || timeStep <= 0.0f || subStepCount <= 0 )
    {
        return;
    }
    b2World* world = b2_worlds+id.index1-1;
    if ( world->inUse == false || world->locked )
    {
        return;
    }
    world->locked = true;
    b2UpdateBroadPhaseContacts( world );
    b2UpdateContacts( world );
    b2SplitContactIslands( world );
    b2SolveFreeBodies( world, timeStep, subStepCount );
    world->locked = false;
}

void b2World_EnableWarmStarting(b2WorldId id,bool flag)
{
    b2World* world=b2GetWorldFromId(id);
    if(world==NULL || world->locked) { return; }
    world->enableWarmStarting=flag;
}

bool b2World_IsWarmStartingEnabled(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);
    return world!=NULL && world->enableWarmStarting;
}

void b2World_SetContactTuning(b2WorldId id,float hertz,float dampingRatio,float pushSpeed)
{
    if(!b2IsValidFloat(hertz) || !b2IsValidFloat(dampingRatio) || !b2IsValidFloat(pushSpeed)) { return; }
    b2World* world=b2GetWorldFromId(id);
    if(world==NULL || world->locked) { return; }
    world->contactHertz=b2ClampFloat(hertz,0.0f,FLT_MAX);
    world->contactDampingRatio=b2ClampFloat(dampingRatio,0.0f,FLT_MAX);
    world->contactSpeed=b2ClampFloat(pushSpeed,0.0f,FLT_MAX);
}

void b2World_GetContactTuning(b2WorldId id,float* hertz,float* dampingRatio,float* pushSpeed)
{
    b2World* world=b2GetWorldFromId(id);
    if(world==NULL || hertz==NULL || dampingRatio==NULL || pushSpeed==NULL) { return; }
    *hertz=world->contactHertz;*dampingRatio=world->contactDampingRatio;*pushSpeed=world->contactSpeed;
}

void b2World_SetContactRecycleDistance(b2WorldId id,float distance)
{
    if(!b2IsValidFloat(distance)) { return; }
    b2World* world=b2GetWorldFromId(id);if(world==NULL || world->locked) { return; }
    world->contactRecycleDistance=b2ClampFloat(distance,0.0f,FLT_MAX);
}

float b2World_GetContactRecycleDistance(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);return world!=NULL?world->contactRecycleDistance:0.0f;
}

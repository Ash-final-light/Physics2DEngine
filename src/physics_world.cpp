// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "world.h"

#include "core.h"
#include "solver.h"

#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <string.h>
#include <stdio.h>

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
	b2Array_CreateN(world->sensors,4);b2Array_CreateN(world->sensorTaskContexts,1);
    b2SensorTaskContext sensorContext={};sensorContext.eventBits=b2CreateBitSet(16);b2Array_Push(world->sensorTaskContexts,sensorContext);
    b2Array_CreateN(world->sensorBeginEvents,4);b2Array_CreateN(world->sensorEndEvents[0],4);b2Array_CreateN(world->sensorEndEvents[1],4);
	b2Array_CreateN(world->bodyMoveEvents,4);
	b2Array_CreateN(world->contactBeginEvents,4);
	b2Array_CreateN(world->contactHitEvents,4);
	b2Array_CreateN(world->contactEndEvents[0],4);
	b2Array_CreateN(world->contactEndEvents[1],4);
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

    for(int i=0;i<world->sensors.count;++i)
    { b2Sensor* sensor=world->sensors.data+i;b2Array_Destroy(sensor->hits);b2Array_Destroy(sensor->overlaps1);b2Array_Destroy(sensor->overlaps2); }
    b2Array_Destroy(world->sensors);
    for(int i=0;i<world->sensorTaskContexts.count;++i) { b2DestroyBitSet(&world->sensorTaskContexts.data[i].eventBits); }
    b2Array_Destroy(world->sensorTaskContexts);b2Array_Destroy(world->sensorBeginEvents);b2Array_Destroy(world->sensorEndEvents[0]);b2Array_Destroy(world->sensorEndEvents[1]);
	b2Array_Destroy(world->bodyMoveEvents);
	b2Array_Destroy(world->contactBeginEvents);
	b2Array_Destroy(world->contactHitEvents);
	b2Array_Destroy(world->contactEndEvents[0]);
	b2Array_Destroy(world->contactEndEvents[1]);
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
    b2Array_Clear(world->bodyMoveEvents);
    b2Array_Clear(world->contactBeginEvents);
    b2Array_Clear(world->sensorBeginEvents);
    b2Array_Clear(world->contactHitEvents);
    for(int i=0;i<world->bodies.count;++i) { world->bodies.data[i].bodyMoveIndex=B2_NULL_INDEX; }
    uint64_t stepTicks=b2GetTicks();world->profile=b2Profile{};
    world->locked = true;
	{
		b2Capacity* c = &world->maxCapacity;
		c->staticShapeCount = b2MaxInt( c->staticShapeCount, world->broadPhase.trees[b2_staticBody].proxyCount );
		c->dynamicShapeCount = b2MaxInt( c->dynamicShapeCount, world->broadPhase.trees[b2_dynamicBody].proxyCount );

		int staticBodyCount = world->solverSets.data[b2_staticSet].bodySims.count;
		c->staticBodyCount = b2MaxInt( c->staticBodyCount, staticBodyCount );

		// 这包括运动学刚体
		int totalBodyCount = b2GetIdCount( &world->bodyIdPool );
		c->dynamicBodyCount = b2MaxInt( c->dynamicBodyCount, totalBodyCount - staticBodyCount );

		int totalContactCount = b2GetIdCount( &world->contactIdPool );
		c->contactCount = b2MaxInt( c->contactCount, totalContactCount );
	}

    uint64_t stageTicks=b2GetTicks();
    b2UpdateBroadPhaseContacts( world );world->profile.pairs=b2GetMillisecondsAndReset(&stageTicks);
    b2UpdateContacts( world );world->profile.collide=b2GetMillisecondsAndReset(&stageTicks);
    b2SplitContactIslands( world );world->profile.splitIslands=b2GetMillisecondsAndReset(&stageTicks);
    b2SolveFreeBodies( world, timeStep, subStepCount );world->profile.solve=b2GetMillisecondsAndReset(&stageTicks);
    world->maxCapacity.arenaByteCount=b2MaxInt(world->maxCapacity.arenaByteCount,b2GetMaxStackAllocation(&world->stack));
    uint64_t sensorTicks=b2GetTicks();b2BeginSensors(world);b2EndSensors(world);world->profile.sensors=b2GetMilliseconds(sensorTicks);
    world->endEventArrayIndex=1-world->endEventArrayIndex;
    b2Array_Clear(world->contactEndEvents[world->endEventArrayIndex]);
    b2Array_Clear(world->sensorEndEvents[world->endEventArrayIndex]);
    world->profile.step=b2GetMilliseconds(stepTicks);
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

void b2World_SetRestitutionThreshold(b2WorldId id,float value)
{
    if(!b2IsValidFloat(value)) { return; }b2World* world=b2GetWorldFromId(id);
    if(world==NULL || world->locked) { return; }world->restitutionThreshold=b2ClampFloat(value,0.0f,FLT_MAX);
}

float b2World_GetRestitutionThreshold(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);return world!=NULL?world->restitutionThreshold:0.0f;
}

void b2World_SetRestitutionIterations(b2WorldId id,int iterations)
{
    if(iterations<0) { return; }b2World* world=b2GetWorldFromId(id);
    if(world==NULL || world->locked) { return; }world->restitutionIterations=b2ClampInt(iterations,0,B2_MAX_RESTITUTION_ITERATIONS);
}

int b2World_GetRestitutionIterations(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);return world!=NULL?world->restitutionIterations:0;
}

void b2World_EnableRestitutionPropagation(b2WorldId id,bool flag)
{
    b2World* world=b2GetWorldFromId(id);if(world==NULL || world->locked) { return; }world->enableRestitutionPropagation=flag;
}

bool b2World_IsRestitutionPropagationEnabled(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);return world!=NULL && world->enableRestitutionPropagation;
}

void b2World_SetFrictionCallback(b2WorldId id,b2FrictionCallbackFcn* callback)
{
    b2World* world=b2GetWorldFromId(id);if(world==NULL || world->locked) { return; }world->frictionCallback=callback;
}

void b2World_SetRestitutionCallback(b2WorldId id,b2RestitutionCallbackFcn* callback)
{
    b2World* world=b2GetWorldFromId(id);if(world==NULL || world->locked) { return; }world->restitutionCallback=callback;
}

void b2World_EnableSleeping(b2WorldId id,bool flag)
{
    b2World* world=b2GetWorldFromId(id);if(world==NULL || world->locked || flag==world->enableSleep) { return; }
    world->enableSleep=flag;
    if(!flag)
    {
        for(int i=b2_firstSleepingSet;i<world->solverSets.count;++i)
        { if(world->solverSets.data[i].bodySims.count>0) { b2WakeSolverSet(world,i); } }
    }
}

bool b2World_IsSleepingEnabled(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);return world!=NULL && world->enableSleep;
}

void b2World_SetMaxLinearSpeed(b2WorldId id,float speed)
{
    if(!b2IsValidFloat(speed) || speed<=0) { return; }b2World* world=b2GetWorldFromId(id);
    if(world==NULL || world->locked) { return; }world->maxLinearSpeed=speed;
}

float b2World_GetMaxLinearSpeed(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);return world!=NULL?world->maxLinearSpeed:0.0f;
}

void b2World_SetGravity(b2WorldId id,b2Vec2 gravity)
{
    if(!b2IsValidVec2(gravity)) { return; }b2World* world=b2GetWorldFromId(id);if(world!=NULL) { world->gravity=gravity; }
}

b2Vec2 b2World_GetGravity(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);return world!=NULL?world->gravity:b2Vec2_zero;
}

void b2World_EnableContinuous(b2WorldId id,bool flag)
{
    b2World* world=b2GetWorldFromId(id);if(world==NULL || world->locked) { return; }world->enableContinuous=flag;
}

bool b2World_IsContinuousEnabled(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);return world!=NULL && world->enableContinuous;
}

b2Counters b2World_GetCounters(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);b2Counters counters={};if(world==NULL) { return counters; }
    counters.bodyCount=b2GetIdCount(&world->bodyIdPool);counters.shapeCount=b2GetIdCount(&world->shapeIdPool);
    counters.contactCount=b2GetIdCount(&world->contactIdPool);counters.islandCount=b2GetIdCount(&world->islandIdPool);
    counters.staticTreeHeight=b2DynamicTree_GetHeight(world->broadPhase.trees+b2_staticBody);
    counters.treeHeight=b2MaxInt(b2DynamicTree_GetHeight(world->broadPhase.trees+b2_dynamicBody),b2DynamicTree_GetHeight(world->broadPhase.trees+b2_kinematicBody));
    counters.stackUsed=b2GetMaxStackAllocation(&world->stack);counters.byteCount=b2GetByteCount();
    counters.recycledContactCount=world->recycledContactCount;
    for(int i=0;i<B2_GRAPH_COLOR_COUNT;++i)
    {
        counters.colorCounts[i]=world->constraintGraph.colors[i].contactSims.count;
        counters.awakeContactCount+=world->constraintGraph.colors[i].contactSims.count;
    }
    counters.awakeContactCount+=world->solverSets.data[b2_awakeSet].contactSims.count;
    return counters;
}

b2Capacity b2World_GetMaxCapacity(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);return world!=NULL?world->maxCapacity:b2Capacity{};
}

b2Profile b2World_GetProfile(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);return world!=NULL?world->profile:b2Profile{};
}

static void b2WriteMemoryStat(FILE* file,const char* name,int bytes,int* total)
{
    fprintf(file,"%s: %d\n",name,bytes);*total+=bytes;
}

void b2World_DumpMemoryStats(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);if(world==NULL) { return; }
    FILE* file=fopen("box2d_memory.txt","w");if(file==NULL) { return; }
    int total=0;
    fprintf(file,"id pools\n");
    b2WriteMemoryStat(file,"body ids",b2GetIdBytes(&world->bodyIdPool),&total);
    b2WriteMemoryStat(file,"solver set ids",b2GetIdBytes(&world->solverSetIdPool),&total);
    b2WriteMemoryStat(file,"contact ids",b2GetIdBytes(&world->contactIdPool),&total);
    b2WriteMemoryStat(file,"island ids",b2GetIdBytes(&world->islandIdPool),&total);
    b2WriteMemoryStat(file,"shape ids",b2GetIdBytes(&world->shapeIdPool),&total);
    b2WriteMemoryStat(file,"chain ids",b2GetIdBytes(&world->chainIdPool),&total);
    fprintf(file,"\nworld arrays\n");
    b2WriteMemoryStat(file,"bodies",b2Array_ByteCount(world->bodies),&total);
    b2WriteMemoryStat(file,"solver sets",b2Array_ByteCount(world->solverSets),&total);
    b2WriteMemoryStat(file,"contacts",b2Array_ByteCount(world->contacts),&total);
    b2WriteMemoryStat(file,"islands",b2Array_ByteCount(world->islands),&total);
    b2WriteMemoryStat(file,"shapes",b2Array_ByteCount(world->shapes),&total);
    b2WriteMemoryStat(file,"fat AABBs",b2Array_ByteCount(world->fatAABBs),&total);
    b2WriteMemoryStat(file,"chains",b2Array_ByteCount(world->chainShapes),&total);
    int islandBytes=0,chainBytes=0;
    for(int i=0;i<world->islands.count;++i)
    { islandBytes+=b2Array_ByteCount(world->islands.data[i].bodies)+b2Array_ByteCount(world->islands.data[i].contacts); }
    for(int i=0;i<world->chainShapes.count;++i)
    { if(world->chainShapes.data[i].id!=B2_NULL_INDEX) { chainBytes+=world->chainShapes.data[i].segmentCount*sizeof(int); } }
    b2WriteMemoryStat(file,"island links",islandBytes,&total);
    fprintf(file,"\nowned arrays\n");b2WriteMemoryStat(file,"chain data",chainBytes,&total);
    fprintf(file,"\nbroad-phase\n");
    b2WriteMemoryStat(file,"static tree",(int)b2DynamicTree_GetByteCount(world->broadPhase.trees+b2_staticBody),&total);
    b2WriteMemoryStat(file,"kinematic tree",(int)b2DynamicTree_GetByteCount(world->broadPhase.trees+b2_kinematicBody),&total);
    b2WriteMemoryStat(file,"dynamic tree",(int)b2DynamicTree_GetByteCount(world->broadPhase.trees+b2_dynamicBody),&total);
    b2WriteMemoryStat(file,"pairSet",b2GetHashSetBytes(&world->broadPhase.pairSet),&total);
    int bodyBytes=0,stateBytes=0,contactBytes=0,islandSimBytes=0;
    for(int i=0;i<world->solverSets.count;++i)
    {
        b2SolverSet* set=world->solverSets.data+i;if(set->setIndex==B2_NULL_INDEX) { continue; }
        bodyBytes+=b2Array_ByteCount(set->bodySims);stateBytes+=b2Array_ByteCount(set->bodyStates);
        contactBytes+=b2Array_ByteCount(set->contactSims);islandSimBytes+=b2Array_ByteCount(set->islandSims);
    }
    fprintf(file,"\nsolver sets\n");
    b2WriteMemoryStat(file,"body sim",bodyBytes,&total);b2WriteMemoryStat(file,"body state",stateBytes,&total);
    b2WriteMemoryStat(file,"contact sim",contactBytes,&total);b2WriteMemoryStat(file,"island sim",islandSimBytes,&total);
    int bitBytes=0,graphBytes=0;
    for(int i=0;i<B2_GRAPH_COLOR_COUNT;++i)
    { bitBytes+=b2GetBitSetBytes(&world->constraintGraph.colors[i].bodySet);graphBytes+=b2Array_ByteCount(world->constraintGraph.colors[i].contactSims); }
    fprintf(file,"\nconstraint graph\n");b2WriteMemoryStat(file,"body bit sets",bitBytes,&total);b2WriteMemoryStat(file,"contact sim",graphBytes,&total);
    fprintf(file,"\n");b2WriteMemoryStat(file,"stack allocator",world->stack.capacity,&total);
    fprintf(file,"\nevents\n");b2WriteMemoryStat(file,"body moves",b2Array_ByteCount(world->bodyMoveEvents),&total);
    b2WriteMemoryStat(file,"contact begins",b2Array_ByteCount(world->contactBeginEvents),&total);
    b2WriteMemoryStat(file,"contact hits",b2Array_ByteCount(world->contactHitEvents),&total);
    b2WriteMemoryStat(file,"contact ends",b2Array_ByteCount(world->contactEndEvents[0])+b2Array_ByteCount(world->contactEndEvents[1]),&total);
    int sensorBytes=b2Array_ByteCount(world->sensors)+b2Array_ByteCount(world->sensorTaskContexts);
    for(int i=0;i<world->sensors.count;++i)
    { b2Sensor* sensor=world->sensors.data+i;sensorBytes+=b2Array_ByteCount(sensor->hits)+b2Array_ByteCount(sensor->overlaps1)+b2Array_ByteCount(sensor->overlaps2); }
    for(int i=0;i<world->sensorTaskContexts.count;++i) { sensorBytes+=b2GetBitSetBytes(&world->sensorTaskContexts.data[i].eventBits); }
    fprintf(file,"\nsensors\n");b2WriteMemoryStat(file,"sensor storage",sensorBytes,&total);
    b2WriteMemoryStat(file,"sensor events",b2Array_ByteCount(world->sensorBeginEvents)+b2Array_ByteCount(world->sensorEndEvents[0])+b2Array_ByteCount(world->sensorEndEvents[1]),&total);
    fprintf(file,"\ntotal: %d\n",total);fclose(file);
}

b2BodyEvents b2World_GetBodyEvents(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);if(world==NULL || world->locked) { return b2BodyEvents{}; }
    return b2BodyEvents{world->bodyMoveEvents.data,world->bodyMoveEvents.count};
}

b2ContactEvents b2World_GetContactEvents(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);if(world==NULL || world->locked) { return b2ContactEvents{}; }
    b2DynamicArray<b2ContactEndTouchEvent>* ends=world->contactEndEvents+(1-world->endEventArrayIndex);
    return b2ContactEvents{world->contactBeginEvents.data,ends->data,world->contactHitEvents.data,world->contactBeginEvents.count,ends->count,world->contactHitEvents.count};
}

void b2World_SetHitEventThreshold(b2WorldId id,float value)
{
    if(!b2IsValidFloat(value)) { return; }b2World* world=b2GetWorldFromId(id);if(world==NULL || world->locked) { return; }
    world->hitEventThreshold=b2ClampFloat(value,0.0f,FLT_MAX);
}

float b2World_GetHitEventThreshold(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);return world!=NULL?world->hitEventThreshold:0.0f;
}

b2SensorEvents b2World_GetSensorEvents(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);if(world==NULL || world->locked) { return b2SensorEvents{}; }
    b2DynamicArray<b2SensorEndTouchEvent>* ends=world->sensorEndEvents+(1-world->endEventArrayIndex);
    return b2SensorEvents{world->sensorBeginEvents.data,ends->data,world->sensorBeginEvents.count,ends->count};
}

void b2World_SetCustomFilterCallback(b2WorldId id,b2CustomFilterCallbackFcn* fcn,void* context)
{
    b2World* world=b2GetWorldFromId(id);if(world==NULL) { return; }world->customFilterFcn=fcn;world->customFilterContext=context;
}

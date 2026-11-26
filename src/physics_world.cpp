// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "world.h"

#include "core.h"
#include "aabb.h"
#include "solver.h"
#include "scheduler.h"
#include "parallel_for.h"
#include "platform.h"

#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <bit>
#include <string.h>
#include <stdio.h>

static_assert( B2_MAX_WORLDS > 0, "must be 1 or more" );
static_assert( B2_MAX_WORLDS < UINT16_MAX, "B2_MAX_WORLDS limit exceeded" );
b2World b2_worlds[B2_MAX_WORLDS];

static void* b2DefaultAddTaskFcn( b2TaskCallbackFcn* task, void* taskContext, void* userContext )
{
	(void)userContext;
	task( taskContext );
	return NULL;
}

static void b2DefaultFinishTaskFcn( void* userTask, void* userContext )
{
	(void)userTask;
	(void)userContext;
}

static void b2ResizeTaskContexts( b2World* world, int count )
{
 int oldCount=world->taskContexts.count;
 for(int i=count;i<oldCount;++i) { b2Array_Destroy(world->taskContexts.data[i].pairKeys);b2DestroyBitSet(&world->taskContexts.data[i].contactStateBitSet);b2DestroyBitSet(&world->taskContexts.data[i].jointStateBitSet);b2DestroyBitSet(&world->taskContexts.data[i].hitEventBitSet); }
 b2Array_Resize(world->taskContexts,count);
 for(int i=oldCount;i<count;++i) { world->taskContexts.data[i]={};b2Array_Create(world->taskContexts.data[i].pairKeys);world->taskContexts.data[i].contactStateBitSet=b2CreateBitSet(1024);world->taskContexts.data[i].jointStateBitSet=b2CreateBitSet(1024);world->taskContexts.data[i].hitEventBitSet=b2CreateBitSet(1024); }
}

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
	world->stepIndex = 0;
	world->splitIslandId = B2_NULL_INDEX;
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
	world->scheduler = NULL;
	world->useScheduler = false;
	if ( def->workerCount > 0 && def->enqueueTask != NULL )
	{
		world->workerCount = b2MinInt( def->workerCount, B2_MAX_WORKERS );
		world->enqueueTaskFcn = def->enqueueTask;
		world->finishTaskFcn = def->finishTask;
		world->userTaskContext = def->userTaskContext;
	}
	else
	{
		world->workerCount = b2ClampInt( def->workerCount, 1, B2_MAX_WORKERS );
		world->enqueueTaskFcn = b2DefaultAddTaskFcn;
		world->finishTaskFcn = b2DefaultFinishTaskFcn;
		world->userTaskContext = NULL;
		if ( world->workerCount > 1 )
		{
			world->useScheduler = true;
			world->enqueueTaskFcn = b2SchedulerEnqueueTask;
			world->finishTaskFcn = b2SchedulerFinishTask;
			world->scheduler = b2CreateScheduler( world->workerCount );
		}
	}
	world->userData = def->userData;
	world->stepContext = static_cast<b2StepContext*>( b2AllocZero( sizeof( b2StepContext ) ) );
	world->stepContext->phase = 0;
	world->stepContext->world = world;
	b2Array_Create(world->taskContexts);
	b2ResizeTaskContexts(world,world->workerCount);
	world->bodyIdPool = b2CreateIdPool();
	b2Array_Create( world->bodies );
	world->shapeIdPool = b2CreateIdPool();
	b2Array_Create( world->shapes );
	b2Array_Create( world->fatAABBs );
	world->jointIdPool=b2CreateIdPool();b2Array_Create(world->joints);
	world->chainIdPool = b2CreateIdPool();
	b2Array_Create( world->chainShapes );
	b2CreateBroadPhase( &world->broadPhase, &def->capacity );
	world->contactIdPool = b2CreateIdPool();
	b2Array_Create( world->contacts );
	b2Array_CreateN(world->sensors,4);b2Array_CreateN(world->sensorTaskContexts,1);
    for(int i=0;i<world->workerCount;++i) { b2SensorTaskContext sensorContext={};sensorContext.eventBits=b2CreateBitSet(16);b2Array_Push(world->sensorTaskContexts,sensorContext); }
    b2Array_CreateN(world->sensorBeginEvents,4);b2Array_CreateN(world->sensorEndEvents[0],4);b2Array_CreateN(world->sensorEndEvents[1],4);
	b2Array_CreateN(world->bodyMoveEvents,4);
	b2Array_CreateN(world->contactBeginEvents,4);
	b2Array_CreateN(world->contactHitEvents,4);
	b2Array_CreateN(world->jointEvents,4);
	world->jointStateBitSet=b2CreateBitSet(1024);
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
	b2DestroyBitSet(&world->debugBodySet);
	b2DestroyBitSet(&world->debugContactSet);
	b2DestroyBitSet(&world->debugIslandSet);
	b2DestroyBitSet(&world->debugJointSet);
	b2Array_Destroy(world->bodyMoveEvents);
	b2Array_Destroy(world->contactBeginEvents);
	b2Array_Destroy(world->contactHitEvents);
	b2Array_Destroy(world->jointEvents);
	b2DestroyBitSet(&world->jointStateBitSet);
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
	for(int i=0;i<world->taskContexts.count;++i) { b2Array_Destroy(world->taskContexts.data[i].pairKeys);b2DestroyBitSet(&world->taskContexts.data[i].contactStateBitSet);b2DestroyBitSet(&world->taskContexts.data[i].jointStateBitSet);b2DestroyBitSet(&world->taskContexts.data[i].hitEventBitSet); }
	b2Array_Destroy(world->taskContexts);
	b2DestroyStack( &world->stack );
	b2Array_Destroy( world->bodies );
    for ( int i = 0; i < world->chainShapes.count; ++i )
    {
        b2ChainShape* chain = world->chainShapes.data + i;
        b2Free( chain->shapeIndices, chain->segmentCount * sizeof( int ) );
    }
    b2Array_Destroy( world->chainShapes );
    b2Array_Destroy(world->joints);b2DestroyIdPool(&world->jointIdPool);
    b2DestroyIdPool( &world->chainIdPool );
	b2Array_Destroy( world->shapes );
	b2Array_Destroy( world->fatAABBs );
	b2DestroyIdPool( &world->shapeIdPool );
	b2DestroyIdPool( &world->bodyIdPool );
	b2Free( world->stepContext, sizeof( b2StepContext ) );
	if ( world->scheduler != NULL )
	{
		b2DestroyScheduler( world->scheduler );
		world->scheduler = NULL;
	}
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

void b2World_SetWorkerCount( b2WorldId worldId, int count )
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( world == NULL || world->locked )
	{
		return;
	}

	int clampedCount = b2ClampInt( count, 1, B2_MAX_WORKERS );
	if ( clampedCount == world->workerCount )
	{
		return;
	}

	if ( clampedCount == 1 && world->useScheduler )
	{
		world->useScheduler = false;
		world->enqueueTaskFcn = b2DefaultAddTaskFcn;
		world->finishTaskFcn = b2DefaultFinishTaskFcn;
		world->userTaskContext = NULL;
	}
	else if ( clampedCount > 1 && world->enqueueTaskFcn == b2DefaultAddTaskFcn )
	{
		world->useScheduler = true;
		world->enqueueTaskFcn = b2SchedulerEnqueueTask;
		world->finishTaskFcn = b2SchedulerFinishTask;
		world->userTaskContext = NULL;
	}

	// 线程数变化后旧调度器的线程与资源全部迁移。
	if ( world->scheduler != NULL )
	{
		b2DestroyScheduler( world->scheduler );
		world->scheduler = NULL;
	}
	if ( world->useScheduler && clampedCount > 1 )
	{
		world->scheduler = b2CreateScheduler( clampedCount );
	}
 int oldSensorCount=world->sensorTaskContexts.count;
 for(int i=clampedCount;i<oldSensorCount;++i) { b2DestroyBitSet(&world->sensorTaskContexts.data[i].eventBits); }
 b2Array_Resize(world->sensorTaskContexts,clampedCount);
 for(int i=oldSensorCount;i<clampedCount;++i) { world->sensorTaskContexts.data[i].eventBits=b2CreateBitSet(16); }
 b2ResizeTaskContexts(world,clampedCount);
	world->workerCount = clampedCount;
}

int b2World_GetWorkerCount( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	return world != NULL ? world->workerCount : 0;
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


static b2TaskBatch b2ContinueStepInternal( b2World* world );

static b2TaskBatch b2StartStep( b2World* world, float timeStep, int subStepCount )
{
	world->inv_dt = 1.0f / timeStep;
	world->inv_h = subStepCount / timeStep;
	b2Array_Clear( world->bodyMoveEvents );
	b2Array_Clear( world->contactBeginEvents );
	b2Array_Clear( world->sensorBeginEvents );
	b2Array_Clear( world->contactHitEvents );
	b2Array_Clear( world->jointEvents );
	for ( int i = 0; i < world->bodies.count; ++i )
	{
		world->bodies.data[i].bodyMoveIndex = B2_NULL_INDEX;
	}

	world->profile = b2Profile{};
	world->locked = true;
	{
		b2Capacity* c = &world->maxCapacity;
		c->staticShapeCount = b2MaxInt( c->staticShapeCount, world->broadPhase.trees[b2_staticBody].proxyCount );
		c->dynamicShapeCount = b2MaxInt( c->dynamicShapeCount, world->broadPhase.trees[b2_dynamicBody].proxyCount );

		int staticBodyCount = world->solverSets.data[b2_staticSet].bodySims.count;
		c->staticBodyCount = b2MaxInt( c->staticBodyCount, staticBodyCount );

		int totalBodyCount = b2GetIdCount( &world->bodyIdPool );
		c->dynamicBodyCount = b2MaxInt( c->dynamicBodyCount, totalBodyCount - staticBodyCount );

		int totalContactCount = b2GetIdCount( &world->contactIdPool );
		c->contactCount = b2MaxInt( c->contactCount, totalContactCount );
		c->jointCount = b2MaxInt( c->jointCount, b2GetIdCount( &world->jointIdPool ) );
	}

	b2StepContext* context = world->stepContext;
	context->timeStep = timeStep;
	context->subStepCount = subStepCount;
	context->phase = 1;
	context->stepTicks=b2GetTicks();
	context->pairTicks=context->stepTicks;
 for(int i=0;i<world->contacts.count;++i)
 {
  b2Contact* contact=world->contacts.data+i;
  if(contact->contactId!=B2_NULL_INDEX && !b2AABB_Overlaps(world->fatAABBs.data[contact->shapeIdA],world->fatAABBs.data[contact->shapeIdB])) { b2DestroyContact(world,contact); }
 }
 return b2ContinueStepInternal(world);
}

static b2TaskBatch b2ContinueStepInternal( b2World* world )
{
	b2StepContext* context = world->stepContext;
 for(;;)
 {
  b2ResetTaskBatch(context);
  switch(context->phase)
  {
   case 1: context->phase=2;context->pairsUpdated=b2BeginPairs(world);break;
   case 2: context->phase=3;if(context->pairsUpdated) { b2StageSelfPairs(world); }break;
   case 3:
    if(context->pairsUpdated) { b2EndPairs(world); }
    world->profile.pairs=b2GetMilliseconds(context->pairTicks);
    context->phase=4;context->collideTicks=b2GetTicks();b2BeginCollide(context);break;
   case 4:
    b2EndCollide(context);world->profile.collide=b2GetMilliseconds(context->collideTicks);
    { uint64_t ticks=b2GetTicks();b2SplitContactIslands(world);world->profile.splitIslands=b2GetMilliseconds(ticks); }
    context->phase=5;context->solveTicks=b2GetTicks();b2BeginSolve(world,context);break;
   case 5:
    if(b2ContinueSolve(context)==false)
    {
     b2EndSolve(context);world->profile.solve=b2GetMilliseconds(context->solveTicks);
     context->phase=6;context->sensorTicks=b2GetTicks();b2BeginSensors(world);
    }
    break;
   case 6:b2EndSensors(world);world->profile.sensors=b2GetMilliseconds(context->sensorTicks);context->phase=0;break;
   default:return b2TaskBatch{};
  }
  if(context->batchTaskCount>0)
  {
   b2AtomicStoreInt(&context->pendingCount,context->batchTaskCount);
   return b2MakeTaskBatch(context);
  }
  if(context->phase==0) { break; }
 }

	uint64_t stageTicks = b2GetTicks();

	world->maxCapacity.arenaByteCount = b2MaxInt( world->maxCapacity.arenaByteCount, b2GetMaxStackAllocation( &world->stack ) );

	world->stepIndex += 1;
	world->endEventArrayIndex = 1 - world->endEventArrayIndex;
	b2Array_Clear( world->contactEndEvents[world->endEventArrayIndex] );
	b2Array_Clear( world->sensorEndEvents[world->endEventArrayIndex] );
	world->profile.step = b2GetMilliseconds( context->stepTicks );
	world->locked = false;
	return b2TaskBatch{};
}

b2TaskBatch b2World_BeginStep( b2WorldId id, float timeStep, int subStepCount )
{
	if ( b2World_IsValid( id ) == false || b2IsValidFloat( timeStep ) == false || timeStep <= 0.0f || subStepCount <= 0 )
	{
		return b2TaskBatch{};
	}
	b2World* world = b2_worlds + id.index1 - 1;
	if ( world->locked )
	{
		return b2TaskBatch{};
	}
	return b2StartStep( world, timeStep, subStepCount );
}

b2TaskBatch b2World_ContinueStep( b2WorldId id )
{
	if ( b2World_IsValid( id ) == false )
	{
		return b2TaskBatch{};
	}
	b2World* world = b2_worlds + id.index1 - 1;
	return b2ContinueStepInternal( world );
}

void b2World_Step( b2WorldId id, float timeStep, int subStepCount )
{
	if ( b2World_IsValid( id ) == false || b2IsValidFloat( timeStep ) == false || timeStep <= 0.0f || subStepCount <= 0 )
	{
		return;
	}
	b2World* world = b2_worlds + id.index1 - 1;
	if ( world->locked )
	{
		return;
	}
	if ( world->useScheduler && world->scheduler == NULL && world->workerCount > 1 )
	{
		world->scheduler = b2CreateScheduler( world->workerCount );
	}

	uint64_t stepTicks = b2GetTicks();
 if(world->useScheduler)
 {
  b2ResetScheduler(world->scheduler);world->userTaskContext=world->scheduler;
 }
 b2TaskBatch batch=b2StartStep(world,timeStep,subStepCount);
 while(batch.count>0)
 {
  // 上一批已全部接合，可复用调度器槽位，避免大量颜色/子步耗尽固定队列。
  if(world->useScheduler) { b2ResetScheduler(world->scheduler); }
  b2RunTaskBatch(world->enqueueTaskFcn,world->finishTaskFcn,world->userTaskContext,batch);
  batch=b2ContinueStepInternal(world);
 }
	world->profile.step = b2GetMilliseconds( stepTicks );
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
    counters.jointCount=b2GetIdCount(&world->jointIdPool);
    counters.contactCount=b2GetIdCount(&world->contactIdPool);counters.islandCount=b2GetIdCount(&world->islandIdPool);
    counters.staticTreeHeight=b2DynamicTree_GetHeight(world->broadPhase.trees+b2_staticBody);
    counters.treeHeight=b2MaxInt(b2DynamicTree_GetHeight(world->broadPhase.trees+b2_dynamicBody),b2DynamicTree_GetHeight(world->broadPhase.trees+b2_kinematicBody));
    counters.stackUsed=b2GetMaxStackAllocation(&world->stack);counters.byteCount=b2GetByteCount();
    counters.recycledContactCount=world->recycledContactCount;
    for(int i=0;i<B2_GRAPH_COLOR_COUNT;++i)
    {
        counters.colorCounts[i]=world->constraintGraph.colors[i].contactSims.count+world->constraintGraph.colors[i].jointSims.count;
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
    b2WriteMemoryStat(file,"joint ids",b2GetIdBytes(&world->jointIdPool),&total);
    fprintf(file,"\nworld arrays\n");
    b2WriteMemoryStat(file,"joints",b2Array_ByteCount(world->joints),&total);
    b2WriteMemoryStat(file,"bodies",b2Array_ByteCount(world->bodies),&total);
    b2WriteMemoryStat(file,"solver sets",b2Array_ByteCount(world->solverSets),&total);
    b2WriteMemoryStat(file,"contacts",b2Array_ByteCount(world->contacts),&total);
    b2WriteMemoryStat(file,"islands",b2Array_ByteCount(world->islands),&total);
    b2WriteMemoryStat(file,"shapes",b2Array_ByteCount(world->shapes),&total);
    b2WriteMemoryStat(file,"fat AABBs",b2Array_ByteCount(world->fatAABBs),&total);
    b2WriteMemoryStat(file,"chains",b2Array_ByteCount(world->chainShapes),&total);
    int islandBytes=0,chainBytes=0;
    for(int i=0;i<world->islands.count;++i)
    { islandBytes+=b2Array_ByteCount(world->islands.data[i].bodies)+b2Array_ByteCount(world->islands.data[i].contacts)+b2Array_ByteCount(world->islands.data[i].joints); }
    for(int i=0;i<world->chainShapes.count;++i)
    { if(world->chainShapes.data[i].id!=B2_NULL_INDEX) { chainBytes+=world->chainShapes.data[i].segmentCount*sizeof(int); } }
    b2WriteMemoryStat(file,"island links",islandBytes,&total);
    fprintf(file,"\nowned arrays\n");b2WriteMemoryStat(file,"chain data",chainBytes,&total);
    fprintf(file,"\nbroad-phase\n");
    b2WriteMemoryStat(file,"static tree",(int)b2DynamicTree_GetByteCount(world->broadPhase.trees+b2_staticBody),&total);
    b2WriteMemoryStat(file,"kinematic tree",(int)b2DynamicTree_GetByteCount(world->broadPhase.trees+b2_kinematicBody),&total);
    b2WriteMemoryStat(file,"dynamic tree",(int)b2DynamicTree_GetByteCount(world->broadPhase.trees+b2_dynamicBody),&total);
    b2WriteMemoryStat(file,"pairSet",b2GetHashSetBytes(&world->broadPhase.pairSet),&total);
    int bodyBytes=0,stateBytes=0,contactBytes=0,jointBytes=0,islandSimBytes=0;
    for(int i=0;i<world->solverSets.count;++i)
    {
        b2SolverSet* set=world->solverSets.data+i;if(set->setIndex==B2_NULL_INDEX) { continue; }
        jointBytes+=b2Array_ByteCount(set->jointSims);bodyBytes+=b2Array_ByteCount(set->bodySims);stateBytes+=b2Array_ByteCount(set->bodyStates);
        contactBytes+=b2Array_ByteCount(set->contactSims);islandSimBytes+=b2Array_ByteCount(set->islandSims);
    }
    fprintf(file,"\nsolver sets\n");
    b2WriteMemoryStat(file,"body sim",bodyBytes,&total);b2WriteMemoryStat(file,"body state",stateBytes,&total);
    b2WriteMemoryStat(file,"contact sim",contactBytes,&total);b2WriteMemoryStat(file,"island sim",islandSimBytes,&total);
    b2WriteMemoryStat(file,"joint sims",jointBytes,&total);
    int bitBytes=0,graphBytes=0;
    for(int i=0;i<B2_GRAPH_COLOR_COUNT;++i)
    { bitBytes+=b2GetBitSetBytes(&world->constraintGraph.colors[i].bodySet);graphBytes+=b2Array_ByteCount(world->constraintGraph.colors[i].contactSims)+b2Array_ByteCount(world->constraintGraph.colors[i].jointSims); }
    fprintf(file,"\nconstraint graph\n");b2WriteMemoryStat(file,"body bit sets",bitBytes,&total);b2WriteMemoryStat(file,"contact sim",graphBytes,&total);
    fprintf(file,"\n");b2WriteMemoryStat(file,"stack allocator",world->stack.capacity,&total);
    fprintf(file,"\nevents\n");b2WriteMemoryStat(file,"body moves",b2Array_ByteCount(world->bodyMoveEvents),&total);
    b2WriteMemoryStat(file,"contact begins",b2Array_ByteCount(world->contactBeginEvents),&total);
    b2WriteMemoryStat(file,"contact hits",b2Array_ByteCount(world->contactHitEvents),&total);
    b2WriteMemoryStat(file,"contact ends",b2Array_ByteCount(world->contactEndEvents[0])+b2Array_ByteCount(world->contactEndEvents[1]),&total);
    b2WriteMemoryStat(file,"joint events",b2Array_ByteCount(world->jointEvents),&total);
    b2WriteMemoryStat(file,"joint state bits",b2GetBitSetBytes(&world->jointStateBitSet),&total);
    int sensorBytes=b2Array_ByteCount(world->sensors)+b2Array_ByteCount(world->sensorTaskContexts);
    for(int i=0;i<world->sensors.count;++i)
    { b2Sensor* sensor=world->sensors.data+i;sensorBytes+=b2Array_ByteCount(sensor->hits)+b2Array_ByteCount(sensor->overlaps1)+b2Array_ByteCount(sensor->overlaps2); }
    for(int i=0;i<world->sensorTaskContexts.count;++i) { sensorBytes+=b2GetBitSetBytes(&world->sensorTaskContexts.data[i].eventBits); }
    fprintf(file,"\nsensors\n");b2WriteMemoryStat(file,"sensor storage",sensorBytes,&total);
    b2WriteMemoryStat(file,"sensor events",b2Array_ByteCount(world->sensorBeginEvents)+b2Array_ByteCount(world->sensorEndEvents[0])+b2Array_ByteCount(world->sensorEndEvents[1]),&total);
    b2WriteMemoryStat(file,"debug bit sets",b2GetBitSetBytes(&world->debugBodySet)+b2GetBitSetBytes(&world->debugJointSet)+b2GetBitSetBytes(&world->debugContactSet)+b2GetBitSetBytes(&world->debugIslandSet),&total);
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

b2JointEvents b2World_GetJointEvents(b2WorldId id)
{
    b2World* world=b2GetWorldFromId(id);if(world==NULL || world->locked) { return b2JointEvents{}; }
    return b2JointEvents{world->jointEvents.data,world->jointEvents.count};
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

void b2World_SetPreSolveCallback(b2WorldId id,b2PreSolveCallbackFcn* preSolve,b2PreContinuousCallbackFcn* preContinuous,void* context)
{
    b2World* world=b2GetWorldFromId(id);if(world==NULL) { return; }
    world->preSolveFcn=preSolve;world->preContinuousFcn=preContinuous;world->preSolveContext=context;
}

typedef struct WorldQueryContext
{
	b2World* world;
	b2OverlapResultCallbackFcn* fcn;
	b2QueryFilter filter;
	void* userContext;
} WorldQueryContext;

static bool TreeQueryCallback( int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;

	int shapeId = (int)userData;

	WorldQueryContext* worldContext = static_cast<WorldQueryContext *>( context );
	b2World* world = worldContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	if ( b2ShouldQueryCollide( shape->filter, worldContext->filter ) == false )
	{
		return true;
	}

	b2ShapeId id = { shapeId + 1, world->worldId, shape->generation };
	bool result = worldContext->fcn( id, worldContext->userContext );
	return result;
}

b2TreeStats b2World_OverlapAABB( b2WorldId worldId, b2Pos origin, b2AABB aabb, b2QueryFilter filter,
								 b2OverlapResultCallbackFcn* fcn, void* context )
{
	b2TreeStats treeStats = {};

	b2World* world = b2GetWorldFromId( worldId );

	if ( world==NULL || world->locked || fcn==NULL )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidVec2( origin ) );
	B2_ASSERT( b2IsValidAABB( aabb ) );



	// 向外取整提升到世界 float 盒，使保守树测试永不漏检
	b2AABB worldBox = b2OffsetAABB( aabb, origin );

	WorldQueryContext worldContext = { world, fcn, filter, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult = b2DynamicTree_OverlapAABB( world->broadPhase.trees + i, worldBox, filter.maskBits,
															TreeQueryCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}



	return treeStats;
}

typedef struct WorldOverlapContext
{
	b2World* world;
	b2OverlapResultCallbackFcn* fcn;
	b2QueryFilter filter;
	const b2ShapeProxy* proxy;
	b2Pos origin;
	void* userContext;
} WorldOverlapContext;

static bool TreeOverlapCallback( int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;

	int shapeId = (int)userData;

	WorldOverlapContext* worldContext = static_cast<WorldOverlapContext *>( context );
	b2World* world = worldContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	if ( b2ShouldQueryCollide( shape->filter, worldContext->filter ) == false )
	{
		return true;
	}

	// 以查询原点重定心，使远离世界原点时距离测试仍保持 float 精度
	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2Transform transform = b2ToRelativeTransform( b2GetBodySim(world,body)->transform, worldContext->origin );

	b2DistanceInput input;
	input.proxyA = *worldContext->proxy;
	input.proxyB = b2MakeShapeDistanceProxy( shape );
	input.transform = transform;
	input.useRadii = true;

	b2SimplexCache cache = {};
	b2DistanceOutput output = b2ComputeShapeDistance( &input, &cache, NULL, 0 );

	float tolerance = 0.1f * B2_LINEAR_SLOP;
	if ( output.distance > tolerance )
	{
		return true;
	}

	b2ShapeId id = { shape->id + 1, world->worldId, shape->generation };
	bool result = worldContext->fcn( id, worldContext->userContext );
	return result;
}

b2TreeStats b2World_OverlapShape( b2WorldId worldId, b2Pos origin, const b2ShapeProxy* proxy, b2QueryFilter filter,
								  b2OverlapResultCallbackFcn* fcn, void* context )
{
	b2TreeStats treeStats = {};

	b2World* world = b2GetWorldFromId( worldId );

	if ( world==NULL || world->locked || fcn==NULL || proxy==NULL )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidVec2( origin ) );



	// 相对盒向外取整提升到世界 float，对树保守
	b2AABB aabb = b2OffsetAABB( b2MakeAABB( proxy->points, proxy->count, proxy->radius ), origin );
	WorldOverlapContext worldContext = {
		world, fcn, filter, proxy, origin, context,
	};

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult = b2DynamicTree_OverlapAABB( world->broadPhase.trees + i, aabb, filter.maskBits,
															TreeOverlapCallback, &worldContext );

		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;
	}



	return treeStats;
}

typedef struct WorldRayCastContext
{
	b2World* world;
	b2CastResultCallbackFcn* fcn;
	b2QueryFilter filter;
	float fraction;
	b2Pos origin;
	void* userContext;
} WorldRayCastContext;

static float RayCastCallback( const b2RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;

	int shapeId = (int)userData;

	WorldRayCastContext* worldContext = static_cast<WorldRayCastContext *>( context );
	b2World* world = worldContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	if ( b2ShouldQueryCollide( shape->filter, worldContext->filter ) == false )
	{
		return input->maxFraction;
	}

	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2WorldTransform bodyTransform = b2GetBodySim(world,body)->transform;
	b2Transform transform = b2ToRelativeTransform( bodyTransform, worldContext->origin );

	b2RayCastInput localInput = *input;
	localInput.origin = b2Vec2_zero;
	b2CastOutput output = b2RayCastShape( &localInput, shape, transform );

	if ( output.hit )
	{
		b2ShapeId id = { shapeId + 1, world->worldId, shape->generation };
		b2Pos point = b2OffsetPos( worldContext->origin, output.point );
		float fraction = worldContext->fcn( id, point, output.normal, output.fraction, worldContext->userContext );

		// 用户可返回 -1 以跳过此形状
		if ( 0.0f <= fraction && fraction <= 1.0f )
		{
			worldContext->fraction = fraction;
		}

		return fraction;
	}

	return input->maxFraction;
}

b2TreeStats b2World_CastRay( b2WorldId worldId, b2Pos origin, b2Vec2 translation, b2QueryFilter filter,
							 b2CastResultCallbackFcn* fcn, void* context )
{
	b2TreeStats treeStats = {};

	b2World* world = b2GetWorldFromId( worldId );

	if ( world==NULL || world->locked || fcn==NULL )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidVec2( origin ) );
	B2_ASSERT( b2IsValidVec2( translation ) );



	// 树遍历看到的原点被截断为 float，射线最多偏移一个坐标 ULP，
	// 在极端距离上相当于擦边漏检容差。每形状投射相对上下文携带的全精度原点重新做差。
	b2RayCastInput input = { b2ToVec2( origin ), translation, 1.0f };

	WorldRayCastContext worldContext = { world, fcn, filter, 1.0f, origin, context };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult = b2DynamicTree_CastRay( world->broadPhase.trees + i, &input, filter.maskBits,
														RayCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			break;
		}

		input.maxFraction = worldContext.fraction;
	}



	return treeStats;
}

// 此回调找出最近命中。这是游戏中最常用的回调。
static float b2RayCastClosestFcn( b2ShapeId shapeId, b2Pos point, b2Vec2 normal, float fraction, void* context )
{
	// 忽略初始重叠
	if ( fraction == 0.0f )
	{
		return -1.0f;
	}

	b2RayResult* rayResult = (b2RayResult*)context;
	rayResult->shapeId = shapeId;
	rayResult->point = point;
	rayResult->normal = normal;
	rayResult->fraction = fraction;
	rayResult->hit = true;
	return fraction;
}

b2RayResult b2World_CastRayClosest( b2WorldId worldId, b2Pos origin, b2Vec2 translation, b2QueryFilter filter )
{
	b2RayResult result = {};

	b2World* world = b2GetWorldFromId( worldId );

	if ( world == nullptr || world->locked )
	{
		return result;
	}

	B2_ASSERT( b2IsValidVec2( origin ) );
	B2_ASSERT( b2IsValidVec2( translation ) );

	b2RayCastInput input = { b2ToVec2( origin ), translation, 1.0f };
	WorldRayCastContext worldContext = { world, b2RayCastClosestFcn, filter, 1.0f, origin, &result };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult = b2DynamicTree_CastRay( world->broadPhase.trees + i, &input, filter.maskBits,
														RayCastCallback, &worldContext );
		result.nodeVisits += treeResult.nodeVisits;
		result.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			break;
		}

		input.maxFraction = worldContext.fraction;
	}



	return result;
}

typedef struct WorldShapeCastContext
{
	b2World* world;
	b2CastResultCallbackFcn* fcn;
	b2QueryFilter filter;
	float fraction;
	b2Pos origin;
	// 相对原点的输入
	b2ShapeCastInput input;
	void* userContext;
} WorldShapeCastContext;

static float ShapeCastCallback( const b2BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;

	int shapeId = (int)userData;

	WorldShapeCastContext* worldContext = static_cast<WorldShapeCastContext *>( context );
	b2World* world = worldContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	if ( b2ShouldQueryCollide( shape->filter, worldContext->filter ) == false )
	{
		return input->maxFraction;
	}

	// 从相对原点的输入重建，只从树取推进分数。
	// 树输入是世界 float，远离原点时会丢失投射。
	b2ShapeCastInput localInput = worldContext->input;
	localInput.maxFraction = input->maxFraction;

	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2WorldTransform transform = b2GetBodySim(world,body)->transform;
	b2Transform localTransform = b2ToRelativeTransform( transform, worldContext->origin );

	b2CastOutput output = b2ShapeCastShape( &localInput, shape, localTransform );

	if ( output.hit )
	{
		b2ShapeId id = { shapeId + 1, world->worldId, shape->generation };

		b2Pos point = b2OffsetPos( worldContext->origin, output.point );
		float fraction = worldContext->fcn( id, point, output.normal, output.fraction, worldContext->userContext );

		// 用户可返回 -1 以跳过此形状
		if ( 0.0f <= fraction && fraction <= 1.0f )
		{
			worldContext->fraction = fraction;
		}

		return fraction;
	}

	return input->maxFraction;
}

b2TreeStats b2World_CastShape( b2WorldId worldId, b2Pos origin, const b2ShapeProxy* proxy, b2Vec2 translation,
							   b2QueryFilter filter, b2CastResultCallbackFcn* fcn, void* context )
{
	b2TreeStats treeStats = {};

	b2World* world = b2GetWorldFromId( worldId );

	if ( world == nullptr || world->locked || fcn == nullptr )
	{
		return treeStats;
	}

	B2_ASSERT( b2IsValidVec2( origin ) );
	B2_ASSERT( b2IsValidVec2( translation ) );




	WorldShapeCastContext worldContext = {};
	worldContext.world = world;
	worldContext.fcn = fcn;
	worldContext.filter = filter;
	worldContext.fraction = 1.0f;
	worldContext.origin = origin;
	worldContext.input.proxy = *proxy;
	worldContext.input.translation = translation;
	worldContext.input.maxFraction = 1.0f;
	worldContext.userContext = context;

	// 从代理和原点创建保守的世界空间盒。
	b2AABB localBox = b2MakeAABB( proxy->points, proxy->count, proxy->radius );
	b2AABB box = b2OffsetAABB( localBox, origin );
	b2BoxCastInput treeInput = { box, translation, 1.0f };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2TreeStats treeResult = b2DynamicTree_CastBox( world->broadPhase.trees + i, &treeInput, filter.maskBits,
														ShapeCastCallback, &worldContext );
		treeStats.nodeVisits += treeResult.nodeVisits;
		treeStats.leafVisits += treeResult.leafVisits;

		if ( worldContext.fraction == 0.0f )
		{
			break;
		}

		treeInput.maxFraction = worldContext.fraction;
	}



	return treeStats;
}

struct ExplosionContext
{
	b2World* world;
	b2Pos position;
	float radius;
	float falloff;
	float impulsePerLength;
};

static bool ExplosionCallback( int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;

	int shapeId = (int)userData;
	struct ExplosionContext* explosionContext = static_cast<ExplosionContext *>( context );
	b2World* world = explosionContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	B2_ASSERT( body->type == b2_dynamicBody );

	b2WorldTransform xf = b2GetBodySim(world,body)->transform;

	// 将爆炸重定心到形状局部坐标系，使远离原点时距离和方向仍精确。
	// 下面全部在该近原点坐标系中运行。
	b2Vec2 localPosition = b2InvTransformWorldPoint( xf, explosionContext->position );

	b2DistanceInput input;
	input.proxyA = b2MakeShapeDistanceProxy( shape );
	input.proxyB = b2MakeProxy( &localPosition, 1, 0.0f );
	input.transform = b2Transform_identity;
	input.useRadii = true;

	b2SimplexCache cache = {};
	b2DistanceOutput output = b2ComputeShapeDistance( &input, &cache, NULL, 0 );

	float radius = explosionContext->radius;
	float falloff = explosionContext->falloff;
	if ( output.distance > radius + falloff )
	{
		return true;
	}

	b2WakeBody( world, body );

	if ( body->setIndex != b2_awakeSet )
	{
		return true;
	}

	b2Vec2 closestPoint = output.pointA;
	if ( output.distance == 0.0f )
	{
		closestPoint = b2GetShapeCentroid( shape );
	}

	b2Vec2 direction = b2Sub( closestPoint, localPosition );
	if ( b2LengthSquared( direction ) > 100.0f * FLT_EPSILON * FLT_EPSILON )
	{
		direction = b2Normalize( direction );
	}
	else
	{
		direction = b2Vec2{ 1.0f, 0.0f };
	}

	b2Vec2 localLine = b2LeftPerp( direction );
	float perimeter = b2GetShapeProjectedPerimeter( shape, localLine );
	float scale = 1.0f;
	if ( output.distance > radius && falloff > 0.0f )
	{
		scale = b2ClampFloat( ( radius + falloff - output.distance ) / falloff, 0.0f, 1.0f );
	}

	float magnitude = explosionContext->impulsePerLength * perimeter * scale;
	b2Vec2 impulse = b2MulSV( magnitude, b2RotateVector( xf.q, direction ) );

	int localIndex = body->localIndex;
	b2SolverSet* set = b2Array_Get( world->solverSets, b2_awakeSet );
	b2BodyState* state = b2Array_Get( set->bodyStates, localIndex );
	b2BodySim* bodySim = b2Array_Get( set->bodySims, localIndex );
	state->linearVelocity = b2MulAdd( state->linearVelocity, bodySim->invMass, impulse );

	// 从质心到最近点的力臂，旋转到世界
	b2Vec2 r = b2RotateVector( xf.q, b2Sub( closestPoint, bodySim->localCenter ) );
	state->angularVelocity += bodySim->invInertia * b2Cross( r, impulse );

	return true;
}

void b2World_Explode( b2WorldId worldId, const b2ExplosionDef* explosionDef )
{
	uint64_t maskBits = explosionDef->maskBits;
	b2Pos position = explosionDef->position;
	float radius = explosionDef->radius;
	float falloff = explosionDef->falloff;
	float impulsePerLength = explosionDef->impulsePerLength;

	if (!b2IsValidVec2(position)) { return; }
	if (!b2IsValidFloat(radius) || radius < 0.0f) { return; }
	if (!b2IsValidFloat(falloff) || falloff < 0.0f) { return; }
	if (!b2IsValidFloat(impulsePerLength)) { return; }

	b2World* world = b2GetWorldFromId( worldId );

	if ( world == nullptr || world->locked )
	{
		return;
	}



	struct ExplosionContext explosionContext = { world, position, radius, falloff, impulsePerLength };

	// 粗检测树是 float，因此将局部查询盒向外取整平移到世界。
	float extent = radius + falloff;
	b2AABB localBox = { { -extent, -extent }, { extent, extent } };
	b2AABB aabb = b2OffsetAABB( localBox, position );

	b2DynamicTree_OverlapAABB( world->broadPhase.trees + b2_dynamicBody, aabb, maskBits, ExplosionCallback,
							   &explosionContext );
}

typedef struct WorldMoverCastContext
{
	b2World* world;
	b2QueryFilter filter;
	float fraction;
	b2Pos origin;
	// 相对原点的输入
	b2ShapeCastInput input;
} WorldMoverCastContext;

static float MoverCastCallback( const b2BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;

	int shapeId = (int)userData;
	WorldMoverCastContext* worldContext = static_cast<WorldMoverCastContext *>( context );
	b2World* world = worldContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	if ( b2ShouldQueryCollide( shape->filter, worldContext->filter ) == false )
	{
		return worldContext->fraction;
	}

	// 从相对原点的输入重建，只从树取推进分数
	b2ShapeCastInput localInput = worldContext->input;
	localInput.maxFraction = input->maxFraction;

	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2WorldTransform bodyTransform = b2GetBodySim(world,body)->transform;
	b2Transform transform = b2ToRelativeTransform( bodyTransform, worldContext->origin );

	b2CastOutput output = b2ShapeCastShape( &localInput, shape, transform );
	if ( output.fraction == 0.0f )
	{
		// 忽略重叠形状
		return worldContext->fraction;
	}

	worldContext->fraction = output.fraction;
	return output.fraction;
}

float b2World_CastMover( b2WorldId worldId, b2Pos origin, const b2Capsule* mover, b2Vec2 translation,
						 b2QueryFilter filter )
{
	B2_ASSERT( b2IsValidVec2( origin ) );
	B2_ASSERT( b2IsValidVec2( translation ) );
	B2_ASSERT( mover->radius > 2.0f * B2_LINEAR_SLOP );

	b2World* world = b2GetWorldFromId( worldId );

	if ( world == nullptr || world->locked )
	{
		return 1.0f;
	}

	WorldMoverCastContext worldContext = {};
	worldContext.world = world;
	worldContext.filter = filter;
	worldContext.fraction = 1.0f;
	worldContext.origin = origin;
	worldContext.input.proxy.points[0] = mover->center1;
	worldContext.input.proxy.points[1] = mover->center2;
	worldContext.input.proxy.count = 2;
	worldContext.input.proxy.radius = mover->radius;
	worldContext.input.translation = translation;
	worldContext.input.maxFraction = 1.0f;
	worldContext.input.canEncroach = true;

	// 在相对原点空间中包围胶囊，再提升到保守的世界 float 盒
	b2Vec2 centers[2] = { mover->center1, mover->center2 };
	b2AABB box = b2OffsetAABB( b2MakeAABB( centers, 2, mover->radius ), origin );
	b2BoxCastInput treeInput = { box, translation, 1.0f };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_CastBox( world->broadPhase.trees + i, &treeInput, filter.maskBits, MoverCastCallback,
							   &worldContext );

		if ( worldContext.fraction == 0.0f )
		{
			break;
		}

		treeInput.maxFraction = worldContext.fraction;
	}



	return worldContext.fraction;
}

typedef struct WorldMoverContext
{
	b2World* world;
	b2PlaneResultCallbackFcn* fcn;
	b2QueryFilter filter;
	b2Capsule mover;
	b2Pos origin;
	void* userContext;
} WorldMoverContext;

static bool TreeCollideCallback( int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;

	int shapeId = (int)userData;
	WorldMoverContext* worldContext = (WorldMoverContext*)context;
	b2World* world = worldContext->world;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );

	if ( b2ShouldQueryCollide( shape->filter, worldContext->filter ) == false )
	{
		return true;
	}

	// 以查询原点重定心，移动体和结果平面相对原点
	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2Transform transform = b2ToRelativeTransform( b2GetBodySim(world,body)->transform, worldContext->origin );

	b2PlaneResult result = b2CollideMover( &worldContext->mover, shape, transform );

	// todo 处理深度重叠
	if ( result.hit && b2IsNormalized( result.plane.normal ) )
	{
		b2ShapeId id = { shape->id + 1, world->worldId, shape->generation };
		return worldContext->fcn( id, &result, worldContext->userContext );
	}

	return true;
}

// 很想对移动体使用形状代理，但这使处理深度重叠变得困难，通用性可能不值得。
void b2World_CollideMover( b2WorldId worldId, b2Pos origin, const b2Capsule* mover, b2QueryFilter filter,
						   b2PlaneResultCallbackFcn* fcn, void* context )
{
	b2World* world = b2GetWorldFromId( worldId );

	if ( world == nullptr || world->locked || fcn == nullptr )
	{
		return;
	}

	B2_ASSERT( b2IsValidVec2( origin ) );




	b2Vec2 r = { mover->radius, mover->radius };

	// 相对盒向外取整提升到世界 float，对树保守
	b2AABB relBox;
	relBox.lowerBound = b2Sub( b2Min( mover->center1, mover->center2 ), r );
	relBox.upperBound = b2Add( b2Max( mover->center1, mover->center2 ), r );
	b2AABB aabb = b2OffsetAABB( relBox, origin );

	WorldMoverContext worldContext = {
		world, fcn, filter, *mover, origin, context,
	};

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_OverlapAABB( world->broadPhase.trees + i, aabb, filter.maskBits, TreeCollideCallback,
								   &worldContext );
	}


}

bool b2ComputeWorldBounds( b2World* world, b2AABB* bounds )
{
	b2AABB worldBounds = {};
	bool haveBounds = false;

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree* tree = world->broadPhase.trees + i;
		if ( b2DynamicTree_GetProxyCount( tree ) == 0 )
		{
			continue;
		}

		b2AABB treeBounds = b2DynamicTree_GetRootBounds( tree );
		worldBounds = haveBounds ? b2AABB_Union( worldBounds, treeBounds ) : treeBounds;
		haveBounds = true;
	}

	*bounds = worldBounds;
	return haveBounds;
}

b2AABB b2World_GetBounds( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( world == NULL || world->locked )
	{
		return b2AABB{};
	}

	b2AABB bounds;
	b2ComputeWorldBounds( world, &bounds );
	return bounds;
}

static void b2DrawShape( b2DebugDraw* draw, b2Shape* shape, b2WorldTransform transform, b2HexColor color,
						 bool drawChainNormals )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
		{
			b2Capsule* capsule = &shape->capsule;
			b2Pos p1 = b2TransformWorldPoint( transform, capsule->center1 );
			b2Pos p2 = b2TransformWorldPoint( transform, capsule->center2 );
			draw->DrawSolidCapsuleFcn( p1, p2, capsule->radius, color, draw->context );
		}
		break;

		case b2_circleShape:
		{
			b2Circle* circle = &shape->circle;
			draw->DrawSolidCircleFcn( transform, circle->center, circle->radius, color, draw->context );
		}
		break;

		case b2_polygonShape:
		{
			b2Polygon* poly = &shape->polygon;
			draw->DrawSolidPolygonFcn( transform, poly->vertices, poly->count, poly->radius, color, draw->context );
		}
		break;

		case b2_segmentShape:
		{
			b2Segment* segment = &shape->segment;
			b2Pos p1 = b2TransformWorldPoint( transform, segment->point1 );
			b2Pos p2 = b2TransformWorldPoint( transform, segment->point2 );
			draw->DrawLineFcn( p1, p2, color, draw->context );
		}
		break;

		case b2_chainSegmentShape:
		{
			b2Segment* segment = &shape->chainSegment.segment;
			b2Pos p1 = b2TransformWorldPoint( transform, segment->point1 );
			b2Pos p2 = b2TransformWorldPoint( transform, segment->point2 );
			draw->DrawLineFcn( p1, p2, color, draw->context );
			draw->DrawPointFcn( p2, 4.0f, color, draw->context );

			if ( drawChainNormals )
			{
				b2Pos c = b2LerpPos( p1, p2, 0.5f );
				b2Vec2 e = b2Normalize( b2SubPos( p2, p1 ) );
				b2Vec2 n = b2RightPerp( e );
				float L = 0.2f * b2GetLengthUnitsPerMeter();
				draw->DrawLineFcn( c, b2OffsetPos( c, b2MulSV( L, n ) ), b2_colorPaleGreen, draw->context );
			}
		}
		break;

		default:
			break;
	}
}

struct DrawContext
{
	b2World* world;
	b2DebugDraw* draw;
};

static bool DrawQueryCallback( int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;

	int shapeId = (int)userData;

	struct DrawContext* drawContext = static_cast<DrawContext *>( context );
	b2World* world = drawContext->world;
	b2DebugDraw* draw = drawContext->draw;

	b2Shape* shape = b2Array_Get( world->shapes, shapeId );
	B2_ASSERT( shape->id == shapeId );
	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2BodySim* bodySim = b2GetBodySim( world, body );

	b2SetBit( &world->debugBodySet, shape->bodyId );

	if ( draw->drawShapes )
	{
		b2HexColor color;

		if ( shape->material.customColor != 0 )
		{
			color = static_cast<b2HexColor>( shape->material.customColor );
		}
		else if ( body->type == b2_dynamicBody && body->mass == 0.0f )
		{
			// 无效刚体
			color = b2_colorRed;
		}
		else if ( body->setIndex == b2_disabledSet )
		{
			color = b2_colorSlateGray;
		}
		else if ( shape->sensorIndex != B2_NULL_INDEX )
		{
			color = b2_colorWheat;
		}
		else if ( bodySim->flags & b2_hadTimeOfImpact )
		{
			color = b2_colorLime;
		}
		else if ( ( bodySim->flags & b2_isBullet ) && body->setIndex == b2_awakeSet )
		{
			color = b2_colorTurquoise;
		}
		else if ( body->flags & b2_isSpeedCapped )
		{
			color = b2_colorYellow;
		}
		else if ( bodySim->flags & b2_isFast )
		{
			color = b2_colorSalmon;
		}
		else if ( body->type == b2_staticBody )
		{
			color = b2_colorPaleGreen;
		}
		else if ( body->type == b2_kinematicBody )
		{
			color = b2_colorRoyalBlue;
		}
		else if ( body->setIndex == b2_awakeSet )
		{
			color = b2_colorPink;
		}
		else
		{
			color = b2_colorGray;
		}

		b2DrawShape( draw, shape, bodySim->transform, color, draw->drawChainNormals );
	}

	if ( draw->drawBounds )
	{
		draw->DrawBoundsFcn( world->fatAABBs.data[shapeId], b2_colorGold, draw->context );
	}

	return true;
}

// todo 移动形状顺序会变化，重叠形状移动时会闪烁
// 方案：按形状 id 模 3 显示顺序，在 GLSolid* 中保留 3 个桶并分 3 遍刷新。
void b2World_Draw( b2WorldId worldId, b2DebugDraw* draw )
{
	b2World* world = b2GetWorldFromId( worldId );

	if ( world == nullptr || world->locked || draw == nullptr )
	{
		return;
	}

	B2_ASSERT( b2IsValidAABB( draw->drawingBounds ) );

	const float k_axisScale = 0.3f;
	b2HexColor speculativeColor = b2_colorGainsboro;
	b2HexColor addColor = b2_colorGreen;
	b2HexColor persistColor = b2_colorBlue;
	b2HexColor normalColor = b2_colorDimGray;
	b2HexColor impulseColor = b2_colorMagenta;
	b2HexColor frictionColor = b2_colorYellow;

	int bodyCapacity = b2GetIdCapacity( &world->bodyIdPool );
	b2SetBitCountAndClear( &world->debugBodySet, bodyCapacity );

	int jointCapacity = b2GetIdCapacity( &world->jointIdPool );
	b2SetBitCountAndClear( &world->debugJointSet, jointCapacity );


	int contactCapacity = b2GetIdCapacity( &world->contactIdPool );
	b2SetBitCountAndClear( &world->debugContactSet, contactCapacity );

	int islandCapacity = b2GetIdCapacity( &world->islandIdPool );
	b2SetBitCountAndClear( &world->debugIslandSet, islandCapacity );

	struct DrawContext drawContext = { world, draw };

	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DynamicTree_OverlapAABBAll( world->broadPhase.trees + i, draw->drawingBounds, DrawQueryCallback,
									  &drawContext );
	}

	uint32_t wordCount = world->debugBodySet.blockCount;
	uint64_t* bits = world->debugBodySet.bits;
	for ( uint32_t k = 0; k < wordCount; ++k )
	{
		uint64_t word = bits[k];
		while ( word != 0 )
		{
			uint32_t ctz = std::countr_zero( word );
			uint32_t bodyId = 64 * k + ctz;

			b2Body* body = b2Array_Get( world->bodies, (int)bodyId );

			if ( draw->drawBodyNames && body->name[0] != 0 )
			{
				b2Vec2 offset = { 0.1f, 0.1f };
				b2BodySim* bodySim = b2GetBodySim( world, body );

				b2WorldTransform transform = { bodySim->center, bodySim->transform.q };
				b2Pos p = b2TransformWorldPoint( transform, offset );
				draw->DrawStringFcn( p, body->name, b2_colorWhiteSmoke, draw->context );
			}

			if ( draw->drawMass && body->type == b2_dynamicBody )
			{
				b2Vec2 offset = { 0.1f, 0.1f };
				b2BodySim* bodySim = b2GetBodySim( world, body );

				b2WorldTransform transform = { bodySim->center, bodySim->transform.q };
				draw->DrawLineFcn( bodySim->center0, bodySim->center, b2_colorWhiteSmoke, draw->context );
				draw->DrawTransformFcn( transform, draw->context );

				b2Pos p = b2TransformWorldPoint( transform, offset );
				char buffer[32];
				snprintf( buffer, 32, "  %.2f", body->mass );
				draw->DrawStringFcn( p, buffer, b2_colorWhite, draw->context );
			}

			if ( draw->drawJoints )
			{
				int jointKey = body->headJointKey;
				while ( jointKey != B2_NULL_INDEX )
				{
					int jointId = jointKey >> 1;
					int edgeIndex = jointKey & 1;
					b2Joint* joint = b2Array_Get( world->joints, jointId );

					// 避免重复绘制
					if ( b2GetBit( &world->debugJointSet, jointId ) == false )
					{
						b2DrawJoint( draw, world, joint );
						b2SetBit( &world->debugJointSet, jointId );
					}

					jointKey = joint->edges[edgeIndex].nextKey;
				}
			}


			const float linearSlop = B2_LINEAR_SLOP;
			if ( draw->drawContacts && body->type == b2_dynamicBody )
			{
				int contactKey = body->headContactKey;
				while ( contactKey != B2_NULL_INDEX )
				{
					int contactId = contactKey >> 1;
					int edgeIndex = contactKey & 1;
					b2Contact* contact = b2Array_Get( world->contacts, contactId );
					contactKey = contact->edges[edgeIndex].nextKey;

					// 避免重复绘制
					if ( b2GetBit( &world->debugContactSet, contactId ) == false )
					{
						b2ContactSim* contactSim = b2GetContactSim( world, contact );
						b2Body* bodyA = b2Array_Get( world->bodies, contact->edges[0].bodyId );
						b2BodySim* bodySimA = b2GetBodySim( world, bodyA );
						b2Body* bodyB = b2Array_Get( world->bodies, contact->edges[1].bodyId );
						b2BodySim* bodySimB = b2GetBodySim( world, bodyB );
						int pointCount = contactSim->manifold.pointCount;
						b2Vec2 normal = contactSim->manifold.normal;
						char buffer[32];

						for ( int j = 0; j < pointCount; ++j )
						{
							b2ManifoldPoint* mp = contactSim->manifold.points + j;

							b2Pos p;
							if ( draw->drawAnchorA )
							{
								p = b2OffsetPos( bodySimA->center, mp->anchorA );
							}
							else
							{
								p = b2OffsetPos( bodySimB->center, mp->anchorB );
							}

							if ( draw->drawGraphColors && contact->colorIndex != B2_NULL_INDEX )
							{
								// 图颜色
								float pointSize = contact->colorIndex == B2_OVERFLOW_INDEX ? 7.5f : 5.0f;
								draw->DrawPointFcn( p, pointSize, b2GetGraphColor( contact->colorIndex ),
													draw->context );
								// m_context->draw.DrawString(point->position, "%d", point->color);
							}
							else if ( mp->separation > linearSlop )
							{
								// 预测性
								draw->DrawPointFcn( p, 5.0f, speculativeColor, draw->context );
							}
							else if ( mp->persisted == false )
							{
								// 添加
								draw->DrawPointFcn( p, 10.0f, addColor, draw->context );
							}
							else if ( mp->persisted == true )
							{
								// 持续
								draw->DrawPointFcn( p, 5.0f, persistColor, draw->context );
							}

							if ( draw->drawContactNormals )
							{
								b2Pos p1 = p;
								b2Pos p2 = b2OffsetPos( p1, b2MulSV( k_axisScale, normal ) );
								draw->DrawLineFcn( p1, p2, normalColor, draw->context );

								snprintf( buffer, sizeof(buffer), " %.2f", mp->separation );
								draw->DrawStringFcn( p1, buffer, b2_colorWhite, draw->context );
							}
							else if ( draw->drawContactForces )
							{
								// todo 校验
								// 因松弛迭代乘以二分之一
								float force = 0.5f * mp->totalNormalImpulse * world->inv_dt;
								b2Pos p1 = p;
								b2Pos p2 = b2OffsetPos( p1, b2MulSV( draw->forceScale * force, normal ) );
								draw->DrawLineFcn( p1, p2, impulseColor, draw->context );
								snprintf( buffer, sizeof(buffer), "%.1f", force );
								draw->DrawStringFcn( p1, buffer, b2_colorWhite, draw->context );
							}

							if ( draw->drawContactFeatures )
							{
								snprintf( buffer, sizeof(buffer), "%d", mp->id );
								draw->DrawStringFcn( p, buffer, b2_colorOrange, draw->context );
							}

							if ( draw->drawFrictionForces )
							{
								float force = 0.5f * mp->tangentImpulse * world->inv_h;
								b2Vec2 tangent = b2RightPerp( normal );
								b2Pos p1 = p;
								b2Pos p2 = b2OffsetPos( p1, b2MulSV( draw->forceScale * force, tangent ) );
								draw->DrawLineFcn( p1, p2, frictionColor, draw->context );
								snprintf( buffer, sizeof(buffer), "%.1f", force );
								draw->DrawStringFcn( p1, buffer, b2_colorWhite, draw->context );
							}
						}

						b2SetBit( &world->debugContactSet, contactId );
					}

					contactKey = contact->edges[edgeIndex].nextKey;
				}
			}

			if ( draw->drawIslands )
			{
				int islandId = body->islandId;
				if ( islandId != B2_NULL_INDEX && b2GetBit( &world->debugIslandSet, islandId ) == false )
				{
					b2Island* island = world->islands.data + islandId;
					if ( island->setIndex == B2_NULL_INDEX )
					{
						continue;
					}

					int shapeCount = 0;
					b2AABB aabb = {
						.lowerBound = { FLT_MAX, FLT_MAX },
						.upperBound = { -FLT_MAX, -FLT_MAX },
					};

					for ( int bodyIndex = 0; bodyIndex < island->bodies.count; ++bodyIndex )
					{
						int islandBodyId = island->bodies.data[bodyIndex];
						b2Body* islandBody = b2Array_Get( world->bodies, islandBodyId );
						int shapeId = islandBody->headShapeId;
						while ( shapeId != B2_NULL_INDEX )
						{
							b2Shape* shape = b2Array_Get( world->shapes, shapeId );
							aabb = b2AABB_Union( aabb, world->fatAABBs.data[shapeId] );
							shapeCount += 1;
							shapeId = shape->nextShapeId;
						}
					}

					if ( shapeCount > 0 )
					{
						draw->DrawBoundsFcn( aabb, b2_colorOrangeRed, draw->context );
					}

					b2SetBit( &world->debugIslandSet, islandId );
				}
			}

			// 清除最低已置位
			word = word & ( word - 1 );
		}
	}
}

// 用于内部测试。未优化。
void b2World_RebuildStaticTree( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );

	if ( world == nullptr || world->locked )
	{
		return;
	}



	b2DynamicTree* staticTree = world->broadPhase.trees + b2_staticBody;

	// 被移动的代理在重建中必须保留该标志。
	int movedCount = 0;
	int* movedProxies = NULL;
	int proxyCount = b2DynamicTree_GetProxyCount( staticTree );
	if ( b2HasTreeMoved( staticTree ) )
	{
		movedProxies = static_cast<int *>( b2Alloc( proxyCount * sizeof( int ) ) );
		movedCount = b2DynamicTree_GatherMovedProxies( staticTree, movedProxies );
	}

	b2DynamicTree_Rebuild( staticTree, true );

	for ( int i = 0; i < movedCount; ++i )
	{
		b2DynamicTree_MarkProxyMovedSerial( staticTree, movedProxies[i] );
	}

	if ( movedProxies != NULL )
	{
		b2Free( movedProxies, proxyCount * sizeof( int ) );
	}
}

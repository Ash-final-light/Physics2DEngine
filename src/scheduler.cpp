// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "scheduler.h"

#include "core.h"
#include "platform.h"

#include "box2d/base.h"
#include "box2d/constants.h"

#include <stdio.h>
#include <string.h>

#define B2_SCHEDULER_SPIN_MILLISECONDS 0.05f

enum b2SchedulerTaskStatus
{
	b2_schedulerFree = 0,
	b2_schedulerPending = 1,
	b2_schedulerClaimed = 2,
	b2_schedulerComplete = 3,
};

typedef struct b2SchedulerTask
{
	b2TaskCallbackFcn* callback;
	void* taskContext;
	b2AtomicInt status;
} b2SchedulerTask;

typedef struct b2SchedulerWorkerContext
{
	struct b2Scheduler* scheduler;
	int threadIndex;
} b2SchedulerWorkerContext;

struct b2Scheduler
{
	b2Thread* threads[B2_MAX_WORKERS];
	b2SchedulerWorkerContext workerContexts[B2_MAX_WORKERS];

	int workerCount;
	int threadCount;

	b2SchedulerTask tasks[B2_MAX_TASKS];
	b2AtomicInt nextSlot;

	b2Semaphore* taskSemaphore;
	b2AtomicInt sleeperCount;
	b2AtomicInt shutdown;
};

static bool b2SchedulerHasPending( b2Scheduler* scheduler )
{
	int taskCount = b2AtomicLoadInt( &scheduler->nextSlot );
	for ( int t = 0; t < taskCount; ++t )
	{
		if ( b2AtomicLoadInt( &scheduler->tasks[t].status ) == b2_schedulerPending )
		{
			return true;
		}
	}

	return false;
}

static bool b2SchedulerSpin( b2Scheduler* scheduler )
{
	uint64_t ticks = b2GetTicks();
	while ( true )
	{
		if ( b2SchedulerHasPending( scheduler ) || b2AtomicLoadInt( &scheduler->shutdown ) != 0 )
		{
			return true;
		}

		if ( b2GetMilliseconds( ticks ) >= B2_SCHEDULER_SPIN_MILLISECONDS )
		{
			return false;
		}

		for ( int i = 0; i < 8; ++i )
		{
			b2Pause();
		}
	}
}

static bool b2SchedulerTakeSleeper( b2Scheduler* scheduler )
{
	int sleeperCount = b2AtomicLoadInt( &scheduler->sleeperCount );
	while ( sleeperCount > 0 )
	{
		if ( b2AtomicCompareExchangeInt( &scheduler->sleeperCount, sleeperCount, sleeperCount - 1 ) )
		{
			return true;
		}

		sleeperCount = b2AtomicLoadInt( &scheduler->sleeperCount );
	}

	return false;
}

static bool b2SchedulerExecuteOne( b2Scheduler* scheduler )
{
	int taskCount = b2AtomicLoadInt( &scheduler->nextSlot );
	for ( int t = 0; t < taskCount; ++t )
	{
		b2SchedulerTask* task = scheduler->tasks + t;
		if ( b2AtomicLoadInt( &task->status ) != b2_schedulerPending )
		{
			continue;
		}

		if ( b2AtomicCompareExchangeInt( &task->status, b2_schedulerPending, b2_schedulerClaimed ) == false )
		{
			continue;
		}

		task->callback( task->taskContext );

		b2AtomicStoreInt( &task->status, b2_schedulerComplete );
		return true;
	}

	return false;
}

static void b2SchedulerWorkerMain( void* context )
{
	b2SchedulerWorkerContext* workerContext = static_cast<b2SchedulerWorkerContext*>( context );
	b2Scheduler* scheduler = workerContext->scheduler;

	while ( true )
	{
		while ( b2SchedulerExecuteOne( scheduler ) )
		{
		}

		if ( b2AtomicLoadInt( &scheduler->shutdown ) != 0 )
		{
			break;
		}

		if ( b2SchedulerSpin( scheduler ) )
		{
			continue;
		}

		b2AtomicFetchAddInt( &scheduler->sleeperCount, 1 );

		if ( b2SchedulerHasPending( scheduler ) || b2AtomicLoadInt( &scheduler->shutdown ) != 0 )
		{
			b2SchedulerTakeSleeper( scheduler );
			continue;
		}

		b2WaitSemaphore( scheduler->taskSemaphore );
	}
}

b2Scheduler* b2CreateScheduler( int workerCount )
{
	B2_ASSERT( 0 < workerCount && workerCount <= B2_MAX_WORKERS );

	b2Scheduler* scheduler = static_cast<b2Scheduler*>( b2Alloc( sizeof( b2Scheduler ) ) );
	memset( scheduler, 0, sizeof( b2Scheduler ) );

	scheduler->workerCount = workerCount;
	int threadCount = workerCount - 1;
	scheduler->threadCount = threadCount;
	scheduler->taskSemaphore = b2CreateSemaphore( 0 );
	b2AtomicStoreInt( &scheduler->sleeperCount, 0 );
	b2AtomicStoreInt( &scheduler->shutdown, 0 );
	b2AtomicStoreInt( &scheduler->nextSlot, 0 );

	for ( int i = 0; i < threadCount; ++i )
	{
		scheduler->workerContexts[i].scheduler = scheduler;
		scheduler->workerContexts[i].threadIndex = i + 1;

		char name[16];
		snprintf( name, sizeof( name ), "box2d_worker_%02d", i + 1 );
		scheduler->threads[i] = b2CreateThread( b2SchedulerWorkerMain, scheduler->workerContexts + i, name );
	}

	return scheduler;
}

void b2DestroyScheduler( b2Scheduler* scheduler )
{
	b2AtomicStoreInt( &scheduler->shutdown, 1 );

	for ( int i = 0; i < scheduler->threadCount; ++i )
	{
		b2SignalSemaphore( scheduler->taskSemaphore );
	}

	for ( int i = 0; i < scheduler->threadCount; ++i )
	{
		b2JoinThread( scheduler->threads[i] );
		scheduler->threads[i] = NULL;
	}

	b2DestroySemaphore( scheduler->taskSemaphore );
	b2Free( scheduler, sizeof( b2Scheduler ) );
}

void b2ResetScheduler( b2Scheduler* scheduler )
{
	b2AtomicStoreInt( &scheduler->nextSlot, 0 );
}

void* b2SchedulerEnqueueTask( b2TaskCallbackFcn* task, void* taskContext, void* userContext )
{
	b2Scheduler* scheduler = static_cast<b2Scheduler*>( userContext );

	int slot = b2AtomicFetchAddInt( &scheduler->nextSlot, 1 );
	B2_ASSERT( slot < B2_MAX_TASKS );

	b2SchedulerTask* schedulerTask = scheduler->tasks + slot;
	schedulerTask->callback = task;
	schedulerTask->taskContext = taskContext;

	b2AtomicStoreInt( &schedulerTask->status, b2_schedulerPending );

	if ( b2SchedulerTakeSleeper( scheduler ) )
	{
		b2SignalSemaphore( scheduler->taskSemaphore );
	}

	return schedulerTask;
}

void b2SchedulerFinishTask( void* userTask, void* userContext )
{
	if ( userTask == NULL )
	{
		return;
	}

	b2Scheduler* scheduler = static_cast<b2Scheduler*>( userContext );
	b2SchedulerTask* waitTask = static_cast<b2SchedulerTask*>( userTask );

	while ( b2AtomicLoadInt( &waitTask->status ) != b2_schedulerComplete )
	{
		if ( b2SchedulerExecuteOne( scheduler ) == false )
		{
			b2Yield();
		}
	}
}

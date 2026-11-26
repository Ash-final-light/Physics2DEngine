// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"
#include "parallel_for.h"
#include "platform.h"
#include "scheduler.h"
#include "solver.h"
#include "test_macros.h"
#include "world.h"

#include <atomic>
#include <thread>

int MutexTest( void )
{
	size_t before = b2GetByteCount();
	b2Mutex* mutex = b2CreateMutex();
	ENSURE( mutex != NULL );
	ENSURE( b2GetByteCount() > before );

	// 主线程持锁期间竞争线程进入函数但无法更新共享值
	std::atomic<int> entered{ 0 };
	int value = 0;
	b2LockMutex( mutex );
	std::thread contender( [&]() {
		entered.fetch_add( 1 );
		b2LockMutex( mutex );
		value = 7;
		b2UnlockMutex( mutex );
	} );
	for ( int i = 0; i < 1000000 && entered.load() == 0; ++i )
	{
		std::this_thread::yield();
	}
	ENSURE( entered.load() == 1 );
	ENSURE( value == 0 );
	b2UnlockMutex( mutex );
	contender.join();
	ENSURE( value == 7 );

	// 四个线程在锁保护下累加，结果必须精确
	int sum = 0;
	std::thread workers[4];
	for ( int i = 0; i < 4; ++i )
	{
		workers[i] = std::thread( [&]() {
			for ( int j = 0; j < 500; ++j )
			{
				b2LockMutex( mutex );
				sum += 1;
				b2UnlockMutex( mutex );
			}
		} );
	}
	for ( int i = 0; i < 4; ++i )
	{
		workers[i].join();
	}
	ENSURE( sum == 2000 );

	b2DestroyMutex( mutex );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

int SemaphoreTest( void )
{
	size_t before = b2GetByteCount();
	b2Semaphore* sem = b2CreateSemaphore( 0 );
	ENSURE( sem != NULL );
	ENSURE( b2GetByteCount() > before );

	// 初值为0：工作线程在等待处阻塞，通知后才继续
	std::atomic<int> entered{ 0 };
	int value = 0;
	std::thread worker( [&]() {
		entered.fetch_add( 1 );
		b2WaitSemaphore( sem );
		value = 99;
	} );
	for ( int i = 0; i < 1000000 && entered.load() == 0; ++i )
	{
		std::this_thread::yield();
	}
	ENSURE( entered.load() == 1 );
	ENSURE( value == 0 );
	b2SignalSemaphore( sem );
	worker.join();
	ENSURE( value == 99 );

	// 两次通知对应两次等待，计数归零后下一个等待再次阻塞
	b2SignalSemaphore( sem );
	b2SignalSemaphore( sem );
	b2WaitSemaphore( sem );
	b2WaitSemaphore( sem );

	std::atomic<int> waiting{ 0 };
	int released = 0;
	std::thread waiter( [&]() {
		waiting.fetch_add( 1 );
		b2WaitSemaphore( sem );
		released = 1;
	} );
	for ( int i = 0; i < 1000000 && waiting.load() == 0; ++i )
	{
		std::this_thread::yield();
	}
	ENSURE( waiting.load() == 1 );
	ENSURE( released == 0 );
	b2SignalSemaphore( sem );
	waiter.join();
	ENSURE( released == 1 );

	b2DestroySemaphore( sem );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

typedef struct ThreadData
{
	int value;
} ThreadData;

static void ThreadWorker( void* context )
{
	ThreadData* data = (ThreadData*)context;
	data->value = 99;
}

typedef struct JoinWaitData
{
	b2Semaphore* release;
	std::atomic<int>* done;
} JoinWaitData;

static void JoinWaitWorker( void* context )
{
	JoinWaitData* data = (JoinWaitData*)context;
	b2WaitSemaphore( data->release );
	data->done->fetch_add( 1 );
}

typedef struct ThreadSumData
{
	b2Mutex* mutex;
	int sum;
} ThreadSumData;

static void ThreadSumWorker( void* context )
{
	ThreadSumData* data = (ThreadSumData*)context;
	for ( int i = 0; i < 250; ++i )
	{
		b2LockMutex( data->mutex );
		data->sum += 1;
		b2UnlockMutex( data->mutex );
	}
}

int ThreadTest( void )
{
	size_t before = b2GetByteCount();

	ThreadData data = {};
	b2Thread* thread = b2CreateThread( ThreadWorker, &data, "worker" );
	ENSURE( thread != NULL );
	ENSURE( b2GetByteCount() > before );
	b2JoinThread( thread );
	ENSURE( data.value == 99 );

	// join 必须等待任务真正完成
	b2Semaphore* release = b2CreateSemaphore( 0 );
	std::atomic<int> done{ 0 };
	std::atomic<int> started{ 0 };
	std::atomic<int> joined{ 0 };
	JoinWaitData waitData = { release, &done };
	b2Thread* blocked = b2CreateThread( JoinWaitWorker, &waitData, "join wait" );
	std::thread joiner( [&]() {
		started.fetch_add( 1 );
		b2JoinThread( blocked );
		joined.fetch_add( 1 );
	} );
	for ( int i = 0; i < 1000000 && started.load() == 0; ++i )
	{
		std::this_thread::yield();
	}
	ENSURE( started.load() == 1 );
	for ( int i = 0; i < 1000; ++i )
	{
		std::this_thread::yield();
	}
	ENSURE( joined.load() == 0 );
	ENSURE( done.load() == 0 );
	b2SignalSemaphore( release );
	joiner.join();
	ENSURE( joined.load() == 1 );
	ENSURE( done.load() == 1 );
	b2DestroySemaphore( release );

	ThreadSumData sumData = {};
	sumData.mutex = b2CreateMutex();
	b2Thread* workers[4];
	for ( int i = 0; i < 4; ++i )
	{
		workers[i] = b2CreateThread( ThreadSumWorker, &sumData, "sum worker" );
	}
	for ( int i = 0; i < 4; ++i )
	{
		b2JoinThread( workers[i] );
	}
	ENSURE( sumData.sum == 1000 );
	b2DestroyMutex( sumData.mutex );

	ENSURE( b2GetByteCount() == before );
	return 0;
}

typedef struct BatchTaskData
{
	int id;
	int* calls;
	int* sequence;
	int* sequenceCount;
} BatchTaskData;

static void BatchTaskFcn( void* context )
{
	BatchTaskData* data = (BatchTaskData*)context;
	data->calls[data->id] += 1;
	data->sequence[data->sequenceCount[0]++] = data->id;
}

int TaskBatchTest( void )
{
	b2StepContext context = {};
	int calls[5] = {};
	int sequence[8] = {};
	int sequenceCount = 0;
	BatchTaskData data[5];
	for ( int i = 0; i < 5; ++i )
	{
		data[i] = BatchTaskData{ i, calls, sequence, &sequenceCount };
	}

	b2ResetTaskBatch( &context );
	for ( int i = 0; i < 3; ++i )
	{
		b2AddTask( &context, (b2TaskType)( b2_taskPairs + i ), BatchTaskFcn, &data[i] );
	}
	b2TaskBatch batch = b2MakeTaskBatch( &context );
	ENSURE( batch.count == 3 && batch.joinCount == 0 );
	for ( int i = 0; i < batch.count; ++i )
	{
		ENSURE( batch.tasks[i].fcn != NULL );
		ENSURE( batch.tasks[i].type == (b2TaskType)( b2_taskPairs + i ) );
		ENSURE( batch.tasks[i].deferred == false );
		ENSURE( batch.tasks[i].context == &context.stepTasks[1 + i] );
	}

	// 上一批作为新批次的依赖：先执行接合任务，再执行本批任务，每个任务只执行一次
	b2ResetTaskBatch( &context );
	b2AddTask( &context, b2_taskSolver, BatchTaskFcn, &data[3] );
	b2AddTask( &context, b2_taskFinalize, BatchTaskFcn, &data[4] );
	batch = b2MakeTaskBatch( &context );
	ENSURE( batch.count == 2 && batch.joinCount == 3 );
	for ( int i = 0; i < batch.joinCount; ++i )
	{
		batch.joinTasks[i].fcn( batch.joinTasks[i].context );
	}
	for ( int i = 0; i < batch.count; ++i )
	{
		batch.tasks[i].fcn( batch.tasks[i].context );
	}
	for ( int i = 0; i < 5; ++i )
	{
		ENSURE( calls[i] == 1 );
	}
	ENSURE( sequenceCount == 5 );
	for ( int i = 0; i < 5; ++i )
	{
		ENSURE( sequence[i] == i );
	}

	// 再次重置只保留最近一批，旧任务不会重复执行
	b2ResetTaskBatch( &context );
	batch = b2MakeTaskBatch( &context );
	ENSURE( batch.count == 0 && batch.joinCount == 2 );
	for ( int i = 0; i < batch.joinCount; ++i )
	{
		ENSURE( batch.joinTasks[i].context == &context.joinStepTasks[i] );
		ENSURE( ((b2StepTask*)batch.joinTasks[i].context)->context == &data[3 + i] );
	}
	ENSURE( context.taskCount == 5 );
	return 0;
}

typedef struct SchedulerTaskData
{
	b2AtomicInt* total;
	int* calls;
	int id;
} SchedulerTaskData;

static void SchedulerTaskFcn( void* context )
{
	SchedulerTaskData* data = (SchedulerTaskData*)context;
	b2AtomicFetchAddInt( data->total, 1 );
	data->calls[data->id] += 1;
}

static int RunSchedulerPass( int workerCount, int taskCount )
{
	size_t before = b2GetByteCount();
	b2Scheduler* scheduler = b2CreateScheduler( workerCount );
	ENSURE( scheduler != NULL );
	ENSURE( b2GetByteCount() > before );

	b2AtomicInt total;
	b2AtomicStoreInt( &total, 0 );
	int calls[32] = {};
	SchedulerTaskData data[32];
	void* handles[32];
	for ( int i = 0; i < taskCount; ++i )
	{
		data[i] = SchedulerTaskData{ &total, calls, i };
		handles[i] = b2SchedulerEnqueueTask( SchedulerTaskFcn, &data[i], scheduler );
		ENSURE( handles[i] != NULL );
	}
	for ( int i = taskCount - 1; i >= 0; --i )
	{
		b2SchedulerFinishTask( handles[i], scheduler );
	}
	ENSURE( b2AtomicLoadInt( &total ) == taskCount );
	for ( int i = 0; i < taskCount; ++i )
	{
		ENSURE( calls[i] == 1 );
	}

	// reset 后继续入队，槽位重新使用但任务仍全部执行
	b2ResetScheduler( scheduler );
	int secondCalls[32] = {};
	for ( int i = 0; i < taskCount; ++i )
	{
		data[i] = SchedulerTaskData{ &total, secondCalls, i };
		handles[i] = b2SchedulerEnqueueTask( SchedulerTaskFcn, &data[i], scheduler );
	}
	for ( int i = 0; i < taskCount; ++i )
	{
		b2SchedulerFinishTask( handles[i], scheduler );
	}
	ENSURE( b2AtomicLoadInt( &total ) == 2 * taskCount );
	for ( int i = 0; i < taskCount; ++i )
	{
		ENSURE( secondCalls[i] == 1 );
	}

	b2DestroyScheduler( scheduler );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

int SchedulerTest( void )
{
	ENSURE( RunSchedulerPass( 4, 24 ) == 0 );
	ENSURE( RunSchedulerPass( 2, 8 ) == 0 );
	return 0;
}

typedef struct ExternalTaskRecord
{
	int enqueueCount;
	int finishCount;
	int runCount;
	bool inlineMode;
	b2TaskCallbackFcn* pendingFcn[8];
	void* pendingContext[8];
	int pendingCount;
} ExternalTaskRecord;

typedef struct ExternalCounter
{
	int* calls;
	int* sequence;
	int* sequenceCount;
	int id;
} ExternalCounter;

static void ExternalTaskFcn( void* context )
{
	ExternalCounter* counter = (ExternalCounter*)context;
	counter->calls[counter->id] += 1;
	counter->sequence[counter->sequenceCount[0]++] = counter->id;
}

static void* ExternalEnqueue( b2TaskCallbackFcn* task, void* taskContext, void* userContext )
{
	ExternalTaskRecord* record = (ExternalTaskRecord*)userContext;
	record->enqueueCount += 1;
	if ( record->inlineMode )
	{
		task( taskContext );
		return NULL;
	}
	int slot = record->pendingCount++;
	record->pendingFcn[slot] = task;
	record->pendingContext[slot] = taskContext;
	return (void*)(intptr_t)( slot + 1 );
}

static void ExternalFinish( void* userTask, void* userContext )
{
	ExternalTaskRecord* record = (ExternalTaskRecord*)userContext;
	record->finishCount += 1;
	int slot = (int)(intptr_t)userTask - 1;
	record->pendingFcn[slot]( record->pendingContext[slot] );
}

static b2TaskBatch MakeExternalBatch( b2StepContext* context, ExternalCounter* counters )
{
	b2ResetTaskBatch( context );
	for ( int i = 0; i < 4; ++i )
	{
		b2AddTask( context, (b2TaskType)( b2_taskPairs + i ), ExternalTaskFcn, counters + i );
	}
	return b2MakeTaskBatch( context );
}

int ExternalTaskTest( void )
{
	ExternalTaskRecord record = {};
	int calls[4] = {};
	int sequence[8] = {};
	int sequenceCount = 0;
	ExternalCounter counters[4];
	for ( int i = 0; i < 4; ++i )
	{
		counters[i] = ExternalCounter{ calls, sequence, &sequenceCount, i };
	}

	// 入队回调内同步执行并返回空句柄：不调用 finish
	b2StepContext context = {};
	b2TaskBatch batch = MakeExternalBatch( &context, counters );
	record.inlineMode = true;
	b2RunTaskBatch( ExternalEnqueue, ExternalFinish, &record, batch );
	ENSURE( record.enqueueCount == 3 && record.finishCount == 0 );
	ENSURE( sequenceCount == 4 );
	for ( int i = 0; i < 4; ++i )
	{
		ENSURE( calls[i] == 1 );
	}

	// 返回非空句柄时由 finish 完成剩余任务；最后一个任务仍本地执行
	record = ExternalTaskRecord{};
	for ( int i = 0; i < 4; ++i )
	{
		calls[i] = 0;
	}
	sequenceCount = 0;
	batch = MakeExternalBatch( &context, counters );
	b2RunTaskBatch( ExternalEnqueue, ExternalFinish, &record, batch );
	ENSURE( record.enqueueCount == 3 && record.finishCount == 3 );
	ENSURE( sequenceCount == 4 );
	for ( int i = 0; i < 4; ++i )
	{
		ENSURE( calls[i] == 1 );
	}

	// 世界定义保存外部回调与用户上下文；默认世界使用串行回调
	b2WorldDef def = b2DefaultWorldDef();
	ENSURE( def.enqueueTask == NULL && def.finishTask == NULL && def.userTaskContext == NULL );
	def.workerCount = 4;
	def.enqueueTask = ExternalEnqueue;
	def.finishTask = ExternalFinish;
	def.userTaskContext = &record;
	b2WorldId worldId = b2CreateWorld( &def );
	ENSURE( b2World_IsValid( worldId ) );
	b2World* world = b2_worlds + worldId.index1 - 1;
	ENSURE( world->workerCount == 4 && world->enqueueTaskFcn == ExternalEnqueue );
	ENSURE( world->finishTaskFcn == ExternalFinish && world->userTaskContext == &record );
	b2DestroyWorld( worldId );

	def = b2DefaultWorldDef();
	worldId = b2CreateWorld( &def );
	world = b2_worlds + worldId.index1 - 1;
	ENSURE( world->workerCount == 1 && world->enqueueTaskFcn != NULL && world->finishTaskFcn != NULL );
	ENSURE( world->userTaskContext == NULL );
	b2DestroyWorld( worldId );
	return 0;
}

int WorkerCountTest( void )
{
	size_t before = b2GetByteCount();
	b2WorldDef def = b2DefaultWorldDef();
	def.workerCount = 1;
	b2WorldId id = b2CreateWorld( &def );
	ENSURE( b2World_IsValid( id ) && b2World_GetWorkerCount( id ) == 1 );

	b2BodyDef bd = b2DefaultBodyDef();
	bd.type = b2_dynamicBody;
	bd.position = { 0, 2 };
	bd.linearVelocity = { 0, 0 };
	b2BodyId body = b2CreateBody( id, &bd );
	b2ShapeDef sd = b2DefaultShapeDef();
	sd.density = 1.0f;
	b2Circle circle = { { 0, 0 }, 0.5f };
	b2CreateCircleShape( body, &sd, &circle );

	b2World_Step( id, 1.0f / 60.0f, 4 );
	ENSURE( b2Body_GetPosition( body ).y < 2.0f );

	b2World_SetWorkerCount( id, 4 );
	ENSURE( b2World_GetWorkerCount( id ) == 4 );
	b2World* world = b2_worlds + id.index1 - 1;
	ENSURE( world->scheduler != NULL );
	b2Scheduler* scheduler = world->scheduler;
	b2World_Step( id, 1.0f / 60.0f, 4 );
	ENSURE( world->scheduler == scheduler );
	b2World_SetWorkerCount( id, 4 );
	ENSURE( world->scheduler == scheduler );

	b2World_SetWorkerCount( id, 0 );
	ENSURE( b2World_GetWorkerCount( id ) == 1 && world->scheduler == NULL );
	b2World_Step( id, 1.0f / 60.0f, 4 );
	ENSURE( b2Body_GetPosition( body ).y < 2.0f );

	b2World_SetWorkerCount( id, -5 );
	ENSURE( b2World_GetWorkerCount( id ) == 1 );
	b2World_SetWorkerCount( id, B2_MAX_WORKERS + 10 );
	ENSURE( b2World_GetWorkerCount( id ) == B2_MAX_WORKERS && world->scheduler != NULL );
	b2World_Step( id, 1.0f / 60.0f, 4 );
	ENSURE( world->scheduler != NULL );

	b2WorldId invalid = b2WorldId{};
	b2World_SetWorkerCount( invalid, 2 );
	ENSURE( b2World_GetWorkerCount( invalid ) == 0 );

	b2DestroyWorld( id );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

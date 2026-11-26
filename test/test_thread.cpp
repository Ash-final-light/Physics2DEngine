// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"
#include "test_macros.h"

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

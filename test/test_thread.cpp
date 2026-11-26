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

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

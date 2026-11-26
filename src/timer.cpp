// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"
#include "box2d/base.h"

#if defined( _WIN32 )

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#ifndef NOGDI
#define NOGDI
#endif

#ifndef NOUSER
#define NOUSER
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>

static double s_invFrequency = 0.0;

static void b2InitTimerFrequency( void )
{
	if ( s_invFrequency == 0.0 )
	{
		LARGE_INTEGER frequency;
		QueryPerformanceFrequency( &frequency );

		s_invFrequency = (double)frequency.QuadPart;
		if ( s_invFrequency > 0.0 )
		{
			s_invFrequency = 1000.0 / s_invFrequency;
		}
	}
}

uint64_t b2GetTicks( void )
{
	LARGE_INTEGER counter;
	QueryPerformanceCounter( &counter );
	return (uint64_t)counter.QuadPart;
}

float b2GetMilliseconds( uint64_t ticks )
{
	b2InitTimerFrequency();
	uint64_t ticksNow = b2GetTicks();
	return (float)( s_invFrequency * ( ticksNow - ticks ) );
}

float b2GetMillisecondsAndReset( uint64_t* ticks )
{
	b2InitTimerFrequency();
	uint64_t ticksNow = b2GetTicks();
	float ms = (float)( s_invFrequency * ( ticksNow - *ticks ) );
	*ticks = ticksNow;
	return ms;
}

typedef struct b2Mutex
{
	CRITICAL_SECTION cs;
} b2Mutex;

b2Mutex* b2CreateMutex( void )
{
	b2Mutex* m = static_cast<b2Mutex*>( b2Alloc( sizeof( b2Mutex ) ) );
	InitializeCriticalSection( &m->cs );
	return m;
}

void b2DestroyMutex( b2Mutex* m )
{
	DeleteCriticalSection( &m->cs );
	*m = b2Mutex{};
	b2Free( m, sizeof( b2Mutex ) );
}

void b2LockMutex( b2Mutex* m )
{
	EnterCriticalSection( &m->cs );
}

void b2UnlockMutex( b2Mutex* m )
{
	LeaveCriticalSection( &m->cs );
}

#elif defined( __linux__ ) || defined( __EMSCRIPTEN__ )

#include <time.h>

uint64_t b2GetTicks( void )
{
	struct timespec ts;
	clock_gettime( CLOCK_MONOTONIC, &ts );
	return (uint64_t)ts.tv_sec * 1000000000LL + (uint64_t)ts.tv_nsec;
}

float b2GetMilliseconds( uint64_t ticks )
{
	uint64_t ticksNow = b2GetTicks();
	return (float)( ( ticksNow - ticks ) / 1000000.0 );
}

float b2GetMillisecondsAndReset( uint64_t* ticks )
{
	uint64_t ticksNow = b2GetTicks();
	float ms = (float)( ( ticksNow - *ticks ) / 1000000.0 );
	*ticks = ticksNow;
	return ms;
}

#include <pthread.h>

typedef struct b2Mutex
{
	pthread_mutex_t mtx;
} b2Mutex;

b2Mutex* b2CreateMutex( void )
{
	b2Mutex* m = static_cast<b2Mutex*>( b2Alloc( sizeof( b2Mutex ) ) );
	pthread_mutex_init( &m->mtx, NULL );
	return m;
}

void b2DestroyMutex( b2Mutex* m )
{
	pthread_mutex_destroy( &m->mtx );
	*m = b2Mutex{};
	b2Free( m, sizeof( b2Mutex ) );
}

void b2LockMutex( b2Mutex* m )
{
	pthread_mutex_lock( &m->mtx );
}

void b2UnlockMutex( b2Mutex* m )
{
	pthread_mutex_unlock( &m->mtx );
}

#elif defined( __APPLE__ )

#include <mach/mach_time.h>

static double s_invFrequency = 0.0;

static void b2InitTimerFrequency( void )
{
	if ( s_invFrequency == 0.0 )
	{
		mach_timebase_info_data_t timebase;
		mach_timebase_info( &timebase );
		s_invFrequency = 1e-6 * (double)timebase.numer / (double)timebase.denom;
	}
}

uint64_t b2GetTicks( void )
{
	return mach_absolute_time();
}

float b2GetMilliseconds( uint64_t ticks )
{
	b2InitTimerFrequency();
	uint64_t ticksNow = b2GetTicks();
	return (float)( s_invFrequency * ( ticksNow - ticks ) );
}

float b2GetMillisecondsAndReset( uint64_t* ticks )
{
	b2InitTimerFrequency();
	uint64_t ticksNow = b2GetTicks();
	float ms = (float)( s_invFrequency * ( ticksNow - *ticks ) );
	*ticks = ticksNow;
	return ms;
}

#include <pthread.h>

typedef struct b2Mutex
{
	pthread_mutex_t mtx;
} b2Mutex;

b2Mutex* b2CreateMutex( void )
{
	b2Mutex* m = static_cast<b2Mutex*>( b2Alloc( sizeof( b2Mutex ) ) );
	pthread_mutex_init( &m->mtx, NULL );
	return m;
}

void b2DestroyMutex( b2Mutex* m )
{
	pthread_mutex_destroy( &m->mtx );
	*m = b2Mutex{};
	b2Free( m, sizeof( b2Mutex ) );
}

void b2LockMutex( b2Mutex* m )
{
	pthread_mutex_lock( &m->mtx );
}

void b2UnlockMutex( b2Mutex* m )
{
	pthread_mutex_unlock( &m->mtx );
}

#else

uint64_t b2GetTicks( void )
{
	return 0;
}

float b2GetMilliseconds( uint64_t ticks )
{
	(void)ticks;
	return 0.0f;
}

float b2GetMillisecondsAndReset( uint64_t* ticks )
{
	(void)ticks;
	return 0.0f;
}

typedef struct b2Mutex
{
	int dummy;
} b2Mutex;

b2Mutex* b2CreateMutex( void )
{
	b2Mutex* m = static_cast<b2Mutex*>( b2Alloc( sizeof( b2Mutex ) ) );
	m->dummy = 42;
	return m;
}

void b2DestroyMutex( b2Mutex* m )
{
	*m = b2Mutex{};
	b2Free( m, sizeof( b2Mutex ) );
}

void b2LockMutex( b2Mutex* m )
{
	(void)m;
}

void b2UnlockMutex( b2Mutex* m )
{
	(void)m;
}

#endif

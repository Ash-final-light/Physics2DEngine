// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#if defined( _MSC_VER ) && ( defined( _M_ARM ) || defined( _M_ARM64 ) || defined( _M_ARM64EC ) )
#include <intrin.h>
#elif defined( _MSC_VER )
#include <intrin0.h>
#endif

#include "box2d/base.h"

#include <stdint.h>

#if defined( _WIN32 ) || defined( _WIN64 )
#define B2_PLATFORM_WINDOWS
#endif

#define B2_ALIGNMENT 64
#define B2_SECRET_COOKIE 1152023
#define B2_CHECK_DEF( DEF ) B2_ASSERT( ( DEF )->internalValue == B2_SECRET_COOKIE )

typedef struct b2AtomicI64
{
	alignas( 8 ) int64_t value;
} b2AtomicI64;

static inline int64_t b2AtomicFetchAddI64( b2AtomicI64* a, int64_t increment )
{
#if defined( _MSC_VER )
	return (int64_t)_InterlockedExchangeAdd64( (__int64*)&a->value, (__int64)increment );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_fetch_add( &a->value, increment, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

static inline int64_t b2AtomicLoadI64( b2AtomicI64* a )
{
#if defined( _MSC_VER ) && !defined( __clang__ ) && !defined( _M_ARM )
	int64_t value = __iso_volatile_load64( (volatile __int64*)&a->value );
#if defined( _M_ARM64 ) || defined( _M_ARM64EC )
	__dmb( 0xB );
#else
	_ReadWriteBarrier();
#endif
	return value;
#elif defined( _MSC_VER ) && !defined( __clang__ )
	return _InterlockedOr64( (__int64*)&a->value, 0 );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_load_n( &a->value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

void b2Log( const char* format, ... );
void* b2Alloc( size_t size );
void b2Free( void* mem, size_t size );
void* b2GrowAlloc( void* oldMem, size_t oldSize, size_t newSize );
void* b2AllocZero( size_t size );
void* b2GrowAllocZero( void* oldMem, size_t oldSize, size_t newSize );

typedef struct b2Mutex b2Mutex;
b2Mutex* b2CreateMutex( void );
void b2DestroyMutex( b2Mutex* m );
void b2LockMutex( b2Mutex* m );
void b2UnlockMutex( b2Mutex* m );

typedef struct b2Semaphore b2Semaphore;
b2Semaphore* b2CreateSemaphore( int initCount );
void b2DestroySemaphore( b2Semaphore* s );
void b2WaitSemaphore( b2Semaphore* s );
void b2SignalSemaphore( b2Semaphore* s );

typedef void b2ThreadFunction( void* context );
typedef struct b2Thread b2Thread;
// name 可为 NULL，否则会被复制。
b2Thread* b2CreateThread( b2ThreadFunction* function, void* context, const char* name );
void b2JoinThread( b2Thread* t );

#ifdef __cplusplus
#include <type_traits>
#define B2_TYPE_OF( A ) std::remove_cvref_t<decltype( A )>
#define B2_GROW_ZERO( A, N, M )                                                                                                  \
	static_cast<std::remove_reference_t<decltype( A )>>( b2GrowAllocZero( ( A ), ( N ) * sizeof( *( A ) ), ( M ) * sizeof( *( A ) ) ) )
#else
#define B2_TYPE_OF( A ) __typeof__( A )
#define B2_GROW_ZERO( A, N, M ) (A)
#endif

#define B2_SWAP( x, y )                                                                                                          \
	do                                                                                                                           \
	{                                                                                                                            \
		B2_TYPE_OF( x ) B2_SWAP_TEMP = x;                                                                                        \
		x = y;                                                                                                                   \
		y = B2_SWAP_TEMP;                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

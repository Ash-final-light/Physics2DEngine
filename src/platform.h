// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include <stdbool.h>
#include <stdint.h>

#if defined( _M_X64 ) || defined( __x86_64__ ) || defined( _M_IX86 ) || defined( __i386__ )
#include <xmmintrin.h>
#endif

#if defined( _MSC_VER ) && !defined( __clang__ )
#include <intrin.h>

static inline uint32_t b2CLZ32( uint32_t value )
{
	unsigned long index;
	if ( _BitScanReverse( &index, value ) )
	{
		return 31 - index;
	}

	return 32;
}
#else
static inline uint32_t b2CLZ32( uint32_t value )
{
	return __builtin_clz( value );
}
#endif

static inline int b2RoundUpPowerOf2( int x )
{
	if ( x <= 1 )
	{
		return 1;
	}

	return 1 << ( 32 - (int)b2CLZ32( (uint32_t)x - 1 ) );
}

#if ( defined( __GNUC__ ) || defined( __clang__ ) ) && ( defined( __i386__ ) || defined( __x86_64__ ) )
static inline void b2Pause( void )
{
	__asm__ __volatile__( "pause\n" );
}
#elif ( defined( __arm__ ) && defined( __ARM_ARCH ) && __ARM_ARCH >= 7 ) || defined( __aarch64__ )
static inline void b2Pause( void )
{
	__asm__ __volatile__( "yield" ::: "memory" );
}
#elif defined( _MSC_VER ) && ( defined( _M_IX86 ) || defined( _M_X64 ) )
static inline void b2Pause( void )
{
	_mm_pause();
}
#elif defined( _MSC_VER ) && ( defined( _M_ARM ) || defined( _M_ARM64 ) )
static inline void b2Pause( void )
{
	__yield();
}
#else
static inline void b2Pause( void )
{
}
#endif

static inline void b2AtomicStoreInt( b2AtomicInt* a, int value )
{
#if defined( _MSC_VER )
	(void)_InterlockedExchange( (long*)&a->value, value );
#elif defined( __GNUC__ ) || defined( __clang__ )
	__atomic_store_n( &a->value, value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

static inline int b2AtomicLoadInt( b2AtomicInt* a )
{
#if defined( _MSC_VER ) && !defined( __clang__ )
	int value = __iso_volatile_load32( (volatile __int32*)&a->value );
#if defined( _M_ARM ) || defined( _M_ARM64 ) || defined( _M_ARM64EC )
	__dmb( 0xB );
#else
	_ReadWriteBarrier();
#endif
	return value;
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_load_n( &a->value, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

static inline int b2AtomicFetchAddInt( b2AtomicInt* a, int increment )
{
#if defined( _MSC_VER )
	return _InterlockedExchangeAdd( (long*)&a->value, (long)increment );
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_fetch_add( &a->value, increment, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

static inline bool b2AtomicCompareExchangeInt( b2AtomicInt* a, int expected, int desired )
{
#if defined( _MSC_VER )
	return _InterlockedCompareExchange( (long*)&a->value, (long)desired, (long)expected ) == expected;
#elif defined( __GNUC__ ) || defined( __clang__ )
	return __atomic_compare_exchange_n( &a->value, &expected, desired, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST );
#else
#error "Unsupported platform"
#endif
}

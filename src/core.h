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

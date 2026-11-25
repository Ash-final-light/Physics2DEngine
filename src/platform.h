// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

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

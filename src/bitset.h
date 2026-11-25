// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b2BitSet
{
	uint64_t* bits;
	uint32_t blockCapacity;
	uint32_t blockCount;
} b2BitSet;

b2BitSet b2CreateBitSet( uint32_t bitCapacity );
void b2DestroyBitSet( b2BitSet* bitSet );
void b2SetBitCountAndClear( b2BitSet* bitSet, uint32_t bitCount );

static inline void b2SetBit( b2BitSet* bitSet, uint32_t bitIndex )
{
	uint32_t blockIndex = bitIndex / 64;
	B2_ASSERT( blockIndex < bitSet->blockCount );
	bitSet->bits[blockIndex] |= ( (uint64_t)1 << bitIndex % 64 );
}

static inline void b2ClearBit( b2BitSet* bitSet, uint32_t bitIndex )
{
	uint32_t blockIndex = bitIndex / 64;
	if ( blockIndex >= bitSet->blockCount )
	{
		return;
	}
	bitSet->bits[blockIndex] &= ~( (uint64_t)1 << bitIndex % 64 );
}

static inline bool b2GetBit( const b2BitSet* bitSet, uint32_t bitIndex )
{
	uint32_t blockIndex = bitIndex / 64;
	if ( blockIndex >= bitSet->blockCount )
	{
		return false;
	}
	return ( bitSet->bits[blockIndex] & ( (uint64_t)1 << bitIndex % 64 ) ) != 0;
}

static inline int b2GetBitSetBytes( b2BitSet* bitSet )
{
	return bitSet->blockCapacity * sizeof( uint64_t );
}

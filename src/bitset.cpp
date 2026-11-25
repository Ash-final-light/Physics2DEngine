// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "bitset.h"

#include <string.h>

b2BitSet b2CreateBitSet( uint32_t bitCapacity )
{
	b2BitSet bitSet = {};

	bitSet.blockCapacity = ( bitCapacity + sizeof( uint64_t ) * 8 - 1 ) / ( sizeof( uint64_t ) * 8 );
	bitSet.blockCount = 0;
	bitSet.bits = static_cast<uint64_t*>( b2Alloc( bitSet.blockCapacity * sizeof( uint64_t ) ) );
	memset( bitSet.bits, 0, bitSet.blockCapacity * sizeof( uint64_t ) );
	return bitSet;
}

void b2DestroyBitSet( b2BitSet* bitSet )
{
	b2Free( bitSet->bits, bitSet->blockCapacity * sizeof( uint64_t ) );
	bitSet->blockCapacity = 0;
	bitSet->blockCount = 0;
	bitSet->bits = NULL;
}

void b2SetBitCountAndClear( b2BitSet* bitSet, uint32_t bitCount )
{
	uint32_t blockCount = ( bitCount + sizeof( uint64_t ) * 8 - 1 ) / ( sizeof( uint64_t ) * 8 );
	if ( bitSet->blockCapacity < blockCount )
	{
		b2DestroyBitSet( bitSet );
		uint32_t newBitCapacity = bitCount + ( bitCount >> 1 );
		*bitSet = b2CreateBitSet( newBitCapacity );
	}

	bitSet->blockCount = blockCount;
	memset( bitSet->bits, 0, bitSet->blockCount * sizeof( uint64_t ) );
}

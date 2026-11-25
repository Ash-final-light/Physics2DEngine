// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "bitset.h"
#include "test_macros.h"

int BitSetTest( void )
{
	size_t before = b2GetByteCount();
	b2BitSet bitSet = b2CreateBitSet( 169 );
	ENSURE( bitSet.blockCapacity == 3 && bitSet.blockCount == 0 );
	ENSURE( b2GetBitSetBytes( &bitSet ) == 3 * (int)sizeof( uint64_t ) );
	ENSURE( b2GetByteCount() == before + 3 * sizeof( uint64_t ) );

	b2SetBitCountAndClear( &bitSet, 169 );
	ENSURE( bitSet.blockCount == 3 );
	b2SetBit( &bitSet, 0 );
	b2SetBit( &bitSet, 1 );
	b2SetBit( &bitSet, 168 );
	ENSURE( b2GetBit( &bitSet, 0 ) && b2GetBit( &bitSet, 1 ) && b2GetBit( &bitSet, 168 ) );
	ENSURE( b2GetBit( &bitSet, 2 ) == false );
	ENSURE( b2GetBit( &bitSet, 200 ) == false );

	b2ClearBit( &bitSet, 1 );
	b2ClearBit( &bitSet, 200 );
	ENSURE( b2GetBit( &bitSet, 0 ) && b2GetBit( &bitSet, 1 ) == false && b2GetBit( &bitSet, 168 ) );

	b2SetBitCountAndClear( &bitSet, 10 );
	ENSURE( bitSet.blockCount == 1 && b2GetBit( &bitSet, 0 ) == false );

	b2SetBitCountAndClear( &bitSet, 200 );
	ENSURE( bitSet.blockCapacity >= 5 && bitSet.blockCount == 4 );
	b2SetBit( &bitSet, 199 );
	ENSURE( b2GetBit( &bitSet, 199 ) && b2GetBit( &bitSet, 0 ) == false );
	ENSURE( b2GetByteCount() == before + bitSet.blockCapacity * sizeof( uint64_t ) );

	b2DestroyBitSet( &bitSet );
	ENSURE( bitSet.bits == nullptr && bitSet.blockCapacity == 0 && bitSet.blockCount == 0 );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

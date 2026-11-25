// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "table.h"
#include "test_macros.h"

int HashSetInsertTest( void )
{
	size_t before = b2GetByteCount();

	b2HashSet small = b2CreateSet( 1 );
	ENSURE( b2GetSetCapacity( &small ) == 16 && b2GetSetCount( &small ) == 0 );
	b2DestroySet( &small );

	b2HashSet rounded = b2CreateSet( 15 );
	ENSURE( b2GetSetCapacity( &rounded ) == 16 );
	b2DestroySet( &rounded );

	b2HashSet exact = b2CreateSet( 32 );
	ENSURE( b2GetSetCapacity( &exact ) == 32 );
	ENSURE( b2GetHashSetBytes( &exact ) == 32 * (int)sizeof( b2SetItem ) );
	ENSURE( b2GetByteCount() == before + 32 * sizeof( b2SetItem ) );
	b2DestroySet( &exact );

	b2HashSet larger = b2CreateSet( 33 );
	ENSURE( b2GetSetCapacity( &larger ) == 64 );
	b2DestroySet( &larger );
	ENSURE( b2GetByteCount() == before );

	b2HashSet set = b2CreateSet( 64 );
	uint64_t keyA = 1;
	uint32_t slotA = (uint32_t)b2KeyHash( keyA ) & ( set.capacity - 1 );
	uint64_t keyB = 0;
	for ( uint64_t key = 2; key < 100000; ++key )
	{
		if ( ( (uint32_t)b2KeyHash( key ) & ( set.capacity - 1 ) ) == slotA )
		{
			keyB = key;
			break;
		}
	}
	ENSURE( keyB != 0 );

	ENSURE( b2AddKey( &set, keyA ) == false );
	ENSURE( b2AddKey( &set, keyB ) == false );
	ENSURE( b2AddKey( &set, keyA ) == true );
	ENSURE( b2GetSetCount( &set ) == 2 );
	ENSURE( set.items[slotA].key == keyA );
	ENSURE( set.items[slotA].key != keyB );

	bool foundLater = false;
	for ( uint32_t step = 1; step < set.capacity; ++step )
	{
		uint32_t index = ( slotA + step ) & ( set.capacity - 1 );
		if ( set.items[index].key == keyB )
		{
			foundLater = true;
			break;
		}
		if ( set.items[index].key == 0 )
		{
			break;
		}
	}
	ENSURE( foundLater );
	ENSURE( b2ContainsKey( &set, keyA ) && b2ContainsKey( &set, keyB ) );
	ENSURE( b2ContainsHashedKey( &set, keyB, b2KeyHash( keyB ) ) );
	ENSURE( b2ContainsKey( &set, 999 ) == false );

	uint64_t pair = B2_SHAPE_PAIR_KEY( 5, 10 );
	ENSURE( pair == B2_SHAPE_PAIR_KEY( 10, 5 ) );
	ENSURE( pair != B2_SHAPE_PAIR_KEY( 1, 2 ) );
	ENSURE( b2AddKey( &set, pair ) == false );
	ENSURE( b2AddKey( &set, B2_SHAPE_PAIR_KEY( 10, 5 ) ) == true );
	ENSURE( b2AddKey( &set, B2_SHAPE_PAIR_KEY( 1, 2 ) ) == false );
	ENSURE( b2ContainsKey( &set, B2_SHAPE_PAIR_KEY( 5, 10 ) ) );
	ENSURE( b2GetSetCount( &set ) == 4 );
	ENSURE( b2GetSetCapacity( &set ) == 64 );
	ENSURE( b2GetHashSetBytes( &set ) == 64 * (int)sizeof( b2SetItem ) );

	b2DestroySet( &set );
	ENSURE( set.items == nullptr && set.count == 0 && set.capacity == 0 );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

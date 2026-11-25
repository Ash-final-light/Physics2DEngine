// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "table.h"

#include "core.h"
#include "platform.h"

#include <string.h>

b2HashSet b2CreateSet( int capacity )
{
	b2HashSet set = {};

	if ( capacity > 16 )
	{
		set.capacity = (uint32_t)b2RoundUpPowerOf2( capacity );
	}
	else
	{
		set.capacity = 16;
	}

	set.count = 0;
	set.items = static_cast<b2SetItem*>( b2Alloc( set.capacity * sizeof( b2SetItem ) ) );
	memset( set.items, 0, set.capacity * sizeof( b2SetItem ) );
	return set;
}

void b2DestroySet( b2HashSet* set )
{
	b2Free( set->items, set->capacity * sizeof( b2SetItem ) );
	set->items = NULL;
	set->count = 0;
	set->capacity = 0;
}

static int b2FindSlot( const b2HashSet* set, uint64_t key, uint64_t hash )
{
	uint32_t capacity = set->capacity;
	uint32_t index = (uint32_t)hash & ( capacity - 1 );
	const b2SetItem* items = set->items;
	while ( items[index].key != 0 && items[index].key != key )
	{
		index = ( index + 1 ) & ( capacity - 1 );
	}

	return (int)index;
}

static void b2AddKeyHaveCapacity( b2HashSet* set, uint64_t key, uint64_t hash )
{
	int index = b2FindSlot( set, key, hash );
	b2SetItem* items = set->items;

	B2_ASSERT( items[index].key == 0 );
	items[index].key = key;
	set->count += 1;
}

bool b2ContainsKey( const b2HashSet* set, uint64_t key )
{
	B2_VALIDATE( key != 0 );
	uint64_t hash = b2KeyHash( key );
	int index = b2FindSlot( set, key, hash );
	return set->items[index].key == key;
}

bool b2ContainsHashedKey( const b2HashSet* set, uint64_t key, uint64_t hash )
{
	B2_VALIDATE( key != 0 );
	int index = b2FindSlot( set, key, hash );
	return set->items[index].key == key;
}

int b2GetHashSetBytes( b2HashSet* set )
{
	return (int)( set->capacity * sizeof( b2SetItem ) );
}

bool b2AddKey( b2HashSet* set, uint64_t key )
{
	B2_ASSERT( key != 0 );

	uint64_t hash = b2KeyHash( key );
	B2_ASSERT( hash != 0 );

	int index = b2FindSlot( set, key, hash );
	if ( set->items[index].key != 0 )
	{
		B2_ASSERT( set->items[index].key == key );
		return true;
	}

	b2AddKeyHaveCapacity( set, key, hash );
	return false;
}

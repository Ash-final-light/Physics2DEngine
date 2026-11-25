// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "arena_allocator.h"
#include "test_macros.h"

#include <stdint.h>

int StackAllocatorTest( void )
{
	size_t before = b2GetByteCount();
	int entryBytes = 32 * (int)sizeof( b2StackEntry );

	b2Stack stack = b2CreateStack( 128 );
	ENSURE( b2GetStackCapacity( &stack ) == 128 );
	ENSURE( b2GetStackAllocation( &stack ) == 0 && b2GetMaxStackAllocation( &stack ) == 0 );
	ENSURE( stack.index == 0 );
	ENSURE( b2GetByteCount() == before + 128 + entryBytes );

	char* first = static_cast<char*>( b2StackAlloc( &stack, 1, "first" ) );
	char* second = static_cast<char*>( b2StackAlloc( &stack, 64, "second" ) );
	ENSURE( first == stack.data && second == stack.data + 64 );
	ENSURE( ( (uintptr_t)first & ( B2_ALIGNMENT - 1 ) ) == 0 );
	first[0] = 7;
	second[0] = 8;
	ENSURE( stack.index == 128 );
	ENSURE( b2GetStackAllocation( &stack ) == 128 && b2GetMaxStackAllocation( &stack ) == 128 );
	ENSURE( stack.entries.count == 2 );

	char* overflow = static_cast<char*>( b2StackAlloc( &stack, 1, "overflow" ) );
	ENSURE( stack.entries.data[2].usedMalloc == true );
	ENSURE( stack.entries.data[2].data == overflow );
	ENSURE( ( (uintptr_t)overflow & ( B2_ALIGNMENT - 1 ) ) == 0 );
	overflow[0] = 9;
	ENSURE( stack.index == 128 );
	ENSURE( b2GetStackAllocation( &stack ) == 192 && b2GetMaxStackAllocation( &stack ) == 192 );
	ENSURE( b2GetByteCount() == before + 128 + entryBytes + 64 );
	ENSURE( first[0] == 7 && second[0] == 8 );

	b2StackFree( &stack, overflow );
	ENSURE( stack.index == 128 && b2GetStackAllocation( &stack ) == 128 );
	ENSURE( b2GetMaxStackAllocation( &stack ) == 192 );
	ENSURE( b2GetByteCount() == before + 128 + entryBytes );
	ENSURE( first[0] == 7 );

	b2StackFree( &stack, second );
	b2StackFree( &stack, first );
	ENSURE( stack.index == 0 && b2GetStackAllocation( &stack ) == 0 );
	ENSURE( b2GetMaxStackAllocation( &stack ) == 192 && stack.entries.count == 0 );

	char* reused = static_cast<char*>( b2StackAlloc( &stack, 1, "reused" ) );
	ENSURE( reused == stack.data );
	b2StackFree( &stack, reused );

	b2GrowStack( &stack );
	ENSURE( b2GetStackCapacity( &stack ) == 192 + 192 / 2 );
	ENSURE( b2GetStackAllocation( &stack ) == 0 && b2GetMaxStackAllocation( &stack ) == 192 );
	ENSURE( b2GetByteCount() == before + ( 192 + 192 / 2 ) + entryBytes );

	char* wide = static_cast<char*>( b2StackAlloc( &stack, 200, "wide" ) );
	ENSURE( wide == stack.data );
	ENSURE( b2GetStackAllocation( &stack ) == 256 && b2GetMaxStackAllocation( &stack ) == 256 );
	ENSURE( stack.index == 256 );
	wide[0] = 3;
	b2StackFree( &stack, wide );
	ENSURE( stack.index == 0 && b2GetStackAllocation( &stack ) == 0 );
	ENSURE( b2GetMaxStackAllocation( &stack ) == 256 );

	int capacityBefore = b2GetStackCapacity( &stack );
	b2GrowStack( &stack );
	ENSURE( b2GetStackCapacity( &stack ) == capacityBefore );

	b2DestroyStack( &stack );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

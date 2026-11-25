// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include <malloc.h>
#include <stdint.h>

static int s_allocCalls;
static int s_freeCalls;
static int s_alignment;
static size_t s_allocSize;

static void* CountingAlloc( size_t size, int alignment )
{
	s_allocCalls += 1;
	s_alignment = alignment;
	s_allocSize = size;
	return _aligned_malloc( size, (size_t)alignment );
}

static void CountingFree( void* mem, size_t size )
{
	(void)size;
	s_freeCalls += 1;
	_aligned_free( mem );
}

static int CheckAlignment( void* ptr )
{
	return ( ( (uintptr_t)ptr ) & ( B2_ALIGNMENT - 1 ) ) == 0;
}

int MemoryTest( void )
{
	size_t before = b2GetByteCount();
	if ( before != 0 )
	{
		return 1;
	}

	if ( b2Alloc( 0 ) != NULL || b2GetByteCount() != 0 )
	{
		return 1;
	}

	void* block = b2Alloc( 24 );
	if ( block == NULL || CheckAlignment( block ) == 0 || b2GetByteCount() != 24 )
	{
		return 1;
	}

	b2Free( block, 24 );
	b2Free( NULL, 24 );
	if ( b2GetByteCount() != 0 )
	{
		return 1;
	}

	s_allocCalls = 0;
	s_freeCalls = 0;
	s_alignment = 0;
	b2SetAllocator( CountingAlloc, CountingFree );
	void* custom = b2Alloc( 3 );
	if ( custom == NULL || s_allocCalls != 1 || s_alignment != B2_ALIGNMENT || s_allocSize != B2_ALIGNMENT )
	{
		b2SetAllocator( NULL, NULL );
		return 1;
	}
	if ( CheckAlignment( custom ) == 0 || b2GetByteCount() != 3 )
	{
		b2Free( custom, 3 );
		b2SetAllocator( NULL, NULL );
		return 1;
	}

	b2Free( custom, 3 );
	b2SetAllocator( NULL, NULL );
	if ( s_freeCalls != 1 || b2GetByteCount() != 0 )
	{
		return 1;
	}

	return 0;
}

int GrowAllocTest( void )
{
	int* first = static_cast<int*>( b2GrowAlloc( NULL, 0, sizeof( int ) * 2 ) );
	if ( first == NULL || b2GetByteCount() != sizeof( int ) * 2 )
	{
		return 1;
	}
	first[0] = 11;
	first[1] = 22;

	int* second = static_cast<int*>( b2GrowAlloc( first, sizeof( int ) * 2, sizeof( int ) * 4 ) );
	if ( second == NULL || second[0] != 11 || second[1] != 22 )
	{
		return 1;
	}
	if ( b2GetByteCount() != sizeof( int ) * 4 )
	{
		b2Free( second, sizeof( int ) * 4 );
		return 1;
	}

	second[2] = 33;
	second[3] = 44;
	b2Free( second, sizeof( int ) * 4 );
	if ( b2GetByteCount() != 0 )
	{
		return 1;
	}
	return 0;
}

int ZeroAllocTest( void )
{
	unsigned char* block = static_cast<unsigned char*>( b2AllocZero( 8 ) );
	if ( block == NULL || b2GetByteCount() != 8 )
	{
		return 1;
	}
	for ( int i = 0; i < 8; ++i )
	{
		if ( block[i] != 0 )
		{
			b2Free( block, 8 );
			return 1;
		}
	}
	block[0] = 9;
	block[1] = 8;

	unsigned char* grown = static_cast<unsigned char*>( b2GrowAllocZero( block, 8, 16 ) );
	if ( grown == NULL || grown[0] != 9 || grown[1] != 8 || b2GetByteCount() != 16 )
	{
		return 1;
	}
	for ( int i = 2; i < 16; ++i )
	{
		if ( grown[i] != 0 )
		{
			b2Free( grown, 16 );
			return 1;
		}
	}

	b2Free( grown, 16 );
	return b2GetByteCount() == 0 ? 0 : 1;
}

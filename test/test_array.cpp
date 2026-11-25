// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "container.h"
#include "test_macros.h"

int ArrayCreateTest( void )
{
	size_t before = b2GetByteCount();
	b2Array( int ) empty;
	b2Array_Create( empty );
	ENSURE( empty.data == nullptr );
	ENSURE( empty.count == 0 );
	ENSURE( empty.capacity == 0 );
	b2Array_Destroy( empty );
	ENSURE( empty.data == nullptr && empty.count == 0 && empty.capacity == 0 );
	ENSURE( b2GetByteCount() == before );

	b2Array( int ) values;
	b2Array_CreateN( values, 4 );
	ENSURE( values.data != nullptr );
	ENSURE( values.count == 0 );
	ENSURE( values.capacity == 4 );
	ENSURE( b2GetByteCount() == before + 4 * sizeof( int ) );
	b2Array_Destroy( values );
	ENSURE( values.capacity == 0 && values.data == nullptr );
	ENSURE( b2GetByteCount() == before );

	b2Array( int ) zero;
	b2Array_CreateN( zero, 0 );
	ENSURE( zero.data == nullptr && zero.capacity == 0 );
	b2Array_Destroy( zero );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

int ArrayReserveTest( void )
{
	size_t before = b2GetByteCount();
	b2Array( int ) values;
	b2Array_Create( values );
	b2Array_Reserve( values, 4 );
	ENSURE( values.capacity == 4 && values.count == 0 );
	values.data[0] = 7;
	int* saved = values.data;
	b2Array_Reserve( values, 2 );
	b2Array_Reserve( values, 4 );
	ENSURE( values.capacity == 4 && values.data == saved && values.data[0] == 7 );

	b2Array_Grow( values, 5 );
	ENSURE( values.capacity == 8 && values.data[0] == 7 );
	b2Array_Grow( values, 8 );
	ENSURE( values.capacity == 8 );
	ENSURE( b2GetByteCount() == before + 8 * sizeof( int ) );
	b2Array_Destroy( values );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

int ArrayPushTest( void )
{
	b2Array( int ) values;
	b2Array_Create( values );
	b2Array_Push( values, 1 );
	b2Array_Push( values, 2 );
	ENSURE( values.count == 2 && values.data[0] == 1 && values.data[1] == 2 );

	for ( int i = 2; i < 8; ++i )
	{
		b2Array_Push( values, i );
	}
	ENSURE( values.capacity == 8 && values.count == 8 );
	b2Array_Push( values, values.data[0] );
	ENSURE( values.count == 9 && values.capacity == 16 && values.data[8] == 1 );

	b2Array( int ) placed;
	b2Array_Create( placed );
	for ( int i = 0; i < 10; ++i )
	{
		int* slot = b2Array_Emplace( placed );
		*slot = i;
	}
	int sum = 0;
	for ( int i = 0; i < placed.count; ++i )
	{
		sum += placed.data[i];
	}
	ENSURE( sum == 45 && placed.count == 10 );
	b2Array_Destroy( placed );
	b2Array_Destroy( values );
	return 0;
}

int ArrayIndexTest( void )
{
	b2Array( int ) values;
	b2Array_Create( values );
	b2Array_Push( values, 10 );
	b2Array_Push( values, 20 );
	b2Array_Push( values, 30 );
	ENSURE( *b2Array_Get( values, 0 ) == 10 );
	ENSURE( *b2Array_Get( values, 2 ) == 30 );
	const b2Array( int )& readonly = values;
	ENSURE( *b2Array_Get( readonly, 1 ) == 20 );

	int sum = 0;
	for ( int i = 0; i < values.count; ++i )
	{
		sum += values.data[i];
	}
	ENSURE( sum == 60 );
	b2Array_Destroy( values );
	return 0;
}

int ArrayPopTest( void )
{
	b2Array( int ) values;
	b2Array_Create( values );
	b2Array_Push( values, 10 );
	b2Array_Push( values, 20 );
	b2Array_Push( values, 30 );
	ENSURE( b2Array_Pop( values ) == 30 && values.count == 2 );
	ENSURE( b2Array_Pop( values ) == 20 && values.count == 1 );
	ENSURE( b2Array_Pop( values ) == 10 && values.count == 0 );
	ENSURE( values.data[0] == 10 );
	b2Array_Destroy( values );
	return 0;
}

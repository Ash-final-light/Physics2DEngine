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

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "id_pool.h"
#include "test_macros.h"

static bool IdPoolContains( const b2IdPool* pool, int id )
{
	for ( int i = 0; i < pool->freeArray.count; ++i )
	{
		if ( pool->freeArray.data[i] == id )
		{
			return true;
		}
	}
	return false;
}

int IdPoolTest( void )
{
	size_t before = b2GetByteCount();
	b2IdPool pool = b2CreateIdPool();
	ENSURE( b2GetIdCount( &pool ) == 0 && b2GetIdCapacity( &pool ) == 0 );
	ENSURE( b2GetIdBytes( &pool ) == 32 * (int)sizeof( int ) );
	ENSURE( b2GetByteCount() == before + 32 * sizeof( int ) );

	for ( int i = 0; i < 40; ++i )
	{
		ENSURE( b2AllocId( &pool ) == i );
	}
	ENSURE( b2GetIdCount( &pool ) == 40 && b2GetIdCapacity( &pool ) == 40 );
	ENSURE( pool.freeArray.count == 0 );

	for ( int id = 38; id >= 0; id -= 2 )
	{
		b2FreeId( &pool, id );
	}
	ENSURE( pool.freeArray.count == 20 );
	ENSURE( b2GetIdCount( &pool ) == 20 && b2GetIdCapacity( &pool ) == 40 );
	for ( int id = 1; id < 40; id += 2 )
	{
		ENSURE( IdPoolContains( &pool, id ) == false );
	}

	for ( int id = 0; id < 40; id += 2 )
	{
		ENSURE( b2AllocId( &pool ) == id );
		ENSURE( IdPoolContains( &pool, id ) == false );
	}
	ENSURE( pool.freeArray.count == 0 );
	ENSURE( b2GetIdCount( &pool ) == 40 && b2GetIdCapacity( &pool ) == 40 );

	ENSURE( b2AllocId( &pool ) == 40 );
	ENSURE( b2GetIdCount( &pool ) == 41 && b2GetIdCapacity( &pool ) == 41 );

	for ( int id = 40; id >= 0; --id )
	{
		b2FreeId( &pool, id );
	}
	ENSURE( pool.freeArray.count == 41 && pool.freeArray.capacity == 64 );
	ENSURE( b2GetIdCount( &pool ) == 0 && b2GetIdCapacity( &pool ) == 41 );
	ENSURE( b2GetIdBytes( &pool ) == 64 * (int)sizeof( int ) );
	ENSURE( b2GetByteCount() == before + 64 * sizeof( int ) );

	for ( int id = 0; id <= 40; ++id )
	{
		ENSURE( b2AllocId( &pool ) == id );
	}
	ENSURE( b2GetIdCount( &pool ) == 41 && b2GetIdCapacity( &pool ) == 41 );
	ENSURE( pool.freeArray.count == 0 );

	b2DestroyIdPool( &pool );
	ENSURE( b2GetIdCount( &pool ) == 0 && b2GetIdCapacity( &pool ) == 0 );
	ENSURE( pool.freeArray.data == nullptr );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

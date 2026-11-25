// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/base.h"
#include "box2d/types.h"

#include <math.h>

int WorldCreateTest( void )
{
	size_t before = b2GetByteCount();
	b2WorldDef def = b2DefaultWorldDef();
	ENSURE( def.gravity.y == -10.0f );
	b2WorldId worldId = b2CreateWorld( &def );
	ENSURE( worldId.index1 == 1 );
	ENSURE( b2GetByteCount() > before );
	b2DestroyWorld( worldId );
	ENSURE( b2GetByteCount() == before );

	def.gravity.y = NAN;
	b2WorldId invalid = b2CreateWorld( &def );
	ENSURE( invalid.index1 == 0 );
	ENSURE( b2GetByteCount() == before );

	def = b2DefaultWorldDef();
	b2WorldId first = b2CreateWorld( &def );
	b2WorldId second = b2CreateWorld( &def );
	ENSURE( first.index1 != 0 && second.index1 != 0 && first.index1 != second.index1 );
	b2DestroyWorld( second );
	b2DestroyWorld( first );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

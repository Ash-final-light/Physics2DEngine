// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/base.h"
#include "box2d/collision.h"

int DynamicTreeCreateTest( void )
{
	size_t before = b2GetByteCount();
	b2DynamicTree* empty = b2CreateDynamicTree( 0 );
	ENSURE( empty != NULL );
	ENSURE( b2GetByteCount() > before );
	b2DestroyDynamicTree( empty );
	ENSURE( b2GetByteCount() == before );

	b2DynamicTree* small = b2CreateDynamicTree( 4 );
	b2DynamicTree* large = b2CreateDynamicTree( 32 );
	ENSURE( small != NULL && large != NULL );
	ENSURE( b2GetByteCount() > before );
	b2DestroyDynamicTree( large );
	b2DestroyDynamicTree( small );
	b2DestroyDynamicTree( NULL );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

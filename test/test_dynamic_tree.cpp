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

static bool SameBounds( b2AABB a, b2AABB b )
{
	return a.lowerBound.x == b.lowerBound.x && a.lowerBound.y == b.lowerBound.y && a.upperBound.x == b.upperBound.x &&
		   a.upperBound.y == b.upperBound.y;
}

int DynamicTreeInsertTest( void )
{
	b2AABB first = { { 0.0f, 0.0f }, { 1.0f, 1.0f } };
	b2AABB second = { { 3.0f, 0.0f }, { 4.0f, 2.0f } };
	size_t before = b2GetByteCount();
	b2DynamicTree* tree = b2CreateDynamicTree( 16 );
	int id1 = b2CreateTreeProxy( tree, first, 1, 42 );
	ENSURE( b2DynamicTree_GetProxyCount( tree ) == 1 );
	ENSURE( b2DynamicTree_GetUserData( tree, id1 ) == 42 );
	ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, id1 ), first ) );

	int id2 = b2CreateTreeProxy( tree, second, 1, 43 );
	ENSURE( id2 != id1 );
	ENSURE( b2DynamicTree_GetProxyCount( tree ) == 2 );
	ENSURE( b2DynamicTree_GetUserData( tree, id1 ) == 42 );
	ENSURE( b2DynamicTree_GetUserData( tree, id2 ) == 43 );
	ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, id1 ), first ) );
	ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, id2 ), second ) );
	b2DestroyDynamicTree( tree );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

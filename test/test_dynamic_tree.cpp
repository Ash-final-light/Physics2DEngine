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

int DynamicTreeRemoveTest( void )
{
	b2AABB first = { { 0.0f, 0.0f }, { 1.0f, 1.0f } };
	b2AABB second = { { 3.0f, 0.0f }, { 4.0f, 2.0f } };
	b2AABB third = { { 8.0f, 8.0f }, { 9.0f, 9.0f } };
	size_t before = b2GetByteCount();
	b2DynamicTree* tree = b2CreateDynamicTree( 16 );
	int id1 = b2CreateTreeProxy( tree, first, 1, 42 );
	int id2 = b2CreateTreeProxy( tree, second, 1, 43 );
	b2DestroyTreeProxy( tree, id1 );
	ENSURE( b2DynamicTree_GetProxyCount( tree ) == 1 );
	ENSURE( b2DynamicTree_GetUserData( tree, id2 ) == 43 );
	ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, id2 ), second ) );

	int id3 = b2CreateTreeProxy( tree, third, 1, 44 );
	ENSURE( id3 == id1 );
	ENSURE( b2DynamicTree_GetProxyCount( tree ) == 2 );
	ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, id3 ), third ) );
	b2DestroyTreeProxy( tree, id2 );
	b2DestroyTreeProxy( tree, id3 );
	ENSURE( b2DynamicTree_GetProxyCount( tree ) == 0 );
	b2DestroyDynamicTree( tree );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

int DynamicTreeMoveTest( void )
{
	b2AABB first = { { 0.0f, 0.0f }, { 1.0f, 1.0f } };
	b2AABB moved = { { 5.0f, 5.0f }, { 6.0f, 7.0f } };
	b2AABB enlarged = { { 4.0f, 4.0f }, { 8.0f, 8.0f } };
	b2AABB other = { { -3.0f, -1.0f }, { -1.0f, 1.0f } };
	b2DynamicTree* tree = b2CreateDynamicTree( 16 );
	int id1 = b2CreateTreeProxy( tree, first, 1, 7 );
	int id2 = b2CreateTreeProxy( tree, other, 1, 8 );
	b2DynamicTree_MoveProxy( tree, id1, moved );
	ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, id1 ), moved ) );
	ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, id2 ), other ) );
	ENSURE( b2DynamicTree_GetUserData( tree, id1 ) == 7 );
	b2DynamicTree_EnlargeProxy( tree, id1, enlarged );
	ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, id1 ), enlarged ) );
	ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, id2 ), other ) );
	b2DestroyDynamicTree( tree );
	return 0;
}

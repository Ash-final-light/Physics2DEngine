// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "aabb.h"
#include "dynamic_tree.h"

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

int DynamicTreeCategoryTest( void )
{
	b2AABB box = { { 0.0f, 0.0f }, { 1.0f, 1.0f } };
	b2DynamicTree* tree = b2CreateDynamicTree( 16 );
	int id1 = b2CreateTreeProxy( tree, box, 0x1ull, 1 );
	int id2 = b2CreateTreeProxy( tree, box, 0x4ull, 2 );
	ENSURE( b2DynamicTree_GetCategoryBits( tree, id1 ) == 0x1ull );
	ENSURE( b2DynamicTree_GetCategoryBits( tree, id2 ) == 0x4ull );
	b2DynamicTree_SetCategoryBits( tree, id1, 0xAAull );
	ENSURE( b2DynamicTree_GetCategoryBits( tree, id1 ) == 0xAAull );
	ENSURE( b2DynamicTree_GetCategoryBits( tree, id2 ) == 0x4ull );
	ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, id1 ), box ) );
	b2DestroyDynamicTree( tree );
	return 0;
}

static int CountRebuiltLeaves( const b2DynamicTree* tree, int nodeIndex )
{
	const b2TreeNode* node = tree->nodes + nodeIndex;
	if ( b2IsLeaf( node ) )
	{
		int proxyId = b2GetProxyId( node );
		if ( tree->proxies[proxyId].node != nodeIndex )
		{
			return -1;
		}
		return 1;
	}

	int pair = b2GetLeftChild( node );
	if ( tree->parents[pair] != nodeIndex || tree->parents[pair + 1] != nodeIndex )
	{
		return -1;
	}
	if ( b2AABB_Contains( node->aabb, tree->nodes[pair].aabb ) == false ||
		 b2AABB_Contains( node->aabb, tree->nodes[pair + 1].aabb ) == false )
	{
		return -1;
	}

	int left = CountRebuiltLeaves( tree, pair );
	int right = CountRebuiltLeaves( tree, pair + 1 );
	if ( left < 0 || right < 0 )
	{
		return -1;
	}
	return left + right;
}

int DynamicTreeRebuildTest( void )
{
	b2AABB boxes[8];
	uint64_t users[8];
	size_t before = b2GetByteCount();
	b2DynamicTree* tree = b2CreateDynamicTree( 16 );
	int ids[8];
	for ( int i = 0; i < 8; ++i )
	{
		boxes[i] = { { (float)i, 0.0f }, { (float)i + 0.5f, 1.0f } };
		users[i] = (uint64_t)( 100 + i );
		ids[i] = b2CreateTreeProxy( tree, boxes[i], 1, users[i] );
	}

	int built = b2DynamicTree_Rebuild( tree, true );
	ENSURE( built == 8 );
	ENSURE( b2DynamicTree_GetProxyCount( tree ) == 8 );
	ENSURE( CountRebuiltLeaves( tree, B2_ROOT_NODE ) == 8 );
	for ( int i = 0; i < 8; ++i )
	{
		ENSURE( b2DynamicTree_GetUserData( tree, ids[i] ) == users[i] );
		ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, ids[i] ), boxes[i] ) );
	}
	b2DynamicTree_Validate( tree );
	ENSURE( b2DynamicTree_Rebuild( tree, false ) == 0 );

	b2DynamicTree* empty = b2CreateDynamicTree( 16 );
	ENSURE( b2DynamicTree_Rebuild( empty, true ) == 0 );
	b2DestroyDynamicTree( empty );
	b2DestroyDynamicTree( tree );
	ENSURE( b2GetByteCount() == before );
	return 0;
}

int DynamicTreeRefitTest( void )
{
	b2DynamicTree* tree = b2CreateDynamicTree( 16 );
	b2AABB boxes[4];
	int ids[4];
	for ( int i = 0; i < 4; ++i )
	{
		boxes[i] = { { (float)i, 0.0f }, { (float)i + 0.4f, 0.4f } };
		ids[i] = b2CreateTreeProxy( tree, boxes[i], 1, (uint64_t)i );
	}
	ENSURE( b2DynamicTree_Rebuild( tree, true ) == 4 );
	int none[4];
	ENSURE( b2DynamicTree_GatherMovedProxies( tree, none ) == 0 );

	b2AABB grown = { { 0.0f, 0.0f }, { 0.4f, 5.0f } };
	b2DynamicTree_MarkProxyMoved( tree, ids[0], grown );
	int moved[4];
	int movedCount = b2DynamicTree_GatherMovedProxies( tree, moved );
	ENSURE( movedCount == 1 );
	ENSURE( moved[0] == ids[0] );
	ENSURE( b2AABB_Contains( tree->nodes[B2_ROOT_NODE].aabb, grown ) == false );
	b2DynamicTree_Refit( tree );
	ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, ids[0] ), grown ) );
	ENSURE( b2AABB_Contains( tree->nodes[B2_ROOT_NODE].aabb, grown ) );
	for ( int i = 1; i < 4; ++i )
	{
		ENSURE( SameBounds( b2DynamicTree_GetProxyBounds( tree, ids[i] ), boxes[i] ) );
	}
	b2DestroyDynamicTree( tree );
	return 0;
}

static bool CollectOverlap( int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	int* hits = static_cast<int*>( context );
	hits[proxyId] = 1;
	return true;
}

int DynamicTreeOverlapTest( void )
{
	b2DynamicTree* tree = b2CreateDynamicTree( 16 );
	b2AABB boxes[3] = {
		{ { 0.0f, 0.0f }, { 1.0f, 1.0f } },
		{ { 0.5f, 0.5f }, { 1.5f, 1.5f } },
		{ { 4.0f, 4.0f }, { 5.0f, 5.0f } },
	};
	uint64_t categories[3] = { 0x1ull, 0x2ull, 0x4ull };
	int ids[3];
	for ( int i = 0; i < 3; ++i )
	{
		ids[i] = b2CreateTreeProxy( tree, boxes[i], categories[i], (uint64_t)i );
	}

	b2AABB query = { { 0.2f, 0.2f }, { 1.2f, 1.2f } };
	int hits[16] = {};
	b2TreeStats stats = b2DynamicTree_OverlapAABB( tree, query, 0xFFFFFFFFull, CollectOverlap, hits );
	ENSURE( stats.leafVisits == 2 );
	for ( int i = 0; i < 3; ++i )
	{
		bool overlaps = b2AABB_Overlaps( boxes[i], query );
		ENSURE( ( hits[ids[i]] == 1 ) == overlaps );
	}

	int filtered[16] = {};
	stats = b2DynamicTree_OverlapAABB( tree, query, 0x1ull, CollectOverlap, filtered );
	ENSURE( filtered[ids[0]] == 1 );
	ENSURE( filtered[ids[1]] == 0 );
	ENSURE( stats.leafVisits == 1 );

	int allHits[16] = {};
	b2AABB wide = { { -1.0f, -1.0f }, { 6.0f, 6.0f } };
	stats = b2DynamicTree_OverlapAABBAll( tree, wide, CollectOverlap, allHits );
	ENSURE( stats.leafVisits == 3 );
	ENSURE( allHits[ids[0]] == 1 && allHits[ids[1]] == 1 && allHits[ids[2]] == 1 );

	b2DynamicTree* empty = b2CreateDynamicTree( 16 );
	stats = b2DynamicTree_OverlapAABB( empty, query, 0xFFFFFFFFull, CollectOverlap, hits );
	ENSURE( stats.nodeVisits == 0 && stats.leafVisits == 0 );
	b2DestroyDynamicTree( empty );
	b2DestroyDynamicTree( tree );
	return 0;
}

struct RayHit
{
	const b2DynamicTree* tree;
	int proxyId;
	float fraction;
};

static float ClosestTreeRay( const b2RayCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	RayHit* hit = static_cast<RayHit*>( context );
	b2Vec2 end = b2MulAdd( input->origin, input->maxFraction, input->translation );
	b2CastOutput output = b2AABB_RayCast( b2DynamicTree_GetProxyBounds( hit->tree, proxyId ), input->origin, end );
	if ( output.hit == false )
	{
		return input->maxFraction;
	}
	float fraction = output.fraction * input->maxFraction;
	if ( fraction < hit->fraction )
	{
		hit->fraction = fraction;
		hit->proxyId = proxyId;
	}
	return fraction;
}

int DynamicTreeRayTest( void )
{
	b2DynamicTree* tree = b2CreateDynamicTree( 16 );
	b2AABB nearBox = { { 2.0f, -0.5f }, { 3.0f, 0.5f } };
	b2AABB farBox = { { 6.0f, -0.5f }, { 7.0f, 0.5f } };
	b2AABB offBox = { { 2.0f, 3.0f }, { 3.0f, 4.0f } };
	int nearId = b2CreateTreeProxy( tree, nearBox, 0x1ull, 1 );
	int farId = b2CreateTreeProxy( tree, farBox, 0x1ull, 2 );
	int offId = b2CreateTreeProxy( tree, offBox, 0x1ull, 3 );
	b2RayCastInput input = { { 0.0f, 0.0f }, { 10.0f, 0.0f }, 1.0f };
	RayHit hit = { tree, -1, 2.0f };
	b2DynamicTree_CastRay( tree, &input, 0xFFFFFFFFull, ClosestTreeRay, &hit );
	ENSURE( hit.proxyId == nearId );
	ENSURE( hit.fraction > 0.15f && hit.fraction < 0.25f );

	float best = 2.0f;
	int bestId = -1;
	b2AABB boxes[3] = { nearBox, farBox, offBox };
	int ids[3] = { nearId, farId, offId };
	for ( int i = 0; i < 3; ++i )
	{
		b2CastOutput output = b2AABB_RayCast( boxes[i], input.origin, { 10.0f, 0.0f } );
		if ( output.hit && output.fraction < best )
		{
			best = output.fraction;
			bestId = ids[i];
		}
	}
	ENSURE( bestId == hit.proxyId );
	ENSURE_SMALL( best - hit.fraction, 0.0001f );

	RayHit masked = { tree, -1, 2.0f };
	b2DynamicTree_CastRay( tree, &input, 0x1ull, ClosestTreeRay, &masked );
	b2DynamicTree_SetCategoryBits( tree, nearId, 0x2ull );
	RayHit skipped = { tree, -1, 2.0f };
	b2DynamicTree_CastRay( tree, &input, 0x1ull, ClosestTreeRay, &skipped );
	ENSURE( skipped.proxyId == farId );

	b2RayCastInput miss = { { 0.0f, 8.0f }, { 10.0f, 0.0f }, 1.0f };
	RayHit missed = { tree, -1, 2.0f };
	b2DynamicTree_CastRay( tree, &miss, 0xFFFFFFFFull, ClosestTreeRay, &missed );
	ENSURE( missed.proxyId == -1 );
	(void)offId;
	(void)masked;
	b2DestroyDynamicTree( tree );
	return 0;
}

struct BoxHit
{
	const b2DynamicTree* tree;
	int proxyId;
	float fraction;
};

static float ClosestTreeBox( const b2BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	(void)userData;
	BoxHit* hit = static_cast<BoxHit*>( context );
	b2Vec2 extension = b2AABB_Extents( input->box );
	b2AABB target = b2DynamicTree_GetProxyBounds( hit->tree, proxyId );
	b2AABB expanded = {
		{ target.lowerBound.x - extension.x, target.lowerBound.y - extension.y },
		{ target.upperBound.x + extension.x, target.upperBound.y + extension.y },
	};
	b2Vec2 origin = b2AABB_Center( input->box );
	b2Vec2 end = b2MulAdd( origin, input->maxFraction, input->translation );
	b2CastOutput output = b2AABB_RayCast( expanded, origin, end );
	if ( output.hit == false )
	{
		return input->maxFraction;
	}
	float fraction = output.fraction * input->maxFraction;
	if ( fraction < hit->fraction )
	{
		hit->fraction = fraction;
		hit->proxyId = proxyId;
	}
	return fraction;
}

int DynamicTreeBoxCastTest( void )
{
	b2DynamicTree* tree = b2CreateDynamicTree( 16 );
	b2AABB nearBox = { { 4.0f, -1.0f }, { 5.0f, 1.0f } };
	b2AABB farBox = { { 8.0f, -1.0f }, { 9.0f, 1.0f } };
	int nearId = b2CreateTreeProxy( tree, nearBox, 0x1ull, 1 );
	int farId = b2CreateTreeProxy( tree, farBox, 0x1ull, 2 );
	b2BoxCastInput input = {};
	input.box = { { -0.5f, -0.5f }, { 0.5f, 0.5f } };
	input.translation = { 10.0f, 0.0f };
	input.maxFraction = 1.0f;
	BoxHit hit = { tree, -1, 2.0f };
	b2DynamicTree_CastBox( tree, &input, 0xFFFFFFFFull, ClosestTreeBox, &hit );
	ENSURE( hit.proxyId == nearId );
	ENSURE( hit.fraction > 0.30f && hit.fraction < 0.40f );

	b2Vec2 extension = b2AABB_Extents( input.box );
	float best = 2.0f;
	int bestId = -1;
	b2AABB boxes[2] = { nearBox, farBox };
	int ids[2] = { nearId, farId };
	for ( int i = 0; i < 2; ++i )
	{
		b2AABB expanded = {
			{ boxes[i].lowerBound.x - extension.x, boxes[i].lowerBound.y - extension.y },
			{ boxes[i].upperBound.x + extension.x, boxes[i].upperBound.y + extension.y },
		};
		b2CastOutput output = b2AABB_RayCast( expanded, { 0.0f, 0.0f }, { 10.0f, 0.0f } );
		if ( output.hit && output.fraction < best )
		{
			best = output.fraction;
			bestId = ids[i];
		}
	}
	ENSURE( bestId == hit.proxyId );
	ENSURE_SMALL( best - hit.fraction, 0.0001f );

	b2DynamicTree_SetCategoryBits( tree, nearId, 0x2ull );
	BoxHit skipped = { tree, -1, 2.0f };
	b2DynamicTree_CastBox( tree, &input, 0x1ull, ClosestTreeBox, &skipped );
	ENSURE( skipped.proxyId == farId );

	b2BoxCastInput miss = input;
	miss.box = { { -0.5f, 4.0f }, { 0.5f, 5.0f } };
	BoxHit missed = { tree, -1, 2.0f };
	b2DynamicTree_CastBox( tree, &miss, 0xFFFFFFFFull, ClosestTreeBox, &missed );
	ENSURE( missed.proxyId == -1 );
	b2DestroyDynamicTree( tree );
	return 0;
}

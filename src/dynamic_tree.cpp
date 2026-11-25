// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "dynamic_tree.h"

#include "core.h"

#include <string.h>

static_assert( B2_ROOT_NODE == 0, "bad root" );
static_assert( sizeof( b2TreeNode ) == 32, "expected size" );
static_assert( sizeof( b2TreeProxy ) == 24, "expected size" );

void b2CreateDynamicTreeInternal( b2DynamicTree* tree, int proxyCapacity )
{
	int capacity = b2MaxInt( proxyCapacity, 16 );
	memset( tree, 0, sizeof( b2DynamicTree ) );
	tree->nodeCapacity = 2 * capacity;
	tree->nodes = static_cast<b2TreeNode*>( b2AllocZero( tree->nodeCapacity * sizeof( b2TreeNode ) ) );
	tree->parents = static_cast<int32_t*>( b2AllocZero( tree->nodeCapacity * sizeof( int32_t ) ) );
	tree->pairFreeList = B2_NULL_INDEX;
	tree->nodes[B2_ROOT_NODE] = b2MakeEmptyNode();
	tree->nodes[B2_ROOT_NODE + 1] = b2MakeEmptyNode();
	tree->parents[B2_ROOT_NODE] = B2_NULL_INDEX;
	tree->parents[B2_ROOT_NODE + 1] = B2_NULL_INDEX;
	tree->nodeEnd = 2;
	tree->dfsOrdered = true;
	tree->proxyCapacity = capacity;
	tree->proxyCount = 0;
	tree->proxies = static_cast<b2TreeProxy*>( b2AllocZero( tree->proxyCapacity * sizeof( b2TreeProxy ) ) );
	for ( int i = 0; i < tree->proxyCapacity - 1; ++i )
	{
		tree->proxies[i].node = B2_NULL_INDEX;
		tree->proxies[i].next = i + 1;
	}

	tree->proxies[tree->proxyCapacity - 1].node = B2_NULL_INDEX;
	tree->proxies[tree->proxyCapacity - 1].next = B2_NULL_INDEX;
	tree->proxyFreeList = 0;
	tree->leafIndices = NULL;
	tree->leafNodes = NULL;
	tree->leafBoxes = NULL;
	tree->leafCenters = NULL;
	tree->binIndices = NULL;
	tree->rebuildCapacity = 0;
}

void b2DestroyDynamicTreeInternal( b2DynamicTree* tree )
{
	b2Free( tree->nodes, tree->nodeCapacity * sizeof( b2TreeNode ) );
	b2Free( tree->parents, tree->nodeCapacity * sizeof( int32_t ) );
	b2Free( tree->proxies, tree->proxyCapacity * sizeof( b2TreeProxy ) );
	b2Free( tree->swapNodes, tree->nodeCapacity * sizeof( b2TreeNode ) );
	b2Free( tree->leafIndices, tree->rebuildCapacity * sizeof( int32_t ) );
	b2Free( tree->leafNodes, tree->rebuildCapacity * sizeof( b2TreeNode ) );
	b2Free( tree->leafBoxes, tree->rebuildCapacity * sizeof( b2AABB ) );
	b2Free( tree->leafCenters, tree->rebuildCapacity * sizeof( b2Vec2 ) );
	b2Free( tree->binIndices, tree->rebuildCapacity * sizeof( int32_t ) );
	memset( tree, 0, sizeof( b2DynamicTree ) );
}

b2DynamicTree* b2CreateDynamicTree( int proxyCapacity )
{
	b2DynamicTree* tree = static_cast<b2DynamicTree*>( b2Alloc( sizeof( b2DynamicTree ) ) );
	b2CreateDynamicTreeInternal( tree, proxyCapacity );
	return tree;
}

void b2DestroyDynamicTree( b2DynamicTree* tree )
{
	if ( tree == NULL )
	{
		return;
	}

	b2DestroyDynamicTreeInternal( tree );
	b2Free( tree, sizeof( b2DynamicTree ) );
}

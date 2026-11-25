// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/collision.h"
#include "box2d/math_functions.h"

#include <math.h>

#define B2_MOVED_NODE ( 1u << 30 )
#define B2_LEAF_NODE ( 1u << 31 )
#define B2_NODE_INDEX_MASK ( 0xFFFFFFFFu & ~( B2_MOVED_NODE | B2_LEAF_NODE ) )
#define B2_EMPTY_NODE ( B2_NODE_INDEX_MASK | B2_LEAF_NODE )
#define B2_ROOT_NODE 0

typedef struct b2TreeNode
{
	b2AABB aabb;
	uint64_t padding;
	uint32_t flagIndex;
	union
	{
		int32_t height;
		int32_t shapeIndex;
	};
} b2TreeNode;

typedef struct b2TreeProxy
{
	uint64_t userData;
	uint64_t categoryBits;
	int32_t node;
	int32_t next;
} b2TreeProxy;

struct b2DynamicTree
{
	b2TreeNode* nodes;
	int32_t* parents;
	b2TreeProxy* proxies;
	int32_t nodeEnd;
	int32_t nodeCapacity;
	int32_t pairFreeList;
	int32_t proxyCount;
	int32_t proxyCapacity;
	int32_t proxyFreeList;
	b2TreeNode* swapNodes;
	int32_t* leafIndices;
	b2TreeNode* leafNodes;
	b2AABB* leafBoxes;
	b2Vec2* leafCenters;
	int32_t* binIndices;
	int32_t rebuildCapacity;
	bool dfsOrdered;
};

B2_FORCE_INLINE bool b2IsLeaf( const b2TreeNode* node )
{
	return ( node->flagIndex & B2_LEAF_NODE ) == B2_LEAF_NODE;
}

B2_FORCE_INLINE bool b2IsEmptyNode( const b2TreeNode* node )
{
	return node->flagIndex == B2_EMPTY_NODE;
}

B2_FORCE_INLINE int b2GetLeftChild( const b2TreeNode* node )
{
	return (int)( node->flagIndex & B2_NODE_INDEX_MASK );
}

B2_FORCE_INLINE int b2GetProxyId( const b2TreeNode* node )
{
	return (int)( node->flagIndex & B2_NODE_INDEX_MASK );
}

B2_FORCE_INLINE b2TreeNode b2MakeEmptyNode( void )
{
	return b2TreeNode{
		.aabb =
			{
				.lowerBound = { .x = INFINITY, .y = INFINITY },
				.upperBound = { .x = -INFINITY, .y = -INFINITY },
			},
		.flagIndex = B2_EMPTY_NODE,
		.height = 0,
	};
}

void b2CreateDynamicTreeInternal( b2DynamicTree* tree, int proxyCapacity );
void b2DestroyDynamicTreeInternal( b2DynamicTree* tree );

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "dynamic_tree.h"

#include "core.h"

#include <float.h>
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

static int b2AllocateSiblingPair( b2DynamicTree* tree )
{
	if ( tree->pairFreeList != B2_NULL_INDEX )
	{
		int pair = tree->pairFreeList;
		tree->pairFreeList = tree->parents[pair];
		return pair;
	}

	if ( tree->nodeEnd + 2 > tree->nodeCapacity )
	{
		int oldCapacity = tree->nodeCapacity;
		int newCapacity = oldCapacity + ( oldCapacity >> 1 );
		newCapacity += newCapacity & 1;
		tree->nodes = B2_GROW_ZERO( tree->nodes, oldCapacity, newCapacity );
		tree->parents = B2_GROW_ZERO( tree->parents, oldCapacity, newCapacity );
		tree->nodeCapacity = newCapacity;
	}

	int pair = tree->nodeEnd;
	tree->nodeEnd += 2;
	return pair;
}

static int b2AllocateProxy( b2DynamicTree* tree )
{
	if ( tree->proxyFreeList == B2_NULL_INDEX )
	{
		B2_ASSERT( tree->proxyCount == tree->proxyCapacity );
		int oldCapacity = tree->proxyCapacity;
		tree->proxyCapacity += b2MaxInt( oldCapacity >> 1, 1 );
		tree->proxies = B2_GROW_ZERO( tree->proxies, oldCapacity, tree->proxyCapacity );
		for ( int i = oldCapacity; i < tree->proxyCapacity - 1; ++i )
		{
			tree->proxies[i].node = B2_NULL_INDEX;
			tree->proxies[i].next = i + 1;
		}

		tree->proxies[tree->proxyCapacity - 1].node = B2_NULL_INDEX;
		tree->proxies[tree->proxyCapacity - 1].next = B2_NULL_INDEX;
		tree->proxyFreeList = oldCapacity;
	}

	int proxyIndex = tree->proxyFreeList;
	tree->proxyFreeList = tree->proxies[proxyIndex].next;
	memset( tree->proxies + proxyIndex, 0, sizeof( b2TreeProxy ) );
	tree->proxies[proxyIndex].node = B2_NULL_INDEX;
	tree->proxies[proxyIndex].next = B2_NULL_INDEX;
	++tree->proxyCount;
	return proxyIndex;
}

static float b2TreePerimeter( b2AABB a )
{
	float wx = a.upperBound.x - a.lowerBound.x;
	float wy = a.upperBound.y - a.lowerBound.y;
	return 2.0f * ( wx + wy );
}

static int b2GetNodeHeight( const b2TreeNode* node )
{
	return b2IsLeaf( node ) ? 0 : node->height;
}

static b2TreeNode b2MakeInternalNode( const b2TreeNode* nodes, int pair )
{
	const b2TreeNode* c1 = nodes + pair;
	const b2TreeNode* c2 = nodes + pair + 1;
	b2TreeNode node = {};
	node.aabb = b2AABB_Union( c1->aabb, c2->aabb );
	node.flagIndex = (uint32_t)pair | ( ( c1->flagIndex | c2->flagIndex ) & B2_MOVED_NODE );
	node.height = 1 + b2MaxInt( b2GetNodeHeight( c1 ), b2GetNodeHeight( c2 ) );
	return node;
}

static b2TreeNode b2MakeLeafNode( b2AABB aabb, int proxyId, uint64_t userData, bool moved )
{
	b2TreeNode node = {};
	node.aabb = aabb;
	node.flagIndex = (uint32_t)proxyId | B2_LEAF_NODE | ( moved ? B2_MOVED_NODE : 0 );
	node.shapeIndex = (int32_t)userData;
	return node;
}

static void b2LinkChildren( b2DynamicTree* tree, int nodeIndex )
{
	const b2TreeNode* node = tree->nodes + nodeIndex;
	if ( b2IsLeaf( node ) )
	{
		tree->proxies[b2GetProxyId( node )].node = nodeIndex;
	}
	else
	{
		int pair = b2GetLeftChild( node );
		tree->parents[pair] = nodeIndex;
		tree->parents[pair + 1] = nodeIndex;
	}
}

static bool b2IsNodeOrdered( const b2TreeNode* nodes, int nodeIndex )
{
	const b2TreeNode* node = nodes + nodeIndex;
	return b2IsLeaf( node ) || nodeIndex < b2GetLeftChild( node );
}

static int b2FindBestSibling( const b2DynamicTree* tree, b2AABB boxD )
{
	b2Vec2 centerD = b2AABB_Center( boxD );
	float areaD = b2TreePerimeter( boxD );
	const b2TreeNode* nodes = tree->nodes;
	int nodeIndex = B2_ROOT_NODE;
	if ( b2IsLeaf( nodes + nodeIndex ) )
	{
		return nodeIndex;
	}

	b2AABB rootBox = nodes[nodeIndex].aabb;
	float areaBase = b2TreePerimeter( rootBox );
	float directCost = b2TreePerimeter( b2AABB_Union( rootBox, boxD ) );
	float inheritedCost = 0.0f;
	int bestSibling = nodeIndex;
	float bestCost = directCost;
	for ( ;; )
	{
		int child1 = b2GetLeftChild( nodes + nodeIndex );
		int child2 = child1 + 1;
		float cost = directCost + inheritedCost;
		if ( cost < bestCost )
		{
			bestSibling = nodeIndex;
			bestCost = cost;
		}

		inheritedCost += directCost - areaBase;
		bool leaf1 = b2IsLeaf( nodes + child1 );
		bool leaf2 = b2IsLeaf( nodes + child2 );
		float lowerCost1 = FLT_MAX;
		b2AABB box1 = nodes[child1].aabb;
		float directCost1 = b2TreePerimeter( b2AABB_Union( box1, boxD ) );
		float area1 = 0.0f;
		if ( leaf1 )
		{
			float cost1 = directCost1 + inheritedCost;
			if ( cost1 < bestCost )
			{
				bestSibling = child1;
				bestCost = cost1;
			}
		}
		else
		{
			area1 = b2TreePerimeter( box1 );
			lowerCost1 = inheritedCost + directCost1 + b2MinFloat( areaD - area1, 0.0f );
		}

		float lowerCost2 = FLT_MAX;
		b2AABB box2 = nodes[child2].aabb;
		float directCost2 = b2TreePerimeter( b2AABB_Union( box2, boxD ) );
		float area2 = 0.0f;
		if ( leaf2 )
		{
			float cost2 = directCost2 + inheritedCost;
			if ( cost2 < bestCost )
			{
				bestSibling = child2;
				bestCost = cost2;
			}
		}
		else
		{
			area2 = b2TreePerimeter( box2 );
			lowerCost2 = inheritedCost + directCost2 + b2MinFloat( areaD - area2, 0.0f );
		}

		if ( leaf1 && leaf2 )
		{
			break;
		}

		if ( bestCost <= lowerCost1 && bestCost <= lowerCost2 )
		{
			break;
		}

		if ( lowerCost1 == lowerCost2 && leaf1 == false )
		{
			b2Vec2 d1 = b2Sub( b2AABB_Center( box1 ), centerD );
			b2Vec2 d2 = b2Sub( b2AABB_Center( box2 ), centerD );
			lowerCost1 = b2LengthSquared( d1 );
			lowerCost2 = b2LengthSquared( d2 );
		}

		if ( lowerCost1 < lowerCost2 && leaf1 == false )
		{
			nodeIndex = child1;
			areaBase = area1;
			directCost = directCost1;
		}
		else
		{
			nodeIndex = child2;
			areaBase = area2;
			directCost = directCost2;
		}
	}

	return bestSibling;
}

static void b2SwapNodes( b2DynamicTree* tree, int iDown, int iUp )
{
	b2TreeNode* nodes = tree->nodes;
	B2_SWAP( nodes[iDown], nodes[iUp] );
	b2LinkChildren( tree, iDown );
	b2LinkChildren( tree, iUp );
	if ( b2IsNodeOrdered( nodes, iDown ) == false || b2IsNodeOrdered( nodes, iUp ) == false )
	{
		tree->dfsOrdered = false;
	}

	int iC = iDown ^ 1;
	nodes[iC] = b2MakeInternalNode( nodes, b2GetLeftChild( nodes + iC ) );
}

static void b2RotateNodes( b2DynamicTree* tree, int iA )
{
	b2TreeNode* nodes = tree->nodes;
	const b2TreeNode* A = nodes + iA;
	int iB = b2GetLeftChild( A );
	int iC = iB + 1;
	const b2TreeNode* B = nodes + iB;
	const b2TreeNode* C = nodes + iC;
	bool leafB = b2IsLeaf( B );
	bool leafC = b2IsLeaf( C );
	if ( leafB && leafC )
	{
		return;
	}

	int bestDown = B2_NULL_INDEX;
	int bestUp = B2_NULL_INDEX;
	float bestDelta = 0.0f;
	if ( leafC == false )
	{
		int iF = b2GetLeftChild( C );
		int iG = iF + 1;
		float areaC = b2TreePerimeter( C->aabb );
		float deltaBF = b2TreePerimeter( b2AABB_Union( B->aabb, nodes[iG].aabb ) ) - areaC;
		if ( deltaBF < bestDelta )
		{
			bestDown = iB;
			bestUp = iF;
			bestDelta = deltaBF;
		}

		float deltaBG = b2TreePerimeter( b2AABB_Union( B->aabb, nodes[iF].aabb ) ) - areaC;
		if ( deltaBG < bestDelta )
		{
			bestDown = iB;
			bestUp = iG;
			bestDelta = deltaBG;
		}
	}

	if ( leafB == false )
	{
		int iD = b2GetLeftChild( B );
		int iE = iD + 1;
		float areaB = b2TreePerimeter( B->aabb );
		float deltaCD = b2TreePerimeter( b2AABB_Union( C->aabb, nodes[iE].aabb ) ) - areaB;
		if ( deltaCD < bestDelta )
		{
			bestDown = iC;
			bestUp = iD;
			bestDelta = deltaCD;
		}

		float deltaCE = b2TreePerimeter( b2AABB_Union( C->aabb, nodes[iD].aabb ) ) - areaB;
		if ( deltaCE < bestDelta )
		{
			bestDown = iC;
			bestUp = iE;
			bestDelta = deltaCE;
		}
	}

	if ( bestDown != B2_NULL_INDEX )
	{
		b2SwapNodes( tree, bestDown, bestUp );
	}
}

static void b2InsertLeaf( b2DynamicTree* tree, b2AABB aabb, int proxyId, bool moved, bool shouldRotate )
{
	b2TreeProxy* proxy = tree->proxies + proxyId;
	b2TreeNode leaf = b2MakeLeafNode( aabb, proxyId, proxy->userData, moved );
	if ( b2IsEmptyNode( tree->nodes + B2_ROOT_NODE ) )
	{
		tree->nodes[B2_ROOT_NODE] = leaf;
		proxy->node = B2_ROOT_NODE;
		return;
	}

	int sibling = b2FindBestSibling( tree, aabb );
	int pair = b2AllocateSiblingPair( tree );
	b2TreeNode* nodes = tree->nodes;
	int32_t* parents = tree->parents;
	nodes[pair] = nodes[sibling];
	nodes[pair + 1] = leaf;
	parents[pair] = sibling;
	parents[pair + 1] = sibling;
	b2LinkChildren( tree, pair );
	proxy->node = pair + 1;
	nodes[sibling] = b2MakeInternalNode( nodes, pair );
	if ( b2IsNodeOrdered( nodes, sibling ) == false || b2IsNodeOrdered( nodes, pair ) == false )
	{
		tree->dfsOrdered = false;
	}

	int index = sibling;
	while ( index != B2_NULL_INDEX )
	{
		if ( shouldRotate )
		{
			b2RotateNodes( tree, index );
		}

		nodes[index] = b2MakeInternalNode( nodes, b2GetLeftChild( nodes + index ) );
		index = parents[index];
	}
}

int b2CreateTreeProxy( b2DynamicTree* tree, b2AABB aabb, uint64_t categoryBits, uint64_t userData )
{
	B2_VALIDATE( b2IsValidAABB( aabb ) );
	int proxyId = b2AllocateProxy( tree );
	b2TreeProxy* proxy = tree->proxies + proxyId;
	proxy->categoryBits = categoryBits;
	proxy->userData = userData;
	b2InsertLeaf( tree, aabb, proxyId, false, true );
	return proxyId;
}

int b2DynamicTree_GetProxyCount( const b2DynamicTree* tree )
{
	return tree->proxyCount;
}

uint64_t b2DynamicTree_GetUserData( const b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	return tree->proxies[proxyId].userData;
}

b2AABB b2DynamicTree_GetProxyBounds( const b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	int nodeIndex = tree->proxies[proxyId].node;
	B2_ASSERT( 0 <= nodeIndex && nodeIndex < tree->nodeEnd );
	return tree->nodes[nodeIndex].aabb;
}

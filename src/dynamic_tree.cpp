// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "dynamic_tree.h"

#include "core.h"

#include "box2d/constants.h"

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

static void b2FreeProxy( b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	B2_ASSERT( 0 < tree->proxyCount );
	tree->proxies[proxyId].node = B2_NULL_INDEX;
	tree->proxies[proxyId].next = tree->proxyFreeList;
	tree->proxyFreeList = proxyId;
	--tree->proxyCount;
}

static void b2FreePair( b2DynamicTree* tree, int pair )
{
	B2_ASSERT( ( pair & 1 ) == 0 && 2 <= pair && pair < tree->nodeEnd );
	tree->nodes[pair] = b2MakeEmptyNode();
	tree->nodes[pair + 1] = b2MakeEmptyNode();
	tree->parents[pair] = tree->pairFreeList;
	tree->parents[pair + 1] = B2_NULL_INDEX;
	tree->pairFreeList = pair;
}

static void b2RemoveLeaf( b2DynamicTree* tree, int proxyId )
{
	b2TreeNode* nodes = tree->nodes;
	int32_t* parents = tree->parents;
	int leaf = tree->proxies[proxyId].node;
	B2_ASSERT( 0 <= leaf && leaf < tree->nodeEnd );
	B2_ASSERT( b2IsLeaf( nodes + leaf ) && b2GetProxyId( nodes + leaf ) == proxyId );
	if ( leaf == B2_ROOT_NODE )
	{
		nodes[B2_ROOT_NODE] = b2MakeEmptyNode();
		return;
	}

	int parent = parents[leaf];
	nodes[parent] = nodes[leaf ^ 1];
	b2LinkChildren( tree, parent );
	b2FreePair( tree, leaf & ~1 );
	int index = parents[parent];
	while ( index != B2_NULL_INDEX )
	{
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

void b2DestroyTreeProxy( b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	b2RemoveLeaf( tree, proxyId );
	b2FreeProxy( tree, proxyId );
}

static bool b2EnlargeAABB( b2AABB* a, b2AABB b )
{
	bool changed = false;
	if ( b.lowerBound.x < a->lowerBound.x )
	{
		a->lowerBound.x = b.lowerBound.x;
		changed = true;
	}
	if ( b.lowerBound.y < a->lowerBound.y )
	{
		a->lowerBound.y = b.lowerBound.y;
		changed = true;
	}
	if ( a->upperBound.x < b.upperBound.x )
	{
		a->upperBound.x = b.upperBound.x;
		changed = true;
	}
	if ( a->upperBound.y < b.upperBound.y )
	{
		a->upperBound.y = b.upperBound.y;
		changed = true;
	}
	return changed;
}

void b2DynamicTree_MoveProxy( b2DynamicTree* tree, int proxyId, b2AABB aabb )
{
	B2_VALIDATE( b2IsValidAABB( aabb ) );
	B2_VALIDATE( aabb.upperBound.x - aabb.lowerBound.x < B2_HUGE );
	B2_VALIDATE( aabb.upperBound.y - aabb.lowerBound.y < B2_HUGE );
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	b2RemoveLeaf( tree, proxyId );
	b2InsertLeaf( tree, aabb, proxyId, false, false );
}

void b2DynamicTree_EnlargeProxy( b2DynamicTree* tree, int proxyId, b2AABB aabb )
{
	B2_VALIDATE( b2IsValidAABB( aabb ) );
	B2_VALIDATE( aabb.upperBound.x - aabb.lowerBound.x < B2_HUGE );
	B2_VALIDATE( aabb.upperBound.y - aabb.lowerBound.y < B2_HUGE );
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	b2TreeNode* nodes = tree->nodes;
	const int32_t* parents = tree->parents;
	int index = tree->proxies[proxyId].node;
	b2TreeNode* node = nodes + index;
	B2_ASSERT( b2IsLeaf( node ) );
	B2_VALIDATE( b2AABB_Contains( node->aabb, aabb ) == false );
	node->aabb = aabb;
	node->flagIndex |= B2_MOVED_NODE;
	index = parents[index];
	while ( index != B2_NULL_INDEX )
	{
		node = nodes + index;
		bool changed = b2EnlargeAABB( &node->aabb, aabb );
		node->flagIndex |= B2_MOVED_NODE;
		index = parents[index];
		if ( changed == false )
		{
			break;
		}
	}

	while ( index != B2_NULL_INDEX )
	{
		node = nodes + index;
		if ( node->flagIndex & B2_MOVED_NODE )
		{
			break;
		}
		node->flagIndex |= B2_MOVED_NODE;
		index = parents[index];
	}
}

void b2DynamicTree_SetCategoryBits( b2DynamicTree* tree, int proxyId, uint64_t categoryBits )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	tree->proxies[proxyId].categoryBits = categoryBits;
}

uint64_t b2DynamicTree_GetCategoryBits( b2DynamicTree* tree, int proxyId )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	return tree->proxies[proxyId].categoryBits;
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

static int b2PartitionMid( int* indices, b2Vec2* centers, int count )
{
	if ( count <= 2 )
	{
		return count / 2;
	}

	b2Vec2 lowerBound = centers[0];
	b2Vec2 upperBound = centers[0];
	for ( int i = 1; i < count; ++i )
	{
		lowerBound = b2Min( lowerBound, centers[i] );
		upperBound = b2Max( upperBound, centers[i] );
	}

	b2Vec2 d = b2Sub( upperBound, lowerBound );
	b2Vec2 c = { 0.5f * ( lowerBound.x + upperBound.x ), 0.5f * ( lowerBound.y + upperBound.y ) };
	int i1 = 0;
	int i2 = count;
	if ( d.x > d.y )
	{
		float pivot = c.x;
		while ( i1 < i2 )
		{
			while ( i1 < i2 && centers[i1].x < pivot )
			{
				i1 += 1;
			}
			while ( i1 < i2 && centers[i2 - 1].x >= pivot )
			{
				i2 -= 1;
			}
			if ( i1 < i2 )
			{
				int temp = indices[i1];
				indices[i1] = indices[i2 - 1];
				indices[i2 - 1] = temp;
				b2Vec2 tempCenter = centers[i1];
				centers[i1] = centers[i2 - 1];
				centers[i2 - 1] = tempCenter;
				i1 += 1;
				i2 -= 1;
			}
		}
	}
	else
	{
		float pivot = c.y;
		while ( i1 < i2 )
		{
			while ( i1 < i2 && centers[i1].y < pivot )
			{
				i1 += 1;
			}
			while ( i1 < i2 && centers[i2 - 1].y >= pivot )
			{
				i2 -= 1;
			}
			if ( i1 < i2 )
			{
				int temp = indices[i1];
				indices[i1] = indices[i2 - 1];
				indices[i2 - 1] = temp;
				b2Vec2 tempCenter = centers[i1];
				centers[i1] = centers[i2 - 1];
				centers[i2 - 1] = tempCenter;
				i1 += 1;
				i2 -= 1;
			}
		}
	}

	if ( i1 > 0 && i1 < count )
	{
		return i1;
	}
	return count / 2;
}

typedef struct b2RebuildItem
{
	int nodeIndex;
	int pair;
	int childCount;
	int startIndex;
	int splitIndex;
	int endIndex;
} b2RebuildItem;

typedef struct b2CopyItem
{
	int oldPair;
	int newIndex;
} b2CopyItem;

static int b2BumpPair( b2DynamicTree* tree, int parent )
{
	int pair = tree->nodeEnd;
	B2_ASSERT( pair + 2 <= tree->nodeCapacity );
	tree->nodeEnd += 2;
	tree->parents[pair] = parent;
	tree->parents[pair + 1] = parent;
	return pair;
}

static void b2SetLeftChild( b2TreeNode* node, int pair )
{
	node->flagIndex = ( node->flagIndex & ~B2_NODE_INDEX_MASK ) | (uint32_t)pair;
}

static void b2CopySubtree( b2DynamicTree* tree, b2TreeNode node, int newIndex )
{
	const b2TreeNode* oldNodes = tree->nodes;
	b2TreeNode* newNodes = tree->swapNodes;
	b2TreeProxy* proxies = tree->proxies;
	if ( b2IsLeaf( &node ) )
	{
		newNodes[newIndex] = node;
		proxies[b2GetProxyId( &node )].node = newIndex;
		return;
	}

	b2CopyItem stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	int oldPair = b2GetLeftChild( &node );
	int newPair = b2BumpPair( tree, newIndex );
	b2SetLeftChild( &node, newPair );
	newNodes[newIndex] = node;
	for ( ;; )
	{
		b2TreeNode* pair = newNodes + newPair;
		pair[0] = oldNodes[oldPair];
		pair[1] = oldNodes[oldPair + 1];
		if ( b2IsLeaf( pair + 1 ) )
		{
			proxies[b2GetProxyId( pair + 1 )].node = newPair + 1;
		}
		else
		{
			B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
			stack[stackCount++] = b2CopyItem{ b2GetLeftChild( pair + 1 ), newPair + 1 };
		}

		if ( b2IsLeaf( pair ) == false )
		{
			int leftIndex = newPair;
			oldPair = b2GetLeftChild( pair );
			newPair = b2BumpPair( tree, leftIndex );
			b2SetLeftChild( newNodes + leftIndex, newPair );
			continue;
		}

		proxies[b2GetProxyId( pair )].node = newPair;
		if ( stackCount == 0 )
		{
			break;
		}

		b2CopyItem item = stack[--stackCount];
		oldPair = item.oldPair;
		newPair = b2BumpPair( tree, item.newIndex );
		b2SetLeftChild( newNodes + item.newIndex, newPair );
	}
}

static void b2PlaceLeaf( b2DynamicTree* tree, b2TreeNode leaf, int newIndex )
{
	if ( b2IsLeaf( &leaf ) )
	{
		tree->swapNodes[newIndex] = leaf;
		tree->proxies[b2GetProxyId( &leaf )].node = newIndex;
	}
	else
	{
		b2CopySubtree( tree, leaf, newIndex );
	}
}

static void b2BuildTree( b2DynamicTree* tree, int leafCount )
{
	b2TreeNode* nodes = tree->swapNodes;
	const b2TreeNode* leaves = tree->leafNodes;
	int* leafIndices = tree->leafIndices;
	b2Vec2* leafCenters = tree->leafCenters;
	tree->nodeEnd = 2;
	nodes[B2_ROOT_NODE + 1] = b2MakeEmptyNode();
	tree->parents[B2_ROOT_NODE] = B2_NULL_INDEX;
	tree->parents[B2_ROOT_NODE + 1] = B2_NULL_INDEX;
	if ( leafCount == 1 )
	{
		b2PlaceLeaf( tree, leaves[leafIndices[0]], B2_ROOT_NODE );
		return;
	}

	b2RebuildItem stack[B2_TREE_STACK_SIZE];
	int top = 0;
	stack[0].nodeIndex = B2_ROOT_NODE;
	stack[0].pair = b2BumpPair( tree, B2_ROOT_NODE );
	stack[0].childCount = -1;
	stack[0].startIndex = 0;
	stack[0].endIndex = leafCount;
	stack[0].splitIndex = b2PartitionMid( leafIndices, leafCenters, leafCount );
	while ( true )
	{
		b2RebuildItem* item = stack + top;
		item->childCount += 1;
		if ( item->childCount == 2 )
		{
			nodes[item->nodeIndex] = b2MakeInternalNode( nodes, item->pair );
			if ( top == 0 )
			{
				break;
			}
			top -= 1;
			continue;
		}

		int slot = item->childCount;
		int startIndex = slot == 0 ? item->startIndex : item->splitIndex;
		int endIndex = slot == 0 ? item->splitIndex : item->endIndex;
		int count = endIndex - startIndex;
		int nodeIndex = item->pair + slot;
		if ( count == 1 )
		{
			b2PlaceLeaf( tree, leaves[leafIndices[startIndex]], nodeIndex );
			continue;
		}

		B2_ASSERT( top < B2_TREE_STACK_SIZE - 1 );
		top += 1;
		b2RebuildItem* newItem = stack + top;
		newItem->nodeIndex = nodeIndex;
		newItem->pair = b2BumpPair( tree, nodeIndex );
		newItem->childCount = -1;
		newItem->startIndex = startIndex;
		newItem->endIndex = endIndex;
		newItem->splitIndex = startIndex + b2PartitionMid( leafIndices + startIndex, leafCenters + startIndex, count );
	}
}

#if B2_ENABLE_VALIDATION
static int b2ValidateSubtree( const b2DynamicTree* tree, int nodeIndex, int* leafCount )
{
	const b2TreeNode* node = tree->nodes + nodeIndex;
	if ( b2IsLeaf( node ) )
	{
		int proxyId = b2GetProxyId( node );
		B2_ASSERT( tree->proxies[proxyId].node == nodeIndex );
		*leafCount += 1;
		return 0;
	}

	int pair = b2GetLeftChild( node );
	B2_ASSERT( tree->parents[pair] == nodeIndex && tree->parents[pair + 1] == nodeIndex );
	B2_ASSERT( b2AABB_Contains( node->aabb, tree->nodes[pair].aabb ) );
	B2_ASSERT( b2AABB_Contains( node->aabb, tree->nodes[pair + 1].aabb ) );
	int height1 = b2ValidateSubtree( tree, pair, leafCount );
	int height2 = b2ValidateSubtree( tree, pair + 1, leafCount );
	return 1 + b2MaxInt( height1, height2 );
}
#endif

void b2DynamicTree_Validate( const b2DynamicTree* tree )
{
#if B2_ENABLE_VALIDATION
	if ( tree->proxyCount == 0 )
	{
		B2_ASSERT( b2IsEmptyNode( tree->nodes + B2_ROOT_NODE ) );
		return;
	}

	int leafCount = 0;
	b2ValidateSubtree( tree, B2_ROOT_NODE, &leafCount );
	B2_ASSERT( leafCount == tree->proxyCount );
#else
	(void)tree;
#endif
}

static void b2DynamicTree_ValidateNoMoved( const b2DynamicTree* tree )
{
#if B2_ENABLE_VALIDATION
	for ( int i = 0; i < tree->nodeEnd; ++i )
	{
		B2_ASSERT( b2IsNodeMoved( tree->nodes + i ) == false );
	}
#else
	(void)tree;
#endif
}

int b2DynamicTree_Rebuild( b2DynamicTree* tree, bool fullBuild )
{
	int proxyCount = tree->proxyCount;
	if ( proxyCount == 0 )
	{
		return 0;
	}

	const b2TreeNode* nodes = tree->nodes;
	const b2TreeNode* root = nodes + B2_ROOT_NODE;
	if ( fullBuild == false && b2IsNodeMoved( root ) == false && tree->dfsOrdered )
	{
		return 0;
	}

	if ( tree->swapNodes == NULL )
	{
		tree->swapNodes = static_cast<b2TreeNode*>( b2AllocZero( tree->nodeCapacity * sizeof( b2TreeNode ) ) );
	}

	if ( proxyCount > tree->rebuildCapacity )
	{
		int oldCapacity = tree->rebuildCapacity;
		int newCapacity = proxyCount + proxyCount / 2;
		b2Free( tree->leafIndices, oldCapacity * sizeof( int32_t ) );
		tree->leafIndices = static_cast<int32_t*>( b2Alloc( newCapacity * sizeof( int32_t ) ) );
		b2Free( tree->leafNodes, oldCapacity * sizeof( b2TreeNode ) );
		tree->leafNodes = static_cast<b2TreeNode*>( b2Alloc( newCapacity * sizeof( b2TreeNode ) ) );
		b2Free( tree->leafCenters, oldCapacity * sizeof( b2Vec2 ) );
		tree->leafCenters = static_cast<b2Vec2*>( b2Alloc( newCapacity * sizeof( b2Vec2 ) ) );
		tree->rebuildCapacity = newCapacity;
	}

	int* leafIndices = tree->leafIndices;
	b2TreeNode* leaves = tree->leafNodes;
	b2Vec2* leafCenters = tree->leafCenters;
	int leafCount = 0;
	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	if ( b2IsLeaf( root ) == false && ( fullBuild || b2IsNodeMoved( root ) ) )
	{
		stack[stackCount++] = b2GetLeftChild( root );
	}
	else
	{
		b2TreeNode node = *root;
		node.flagIndex &= ~B2_MOVED_NODE;
		leafIndices[0] = 0;
		leaves[0] = node;
		leafCenters[0] = b2AABB_Center( node.aabb );
		leafCount = 1;
	}

	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		for ( int i = 0; i < 2; ++i )
		{
			b2TreeNode node = nodes[pair + i];
			if ( b2IsLeaf( &node ) == false && ( fullBuild || b2IsNodeMoved( &node ) ) )
			{
				B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
				stack[stackCount++] = b2GetLeftChild( &node );
				continue;
			}

			node.flagIndex &= ~B2_MOVED_NODE;
			leafIndices[leafCount] = leafCount;
			leaves[leafCount] = node;
			leafCenters[leafCount] = b2AABB_Center( node.aabb );
			leafCount += 1;
		}
	}

	b2BuildTree( tree, leafCount );
	B2_SWAP( tree->nodes, tree->swapNodes );
	tree->pairFreeList = B2_NULL_INDEX;
	tree->dfsOrdered = true;
	b2DynamicTree_Validate( tree );
	b2DynamicTree_ValidateNoMoved( tree );
	return leafCount;
}

void b2DynamicTree_MarkProxyMoved( b2DynamicTree* tree, int proxyId, b2AABB aabb )
{
	B2_ASSERT( 0 <= proxyId && proxyId < tree->proxyCapacity );
	b2TreeNode* nodes = tree->nodes;
	const int32_t* parents = tree->parents;
	int index = tree->proxies[proxyId].node;
	b2TreeNode* node = nodes + index;
	B2_ASSERT( b2IsLeaf( node ) );
	B2_VALIDATE( b2AABB_Contains( node->aabb, aabb ) == false );
	node->aabb = aabb;
	node->flagIndex |= B2_MOVED_NODE;
	index = parents[index];
	while ( index != B2_NULL_INDEX )
	{
		nodes[index].flagIndex |= B2_MOVED_NODE;
		index = parents[index];
	}
}

int b2DynamicTree_GatherMovedProxies( const b2DynamicTree* tree, int* proxyIds )
{
	const b2TreeNode* nodes = tree->nodes;
	const b2TreeNode* root = nodes + B2_ROOT_NODE;
	if ( b2IsNodeMoved( root ) == false )
	{
		return 0;
	}
	if ( b2IsLeaf( root ) )
	{
		proxyIds[0] = b2GetProxyId( root );
		return 1;
	}

	int count = 0;
	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b2GetLeftChild( root );
	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		for ( int i = 0; i < 2; ++i )
		{
			const b2TreeNode* node = nodes + pair + i;
			if ( b2IsNodeMoved( node ) == false )
			{
				continue;
			}
			if ( b2IsLeaf( node ) )
			{
				proxyIds[count++] = b2GetProxyId( node );
			}
			else if ( stackCount < B2_TREE_STACK_SIZE )
			{
				stack[stackCount++] = b2GetLeftChild( node );
			}
		}
	}
	return count;
}

static void b2RefitSubtree( b2TreeNode* nodes, int rootIndex )
{
	b2TreeNode* root = nodes + rootIndex;
	if ( b2IsLeaf( root ) || b2IsNodeMoved( root ) == false )
	{
		return;
	}

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = ~rootIndex;
	stack[stackCount++] = b2GetLeftChild( root );
	stack[stackCount++] = b2GetLeftChild( root ) + 1;
	while ( stackCount > 0 )
	{
		int item = stack[--stackCount];
		if ( item < 0 )
		{
			b2TreeNode* node = nodes + ~item;
			int pair = b2GetLeftChild( node );
			node->aabb = b2AABB_Union( nodes[pair].aabb, nodes[pair + 1].aabb );
			continue;
		}

		b2TreeNode* node = nodes + item;
		if ( b2IsLeaf( node ) || b2IsNodeMoved( node ) == false )
		{
			continue;
		}
		if ( stackCount + 3 <= B2_TREE_STACK_SIZE )
		{
			int pair = b2GetLeftChild( node );
			stack[stackCount++] = ~item;
			stack[stackCount++] = pair;
			stack[stackCount++] = pair + 1;
		}
	}
}

void b2DynamicTree_Refit( b2DynamicTree* tree )
{
	if ( b2IsNodeMoved( tree->nodes + B2_ROOT_NODE ) == false )
	{
		return;
	}
	if ( tree->dfsOrdered == false )
	{
		b2RefitSubtree( tree->nodes, B2_ROOT_NODE );
		return;
	}

	b2TreeNode* nodes = tree->nodes;
	for ( int pair = tree->nodeEnd - 2; pair >= 0; pair -= 2 )
	{
		uint32_t flags1 = nodes[pair].flagIndex;
		uint32_t flags2 = nodes[pair + 1].flagIndex;
		if ( ( ( flags1 | flags2 ) & B2_MOVED_NODE ) == 0 )
		{
			continue;
		}
		if ( ( flags1 & ( B2_LEAF_NODE | B2_MOVED_NODE ) ) == B2_MOVED_NODE )
		{
			int children = (int)( flags1 & B2_NODE_INDEX_MASK );
			nodes[pair].aabb = b2AABB_Union( nodes[children].aabb, nodes[children + 1].aabb );
		}
		if ( ( flags2 & ( B2_LEAF_NODE | B2_MOVED_NODE ) ) == B2_MOVED_NODE )
		{
			int children = (int)( flags2 & B2_NODE_INDEX_MASK );
			nodes[pair + 1].aabb = b2AABB_Union( nodes[children].aabb, nodes[children + 1].aabb );
		}
	}
}

static int b2GetRootPair( const b2TreeNode* nodes )
{
	const b2TreeNode* root = nodes + B2_ROOT_NODE;
	return b2IsLeaf( root ) ? B2_ROOT_NODE : b2GetLeftChild( root );
}

static b2TreeStats b2QueryAABB( const b2DynamicTree* tree, b2AABB aabb, uint64_t maskBits, bool useMask,
								b2TreeQueryCallbackFcn* callback, void* context )
{
	b2TreeStats result = {};
	if ( tree->proxyCount == 0 )
	{
		return result;
	}

	const b2TreeNode* nodes = tree->nodes;
	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b2GetRootPair( nodes );
	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		result.nodeVisits += 1;
		for ( int i = 0; i < 2; ++i )
		{
			const b2TreeNode* node = nodes + pair + i;
			if ( b2IsEmptyNode( node ) || b2AABB_Overlaps( node->aabb, aabb ) == false )
			{
				continue;
			}
			if ( b2IsLeaf( node ) )
			{
				int proxyId = b2GetProxyId( node );
				const b2TreeProxy* proxy = tree->proxies + proxyId;
				if ( useMask && ( proxy->categoryBits & maskBits ) == 0 )
				{
					continue;
				}
				bool proceed = callback( proxyId, proxy->userData, context );
				result.leafVisits += 1;
				if ( proceed == false )
				{
					return result;
				}
			}
			else if ( stackCount < B2_TREE_STACK_SIZE - 1 )
			{
				stack[stackCount++] = b2GetLeftChild( node );
			}
		}
	}
	return result;
}

b2TreeStats b2DynamicTree_OverlapAABB( const b2DynamicTree* tree, b2AABB aabb, uint64_t maskBits, b2TreeQueryCallbackFcn* callback,
									   void* context )
{
	return b2QueryAABB( tree, aabb, maskBits, true, callback, context );
}

b2TreeStats b2DynamicTree_OverlapAABBAll( const b2DynamicTree* tree, b2AABB aabb, b2TreeQueryCallbackFcn* callback, void* context )
{
	return b2QueryAABB( tree, aabb, 0, false, callback, context );
}

b2TreeStats b2DynamicTree_CastRay( const b2DynamicTree* tree, const b2RayCastInput* input, uint64_t maskBits,
								   b2TreeRayCastCallbackFcn* callback, void* context )
{
	b2TreeStats result = {};
	if ( tree->proxyCount == 0 )
	{
		return result;
	}

	b2Vec2 p1 = input->origin;
	b2Vec2 d = input->translation;
	b2Vec2 r = b2Normalize( d );
	b2Vec2 v = b2CrossSV( 1.0f, r );
	b2Vec2 absV = b2Abs( v );
	float maxFraction = input->maxFraction;
	b2Vec2 p2 = b2MulAdd( p1, maxFraction, d );
	b2AABB segmentAABB = { b2Min( p1, p2 ), b2Max( p1, p2 ) };
	const b2TreeNode* nodes = tree->nodes;
	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b2GetRootPair( nodes );
	b2RayCastInput subInput = *input;
	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		result.nodeVisits += 1;
		const b2TreeNode* hit[2];
		bool isLeaf[2];
		int hitCount = 0;
		for ( int i = 0; i < 2; ++i )
		{
			const b2TreeNode* node = nodes + pair + i;
			if ( b2IsEmptyNode( node ) || b2AABB_Overlaps( node->aabb, segmentAABB ) == false )
			{
				continue;
			}

			b2Vec2 c = b2AABB_Center( node->aabb );
			b2Vec2 h = b2AABB_Extents( node->aabb );
			float term1 = b2AbsFloat( b2Dot( v, b2Sub( p1, c ) ) );
			float term2 = b2Dot( absV, h );
			if ( term2 < term1 )
			{
				continue;
			}

			isLeaf[hitCount] = b2IsLeaf( node );
			hit[hitCount] = node;
			hitCount += 1;
		}

		if ( hitCount == 2 && isLeaf[0] == false && isLeaf[1] == false )
		{
			float d1 = b2DistanceSquared( b2AABB_Center( hit[0]->aabb ), p1 );
			float d2 = b2DistanceSquared( b2AABB_Center( hit[1]->aabb ), p1 );
			if ( d1 < d2 )
			{
				B2_SWAP( hit[0], hit[1] );
			}
		}

		for ( int i = 0; i < hitCount; ++i )
		{
			if ( isLeaf[i] )
			{
				int proxyId = b2GetProxyId( hit[i] );
				const b2TreeProxy* proxy = tree->proxies + proxyId;
				if ( ( proxy->categoryBits & maskBits ) == 0 )
				{
					continue;
				}

				subInput.maxFraction = maxFraction;
				float value = callback( &subInput, proxyId, proxy->userData, context );
				result.leafVisits += 1;
				if ( value == 0.0f )
				{
					return result;
				}
				if ( 0.0f < value && value <= maxFraction )
				{
					maxFraction = value;
					p2 = b2MulAdd( p1, maxFraction, d );
					segmentAABB.lowerBound = b2Min( p1, p2 );
					segmentAABB.upperBound = b2Max( p1, p2 );
				}
			}
			else if ( stackCount < B2_TREE_STACK_SIZE - 1 )
			{
				stack[stackCount++] = b2GetLeftChild( hit[i] );
			}
		}
	}

	return result;
}

b2TreeStats b2DynamicTree_CastBox( const b2DynamicTree* tree, const b2BoxCastInput* input, uint64_t maskBits,
								   b2TreeBoxCastCallbackFcn* callback, void* context )
{
	b2TreeStats result = {};
	if ( tree->proxyCount == 0 )
	{
		return result;
	}

	b2AABB originAABB = input->box;
	b2Vec2 p1 = b2AABB_Center( originAABB );
	b2Vec2 extension = b2AABB_Extents( originAABB );
	b2Vec2 r = input->translation;
	b2Vec2 v = b2CrossSV( 1.0f, r );
	b2Vec2 absV = b2Abs( v );
	float maxFraction = input->maxFraction;
	b2Vec2 t = b2MulSV( maxFraction, input->translation );
	b2AABB totalAABB = {
		b2Min( originAABB.lowerBound, b2Add( originAABB.lowerBound, t ) ),
		b2Max( originAABB.upperBound, b2Add( originAABB.upperBound, t ) ),
	};
	const b2TreeNode* nodes = tree->nodes;
	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = b2GetRootPair( nodes );
	b2BoxCastInput subInput = *input;
	while ( stackCount > 0 )
	{
		int pair = stack[--stackCount];
		result.nodeVisits += 1;
		const b2TreeNode* hit[2];
		bool isLeaf[2];
		int hitCount = 0;
		for ( int i = 0; i < 2; ++i )
		{
			const b2TreeNode* node = nodes + pair + i;
			if ( b2IsEmptyNode( node ) || b2AABB_Overlaps( node->aabb, totalAABB ) == false )
			{
				continue;
			}

			b2Vec2 c = b2AABB_Center( node->aabb );
			b2Vec2 h = b2Add( b2AABB_Extents( node->aabb ), extension );
			if ( b2Dot( absV, h ) < b2AbsFloat( b2Dot( v, b2Sub( p1, c ) ) ) )
			{
				continue;
			}

			isLeaf[hitCount] = b2IsLeaf( node );
			hit[hitCount] = node;
			hitCount += 1;
		}

		if ( hitCount == 2 && isLeaf[0] == false && isLeaf[1] == false )
		{
			float d1 = b2DistanceSquared( b2AABB_Center( hit[0]->aabb ), p1 );
			float d2 = b2DistanceSquared( b2AABB_Center( hit[1]->aabb ), p1 );
			if ( d1 < d2 )
			{
				B2_SWAP( hit[0], hit[1] );
			}
		}

		for ( int i = 0; i < hitCount; ++i )
		{
			if ( isLeaf[i] )
			{
				int proxyId = b2GetProxyId( hit[i] );
				const b2TreeProxy* proxy = tree->proxies + proxyId;
				if ( ( proxy->categoryBits & maskBits ) == 0 )
				{
					continue;
				}

				subInput.maxFraction = maxFraction;
				float value = callback( &subInput, proxyId, proxy->userData, context );
				result.leafVisits += 1;
				if ( value == 0.0f )
				{
					return result;
				}
				if ( 0.0f < value && value < maxFraction )
				{
					maxFraction = value;
					t = b2MulSV( maxFraction, input->translation );
					totalAABB.lowerBound = b2Min( originAABB.lowerBound, b2Add( originAABB.lowerBound, t ) );
					totalAABB.upperBound = b2Max( originAABB.upperBound, b2Add( originAABB.upperBound, t ) );
				}
			}
			else if ( stackCount < B2_TREE_STACK_SIZE - 1 )
			{
				stack[stackCount++] = b2GetLeftChild( hit[i] );
			}
		}
	}

	return result;
}

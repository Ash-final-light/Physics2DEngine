// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "broad_phase.h"
#include "core.h"
#include "world.h"
#include "simd.h"
#include "contact.h"
#include <bit>
#include <cstring>

void b2CreateBroadPhase( b2BroadPhase* bp, const b2Capacity* capacity )
{
	static_assert( b2_bodyTypeCount == 3, "must be three body types" );

	bp->pairSet = b2CreateSet( b2MaxInt( 32, 2 * capacity->contactCount ) );

	int staticCapacity = b2MaxInt( 16, capacity->staticShapeCount );
	b2CreateDynamicTreeInternal( bp->trees + b2_staticBody, staticCapacity );

	int kinematicCapacity = 16;
	b2CreateDynamicTreeInternal( bp->trees + b2_kinematicBody, kinematicCapacity );

	int dynamicCapacity = b2MaxInt( 16, capacity->dynamicShapeCount );
	b2CreateDynamicTreeInternal( bp->trees + b2_dynamicBody, dynamicCapacity );

	bp->movedSiblings = NULL;
	bp->crossContext = NULL;
}

void b2DestroyBroadPhase( b2BroadPhase* bp )
{
	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DestroyDynamicTreeInternal( bp->trees + i );
	}

	b2DestroySet( &bp->pairSet );

	memset( bp, 0, sizeof( b2BroadPhase ) );
}


int b2BroadPhase_CreateProxy( b2BroadPhase* bp, b2BodyType proxyType, b2AABB aabb, uint64_t categoryBits, int shapeIndex,
							  bool forcePairCreation )
{
	B2_ASSERT( 0 <= proxyType && proxyType < b2_bodyTypeCount );

	bool mark = ( proxyType != b2_staticBody || forcePairCreation );

	int proxyId = b2CreateTreeProxyInternal( bp->trees + proxyType, aabb, categoryBits, shapeIndex, mark );
	int proxyKey = B2_PROXY_KEY( proxyId, proxyType );
	return proxyKey;
}

void b2BroadPhase_DestroyProxy( b2BroadPhase* bp, int proxyKey )
{
	b2BodyType proxyType = B2_PROXY_TYPE( proxyKey );
	int proxyId = B2_PROXY_ID( proxyKey );

	B2_ASSERT( 0 <= proxyType && proxyType < b2_bodyTypeCount );
	b2DestroyTreeProxy( bp->trees + proxyType, proxyId );
}


void b2BroadPhase_MoveProxy( b2BroadPhase* bp, int proxyKey, b2AABB aabb )
{
    b2BodyType type = B2_PROXY_TYPE( proxyKey );
    int proxyId = B2_PROXY_ID( proxyKey );
    b2DynamicTree_MoveProxyInternal( bp->trees+type, proxyId, aabb, true );
}


static int b2GatherMovedSiblings( const b2DynamicTree* tree, int* pairIndices )
{
	const b2TreeNode* nodes = tree->nodes;
	int nodeEnd = tree->nodeEnd;

	int count = 0;

	// 跳过根。
	for ( int pair = 2; pair < nodeEnd; pair += 2 )
	{
		// 任一兄弟已移动则压入。
		if ( ( nodes[pair].flagIndex | nodes[pair + 1].flagIndex ) & B2_MOVED_NODE )
		{
			pairIndices[count++] = pair;
		}
	}

	return count;
}

#define B2_CANDIDATE_BATCH 32

typedef struct b2Candidate
{
	int shapeIdA;
	int shapeIdB;
} b2CandidatePair;

typedef struct b2PairContext
{
	b2World* world;
	b2DynamicArray<uint64_t>* pairKeys;
	b2CandidatePair batch[B2_CANDIDATE_BATCH];
	int batchCount;
} b2PairContext;

typedef struct b2NodePair
{
	b2TreeNode a;
	b2TreeNode b;
} b2NodePair;

typedef struct b2IndexPair
{
	int a, b;
} b2IndexPair;


static inline int b2CTZ32( uint32_t value )
{
    return std::countr_zero( value );
}

static void b2FlushCandidatePairs( b2PairContext* context )
{
    for ( int i = 0; i < context->batchCount; ++i )
    {
        const b2CandidatePair* pair = context->batch+i;
        uint64_t key = (uint64_t)(uint32_t)pair->shapeIdA << 32 | (uint32_t)pair->shapeIdB;
        uint64_t hash = b2KeyHash( key );
        if ( b2ContainsHashedKey( &context->world->broadPhase.pairSet, key, hash ) )
        {
            continue;
        }
        const b2Shape* a = context->world->shapes.data+pair->shapeIdA;
        const b2Shape* b = context->world->shapes.data+pair->shapeIdB;
        if ( a->bodyId == b->bodyId || b2ShouldShapesCollide( a->filter, b->filter ) == false )
        {
            continue;
        }
        // This is the support pattern of the reference contact registry.
        // Edge-edge pairs have no manifold function.
        if ( b2CanCollide(a->type,b->type) == false )
        {
            continue;
        }
        b2Array_Push( *context->pairKeys, key );
    }
    context->batchCount = 0;
}


B2_FORCE_INLINE void b2AddCandidatePair( int shapeIdA, int shapeIdB, b2PairContext* context )
{
	// 按形状下标顺序。
	b2CandidatePair* candidate = context->batch + context->batchCount;
	candidate->shapeIdA = b2MinInt( shapeIdA, shapeIdB );
	candidate->shapeIdB = b2MaxInt( shapeIdA, shapeIdB );
	context->batchCount += 1;
	if ( context->batchCount == B2_CANDIDATE_BATCH )
	{
		b2FlushCandidatePairs( context );
	}
}

B2_FORCE_INLINE bool b2TestPair( const b2TreeNode* a, const b2TreeNode* b )
{
	if ( ( ( a->flagIndex | b->flagIndex ) & B2_MOVED_NODE ) == 0 )
	{
		return false;
	}

	return b2AABB_Overlaps( a->aabb, b->aabb );
}

static void b2CollideProxyAndSubtree( const b2TreeNode* proxy, const b2TreeNode* nodes, int pair, b2PairContext* context )
{
	uint32_t proxyMark = proxy->flagIndex & B2_MOVED_NODE;
	b2AABBV boxv = b2LoadAABBV( &proxy->aabb );
	int shapeId = proxy->shapeIndex;

	int stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;
	stack[stackCount++] = pair;

	while ( stackCount > 0 )
	{
		pair = stack[--stackCount];
		const b2TreeNode* children = nodes + pair;

		int movedMask = 3;
		if ( proxyMark == 0 )
		{
			movedMask = ( b2IsNodeMoved( children + 0 ) ? 1 : 0 ) | ( b2IsNodeMoved( children + 1 ) ? 2 : 0 );
		}

		int mask = b2OverlapNodePairMask( boxv, children ) & movedMask;
		while ( mask != 0 )
		{
			int i = (int)b2CTZ32( (uint32_t)mask );
			mask &= mask - 1;

			const b2TreeNode* node = children + i;
			if ( b2IsLeaf( node ) )
			{
				b2AddCandidatePair( shapeId, node->shapeIndex, context );
			}
			else
			{
				if ( stackCount < B2_TREE_STACK_SIZE )
				{
					stack[stackCount++] = b2GetLeftChild( node );
				}
				else
				{
					B2_ASSERT( stackCount < B2_TREE_STACK_SIZE );
				}
			}
		}
	}
}

B2_FORCE_INLINE void b2ExpandPair( const b2TreeNode* arrayA, const b2TreeNode* arrayB, const b2TreeNode* nodeA,
								   const b2TreeNode* nodeB, b2IndexPair* stack, int* stackCount, b2PairContext* context )
{
	bool leafA = b2IsLeaf( nodeA );
	bool leafB = b2IsLeaf( nodeB );
	if ( leafA && leafB )
	{
		b2AddCandidatePair( nodeA->shapeIndex, nodeB->shapeIndex, context );
	}
	else if ( leafA )
	{
		b2CollideProxyAndSubtree( nodeA, arrayB, b2GetLeftChild( nodeB ), context );
	}
	else if ( leafB )
	{
		b2CollideProxyAndSubtree( nodeB, arrayA, b2GetLeftChild( nodeA ), context );
	}
	else
	{
		if ( *stackCount < B2_TREE_STACK_SIZE )
		{
			stack[*stackCount] = b2IndexPair{ .a = b2GetLeftChild( nodeA ), .b = b2GetLeftChild( nodeB ) };
			*stackCount += 1;
		}
		else
		{
			B2_ASSERT( *stackCount < B2_TREE_STACK_SIZE );
		}
	}
}

static void b2CollideCrossPairs( const b2TreeNode* arrayA, const b2TreeNode* arrayB, const b2TreeNode* subtreeA,
								 const b2TreeNode* subtreeB, b2PairContext* context )
{
	b2IndexPair stack[B2_TREE_STACK_SIZE];
	int stackCount = 0;

	// 初始化栈。
	if ( b2TestPair( subtreeA, subtreeB ) )
	{
		b2ExpandPair( arrayA, arrayB, subtreeA, subtreeB, stack, &stackCount, context );
	}

	while ( stackCount > 0 )
	{
		b2IndexPair pair = stack[--stackCount];
		const b2TreeNode* pairA = arrayA + pair.a;
		const b2TreeNode* pairB = arrayB + pair.b;

		int movedA = ( b2IsNodeMoved( pairA + 0 ) ? 0x3 : 0 ) | ( b2IsNodeMoved( pairA + 1 ) ? 0xC : 0 );
		int movedB = ( b2IsNodeMoved( pairB + 0 ) ? 0x5 : 0 ) | ( b2IsNodeMoved( pairB + 1 ) ? 0xA : 0 );

		int mask = b2OverlapPairMask( pairA, pairB ) & ( movedA | movedB );
		while ( mask != 0 )
		{
			int lane = (int)b2CTZ32( (uint32_t)mask );
			mask &= mask - 1;
			b2ExpandPair( arrayA, arrayB, pairA + ( lane >> 1 ), pairB + ( lane & 1 ), stack, &stackCount, context );
		}
	}
}

#define B2_CROSS_SEED_COUNT 64

static int b2GatherCrossSeeds( const b2DynamicTree* treeA, const b2DynamicTree* treeB, b2NodePair* seeds )
{
	const b2TreeNode* nodesA = treeA->nodes;
	const b2TreeNode* nodesB = treeB->nodes;

	// 以两棵树的根作为一对开始广度优先搜索。空树有一个哨兵根，不会产生任何结果。
	b2NodePair queue[2 * B2_CROSS_SEED_COUNT];
	int mask = 2 * B2_CROSS_SEED_COUNT - 1;
	int head = 0;
	int tail = 0;

	const b2TreeNode* rootA = nodesA + B2_ROOT_NODE;
	const b2TreeNode* rootB = nodesB + B2_ROOT_NODE;
	if ( b2TestPair( rootA, rootB ) )
	{
		queue[tail & mask] = b2NodePair{ .a = *rootA, .b = *rootB };
		tail += 1;
	}

	int seedCount = 0;
	while ( head < tail && seedCount + ( tail - head ) + 3 < B2_CROSS_SEED_COUNT )
	{
		b2NodePair pair = queue[head & mask];
		head += 1;

		if ( b2IsLeaf( &pair.a ) || b2IsLeaf( &pair.b ) )
		{
			seeds[seedCount++] = pair;
			continue;
		}

		const b2TreeNode* a = nodesA + b2GetLeftChild( &pair.a );
		const b2TreeNode* b = nodesB + b2GetLeftChild( &pair.b );

		// 每个节点有两个子节点，共四种组合。
		for ( int i = 0; i < 2; ++i )
		{
			for ( int j = 0; j < 2; ++j )
			{
				if ( b2TestPair( a + i, b + j ) )
				{
					queue[tail & mask] = b2NodePair{ a[i], b[j] };
					tail += 1;
				}
			}
		}
	}

	while ( head < tail )
	{
		seeds[seedCount] = queue[head & mask];
		seedCount += 1;
		head += 1;
	}

	return seedCount;
}


void b2CollectCandidatePairs( b2World* world, b2DynamicArray<uint64_t>* pairKeys )
{
    b2Array_Clear( *pairKeys );
    b2BroadPhase* bp = &world->broadPhase;
    b2PairContext context = { .world = world, .pairKeys = pairKeys };
    const b2DynamicTree* dynamicTree = bp->trees+b2_dynamicBody;
    for ( int type = b2_staticBody; type <= b2_kinematicBody; ++type )
    {
        const b2DynamicTree* other = bp->trees+type;
        b2NodePair seeds[B2_CROSS_SEED_COUNT];
        int count = b2GatherCrossSeeds( dynamicTree, other, seeds );
        B2_ASSERT( count <= B2_CROSS_SEED_COUNT );
        for ( int i = 0; i < count; ++i )
        {
            b2CollideCrossPairs( dynamicTree->nodes, other->nodes, &seeds[i].a, &seeds[i].b, &context );
        }
    }
    int* siblings = static_cast<int*>( b2StackAlloc( &world->stack, dynamicTree->nodeEnd/2*sizeof(int), "moved pairs" ) );
    int count = b2GatherMovedSiblings( dynamicTree, siblings );
    for ( int i = 0; i < count; ++i )
    {
        int index = siblings[i];
        b2CollideCrossPairs( dynamicTree->nodes, dynamicTree->nodes, dynamicTree->nodes+index, dynamicTree->nodes+index+1, &context );
    }
    b2FlushCandidatePairs( &context );
    if ( pairKeys->count > 1 )
    {
        uint64_t* temporary = static_cast<uint64_t*>( b2StackAlloc( &world->stack, pairKeys->count*sizeof(uint64_t), "pair sort" ) );
        b2RadixSortKeys( pairKeys->data, temporary, pairKeys->count );
        b2StackFree( &world->stack, temporary );
    }
    b2StackFree( &world->stack, siblings );
    for ( int type = 0; type < b2_bodyTypeCount; ++type )
    {
        b2DynamicTree_ClearMoved( bp->trees+type );
    }
}


void b2RadixSortKeys( uint64_t* keys, uint64_t* tempKeys, int count )
{
    if ( count < 2 )
    {
        return;
    }
	int digitCounts[8][256] = {};
	for ( int i = 0; i < count; ++i )
	{
		uint64_t key = keys[i];
		for ( int digit = 0; digit < 8; ++digit )
		{
			digitCounts[digit][( key >> ( 8 * digit ) ) & 0xFF] += 1;
		}
	}

	uint64_t* source = keys;
	uint64_t* target = tempKeys;

	for ( int digit = 0; digit < 8; ++digit )
	{
		int shift = 8 * digit;
		int* offsets = digitCounts[digit];

		if ( offsets[( source[0] >> shift ) & 0xFF] == count )
		{
			// 此位上所有键值相同。形状对键常见这种情况。
			continue;
		}

		int sum = 0;
		for ( int i = 0; i < 256; ++i )
		{
			int digitCount = offsets[i];
			offsets[i] = sum;
			sum += digitCount;
		}

		for ( int i = 0; i < count; ++i )
		{
			uint64_t key = source[i];
			target[offsets[( key >> shift ) & 0xFF]++] = key;
		}

		// 交换以进行下一趟
		uint64_t* swap = source;
		source = target;
		target = swap;
	}

	if ( source != keys )
	{
		memcpy( keys, source, count * sizeof( uint64_t ) );
	}
}

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "snapshot.h"

#include "bitset.h"
#include "body.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "contact.h"
#include "container.h"
#include "core.h"
#include "dynamic_tree.h"
#include "id_pool.h"
#include "island.h"
#include "joint.h"
#include "sensor.h"
#include "shape.h"
#include "solver_set.h"
#include "table.h"
#include "world.h"

#include "box2d/collision.h"
#include "box2d/types.h"

#include <string.h>

// 快照映像魔数与版本
#define B2_SNAP_MAGIC 0x32534E42u // 'BNS2'
#define B2_SNAP_VERSION 13u

#define B2_SNAP_FLAG_VALIDATION 0x1u
#define B2_SNAP_FLAG_DOUBLE_PRECISION 0x2u

// 布局哈希覆盖所有以原始字节写入的结构和关键常量。
static uint32_t b2ComputeLayoutHash( void )
{
	uint32_t h = 2166136261u;
#define MIX( x )                                                                                   \
	h ^= (uint32_t)( x );                                                                          \
	h *= 16777619u;
	MIX( sizeof( b2Body ) )
	MIX( sizeof( b2BodySim ) )
	MIX( sizeof( b2BodyState ) )
	MIX( sizeof( b2Shape ) )
	MIX( sizeof( b2ChainShape ) )
	MIX( sizeof( b2Contact ) )
	MIX( sizeof( b2ContactSim ) )
	MIX( sizeof( b2Joint ) )
	MIX( sizeof( b2JointSim ) )
	MIX( sizeof( b2Island ) )
	MIX( sizeof( b2IslandSim ) )
	MIX( sizeof( b2ContactLink ) )
	MIX( sizeof( b2JointLink ) )
	MIX( sizeof( b2Sensor ) )
	MIX( sizeof( b2Visitor ) )
	MIX( sizeof( b2SolverSet ) )
	MIX( sizeof( b2GraphColor ) )
	MIX( sizeof( b2DynamicTree ) )
	MIX( sizeof( b2TreeNode ) )
	MIX( sizeof( b2TreeProxy ) )
	MIX( sizeof( b2SetItem ) )
	MIX( sizeof( b2IdPool ) )
	MIX( sizeof( b2SurfaceMaterial ) )
	MIX( B2_GRAPH_COLOR_COUNT )
	MIX( b2_bodyTypeCount )
	MIX( sizeof( void* ) )
#undef MIX
	return h;
}

typedef struct b2SnapHeader
{
	uint32_t magic;
	uint32_t version;
	uint32_t layoutHash;
	uint32_t flags;
} b2SnapHeader;

#if INTPTR_MAX == INT64_MAX
static_assert( sizeof( b2ChainShape ) == 32, "b2ChainShape layout changed; resync snapshot chain serialization" );
static_assert( sizeof( b2Sensor ) == 56, "b2Sensor layout changed; resync snapshot sensor serialization" );
static_assert( sizeof( b2Island ) == 64, "b2Island layout changed; resync snapshot island serialization" );
#endif

static void b2SnapW_I32( b2RecBuffer* buf, int v )
{
	int32_t w = (int32_t)v;
	b2RecBufAppend( buf, &w, 4 );
}

static void b2SnapW_U32( b2RecBuffer* buf, uint32_t v )
{
	b2RecBufAppend( buf, &v, 4 );
}

static void b2SnapW_Bytes( b2RecBuffer* buf, const void* src, int n )
{
	b2RecBufAppend( buf, src, n );
}

#define b2SerPodArray( buf, arr )                                                                  \
	do                                                                                             \
	{                                                                                              \
		b2SnapW_I32( buf, ( arr ).count );                                                         \
		if ( ( arr ).count > 0 )                                                                   \
		{                                                                                          \
			b2SnapW_Bytes( buf, ( arr ).data, ( arr ).count * (int)sizeof( *( arr ).data ) );       \
		}                                                                                          \
	}                                                                                              \
	while ( 0 )

#define b2SerSimArray( buf, arr, type )                                                            \
	do                                                                                             \
	{                                                                                              \
		b2SnapW_I32( buf, ( arr ).count );                                                         \
		for ( int slot = 0; slot < ( arr ).count; ++slot )                                         \
		{                                                                                          \
			type elem = ( arr ).data[slot];                                                        \
			elem.userData = NULL;                                                                  \
			b2SnapW_Bytes( buf, &elem, (int)sizeof( type ) );                                      \
		}                                                                                          \
	}                                                                                              \
	while ( 0 )

static void b2SerIdPool( b2RecBuffer* buf, const b2IdPool* pool )
{
	b2SnapW_I32( buf, pool->nextIndex );
	b2SerPodArray( buf, pool->freeArray );
}

static void b2SerBitSet( b2RecBuffer* buf, const b2BitSet* bs )
{
	b2SnapW_U32( buf, bs->blockCount );
	if ( bs->blockCount > 0 )
	{
		b2SnapW_Bytes( buf, bs->bits, (int)( bs->blockCount * sizeof( uint64_t ) ) );
	}
}

static void b2SerHashSet( b2RecBuffer* buf, const b2HashSet* hs )
{
	b2SnapW_U32( buf, hs->capacity );
	b2SnapW_U32( buf, hs->count );
	if ( hs->capacity > 0 )
	{
		b2SnapW_Bytes( buf, hs->items, (int)( hs->capacity * sizeof( b2SetItem ) ) );
	}
}

static void b2SerTree( b2RecBuffer* buf, const b2DynamicTree* tree )
{
	b2SnapW_I32( buf, tree->nodeEnd );
	b2SnapW_I32( buf, tree->nodeCapacity );
	b2SnapW_I32( buf, tree->pairFreeList );
	b2SnapW_I32( buf, tree->proxyCount );
	b2SnapW_I32( buf, tree->proxyCapacity );
	b2SnapW_I32( buf, tree->proxyFreeList );
	b2SnapW_I32( buf, tree->dfsOrdered ? 1 : 0 );

	b2SnapW_Bytes( buf, tree->nodes, tree->nodeEnd * (int)sizeof( b2TreeNode ) );
	b2SnapW_Bytes( buf, tree->parents, tree->nodeEnd * (int)sizeof( int32_t ) );
	b2SnapW_Bytes( buf, tree->proxies, tree->proxyCapacity * (int)sizeof( b2TreeProxy ) );
}

static void b2SerSolverSet( b2RecBuffer* buf, const b2SolverSet* set )
{
	b2SnapW_I32( buf, set->setIndex );
	b2SerPodArray( buf, set->bodySims );
	b2SerPodArray( buf, set->bodyStates );
	b2SerPodArray( buf, set->jointSims );
	b2SerPodArray( buf, set->contactSims );
	b2SerPodArray( buf, set->islandSims );
}

static void b2SerGraphColor( b2RecBuffer* buf, const b2GraphColor* color, bool isOverflow )
{
	if ( !isOverflow )
	{
		b2SerBitSet( buf, &color->bodySet );
	}
	b2SerPodArray( buf, color->contactSims );
	b2SerPodArray( buf, color->jointSims );
}

static void b2SerWorldConfig( b2RecBuffer* buf, const b2World* world )
{
	b2SnapW_Bytes( buf, &world->gravity, sizeof( b2Vec2 ) );
	b2SnapW_Bytes( buf, &world->hitEventThreshold, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->restitutionThreshold, sizeof( float ) );
	b2SnapW_I32( buf, world->restitutionIterations );
	b2SnapW_Bytes( buf, &world->maxLinearSpeed, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->contactSpeed, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->contactHertz, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->contactDampingRatio, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->contactRecycleDistance, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->stepIndex, sizeof( uint64_t ) );
	b2SnapW_I32( buf, world->splitIslandId );
	b2SnapW_Bytes( buf, &world->inv_h, sizeof( float ) );
	b2SnapW_Bytes( buf, &world->inv_dt, sizeof( float ) );
	b2SnapW_I32( buf, world->endEventArrayIndex );
	b2SnapW_Bytes( buf, &world->maxCapacity, sizeof( b2Capacity ) );

	uint8_t flags = 0;
	flags |= world->enableSleep ? 0x01u : 0u;
	flags |= world->enableWarmStarting ? 0x02u : 0u;
	flags |= world->enableContinuous ? 0x08u : 0u;
	flags |= world->enableRestitutionPropagation ? 0x10u : 0u;
	b2RecBufAppend( buf, &flags, 1 );
}

void b2SerializeWorld( b2World* world, b2RecBuffer* buf )
{
	b2SnapHeader hdr;
	hdr.magic = B2_SNAP_MAGIC;
	hdr.version = B2_SNAP_VERSION;
	hdr.layoutHash = b2ComputeLayoutHash();
	hdr.flags = B2_ENABLE_VALIDATION ? B2_SNAP_FLAG_VALIDATION : 0u;
	if ( b2IsDoublePrecision() )
	{
		hdr.flags |= B2_SNAP_FLAG_DOUBLE_PRECISION;
	}
	b2SnapW_Bytes( buf, &hdr, (int)sizeof( hdr ) );

	b2SerWorldConfig( buf, world );

	b2SerIdPool( buf, &world->bodyIdPool );
	b2SerIdPool( buf, &world->shapeIdPool );
	b2SerIdPool( buf, &world->chainIdPool );
	b2SerIdPool( buf, &world->contactIdPool );
	b2SerIdPool( buf, &world->jointIdPool );
	b2SerIdPool( buf, &world->islandIdPool );
	b2SerIdPool( buf, &world->solverSetIdPool );

	int setCount = world->solverSets.count;
	b2SnapW_I32( buf, setCount );
	for ( int i = 0; i < setCount; ++i )
	{
		b2SerSolverSet( buf, world->solverSets.data + i );
	}

	b2SerSimArray( buf, world->bodies, b2Body );
	b2SerSimArray( buf, world->shapes, b2Shape );
	b2SerPodArray( buf, world->fatAABBs );
	b2SerPodArray( buf, world->contacts );
	b2SerSimArray( buf, world->joints, b2Joint );

	int chainCount = world->chainShapes.count;
	b2SnapW_I32( buf, chainCount );
	for ( int i = 0; i < chainCount; ++i )
	{
		b2ChainShape* chain = world->chainShapes.data + i;
		b2SnapW_I32( buf, chain->id );
		b2SnapW_I32( buf, chain->bodyId );
		b2SnapW_I32( buf, chain->nextChainId );
		b2SnapW_I32( buf, chain->segmentCount );
		b2SnapW_Bytes( buf, &chain->generation, sizeof( uint16_t ) );
		if ( chain->id != B2_NULL_INDEX )
		{
			b2SnapW_Bytes( buf, chain->shapeIndices, chain->segmentCount * (int)sizeof( int ) );
		}
	}

	int sensorCount = world->sensors.count;
	b2SnapW_I32( buf, sensorCount );
	for ( int i = 0; i < sensorCount; ++i )
	{
		b2Sensor* s = world->sensors.data + i;
		b2SnapW_I32( buf, s->shapeId );
		b2SerPodArray( buf, s->hits );
		b2SerPodArray( buf, s->overlaps1 );
		b2SerPodArray( buf, s->overlaps2 );
	}

	int islandCount = world->islands.count;
	b2SnapW_I32( buf, islandCount );
	for ( int i = 0; i < islandCount; ++i )
	{
		b2Island* island = world->islands.data + i;
		b2SnapW_I32( buf, island->setIndex );
		b2SnapW_I32( buf, island->localIndex );
		b2SnapW_I32( buf, island->islandId );
		b2SnapW_I32( buf, island->constraintRemoveCount );
		b2SerPodArray( buf, island->bodies );
		b2SerPodArray( buf, island->contacts );
		b2SerPodArray( buf, island->joints );
	}

	b2BroadPhase* bp = &world->broadPhase;
	for ( int t = 0; t < b2_bodyTypeCount; ++t )
	{
		b2SerTree( buf, &bp->trees[t] );
	}
	b2SerHashSet( buf, &bp->pairSet );

	b2ConstraintGraph* graph = &world->constraintGraph;
	for ( int c = 0; c < B2_GRAPH_COLOR_COUNT; ++c )
	{
		b2SerGraphColor( buf, &graph->colors[c], c == B2_OVERFLOW_INDEX );
	}
}

typedef struct b2SnapReader
{
	const uint8_t* data;
	int cursor;
	int size;
	bool ok;
} b2SnapReader;

static void b2SnapRCheck( b2SnapReader* r, int need )
{
	if ( need < 0 || (int64_t)r->cursor + (int64_t)need > (int64_t)r->size )
	{
		r->ok = false;
	}
}

static void b2SnapR_Bytes( b2SnapReader* r, void* dst, int n )
{
	b2SnapRCheck( r, n );
	if ( !r->ok )
	{
		return;
	}
	memcpy( dst, r->data + r->cursor, (size_t)n );
	r->cursor += n;
}

static int b2SnapR_I32( b2SnapReader* r )
{
	int32_t v = 0;
	b2SnapR_Bytes( r, &v, 4 );
	return (int)v;
}

static uint32_t b2SnapR_U32( b2SnapReader* r )
{
	uint32_t v = 0;
	b2SnapR_Bytes( r, &v, 4 );
	return v;
}

static bool b2SnapCheckCount( const b2SnapReader* r, int count, int memSize, int minStreamBytes )
{
	if ( count < 0 || memSize < 0 || minStreamBytes < 0 )
	{
		return false;
	}
	if ( memSize > 0 && count > INT32_MAX / memSize )
	{
		return false;
	}
	int64_t remaining = (int64_t)r->size - (int64_t)r->cursor;
	return (int64_t)count * (int64_t)minStreamBytes <= remaining;
}

#define b2DesPodArray( r, arr )                                                                    \
	do                                                                                             \
	{                                                                                              \
		int cnt = b2SnapR_I32( r );                                                                \
		int elemSize = (int)sizeof( *( arr ).data );                                               \
		if ( ( r )->ok && b2SnapCheckCount( r, cnt, elemSize, elemSize ) == false )                \
		{                                                                                          \
			( r )->ok = false;                                                                     \
		}                                                                                          \
		if ( ( r )->ok && cnt > 0 )                                                                \
		{                                                                                          \
			b2Array_Resize( arr, cnt );                                                            \
			b2SnapR_Bytes( r, ( arr ).data, cnt * elemSize );                                      \
		}                                                                                          \
		else if ( ( r )->ok )                                                                      \
		{                                                                                          \
			( arr ).count = 0;                                                                     \
		}                                                                                          \
	}                                                                                              \
	while ( 0 )

static void b2DesIdPool( b2SnapReader* r, b2IdPool* pool )
{
	pool->nextIndex = b2SnapR_I32( r );
	b2DesPodArray( r, pool->freeArray );
}

static void b2DesBitSet( b2SnapReader* r, b2BitSet* bs )
{
	uint32_t blockCount = b2SnapR_U32( r );
	if ( r->ok && b2SnapCheckCount( r, (int)blockCount, (int)sizeof( uint64_t ),
									(int)sizeof( uint64_t ) ) == false )
	{
		r->ok = false;
	}
	b2DestroyBitSet( bs );
	if ( !r->ok )
	{
		return;
	}
	uint32_t blockCapacity = blockCount > 0 ? blockCount : 1;
	bs->bits = static_cast<uint64_t*>( b2Alloc( blockCapacity * sizeof( uint64_t ) ) );
	memset( bs->bits, 0, blockCapacity * sizeof( uint64_t ) );
	bs->blockCapacity = blockCapacity;
	bs->blockCount = blockCount;
	if ( blockCount > 0 )
	{
		b2SnapR_Bytes( r, bs->bits, (int)( blockCount * sizeof( uint64_t ) ) );
	}
}

static void b2DesHashSet( b2SnapReader* r, b2HashSet* hs )
{
	uint32_t cap = b2SnapR_U32( r );
	uint32_t cnt = b2SnapR_U32( r );
	bool valid = b2SnapCheckCount( r, (int)cap, (int)sizeof( b2SetItem ),
									(int)sizeof( b2SetItem ) ) &&
				 ( cap & ( cap - 1 ) ) == 0 && cnt <= cap;
	if ( r->ok && valid == false )
	{
		r->ok = false;
	}
	b2DestroySet( hs );
	if ( !r->ok )
	{
		return;
	}
	if ( cap > 0 )
	{
		hs->items = static_cast<b2SetItem*>( b2Alloc( cap * sizeof( b2SetItem ) ) );
		hs->capacity = cap;
		hs->count = cnt;
		b2SnapR_Bytes( r, hs->items, (int)( cap * sizeof( b2SetItem ) ) );
	}
	else
	{
		hs->items = NULL;
		hs->capacity = 0;
		hs->count = 0;
	}
}

static void b2DesTree( b2SnapReader* r, b2DynamicTree* tree )
{
	int nodeEnd = b2SnapR_I32( r );
	int nodeCapacity = b2SnapR_I32( r );
	int pairFreeList = b2SnapR_I32( r );
	int proxyCount = b2SnapR_I32( r );
	int proxyCapacity = b2SnapR_I32( r );
	int proxyFreeList = b2SnapR_I32( r );
	int ordered = b2SnapR_I32( r );

	if ( r->ok && ( nodeEnd < 2 || ( nodeEnd & 1 ) != 0 || nodeCapacity < nodeEnd ||
					proxyCount < 0 || proxyCapacity < 1 || proxyCapacity < proxyCount ) )
	{
		r->ok = false;
	}
	if ( r->ok && b2SnapCheckCount( r, nodeEnd, (int)sizeof( b2TreeNode ),
									(int)sizeof( b2TreeNode ) + (int)sizeof( int32_t ) ) == false )
	{
		r->ok = false;
	}
	if ( r->ok && b2SnapCheckCount( r, proxyCapacity, (int)sizeof( b2TreeProxy ),
									(int)sizeof( b2TreeProxy ) ) == false )
	{
		r->ok = false;
	}

	b2DestroyDynamicTreeInternal( tree );
	if ( !r->ok )
	{
		b2CreateDynamicTreeInternal( tree, 0 );
		return;
	}

	nodeCapacity = b2MinInt( nodeCapacity, 2 * nodeEnd );
	tree->nodeEnd = nodeEnd;
	tree->nodeCapacity = nodeCapacity;
	tree->pairFreeList = pairFreeList;
	tree->proxyCount = proxyCount;
	tree->proxyCapacity = proxyCapacity;
	tree->proxyFreeList = proxyFreeList;
	tree->dfsOrdered = ordered != 0;

	tree->nodes = static_cast<b2TreeNode*>( b2AllocZero( nodeCapacity * sizeof( b2TreeNode ) ) );
	b2SnapR_Bytes( r, tree->nodes, nodeEnd * (int)sizeof( b2TreeNode ) );

	tree->parents = static_cast<int32_t*>( b2AllocZero( nodeCapacity * sizeof( int32_t ) ) );
	b2SnapR_Bytes( r, tree->parents, nodeEnd * (int)sizeof( int32_t ) );

	tree->proxies = static_cast<b2TreeProxy*>( b2AllocZero( proxyCapacity * sizeof( b2TreeProxy ) ) );
	b2SnapR_Bytes( r, tree->proxies, proxyCapacity * (int)sizeof( b2TreeProxy ) );
}

static void b2DesSolverSet( b2SnapReader* r, b2SolverSet* set )
{
	set->setIndex = b2SnapR_I32( r );
	b2DesPodArray( r, set->bodySims );
	b2DesPodArray( r, set->bodyStates );
	b2DesPodArray( r, set->jointSims );
	b2DesPodArray( r, set->contactSims );
	b2DesPodArray( r, set->islandSims );
}

static void b2DesGraphColor( b2SnapReader* r, b2GraphColor* color, bool isOverflow )
{
	if ( !isOverflow )
	{
		b2DesBitSet( r, &color->bodySet );
	}
	b2DesPodArray( r, color->contactSims );
	b2DesPodArray( r, color->jointSims );
}

static void b2DesWorldConfig( b2SnapReader* r, b2World* world )
{
	b2SnapR_Bytes( r, &world->gravity, sizeof( b2Vec2 ) );
	b2SnapR_Bytes( r, &world->hitEventThreshold, sizeof( float ) );
	b2SnapR_Bytes( r, &world->restitutionThreshold, sizeof( float ) );
	world->restitutionIterations = b2SnapR_I32( r );
	b2SnapR_Bytes( r, &world->maxLinearSpeed, sizeof( float ) );
	b2SnapR_Bytes( r, &world->contactSpeed, sizeof( float ) );
	b2SnapR_Bytes( r, &world->contactHertz, sizeof( float ) );
	b2SnapR_Bytes( r, &world->contactDampingRatio, sizeof( float ) );
	b2SnapR_Bytes( r, &world->contactRecycleDistance, sizeof( float ) );
	b2SnapR_Bytes( r, &world->stepIndex, sizeof( uint64_t ) );
	world->splitIslandId = b2SnapR_I32( r );
	b2SnapR_Bytes( r, &world->inv_h, sizeof( float ) );
	b2SnapR_Bytes( r, &world->inv_dt, sizeof( float ) );
	world->endEventArrayIndex = b2SnapR_I32( r );
	b2SnapR_Bytes( r, &world->maxCapacity, sizeof( b2Capacity ) );
	uint8_t flags = 0;
	b2SnapR_Bytes( r, &flags, 1 );
	world->enableSleep = ( flags & 0x01u ) != 0;
	world->enableWarmStarting = ( flags & 0x02u ) != 0;
	world->enableContinuous = ( flags & 0x08u ) != 0;
	world->enableRestitutionPropagation = ( flags & 0x10u ) != 0;
}

static void b2FreeLiveSimElements( b2World* world )
{
	for ( int i = 0; i < world->chainShapes.count; ++i )
	{
		b2ChainShape* chain = world->chainShapes.data + i;
		if ( chain->id != B2_NULL_INDEX && chain->shapeIndices != NULL )
		{
			b2Free( chain->shapeIndices, chain->segmentCount * (int)sizeof( int ) );
			chain->shapeIndices = NULL;
		}
	}

	for ( int i = 0; i < world->sensors.count; ++i )
	{
		b2Sensor* sensor = world->sensors.data + i;
		b2Array_Destroy( sensor->hits );
		b2Array_Destroy( sensor->overlaps1 );
		b2Array_Destroy( sensor->overlaps2 );
	}

	for ( int i = 0; i < world->islands.count; ++i )
	{
		b2Island* island = world->islands.data + i;
		b2Array_Destroy( island->bodies );
		b2Array_Destroy( island->contacts );
		b2Array_Destroy( island->joints );
	}
}

static bool b2DeserializeIntoShell( b2SnapReader* r, b2World* world )
{
	b2DesWorldConfig( r, world );

	b2DesIdPool( r, &world->bodyIdPool );
	b2DesIdPool( r, &world->shapeIdPool );
	b2DesIdPool( r, &world->chainIdPool );
	b2DesIdPool( r, &world->contactIdPool );
	b2DesIdPool( r, &world->jointIdPool );
	b2DesIdPool( r, &world->islandIdPool );
	b2DesIdPool( r, &world->solverSetIdPool );

	for ( int i = 0; i < world->solverSets.count; ++i )
	{
		b2SolverSet* set = world->solverSets.data + i;
		b2Array_Destroy( set->bodySims );
		b2Array_Destroy( set->bodyStates );
		b2Array_Destroy( set->contactSims );
		b2Array_Destroy( set->jointSims );
		b2Array_Destroy( set->islandSims );
	}

	int setCount = b2SnapR_I32( r );
	if ( r->ok && b2SnapCheckCount( r, setCount, (int)sizeof( b2SolverSet ),
									6 * (int)sizeof( int ) ) == false )
	{
		r->ok = false;
	}
	if ( r->ok )
	{
		b2Array_ResizeAndSetZero( world->solverSets, setCount );
		for ( int i = 0; i < setCount; ++i )
		{
			b2DesSolverSet( r, world->solverSets.data + i );
		}
	}

	b2DesPodArray( r, world->bodies );
	b2DesPodArray( r, world->shapes );
	b2DesPodArray( r, world->fatAABBs );
	b2DesPodArray( r, world->contacts );
	b2DesPodArray( r, world->joints );

	{
		b2Array_Destroy( world->chainShapes );
		b2Array_Create( world->chainShapes );
		int chainCount = b2SnapR_I32( r );
		if ( r->ok && b2SnapCheckCount( r, chainCount, (int)sizeof( b2ChainShape ),
										4 * (int)sizeof( int ) + (int)sizeof( uint16_t ) ) == false )
		{
			r->ok = false;
		}
		if ( r->ok )
		{
			b2Array_Resize( world->chainShapes, chainCount );
			memset( world->chainShapes.data, 0, chainCount * sizeof( b2ChainShape ) );
		}
		for ( int i = 0; i < chainCount && r->ok; ++i )
		{
			b2ChainShape* chain = world->chainShapes.data + i;
			chain->id = b2SnapR_I32( r );
			chain->bodyId = b2SnapR_I32( r );
			chain->nextChainId = b2SnapR_I32( r );
			chain->segmentCount = b2SnapR_I32( r );
			b2SnapR_Bytes( r, &chain->generation, sizeof( uint16_t ) );
			if ( r->ok && chain->id != B2_NULL_INDEX )
			{
				if ( b2SnapCheckCount( r, chain->segmentCount, (int)sizeof( int ),
										(int)sizeof( int ) ) == false )
				{
					r->ok = false;
					break;
				}
				chain->shapeIndices = static_cast<int*>( b2Alloc( chain->segmentCount * (int)sizeof( int ) ) );
				b2SnapR_Bytes( r, chain->shapeIndices, chain->segmentCount * (int)sizeof( int ) );
			}
		}
	}

	{
		b2Array_Destroy( world->sensors );
		b2Array_Create( world->sensors );
		int sensorCount = b2SnapR_I32( r );
		if ( r->ok && b2SnapCheckCount( r, sensorCount, (int)sizeof( b2Sensor ),
										4 * (int)sizeof( int ) ) == false )
		{
			r->ok = false;
		}
		if ( r->ok )
		{
			b2Array_Resize( world->sensors, sensorCount );
			memset( world->sensors.data, 0, sensorCount * sizeof( b2Sensor ) );
		}
		for ( int i = 0; i < sensorCount && r->ok; ++i )
		{
			b2Sensor* s = world->sensors.data + i;
			s->shapeId = b2SnapR_I32( r );
			b2Array_Create( s->hits );
			b2Array_Create( s->overlaps1 );
			b2Array_Create( s->overlaps2 );
			b2DesPodArray( r, s->hits );
			b2DesPodArray( r, s->overlaps1 );
			b2DesPodArray( r, s->overlaps2 );
		}
	}

	{
		b2Array_Destroy( world->islands );
		b2Array_Create( world->islands );
		int islandCount = b2SnapR_I32( r );
		if ( r->ok && b2SnapCheckCount( r, islandCount, (int)sizeof( b2Island ),
										7 * (int)sizeof( int ) ) == false )
		{
			r->ok = false;
		}
		if ( r->ok )
		{
			b2Array_Resize( world->islands, islandCount );
			memset( world->islands.data, 0, islandCount * sizeof( b2Island ) );
		}
		for ( int i = 0; i < islandCount && r->ok; ++i )
		{
			b2Island* island = world->islands.data + i;
			island->setIndex = b2SnapR_I32( r );
			island->localIndex = b2SnapR_I32( r );
			island->islandId = b2SnapR_I32( r );
			island->constraintRemoveCount = b2SnapR_I32( r );
			b2Array_Create( island->bodies );
			b2Array_Create( island->contacts );
			b2Array_Create( island->joints );
			b2DesPodArray( r, island->bodies );
			b2DesPodArray( r, island->contacts );
			b2DesPodArray( r, island->joints );
		}
	}

	{
		b2BroadPhase* bp = &world->broadPhase;
		for ( int t = 0; t < b2_bodyTypeCount; ++t )
		{
			b2DesTree( r, &bp->trees[t] );
		}
		b2DesHashSet( r, &bp->pairSet );
	}

	{
		b2ConstraintGraph* graph = &world->constraintGraph;
		for ( int c = 0; c < B2_GRAPH_COLOR_COUNT; ++c )
		{
			b2DesGraphColor( r, &graph->colors[c], c == B2_OVERFLOW_INDEX );
		}
	}

	b2Array_Clear( world->bodyMoveEvents );
	b2Array_Clear( world->sensorBeginEvents );
	b2Array_Clear( world->sensorEndEvents[0] );
	b2Array_Clear( world->sensorEndEvents[1] );
	b2Array_Clear( world->contactBeginEvents );
	b2Array_Clear( world->contactEndEvents[0] );
	b2Array_Clear( world->contactEndEvents[1] );
	b2Array_Clear( world->contactHitEvents );
	b2Array_Clear( world->jointEvents );

	return r->ok;
}

static bool b2OpenSnapshotImage( const uint8_t* image, int size, b2SnapReader* r )
{
	if ( image == NULL || size < (int)sizeof( b2SnapHeader ) )
	{
		return false;
	}

	b2SnapHeader hdr;
	memcpy( &hdr, image, sizeof( hdr ) );
	if ( hdr.magic != B2_SNAP_MAGIC || hdr.version != B2_SNAP_VERSION )
	{
		return false;
	}

	bool imageDouble = ( hdr.flags & B2_SNAP_FLAG_DOUBLE_PRECISION ) != 0;
	bool buildDouble = b2IsDoublePrecision();
	if ( imageDouble != buildDouble )
	{
		b2Log( "snapshot precision mismatch: image %s, this build %s\n",
			   imageDouble ? "double" : "float", buildDouble ? "double" : "float" );
		return false;
	}

	if ( hdr.layoutHash != b2ComputeLayoutHash() )
	{
		bool imageValidation = ( hdr.flags & B2_SNAP_FLAG_VALIDATION ) != 0;
		if ( imageValidation != (bool)B2_ENABLE_VALIDATION )
		{
			b2Log( "snapshot layout mismatch: image built with validation %s, this build %s\n",
				   imageValidation ? "on" : "off", B2_ENABLE_VALIDATION ? "on" : "off" );
		}
		else
		{
			b2Log( "snapshot layout mismatch\n" );
		}
		return false;
	}

	r->data = image;
	r->cursor = (int)sizeof( hdr );
	r->size = size;
	r->ok = true;
	return true;
}

b2WorldId b2CreateWorldFromSnapshot( const uint8_t* image, int size, int workerCount )
{
	b2WorldId nullId = {};
	b2SnapReader reader;
	if ( b2OpenSnapshotImage( image, size, &reader ) == false )
	{
		return nullId;
	}

	b2WorldDef def = b2DefaultWorldDef();
	def.workerCount = workerCount;
	b2WorldId id = b2CreateWorld( &def );
	if ( !b2World_IsValid( id ) )
	{
		return nullId;
	}

	b2World* world = b2_worlds + ( id.index1 - 1 );
	if ( b2DeserializeIntoShell( &reader, world ) == false )
	{
		b2DestroyWorld( id );
		return nullId;
	}

	world->preSolveFcn = NULL;
	world->preSolveContext = NULL;
	world->customFilterFcn = NULL;
	world->customFilterContext = NULL;
	world->userData = NULL;
	return id;
}

bool b2World_RestoreFromSnapshot( b2WorldId worldId, const uint8_t* image, int size )
{
	b2SnapReader reader;
	if ( b2OpenSnapshotImage( image, size, &reader ) == false )
	{
		return false;
	}

	B2_ASSERT( 1 <= worldId.index1 && worldId.index1 <= B2_MAX_WORLDS );
	b2World* world = b2_worlds + ( worldId.index1 - 1 );
	B2_ASSERT( worldId.index1 == world->worldId + 1 );
	B2_ASSERT( worldId.generation == world->generation );
	if ( world->locked )
	{
		return false;
	}

	b2FreeLiveSimElements( world );
	return b2DeserializeIntoShell( &reader, world );
}

int b2World_GetSnapshot( b2WorldId worldId, uint8_t* image, int capacity )
{
	B2_ASSERT( 1 <= worldId.index1 && worldId.index1 <= B2_MAX_WORLDS );
	b2World* world = b2_worlds + ( worldId.index1 - 1 );
	B2_ASSERT( worldId.index1 == world->worldId + 1 );
	B2_ASSERT( worldId.generation == world->generation );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return 0;
	}

	if ( image == NULL )
	{
		b2RecBuffer counter = {};
		counter.countOnly = true;
		b2SerializeWorld( world, &counter );
		return counter.size;
	}

	b2RecBuffer buf = {};
	b2SerializeWorld( world, &buf );
	int size = buf.size;

	if ( size <= capacity )
	{
		memcpy( image, buf.data, size );
	}

	b2RecBufFree( &buf );
	return size;
}

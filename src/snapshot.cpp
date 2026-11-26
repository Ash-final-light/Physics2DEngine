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

struct b2SnapBuffer
{
	uint8_t* data;
	int capacity;
	int size;
	bool countOnly;
};

static void b2SnapBufAppend( b2SnapBuffer* buf, const void* data, int size )
{
	if ( size <= 0 )
	{
		return;
	}

	if ( buf->countOnly )
	{
		buf->size += size;
		return;
	}

	if ( buf->size + size > buf->capacity )
	{
		int newCap = buf->capacity * 2;
		if ( newCap < buf->size + size + 64 )
		{
			newCap = buf->size + size + 64;
		}
		if ( buf->data == NULL )
		{
			buf->data = static_cast<uint8_t*>( b2Alloc( newCap ) );
		}
		else
		{
			buf->data = static_cast<uint8_t*>( b2GrowAlloc( buf->data, buf->capacity, newCap ) );
		}
		buf->capacity = newCap;
	}

	memcpy( buf->data + buf->size, data, (size_t)size );
	buf->size += size;
}

static void b2SnapBufFree( b2SnapBuffer* buf )
{
	if ( buf->data != NULL )
	{
		b2Free( buf->data, buf->capacity );
		buf->data = NULL;
		buf->capacity = 0;
		buf->size = 0;
	}
}

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

static void b2SnapW_I32( b2SnapBuffer* buf, int v )
{
	int32_t w = (int32_t)v;
	b2SnapBufAppend( buf, &w, 4 );
}

static void b2SnapW_U32( b2SnapBuffer* buf, uint32_t v )
{
	b2SnapBufAppend( buf, &v, 4 );
}

static void b2SnapW_Bytes( b2SnapBuffer* buf, const void* src, int n )
{
	b2SnapBufAppend( buf, src, n );
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

static void b2SerIdPool( b2SnapBuffer* buf, const b2IdPool* pool )
{
	b2SnapW_I32( buf, pool->nextIndex );
	b2SerPodArray( buf, pool->freeArray );
}

static void b2SerBitSet( b2SnapBuffer* buf, const b2BitSet* bs )
{
	b2SnapW_U32( buf, bs->blockCount );
	if ( bs->blockCount > 0 )
	{
		b2SnapW_Bytes( buf, bs->bits, (int)( bs->blockCount * sizeof( uint64_t ) ) );
	}
}

static void b2SerHashSet( b2SnapBuffer* buf, const b2HashSet* hs )
{
	b2SnapW_U32( buf, hs->capacity );
	b2SnapW_U32( buf, hs->count );
	if ( hs->capacity > 0 )
	{
		b2SnapW_Bytes( buf, hs->items, (int)( hs->capacity * sizeof( b2SetItem ) ) );
	}
}

static void b2SerTree( b2SnapBuffer* buf, const b2DynamicTree* tree )
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

static void b2SerSolverSet( b2SnapBuffer* buf, const b2SolverSet* set )
{
	b2SnapW_I32( buf, set->setIndex );
	b2SerPodArray( buf, set->bodySims );
	b2SerPodArray( buf, set->bodyStates );
	b2SerPodArray( buf, set->jointSims );
	b2SerPodArray( buf, set->contactSims );
	b2SerPodArray( buf, set->islandSims );
}

static void b2SerGraphColor( b2SnapBuffer* buf, const b2GraphColor* color, bool isOverflow )
{
	if ( !isOverflow )
	{
		b2SerBitSet( buf, &color->bodySet );
	}
	b2SerPodArray( buf, color->contactSims );
	b2SerPodArray( buf, color->jointSims );
}

static void b2SerWorldConfig( b2SnapBuffer* buf, const b2World* world )
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
	b2SnapBufAppend( buf, &flags, 1 );
}

void b2SerializeWorld( b2World* world, b2SnapBuffer* buf )
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
		b2SnapBuffer counter = {};
		counter.countOnly = true;
		b2SerializeWorld( world, &counter );
		return counter.size;
	}

	b2SnapBuffer buf = {};
	b2SerializeWorld( world, &buf );
	int size = buf.size;

	if ( size <= capacity )
	{
		memcpy( image, buf.data, size );
	}

	b2SnapBufFree( &buf );
	return size;
}

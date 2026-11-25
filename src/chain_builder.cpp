// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "container.h"
#include "core.h"

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>

#define NAME b2IndexMap
#define KEY_TY uint64_t
#define VAL_TY int
#define HASH_FN vt_hash_integer
#define CMPR_FN vt_cmpr_integer
#define MALLOC_FN b2Alloc
#define FREE_FN b2Free
#include "verstable.h"

typedef struct b2BuilderEdge
{
	int vertex1;
	int vertex2;
} b2BuilderEdge;

typedef struct b2EdgeSplit
{
	float fraction;
	int vertex;
} b2EdgeSplit;

b2DeclareArray( b2BuilderEdge );
b2DeclareArray( b2EdgeSplit );
b2DeclareArray( b2ChainData );
b2DeclareArray( b2Vec2 );

typedef struct b2SplitContext
{
	const b2Vec2* vertices;
	b2Vec2 point1;
	b2Vec2 point2;
	int vertex1;
	int vertex2;
	float toleranceSquared;

	// T 形交叉拆分。
	b2Array( b2EdgeSplit ) splits;
} b2SplitContext;

static inline uint64_t b2CellKey( int x, int y )
{
	return (uint64_t)(uint32_t)x << 32 | (uint64_t)(uint32_t)y;
}

static inline uint64_t b2EdgeKey( int vertex1, int vertex2 )
{
	return (uint64_t)(uint32_t)vertex1 << 32 | (uint64_t)(uint32_t)vertex2;
}

static inline float b2DistanceSquaredToSegment( b2Vec2 p, b2Vec2 a, b2Vec2 b )
{
	b2Vec2 d = b2Sub( b, a );
	float lengthSquared = b2Dot( d, d );
	if ( lengthSquared == 0.0f )
	{
		return b2DistanceSquared( p, a );
	}

	float t = b2ClampFloat( b2Dot( b2Sub( p, a ), d ) / lengthSquared, 0.0f, 1.0f );
	return b2DistanceSquared( p, b2MulAdd( a, t, d ) );
}

static void b2TestSplitVertex( b2SplitContext* context, int vertex )
{
	if ( vertex == context->vertex1 || vertex == context->vertex2 )
	{
		return;
	}

	b2Vec2 p = context->vertices[vertex];
	b2Vec2 a = context->point1;
	b2Vec2 d = b2Sub( context->point2, a );
	float dd = b2Dot( d, d );
	float fraction = b2Dot( b2Sub( p, a ), d );
	if ( fraction <= 0.0f || dd <= fraction )
	{
		return;
	}

	fraction /= dd;

	if ( b2DistanceSquared( p, b2MulAdd( a, fraction, d ) ) > context->toleranceSquared )
	{
		return;
	}

	b2EdgeSplit split = {
		.fraction = fraction,
		.vertex = vertex,
	};

	b2Array_Push( context->splits, split );
}

static inline int b2CellCoordinate( float x, float inverseCellSize )
{
	return (int)floorf( x * inverseCellSize );
}

// 用空间哈希焊接顶点。与 Box3D 的网格焊机非常相似。
static int b2WeldVertices( const b2Polygon* polygons, int polygonCount, float tolerance,
						   b2Vec2* vertices, int* remap, int* cellNext )
{
	b2IndexMap cellMap;
	b2IndexMap_init( &cellMap );

	float cellSize = 2.0f * tolerance;
	float toleranceSquared = tolerance * tolerance;
	int vertexCount = 0;
	int inputIndex = 0;

	for ( int i = 0; i < polygonCount; ++i )
	{
		const b2Polygon* polygon = polygons + i;
		for ( int j = 0; j < polygon->count; ++j )
		{
			b2Vec2 p = polygon->vertices[j];
			int cellX = (int)floorf( p.x / cellSize );
			int cellY = (int)floorf( p.y / cellSize );

			int bestVertex = B2_NULL_INDEX;
			float bestDistanceSquared = FLT_MAX;
			for ( int dx = -1; dx <= 1; ++dx )
			{
				for ( int dy = -1; dy <= 1; ++dy )
				{
					b2IndexMap_itr itr =
						b2IndexMap_get( &cellMap, b2CellKey( cellX + dx, cellY + dy ) );
					if ( b2IndexMap_is_end( itr ) )
					{
						continue;
					}

					for ( int k = itr.data->val; k != B2_NULL_INDEX; k = cellNext[k] )
					{
						float distanceSquared = b2DistanceSquared( vertices[k], p );
						if ( distanceSquared < bestDistanceSquared )
						{
							bestVertex = k;
							bestDistanceSquared = distanceSquared;
						}
					}
				}
			}

			if ( bestVertex == B2_NULL_INDEX || bestDistanceSquared > toleranceSquared )
			{
				bestVertex = vertexCount++;
				vertices[bestVertex] = p;

				uint64_t key = b2CellKey( cellX, cellY );
				b2IndexMap_itr itr = b2IndexMap_get( &cellMap, key );
				cellNext[bestVertex] = b2IndexMap_is_end( itr ) ? B2_NULL_INDEX : itr.data->val;
				b2IndexMap_insert( &cellMap, key, bestVertex );
			}

			remap[inputIndex++] = bestVertex;
		}
	}

	b2IndexMap_cleanup( &cellMap );
	return vertexCount;
}

// 拆分 T 形交叉。会产生共线边，但后续阶段会抵消。
static void b2SplitEdges( const b2Polygon* polygons, int polygonCount, const b2Vec2* vertices,
						  int vertexCount, const int* remap, float tolerance,
						  b2Array( b2BuilderEdge ) * edges )
{
	float lengthSum = 0.0f;
	int inputEdgeCount = 0;
	int base = 0;

	// 统计边并计算总边长，跳过已被焊接掉的顶点。
	for ( int i = 0; i < polygonCount; ++i )
	{
		int count = polygons[i].count;
		for ( int j = 0; j < count; ++j )
		{
			int vertex1 = remap[base + j];
			int vertex2 = remap[base + ( j + 1 < count ? j + 1 : 0 )];
			if ( vertex1 != vertex2 )
			{
				lengthSum += b2Distance( vertices[vertex1], vertices[vertex2] );
				inputEdgeCount += 1;
			}
		}
		base += count;
	}

	if ( inputEdgeCount == 0 )
	{
		return;
	}

	// 根据平均边长确定哈希网格单元大小。
	float cellSize = b2MaxFloat( lengthSum / inputEdgeCount, 4.0f * tolerance );
	float inverseCellSize = 1.0f / cellSize;

	// 把所有顶点放入空间网格，每格用顶点链表。
	b2IndexMap grid;
	b2IndexMap_init( &grid );
	b2IndexMap_reserve( &grid, vertexCount );
	int* gridNext = static_cast<int *>( b2Alloc( vertexCount * sizeof( int ) ) );
	for ( int i = 0; i < vertexCount; ++i )
	{
		uint64_t key = b2CellKey( b2CellCoordinate( vertices[i].x, inverseCellSize ),
								  b2CellCoordinate( vertices[i].y, inverseCellSize ) );
		b2IndexMap_itr itr = b2IndexMap_get( &grid, key );

		// 追加链表。
		gridNext[i] = b2IndexMap_is_end( itr ) ? B2_NULL_INDEX : itr.data->val;

		// 覆盖链表头。
		b2IndexMap_insert( &grid, key, i );
	}

	b2SplitContext context = {};
	context.vertices = vertices;
	context.toleranceSquared = tolerance * tolerance;

	// 在网格上遍历每个多边形的边。
	base = 0;
	for ( int i = 0; i < polygonCount; ++i )
	{
		int count = polygons[i].count;
		for ( int j = 0; j < count; ++j )
		{
			int vertex1 = remap[base + j];
			int vertex2 = remap[base + ( j + 1 < count ? j + 1 : 0 )];
			if ( vertex1 == vertex2 )
			{
				continue;
			}

			b2Vec2 p1 = vertices[vertex1];
			b2Vec2 p2 = vertices[vertex2];

			context.point1 = p1;
			context.point2 = p2;
			context.vertex1 = vertex1;
			context.vertex2 = vertex2;
			context.splits.count = 0;

			// 遍历网格。先是列范围。再对每行使用线段经过的单元格范围。
			float minX = b2MinFloat( p1.x, p2.x );
			float maxX = b2MaxFloat( p1.x, p2.x );
			float dx = p2.x - p1.x;
			int firstColumn = b2CellCoordinate( minX - tolerance, inverseCellSize );
			int lastColumn = b2CellCoordinate( maxX + tolerance, inverseCellSize );
			for ( int column = firstColumn; column <= lastColumn; ++column )
			{
				float xa = b2ClampFloat( column * cellSize - tolerance, minX, maxX );
				float xb = b2ClampFloat( ( column + 1 ) * cellSize + tolerance, minX, maxX );
				float ya = p1.y;
				float yb = p2.y;
				if ( dx != 0.0f )
				{
					float ta = b2ClampFloat( ( xa - p1.x ) / dx, 0.0f, 1.0f );
					float tb = b2ClampFloat( ( xb - p1.x ) / dx, 0.0f, 1.0f );
					ya = p1.y + ta * ( p2.y - p1.y );
					yb = p1.y + tb * ( p2.y - p1.y );
				}

				int firstRow =
					b2CellCoordinate( b2MinFloat( ya, yb ) - tolerance, inverseCellSize );
				int lastRow = b2CellCoordinate( b2MaxFloat( ya, yb ) + tolerance, inverseCellSize );
				for ( int row = firstRow; row <= lastRow; ++row )
				{
					b2IndexMap_itr itr = b2IndexMap_get( &grid, b2CellKey( column, row ) );
					if ( b2IndexMap_is_end( itr ) )
					{
						continue;
					}

					// 遍历此网格单元中的顶点链表。查找与此边重叠的其他边顶点。收集到上下文中。
					for ( int k = itr.data->val; k != B2_NULL_INDEX; k = gridNext[k] )
					{
						b2TestSplitVertex( &context, k );
					}
				}
			}

			// 用插入排序按 fraction 排序拆分点。若两个拆分 fraction 相同，
			// 则较小顶点下标在前。
			// https://en.wikipedia.org/wiki/Insertion_sort
			b2EdgeSplit* splits = context.splits.data;
			int splitCount = context.splits.count;
			for ( int k = 1; k < splitCount; ++k )
			{
				b2EdgeSplit split = splits[k];
				int m = k;
				while ( m > 0 && ( splits[m - 1].fraction > split.fraction ||
								   ( splits[m - 1].fraction == split.fraction &&
									 splits[m - 1].vertex > split.vertex ) ) )
				{
					splits[m] = splits[m - 1];
					m -= 1;
				}
				splits[m] = split;
			}

			// 在原两端点之间按顺序构建子边。
			int previous = vertex1;
			for ( int k = 0; k < splitCount; ++k )
			{
				b2BuilderEdge edge = { previous, splits[k].vertex };
				b2Array_Push( *edges, edge );

				// 此顶点已存在，因为它属于另一条边。
				previous = splits[k].vertex;
			}

			// 最后一段。
			b2BuilderEdge edge = { previous, vertex2 };
			b2Array_Push( *edges, edge );
		}

		base += count;
	}

	b2Array_Destroy( context.splits );
	b2Free( gridNext, vertexCount * sizeof( int ) );
	b2IndexMap_cleanup( &grid );
}

// 按顶点下标建集合来抵消边。然后对每条边查找孪生边。找到则两边都剔除。
// 还会抵消重复对。因此可能有 3 条相同边和 3 条相同孪生边，全部抵消。
// 3/2 会留下一条。
static bool b2CancelEdges( const b2BuilderEdge* edges, int edgeCount, bool* alive )
{
	bool hasDuplicate = false;

	// 用于检测重复。
	int* edgeNext = static_cast<int *>( b2Alloc( edgeCount * sizeof( int ) ) );

	b2IndexMap edgeMap;
	b2IndexMap_init( &edgeMap );
	b2IndexMap_reserve( &edgeMap, edgeCount );

	for ( int i = 0; i < edgeCount; ++i )
	{
		alive[i] = true;

		b2BuilderEdge edge = edges[i];

		// 在映射中查找反向边。
		b2IndexMap_itr reverse =
			b2IndexMap_get( &edgeMap, b2EdgeKey( edge.vertex2, edge.vertex1 ) );

		// 反向边是否在映射中且链表非空？
		if ( b2IndexMap_is_end( reverse ) == false && reverse.data->val != B2_NULL_INDEX )
		{
			// 弹出链表头。可能变为空。
			int other = reverse.data->val;
			reverse.data->val = edgeNext[other];
			alive[other] = false;
			alive[i] = false;
			continue;
		}

		// 在映射中查找已有边。
		uint64_t key = b2EdgeKey( edge.vertex1, edge.vertex2 );
		b2IndexMap_itr itr = b2IndexMap_get( &edgeMap, key );

		// 若该边已存在则追加到重复列表。
		edgeNext[i] = b2IndexMap_is_end( itr ) ? B2_NULL_INDEX : itr.data->val;

		// 插入或覆盖。
		b2IndexMap_insert( &edgeMap, key, i );

		// 发现相同边。数据有误。
		hasDuplicate = hasDuplicate || edgeNext[i] != B2_NULL_INDEX;
	}

	b2IndexMap_cleanup( &edgeMap );
	b2Free( edgeNext, edgeCount * sizeof( int ) );
	return hasDuplicate;
}

static int b2RemoveCollinearPoints( const b2Vec2* points, int count, float tolerance,
									b2Vec2* output )
{
	float toleranceSquared = tolerance * tolerance;

	int start = B2_NULL_INDEX;
	for ( int i = 0; i < count; ++i )
	{
		b2Vec2 previous = points[i > 0 ? i - 1 : count - 1];
		b2Vec2 next = points[i + 1 < count ? i + 1 : 0];
		if ( b2DistanceSquaredToSegment( points[i], previous, next ) > toleranceSquared )
		{
			start = i;
			break;
		}
	}

	if ( start == B2_NULL_INDEX )
	{
		return 0;
	}

	int outputCount = 0;
	output[outputCount++] = points[start];

	b2Vec2 anchor = points[start];
	bool hasCone = false;
	b2Vec2 left = b2Vec2_zero;
	b2Vec2 right = b2Vec2_zero;
	float reachSquared = 0.0f;

	for ( int k = 1; k < count; ++k )
	{
		b2Vec2 point = points[( start + k ) % count];
		b2Vec2 next = points[( start + k + 1 ) % count];

		b2Vec2 offset = b2Sub( point, anchor );
		float distanceSquared = b2Dot( offset, offset );

		bool coneValid = hasCone;
		b2Vec2 coneLeft = left;
		b2Vec2 coneRight = right;
		if ( distanceSquared > toleranceSquared )
		{
			float distance = sqrtf( distanceSquared );
			b2Vec2 u = b2MulSV( 1.0f / distance, offset );
			float s = tolerance / distance;
			float c = sqrtf( 1.0f - s * s );
			b2Vec2 pointLeft = { c * u.x - s * u.y, s * u.x + c * u.y };
			b2Vec2 pointRight = { c * u.x + s * u.y, c * u.y - s * u.x };

			if ( coneValid == false )
			{
				coneLeft = pointLeft;
				coneRight = pointRight;
				coneValid = true;
			}
			else
			{
				if ( b2Cross( pointLeft, coneLeft ) > 0.0f )
				{
					coneLeft = pointLeft;
				}

				if ( b2Cross( coneRight, pointRight ) > 0.0f )
				{
					coneRight = pointRight;
				}
			}
		}

		b2Vec2 reach = b2Sub( next, anchor );
		float newReachSquared = b2MaxFloat( reachSquared, distanceSquared );
		bool collinear = b2Dot( reach, reach ) >= newReachSquared;
		if ( collinear && coneValid )
		{
			collinear = b2Cross( coneRight, reach ) >= 0.0f && b2Cross( reach, coneLeft ) >= 0.0f;
		}

		if ( collinear )
		{
			hasCone = coneValid;
			left = coneLeft;
			right = coneRight;
			reachSquared = newReachSquared;
			continue;
		}

		if ( b2DistanceSquared( point, output[outputCount - 1] ) > toleranceSquared )
		{
			output[outputCount++] = point;
		}

		anchor = point;
		hasCone = false;
		reachSquared = 0.0f;
	}

	while ( outputCount > 1 &&
			b2DistanceSquared( output[outputCount - 1], output[0] ) <= toleranceSquared )
	{
		outputCount -= 1;
	}

	return outputCount;
}

b2ChainResult b2ComputeChains( const b2Polygon* polygons, int polygonCount, float weldTolerance )
{
	b2ChainResult result = { NULL, 0, b2_chainStatusSuccess };

	if ( polygons == NULL || polygonCount <= 0 )
	{
		return result;
	}

	if ( b2IsValidFloat( weldTolerance ) == false )
	{
		B2_ASSERT( false );
		return result;
	}

	int inputCount = 0;
	for ( int i = 0; i < polygonCount; ++i )
	{
		B2_ASSERT( 3 <= polygons[i].count && polygons[i].count <= B2_MAX_POLYGON_VERTICES );
		inputCount += polygons[i].count;
	}

	// 最小焊接容差为线性间隙，因为流形代码使用该值。
	float tolerance = b2MaxFloat( weldTolerance, B2_LINEAR_SLOP );

	// 焊接容差内重叠的顶点。
	b2Vec2* vertices = static_cast<b2Vec2 *>( b2Alloc( inputCount * sizeof( b2Vec2 ) ) );
	int* vertexMap = static_cast<int *>( b2Alloc( inputCount * sizeof( int ) ) );
	int* cellNext = static_cast<int *>( b2Alloc( inputCount * sizeof( int ) ) );

	int vertexCount =
		b2WeldVertices( polygons, polygonCount, tolerance, vertices, vertexMap, cellNext );

	// 拆分 T 形交叉。为将构建的边预留空间。
	b2Array( b2BuilderEdge ) edgeArray;
	b2Array_CreateN( edgeArray, inputCount );
	b2SplitEdges( polygons, polygonCount, vertices, vertexCount, vertexMap, tolerance, &edgeArray );

	b2Free( cellNext, inputCount * sizeof( int ) );
	b2Free( vertexMap, inputCount * sizeof( int ) );

	// 这些边现在按每个多边形堆叠后的绕序。每个多边形的线段在 T 形交叉处被拆分。
	const b2BuilderEdge* edges = edgeArray.data;
	int edgeCount = edgeArray.count;

	// 抵消相同但反向平行的边。从而得到轮廓。
	bool* alive = static_cast<bool *>( b2Alloc( edgeCount * sizeof( bool ) ) );
	bool hasDuplicate = b2CancelEdges( edges, edgeCount, alive );

	// 按起始顶点用计数排序对幸存边分组。得到邻接表。
	// https://en.wikipedia.org/wiki/Counting_sort

	// 用计数排序得到每个顶点的出边数。
	// 例如：[0 3 4] 表示 vertex0 有 3 条出边，vertex1 有 1 条。
	int* outStart = static_cast<int *>( b2AllocZero( ( vertexCount + 1 ) * sizeof( int ) ) );

	// 每个顶点的出边下标。
	// 例如，用 outStart [0 3 4]：[1 3 4 5] 表示 vertex0 的出边为 1,3,4，
	// vertex1 的出边为 5
	int* outEdges = static_cast<int *>( b2Alloc( edgeCount * sizeof( int ) ) );

	for ( int i = 0; i < edgeCount; ++i )
	{
		// 未被抵消？
		if ( alive[i] )
		{
			outStart[edges[i].vertex1 + 1] += 1;
		}
	}

	// 前缀和
	for ( int i = 0; i < vertexCount; ++i )
	{
		outStart[i + 1] += outStart[i];
	}

	// 填充出边列表，每个顶点对应一段连续子数组。
	// 此数组是每个顶点在出边列表中的游标。
	int* fill = static_cast<int *>( b2Alloc( vertexCount * sizeof( int ) ) );
	memcpy( fill, outStart, vertexCount * sizeof( int ) );
	for ( int i = 0; i < edgeCount; ++i )
	{
		if ( alive[i] )
		{
			// 按顶点的出边。
			outEdges[fill[edges[i].vertex1]] = i;

			// 递增游标。
			fill[edges[i].vertex1] += 1;
		}
	}
	b2Free( fill, vertexCount * sizeof( int ) );

	// 一个顶点有多条指向不同目标顶点的出边时称为 pinch。链条会两次经过它，
	// 那里的幽灵顶点有歧义。指向同一目标顶点的出边只来自重复边。
	bool hasPinch = false;
	for ( int i = 0; i < vertexCount && hasPinch == false; ++i )
	{
		int outGoingCount = outStart[i + 1] - outStart[i];
		if ( outGoingCount < 2 )
		{
			continue;
		}

		int target = edges[outEdges[outStart[i]]].vertex2;
		for ( int j = outStart[i] + 1; j < outStart[i + 1]; ++j )
		{
			if ( edges[outEdges[j]].vertex2 != target )
			{
				hasPinch = true;
				break;
			}
		}
	}

	// 重复是更严重的问题。
	if ( hasDuplicate )
	{
		result.status = b2_chainStatusOverlap;
	}
	else if ( hasPinch )
	{
		result.status = b2_chainStatusCornerTouch;
	}

	// 用出边列表构建环。需要通过选择最右转来处理 pinch。
	bool* used = static_cast<bool *>( b2AllocZero( edgeCount * sizeof( bool ) ) );

	b2Array( b2Vec2 ) loop = {};
	b2Array( b2Vec2 ) reduced = {};
	b2Array( b2ChainData ) chains = {};

	for ( int firstEdge = 0; firstEdge < edgeCount; ++firstEdge )
	{
		if ( alive[firstEdge] == false || used[firstEdge] )
		{
			continue;
		}

		// 从当前边开始构建环。
		loop.count = 0;
		int edgeIndex = firstEdge;
		bool openPath = false;

		while ( true )
		{
			used[edgeIndex] = true;

			b2BuilderEdge edge = edges[edgeIndex];
			b2Array_Push( loop, vertices[edge.vertex1] );

			// 边方向。
			b2Vec2 p1 = vertices[edge.vertex1];
			b2Vec2 p2 = vertices[edge.vertex2];
			b2Vec2 direction = b2Sub( p2, p1 );

			// 取最锐利的右转。相对入边方向的最小有符号角。
			int nextEdge = B2_NULL_INDEX;
			float bestAngle = FLT_MAX;

			// 考虑 vertex2 的每条出边。
			for ( int k = outStart[edge.vertex2]; k < outStart[edge.vertex2 + 1]; ++k )
			{
				int candidate = outEdges[k];
				if ( used[candidate] && candidate != firstEdge )
				{
					continue;
				}

				b2Vec2 p3 = vertices[edges[candidate].vertex2];
				b2Vec2 candidateDirection = b2Sub( p3, p2 );

				// 有符号角
				float angle = b2Atan2( b2Cross( direction, candidateDirection ),
									   b2Dot( direction, candidateDirection ) );

				if ( angle < bestAngle )
				{
					nextEdge = candidate;
					bestAngle = angle;
				}
			}

			if ( nextEdge == B2_NULL_INDEX )
			{
				// 下一条边为空则算法失败。
				B2_ASSERT( false );
				openPath = true;
				break;
			}

			if ( nextEdge == firstEdge )
			{
				// 环已闭合。
				break;
			}

			edgeIndex = nextEdge;
		}

		// 失败
		if ( openPath )
		{
			continue;
		}

		// 环已就绪。现在移除共线点。
		b2Array_Reserve( reduced, loop.count );
		int pointCount =
			b2RemoveCollinearPoints( loop.data, loop.count, B2_LINEAR_SLOP, reduced.data );

		if ( pointCount < 3 )
		{
			continue;
		}

		// 发布此环。
		b2ChainData chain;
		chain.points = static_cast<b2Vec2 *>( b2Alloc( pointCount * sizeof( b2Vec2 ) ) );
		chain.count = pointCount;
		memcpy( chain.points, reduced.data, pointCount * sizeof( b2Vec2 ) );
		b2Array_Push( chains, chain );
	}

	b2Free( used, edgeCount * sizeof( bool ) );
	b2Free( outEdges, edgeCount * sizeof( int ) );
	b2Free( outStart, ( vertexCount + 1 ) * sizeof( int ) );
	b2Free( alive, edgeCount * sizeof( bool ) );
	b2Array_Destroy( edgeArray );
	b2Free( vertices, inputCount * sizeof( b2Vec2 ) );
	b2Array_Destroy( loop );
	b2Array_Destroy( reduced );

	// 完成结果。
	if ( chains.count > 0 )
	{
		result.chains = static_cast<b2ChainData *>( b2Alloc( chains.count * sizeof( b2ChainData ) ) );
		memcpy( result.chains, chains.data, chains.count * sizeof( b2ChainData ) );
		result.chainCount = chains.count;
	}

	b2Array_Destroy( chains );
	return result;
}

void b2FreeChains( b2ChainResult* result )
{
	if ( result == NULL || result->chains == NULL )
	{
		return;
	}

	for ( int i = 0; i < result->chainCount; ++i )
	{
		b2Free( result->chains[i].points, result->chains[i].count * sizeof( b2Vec2 ) );
	}

	b2Free( result->chains, result->chainCount * sizeof( b2ChainData ) );
	result->chains = NULL;
	result->chainCount = 0;
}

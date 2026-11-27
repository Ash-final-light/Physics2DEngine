// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "sensor.h"

#include "body.h"
#include "contact.h"
#include "world.h"
#include "shape.h"
#include "solver_set.h"
#include "parallel_for.h"

#include "box2d/collision.h"

#include <stddef.h>
#include <stdlib.h>
#include <float.h>
#include <bit>

struct b2SensorQueryContext
{
	b2World* world;
	b2SensorTaskContext* taskContext;
	b2Sensor* sensor;
	b2Shape* sensorShape;
	b2WorldTransform transform;
};

// 传感器形状需要
// - 检测重叠开始和结束事件
// - 事件必须以确定性顺序报告
// - 维护可供查询的活动重叠列表

// 假设
// - 传感器不检测同一刚体上的形状

// 算法
// 查询所有传感器的重叠
// 与先前重叠对照

// 数据结构
// 每个传感器有双缓冲的重叠数组
// 这些重叠使用带索引和世代的形状引用

static bool b2SensorQueryCallback( int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;

	int shapeId = (int)userData;

	struct b2SensorQueryContext* queryContext = static_cast<b2SensorQueryContext *>( context );
	b2Shape* sensorShape = queryContext->sensorShape;
	int sensorShapeId = sensorShape->id;

	if ( shapeId == sensorShapeId )
	{
		return true;
	}

	b2World* world = queryContext->world;
	b2Shape* otherShape = b2Array_Get( world->shapes, shapeId );

	// 另一形状是否启用了传感器事件？
	if ( otherShape->enableSensorEvents == false )
	{
		return true;
	}

	// 跳过同一刚体上的形状
	if ( otherShape->bodyId == sensorShape->bodyId )
	{
		return true;
	}

	// 检查过滤器
	if ( b2ShouldShapesCollide( sensorShape->filter, otherShape->filter ) == false )
	{
		return true;
	}

	// 自定义用户过滤器


	// 相对位姿用 double 做差，使远离原点时传感器重叠仍精确
    if((sensorShape->enableCustomFiltering || otherShape->enableCustomFiltering) && world->customFilterFcn!=NULL)
    {
        b2ShapeId idA={sensorShape->id+1,world->worldId,sensorShape->generation},idB={otherShape->id+1,world->worldId,otherShape->generation};
        if(!world->customFilterFcn(idA,idB,world->customFilterContext)) { return true; }
    }
	b2WorldTransform otherTransform = b2GetBodySim(world,world->bodies.data+otherShape->bodyId)->transform;

	b2DistanceInput input;
	input.proxyA = b2MakeShapeDistanceProxy( sensorShape );
	input.proxyB = b2MakeShapeDistanceProxy( otherShape );
	input.transform = b2InvMulWorldTransforms( queryContext->transform, otherTransform );
	input.useRadii = true;
	b2SimplexCache cache = {};
	b2DistanceOutput output = b2ComputeShapeDistance( &input, &cache, NULL, 0 );

	bool overlaps = output.distance < 10.0f * FLT_EPSILON;
	if ( overlaps == false )
	{
		return true;
	}

	// 记录重叠
	b2Sensor* sensor = queryContext->sensor;
	b2Visitor* shapeRef = b2Array_Emplace( sensor->overlaps2 );
	shapeRef->shapeId = shapeId;
	shapeRef->generation = otherShape->generation;

	return true;
}

static int b2CompareVisitors( const void* a, const void* b )
{
	const b2Visitor* sa = static_cast<const b2Visitor *>( a );
	const b2Visitor* sb = static_cast<const b2Visitor *>( b );

	if ( sa->shapeId < sb->shapeId )
	{
		return -1;
	}

	return 1;
}

static void b2SensorTask( int startIndex, int endIndex, int threadIndex, void* context )
{

	b2World* world = static_cast<b2World *>( context );
	b2SensorTaskContext* taskContext = world->sensorTaskContexts.data + threadIndex;

	B2_ASSERT( startIndex < endIndex );

	b2DynamicTree* trees = world->broadPhase.trees;
	for ( int sensorIndex = startIndex; sensorIndex < endIndex; ++sensorIndex )
	{
		b2Sensor* sensor = b2Array_Get( world->sensors, sensorIndex );
		b2Shape* sensorShape = b2Array_Get( world->shapes, sensor->shapeId );

		// 交换重叠数组
		b2Array( b2Visitor ) temp = sensor->overlaps1;
		sensor->overlaps1 = sensor->overlaps2;
		sensor->overlaps2 = temp;
		b2Array_Clear( sensor->overlaps2 );

		// 追加传感器命中
		int hitCount = sensor->hits.count;
		for ( int i = 0; i < hitCount; ++i )
		{
			b2Array_Push( sensor->overlaps2, sensor->hits.data[i] );
		}

		// 清除命中
		b2Array_Clear( sensor->hits );

		b2Body* body = b2Array_Get( world->bodies, sensorShape->bodyId );
		if ( body->setIndex == b2_disabledSet || sensorShape->enableSensorEvents == false )
		{
			if ( sensor->overlaps1.count != 0 )
			{
				// 此传感器已被禁用，因此丢弃全部重叠。
				b2SetBit( &taskContext->eventBits, sensorIndex );
			}
			continue;
		}

		b2WorldTransform transform = b2GetBodySim(world,body)->transform;

		struct b2SensorQueryContext queryContext = {
			.world = world,
			.taskContext = taskContext,
			.sensor = sensor,
			.sensorShape = sensorShape,
			.transform = transform,
		};

		B2_ASSERT( sensorShape->sensorIndex == sensorIndex );
		b2AABB queryBounds = sensorShape->aabb;

		// 查询所有树
		b2DynamicTree_OverlapAABB( trees + 0, queryBounds, sensorShape->filter.maskBits, b2SensorQueryCallback, &queryContext );
		b2DynamicTree_OverlapAABB( trees + 1, queryBounds, sensorShape->filter.maskBits, b2SensorQueryCallback, &queryContext );
		b2DynamicTree_OverlapAABB( trees + 2, queryBounds, sensorShape->filter.maskBits, b2SensorQueryCallback, &queryContext );

		// 对重叠排序，以便找出开始和结束事件。
		qsort( sensor->overlaps2.data, sensor->overlaps2.count, sizeof( b2Visitor ), b2CompareVisitors );

		// 从 overlaps2（已排序）中去除重复。先前追加的命中事件可能导致重复。
		int uniqueCount = 0;
		int overlapCount = sensor->overlaps2.count;
		b2Visitor* overlapData = sensor->overlaps2.data;
		for ( int i = 0; i < overlapCount; ++i )
		{
			if ( uniqueCount == 0 || overlapData[i].shapeId != overlapData[uniqueCount - 1].shapeId )
			{
				overlapData[uniqueCount] = overlapData[i];
				uniqueCount += 1;
			}
		}
		sensor->overlaps2.count = uniqueCount;

		int count1 = sensor->overlaps1.count;
		int count2 = sensor->overlaps2.count;
		if ( count1 != count2 )
		{
			// 有变化
			b2SetBit( &taskContext->eventBits, sensorIndex );
		}
		else
		{
			for ( int i = 0; i < count1; ++i )
			{
				b2Visitor* s1 = sensor->overlaps1.data + i;
				b2Visitor* s2 = sensor->overlaps2.data + i;

				if ( s1->shapeId != s2->shapeId || s1->generation != s2->generation )
				{
					// 有变化
					b2SetBit( &taskContext->eventBits, sensorIndex );
					break;
				}
			}
		}
	}

}

void b2BeginSensors( b2World* world )
{
	int sensorCount = world->sensors.count;
	if ( sensorCount == 0 )
	{
		return;
	}

	B2_ASSERT( world->workerCount > 0 );

	for ( int i = 0; i < world->workerCount; ++i )
	{
		b2SetBitCountAndClear( &world->sensorTaskContexts.data[i].eventBits, sensorCount );
	}

	// 对传感器重叠做 parallel-for
	b2StageParallelFor(world,b2_taskSensors,b2SensorTask,sensorCount,16,world);
}

void b2EndSensors( b2World* world )
{
	int sensorCount = world->sensors.count;
	if ( sensorCount == 0 )
	{
		return;
	}


	b2BitSet* bitSet = &world->sensorTaskContexts.data[0].eventBits;
	for ( int i = 1; i < world->workerCount; ++i )
	{
		b2InPlaceUnion( bitSet, &world->sensorTaskContexts.data[i].eventBits );
	}

	// 遍历传感器位并发布事件
	// 处理传感器状态变化。遍历已置位的位
	uint64_t* bits = bitSet->bits;
	uint32_t blockCount = bitSet->blockCount;

	for ( uint32_t k = 0; k < blockCount; ++k )
	{
		uint64_t word = bits[k];
		while ( word != 0 )
		{
			uint32_t ctz = std::countr_zero( word );
			int sensorIndex = (int)( 64 * k + ctz );

			b2Sensor* sensor = b2Array_Get( world->sensors, sensorIndex );
			b2Shape* sensorShape = b2Array_Get( world->shapes, sensor->shapeId );
			b2ShapeId sensorId = { sensor->shapeId + 1, world->worldId, sensorShape->generation };

			int count1 = sensor->overlaps1.count;
			int count2 = sensor->overlaps2.count;
			const b2Visitor* refs1 = sensor->overlaps1.data;
			const b2Visitor* refs2 = sensor->overlaps2.data;

			// overlaps1 可含结束的重叠
			// overlaps2 可含开始的重叠
			int index1 = 0, index2 = 0;
			while ( index1 < count1 && index2 < count2 )
			{
				const b2Visitor* r1 = refs1 + index1;
				const b2Visitor* r2 = refs2 + index2;
				if ( r1->shapeId == r2->shapeId )
				{
					if ( r1->generation < r2->generation )
					{
						// 结束
						b2ShapeId visitorId = { r1->shapeId + 1, world->worldId, r1->generation };
						b2SensorEndTouchEvent event = {
							.sensorShapeId = sensorId,
							.visitorShapeId = visitorId,
						};
						b2Array_Push( world->sensorEndEvents[world->endEventArrayIndex], event );
						index1 += 1;
					}
					else if ( r1->generation > r2->generation )
					{
						// 开始
						b2ShapeId visitorId = { r2->shapeId + 1, world->worldId, r2->generation };
						b2SensorBeginTouchEvent event = { sensorId, visitorId };
						b2Array_Push( world->sensorBeginEvents, event );
						index2 += 1;
					}
					else
					{
						// 持续
						index1 += 1;
						index2 += 1;
					}
				}
				else if ( r1->shapeId < r2->shapeId )
				{
					// 结束
					b2ShapeId visitorId = { r1->shapeId + 1, world->worldId, r1->generation };
					b2SensorEndTouchEvent event = { sensorId, visitorId };
					b2Array_Push( world->sensorEndEvents[world->endEventArrayIndex], event );
					index1 += 1;
				}
				else
				{
					// 开始
					b2ShapeId visitorId = { r2->shapeId + 1, world->worldId, r2->generation };
					b2SensorBeginTouchEvent event = { sensorId, visitorId };
					b2Array_Push( world->sensorBeginEvents, event );
					index2 += 1;
				}
			}

			while ( index1 < count1 )
			{
				// 结束
				const b2Visitor* r1 = refs1 + index1;
				b2ShapeId visitorId = { r1->shapeId + 1, world->worldId, r1->generation };
				b2SensorEndTouchEvent event = { sensorId, visitorId };
				b2Array_Push( world->sensorEndEvents[world->endEventArrayIndex], event );
				index1 += 1;
			}

			while ( index2 < count2 )
			{
				// 开始
				const b2Visitor* r2 = refs2 + index2;
				b2ShapeId visitorId = { r2->shapeId + 1, world->worldId, r2->generation };
				b2SensorBeginTouchEvent event = { sensorId, visitorId };
				b2Array_Push( world->sensorBeginEvents, event );
				index2 += 1;
			}

			// 清除最低已置位
			word = word & ( word - 1 );
		}
	}

}

void b2DestroySensor( b2World* world, b2Shape* sensorShape )
{
	b2Sensor* sensor = b2Array_Get( world->sensors, sensorShape->sensorIndex );
	for ( int i = 0; i < sensor->overlaps2.count; ++i )
	{
		b2Visitor* ref = sensor->overlaps2.data + i;
		b2SensorEndTouchEvent event = {
			.sensorShapeId =
				{
					.index1 = sensorShape->id + 1,
					.world0 = world->worldId,
					.generation = sensorShape->generation,
				},
			.visitorShapeId =
				{
					.index1 = ref->shapeId + 1,
					.world0 = world->worldId,
					.generation = ref->generation,
				},
		};

		b2Array_Push( world->sensorEndEvents[world->endEventArrayIndex], event );
	}

	// 销毁传感器
	b2Array_Destroy( sensor->hits );
	b2Array_Destroy( sensor->overlaps1 );
	b2Array_Destroy( sensor->overlaps2 );

	int movedIndex = b2Array_RemoveSwap( world->sensors, sensorShape->sensorIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// 修正被移动的传感器
		b2Sensor* movedSensor = b2Array_Get( world->sensors, sensorShape->sensorIndex );
		b2Shape* otherSensorShape = b2Array_Get( world->shapes, movedSensor->shapeId );
		otherSensorShape->sensorIndex = sensorShape->sensorIndex;
	}
}

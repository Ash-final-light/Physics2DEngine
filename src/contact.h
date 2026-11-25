// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"
#include "box2d/types.h"

struct b2World;
struct b2Shape;
struct b2Body;

enum b2ContactFlags
{
	// 实体形状接触时设置。
	b2_contactTouchingFlag = 0x00000001,

	// 接触有命中事件
	b2_contactHitEventFlag = 0x00000002,

	// 此接触需要接触事件
	b2_contactEnableContactEvents = 0x00000004,

	b2_contactRecycleFlag = 0x00000008,

	// 形状接触时设置
	b2_simTouchingFlag = 0x00010000,

	// 此接触的 AABB 不再重叠
	b2_simDisjoint = 0x00020000,

	// 此接触开始接触
	b2_simStartedTouching = 0x00040000,

	// 此接触停止接触
	b2_simStoppedTouching = 0x00080000,

	// 此接触有命中事件
	b2_simEnableHitEvent = 0x00100000,

	// 此接触需要预求解事件
	b2_simEnablePreSolveEvents = 0x00200000,

	// 此接触有缓存的相对变换
	b2_simRelativeTransformValid = 0x00400000,
};

// 接触边用于把刚体和接触连成接触图：每个刚体是节点，每个接触是边。
// 接触边属于每个附着刚体维护的双向链表。
// 每个接触有两条接触边，分别对应两侧刚体。
typedef struct b2ContactEdge
{
	int bodyId;
	int prevKey;
	int nextKey;
} b2ContactEdge;

// 冷接触数据。用作持久句柄以及持久岛连通性。
typedef struct b2Contact
{
	b2ContactEdge edges[2];

	// 接触仅在处于接触时属于岛，否则为 B2_NULL_INDEX。
	int islandId;

	// 岛接触数组下标，用于 O(1) 交换删除。
	// 不在岛中时为 B2_NULL_INDEX。
	int islandIndex;

	// 存储在 b2World 中的仿真集合下标
	// 槽位空闲时为 B2_NULL_INDEX
	int setIndex;

	// 约束图颜色数组下标
	// 非接触或休眠接触为 B2_NULL_INDEX
	// 槽位空闲时为 B2_NULL_INDEX
	int colorIndex;

	// 集合或图颜色内的接触下标
	// 槽位空闲时为 B2_NULL_INDEX
	int localIndex;

	int shapeIdA;
	int shapeIdB;
	int contactId;

	// b2ContactFlags
	uint32_t flags;

	// 此槽位分配接触时单调递增
	// 用于检查无效的 b2ContactId
	uint32_t generation;
} b2Contact;



bool b2CanCollide( b2ShapeType typeA, b2ShapeType typeB );
void b2CreateContact( b2World* world, b2Shape* shapeA, b2Shape* shapeB );
void b2DestroyContact( b2World* world, b2Contact* contact );
void b2DestroyBodyContacts( b2World* world, b2Body* body );
void b2DestroyShapeContacts( b2World* world, const b2Shape* shape );
void b2UpdateBroadPhaseContacts( b2World* world );

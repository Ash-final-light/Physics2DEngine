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


typedef struct b2ContactSim
{
	int contactId;

	// 接触回收缓存。
	b2Rot cachedRotationA;
	b2Rot cachedRotationB;
	b2Transform cachedRelativePose;

#if B2_ENABLE_VALIDATION
	int bodyIdA;
	int bodyIdB;
#endif

	int encodedBodySimA;
	int encodedBodySimB;

	// b2ContactFlags
	uint32_t simFlags;

	int shapeIdA;
	int shapeIdB;

	float invMassA;
	float invIA;

	float invMassB;
	float invIB;

	b2Manifold manifold;

	// 混合摩擦和恢复系数
	float friction;
	float restitution;
	float rollingResistance;
	float tangentSpeed;

	b2SimplexCache cache;
} b2ContactSim;


b2ContactSim* b2GetContactSim( b2World* world, b2Contact* contact );
bool b2UpdateContact( b2World* world, b2ContactSim* contactSim, b2Shape* a, b2WorldTransform transformA,
                      b2Vec2 centerOffsetA, b2Shape* b, b2WorldTransform transformB, b2Vec2 centerOffsetB );
void b2UpdateContacts( b2World* world );
struct b2StepContext;
void b2BeginCollide(b2StepContext* context);
void b2EndCollide(b2StepContext* context);

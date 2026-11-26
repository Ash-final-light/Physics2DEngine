// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "bitset.h"
#include "contact.h"
#include "joint.h"
#include "container.h"
#include "box2d/constants.h"

#include "box2d/types.h"

typedef struct b2Body b2Body;
typedef struct b2Contact b2Contact;
typedef struct b2StepContext b2StepContext;
typedef struct b2World b2World;

// 容纳无法放入图着色上限的约束。当单个动态刚体接触许多其他刚体时发生。
#define B2_OVERFLOW_INDEX ( B2_GRAPH_COLOR_COUNT - 1 )

// 使涉及两个动态刚体的约束优先级低于涉及动态与静态刚体的约束。
// 减少因推穿导致的穿透。
#define B2_DYNAMIC_COLOR_COUNT ( B2_GRAPH_COLOR_COUNT - 4 )

typedef struct b2GraphColor
{
	// 此位集按下标 bodyId 索引，因此为包含静态刚体而偏大，
	// 但从不遍历这些位，也不使用位计数。
	// 溢出颜色上不使用此位集。
	//
	// Dirk 建议每个刚体用 uint64_t 跟踪图颜色成员关系，
	// 但我认为那会更难调试且更不灵活。用位集可轻松把图颜色数增到超过 64。
	// 验证见 b2CountSetBits 的用法。
	b2BitSet bodySet;

	// 缓存友好的数组
	b2DynamicArray<b2ContactSim> contactSims;
	b2DynamicArray<b2JointSim> jointSims;

	// Temporary scalar constraints, including the overflow color.
	struct b2ContactConstraint* overflowConstraints;
	void* wideConstraints;
	int wideConstraintCount;
} b2GraphColor;

typedef struct b2ConstraintGraph
{
	// 末尾包含溢出色
	b2GraphColor colors[B2_GRAPH_COLOR_COUNT];
} b2ConstraintGraph;

void b2CreateGraph( b2ConstraintGraph* graph, const b2Capacity* capacity );
void b2DestroyGraph( b2ConstraintGraph* graph );

void b2AddContactToGraph( b2World* world, b2ContactSim* contactSim, b2Contact* contact );
void b2RemoveContactFromGraph( b2World* world, int bodyIdA, int bodyIdB, int colorIndex, int localIndex );

b2JointSim* b2CreateJointInGraph(b2World* world,b2Joint* joint);
void b2AddJointToGraph(b2World* world,b2JointSim* sim,b2Joint* joint);
void b2RemoveJointFromGraph(b2World* world,int bodyIdA,int bodyIdB,int colorIndex,int localIndex);

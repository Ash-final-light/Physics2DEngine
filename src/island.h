// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "container.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct b2Contact b2Contact;
typedef struct b2World b2World;

// 缓存在岛中的接触数据，便于连续遍历。
// 避免 b2SplitIsland 并查集时访问 b2Contact。
typedef struct b2ContactLink
{
	int contactId;
	int bodyIdA;
	int bodyIdB;
} b2ContactLink;

b2DeclareArray( b2ContactLink );

// Persistent connected components of touching contacts.
// Static bodies can be contact endpoints but never join or bridge islands.
typedef struct b2Island
{
	// 存储在 b2World 中的求解器集合下标
	// 可为 B2_NULL_INDEX
	int setIndex;

	// 集合内的岛下标
	// 可为 B2_NULL_INDEX
	int localIndex;

	int islandId;

	// 记录从此岛移除了多少接触。
	// 用于判断岛是否适合拆分。
	int constraintRemoveCount;

	// 曾尝试用栈数组，但世界岛数组增长时数据指针会失效。
	b2Array( int ) bodies;

	// 属于此岛的接触。可连接到不在岛内的静态刚体。
	// 每条链接保存两个刚体 id，使 b2SplitIsland 的并查集遍历
	// 无需访问 b2Contact。
	b2Array( b2ContactLink ) contacts;

} b2Island;

b2DeclareArray( b2Island );

// 用于在求解器集合之间移动岛
typedef struct b2IslandSim
{
	int islandId;
} b2IslandSim;

b2DeclareArray( b2IslandSim );

b2Island* b2CreateIsland( b2World* world, int setIndex );
void b2DestroyIsland( b2World* world, int islandId );

// 接触开始产生接触点时，将其链入岛图
void b2LinkContact( b2World* world, b2Contact* contact );

// 接触不再有接触点时，从岛图中解除链接
void b2UnlinkContact( b2World* world, b2Contact* contact );


void b2SplitIsland( b2World* world, int baseId );
void b2SplitContactIslands( b2World* world );

void b2ValidateIsland( b2World* world, int islandId );

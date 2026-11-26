// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "island.h"

#include "body.h"
#include "contact.h"
#include "core.h"
#include "joint.h"
#include "world.h"
#include "solver_set.h"

#include <stddef.h>

b2Island* b2CreateIsland( b2World* world, int setIndex )
{
	B2_ASSERT( setIndex == b2_awakeSet || setIndex >= b2_firstSleepingSet );

	int islandId = b2AllocId( &world->islandIdPool );

	if ( islandId == world->islands.count )
	{
		b2Island emptyIsland = {};
		b2Array_Push( world->islands, emptyIsland );
	}
	else
	{
		B2_ASSERT( world->islands.data[islandId].setIndex == B2_NULL_INDEX );
	}

	b2SolverSet* set = b2Array_Get( world->solverSets, setIndex );

	b2Island* island = b2Array_Get( world->islands, islandId );
	island->setIndex = setIndex;
	island->localIndex = set->islandSims.count;
	island->islandId = islandId;
	b2Array_Create( island->bodies );
	b2Array_Create( island->contacts );
	b2Array_Create( island->joints );
	island->constraintRemoveCount = 0;

	b2IslandSim* islandSim = b2Array_Emplace( set->islandSims );
	islandSim->islandId = islandId;

	return island;
}

void b2DestroyIsland( b2World* world, int islandId )
{

	// 假定岛为空
	b2Island* island = b2Array_Get( world->islands, islandId );
	b2SolverSet* set = b2Array_Get( world->solverSets, island->setIndex );
	{
		int localIndex = island->localIndex;
		int lastIndex = set->islandSims.count - 1;
		B2_ASSERT( 0 <= localIndex && localIndex <= lastIndex );
		int moveIslandId = set->islandSims.data[lastIndex].islandId;
		set->islandSims.data[localIndex] = set->islandSims.data[lastIndex];
		world->islands.data[moveIslandId].localIndex = localIndex;
		set->islandSims.count -= 1;
	}

	// 释放岛和 id（保留岛代数）
	b2Array_Destroy( island->bodies );
	b2Array_Destroy( island->contacts );
	b2Array_Destroy( island->joints );
	island->constraintRemoveCount = 0;
	island->localIndex = B2_NULL_INDEX;
	island->islandId = B2_NULL_INDEX;
	island->setIndex = B2_NULL_INDEX;

	B2_VALIDATE( island->localIndex == B2_NULL_INDEX );

	b2FreeId( &world->islandIdPool, islandId );
}

static int b2MergeIslands( b2World* world, int islandIdA, int islandIdB )
{
	if ( islandIdA == islandIdB )
	{
		return islandIdA;
	}

	if ( islandIdA == B2_NULL_INDEX )
	{
		B2_ASSERT( islandIdB != B2_NULL_INDEX );
		return islandIdB;
	}

	if ( islandIdB == B2_NULL_INDEX )
	{
		B2_ASSERT( islandIdA != B2_NULL_INDEX );
		return islandIdA;
	}

	b2Island* smallIsland;
	b2Island* bigIsland;
	{
		b2Island* islandA = b2Array_Get( world->islands, islandIdA );
		b2Island* islandB = b2Array_Get( world->islands, islandIdB );

		// 保留最大岛以减少缓存未命中
		if ( islandA->bodies.count >= islandB->bodies.count )
		{
			bigIsland = islandA;
			smallIsland = islandB;
		}
		else
		{
			bigIsland = islandB;
			smallIsland = islandA;
		}
	}

	int bigIslandId = bigIsland->islandId;
	b2Array_Grow( bigIsland->bodies, bigIsland->bodies.count + smallIsland->bodies.count );

	// 将刚体从较小岛移到较大岛
	for ( int i = 0; i < smallIsland->bodies.count; ++i )
	{
		int bodyId = smallIsland->bodies.data[i];
		b2Body* body = b2Array_Get( world->bodies, bodyId );
		B2_VALIDATE( body->islandId == smallIsland->islandId );
		body->islandId = bigIslandId;
		body->islandIndex = bigIsland->bodies.count;
		b2Array_Push( bigIsland->bodies, bodyId );
	}

	// 将接触从较小岛迁到较大岛
	if ( smallIsland->contacts.count > 0 )
	{
		b2Array_Grow( bigIsland->contacts, bigIsland->contacts.count + smallIsland->contacts.count );

		for ( int i = 0; i < smallIsland->contacts.count; ++i )
		{
			b2ContactLink* link = smallIsland->contacts.data + i;
			b2Contact* contact = b2Array_Get( world->contacts, link->contactId );
			contact->islandId = bigIslandId;
			contact->islandIndex = bigIsland->contacts.count;
			b2Array_Push( bigIsland->contacts, *link );
		}
	}

	// 将关节从较小岛迁到较大岛
	if ( smallIsland->joints.count > 0 )
	{
		b2Array_Grow( bigIsland->joints, bigIsland->joints.count + smallIsland->joints.count );

		for ( int i = 0; i < smallIsland->joints.count; ++i )
		{
			b2JointLink* link = smallIsland->joints.data + i;
			b2Joint* joint = b2Array_Get( world->joints, link->jointId );
			joint->islandId = bigIslandId;
			joint->islandIndex = bigIsland->joints.count;
			b2Array_Push( bigIsland->joints, *link );
		}
	}

	// 累计已移除的约束
	bigIsland->constraintRemoveCount += smallIsland->constraintRemoveCount;

	b2DestroyIsland( world, smallIsland->islandId );

	b2ValidateIsland( world, bigIslandId );

	return bigIslandId;
}

static void b2AddContactToIsland( b2World* world, int islandId, b2Contact* contact )
{
	B2_ASSERT( contact->islandId == B2_NULL_INDEX );
	B2_ASSERT( contact->islandIndex == B2_NULL_INDEX );

	b2Island* island = b2Array_Get( world->islands, islandId );

	contact->islandId = islandId;
	contact->islandIndex = island->contacts.count;

	b2ContactLink link;
	link.contactId = contact->contactId;
	link.bodyIdA = contact->edges[0].bodyId;
	link.bodyIdB = contact->edges[1].bodyId;
	b2Array_Push( island->contacts, link );

	b2ValidateIsland( world, islandId );
}

// 将接触链入岛。
void b2LinkContact( b2World* world, b2Contact* contact )
{
	B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) != 0 );

	int bodyIdA = contact->edges[0].bodyId;
	int bodyIdB = contact->edges[1].bodyId;

	b2Body* bodyA = b2Array_Get( world->bodies, bodyIdA );
	b2Body* bodyB = b2Array_Get( world->bodies, bodyIdB );

	B2_ASSERT( bodyA->setIndex != b2_disabledSet && bodyB->setIndex != b2_disabledSet );
	B2_ASSERT( bodyA->setIndex != b2_staticSet || bodyB->setIndex != b2_staticSet );

	// 若 bodyA 已唤醒而 bodyB 休眠，则唤醒 bodyB
	if ( bodyA->setIndex == b2_awakeSet && bodyB->setIndex >= b2_firstSleepingSet )
	{
		b2WakeSolverSet( world, bodyB->setIndex );
	}

	// 若 bodyB 已唤醒而 bodyA 休眠，则唤醒 bodyA
	if ( bodyB->setIndex == b2_awakeSet && bodyA->setIndex >= b2_firstSleepingSet )
	{
		b2WakeSolverSet( world, bodyA->setIndex );
	}

	int islandIdA = bodyA->islandId;
	int islandIdB = bodyB->islandId;

	// 静态刚体的岛下标为空。
	B2_ASSERT( bodyA->setIndex != b2_staticSet || islandIdA == B2_NULL_INDEX );
	B2_ASSERT( bodyB->setIndex != b2_staticSet || islandIdB == B2_NULL_INDEX );
	B2_ASSERT( islandIdA != B2_NULL_INDEX || islandIdB != B2_NULL_INDEX );

	// 合并岛。会销毁其中一个岛。
	int finalIslandId = b2MergeIslands( world, islandIdA, islandIdB );

	// 将接触加入幸存的岛
	b2AddContactToIsland( world, finalIslandId, contact );
}

// 接触不再有接触点或接触被销毁时调用。
void b2UnlinkContact( b2World* world, b2Contact* contact )
{
	B2_ASSERT( contact->islandId != B2_NULL_INDEX );

	// 从岛中移除
	int islandId = contact->islandId;
	b2Island* island = b2Array_Get( world->islands, islandId );

	int removeIndex = contact->islandIndex;
	B2_ASSERT( 0 <= removeIndex && removeIndex < island->contacts.count );
	B2_ASSERT( island->contacts.data[removeIndex].contactId == contact->contactId );

	int movedIndex = b2Array_RemoveSwap( island->contacts, removeIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// 修正被交换到 removeIndex 的接触的 islandIndex
		b2ContactLink* movedLink = island->contacts.data + removeIndex;
		b2Contact* movedContact = b2Array_Get( world->contacts, movedLink->contactId );
		B2_ASSERT( movedContact->islandIndex == movedIndex );
		movedContact->islandIndex = removeIndex;
	}

	contact->islandId = B2_NULL_INDEX;
	contact->islandIndex = B2_NULL_INDEX;
	island->constraintRemoveCount += 1;

	b2ValidateIsland( world, islandId );
}

static void b2AddJointToIsland( b2World* world, int islandId, b2Joint* joint )
{
	B2_ASSERT( joint->islandId == B2_NULL_INDEX );
	B2_ASSERT( joint->islandIndex == B2_NULL_INDEX );

	b2Island* island = b2Array_Get( world->islands, islandId );

	joint->islandId = islandId;
	joint->islandIndex = island->joints.count;

	b2JointLink link;
	link.jointId = joint->jointId;
	link.bodyIdA = joint->edges[0].bodyId;
	link.bodyIdB = joint->edges[1].bodyId;
	b2Array_Push( island->joints, link );

	b2ValidateIsland( world, islandId );
}

void b2LinkJoint( b2World* world, b2Joint* joint )
{
	b2Body* bodyA = b2Array_Get( world->bodies, joint->edges[0].bodyId );
	b2Body* bodyB = b2Array_Get( world->bodies, joint->edges[1].bodyId );

	B2_ASSERT( bodyA->type == b2_dynamicBody || bodyB->type == b2_dynamicBody );

	if ( bodyA->setIndex == b2_awakeSet && bodyB->setIndex >= b2_firstSleepingSet )
	{
		b2WakeSolverSet( world, bodyB->setIndex );
	}
	else if ( bodyB->setIndex == b2_awakeSet && bodyA->setIndex >= b2_firstSleepingSet )
	{
		b2WakeSolverSet( world, bodyA->setIndex );
	}

	int islandIdA = bodyA->islandId;
	int islandIdB = bodyB->islandId;

	B2_ASSERT( islandIdA != B2_NULL_INDEX || islandIdB != B2_NULL_INDEX );

	// 合并岛。会销毁其中一个岛。
	int finalIslandId = b2MergeIslands( world, islandIdA, islandIdB );

	// 将关节加入幸存的岛
	b2AddJointToIsland( world, finalIslandId, joint );
}

void b2UnlinkJoint( b2World* world, b2Joint* joint )
{
	if ( joint->islandId == B2_NULL_INDEX )
	{
		return;
	}

	// 从岛中移除
	int islandId = joint->islandId;
	b2Island* island = b2Array_Get( world->islands, islandId );

	int removeIndex = joint->islandIndex;
	B2_ASSERT( 0 <= removeIndex && removeIndex < island->joints.count );
	B2_ASSERT( island->joints.data[removeIndex].jointId == joint->jointId );

	int movedIndex = b2Array_RemoveSwap( island->joints, removeIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		// 修正被交换到 removeIndex 的关节的 islandIndex
		b2JointLink* movedLink = island->joints.data + removeIndex;
		b2Joint* movedJoint = b2Array_Get( world->joints, movedLink->jointId );
		B2_ASSERT( movedJoint->islandIndex == movedIndex );
		movedJoint->islandIndex = removeIndex;
	}

	joint->islandId = B2_NULL_INDEX;
	joint->islandIndex = B2_NULL_INDEX;
	island->constraintRemoveCount += 1;

	b2ValidateIsland( world, islandId );
}

// 查找节点的父节点。使用路径减半以加速后续查询。
static inline int b2IslandFindParent( int* parents, int node )
{
	// 沿父链走到自身为父的节点（根）
	while ( parents[node] != node )
	{
		int grandParent = parents[parents[node]];
		parents[node] = grandParent;
		node = grandParent;
	}

	return node;
}

// 连接包含 node1 和 node2 的连通分量。
// 用秩保持树平衡。跟踪每个分量的接触和关节计数。
static inline void b2IslandUnion( int* parents, int* ranks, int node1, int node2, int* contactCounts, int* jointCounts )
{
	int root1 = b2IslandFindParent( parents, node1 );
	int root2 = b2IslandFindParent( parents, node2 );
	if ( root1 != root2 )
	{
		if ( ranks[root1] < ranks[root2] )
		{
			parents[root1] = root2;
			contactCounts[root2] += contactCounts[root1];
			jointCounts[root2] += jointCounts[root1];
		}
		else if ( ranks[root1] > ranks[root2] )
		{
			parents[root2] = root1;
			contactCounts[root1] += contactCounts[root2];
			jointCounts[root1] += jointCounts[root2];
		}
		else
		{
			parents[root2] = root1;
			ranks[root1] += 1;
			contactCounts[root1] += contactCounts[root2];
			jointCounts[root1] += jointCounts[root2];
		}
	}
}

// 使用并查集。
// https://en.wikipedia.org/wiki/Disjoint-set_data_structure
void b2SplitIsland( b2World* world, int baseId )
{
	b2Island* baseIsland = b2Array_Get( world->islands, baseId );
	B2_ASSERT( baseIsland->constraintRemoveCount > 0 );
	B2_ASSERT( baseIsland->setIndex == b2_awakeSet );

	b2ValidateIsland( world, baseId );

	// 在 b2CreateIsland 之前缓存基岛字段，因为可能重分配
	// world->islands 并使 baseIsland 指针失效。
	int baseBodyCount = baseIsland->bodies.count;
	int* baseBodyIds = baseIsland->bodies.data;
	int baseBodyCapacity = baseIsland->bodies.capacity;

	int baseContactCount = baseIsland->contacts.count;
	b2ContactLink* baseContacts = baseIsland->contacts.data;
	int baseContactCapacity = baseIsland->contacts.capacity;

	int baseJointCount = baseIsland->joints.count;
	b2JointLink* baseJoints = baseIsland->joints.data;
	int baseJointCapacity = baseIsland->joints.capacity;

	b2Stack* alloc = &world->stack;

	// 无需加锁，因为此任务运行时保证分配器不被占用。
	// 先分配 contactCounts 和 jointCounts，再分配 ranks，以便 ranks 可先释放（LIFO 竞技场）。
	int* parents = static_cast<int *>( b2StackAlloc( alloc, baseBodyCount * sizeof( int ), "parents" ) );
	int* contactCounts = static_cast<int *>( b2StackAlloc( alloc, baseBodyCount * sizeof( int ), "contact counts" ) );
	int* jointCounts = static_cast<int *>( b2StackAlloc( alloc, baseBodyCount * sizeof( int ), "joint counts" ) );
	int* ranks = static_cast<int *>( b2StackAlloc( alloc, baseBodyCount * sizeof( int ), "ranks" ) );
	for ( int i = 0; i < baseBodyCount; ++i )
	{
		parents[i] = i;
		ranks[i] = 0;
		contactCounts[i] = 0;
		jointCounts[i] = 0;
	}

	b2Body* bodies = world->bodies.data;

	// 对接触做并操作，跟踪每个分量的接触计数
	for ( int i = 0; i < baseContactCount; ++i )
	{
		int bodyIdA = baseContacts[i].bodyIdA;
		int bodyIdB = baseContacts[i].bodyIdB;
		B2_VALIDATE( 0 <= bodyIdA && bodyIdA < world->bodies.count );
		B2_VALIDATE( 0 <= bodyIdB && bodyIdB < world->bodies.count );
		b2Body* bodyA = bodies + bodyIdA;
		b2Body* bodyB = bodies + bodyIdB;
		int islandIndexA = bodyA->islandIndex;
		int islandIndexB = bodyB->islandIndex;

		// 只连接非静态刚体
		if ( islandIndexA != B2_NULL_INDEX && islandIndexB != B2_NULL_INDEX )
		{
			B2_VALIDATE( 0 <= islandIndexA && islandIndexA < baseBodyCount );
			B2_VALIDATE( 0 <= islandIndexB && islandIndexB < baseBodyCount );
			b2IslandUnion( parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts );
			int root = b2IslandFindParent( parents, islandIndexA );
			contactCounts[root] += 1;
		}
		else
		{
			int islandIndex = islandIndexA != B2_NULL_INDEX ? islandIndexA : islandIndexB;
			int root = b2IslandFindParent( parents, islandIndex );
			contactCounts[root] += 1;
		}
	}

	// 对关节做并操作，跟踪每个分量的关节计数
	for ( int i = 0; i < baseJointCount; ++i )
	{
		int bodyIdA = baseJoints[i].bodyIdA;
		int bodyIdB = baseJoints[i].bodyIdB;
		B2_VALIDATE( 0 <= bodyIdA && bodyIdA < world->bodies.count );
		B2_VALIDATE( 0 <= bodyIdB && bodyIdB < world->bodies.count );
		b2Body* bodyA = bodies + bodyIdA;
		b2Body* bodyB = bodies + bodyIdB;
		int islandIndexA = bodyA->islandIndex;
		int islandIndexB = bodyB->islandIndex;

		// 只连接非静态刚体
		if ( islandIndexA != B2_NULL_INDEX && islandIndexB != B2_NULL_INDEX )
		{
			B2_VALIDATE( 0 <= islandIndexA && islandIndexA < baseBodyCount );
			B2_VALIDATE( 0 <= islandIndexB && islandIndexB < baseBodyCount );
			b2IslandUnion( parents, ranks, islandIndexA, islandIndexB, contactCounts, jointCounts );
			int root = b2IslandFindParent( parents, islandIndexA );
			jointCounts[root] += 1;
		}
		else
		{
			int islandIndex = islandIndexA != B2_NULL_INDEX ? islandIndexA : islandIndexB;
			int root = b2IslandFindParent( parents, islandIndex );
			jointCounts[root] += 1;
		}
	}

	// ranks 已用完
	b2StackFree( alloc, ranks );
	ranks = NULL;

	// 压平所有父下标并统计连通分量。
	int componentCount = 0;
	for ( int i = 0; i < baseBodyCount; ++i )
	{
		parents[i] = b2IslandFindParent( parents, i );
		if ( parents[i] == i )
		{
			componentCount += 1;
		}
	}

	// 提前返回 — 岛仍完全连通，无需拆分。
	if ( componentCount == 1 )
	{
		baseIsland->constraintRemoveCount = 0;
		b2StackFree( alloc, jointCounts );
		b2StackFree( alloc, contactCounts );
		b2StackFree( alloc, parents );
		return;
	}

	// 从基岛拆下刚体/接触/关节数组，以免 b2DestroyIsland 释放它们
	baseIsland->bodies.data = NULL;
	baseIsland->bodies.count = 0;
	baseIsland->bodies.capacity = 0;

	baseIsland->contacts.data = NULL;
	baseIsland->contacts.count = 0;
	baseIsland->contacts.capacity = 0;

	baseIsland->joints.data = NULL;
	baseIsland->joints.count = 0;
	baseIsland->joints.capacity = 0;

	// 置空，防止后续代码误用。
	baseIsland = NULL;

	// 刚体下标到新岛下标的映射。仅根刚体有值。
	int* rootMap = static_cast<int *>( b2StackAlloc( alloc, baseBodyCount * sizeof( int ), "root map" ) );
	for ( int i = 0; i < baseBodyCount; ++i )
	{
		rootMap[i] = B2_NULL_INDEX;
	}

	int* componentBodyCounts = static_cast<int *>( b2StackAlloc( alloc, componentCount * sizeof( int ), "component body counts" ) );
	int* componentContactCounts = static_cast<int *>( b2StackAlloc( alloc, componentCount * sizeof( int ), "component contact counts" ) );
	int* componentJointCounts = static_cast<int *>( b2StackAlloc( alloc, componentCount * sizeof( int ), "component joint counts" ) );
	int islandCount = 0;

	// 为每个刚体找根刚体，按需创建岛。
	// 从根节点的累计计数提取每个分量的计数。
	for ( int i = 0; i < baseBodyCount; ++i )
	{
		int rootIndex = parents[i];
		if ( rootMap[rootIndex] == B2_NULL_INDEX )
		{
			rootMap[rootIndex] = islandCount;
			componentBodyCounts[islandCount] = 0;
			componentContactCounts[islandCount] = contactCounts[rootIndex];
			componentJointCounts[islandCount] = jointCounts[rootIndex];
			islandCount += 1;
		}

		componentBodyCounts[rootMap[rootIndex]] += 1;
	}

	B2_ASSERT( islandCount == componentCount );

	// 新岛下标到岛 id 的映射
	int* islandIds = static_cast<int *>( b2StackAlloc( alloc, islandCount * sizeof( int ), "island ids" ) );

	// 创建新岛并预留刚体/接触/关节数组
	for ( int i = 0; i < islandCount; ++i )
	{
		// 警告：这会使 baseIsland 指针失效
		b2Island* newIsland = b2CreateIsland( world, b2_awakeSet );
		islandIds[i] = newIsland->islandId;

		// 预留数组，避免反复扩容和 memcpy。
		b2Array_Reserve( newIsland->bodies, componentBodyCounts[i] );
		b2Array_Reserve( newIsland->contacts, componentContactCounts[i] );
		b2Array_Reserve( newIsland->joints, componentJointCounts[i] );
	}

	// 将刚体分配到新岛
	for ( int i = 0; i < baseBodyCount; ++i )
	{
		int bodyId = baseBodyIds[i];
		int root = b2IslandFindParent( parents, i );
		int newIslandId = islandIds[rootMap[root]];

		b2Body* body = b2Array_Get( world->bodies, bodyId );
		b2Island* newIsland = b2Array_Get( world->islands, newIslandId );

		body->islandId = newIslandId;
		body->islandIndex = newIsland->bodies.count;

		// 确保数组容量正确
		B2_VALIDATE( newIsland->bodies.count < newIsland->bodies.capacity );
		b2Array_Push( newIsland->bodies, bodyId );
	}

	// 将接触分配到其刚体所在的岛
	for ( int i = 0; i < baseContactCount; ++i )
	{
		b2ContactLink* link = baseContacts + i;
		b2Contact* contact = b2Array_Get( world->contacts, link->contactId );

		// 静态刚体没有岛 id。
		b2Body* bodyA = b2Array_Get( world->bodies, link->bodyIdA );
		b2Body* bodyB = b2Array_Get( world->bodies, link->bodyIdB );
		int targetIslandId = bodyA->islandId != B2_NULL_INDEX ? bodyA->islandId : bodyB->islandId;

		b2Island* targetIsland = b2Array_Get( world->islands, targetIslandId );
		contact->islandId = targetIslandId;
		contact->islandIndex = targetIsland->contacts.count;

		// 确保数组容量正确
		B2_VALIDATE( targetIsland->contacts.count < targetIsland->contacts.capacity );
		b2Array_Push( targetIsland->contacts, *link );
	}

	// 将关节分配到其刚体所在的岛
	for ( int i = 0; i < baseJointCount; ++i )
	{
		b2JointLink* link = baseJoints + i;
		b2Joint* joint = b2Array_Get( world->joints, link->jointId );

		// 静态刚体没有岛 id。
		b2Body* bodyA = b2Array_Get( world->bodies, link->bodyIdA );
		b2Body* bodyB = b2Array_Get( world->bodies, link->bodyIdB );
		int targetIslandId = bodyA->islandId != B2_NULL_INDEX ? bodyA->islandId : bodyB->islandId;

		b2Island* targetIsland = b2Array_Get( world->islands, targetIslandId );
		joint->islandId = targetIslandId;
		joint->islandIndex = targetIsland->joints.count;

		// 确保数组容量正确
		B2_VALIDATE( targetIsland->joints.count < targetIsland->joints.capacity );
		b2Array_Push( targetIsland->joints, *link );
	}

	// 销毁基岛
	b2DestroyIsland( world, baseId );

	// 手动释放拆下的数组
	b2Free( baseBodyIds, baseBodyCapacity * sizeof( int ) );
	b2Free( baseContacts, baseContactCapacity * sizeof( b2ContactLink ) );
	b2Free( baseJoints, baseJointCapacity * sizeof( b2JointLink ) );

	// 按 LIFO 顺序释放竞技场项
	b2StackFree( alloc, islandIds );
	b2StackFree( alloc, componentJointCounts );
	b2StackFree( alloc, componentContactCounts );
	b2StackFree( alloc, componentBodyCounts );
	b2StackFree( alloc, rootMap );
	b2StackFree( alloc, jointCounts );
	b2StackFree( alloc, contactCounts );
	b2StackFree( alloc, parents );
}

// 因部分接触和/或关节已移除而拆分岛。
// 在约束求解期间、岛未被触碰时调用。使用并查集且访问大量内存，可能较慢。
// 注意：连接到静态刚体的接触/关节必须属于某个岛，但不影响岛连通性
// 注意：静态刚体从不在岛内
// 注意：此任务无锁访问部分分配器，假定没有其他任务在操作这些数据结构。
void b2SplitContactIslands( b2World* world )
{
    // Capture existing IDs because splitting can grow/reuse the island array.
    b2DynamicArray<int> dirty = {};
    for ( int i = 0; i < world->islands.count; ++i )
    {
        const b2Island* island = world->islands.data + i;
        if ( island->setIndex == b2_awakeSet && island->constraintRemoveCount > 0 )
        {
            b2Array_Push( dirty, i );
        }
    }
    for ( int i = 0; i < dirty.count; ++i ) { b2SplitIsland( world, dirty.data[i] ); }
    b2Array_Destroy( dirty );
}

#if B2_ENABLE_VALIDATION
void b2ValidateIsland( b2World* world, int islandId )
{
	if ( islandId == B2_NULL_INDEX )
	{
		return;
	}

	b2Island* island = b2Array_Get( world->islands, islandId );
	B2_ASSERT( island->islandId == islandId );
	B2_ASSERT( island->setIndex != B2_NULL_INDEX );

	{
		B2_ASSERT( island->bodies.count > 0 );
		B2_ASSERT( island->bodies.count <= b2GetIdCount( &world->bodyIdPool ) );

		for ( int i = 0; i < island->bodies.count; ++i )
		{
			b2Body* body = b2Array_Get( world->bodies, island->bodies.data[i] );
			B2_ASSERT( body->islandId == islandId );
			B2_ASSERT( body->islandIndex == i );
			B2_ASSERT( body->setIndex == island->setIndex );
		}
	}

	if ( island->contacts.count > 0 )
	{
		B2_ASSERT( island->contacts.count <= b2GetIdCount( &world->contactIdPool ) );

		for ( int i = 0; i < island->contacts.count; ++i )
		{
			b2ContactLink* link = island->contacts.data + i;
			b2Contact* contact = b2Array_Get( world->contacts, link->contactId );
			B2_ASSERT( contact->setIndex == island->setIndex );
			B2_ASSERT( contact->islandId == islandId );
			B2_ASSERT( contact->islandIndex == i );
		}
	}

	if ( island->joints.count > 0 )
	{
		B2_ASSERT( island->joints.count <= b2GetIdCount( &world->jointIdPool ) );

		for ( int i = 0; i < island->joints.count; ++i )
		{
			b2JointLink* link = island->joints.data + i;
			b2Joint* joint = b2Array_Get( world->joints, link->jointId );
			B2_ASSERT( joint->setIndex == island->setIndex );
			B2_ASSERT( joint->islandId == islandId );
			B2_ASSERT( joint->islandIndex == i );
		}
	}
}

#else

void b2ValidateIsland( b2World* world, int islandId )
{
	(void)world;
	(void)islandId;
}
#endif

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver_set.h"
#include "world.h"

int b2CreateSolverSet( b2World* world )
{
    int id=b2AllocId(&world->solverSetIdPool);
    if(id==world->solverSets.count)
    {
        b2Array_Push(world->solverSets,b2SolverSet{});
    }
    world->solverSets.data[id]=b2SolverSet{};
    world->solverSets.data[id].setIndex=id;
    return id;
}

void b2CreateSolverSets( b2World* world )
{
    world->solverSetIdPool=b2CreateIdPool();
    b2Array_Create(world->solverSets);
    for(int i=0;i<b2_firstSleepingSet;++i)
    {
        int id=b2CreateSolverSet(world);
        B2_ASSERT(id==i);
    }
}

void b2DestroySolverSet( b2World* world, int index )
{
    b2SolverSet* set=world->solverSets.data+index;
    b2Array_Destroy(set->bodySims);
    b2Array_Destroy(set->bodyStates);
    b2Array_Destroy(set->contactSims);
    b2Array_Destroy(set->jointSims);
    b2Array_Destroy(set->islandSims);
    set->setIndex=B2_NULL_INDEX;
    b2FreeId(&world->solverSetIdPool,index);
}

void b2DestroySolverSets( b2World* world )
{
    for(int i=0;i<world->solverSets.count;++i)
    {
        if(world->solverSets.data[i].setIndex!=B2_NULL_INDEX)
        {
            b2DestroySolverSet(world,i);
        }
    }
    b2Array_Destroy(world->solverSets);
    b2DestroyIdPool(&world->solverSetIdPool);
}

b2BodySim* b2GetBodySim( b2World* world, b2Body* body )
{
    return world->solverSets.data[body->setIndex].bodySims.data+body->localIndex;
}

b2BodyState* b2GetBodyState( b2World* world, b2Body* body )
{
    return body->setIndex==b2_awakeSet ? world->solverSets.data[b2_awakeSet].bodyStates.data+body->localIndex : NULL;
}

int b2EncodeBodySimIndex( const b2Body* body )
{
    if(body->setIndex==b2_awakeSet) { return body->localIndex; }
    if(body->setIndex==b2_staticSet) { return -(body->localIndex+2); }
    return B2_NULL_INDEX;
}

void b2SyncBodyFlags( b2World* world, b2Body* body )
{
    b2GetBodySim(world,body)->flags=body->flags;
    b2BodyState* state=b2GetBodyState(world,body);
    if(state!=NULL) { state->flags=body->flags; }
}

void b2RefreshContactSets( b2World* world )
{
    for(int i=0;i<world->contacts.count;++i)
    {
        b2Contact* contact=world->contacts.data+i;
        if(contact->contactId==B2_NULL_INDEX) { continue; }
        b2Body* a=world->bodies.data+contact->edges[0].bodyId;
        b2Body* b=world->bodies.data+contact->edges[1].bodyId;
        bool touching=(contact->flags&b2_contactTouchingFlag)!=0;
        int targetIndex=(a->setIndex==b2_awakeSet || b->setIndex==b2_awakeSet)?b2_awakeSet:b2_disabledSet;
        if(touching && targetIndex!=b2_awakeSet)
        { targetIndex=a->setIndex>=b2_firstSleepingSet?a->setIndex:b->setIndex; }
        if(contact->colorIndex!=B2_NULL_INDEX) { B2_ASSERT(targetIndex==b2_awakeSet); }
        if(contact->colorIndex==B2_NULL_INDEX && contact->setIndex!=targetIndex)
        {
            b2SolverSet* source=world->solverSets.data+contact->setIndex;
            b2SolverSet* target=world->solverSets.data+targetIndex;
            b2ContactSim copy=source->contactSims.data[contact->localIndex];
            int oldIndex=contact->localIndex;
            int moved=b2Array_RemoveSwap(source->contactSims,oldIndex);
            if(moved!=B2_NULL_INDEX)
            {
                world->contacts.data[source->contactSims.data[oldIndex].contactId].localIndex=oldIndex;
            }
            contact->setIndex=targetIndex;
            contact->localIndex=target->contactSims.count;
            b2Array_Push(target->contactSims,copy);
        }
        if(touching && contact->islandId!=B2_NULL_INDEX && targetIndex==b2_awakeSet && contact->colorIndex==B2_NULL_INDEX)
        {
            b2SolverSet* source=world->solverSets.data+contact->setIndex;
            int oldIndex=contact->localIndex;
            b2ContactSim copy=source->contactSims.data[oldIndex];
            b2AddContactToGraph(world,&copy,contact);
            int moved=b2Array_RemoveSwap(source->contactSims,oldIndex);
            if(moved!=B2_NULL_INDEX) { world->contacts.data[source->contactSims.data[oldIndex].contactId].localIndex=oldIndex; }
        }
        b2ContactSim* sim=b2GetContactSim(world,contact);
        sim->encodedBodySimA=b2EncodeBodySimIndex(a);
        sim->encodedBodySimB=b2EncodeBodySimIndex(b);
        sim->invMassA=b2GetBodySim(world,a)->invMass;sim->invIA=b2GetBodySim(world,a)->invInertia;
        sim->invMassB=b2GetBodySim(world,b)->invMass;sim->invIB=b2GetBodySim(world,b)->invInertia;
    }
}

void b2RemoveBodyFromSolverSet( b2World* world, b2Body* body )
{
    int setIndex=body->setIndex;
    int index=body->localIndex;
    b2SolverSet* set=world->solverSets.data+setIndex;
    int moved=b2Array_RemoveSwap(set->bodySims,index);
    if(setIndex==b2_awakeSet) { b2Array_RemoveSwap(set->bodyStates,index); }
    if(moved!=B2_NULL_INDEX)
    {
        world->bodies.data[set->bodySims.data[index].bodyId].localIndex=index;
    }
    if(setIndex>=b2_firstSleepingSet && set->bodySims.count==0 && set->contactSims.count==0 && set->islandSims.count==0 && set->jointSims.count==0)
    {
        b2DestroySolverSet(world,setIndex);
    }
    b2RefreshContactSets(world);
}

void b2TransferBody( b2World* world, b2SolverSet* target, b2SolverSet* source, b2Body* body )
{
    if(target==source) { return; }
    b2BodySim sim=source->bodySims.data[body->localIndex];
    b2BodyState state=b2_identityBodyState;
    if(source->setIndex==b2_awakeSet) { state=source->bodyStates.data[body->localIndex]; }
    int oldSet=body->setIndex;
    int oldIndex=body->localIndex;
    int newIndex=target->bodySims.count;
    b2Array_Push(target->bodySims,sim);
    if(target->setIndex==b2_awakeSet)
    {
        state.flags=body->flags;
        b2Array_Push(target->bodyStates,state);
    }
    int moved=b2Array_RemoveSwap(source->bodySims,oldIndex);
    if(oldSet==b2_awakeSet) { b2Array_RemoveSwap(source->bodyStates,oldIndex); }
    if(moved!=B2_NULL_INDEX)
    {
        world->bodies.data[source->bodySims.data[oldIndex].bodyId].localIndex=oldIndex;
    }
    body->setIndex=target->setIndex;
    body->localIndex=newIndex;
    if(oldSet>=b2_firstSleepingSet && source->bodySims.count==0 && source->contactSims.count==0 && source->islandSims.count==0 && source->jointSims.count==0)
    {
        b2DestroySolverSet(world,oldSet);
    }
    b2RefreshContactSets(world);
}

bool b2WakeBody( b2World* world, b2Body* body )
{
    if(body->setIndex<b2_firstSleepingSet) { return false; }
    b2WakeSolverSet(world,body->setIndex);
    return true;
}

void b2WakeSolverSet( b2World* world, int setIndex )
{
    B2_ASSERT(setIndex>=b2_firstSleepingSet);
    b2SolverSet* set=world->solverSets.data+setIndex;
    b2SolverSet* awake=world->solverSets.data+b2_awakeSet;
    for(int i=0;i<set->bodySims.count;++i)
    {
        b2BodySim sim=set->bodySims.data[i];
        b2Body* body=world->bodies.data+sim.bodyId;
        body->setIndex=b2_awakeSet;body->localIndex=awake->bodySims.count;body->sleepTime=0.0f;
        b2Array_Push(awake->bodySims,sim);
        b2BodyState state=b2_identityBodyState;state.flags=body->flags;
        b2Array_Push(awake->bodyStates,state);
    }
    set->bodySims.count=0;
    for(int i=0;i<set->jointSims.count;++i)
    {
        b2JointSim* sim=set->jointSims.data+i;b2Joint* joint=world->joints.data+sim->jointId;
        b2AddJointToGraph(world,sim,joint);joint->setIndex=b2_awakeSet;
    }
    set->jointSims.count=0;

    for(int i=0;i<set->islandSims.count;++i)
    {
        b2IslandSim sim=set->islandSims.data[i];
        b2Island* island=world->islands.data+sim.islandId;
        island->setIndex=b2_awakeSet;island->localIndex=awake->islandSims.count;
        b2Array_Push(awake->islandSims,sim);
    }
    set->islandSims.count=0;
    b2RefreshContactSets(world);
    B2_ASSERT(set->contactSims.count==0);
    b2DestroySolverSet(world,setIndex);
}

void b2TrySleepIsland( b2World* world, int islandId )
{
	b2Island* island = b2Array_Get( world->islands, islandId );
	B2_ASSERT( island->setIndex == b2_awakeSet );

	// 岛有待处理的拆分且刚体多于一个时，不能使其休眠。
	if ( island->constraintRemoveCount > 0 && island->bodies.count > 1 )
	{
		return;
	}

	// 岛正在休眠
	// - 创建新的休眠求解器集合
	// - 将岛移到休眠求解器集合
	// - 识别应移到休眠求解器集合或禁用集合的非接触
	// - 移除旧岛
	// - 修正岛
	int sleepSetId = b2AllocId( &world->solverSetIdPool );
	if ( sleepSetId == world->solverSets.count )
	{
		b2SolverSet set = {};
		set.setIndex = B2_NULL_INDEX;
		b2Array_Push( world->solverSets, set );
	}

	b2SolverSet* sleepSet = b2Array_Get( world->solverSets, sleepSetId );
	*sleepSet = b2SolverSet{};

	// 创建休眠集合后再取唤醒集合，因为求解器集合数组可能已扩容
	b2SolverSet* awakeSet = b2Array_Get( world->solverSets, b2_awakeSet );
	B2_ASSERT( 0 <= island->localIndex && island->localIndex < awakeSet->islandSims.count );

	sleepSet->setIndex = sleepSetId;
	b2Array_CreateN( sleepSet->bodySims, island->bodies.count );
	b2Array_CreateN( sleepSet->contactSims, island->contacts.count );
	b2Array_CreateN(sleepSet->jointSims,island->joints.count);

	// 将唤醒刚体移到休眠集合
	// 这会打乱唤醒集合中的刚体
	{
		b2SolverSet* disabledSet = b2Array_Get( world->solverSets, b2_disabledSet );
		for ( int i = 0; i < island->bodies.count; ++i )
		{
			int bodyId = island->bodies.data[i];
			b2Body* body = b2Array_Get( world->bodies, bodyId );
			B2_ASSERT( body->setIndex == b2_awakeSet );
			B2_ASSERT( body->islandId == islandId );
			B2_ASSERT( body->islandIndex == i );

			body->flags &= ~b2_bodyTransientFlags;
            if(body->bodyMoveIndex!=B2_NULL_INDEX)
            {
                b2BodyMoveEvent* event=b2Array_Get(world->bodyMoveEvents,body->bodyMoveIndex);
                B2_ASSERT(event->bodyId.index1-1==bodyId && event->bodyId.generation==body->generation);
                event->fellAsleep=true;body->bodyMoveIndex=B2_NULL_INDEX;
            }

			int awakeBodyIndex = body->localIndex;
			b2BodySim* awakeSim = b2Array_Get( awakeSet->bodySims, awakeBodyIndex );
			awakeSim->flags &= ~b2_bodyTransientFlags;

			// 将刚体仿真移到休眠集合
			int sleepBodyIndex = sleepSet->bodySims.count;
			b2BodySim* sleepBodySim = b2Array_Emplace( sleepSet->bodySims );
			memcpy( sleepBodySim, awakeSim, sizeof( b2BodySim ) );

            int moved=b2Array_RemoveSwap(awakeSet->bodySims,awakeBodyIndex);
            if(moved!=B2_NULL_INDEX) { world->bodies.data[awakeSet->bodySims.data[awakeBodyIndex].bodyId].localIndex=awakeBodyIndex; }

			// 销毁状态，无需克隆。
			(void)b2Array_RemoveSwap( awakeSet->bodyStates, awakeBodyIndex );

			body->setIndex = sleepSetId;
			body->localIndex = sleepBodyIndex;

			// 将非接触移到禁用集合。
			// 非接触可能存在于休眠岛之间，所有权不明确。
			int contactKey = body->headContactKey;
			while ( contactKey != B2_NULL_INDEX )
			{
				int contactId = contactKey >> 1;
				int edgeIndex = contactKey & 1;

				b2Contact* contact = b2Array_Get( world->contacts, contactId );

				B2_ASSERT( contact->setIndex == b2_awakeSet );

				contactKey = contact->edges[edgeIndex].nextKey;

				if ( contact->colorIndex != B2_NULL_INDEX )
				{
					// 接触处于接触状态，将单独移动
					B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) != 0 );
					continue;
				}

				int localIndex = contact->localIndex;
				b2ContactSim* contactSim = b2Array_Get( awakeSet->contactSims, localIndex );

				if ( edgeIndex == 0 )
				{
					contactSim->encodedBodySimA = B2_NULL_INDEX;
				}
				else
				{
					contactSim->encodedBodySimB = B2_NULL_INDEX;
				}

				// 另一刚体可能仍处于唤醒，它仍可能进入休眠，届时由它负责
				// 将此接触移到禁用集合。
				int otherEdgeIndex = edgeIndex ^ 1;
				int otherBodyId = contact->edges[otherEdgeIndex].bodyId;
				b2Body* otherBody = b2Array_Get( world->bodies, otherBodyId );
				if ( otherBody->setIndex == b2_awakeSet )
				{
					continue;
				}

				B2_ASSERT( contactSim->manifold.pointCount == 0 );
				B2_ASSERT( ( contact->flags & b2_contactTouchingFlag ) == 0 );

				// 将非接触移到禁用集合
				contact->setIndex = b2_disabledSet;
				contact->localIndex = disabledSet->contactSims.count;
				b2ContactSim* disabledContactSim = b2Array_Emplace( disabledSet->contactSims );
				memcpy( disabledContactSim, contactSim, sizeof( b2ContactSim ) );

				int movedLocalIndex = b2Array_RemoveSwap( awakeSet->contactSims, localIndex );
				if ( movedLocalIndex != B2_NULL_INDEX )
				{
					// 修正被移动的元素
					b2ContactSim* movedContactSim = awakeSet->contactSims.data + localIndex;
					b2Contact* movedContact = b2Array_Get( world->contacts, movedContactSim->contactId );
					B2_ASSERT( movedContact->localIndex == movedLocalIndex );
					movedContact->localIndex = localIndex;
				}
			}
		}
	}

	// 移动接触
	// 这会打乱唤醒集合中的接触
	{
		for ( int i = 0; i < island->contacts.count; ++i )
		{
			b2ContactLink* link = island->contacts.data + i;
			b2Contact* contact = b2Array_Get( world->contacts, link->contactId );
			B2_ASSERT( contact->setIndex == b2_awakeSet );
			B2_ASSERT( contact->islandId == islandId );
			int colorIndex = contact->colorIndex;
			B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );

			b2GraphColor* color = world->constraintGraph.colors + colorIndex;

			// 从与此约束关联的图着色中移除刚体
			if ( colorIndex != B2_OVERFLOW_INDEX )
			{
				// 可能清除静态刚体的一位，但没有影响
				b2ClearBit( &color->bodySet, contact->edges[0].bodyId );
				b2ClearBit( &color->bodySet, contact->edges[1].bodyId );
			}

			int localIndex = contact->localIndex;
			b2ContactSim* awakeContactSim = b2Array_Get( color->contactSims, localIndex );

			int sleepContactIndex = sleepSet->contactSims.count;
			b2ContactSim* sleepContactSim = b2Array_Emplace( sleepSet->contactSims );
			memcpy( sleepContactSim, awakeContactSim, sizeof( b2ContactSim ) );

			sleepContactSim->encodedBodySimA = b2SleepBodySimIndex( sleepContactSim->encodedBodySimA );
			sleepContactSim->encodedBodySimB = b2SleepBodySimIndex( sleepContactSim->encodedBodySimB );

			int movedLocalIndex = b2Array_RemoveSwap( color->contactSims, localIndex );
			if ( movedLocalIndex != B2_NULL_INDEX )
			{
				// 修正被移动的元素
				b2ContactSim* movedContactSim = color->contactSims.data + localIndex;
				b2Contact* movedContact = b2Array_Get( world->contacts, movedContactSim->contactId );
				B2_ASSERT( movedContact->localIndex == movedLocalIndex );
				movedContact->localIndex = localIndex;
			}

			contact->setIndex = sleepSetId;
			contact->colorIndex = B2_NULL_INDEX;
			contact->localIndex = sleepContactIndex;
		}
	}

	// 移动关节
	// 这会打乱唤醒集合中的关节
	{
		for ( int i = 0; i < island->joints.count; ++i )
		{
			b2JointLink* link = island->joints.data + i;
			b2Joint* joint = b2Array_Get( world->joints, link->jointId );
			B2_ASSERT( joint->setIndex == b2_awakeSet );
			B2_ASSERT( joint->islandId == islandId );
			int colorIndex = joint->colorIndex;
			int localIndex = joint->localIndex;

			B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );

			b2GraphColor* color = world->constraintGraph.colors + colorIndex;

			b2JointSim* awakeJointSim = b2Array_Get( color->jointSims, localIndex );

			if ( colorIndex != B2_OVERFLOW_INDEX )
			{
				// 可能清除静态刚体的一位，但没有影响
				b2ClearBit( &color->bodySet, joint->edges[0].bodyId );
				b2ClearBit( &color->bodySet, joint->edges[1].bodyId );
			}

			int sleepJointIndex = sleepSet->jointSims.count;
			b2JointSim* sleepJointSim = b2Array_Emplace( sleepSet->jointSims );
			memcpy( sleepJointSim, awakeJointSim, sizeof( b2JointSim ) );

			int movedIndex = b2Array_RemoveSwap( color->jointSims, localIndex );
			if ( movedIndex != B2_NULL_INDEX )
			{
				// 修正被移动的元素
				b2JointSim* movedJointSim = color->jointSims.data + localIndex;
				int movedId = movedJointSim->jointId;
				b2Joint* movedJoint = b2Array_Get( world->joints, movedId );
				B2_ASSERT( movedJoint->localIndex == movedIndex );
				movedJoint->localIndex = localIndex;
			}

			joint->setIndex = sleepSetId;
			joint->colorIndex = B2_NULL_INDEX;
			joint->localIndex = sleepJointIndex;
		}
	}

	// 移动岛结构
	{
		B2_ASSERT( island->setIndex == b2_awakeSet );

		int islandIndex = island->localIndex;
		b2IslandSim* sleepIsland = b2Array_Emplace( sleepSet->islandSims );
		sleepIsland->islandId = islandId;

		int movedIslandIndex = b2Array_RemoveSwap( awakeSet->islandSims, islandIndex );
		if ( movedIslandIndex != B2_NULL_INDEX )
		{
			// 修正被移动元素的索引
			b2IslandSim* movedIslandSim = awakeSet->islandSims.data + islandIndex;
			int movedIslandId = movedIslandSim->islandId;
			b2Island* movedIsland = b2Array_Get( world->islands, movedIslandId );
			B2_ASSERT( movedIsland->localIndex == movedIslandIndex );
			movedIsland->localIndex = islandIndex;
		}

		island->setIndex = sleepSetId;
		island->localIndex = 0;
	}

	b2RefreshContactSets( world );
}

void b2MergeSolverSets( b2World* world, int setId1, int setId2 )
{
	B2_ASSERT( setId1 >= b2_firstSleepingSet );
	B2_ASSERT( setId2 >= b2_firstSleepingSet );
	b2SolverSet* set1 = b2Array_Get( world->solverSets, setId1 );
	b2SolverSet* set2 = b2Array_Get( world->solverSets, setId2 );

	// 移动尽量少的刚体
	if ( set1->bodySims.count < set2->bodySims.count )
	{
		b2SolverSet* tempSet = set1;
		set1 = set2;
		set2 = tempSet;

		int tempId = setId1;
		setId1 = setId2;
		setId2 = tempId;
	}

	// 转移刚体
	{
		b2Body* bodies = world->bodies.data;
		int bodyCount = set2->bodySims.count;
		for ( int i = 0; i < bodyCount; ++i )
		{
			b2BodySim* simSrc = set2->bodySims.data + i;

			b2Body* body = bodies + simSrc->bodyId;
			B2_ASSERT( body->setIndex == setId2 );
			body->setIndex = setId1;
			body->localIndex = set1->bodySims.count;

			b2BodySim* simDst = b2Array_Emplace( set1->bodySims );
			memcpy( simDst, simSrc, sizeof( b2BodySim ) );
		}
	}

	// 转移接触
	{
		int contactCount = set2->contactSims.count;
		for ( int i = 0; i < contactCount; ++i )
		{
			b2ContactSim* contactSrc = set2->contactSims.data + i;

			b2Contact* contact = b2Array_Get( world->contacts, contactSrc->contactId );
			B2_ASSERT( contact->setIndex == setId2 );
			contact->setIndex = setId1;
			contact->localIndex = set1->contactSims.count;

			b2ContactSim* contactDst = b2Array_Emplace( set1->contactSims );
			memcpy( contactDst, contactSrc, sizeof( b2ContactSim ) );
		}
	}

	// 转移关节
	{
		int jointCount = set2->jointSims.count;
		for ( int i = 0; i < jointCount; ++i )
		{
			b2JointSim* jointSrc = set2->jointSims.data + i;

			b2Joint* joint = b2Array_Get( world->joints, jointSrc->jointId );
			B2_ASSERT( joint->setIndex == setId2 );
			joint->setIndex = setId1;
			joint->localIndex = set1->jointSims.count;

			b2JointSim* jointDst = b2Array_Emplace( set1->jointSims );
			memcpy( jointDst, jointSrc, sizeof( b2JointSim ) );
		}
	}

	// 转移岛
	{
		int islandCount = set2->islandSims.count;
		for ( int i = 0; i < islandCount; ++i )
		{
			b2IslandSim* islandSrc = set2->islandSims.data + i;
			int islandId = islandSrc->islandId;

			b2Island* island = b2Array_Get( world->islands, islandId );
			island->setIndex = setId1;
			island->localIndex = set1->islandSims.count;

			b2IslandSim* islandDst = b2Array_Emplace( set1->islandSims );
			memcpy( islandDst, islandSrc, sizeof( b2IslandSim ) );
		}
	}

	// 销毁被合并的集合
	b2DestroySolverSet( world, setId2 );


}

void b2TransferJoint( b2World* world, b2SolverSet* targetSet, b2SolverSet* sourceSet, b2Joint* joint )
{
	if ( targetSet == sourceSet )
	{
		return;
	}

	int localIndex = joint->localIndex;
	int colorIndex = joint->colorIndex;

	// 取源。
	b2JointSim* sourceSim;
	if ( sourceSet->setIndex == b2_awakeSet )
	{
		B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );
		b2GraphColor* color = world->constraintGraph.colors + colorIndex;

		sourceSim = b2Array_Get( color->jointSims, localIndex );
	}
	else
	{
		B2_ASSERT( colorIndex == B2_NULL_INDEX );
		sourceSim = b2Array_Get( sourceSet->jointSims, localIndex );
	}

	// 创建目标并拷贝。修正关节。
	if ( targetSet->setIndex == b2_awakeSet )
	{
		b2AddJointToGraph( world, sourceSim, joint );
		joint->setIndex = b2_awakeSet;
	}
	else
	{
		joint->setIndex = targetSet->setIndex;
		joint->localIndex = targetSet->jointSims.count;
		joint->colorIndex = B2_NULL_INDEX;

		b2JointSim* targetSim = b2Array_Emplace( targetSet->jointSims );
		memcpy( targetSim, sourceSim, sizeof( b2JointSim ) );
	}

	// 销毁源。
	if ( sourceSet->setIndex == b2_awakeSet )
	{
		b2RemoveJointFromGraph( world, joint->edges[0].bodyId, joint->edges[1].bodyId, colorIndex, localIndex );
	}
	else
	{
		int movedIndex = b2Array_RemoveSwap( sourceSet->jointSims, localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			// 修正被交换的元素
			b2JointSim* movedJointSim = sourceSet->jointSims.data + localIndex;
			int movedId = movedJointSim->jointId;
			b2Joint* movedJoint = b2Array_Get( world->joints, movedId );
			movedJoint->localIndex = localIndex;
		}
	}
}

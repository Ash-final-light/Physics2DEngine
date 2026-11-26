// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "joint.h"
#include "body.h"
#include "world.h"
#include <cstring>

b2Joint* b2GetJointFullId(b2World* world,b2JointId id)
{
 if(world==nullptr || !world->inUse || world->worldId!=id.world0 || id.index1<1 || id.index1>world->joints.count) { return nullptr; }
 b2Joint* joint=world->joints.data+id.index1-1;
 return joint->jointId==id.index1-1 && joint->generation==id.generation?joint:nullptr;
}
bool b2Joint_IsValid(b2JointId id)
{
 return id.world0<B2_MAX_WORLDS && b2GetJointFullId(b2_worlds+id.world0,id)!=nullptr;
}
b2JointSim* b2GetJointSim( b2World* world, b2Joint* joint )
{
	if ( joint->setIndex == b2_awakeSet )
	{
		B2_ASSERT( 0 <= joint->colorIndex && joint->colorIndex < B2_GRAPH_COLOR_COUNT );
		b2GraphColor* color = world->constraintGraph.colors + joint->colorIndex;
		return b2Array_Get( color->jointSims, joint->localIndex );
	}

	b2SolverSet* set = b2Array_Get( world->solverSets, joint->setIndex );
	return b2Array_Get( set->jointSims, joint->localIndex );
}


static b2JointDef b2DefaultJointDef(void);
static void b2DestroyContactsBetweenBodies( b2World* world, b2Body* bodyA, b2Body* bodyB )
{
	int contactKey;
	int otherBodyId;

	// 使用两个接触列表中较小的一个
	if ( bodyA->contactCount < bodyB->contactCount )
	{
		contactKey = bodyA->headContactKey;
		otherBodyId = bodyB->id;
	}
	else
	{
		contactKey = bodyB->headContactKey;
		otherBodyId = bodyA->id;
	}

	// 销毁接触
	while ( contactKey != B2_NULL_INDEX )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2Contact* contact = b2Array_Get( world->contacts, contactId );
		contactKey = contact->edges[edgeIndex].nextKey;

		int otherEdgeIndex = edgeIndex ^ 1;
		if ( contact->edges[otherEdgeIndex].bodyId == otherBodyId )
		{
			// 小心，这会从当前双向链表中移除接触
			b2DestroyContact( world, contact );
		}
	}


}

typedef struct b2JointPair
{
	b2Joint* joint;
	b2JointSim* jointSim;
} b2JointPair;

static b2JointPair b2CreateJoint( b2World* world, const b2JointDef* def, b2JointType type )
{
	B2_ASSERT( b2IsValidTransform( def->localFrameA ) );
	B2_ASSERT( b2IsValidTransform( def->localFrameB ) );
	B2_ASSERT( world->worldId == def->bodyIdA.world0 );
	B2_ASSERT( world->worldId == def->bodyIdB.world0 );
	B2_ASSERT( (def->bodyIdA.index1 == def->bodyIdB.index1) == false );

	b2Body* bodyA = b2GetBodyFullId( world, def->bodyIdA );
	b2Body* bodyB = b2GetBodyFullId( world, def->bodyIdB );

	// 若关节禁止碰撞，则销毁附着刚体之间的所有接触。
	// 先做此事，以免后续代码扰动世界数组。
	if ( def->collideConnected == false )
	{
		b2DestroyContactsBetweenBodies( world, bodyA, bodyB );
	}

	int bodyIdA = bodyA->id;
	int bodyIdB = bodyB->id;
	int maxSetIndex = b2MaxInt( bodyA->setIndex, bodyB->setIndex );

	// 创建关节 id 和关节
	int jointId = b2AllocId( &world->jointIdPool );
	if ( jointId == world->joints.count )
	{
		b2Array_Push( world->joints, b2Joint{} );
	}

	b2Joint* joint = b2Array_Get( world->joints, jointId );
	joint->jointId = jointId;
	joint->userData = def->userData;
	joint->generation += 1;
	joint->setIndex = B2_NULL_INDEX;
	joint->colorIndex = B2_NULL_INDEX;
	joint->localIndex = B2_NULL_INDEX;
	joint->islandId = B2_NULL_INDEX;
	joint->islandIndex = B2_NULL_INDEX;
	joint->drawScale = def->drawScale;
	joint->type = type;
	joint->collideConnected = def->collideConnected;

	// 刚体 A 上的双向链表
	joint->edges[0].bodyId = bodyIdA;
	joint->edges[0].prevKey = B2_NULL_INDEX;
	joint->edges[0].nextKey = bodyA->headJointKey;

	int keyA = ( jointId << 1 ) | 0;
	if ( bodyA->headJointKey != B2_NULL_INDEX )
	{
		b2Joint* jointA = b2Array_Get( world->joints, bodyA->headJointKey >> 1 );
		b2JointEdge* edgeA = jointA->edges + ( bodyA->headJointKey & 1 );
		edgeA->prevKey = keyA;
	}
	bodyA->headJointKey = keyA;
	bodyA->jointCount += 1;

	// 刚体 B 上的双向链表
	joint->edges[1].bodyId = bodyIdB;
	joint->edges[1].prevKey = B2_NULL_INDEX;
	joint->edges[1].nextKey = bodyB->headJointKey;

	int keyB = ( jointId << 1 ) | 1;
	if ( bodyB->headJointKey != B2_NULL_INDEX )
	{
		b2Joint* jointB = b2Array_Get( world->joints, bodyB->headJointKey >> 1 );
		b2JointEdge* edgeB = jointB->edges + ( bodyB->headJointKey & 1 );
		edgeB->prevKey = keyB;
	}
	bodyB->headJointKey = keyB;
	bodyB->jointCount += 1;

	b2JointSim* jointSim;

	if ( bodyA->setIndex == b2_disabledSet || bodyB->setIndex == b2_disabledSet )
	{
		// 若任一侧刚体禁用，则在禁用集合中创建
		b2SolverSet* set = b2Array_Get( world->solverSets, b2_disabledSet );
		joint->setIndex = b2_disabledSet;
		joint->localIndex = set->jointSims.count;

		jointSim = b2Array_Emplace( set->jointSims );
		memset( jointSim, 0, sizeof( b2JointSim ) );

		jointSim->jointId = jointId;
		jointSim->bodyIdA = bodyIdA;
		jointSim->bodyIdB = bodyIdB;
	}
	else if ( bodyA->type != b2_dynamicBody && bodyB->type != b2_dynamicBody )
	{
		// 关节未附着到动态刚体
		b2SolverSet* set = b2Array_Get( world->solverSets, b2_staticSet );
		joint->setIndex = b2_staticSet;
		joint->localIndex = set->jointSims.count;

		jointSim = b2Array_Emplace( set->jointSims );
		memset( jointSim, 0, sizeof( b2JointSim ) );

		jointSim->jointId = jointId;
		jointSim->bodyIdA = bodyIdA;
		jointSim->bodyIdB = bodyIdB;
	}
	else if ( bodyA->setIndex == b2_awakeSet || bodyB->setIndex == b2_awakeSet )
	{
		// 若任一侧刚体休眠则唤醒
		if ( maxSetIndex >= b2_firstSleepingSet )
		{
			b2WakeSolverSet( world, maxSetIndex );
		}

		joint->setIndex = b2_awakeSet;

		jointSim = b2CreateJointInGraph( world, joint );
		jointSim->jointId = jointId;
		jointSim->bodyIdA = bodyIdA;
		jointSim->bodyIdB = bodyIdB;
	}
	else
	{
		// 关节连接休眠和/或静态刚体
		B2_ASSERT( bodyA->setIndex >= b2_firstSleepingSet || bodyB->setIndex >= b2_firstSleepingSet );
		B2_ASSERT( bodyA->setIndex != b2_staticSet || bodyB->setIndex != b2_staticSet );

		// 关节应进入休眠集合（非静态集合）
		int setIndex = maxSetIndex;

		b2SolverSet* set = b2Array_Get( world->solverSets, setIndex );
		joint->setIndex = setIndex;
		joint->localIndex = set->jointSims.count;

		jointSim = b2Array_Emplace( set->jointSims );
		memset( jointSim, 0, sizeof( b2JointSim ) );

		// 必须设置这些以配合下面的合并
		jointSim->jointId = jointId;
		jointSim->bodyIdA = bodyIdA;
		jointSim->bodyIdB = bodyIdB;

		if ( bodyA->setIndex != bodyB->setIndex && bodyA->setIndex >= b2_firstSleepingSet &&
			 bodyB->setIndex >= b2_firstSleepingSet )
		{
			// 合并休眠集合
			b2MergeSolverSets( world, bodyA->setIndex, bodyB->setIndex );
			B2_ASSERT( bodyA->setIndex == bodyB->setIndex );

			// 修正可能无效的集合下标
			setIndex = bodyA->setIndex;

			b2SolverSet* mergedSet = b2Array_Get( world->solverSets, setIndex );

			// 小心！集合合并使关节仿真指针失效。
			jointSim = b2Array_Get( mergedSet->jointSims, joint->localIndex );
		}

		B2_ASSERT( joint->setIndex == setIndex );
	}

	jointSim->localFrameA = def->localFrameA;
	jointSim->localFrameB = def->localFrameB;
	jointSim->type = type;
	jointSim->constraintHertz = def->constraintHertz;
	jointSim->constraintDampingRatio = def->constraintDampingRatio;
	jointSim->constraintSoftness = b2Softness{
		.biasRate = 0.0f,
		.massScale = 1.0f,
		.impulseScale = 0.0f,
	};



	B2_ASSERT( jointSim->jointId == jointId );
	B2_ASSERT( jointSim->bodyIdA == bodyIdA );
	B2_ASSERT( jointSim->bodyIdB == bodyIdB );

	if ( joint->setIndex > b2_disabledSet )
	{
		// 将边加入岛图
		b2LinkJoint( world, joint );
	}



	return b2JointPair{ joint, jointSim };
}

b2JointId b2CreateJointRecord(b2World* world,b2BodyId a,b2BodyId b)
{
 if(world==nullptr || world->locked || a.world0!=world->worldId || b.world0!=world->worldId || a.index1==b.index1 || !b2Body_IsValid(a) || !b2Body_IsValid(b)) { return b2_nullJointId; }
 b2JointDef def=b2DefaultJointDef();def.bodyIdA=a;def.bodyIdB=b;
 b2JointPair pair=b2CreateJoint(world,&def,b2_distanceJoint);
 pair.jointSim->distanceJoint.length=1.0f;
 return b2JointId{pair.joint->jointId+1,world->worldId,pair.joint->generation};
}
void b2DestroyJointInternal( b2World* world, b2Joint* joint )
{
	int jointId = joint->jointId;

	b2JointEdge* edgeA = joint->edges + 0;
	b2JointEdge* edgeB = joint->edges + 1;

	int idA = edgeA->bodyId;
	int idB = edgeB->bodyId;
	b2Body* bodyA = b2Array_Get( world->bodies, idA );
	b2Body* bodyB = b2Array_Get( world->bodies, idB );

	// 从刚体 A 移除
	if ( edgeA->prevKey != B2_NULL_INDEX )
	{
		b2Joint* prevJoint = b2Array_Get( world->joints, edgeA->prevKey >> 1 );
		b2JointEdge* prevEdge = prevJoint->edges + ( edgeA->prevKey & 1 );
		prevEdge->nextKey = edgeA->nextKey;
	}

	if ( edgeA->nextKey != B2_NULL_INDEX )
	{
		b2Joint* nextJoint = b2Array_Get( world->joints, edgeA->nextKey >> 1 );
		b2JointEdge* nextEdge = nextJoint->edges + ( edgeA->nextKey & 1 );
		nextEdge->prevKey = edgeA->prevKey;
	}

	int edgeKeyA = ( jointId << 1 ) | 0;
	if ( bodyA->headJointKey == edgeKeyA )
	{
		bodyA->headJointKey = edgeA->nextKey;
	}

	bodyA->jointCount -= 1;

	// 从刚体 B 移除
	if ( edgeB->prevKey != B2_NULL_INDEX )
	{
		b2Joint* prevJoint = b2Array_Get( world->joints, edgeB->prevKey >> 1 );
		b2JointEdge* prevEdge = prevJoint->edges + ( edgeB->prevKey & 1 );
		prevEdge->nextKey = edgeB->nextKey;
	}

	if ( edgeB->nextKey != B2_NULL_INDEX )
	{
		b2Joint* nextJoint = b2Array_Get( world->joints, edgeB->nextKey >> 1 );
		b2JointEdge* nextEdge = nextJoint->edges + ( edgeB->nextKey & 1 );
		nextEdge->prevKey = edgeB->prevKey;
	}

	int edgeKeyB = ( jointId << 1 ) | 1;
	if ( bodyB->headJointKey == edgeKeyB )
	{
		bodyB->headJointKey = edgeB->nextKey;
	}

	bodyB->jointCount -= 1;

	if ( joint->islandId != B2_NULL_INDEX )
	{
		B2_ASSERT( joint->setIndex > b2_disabledSet );
		b2UnlinkJoint( world, joint );
	}
	else
	{
		B2_ASSERT( joint->setIndex <= b2_disabledSet );
	}

	// 从拥有它的求解器集合中移除关节
	int setIndex = joint->setIndex;
	int localIndex = joint->localIndex;

	if ( setIndex == b2_awakeSet )
	{
		b2RemoveJointFromGraph( world, joint->edges[0].bodyId, joint->edges[1].bodyId, joint->colorIndex, localIndex );
	}
	else
	{
		b2SolverSet* set = b2Array_Get( world->solverSets, setIndex );
		int movedIndex = b2Array_RemoveSwap( set->jointSims, localIndex );
		if ( movedIndex != B2_NULL_INDEX )
		{
			// 修正被移动的关节
			b2JointSim* movedJointSim = set->jointSims.data + localIndex;
			int movedId = movedJointSim->jointId;
			b2Joint* movedJoint = b2Array_Get( world->joints, movedId );
			B2_ASSERT( movedJoint->localIndex == movedIndex );
			movedJoint->localIndex = localIndex;
		}
	}

	// 释放关节和 id（保留关节代数）
	joint->setIndex = B2_NULL_INDEX;
	joint->localIndex = B2_NULL_INDEX;
	joint->colorIndex = B2_NULL_INDEX;
	joint->jointId = B2_NULL_INDEX;
	b2FreeId( &world->jointIdPool, jointId );

	b2WakeBody( world, bodyA );
	b2WakeBody( world, bodyB );


}

void b2DestroyJoint(b2JointId id)
{
 if(!b2Joint_IsValid(id)) { return; }b2World* world=b2_worlds+id.world0;
 if(!world->locked) { b2DestroyJointInternal(world,b2GetJointFullId(world,id)); }
}
b2WorldId b2Joint_GetWorld(b2JointId id)
{
 if(!b2Joint_IsValid(id)) { return b2WorldId{}; }return b2WorldId{uint16_t(id.world0+1),b2_worlds[id.world0].generation};
}
static b2BodyId b2GetJointBody(b2JointId id,int edgeIndex)
{
 if(!b2Joint_IsValid(id)) { return b2_nullBodyId; }b2World* world=b2_worlds+id.world0;b2Joint* joint=b2GetJointFullId(world,id);b2Body* body=world->bodies.data+joint->edges[edgeIndex].bodyId;
 return b2BodyId{body->id+1,world->worldId,body->generation};
}
b2BodyId b2Joint_GetBodyA(b2JointId id) { return b2GetJointBody(id,0); }
b2BodyId b2Joint_GetBodyB(b2JointId id) { return b2GetJointBody(id,1); }
void b2Joint_SetUserData(b2JointId id,void* data) { if(b2Joint_IsValid(id)) { b2GetJointFullId(b2_worlds+id.world0,id)->userData=data; } }

void b2Joint_SetCollideConnected( b2JointId jointId, bool shouldCollide )
{
	if ( jointId.world0 >= B2_MAX_WORLDS )
	{
		return;
	}

	b2World* world = b2_worlds + jointId.world0;
	if ( world->inUse == false || world->locked )
	{
		return;
	}

	b2Joint* joint = b2GetJointFullId( world, jointId );
	if ( joint == nullptr || joint->collideConnected == shouldCollide )
	{
		return;
	}

	joint->collideConnected = shouldCollide;
	b2Body* bodyA = world->bodies.data + joint->edges[0].bodyId;
	b2Body* bodyB = world->bodies.data + joint->edges[1].bodyId;
	if ( shouldCollide )
	{
		int shapeId = B2_NULL_INDEX;
		if ( bodyA->type == b2_dynamicBody && bodyB->type == b2_dynamicBody )
		{
			shapeId = bodyA->shapeCount < bodyB->shapeCount ? bodyA->headShapeId : bodyB->headShapeId;
		}
		else if ( bodyA->type == b2_dynamicBody )
		{
			shapeId = bodyA->headShapeId;
		}
		else if ( bodyB->type == b2_dynamicBody )
		{
			shapeId = bodyB->headShapeId;
		}

		while ( shapeId != B2_NULL_INDEX )
		{
			b2Shape* shape = world->shapes.data + shapeId;
			if ( shape->proxyKey != B2_NULL_INDEX )
			{
				b2BroadPhase_MarkProxyMovedSerial( &world->broadPhase, shape->proxyKey );
			}
			shapeId = shape->nextShapeId;
		}
	}
	else
	{
		b2DestroyContactsBetweenBodies( world, bodyA, bodyB );
	}
}

bool b2Joint_GetCollideConnected( b2JointId jointId )
{
	if ( b2Joint_IsValid( jointId ) == false )
	{
		return false;
	}
	return b2GetJointFullId( b2_worlds + jointId.world0, jointId )->collideConnected;
}
void* b2Joint_GetUserData(b2JointId id) { return b2Joint_IsValid(id)?b2GetJointFullId(b2_worlds+id.world0,id)->userData:nullptr; }

static b2JointDef b2DefaultJointDef( void )
{
	b2JointDef def = {};
	def.localFrameA.q = b2Rot_identity;
	def.localFrameB.q = b2Rot_identity;
	def.constraintHertz = 60.0f;
	def.constraintDampingRatio = 2.0f;
	def.drawScale = b2GetLengthUnitsPerMeter();
	return def;
}

b2DistanceJointDef b2DefaultDistanceJointDef( void )
{
	b2DistanceJointDef def = {};
	def.base = b2DefaultJointDef();
	def.lowerSpringForce=-FLT_MAX;def.upperSpringForce=FLT_MAX;
	def.length = 1.0f;
	def.maxLength=B2_HUGE;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

void b2UnlinkBodyJoints(b2World* world,b2Body* body)
{
 for(int key=body->headJointKey;key!=B2_NULL_INDEX;)
 {
  b2Joint* joint=world->joints.data+(key>>1);int edge=key&1;key=joint->edges[edge].nextKey;
  b2WakeBody(world,world->bodies.data+joint->edges[0].bodyId);b2WakeBody(world,world->bodies.data+joint->edges[1].bodyId);
  b2UnlinkJoint(world,joint);b2TransferJoint(world,world->solverSets.data+b2_disabledSet,world->solverSets.data+joint->setIndex,joint);
 }
}
void b2RelinkBodyJoints(b2World* world,b2Body* body)
{
 for(int key=body->headJointKey;key!=B2_NULL_INDEX;)
 {
  b2Joint* joint=world->joints.data+(key>>1);int edge=key&1;key=joint->edges[edge].nextKey;
  b2Body* a=world->bodies.data+joint->edges[0].bodyId;b2Body* b=world->bodies.data+joint->edges[1].bodyId;
  if(a->setIndex==b2_disabledSet || b->setIndex==b2_disabledSet) { continue; }
  int target=b2_staticSet;
  if(a->type==b2_dynamicBody || b->type==b2_dynamicBody) { b2WakeBody(world,a);b2WakeBody(world,b);target=b2_awakeSet; }
  b2TransferJoint(world,world->solverSets.data+target,world->solverSets.data+joint->setIndex,joint);
  if(target==b2_awakeSet) { b2LinkJoint(world,joint); }
 }
}

b2JointSim* b2GetJointSimCheckType(b2JointId id,b2JointType type)
{
 B2_ASSERT(b2Joint_IsValid(id));b2World* world=b2_worlds+id.world0;B2_ASSERT(!world->locked);
 if(world->locked) { return nullptr; }b2Joint* joint=b2GetJointFullId(world,id);B2_ASSERT(joint->type==type);return b2GetJointSim(world,joint);
}
b2JointId b2CreateDistanceJoint(b2WorldId worldId,const b2DistanceJointDef* def)
{
 B2_CHECK_DEF(def);
 if(!b2World_IsValid(worldId) || !b2IsValidFloat(def->length) || def->length<=0 || !b2IsValidTransform(def->base.localFrameA) || !b2IsValidTransform(def->base.localFrameB) || !b2IsValidFloat(def->base.constraintHertz) || def->base.constraintHertz<0 || !b2IsValidFloat(def->base.constraintDampingRatio) || def->base.constraintDampingRatio<0 || !b2IsValidFloat(def->base.drawScale)) { return b2_nullJointId; }
 if(!b2IsValidFloat(def->hertz) || !b2IsValidFloat(def->dampingRatio) || !b2IsValidFloat(def->lowerSpringForce) || !b2IsValidFloat(def->upperSpringForce)) { return b2_nullJointId; }
 if(!b2IsValidFloat(def->minLength) || !b2IsValidFloat(def->maxLength)) { return b2_nullJointId; }
 if(!b2IsValidFloat(def->maxMotorForce) || !b2IsValidFloat(def->motorSpeed)) { return b2_nullJointId; }
 b2World* world=b2_worlds+worldId.index1-1;const b2JointDef* base=&def->base;
 if(!world->inUse || world->locked || base->bodyIdA.world0!=world->worldId || base->bodyIdB.world0!=world->worldId || base->bodyIdA.index1==base->bodyIdB.index1 || !b2Body_IsValid(base->bodyIdA) || !b2Body_IsValid(base->bodyIdB)) { return b2_nullJointId; }
 b2JointPair pair=b2CreateJoint(world,base,b2_distanceJoint);pair.jointSim->distanceJoint={};pair.jointSim->distanceJoint.length=b2MaxFloat(def->length,B2_LINEAR_SLOP);

 pair.jointSim->distanceJoint.enableMotor=def->enableMotor;pair.jointSim->distanceJoint.maxMotorForce=def->maxMotorForce;pair.jointSim->distanceJoint.motorSpeed=def->motorSpeed;
 pair.jointSim->distanceJoint.enableLimit=def->enableLimit;pair.jointSim->distanceJoint.minLength=b2MaxFloat(def->minLength,B2_LINEAR_SLOP);pair.jointSim->distanceJoint.maxLength=b2MaxFloat(def->minLength,def->maxLength);
 pair.jointSim->distanceJoint.enableSpring=def->enableSpring;pair.jointSim->distanceJoint.hertz=def->hertz;pair.jointSim->distanceJoint.dampingRatio=def->dampingRatio;
 pair.jointSim->distanceJoint.lowerSpringForce=b2MinFloat(def->lowerSpringForce,def->upperSpringForce);pair.jointSim->distanceJoint.upperSpringForce=b2MaxFloat(def->lowerSpringForce,def->upperSpringForce);

 return b2JointId{pair.joint->jointId+1,world->worldId,pair.joint->generation};
}
b2JointType b2Joint_GetType(b2JointId id) { return b2GetJointFullId(b2_worlds+id.world0,id)->type; }
b2Transform b2Joint_GetLocalFrameA(b2JointId id) { return b2GetJointSim(b2_worlds+id.world0,b2GetJointFullId(b2_worlds+id.world0,id))->localFrameA; }
b2Transform b2Joint_GetLocalFrameB(b2JointId id) { return b2GetJointSim(b2_worlds+id.world0,b2GetJointFullId(b2_worlds+id.world0,id))->localFrameB; }

void b2Joint_SetLocalFrameA( b2JointId jointId, b2Transform localFrame )
{
	if ( b2IsValidTransform( localFrame ) == false || b2Joint_IsValid( jointId ) == false )
	{
		return;
	}
	b2World* world = b2_worlds + jointId.world0;
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2GetJointSim( world, joint )->localFrameA = localFrame;
}

void b2Joint_SetLocalFrameB( b2JointId jointId, b2Transform localFrame )
{
	if ( b2IsValidTransform( localFrame ) == false || b2Joint_IsValid( jointId ) == false )
	{
		return;
	}
	b2World* world = b2_worlds + jointId.world0;
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2GetJointSim( world, joint )->localFrameB = localFrame;
}

void b2Joint_SetConstraintTuning( b2JointId jointId, float hertz, float dampingRatio )
{
	if ( b2IsValidFloat( hertz ) == false || hertz < 0.0f || b2IsValidFloat( dampingRatio ) == false || dampingRatio < 0.0f || b2Joint_IsValid( jointId ) == false )
	{
		return;
	}
	b2World* world = b2_worlds + jointId.world0;
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* base = b2GetJointSim( world, joint );
	base->constraintHertz = hertz;
	base->constraintDampingRatio = dampingRatio;
}

void b2Joint_GetConstraintTuning( b2JointId jointId, float* hertz, float* dampingRatio )
{
	b2World* world = b2_worlds + jointId.world0;
	b2Joint* joint = b2GetJointFullId( world, jointId );
	b2JointSim* base = b2GetJointSim( world, joint );
	*hertz = base->constraintHertz;
	*dampingRatio = base->constraintDampingRatio;
}

void b2PrepareJoint( b2JointSim* joint, b2StepContext* context )
{
	// 按时间步钳制关节 hertz 以减少抖动。
	float hertz = b2MinFloat( joint->constraintHertz, 0.25f * context->inv_h );
	joint->constraintSoftness = b2MakeSoft( hertz, joint->constraintDampingRatio, context->h );

	switch ( joint->type )
	{
		case b2_distanceJoint:
			b2PrepareDistanceJoint( joint, context );
			break;

		case b2_revoluteJoint:
			b2PrepareRevoluteJoint( joint, context );
			break;

		case b2_prismaticJoint:
			b2PreparePrismaticJoint( joint, context );
			break;

		case b2_wheelJoint:
			b2PrepareWheelJoint( joint, context );
			break;

		case b2_weldJoint:
			b2PrepareWeldJoint( joint, context );
			break;

		case b2_motorJoint:
			b2PrepareMotorJoint( joint, context );
			break;

		case b2_moverJoint:
			b2PrepareMoverJoint( joint, context );
			break;

		case b2_pogoJoint:
			b2PreparePogoJoint( joint, context );
			break;

		case b2_filterJoint:
			break;

		default:
			B2_ASSERT( false );
	}
}

void b2WarmStartJoint( b2JointSim* joint, b2StepContext* context )
{
	switch ( joint->type )
	{
		case b2_distanceJoint:
			b2WarmStartDistanceJoint( joint, context );
			break;

		case b2_revoluteJoint:
			b2WarmStartRevoluteJoint( joint, context );
			break;

		case b2_prismaticJoint:
			b2WarmStartPrismaticJoint( joint, context );
			break;

		case b2_wheelJoint:
			b2WarmStartWheelJoint( joint, context );
			break;

		case b2_weldJoint:
			b2WarmStartWeldJoint( joint, context );
			break;

		case b2_motorJoint:
			b2WarmStartMotorJoint( joint, context );
			break;

		case b2_moverJoint:
			b2WarmStartMoverJoint( joint, context );
			break;

		case b2_pogoJoint:
			b2WarmStartPogoJoint( joint, context );
			break;

		case b2_filterJoint:
			break;

		default:
			B2_ASSERT( false );
	}
}

void b2SolveJoint( b2JointSim* joint, b2StepContext* context, bool useBias )
{
	switch ( joint->type )
	{
		case b2_distanceJoint:
			b2SolveDistanceJoint( joint, context, useBias );
			break;

		case b2_revoluteJoint:
			b2SolveRevoluteJoint( joint, context, useBias );
			break;

		case b2_prismaticJoint:
			b2SolvePrismaticJoint( joint, context, useBias );
			break;

		case b2_wheelJoint:
			b2SolveWheelJoint( joint, context, useBias );
			break;

		case b2_weldJoint:
			b2SolveWeldJoint( joint, context, useBias );
			break;

		case b2_motorJoint:
			b2SolveMotorJoint( joint, context );
			break;

		case b2_moverJoint:
			b2SolveMoverJoint( joint, context );
			break;

		case b2_pogoJoint:
			b2SolvePogoJoint( joint, context, useBias );
			break;

		case b2_filterJoint:
			break;

		default:
			B2_ASSERT( false );
	}
}

static b2World* b2GetWorldForJointDef(b2WorldId id,const b2JointDef* def)
{
 if(!b2World_IsValid(id) || !b2IsValidTransform(def->localFrameA) || !b2IsValidTransform(def->localFrameB) || !b2IsValidFloat(def->constraintHertz) || def->constraintHertz<0 || !b2IsValidFloat(def->constraintDampingRatio) || def->constraintDampingRatio<0 || !b2IsValidFloat(def->drawScale)) { return nullptr; }
 b2World* world=b2_worlds+id.index1-1;
 if(!world->inUse || world->locked || def->bodyIdA.world0!=world->worldId || def->bodyIdB.world0!=world->worldId || def->bodyIdA.index1==def->bodyIdB.index1 || !b2Body_IsValid(def->bodyIdA) || !b2Body_IsValid(def->bodyIdB)) { return nullptr; }
 return world;
}

b2RevoluteJointDef b2DefaultRevoluteJointDef( void )
{
	b2RevoluteJointDef def = {};
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}
b2JointId b2CreateRevoluteJoint( b2WorldId worldId, const b2RevoluteJointDef* def )
{
	B2_CHECK_DEF( def );
	if (!(b2IsValidFloat( def->targetAngle ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->hertz ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->dampingRatio ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->lowerAngle ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->upperAngle ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->maxMotorTorque ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->motorSpeed ))) { return b2_nullJointId; }
	B2_ASSERT( def->lowerAngle <= def->upperAngle );

	b2World* world=b2GetWorldForJointDef(worldId,&def->base);if(world==nullptr) { return b2_nullJointId; }

	B2_ASSERT( world->locked == false );

	if ( world->locked )
	{
		return b2JointId{};
	}

	b2JointPair pair = b2CreateJoint( world, &def->base, b2_revoluteJoint );

	b2JointSim* joint = pair.jointSim;

	b2RevoluteJoint empty = {};
	joint->revoluteJoint = empty;

	joint->revoluteJoint.targetAngle = b2ClampFloat( def->targetAngle, -B2_PI, B2_PI );
	joint->revoluteJoint.hertz = def->hertz;
	joint->revoluteJoint.dampingRatio = def->dampingRatio;

	float lowerAngle = b2MinFloat( def->lowerAngle, def->upperAngle );
	float upperAngle = b2MaxFloat( def->lowerAngle, def->upperAngle );
	joint->revoluteJoint.lowerAngle = b2ClampFloat( lowerAngle, -0.99f * B2_PI, 0.99f * B2_PI );
	joint->revoluteJoint.upperAngle = b2ClampFloat( upperAngle, -0.99f * B2_PI, 0.99f * B2_PI );

	joint->revoluteJoint.maxMotorTorque = def->maxMotorTorque;
	joint->revoluteJoint.motorSpeed = def->motorSpeed;
	joint->revoluteJoint.enableSpring = def->enableSpring;
	joint->revoluteJoint.enableLimit = def->enableLimit;
	joint->revoluteJoint.enableMotor = def->enableMotor;

	b2JointId jointId = { joint->jointId + 1, world->worldId, pair.joint->generation };


	return jointId;
}

b2PrismaticJointDef b2DefaultPrismaticJointDef( void )
{
	b2PrismaticJointDef def = {};
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}
b2JointId b2CreatePrismaticJoint( b2WorldId worldId, const b2PrismaticJointDef* def )
{
	B2_CHECK_DEF( def );
	if (!(b2IsValidFloat( def->hertz ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->dampingRatio ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->targetTranslation ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->lowerTranslation ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->upperTranslation ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->maxMotorForce ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->motorSpeed ))) { return b2_nullJointId; }
	B2_ASSERT( def->lowerTranslation <= def->upperTranslation );

	b2World* world=b2GetWorldForJointDef(worldId,&def->base);if(world==nullptr) { return b2_nullJointId; }

	B2_ASSERT( world->locked == false );

	if ( world->locked )
	{
		return b2JointId{};
	}

	b2JointPair pair = b2CreateJoint( world, &def->base, b2_prismaticJoint );

	b2JointSim* joint = pair.jointSim;

	joint->prismaticJoint = b2PrismaticJoint{};
	joint->prismaticJoint.hertz = def->hertz;
	joint->prismaticJoint.dampingRatio = def->dampingRatio;
	joint->prismaticJoint.targetTranslation = def->targetTranslation;
	joint->prismaticJoint.lowerTranslation = b2MinFloat( def->lowerTranslation, def->upperTranslation );
	joint->prismaticJoint.upperTranslation = b2MaxFloat( def->lowerTranslation, def->upperTranslation );
	joint->prismaticJoint.maxMotorForce = def->maxMotorForce;
	joint->prismaticJoint.motorSpeed = def->motorSpeed;
	joint->prismaticJoint.enableSpring = def->enableSpring;
	joint->prismaticJoint.enableLimit = def->enableLimit;
	joint->prismaticJoint.enableMotor = def->enableMotor;

	b2JointId jointId = {
		.index1 = joint->jointId + 1,
		.world0 = world->worldId,
		.generation = pair.joint->generation,
	};


	return jointId;
}

b2WheelJointDef b2DefaultWheelJointDef( void )
{
	b2WheelJointDef def = {};
	def.base = b2DefaultJointDef();
	def.enableSpring = true;
	def.hertz = 1.0f;
	def.dampingRatio = 0.7f;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}
b2JointId b2CreateWheelJoint( b2WorldId worldId, const b2WheelJointDef* def )
{
	B2_CHECK_DEF( def );
	if (!(b2IsValidFloat( def->hertz ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->dampingRatio ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->lowerTranslation ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->upperTranslation ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->maxMotorTorque ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->motorSpeed ))) { return b2_nullJointId; }
	B2_ASSERT( def->lowerTranslation <= def->upperTranslation );

	b2World* world=b2GetWorldForJointDef(worldId,&def->base);if(world==nullptr) { return b2_nullJointId; }

	B2_ASSERT( world->locked == false );

	if ( world->locked )
	{
		return b2JointId{};
	}

	b2JointPair pair = b2CreateJoint( world, &def->base, b2_wheelJoint );

	b2JointSim* joint = pair.jointSim;

	joint->wheelJoint = b2WheelJoint{};
	joint->wheelJoint.perpMass = 0.0f;
	joint->wheelJoint.axialMass = 0.0f;
	joint->wheelJoint.motorImpulse = 0.0f;
	joint->wheelJoint.lowerImpulse = 0.0f;
	joint->wheelJoint.upperImpulse = 0.0f;
	joint->wheelJoint.lowerTranslation = b2MinFloat( def->lowerTranslation, def->upperTranslation );
	joint->wheelJoint.upperTranslation = b2MaxFloat( def->lowerTranslation, def->upperTranslation );
	joint->wheelJoint.maxMotorTorque = def->maxMotorTorque;
	joint->wheelJoint.motorSpeed = def->motorSpeed;
	joint->wheelJoint.hertz = def->hertz;
	joint->wheelJoint.dampingRatio = def->dampingRatio;
	joint->wheelJoint.enableSpring = def->enableSpring;
	joint->wheelJoint.enableLimit = def->enableLimit;
	joint->wheelJoint.enableMotor = def->enableMotor;

	b2JointId jointId = { joint->jointId + 1, world->worldId, pair.joint->generation };


	return jointId;
}

b2WeldJointDef b2DefaultWeldJointDef( void )
{
	b2WeldJointDef def = {};
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}
b2JointId b2CreateWeldJoint( b2WorldId worldId, const b2WeldJointDef* def )
{
	B2_CHECK_DEF( def );
	if (!(b2IsValidFloat( def->linearHertz ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->angularHertz ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->linearDampingRatio ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->angularDampingRatio ))) { return b2_nullJointId; }

	b2World* world=b2GetWorldForJointDef(worldId,&def->base);if(world==nullptr) { return b2_nullJointId; }

	B2_ASSERT( world->locked == false );

	if ( world->locked )
	{
		return b2JointId{};
	}

	b2JointPair pair = b2CreateJoint( world, &def->base, b2_weldJoint );

	b2JointSim* joint = pair.jointSim;

	b2WeldJoint empty = {};
	joint->weldJoint = empty;
	joint->weldJoint.linearHertz = def->linearHertz;
	joint->weldJoint.linearDampingRatio = def->linearDampingRatio;
	joint->weldJoint.angularHertz = def->angularHertz;
	joint->weldJoint.angularDampingRatio = def->angularDampingRatio;
	joint->weldJoint.linearImpulse = b2Vec2_zero;
	joint->weldJoint.angularImpulse = 0.0f;

	b2JointId jointId = { joint->jointId + 1, world->worldId, pair.joint->generation };


	return jointId;
}

b2MotorJointDef b2DefaultMotorJointDef( void )
{
	b2MotorJointDef def = {};
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}
b2JointId b2CreateMotorJoint( b2WorldId worldId, const b2MotorJointDef* def )
{
	B2_CHECK_DEF( def );
	if (!(b2IsValidVec2( def->linearVelocity ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->maxVelocityForce ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->angularVelocity ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->maxVelocityTorque ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->linearHertz ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->linearDampingRatio ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->maxSpringForce ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->angularHertz ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->angularDampingRatio ))) { return b2_nullJointId; }
	if (!(b2IsValidFloat( def->maxSpringTorque ))) { return b2_nullJointId; }

	b2World* world=b2GetWorldForJointDef(worldId,&def->base);if(world==nullptr) { return b2_nullJointId; }

	B2_ASSERT( world->locked == false );

	if ( world->locked )
	{
		return b2JointId{};
	}

	b2JointPair pair = b2CreateJoint( world, &def->base, b2_motorJoint );
	b2JointSim* joint = pair.jointSim;

	joint->motorJoint = b2MotorJoint{};
	joint->motorJoint.linearVelocity = def->linearVelocity;
	joint->motorJoint.maxVelocityForce = def->maxVelocityForce;
	joint->motorJoint.angularVelocity = def->angularVelocity;
	joint->motorJoint.maxVelocityTorque = def->maxVelocityTorque;
	joint->motorJoint.linearHertz = def->linearHertz;
	joint->motorJoint.linearDampingRatio = def->linearDampingRatio;
	joint->motorJoint.maxSpringForce = def->maxSpringForce;
	joint->motorJoint.angularHertz = def->angularHertz;
	joint->motorJoint.angularDampingRatio = def->angularDampingRatio;
	joint->motorJoint.maxSpringTorque = def->maxSpringTorque;

	b2JointId jointId = { joint->jointId + 1, world->worldId, pair.joint->generation };


	return jointId;
}

b2MoverJointDef b2DefaultMoverJointDef( void )
{
	b2MoverJointDef def = {};
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2JointId b2CreateMoverJoint( b2WorldId worldId, const b2MoverJointDef* def )
{
	B2_CHECK_DEF( def );
	if ( b2IsValidVec2( def->linearVelocity ) == false || b2IsValidVec2( def->maxVelocityForce ) == false )
	{
		return b2_nullJointId;
	}

	b2World* world = b2GetWorldForJointDef( worldId, &def->base );
	if ( world == nullptr )
	{
		return b2_nullJointId;
	}
	if ( world->locked )
	{
		return b2JointId{};
	}

	b2JointPair pair = b2CreateJoint( world, &def->base, b2_moverJoint );
	b2JointSim* joint = pair.jointSim;
	joint->moverJoint = b2MoverJoint{};
	joint->moverJoint.linearVelocity = def->linearVelocity;
	joint->moverJoint.maxVelocityForce = def->maxVelocityForce;
	return b2JointId{ joint->jointId + 1, world->worldId, pair.joint->generation };
}

b2PogoJointDef b2DefaultPogoJointDef( void )
{
	b2PogoJointDef def = {};
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2JointId b2CreatePogoJoint( b2WorldId worldId, const b2PogoJointDef* def )
{
	B2_CHECK_DEF( def );
	if ( b2IsValidVec2( def->normal ) == false || b2IsValidFloat( def->restLength ) == false || def->restLength < 0.0f ||
		 b2IsValidFloat( def->hertz ) == false || def->hertz < 0.0f || b2IsValidFloat( def->dampingRatio ) == false ||
		 def->dampingRatio < 0.0f || b2IsValidFloat( def->maxTensionForce ) == false || def->maxTensionForce < 0.0f ||
		 b2IsValidFloat( def->maxCompressionForce ) == false || def->maxCompressionForce < 0.0f ||
		 b2IsValidFloat( def->impulse ) == false || b2IsValidFloat( def->velocity ) == false )
	{
		return b2_nullJointId;
	}

	b2World* world = b2GetWorldForJointDef( worldId, &def->base );
	if ( world == nullptr || world->locked )
	{
		return b2_nullJointId;
	}

	b2JointPair pair = b2CreateJoint( world, &def->base, b2_pogoJoint );
	b2JointSim* joint = pair.jointSim;
	joint->pogoJoint = b2PogoJoint{};
	joint->pogoJoint.restLength = def->restLength;
	joint->pogoJoint.hertz = def->hertz;
	joint->pogoJoint.dampingRatio = def->dampingRatio;
	joint->pogoJoint.maxTensionForce = def->maxTensionForce;
	joint->pogoJoint.maxCompressionForce = def->maxCompressionForce;
	joint->pogoJoint.normal = def->normal;
	joint->pogoJoint.impulse = def->impulse;
	joint->pogoJoint.velocity = def->velocity;
	return b2JointId{ joint->jointId + 1, world->worldId, pair.joint->generation };
}

b2FilterJointDef b2DefaultFilterJointDef( void )
{
	b2FilterJointDef def = {};
	def.base = b2DefaultJointDef();
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

b2JointId b2CreateFilterJoint( b2WorldId worldId, const b2FilterJointDef* def )
{
	B2_CHECK_DEF( def );
	b2World* world = b2GetWorldForJointDef( worldId, &def->base );
	if ( world == nullptr || world->locked )
	{
		return b2_nullJointId;
	}

	b2JointPair pair = b2CreateJoint( world, &def->base, b2_filterJoint );
	return b2JointId{ pair.jointSim->jointId + 1, world->worldId, pair.joint->generation };
}

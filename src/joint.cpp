// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "joint.h"
#include "body.h"
#include "world.h"

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
b2JointId b2CreateJointRecord(b2World* world,b2BodyId idA,b2BodyId idB)
{
 if(world==nullptr || world->locked || idA.world0!=world->worldId || idB.world0!=world->worldId || idA.index1==idB.index1) { return b2_nullJointId; }
 b2Body* bodies[2]={b2GetBodyFullId(world,idA),b2GetBodyFullId(world,idB)};
 if(bodies[0]==nullptr || bodies[1]==nullptr) { return b2_nullJointId; }
 int jointId=b2AllocId(&world->jointIdPool);
 if(jointId==world->joints.count) { b2Array_Push(world->joints,b2Joint{}); }
 b2Joint* joint=world->joints.data+jointId;uint16_t generation=joint->generation+1;*joint=b2Joint{};joint->jointId=jointId;joint->generation=generation;
 for(int i=0;i<2;++i)
 {
  b2Body* body=bodies[i];b2JointEdge* edge=joint->edges+i;edge->bodyId=body->id;edge->prevKey=B2_NULL_INDEX;edge->nextKey=body->headJointKey;
  int key=(jointId<<1)|i;
  if(body->headJointKey!=B2_NULL_INDEX) { int previous=body->headJointKey;world->joints.data[previous>>1].edges[previous&1].prevKey=key; }
  body->headJointKey=key;++body->jointCount;
 }
 return b2JointId{jointId+1,world->worldId,generation};
}
void b2DestroyJointInternal(b2World* world,b2Joint* joint)
{
 b2Body* bodies[2]={world->bodies.data+joint->edges[0].bodyId,world->bodies.data+joint->edges[1].bodyId};
 for(int i=0;i<2;++i)
 {
  b2JointEdge edge=joint->edges[i];
  if(edge.prevKey!=B2_NULL_INDEX) { world->joints.data[edge.prevKey>>1].edges[edge.prevKey&1].nextKey=edge.nextKey; }
  else { bodies[i]->headJointKey=edge.nextKey; }
  if(edge.nextKey!=B2_NULL_INDEX) { world->joints.data[edge.nextKey>>1].edges[edge.nextKey&1].prevKey=edge.prevKey; }
  --bodies[i]->jointCount;
 }
 int index=joint->jointId;joint->jointId=B2_NULL_INDEX;b2FreeId(&world->jointIdPool,index);
 b2WakeBody(world,bodies[0]);b2WakeBody(world,bodies[1]);
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
	def.length = 1.0f;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

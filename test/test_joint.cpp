// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "test_macros.h"
#include "world.h"
#include "joint.h"
int JointLifecycleTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
 b2BodyId a=b2CreateBody(id,&bd),b=b2CreateBody(id,&bd),c=b2CreateBody(id,&bd);
 b2JointId ab=b2CreateJointRecord(world,a,b),ac=b2CreateJointRecord(world,a,c),bc=b2CreateJointRecord(world,b,c);
 ENSURE(b2Joint_IsValid(ab) && b2Joint_IsValid(ac) && b2Joint_IsValid(bc));ENSURE(b2Body_GetJointCount(a)==2 && b2Body_GetJointCount(b)==2);ENSURE(b2World_GetCounters(id).jointCount==3);
 ENSURE(b2Joint_GetBodyA(ab).index1==a.index1 && b2Joint_GetBodyB(ab).index1==b.index1);ENSURE(b2Joint_GetWorld(ab).index1==id.index1);
 int tag=42;b2Joint_SetUserData(ab,&tag);ENSURE(b2Joint_GetUserData(ab)==&tag);b2JointId list[3];ENSURE(b2Body_GetJoints(a,list,1)==1 && list[0].index1==ac.index1);ENSURE(b2Body_GetJoints(a,list,0)==0);
 b2DestroyJoint(ab);ENSURE(!b2Joint_IsValid(ab) && b2Body_GetJointCount(a)==1 && b2Body_GetJointCount(b)==1);ENSURE(b2Body_GetJoints(a,list,3)==1 && list[0].index1==ac.index1);
 b2JointId reused=b2CreateJointRecord(world,b,a);ENSURE(reused.index1==ab.index1 && reused.generation!=ab.generation);b2DestroyJoint(ab);ENSURE(b2Joint_IsValid(reused));
 world->locked=true;b2DestroyJoint(ac);ENSURE(b2CreateJointRecord(world,a,b).index1==0);world->locked=false;ENSURE(b2Joint_IsValid(ac));ENSURE(b2CreateJointRecord(world,a,a).index1==0);
 b2DestroyBody(a);ENSURE(!b2Joint_IsValid(ac) && !b2Joint_IsValid(reused) && b2Joint_IsValid(bc));ENSURE(b2Body_GetJointCount(b)==1 && b2Body_GetJointCount(c)==1);
 b2DestroyWorld(id);ENSURE(!b2Joint_IsValid(bc));ENSURE(b2GetByteCount()==before);return 0;
}

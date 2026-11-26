// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "test_macros.h"
#include "world.h"
#include "joint.h"
#include <math.h>
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

int JointDefinitionTest(void)
{
 size_t before=b2GetByteCount();b2DistanceJointDef def=b2DefaultDistanceJointDef();ENSURE(def.length==1 && def.base.bodyIdA.index1==0 && def.base.bodyIdB.index1==0);
 ENSURE(def.base.localFrameA.q.c==1 && def.base.localFrameB.q.c==1 && def.base.localFrameA.p.x==0);ENSURE(def.base.constraintHertz==60 && def.base.constraintDampingRatio==2 && def.base.drawScale==1 && !def.base.collideConnected);
 b2SetLengthUnitsPerMeter(3);b2DistanceJointDef scaled=b2DefaultDistanceJointDef();ENSURE(scaled.base.drawScale==3 && scaled.length==1);b2SetLengthUnitsPerMeter(1);
 b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.position={100,200};bd.rotation={0,1};b2BodyId body=b2CreateBody(id,&bd);
 b2WorldTransform frame={{100,203},{0,1}};b2Transform local=b2InvMulWorldTransforms(b2Body_GetTransform(body),frame);ENSURE(local.p.x==3 && local.p.y==0 && local.q.c==1);
 b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{2,0},0.5f};b2CreateCircleShape(body,&sd,&circle);b2Vec2 point=b2Body_GetLocalPoint(body,frame.p);ENSURE(point.x==3 && point.y==0);
 b2Pos worldPoint=b2Body_GetWorldPoint(body,local.p);ENSURE(worldPoint.x==100 && worldPoint.y==203);b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int JointOrganizationTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
 b2BodyId a=b2CreateBody(id,&bd),b=b2CreateBody(id,&bd),c=b2CreateBody(id,&bd);b2JointId ab=b2CreateJointRecord(world,a,b),bc=b2CreateJointRecord(world,b,c);
 b2Joint* ja=b2GetJointFullId(world,ab);b2Joint* jb=b2GetJointFullId(world,bc);ENSURE(ja->setIndex==b2_awakeSet && ja->colorIndex!=jb->colorIndex);
 ENSURE(b2GetBodyFullId(world,a)->islandId==b2GetBodyFullId(world,c)->islandId);b2JointSim* sim=b2GetJointSim(world,ja);ENSURE(sim->bodyIdA==a.index1-1 && sim->bodyIdB==b.index1-1);
 b2Body_SetAwake(a,false);ENSURE(!b2Body_IsAwake(a) && !b2Body_IsAwake(b) && !b2Body_IsAwake(c));ENSURE(ja->colorIndex==B2_NULL_INDEX && ja->setIndex>=b2_firstSleepingSet);
 b2Body_SetAwake(c,true);ENSURE(b2Body_IsAwake(a) && b2Body_IsAwake(b));ENSURE(ja->setIndex==b2_awakeSet && ja->colorIndex!=jb->colorIndex);
 b2Body_Disable(b);ENSURE(ja->setIndex==b2_disabledSet && jb->setIndex==b2_disabledSet && ja->islandId==B2_NULL_INDEX);b2SplitContactIslands(world);ENSURE(b2GetBodyFullId(world,a)->islandId!=b2GetBodyFullId(world,c)->islandId);
 b2Body_Enable(b);ENSURE(ja->setIndex==b2_awakeSet && b2GetBodyFullId(world,a)->islandId==b2GetBodyFullId(world,c)->islandId);
 b2Body_SetType(b,b2_staticBody);b2SplitContactIslands(world);ENSURE(b2GetBodyFullId(world,a)->islandId!=b2GetBodyFullId(world,c)->islandId);ENSURE(ja->setIndex==b2_awakeSet && jb->setIndex==b2_awakeSet);
 b2Body_SetType(b,b2_dynamicBody);ENSURE(b2GetBodyFullId(world,a)->islandId==b2GetBodyFullId(world,c)->islandId);
 for(int i=0;i<28;++i) { ENSURE(b2CreateJointRecord(world,a,b).index1>0); }ja=b2GetJointFullId(world,ab);jb=b2GetJointFullId(world,bc);ENSURE(world->constraintGraph.colors[B2_OVERFLOW_INDEX].jointSims.count>0);
 int graphCount=0;for(int i=0;i<B2_GRAPH_COLOR_COUNT;++i) { graphCount+=world->constraintGraph.colors[i].jointSims.count; }ENSURE(graphCount==30);b2DestroyJoint(ab);ENSURE(!b2Joint_IsValid(ab) && b2Joint_IsValid(bc));
 bd.isAwake=false;bd.position={10,0};b2BodyId d=b2CreateBody(id,&bd);bd.position={20,0};b2BodyId e=b2CreateBody(id,&bd);b2JointId de=b2CreateJointRecord(world,d,e);ENSURE(!b2Body_IsAwake(d) && !b2Body_IsAwake(e));ENSURE(b2GetBodyFullId(world,d)->setIndex==b2GetBodyFullId(world,e)->setIndex);
 b2Body_SetAwake(d,true);ENSURE(b2Body_IsAwake(e));ENSURE(b2GetJointFullId(world,de)->setIndex==b2_awakeSet);
 bd.type=b2_staticBody;bd.isAwake=true;b2BodyId s=b2CreateBody(id,&bd);b2JointId ds=b2CreateJointRecord(world,d,s),es=b2CreateJointRecord(world,e,s);ENSURE(b2Joint_IsValid(ds) && b2Joint_IsValid(es));
 b2DestroyJoint(de);b2SplitContactIslands(world);ENSURE(b2GetBodyFullId(world,d)->islandId!=b2GetBodyFullId(world,e)->islandId);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int DistanceJointBasicTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();bd.position={100,200};b2BodyId anchor=b2CreateBody(id,&bd);
 bd.type=b2_dynamicBody;bd.position={100,196};bd.enableSleep=false;b2BodyId bob=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},0.25f};b2CreateCircleShape(bob,&sd,&c);
 int tag=17;b2DistanceJointDef def=b2DefaultDistanceJointDef();def.base.bodyIdA=anchor;def.base.bodyIdB=bob;def.base.userData=&tag;def.length=2;b2JointId joint=b2CreateDistanceJoint(id,&def);
 ENSURE(b2Joint_IsValid(joint) && b2Joint_GetType(joint)==b2_distanceJoint && b2Joint_GetUserData(joint)==&tag);ENSURE(b2DistanceJoint_GetRestLength(joint)==2 && b2DistanceJoint_GetCurrentLength(joint)==4);
 ENSURE(b2Joint_GetLocalFrameA(joint).q.c==1 && b2Joint_GetLocalFrameB(joint).p.y==0);
 for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-2,0.02f);ENSURE_SMALL(b2Body_GetPosition(bob).y-198,0.02f);
 b2DistanceJoint_SetRestLength(joint,1.5f);for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-1.5f,0.02f);ENSURE(b2World_GetMaxCapacity(id).jointCount==1);
 b2Body_SetType(anchor,b2_kinematicBody);b2Body_SetLinearVelocity(anchor,{1,0});for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-1.5f,0.03f);
 b2World* world=b2_worlds+id.index1-1;b2JointSim* sim=b2GetJointSim(world,b2GetJointFullId(world,joint));sim->distanceJoint.impulse=10;b2World_EnableWarmStarting(id,false);b2StepContext context={world,&world->constraintGraph,world->solverSets.data[b2_awakeSet].bodyStates.data,1.0f/60,60};b2PrepareJoint(sim,&context);ENSURE(sim->distanceJoint.impulse==0);
 b2DestroyJoint(joint);float y=b2Body_GetPosition(bob).y;for(int i=0;i<10;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(b2Body_GetPosition(bob).y<y-0.05f);b2DestroyWorld(id);
 wd.gravity=b2Vec2_zero;id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={10,0};b2BodyId a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);bd.position={13,0};b2BodyId b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);b2Body_SetLinearVelocity(a,{-1,0});b2Body_SetLinearVelocity(b,{1,0});
 def=b2DefaultDistanceJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.length=2;joint=b2CreateDistanceJoint(id,&def);
 for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-2,0.005f);ENSURE_SMALL(b2Body_GetPosition(a).x+b2Body_GetPosition(b).x-23,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(a).x+b2Body_GetLinearVelocity(b).x,0.0001f);
 b2DestroyWorld(id);
 id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.position={0,0};a=b2CreateBody(id,&bd);c.radius=1;b2CreateCircleShape(a,&sd,&c);bd.position={1,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);b2World_Step(id,0.01f,1);ENSURE(b2Body_GetContactCapacity(a)>0);
 def=b2DefaultDistanceJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.length=1;joint=b2CreateDistanceJoint(id,&def);ENSURE(b2Body_GetContactCapacity(a)==0);b2World_Step(id,0.01f,1);ENSURE(b2Body_GetContactCapacity(a)==0);
 def.length=0;ENSURE(b2CreateDistanceJoint(id,&def).index1==0);b2DestroyJoint(joint);def.length=1;def.base.collideConnected=true;def.base.localFrameA.p={0,0.5f};def.base.localFrameB.p={0,-0.5f};joint=b2CreateDistanceJoint(id,&def);ENSURE(b2Joint_GetLocalFrameA(joint).p.y==0.5f && b2Joint_GetLocalFrameB(joint).p.y==-0.5f);
 for(int i=0;i<30;++i) { b2World_Step(id,0.01f,4); }ENSURE(b2IsValidFloat(b2DistanceJoint_GetCurrentLength(joint)));
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int DistanceJointSpringTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId anchor=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={3,0};b2BodyId bob=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.density=1/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(bob,&sd,&c);
 b2DistanceJointDef def=b2DefaultDistanceJointDef();ENSURE(!def.enableSpring && def.lowerSpringForce==-FLT_MAX && def.upperSpringForce==FLT_MAX);def.base.bodyIdA=anchor;def.base.bodyIdB=bob;def.length=1;def.enableSpring=true;def.hertz=2;def.dampingRatio=0.7f;b2JointId joint=b2CreateDistanceJoint(id,&def);ENSURE(b2DistanceJoint_IsSpringEnabled(joint));ENSURE(b2DistanceJoint_GetSpringHertz(joint)==2 && b2DistanceJoint_GetSpringDampingRatio(joint)==0.7f);
 for(int i=0;i<240;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-1,0.01f);
 b2World_EnableWarmStarting(id,false);b2DistanceJoint_SetSpringHertz(joint,5);b2DistanceJoint_SetSpringDampingRatio(joint,1);b2DistanceJoint_SetSpringForceRange(joint,-2,3);float lower,upper;b2DistanceJoint_GetSpringForceRange(joint,&lower,&upper);ENSURE(lower==-2 && upper==3);
 b2Body_SetTransform(bob,{3,0},b2Rot_identity);b2Body_SetLinearVelocity(bob,{0,0});b2World_Step(id,0.01f,1);ENSURE_SMALL(b2DistanceJoint_GetSpringForce(joint)+2,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(bob).x+0.02f,0.0001f);
 b2Body_SetTransform(bob,{0.5f,0},b2Rot_identity);b2Body_SetLinearVelocity(bob,{0,0});b2World_Step(id,0.01f,1);ENSURE_SMALL(b2DistanceJoint_GetSpringForce(joint)-3,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(bob).x-0.03f,0.0001f);
 b2DistanceJoint_SetSpringHertz(joint,0);b2Body_SetTransform(bob,{3,0},b2Rot_identity);b2Body_SetLinearVelocity(bob,{1,0});b2World_Step(id,0.1f,1);ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-3.1f,0.0001f);
 b2DistanceJoint_EnableSpring(joint,false);ENSURE(!b2DistanceJoint_IsSpringEnabled(joint));for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-1,0.01f);ENSURE(b2DistanceJoint_GetSpringForce(joint)==0);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int DistanceJointLimitTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId anchor=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={3,0};b2BodyId bob=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},0.2f};b2CreateCircleShape(bob,&sd,&c);
 b2DistanceJointDef def=b2DefaultDistanceJointDef();ENSURE(!def.enableLimit && def.maxLength==B2_HUGE);def.base.bodyIdA=anchor;def.base.bodyIdB=bob;def.enableSpring=true;def.enableLimit=true;def.minLength=1;def.maxLength=2;b2JointId joint=b2CreateDistanceJoint(id,&def);
 ENSURE(b2DistanceJoint_IsLimitEnabled(joint) && b2DistanceJoint_GetLowerLimit(joint)==1 && b2DistanceJoint_GetUpperLimit(joint)==2);
 for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-2,0.01f);
 b2Body_SetTransform(bob,{0.5f,0},b2Rot_identity);b2Body_SetLinearVelocity(bob,{0,0});for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-1,0.01f);
 b2Body_SetTransform(bob,{1.5f,0},b2Rot_identity);b2Body_SetLinearVelocity(bob,{0,0});b2World_EnableWarmStarting(id,false);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-1.5f,0.0001f);
 b2DistanceJoint_SetLimits(joint,1.25f,1.25f);for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-def.length,0.01f);
 // Equal limits use the rigid branch and therefore its rest length, matching the distance definition.
 b2DistanceJoint_SetLimits(joint,2,1);ENSURE(b2DistanceJoint_GetLowerLimit(joint)==1 && b2DistanceJoint_GetUpperLimit(joint)==2);
 b2DistanceJoint_EnableLimit(joint,false);ENSURE(!b2DistanceJoint_IsLimitEnabled(joint));b2Body_SetTransform(bob,{3,0},b2Rot_identity);b2Body_SetLinearVelocity(bob,{1,0});b2World_Step(id,0.1f,1);ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-3.1f,0.0001f);
 b2DistanceJoint_EnableLimit(joint,true);b2DistanceJoint_SetLimits(joint,2,3);b2DistanceJoint_EnableSpring(joint,false);for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-1,0.01f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int DistanceJointMotorTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId anchor=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={2,0};b2BodyId bob=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.density=1/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(bob,&sd,&c);
 b2DistanceJointDef def=b2DefaultDistanceJointDef();ENSURE(!def.enableMotor && def.maxMotorForce==0 && def.motorSpeed==0);def.base.bodyIdA=anchor;def.base.bodyIdB=bob;def.enableSpring=true;def.enableMotor=true;def.maxMotorForce=10;def.motorSpeed=2;b2JointId joint=b2CreateDistanceJoint(id,&def);
 ENSURE(b2DistanceJoint_IsMotorEnabled(joint) && b2DistanceJoint_GetMaxMotorForce(joint)==10 && b2DistanceJoint_GetTargetMotorSpeed(joint)==2);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetLinearVelocity(bob).x-0.1f,0.0001f);ENSURE_SMALL(b2DistanceJoint_GetCurrentMotorForce(joint)-10,0.0001f);
 for(int i=0;i<30;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2Body_GetLinearVelocity(bob).x-2,0.0001f);
 b2DistanceJoint_SetTargetMotorSpeed(joint,-1);b2DistanceJoint_SetMaxMotorForce(joint,5);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2DistanceJoint_GetCurrentMotorForce(joint)+5,0.0001f);
 for(int i=0;i<90;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2Body_GetLinearVelocity(bob).x+1,0.0001f);
 b2DistanceJoint_EnableMotor(joint,false);ENSURE(!b2DistanceJoint_IsMotorEnabled(joint) && b2DistanceJoint_GetCurrentMotorForce(joint)==0);float x=b2Body_GetPosition(bob).x;b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetPosition(bob).x-x+0.01f,0.0001f);
 b2DistanceJoint_EnableMotor(joint,true);b2DistanceJoint_SetMaxMotorForce(joint,0);b2DistanceJoint_SetTargetMotorSpeed(joint,100);b2Body_SetLinearVelocity(bob,{0,0});b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetLinearVelocity(bob).x,0.0001f);
 b2DistanceJoint_SetLimits(joint,1,2.5f);b2DistanceJoint_EnableLimit(joint,true);b2DistanceJoint_SetMaxMotorForce(joint,100);b2DistanceJoint_SetTargetMotorSpeed(joint,5);for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(b2DistanceJoint_GetCurrentLength(joint)<=2.51f);
 b2DistanceJoint_SetTargetMotorSpeed(joint,-5);for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(b2DistanceJoint_GetCurrentLength(joint)>=0.99f);
 b2DistanceJoint_EnableSpring(joint,false);b2DistanceJoint_SetTargetMotorSpeed(joint,5);for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(joint)-1,0.01f);
 b2DestroyJoint(joint);b2Body_SetTransform(bob,{2,0},b2Rot_identity);b2Body_SetLinearVelocity(bob,{0,0});def.maxMotorForce=1;def.motorSpeed=10;for(int i=0;i<26;++i) { ENSURE(b2CreateDistanceJoint(id,&def).index1>0); }
 b2World* world=b2_worlds+id.index1-1;ENSURE(world->constraintGraph.colors[B2_OVERFLOW_INDEX].jointSims.count>0);b2World_EnableWarmStarting(id,false);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetLinearVelocity(bob).x-0.26f,0.0001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

static float JointAnchorError(b2JointId joint)
{
 b2Pos a=b2Body_GetWorldPoint(b2Joint_GetBodyA(joint),b2Joint_GetLocalFrameA(joint).p);
 b2Pos b=b2Body_GetWorldPoint(b2Joint_GetBodyB(joint),b2Joint_GetLocalFrameB(joint).p);return b2Length(b2SubPos(b,a));
}
int RevoluteJointBasicTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();bd.position={100,200};b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={101,198};b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Polygon box=b2MakeBox(0.3f,1);b2CreatePolygonShape(b,&sd,&box);
 b2RevoluteJointDef def=b2DefaultRevoluteJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.base.localFrameA.p={1,0};def.base.localFrameB.p={0,2};b2JointId joint=b2CreateRevoluteJoint(id,&def);ENSURE(b2Joint_IsValid(joint) && b2Joint_GetType(joint)==b2_revoluteJoint);ENSURE_SMALL(JointAnchorError(joint),0.00001f);
 b2Body_SetAngularVelocity(b,2);b2Body_SetLinearVelocity(b,{4,0});b2World_Step(id,1.0f/60,4);ENSURE(b2RevoluteJoint_GetCurrentAngle(joint)>0.02f);ENSURE(b2RevoluteJoint_GetCurrentSpeed(joint)>1);for(int i=0;i<240;++i) { b2World_Step(id,1.0f/60,4);ENSURE(JointAnchorError(joint)<0.03f); }b2DestroyWorld(id);
 wd.gravity=b2Vec2_zero;id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={-1,0};a=b2CreateBody(id,&bd);b2CreatePolygonShape(a,&sd,&box);bd.position={1,0};b=b2CreateBody(id,&bd);b2CreatePolygonShape(b,&sd,&box);
 def=b2DefaultRevoluteJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.base.localFrameA.p={1,0};def.base.localFrameB.p={-1,0};joint=b2CreateRevoluteJoint(id,&def);b2Body_SetLinearVelocity(a,{-1,2});b2Body_SetLinearVelocity(b,{1,-2});for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4);ENSURE(JointAnchorError(joint)<0.02f); }ENSURE_SMALL(b2Body_GetPosition(a).x+b2Body_GetPosition(b).x,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(a).y+b2Body_GetLinearVelocity(b).y,0.0001f);
 b2DestroyWorld(id);id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.rotation={0,1};bd.angularVelocity=1;b=b2CreateBody(id,&bd);b2Circle circle={{0,0},1};b2CreateCircleShape(b,&sd,&circle);def=b2DefaultRevoluteJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.base.localFrameA.q={0,1};joint=b2CreateRevoluteJoint(id,&def);ENSURE_SMALL(b2RevoluteJoint_GetCurrentAngle(joint),0.00001f);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2RevoluteJoint_GetCurrentSpeed(joint)-1,0.00001f);ENSURE_SMALL(JointAnchorError(joint),0.00001f);
 b2Body_Disable(b);b2Body_Enable(b);b2World_Step(id,0.01f,1);ENSURE(JointAnchorError(joint)<0.01f);b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int RevoluteJointSpringTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.rotation=b2MakeRot(1.5f);b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2RevoluteJointDef def=b2DefaultRevoluteJointDef();ENSURE(!def.enableSpring && def.hertz==0 && def.dampingRatio==0 && def.targetAngle==0);def.base.bodyIdA=a;def.base.bodyIdB=b;def.enableSpring=true;def.hertz=2;def.dampingRatio=1;def.targetAngle=0.5f;b2JointId j=b2CreateRevoluteJoint(id,&def);
 ENSURE(b2RevoluteJoint_IsSpringEnabled(j) && b2RevoluteJoint_GetSpringHertz(j)==2 && b2RevoluteJoint_GetSpringDampingRatio(j)==1 && b2RevoluteJoint_GetTargetAngle(j)==0.5f);
 for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2RevoluteJoint_GetCurrentAngle(j)-0.5f,0.002f);ENSURE_SMALL(JointAnchorError(j),0.00001f);
 b2RevoluteJoint_SetTargetAngle(j,-0.8f);b2RevoluteJoint_SetSpringHertz(j,3);b2RevoluteJoint_SetSpringDampingRatio(j,0.8f);for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2RevoluteJoint_GetCurrentAngle(j)+0.8f,0.002f);
 b2RevoluteJoint_EnableSpring(j,false);ENSURE(!b2RevoluteJoint_IsSpringEnabled(j));b2Body_SetAngularVelocity(b,1);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2RevoluteJoint_GetCurrentSpeed(j)-1,0.00001f);
 b2RevoluteJoint_SetSpringHertz(j,0);b2RevoluteJoint_EnableSpring(j,true);b2World_EnableWarmStarting(id,false);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2RevoluteJoint_GetCurrentSpeed(j)-1,0.00001f);
 b2DestroyJoint(j);def.base.localFrameA.q={0,1};def.targetAngle=0;def.hertz=2;def.dampingRatio=1;j=b2CreateRevoluteJoint(id,&def);for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2RevoluteJoint_GetCurrentAngle(j),0.002f);ENSURE_SMALL(b2Rot_GetAngle(b2Body_GetRotation(b))-0.5f*B2_PI,0.002f);
 b2MotionLocks locks={false,false,true};b2Body_SetMotionLocks(b,locks);float angle=b2Rot_GetAngle(b2Body_GetRotation(b));b2RevoluteJoint_SetTargetAngle(j,1);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2Rot_GetAngle(b2Body_GetRotation(b))-angle,0.00001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int RevoluteJointLimitTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.rotation=b2MakeRot(1.2f);b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2RevoluteJointDef def=b2DefaultRevoluteJointDef();ENSURE(!def.enableLimit);def.base.bodyIdA=a;def.base.bodyIdB=b;def.enableLimit=true;def.lowerAngle=-0.5f;def.upperAngle=0.5f;b2JointId j=b2CreateRevoluteJoint(id,&def);ENSURE(b2RevoluteJoint_IsLimitEnabled(j) && b2RevoluteJoint_GetLowerLimit(j)==-0.5f && b2RevoluteJoint_GetUpperLimit(j)==0.5f);
 for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2RevoluteJoint_GetCurrentAngle(j)-0.5f,0.01f);
 b2Body_SetTransform(b,{0,0},b2MakeRot(-1.2f));b2Body_SetAngularVelocity(b,0);b2World_EnableWarmStarting(id,false);for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2RevoluteJoint_GetCurrentAngle(j)+0.5f,0.01f);
 b2Body_SetTransform(b,{0,0},b2Rot_identity);b2Body_SetAngularVelocity(b,1);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2RevoluteJoint_GetCurrentSpeed(j)-1,0.0001f);ENSURE(b2RevoluteJoint_GetCurrentAngle(j)>0.09f);
 b2RevoluteJoint_SetLimits(j,-10,10);ENSURE_SMALL(b2RevoluteJoint_GetLowerLimit(j)+0.99f*B2_PI,0.00001f);ENSURE_SMALL(b2RevoluteJoint_GetUpperLimit(j)-0.99f*B2_PI,0.00001f);
 b2RevoluteJoint_SetLimits(j,-0.2f,0.4f);b2RevoluteJoint_EnableSpring(j,true);b2RevoluteJoint_SetTargetAngle(j,1);b2RevoluteJoint_SetSpringHertz(j,2);b2RevoluteJoint_SetSpringDampingRatio(j,1);for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(b2RevoluteJoint_GetCurrentAngle(j)<=0.42f && b2RevoluteJoint_GetCurrentAngle(j)>=0.39f);
 b2RevoluteJoint_SetTargetAngle(j,-1);for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(b2RevoluteJoint_GetCurrentAngle(j)>=-0.22f && b2RevoluteJoint_GetCurrentAngle(j)<=-0.19f);
 b2RevoluteJoint_EnableLimit(j,false);ENSURE(!b2RevoluteJoint_IsLimitEnabled(j));b2RevoluteJoint_EnableSpring(j,false);b2Body_SetAngularVelocity(b,1);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2RevoluteJoint_GetCurrentSpeed(j)-1,0.0001f);ENSURE_SMALL(JointAnchorError(j),0.00001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int RevoluteJointMotorTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.density=2/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2RevoluteJointDef def=b2DefaultRevoluteJointDef();ENSURE(!def.enableMotor && def.maxMotorTorque==0 && def.motorSpeed==0);def.base.bodyIdA=a;def.base.bodyIdB=b;def.enableMotor=true;def.maxMotorTorque=2;def.motorSpeed=2;b2JointId j=b2CreateRevoluteJoint(id,&def);ENSURE(b2RevoluteJoint_IsMotorEnabled(j) && b2RevoluteJoint_GetMaxMotorTorque(j)==2 && b2RevoluteJoint_GetTargetMotorSpeed(j)==2);
 b2World_Step(id,0.01f,1);ENSURE_SMALL(b2RevoluteJoint_GetCurrentSpeed(j)-0.02f,0.0001f);ENSURE_SMALL(b2RevoluteJoint_GetCurrentMotorTorque(j)-2,0.0001f);for(int i=0;i<150;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2RevoluteJoint_GetCurrentSpeed(j)-2,0.0001f);
 b2RevoluteJoint_SetTargetMotorSpeed(j,-1);b2RevoluteJoint_SetMaxMotorTorque(j,1);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2RevoluteJoint_GetCurrentMotorTorque(j)+1,0.0001f);for(int i=0;i<350;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2RevoluteJoint_GetCurrentSpeed(j)+1,0.0001f);
 b2Body_SetTransform(b,{0,0},b2Rot_identity);b2Body_SetAngularVelocity(b,0);b2World_EnableWarmStarting(id,false);b2RevoluteJoint_EnableLimit(j,true);b2RevoluteJoint_SetLimits(j,-0.3f,0.4f);b2RevoluteJoint_SetMaxMotorTorque(j,10);b2RevoluteJoint_SetTargetMotorSpeed(j,100);for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(b2RevoluteJoint_GetCurrentAngle(j)<=0.42f && b2AbsFloat(b2RevoluteJoint_GetCurrentMotorTorque(j))<=10.001f);
 b2RevoluteJoint_SetTargetMotorSpeed(j,-100);for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(b2RevoluteJoint_GetCurrentAngle(j)>=-0.32f);
 b2RevoluteJoint_EnableMotor(j,false);ENSURE(!b2RevoluteJoint_IsMotorEnabled(j) && b2RevoluteJoint_GetCurrentMotorTorque(j)==0);b2RevoluteJoint_EnableLimit(j,false);b2Body_SetAngularVelocity(b,1);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2RevoluteJoint_GetCurrentSpeed(j)-1,0.0001f);
 b2RevoluteJoint_EnableMotor(j,true);b2RevoluteJoint_SetMaxMotorTorque(j,0);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2RevoluteJoint_GetCurrentSpeed(j)-1,0.0001f);
 b2DestroyJoint(j);b2Body_SetAngularVelocity(b,0);def.maxMotorTorque=1;def.motorSpeed=10;for(int i=0;i<26;++i) { ENSURE(b2CreateRevoluteJoint(id,&def).index1>0); }b2World* world=b2_worlds+id.index1-1;ENSURE(world->constraintGraph.colors[B2_OVERFLOW_INDEX].jointSims.count>0);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-0.26f,0.0001f);ENSURE_SMALL(JointAnchorError(b2Body_GetJointCount(b)>0?b2JointId{1,world->worldId,world->joints.data[0].generation}:b2_nullJointId),0.00001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

static b2Vec2 PrismaticErrors(b2JointId j)
{
 b2BodyId a=b2Joint_GetBodyA(j),b=b2Joint_GetBodyB(j);b2Transform fa=b2Joint_GetLocalFrameA(j),fb=b2Joint_GetLocalFrameB(j);
 b2Rot qa=b2MulRot(b2Body_GetRotation(a),fa.q),qb=b2MulRot(b2Body_GetRotation(b),fb.q);b2Vec2 axis=b2RotateVector(qa,{1,0});
 return {b2Dot(b2LeftPerp(axis),b2SubPos(b2Body_GetWorldPoint(b,fb.p),b2Body_GetWorldPoint(a,fa.p))),b2RelativeAngle(qa,qb)};
}
int PrismaticJointBasicTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();bd.position={100,200};b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={101,203};bd.rotation={0,1};bd.linearVelocity={2,1};bd.angularVelocity=3;b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},0.5f};b2CreateCircleShape(b,&sd,&c);
 b2PrismaticJointDef def=b2DefaultPrismaticJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.base.localFrameA.q={0,1};def.base.localFrameB.p={1,1};b2JointId j=b2CreatePrismaticJoint(id,&def);ENSURE(b2Joint_IsValid(j) && b2Joint_GetType(j)==b2_prismaticJoint);ENSURE_SMALL(b2PrismaticJoint_GetCurrentTranslation(j)-4,0.00001f);
 for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4);b2Vec2 error=PrismaticErrors(j);ENSURE(b2AbsFloat(error.x)<0.02f && b2AbsFloat(error.y)<0.02f); }ENSURE_SMALL(b2PrismaticJoint_GetCurrentSpeed(j)-1,0.0001f);ENSURE_SMALL(b2PrismaticJoint_GetCurrentTranslation(j)-6,0.005f);
 b2MotionLocks locks={false,false,true};b2Body_SetMotionLocks(b,locks);b2Body_SetLinearVelocity(b,{2,1});b2World_Step(id,0.1f,4);ENSURE_SMALL(PrismaticErrors(j).x,0.02f);b2DestroyWorld(id);
 id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={-1,-0.5f};a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);bd.position={1,0.5f};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);def=b2DefaultPrismaticJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;j=b2CreatePrismaticJoint(id,&def);b2Body_SetLinearVelocity(a,{-1,2});b2Body_SetLinearVelocity(b,{1,-2});b2Body_SetAngularVelocity(a,1);b2Body_SetAngularVelocity(b,-1);
 for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }b2Vec2 error=PrismaticErrors(j);ENSURE_SMALL(error.x,0.02f);ENSURE_SMALL(error.y,0.02f);ENSURE_SMALL(b2Body_GetPosition(a).x+b2Body_GetPosition(b).x,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(a).y+b2Body_GetLinearVelocity(b).y,0.0001f);
 b2Body_Disable(b);b2Body_Enable(b);b2World_Step(id,0.01f,4);error=PrismaticErrors(j);ENSURE_SMALL(error.x,0.02f);ENSURE_SMALL(error.y,0.02f);b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int PrismaticJointSpringTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();bd.position={100,200};b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={98,200};b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},0.5f};b2CreateCircleShape(b,&sd,&c);
 b2PrismaticJointDef def=b2DefaultPrismaticJointDef();ENSURE(!def.enableSpring && def.targetTranslation==0);def.base.bodyIdA=a;def.base.bodyIdB=b;def.enableSpring=true;def.hertz=2;def.dampingRatio=1;def.targetTranslation=2;b2JointId j=b2CreatePrismaticJoint(id,&def);ENSURE(b2PrismaticJoint_IsSpringEnabled(j) && b2PrismaticJoint_GetSpringHertz(j)==2 && b2PrismaticJoint_GetSpringDampingRatio(j)==1 && b2PrismaticJoint_GetTargetTranslation(j)==2);
 for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2PrismaticJoint_GetCurrentTranslation(j)-2,0.005f);
 b2PrismaticJoint_SetTargetTranslation(j,-1);b2PrismaticJoint_SetSpringHertz(j,3);b2PrismaticJoint_SetSpringDampingRatio(j,0.8f);for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2PrismaticJoint_GetCurrentTranslation(j)+1,0.005f);
 b2PrismaticJoint_EnableSpring(j,false);ENSURE(!b2PrismaticJoint_IsSpringEnabled(j));b2Body_SetLinearVelocity(b,{1,0});b2World_Step(id,0.1f,4);ENSURE_SMALL(b2PrismaticJoint_GetCurrentSpeed(j)-1,0.0001f);
 b2PrismaticJoint_SetSpringHertz(j,0);b2PrismaticJoint_EnableSpring(j,true);b2World_EnableWarmStarting(id,false);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2PrismaticJoint_GetCurrentSpeed(j)-1,0.0001f);
 b2DestroyJoint(j);b2Body_SetTransform(b,{100,203},b2Rot_identity);b2Body_SetLinearVelocity(b,{0,0});def.base.localFrameA.q={0,1};def.base.localFrameB.q={0,1};def.targetTranslation=1;j=b2CreatePrismaticJoint(id,&def);for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2PrismaticJoint_GetCurrentTranslation(j)-1,0.005f);b2Vec2 error=PrismaticErrors(j);ENSURE_SMALL(error.x,0.005f);ENSURE_SMALL(error.y,0.005f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int PrismaticJointLimitTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={3,0};b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},0.5f};b2CreateCircleShape(b,&sd,&c);
 b2PrismaticJointDef def=b2DefaultPrismaticJointDef();ENSURE(!def.enableLimit);def.base.bodyIdA=a;def.base.bodyIdB=b;def.enableLimit=true;def.lowerTranslation=0;def.upperTranslation=2;b2JointId j=b2CreatePrismaticJoint(id,&def);ENSURE(b2PrismaticJoint_IsLimitEnabled(j) && b2PrismaticJoint_GetLowerLimit(j)==0 && b2PrismaticJoint_GetUpperLimit(j)==2);
 for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2PrismaticJoint_GetCurrentTranslation(j)-2,0.01f);
 b2World_EnableWarmStarting(id,false);b2Body_SetTransform(b,{-1,0},b2Rot_identity);b2Body_SetLinearVelocity(b,{0,0});for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2PrismaticJoint_GetCurrentTranslation(j),0.01f);
 b2Body_SetTransform(b,{1,0},b2Rot_identity);b2Body_SetLinearVelocity(b,{1,0});b2World_Step(id,0.1f,4);ENSURE_SMALL(b2PrismaticJoint_GetCurrentTranslation(j)-1.1f,0.0001f);
 b2PrismaticJoint_SetLimits(j,0.5f,0.5f);for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2PrismaticJoint_GetCurrentTranslation(j)-0.5f,0.01f);
 b2PrismaticJoint_SetLimits(j,-0.5f,0.5f);b2PrismaticJoint_EnableSpring(j,true);b2PrismaticJoint_SetTargetTranslation(j,2);b2PrismaticJoint_SetSpringHertz(j,2);b2PrismaticJoint_SetSpringDampingRatio(j,1);for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(b2PrismaticJoint_GetCurrentTranslation(j)<=0.52f && b2PrismaticJoint_GetCurrentTranslation(j)>=0.49f);
 b2PrismaticJoint_EnableLimit(j,false);ENSURE(!b2PrismaticJoint_IsLimitEnabled(j));for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2PrismaticJoint_GetCurrentTranslation(j)-2,0.005f);b2Vec2 error=PrismaticErrors(j);ENSURE_SMALL(error.x,0.0001f);ENSURE_SMALL(error.y,0.0001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int PrismaticJointMotorTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={1,0};b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.density=1/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2PrismaticJointDef def=b2DefaultPrismaticJointDef();ENSURE(!def.enableMotor && def.maxMotorForce==0 && def.motorSpeed==0);def.base.bodyIdA=a;def.base.bodyIdB=b;def.enableMotor=true;def.motorSpeed=2;def.maxMotorForce=10;b2JointId j=b2CreatePrismaticJoint(id,&def);ENSURE(b2PrismaticJoint_IsMotorEnabled(j) && b2PrismaticJoint_GetTargetMotorSpeed(j)==2 && b2PrismaticJoint_GetMaxMotorForce(j)==10);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2PrismaticJoint_GetCurrentMotorForce(j)-10,0.0001f);ENSURE_SMALL(b2PrismaticJoint_GetCurrentSpeed(j)-0.1f,0.0001f);
 for(int i=0;i<30;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2PrismaticJoint_GetCurrentSpeed(j)-2,0.0001f);
 b2PrismaticJoint_SetTargetMotorSpeed(j,-1);b2PrismaticJoint_SetMaxMotorForce(j,5);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2PrismaticJoint_GetCurrentMotorForce(j)+5,0.0001f);for(int i=0;i<90;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2PrismaticJoint_GetCurrentSpeed(j)+1,0.0001f);
 b2World_EnableWarmStarting(id,false);b2Body_SetTransform(b,{0,0},b2Rot_identity);b2Body_SetLinearVelocity(b,{0,0});b2PrismaticJoint_SetLimits(j,-0.5f,0.5f);b2PrismaticJoint_EnableLimit(j,true);b2PrismaticJoint_SetMaxMotorForce(j,100);b2PrismaticJoint_SetTargetMotorSpeed(j,10);for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(b2PrismaticJoint_GetCurrentTranslation(j)<=0.52f);
 b2PrismaticJoint_SetTargetMotorSpeed(j,-10);for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(b2PrismaticJoint_GetCurrentTranslation(j)>=-0.52f);
 b2PrismaticJoint_EnableMotor(j,false);ENSURE(!b2PrismaticJoint_IsMotorEnabled(j) && b2PrismaticJoint_GetCurrentMotorForce(j)==0);b2PrismaticJoint_EnableLimit(j,false);b2Body_SetLinearVelocity(b,{1,0});b2World_Step(id,0.1f,4);ENSURE_SMALL(b2PrismaticJoint_GetCurrentSpeed(j)-1,0.0001f);b2PrismaticJoint_EnableMotor(j,true);b2PrismaticJoint_SetMaxMotorForce(j,0);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2PrismaticJoint_GetCurrentSpeed(j)-1,0.0001f);
 b2DestroyJoint(j);b2Body_SetLinearVelocity(b,{0,0});def.maxMotorForce=1;def.motorSpeed=10;b2JointId first={};for(int i=0;i<26;++i) { b2JointId next=b2CreatePrismaticJoint(id,&def);ENSURE(next.index1>0);if(i==0) { first=next; } }b2World* world=b2_worlds+id.index1-1;ENSURE(world->constraintGraph.colors[B2_OVERFLOW_INDEX].jointSims.count>0);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-0.26f,0.0001f);b2Vec2 error=PrismaticErrors(first);ENSURE_SMALL(error.x,0.0001f);ENSURE_SMALL(error.y,0.0001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

static float JointAxisTranslation(b2JointId j)
{
 b2BodyId a=b2Joint_GetBodyA(j),b=b2Joint_GetBodyB(j);b2Transform fa=b2Joint_GetLocalFrameA(j),fb=b2Joint_GetLocalFrameB(j);b2Rot q=b2MulRot(b2Body_GetRotation(a),fa.q);
 return b2Dot(b2RotateVector(q,{1,0}),b2SubPos(b2Body_GetWorldPoint(b,fb.p),b2Body_GetWorldPoint(a,fa.p)));
}
int WheelJointSuspensionTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();bd.position={100,200};b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={100,202};bd.angularVelocity=3;b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2WheelJointDef def=b2DefaultWheelJointDef();ENSURE(def.enableSpring && def.hertz==1 && def.dampingRatio==0.7f);def.base.bodyIdA=a;def.base.bodyIdB=b;def.base.localFrameA.q={0,1};b2JointId j=b2CreateWheelJoint(id,&def);ENSURE(b2Joint_IsValid(j) && b2Joint_GetType(j)==b2_wheelJoint && b2WheelJoint_IsSpringEnabled(j));ENSURE(b2WheelJoint_GetSpringHertz(j)==1 && b2WheelJoint_GetSpringDampingRatio(j)==0.7f);
 for(int i=0;i<240;++i) { b2World_Step(id,1.0f/60,4);ENSURE(b2AbsFloat(PrismaticErrors(j).x)<0.01f); }ENSURE_SMALL(JointAxisTranslation(j),0.005f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-3,0.0001f);
 b2WheelJoint_EnableSpring(j,false);ENSURE(!b2WheelJoint_IsSpringEnabled(j));b2Body_SetLinearVelocity(b,{1,2});b2World_Step(id,0.1f,4);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(b).y-2,0.0001f);
 b2WheelJoint_SetSpringHertz(j,0);b2WheelJoint_EnableSpring(j,true);b2World_EnableWarmStarting(id,false);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2Body_GetLinearVelocity(b).y-2,0.0001f);
 b2WheelJoint_SetSpringHertz(j,2);b2WheelJoint_SetSpringDampingRatio(j,1);b2World_SetGravity(id,{0,-10});b2Body_SetTransform(b,{100,198},b2Rot_identity);b2Body_SetLinearVelocity(b,{0,0});for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }float soft=-JointAxisTranslation(j);ENSURE(soft>0.001f && soft<0.2f);
 b2WheelJoint_SetSpringHertz(j,4);for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }float stiff=-JointAxisTranslation(j);ENSURE(stiff>0 && stiff<0.6f*soft);ENSURE_SMALL(PrismaticErrors(j).x,0.01f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-3,0.0001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int WheelJointLimitTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={0,3};bd.angularVelocity=3;b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},0.5f};b2CreateCircleShape(b,&sd,&c);
 b2WheelJointDef def=b2DefaultWheelJointDef();ENSURE(!def.enableLimit);def.base.bodyIdA=a;def.base.bodyIdB=b;def.base.localFrameA.q={0,1};def.enableSpring=false;def.enableLimit=true;def.lowerTranslation=1;def.upperTranslation=2;b2JointId j=b2CreateWheelJoint(id,&def);ENSURE(b2WheelJoint_IsLimitEnabled(j) && b2WheelJoint_GetLowerLimit(j)==1 && b2WheelJoint_GetUpperLimit(j)==2);
 for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(JointAxisTranslation(j)-2,0.01f);
 b2World_EnableWarmStarting(id,false);b2Body_SetTransform(b,{0,0.5f},b2Rot_identity);b2Body_SetLinearVelocity(b,{0,0});for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(JointAxisTranslation(j)-1,0.01f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-3,0.0001f);
 b2Body_SetTransform(b,{0,1.5f},b2Rot_identity);b2Body_SetLinearVelocity(b,{0,20});b2World_Step(id,0.1f,1);ENSURE(JointAxisTranslation(j)<=2.01f);
 b2WheelJoint_SetLimits(j,1,1);b2Body_SetLinearVelocity(b,{0,0});for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(JointAxisTranslation(j)-1,0.01f);
 b2WheelJoint_SetLimits(j,1,2);b2WheelJoint_EnableSpring(j,true);b2WheelJoint_SetSpringHertz(j,2);b2WheelJoint_SetSpringDampingRatio(j,1);for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(JointAxisTranslation(j)>=0.98f && JointAxisTranslation(j)<=1.01f);
 b2WheelJoint_EnableLimit(j,false);ENSURE(!b2WheelJoint_IsLimitEnabled(j));for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(JointAxisTranslation(j),0.005f);ENSURE_SMALL(PrismaticErrors(j).x,0.005f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-3,0.0001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int WheelJointMotorTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.density=2/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2WheelJointDef def=b2DefaultWheelJointDef();ENSURE(!def.enableMotor && def.maxMotorTorque==0 && def.motorSpeed==0);def.base.bodyIdA=a;def.base.bodyIdB=b;def.base.localFrameA.q={0,1};def.enableSpring=false;def.enableMotor=true;def.motorSpeed=2;def.maxMotorTorque=2;b2JointId j=b2CreateWheelJoint(id,&def);ENSURE(b2WheelJoint_IsMotorEnabled(j) && b2WheelJoint_GetTargetMotorSpeed(j)==2 && b2WheelJoint_GetMaxMotorTorque(j)==2);
 b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-0.02f,0.0001f);ENSURE_SMALL(b2WheelJoint_GetCurrentMotorTorque(j)-2,0.0001f);for(int i=0;i<150;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2Body_GetAngularVelocity(b)-2,0.0001f);
 b2WheelJoint_SetTargetMotorSpeed(j,-1);b2WheelJoint_SetMaxMotorTorque(j,1);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2WheelJoint_GetCurrentMotorTorque(j)+1,0.0001f);for(int i=0;i<350;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2Body_GetAngularVelocity(b)+1,0.0001f);
 b2World_EnableWarmStarting(id,false);b2Body_SetTransform(b,{0,3},b2Rot_identity);b2Body_SetLinearVelocity(b,{0,0});b2WheelJoint_EnableLimit(j,true);b2WheelJoint_SetLimits(j,-0.5f,0.5f);b2WheelJoint_EnableSpring(j,true);b2WheelJoint_SetSpringHertz(j,2);b2WheelJoint_SetSpringDampingRatio(j,1);b2WheelJoint_SetTargetMotorSpeed(j,2);b2WheelJoint_SetMaxMotorTorque(j,10);for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(JointAxisTranslation(j),0.01f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-2,0.0001f);ENSURE_SMALL(PrismaticErrors(j).x,0.005f);
 b2WheelJoint_EnableMotor(j,false);ENSURE(!b2WheelJoint_IsMotorEnabled(j) && b2WheelJoint_GetCurrentMotorTorque(j)==0);b2Body_SetAngularVelocity(b,1);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-1,0.0001f);b2WheelJoint_EnableMotor(j,true);b2WheelJoint_SetMaxMotorTorque(j,0);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-1,0.0001f);
 b2DestroyJoint(j);b2Body_SetAngularVelocity(b,0);def.maxMotorTorque=1;def.motorSpeed=10;b2JointId first={};for(int i=0;i<26;++i) { b2JointId next=b2CreateWheelJoint(id,&def);ENSURE(next.index1>0);if(i==0) { first=next; } }b2World* world=b2_worlds+id.index1-1;ENSURE(world->constraintGraph.colors[B2_OVERFLOW_INDEX].jointSims.count>0);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-0.26f,0.0001f);ENSURE_SMALL(PrismaticErrors(first).x,0.0001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int WeldJointTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();bd.position={100,200};b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={101,198};bd.linearVelocity={1,2};bd.angularVelocity=3;b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Polygon box=b2MakeBox(0.3f,1);b2CreatePolygonShape(b,&sd,&box);
 b2WeldJointDef def=b2DefaultWeldJointDef();ENSURE(def.linearHertz==0 && def.angularHertz==0);def.base.bodyIdA=a;def.base.bodyIdB=b;def.base.localFrameA.p={1,0};def.base.localFrameB.p={0,2};def.base.localFrameA.q=b2MakeRot(0.3f);def.base.localFrameB.q=def.base.localFrameA.q;b2JointId j=b2CreateWeldJoint(id,&def);ENSURE(b2Joint_IsValid(j) && b2Joint_GetType(j)==b2_weldJoint);
 for(int i=0;i<240;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(JointAnchorError(j),0.01f);ENSURE_SMALL(PrismaticErrors(j).y,0.01f);b2DestroyWorld(id);
 wd.gravity=b2Vec2_zero;id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={-1,0};a=b2CreateBody(id,&bd);b2CreatePolygonShape(a,&sd,&box);bd.position={1,0};b=b2CreateBody(id,&bd);b2CreatePolygonShape(b,&sd,&box);def=b2DefaultWeldJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.base.localFrameA.p={1,0};def.base.localFrameB.p={-1,0};j=b2CreateWeldJoint(id,&def);b2Body_SetLinearVelocity(a,{-1,1});b2Body_SetLinearVelocity(b,{1,-1});for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(JointAnchorError(j),0.02f);ENSURE_SMALL(PrismaticErrors(j).y,0.01f);ENSURE_SMALL(b2Body_GetPosition(a).x+b2Body_GetPosition(b).x,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(a).y+b2Body_GetLinearVelocity(b).y,0.0001f);b2DestroyWorld(id);
 id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={1,0};bd.rotation=b2MakeRot(1);b=b2CreateBody(id,&bd);b2Circle circle={{0,0},1};b2CreateCircleShape(b,&sd,&circle);def=b2DefaultWeldJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.linearHertz=2;def.linearDampingRatio=1;j=b2CreateWeldJoint(id,&def);b2WeldJoint_SetAngularHertz(j,3);b2WeldJoint_SetAngularDampingRatio(j,0.8f);ENSURE(b2WeldJoint_GetLinearHertz(j)==2 && b2WeldJoint_GetLinearDampingRatio(j)==1 && b2WeldJoint_GetAngularHertz(j)==3 && b2WeldJoint_GetAngularDampingRatio(j)==0.8f);
 for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(JointAnchorError(j),0.005f);ENSURE_SMALL(PrismaticErrors(j).y,0.005f);
 b2World_SetGravity(id,{0,-10});for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }float soft=JointAnchorError(j);ENSURE(soft>0.001f && soft<0.2f);b2WeldJoint_SetLinearHertz(j,4);b2WeldJoint_SetLinearDampingRatio(j,0.8f);for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(JointAnchorError(j)<0.6f*soft);
 b2WeldJoint_SetLinearHertz(j,0);b2WeldJoint_SetAngularHertz(j,0);for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(JointAnchorError(j)<0.2f*soft);ENSURE_SMALL(PrismaticErrors(j).y,0.005f);b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int MotorJointVelocityTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.density=2/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2MotorJointDef def=b2DefaultMotorJointDef();ENSURE(def.maxVelocityForce==0 && def.maxVelocityTorque==0);def.base.bodyIdA=a;def.base.bodyIdB=b;def.linearVelocity={3,4};def.maxVelocityForce=10;def.angularVelocity=2;def.maxVelocityTorque=2;b2JointId j=b2CreateMotorJoint(id,&def);ENSURE(b2Joint_IsValid(j) && b2Joint_GetType(j)==b2_motorJoint);ENSURE(b2MotorJoint_GetLinearVelocity(j).x==3 && b2MotorJoint_GetAngularVelocity(j)==2 && b2MotorJoint_GetMaxVelocityForce(j)==10 && b2MotorJoint_GetMaxVelocityTorque(j)==2);
 b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-0.03f,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(b).y-0.04f,0.0001f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-0.02f,0.0001f);for(int i=0;i<150;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-3,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(b).y-4,0.0001f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-2,0.0001f);
 b2MotorJoint_SetLinearVelocity(j,{-2,0});b2MotorJoint_SetAngularVelocity(j,-1);b2MotorJoint_SetMaxVelocityForce(j,20);b2MotorJoint_SetMaxVelocityTorque(j,5);for(int i=0;i<120;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2Body_GetLinearVelocity(b).x+2,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(b).y,0.0001f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)+1,0.0001f);
 b2MotorJoint_SetMaxVelocityForce(j,0);b2MotorJoint_SetMaxVelocityTorque(j,0);b2World_EnableWarmStarting(id,false);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x+2,0.0001f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)+1,0.0001f);
 b2DestroyJoint(j);b2Body_SetLinearVelocity(b,{0,0});b2Body_SetAngularVelocity(b,0);def.linearVelocity={10,0};def.angularVelocity=10;def.maxVelocityForce=1;def.maxVelocityTorque=1;for(int i=0;i<26;++i) { ENSURE(b2CreateMotorJoint(id,&def).index1>0); }b2World* world=b2_worlds+id.index1-1;ENSURE(world->constraintGraph.colors[B2_OVERFLOW_INDEX].jointSims.count>0);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-0.13f,0.0001f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-0.26f,0.0001f);b2DestroyWorld(id);
 id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={-2,0};a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);bd.position={2,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);def=b2DefaultMotorJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.maxVelocityForce=100;def.maxVelocityTorque=100;def.linearVelocity={2,0};def.angularVelocity=2;j=b2CreateMotorJoint(id,&def);for(int i=0;i<60;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2Body_GetLinearVelocity(a).x+1,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-1,0.0001f);ENSURE_SMALL(b2Body_GetAngularVelocity(a)+1,0.0001f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-1,0.0001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int MotorJointSpringTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.density=2/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2MotorJointDef def=b2DefaultMotorJointDef();ENSURE(def.maxSpringForce==0 && def.maxSpringTorque==0 && def.linearHertz==0 && def.angularHertz==0);def.base.bodyIdA=a;def.base.bodyIdB=b;def.base.localFrameA.p={3,4};def.base.localFrameA.q=b2MakeRot(0.8f);def.linearHertz=2;def.linearDampingRatio=1;def.maxSpringForce=10;def.angularHertz=2;def.angularDampingRatio=1;def.maxSpringTorque=2;b2JointId j=b2CreateMotorJoint(id,&def);
 b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-0.03f,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(b).y-0.04f,0.0001f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-0.02f,0.0001f);
 b2MotorJoint_SetMaxSpringForce(j,100);b2MotorJoint_SetMaxSpringTorque(j,100);b2MotorJoint_SetLinearHertz(j,3);b2MotorJoint_SetAngularHertz(j,3);b2MotorJoint_SetLinearDampingRatio(j,0.8f);b2MotorJoint_SetAngularDampingRatio(j,0.8f);ENSURE(b2MotorJoint_GetMaxSpringForce(j)==100 && b2MotorJoint_GetMaxSpringTorque(j)==100 && b2MotorJoint_GetLinearHertz(j)==3 && b2MotorJoint_GetAngularHertz(j)==3 && b2MotorJoint_GetLinearDampingRatio(j)==0.8f && b2MotorJoint_GetAngularDampingRatio(j)==0.8f);
 for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(JointAnchorError(j),0.005f);ENSURE_SMALL(PrismaticErrors(j).y,0.005f);
 b2MotorJoint_SetLinearVelocity(j,{1,0});b2MotorJoint_SetMaxVelocityForce(j,1);b2MotorJoint_SetAngularVelocity(j,1);b2MotorJoint_SetMaxVelocityTorque(j,1);for(int i=0;i<120;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(JointAnchorError(j)<0.1f && b2AbsFloat(PrismaticErrors(j).y)<0.1f);
 b2MotorJoint_SetMaxVelocityForce(j,0);b2MotorJoint_SetMaxVelocityTorque(j,0);b2MotorJoint_SetLinearHertz(j,0);b2MotorJoint_SetAngularHertz(j,0);b2World_EnableWarmStarting(id,false);b2Body_SetLinearVelocity(b,{1,0});b2Body_SetAngularVelocity(b,1);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-1,0.0001f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-1,0.0001f);
 b2MotorJoint_SetMaxSpringForce(j,-1);b2MotorJoint_SetMaxSpringTorque(j,-1);ENSURE(b2MotorJoint_GetMaxSpringForce(j)==0 && b2MotorJoint_GetMaxSpringTorque(j)==0);b2MotorJoint_SetLinearHertz(j,2);b2MotorJoint_SetAngularHertz(j,2);b2World_Step(id,0.1f,4);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-1,0.0001f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-1,0.0001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int MoverJointTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.density=2/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2MoverJointDef def=b2DefaultMoverJointDef();ENSURE(def.linearVelocity.x==0 && def.maxVelocityForce.x==0);def.base.bodyIdA=a;def.base.bodyIdB=b;def.linearVelocity={3,0};def.maxVelocityForce={10,10};b2JointId j=b2CreateMoverJoint(id,&def);ENSURE(b2Joint_IsValid(j) && b2Joint_GetType(j)==b2_moverJoint);ENSURE(b2MoverJoint_GetLinearVelocity(j).x==3 && b2MoverJoint_GetMaxVelocityForce(j).y==10);
 b2Body_SetAngularVelocity(b,1);b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-0.05f,0.0001f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-1,0.0001f);for(int i=0;i<80;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-3,0.0001f);ENSURE_SMALL(b2Body_GetAngularVelocity(b)-1,0.0001f);
 b2MoverJoint_SetLinearVelocity(j,{0,-2});b2MoverJoint_SetMaxVelocityForce(j,{20,20});for(int i=0;i<50;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2Body_GetLinearVelocity(b).x,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(b).y+2,0.0001f);
 b2MoverJoint_SetMaxVelocityForce(j,{0,0});b2Body_SetLinearVelocity(b,{1,1});b2World_Step(id,0.05f,4);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-1,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(b).y-1,0.0001f);
 b2DestroyWorld(id);id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={-1,0};a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);bd.position={1,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);def=b2DefaultMoverJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.linearVelocity={2,0};def.maxVelocityForce={100,100};j=b2CreateMoverJoint(id,&def);for(int i=0;i<30;++i) { b2World_Step(id,0.01f,1); }ENSURE_SMALL(b2Body_GetLinearVelocity(a).x+1,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x-1,0.0001f);
 def.linearVelocity.x=NAN;ENSURE(b2CreateMoverJoint(id,&def).index1==0);b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int PogoJointTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={0,1};b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.density=2/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2PogoJointDef def=b2DefaultPogoJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.normal={0,1};def.restLength=0.4f;def.hertz=4;def.dampingRatio=1;def.maxTensionForce=50;def.maxCompressionForce=50;def.impulse=0.2f;def.velocity=-0.1f;b2JointId j=b2CreatePogoJoint(id,&def);
 ENSURE(b2Joint_GetType(j)==b2_pogoJoint);ENSURE_SMALL(b2PogoJoint_GetCurrentLength(j)-1,0.0001f);ENSURE(b2PogoJoint_GetImpulse(j)==0.2f && b2PogoJoint_GetCurrentSpeed(j)==-0.1f && b2PogoJoint_GetRestLength(j)==0.4f);
 b2PogoJoint_SetSpringHertz(j,6);b2PogoJoint_SetSpringDampingRatio(j,0.7f);b2PogoJoint_SetRestLength(j,0.2f);ENSURE(b2PogoJoint_GetSpringHertz(j)==6 && b2PogoJoint_GetSpringDampingRatio(j)==0.7f && b2PogoJoint_GetRestLength(j)==0.2f);
 for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }float length=b2PogoJoint_GetCurrentLength(j);ENSURE(length<0.8f && length>0.2f);ENSURE(b2PogoJoint_GetCurrentSpeed(j)<0);
 b2PogoJoint_SetSpringHertz(j,0);b2Body_SetLinearVelocity(b,{0,-1});b2World_Step(id,0.05f,4);ENSURE_SMALL(b2Body_GetLinearVelocity(b).y+1,0.0001f);ENSURE(b2PogoJoint_GetImpulse(j)==0);
 def.restLength=-1;ENSURE(b2CreatePogoJoint(id,&def).index1==0);b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int FilterJointTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableSleep=false;bd.linearVelocity={2,0};b2BodyId a=b2CreateBody(id,&bd);bd.position={0.5f,0};bd.linearVelocity={-2,0};b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};b2CreateCircleShape(a,&sd,&c);b2CreateCircleShape(b,&sd,&c);
 b2World_Step(id,1.0f/60,4);ENSURE(b2World_GetCounters(id).contactCount>0);ENSURE(b2AbsFloat(b2Body_GetLinearVelocity(a).x)<1.5f);
 b2DestroyWorld(id);id=b2CreateWorld(&wd);bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableSleep=false;bd.linearVelocity={2,0};a=b2CreateBody(id,&bd);bd.position={0.5f,0};bd.linearVelocity={-2,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);b2CreateCircleShape(b,&sd,&c);
 b2FilterJointDef def=b2DefaultFilterJointDef();ENSURE(!def.base.collideConnected);def.base.bodyIdA=a;def.base.bodyIdB=b;b2JointId j=b2CreateFilterJoint(id,&def);ENSURE(b2Joint_IsValid(j) && b2Joint_GetType(j)==b2_filterJoint);
 b2World_Step(id,1.0f/60,4);ENSURE(b2World_GetCounters(id).contactCount==0);ENSURE_SMALL(b2Body_GetLinearVelocity(a).x-2,0.0001f);ENSURE_SMALL(b2Body_GetLinearVelocity(b).x+2,0.0001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int JointCollideConnectedTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.enableSleep=false;b2BodyId a=b2CreateBody(id,&bd);bd.position={0.5f,0};b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};b2CreateCircleShape(a,&sd,&c);b2CreateCircleShape(b,&sd,&c);
 b2FilterJointDef def=b2DefaultFilterJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;b2JointId j=b2CreateFilterJoint(id,&def);
 ENSURE(b2Joint_GetCollideConnected(j)==false);b2World_Step(id,1.0f/60,4);ENSURE(b2World_GetCounters(id).contactCount==0);
 b2Joint_SetCollideConnected(j,true);ENSURE(b2Joint_GetCollideConnected(j));b2World_Step(id,1.0f/60,4);ENSURE(b2World_GetCounters(id).contactCount>0);
 b2Joint_SetCollideConnected(j,false);ENSURE(b2Joint_GetCollideConnected(j)==false);ENSURE(b2World_GetCounters(id).contactCount==0);
 b2World_Step(id,1.0f/60,4);ENSURE(b2World_GetCounters(id).contactCount==0);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int JointLocalFrameTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={1,0};b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.density=2/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2DistanceJointDef def=b2DefaultDistanceJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.length=1;b2JointId j=b2CreateDistanceJoint(id,&def);
 ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(j)-1,0.0001f);
 b2Transform frame=b2Transform_identity;frame.p={0.5f,0};b2Joint_SetLocalFrameA(j,frame);ENSURE_SMALL(b2Joint_GetLocalFrameA(j).p.x-0.5f,0.0001f);ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(j)-0.5f,0.0001f);
 frame.p.x=NAN;b2Joint_SetLocalFrameA(j,frame);ENSURE_SMALL(b2Joint_GetLocalFrameA(j).p.x-0.5f,0.0001f);
 frame=b2Transform_identity;frame.p={0,0.25f};b2Joint_SetLocalFrameB(j,frame);ENSURE_SMALL(b2Joint_GetLocalFrameB(j).p.y-0.25f,0.0001f);ENSURE(b2DistanceJoint_GetCurrentLength(j)>0.55f);
 frame=b2Transform_identity;b2Joint_SetLocalFrameB(j,frame);ENSURE_SMALL(b2DistanceJoint_GetCurrentLength(j)-0.5f,0.0001f);
 for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(b2Body_GetPosition(b).x>1.2f);ENSURE(b2DistanceJoint_GetCurrentLength(j)>0.7f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int JointConstraintTuningTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={1,0};b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.density=2/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2DistanceJointDef def=b2DefaultDistanceJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.length=1;def.base.constraintHertz=0;def.base.constraintDampingRatio=0;b2JointId j=b2CreateDistanceJoint(id,&def);
 float hertz=0,damping=0;b2Joint_GetConstraintTuning(j,&hertz,&damping);ENSURE(hertz==0 && damping==0);
 b2Transform frame=b2Transform_identity;frame.p={0.5f,0};b2Joint_SetLocalFrameA(j,frame);
 for(int i=0;i<30;++i) { b2World_Step(id,1.0f/60,4); }ENSURE_SMALL(b2Body_GetPosition(b).x-1,0.0001f);
 b2Joint_SetConstraintTuning(j,-1,1);b2Joint_GetConstraintTuning(j,&hertz,&damping);ENSURE(hertz==0 && damping==0);
 b2Joint_SetConstraintTuning(j,30,1);b2Joint_GetConstraintTuning(j,&hertz,&damping);ENSURE(hertz==30 && damping==1);
 for(int i=0;i<180;++i) { b2World_Step(id,1.0f/60,4); }ENSURE(b2Body_GetPosition(b).x>1.2f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int JointReactionTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={2,0};b2BodyId b=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();sd.density=2/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2DistanceJointDef def=b2DefaultDistanceJointDef();def.base.bodyIdA=a;def.base.bodyIdB=b;def.length=1;def.base.constraintHertz=30;def.base.constraintDampingRatio=1;b2JointId j=b2CreateDistanceJoint(id,&def);
 b2World_Step(id,1.0f/60,4);ENSURE(b2Joint_GetConstraintTorque(j)==0);
 float length=b2DistanceJoint_GetCurrentLength(j);ENSURE_SMALL(b2Joint_GetLinearSeparation(j)-b2AbsFloat(length-1),0.0001f);ENSURE(b2Joint_GetAngularSeparation(j)==0);
 def.enableSpring=true;def.hertz=0;def.enableMotor=true;def.motorSpeed=5;def.maxMotorForce=50;b2JointId motor=b2CreateDistanceJoint(id,&def);
 b2World_Step(id,1.0f/60,4);ENSURE(b2Joint_GetConstraintForce(motor).x>0);ENSURE(b2Joint_GetLinearSeparation(motor)==0);
 b2FilterJointDef filter=b2DefaultFilterJointDef();filter.base.bodyIdA=a;filter.base.bodyIdB=b;b2JointId fj=b2CreateFilterJoint(id,&filter);
 ENSURE(b2Joint_GetConstraintForce(fj).x==0 && b2Joint_GetConstraintForce(fj).y==0 && b2Joint_GetConstraintTorque(fj)==0);ENSURE(b2Joint_GetLinearSeparation(fj)==0 && b2Joint_GetAngularSeparation(fj)==0);
 b2Body_SetTransform(b,{0,0},b2MakeRot(0.4f));b2RevoluteJointDef rd=b2DefaultRevoluteJointDef();rd.base.bodyIdA=a;rd.base.bodyIdB=b;rd.enableLimit=true;b2JointId rj=b2CreateRevoluteJoint(id,&rd);
 ENSURE_SMALL(b2Joint_GetAngularSeparation(rj)-b2Rot_GetAngle(b2Body_GetRotation(b)),0.0001f);ENSURE(b2AbsFloat(b2Joint_GetAngularSeparation(rj)-0.4f)<0.002f);ENSURE(b2Joint_GetLinearSeparation(rj)<0.001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}
int JointEventTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);b2BodyDef bd=b2DefaultBodyDef();b2BodyId a=b2CreateBody(id,&bd);bd.type=b2_dynamicBody;bd.enableSleep=false;bd.position={0,-1};b2BodyId b=b2CreateBody(id,&bd);
 b2ShapeDef sd=b2DefaultShapeDef();sd.density=2/B2_PI;b2Circle c={{0,0},1};b2CreateCircleShape(b,&sd,&c);
 b2DistanceJointDef def=b2DefaultDistanceJointDef();ENSURE(def.base.forceThreshold==FLT_MAX && def.base.torqueThreshold==FLT_MAX);
 def.base.bodyIdA=a;def.base.bodyIdB=b;def.length=1;def.base.constraintHertz=120;def.base.constraintDampingRatio=4;int marker=42;def.base.userData=&marker;b2JointId j=b2CreateDistanceJoint(id,&def);
 ENSURE(b2Joint_GetForceThreshold(j)==FLT_MAX && b2Joint_GetTorqueThreshold(j)==FLT_MAX);
 b2World_Step(id,1.0f/60,4);b2JointEvents events=b2World_GetJointEvents(id);ENSURE(events.count==0);
 // 非法阈值被拒绝，原值保持
 b2Joint_SetForceThreshold(j,-1);ENSURE(b2Joint_GetForceThreshold(j)==FLT_MAX);
 b2Joint_SetForceThreshold(j,NAN);ENSURE(b2Joint_GetForceThreshold(j)==FLT_MAX);
 b2Joint_SetTorqueThreshold(j,-1);ENSURE(b2Joint_GetTorqueThreshold(j)==FLT_MAX);
 b2Joint_SetTorqueThreshold(j,NAN);ENSURE(b2Joint_GetTorqueThreshold(j)==FLT_MAX);
 // 超过力阈值：每步一条事件，事件内容为关节 id 与用户数据
 b2Joint_SetForceThreshold(j,5);ENSURE(b2Joint_GetForceThreshold(j)==5);
 b2World_Step(id,1.0f/60,4);events=b2World_GetJointEvents(id);ENSURE(events.count==1);
 ENSURE(events.jointEvents[0].jointId.index1==j.index1 && events.jointEvents[0].jointId.world0==j.world0 && events.jointEvents[0].jointId.generation==j.generation);ENSURE(events.jointEvents[0].userData==&marker);
 b2World_Step(id,1.0f/60,4);ENSURE(b2World_GetJointEvents(id).count==1);
 // 高于阈值不再报告，同时验证上一步事件已清理
 b2Joint_SetForceThreshold(j,1000);b2World_Step(id,1.0f/60,4);ENSURE(b2World_GetJointEvents(id).count==0);
 // 零阈值报告所有唤醒关节
 b2Joint_SetForceThreshold(j,1000);b2Joint_SetTorqueThreshold(j,0);b2World_Step(id,1.0f/60,4);ENSURE(b2World_GetJointEvents(id).count==1);
 // 多个关节超过阈值时全部报告
 b2Joint_SetTorqueThreshold(j,FLT_MAX);b2Joint_SetForceThreshold(j,0);
 def.base.forceThreshold=0;def.base.torqueThreshold=0;b2JointId j2=b2CreateDistanceJoint(id,&def);ENSURE(b2Joint_GetForceThreshold(j2)==0 && b2Joint_GetTorqueThreshold(j2)==0);
 b2World_Step(id,1.0f/60,4);events=b2World_GetJointEvents(id);ENSURE(events.count==2);
 ENSURE((events.jointEvents[0].jointId.index1==j.index1 && events.jointEvents[1].jointId.index1==j2.index1) || (events.jointEvents[0].jointId.index1==j2.index1 && events.jointEvents[1].jointId.index1==j.index1));
 // 销毁关节后不再报告该关节
 b2DestroyJoint(j2);b2World_Step(id,1.0f/60,4);events=b2World_GetJointEvents(id);ENSURE(events.count==1);ENSURE(events.jointEvents[0].jointId.index1==j.index1);
 b2Joint_SetForceThreshold(j,FLT_MAX);b2World_Step(id,1.0f/60,4);ENSURE(b2World_GetJointEvents(id).count==0);
 ENSURE(b2Joint_IsValid(j));ENSURE(!b2Joint_IsValid(j2));
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

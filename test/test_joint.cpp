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

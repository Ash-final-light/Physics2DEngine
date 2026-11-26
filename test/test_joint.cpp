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

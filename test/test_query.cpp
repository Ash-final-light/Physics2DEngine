// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "test_macros.h"
#include "world.h"
#include "body.h"
#include "shape.h"

struct QueryHits { b2ShapeId shapes[16];int count;bool keepGoing;bool valid; };
static bool CollectOverlap(b2ShapeId shape,void* context)
{
    QueryHits* hits=static_cast<QueryHits*>(context);hits->valid&=b2Shape_IsValid(shape);
    if(hits->count<16) { hits->shapes[hits->count]=shape; }++hits->count;return hits->keepGoing;
}

int WorldOverlapAABBTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);
    b2World* world=b2_worlds+id.index1-1;b2Pos origin={1000,2000};b2BodyDef bd=b2DefaultBodyDef();b2ShapeDef sd=b2DefaultShapeDef();
    b2Circle circle={{0,0},0.2f};b2ShapeId shapes[4];
    for(int i=0;i<4;++i)
    {
        bd.type=i==0||i==3?b2_staticBody:i==1?b2_dynamicBody:b2_kinematicBody;bd.position={origin.x+i,origin.y};
        b2BodyId body=b2CreateBody(id,&bd);sd.filter.categoryBits=uint64_t(1)<<i;sd.filter.maskBits=i<2?uint64_t(1)<<i:UINT64_MAX;sd.isSensor=i==3;
        shapes[i]=b2CreateCircleShape(body,&sd,&circle);
    }
    bd.type=b2_dynamicBody;bd.position=origin;bd.isEnabled=false;b2BodyId disabled=b2CreateBody(id,&bd);b2CreateCircleShape(disabled,&sd,&circle);
    b2QueryFilter filter=b2DefaultQueryFilter();ENSURE(filter.categoryBits==1 && filter.maskBits==UINT64_MAX);
    QueryHits hits={};hits.keepGoing=true;hits.valid=true;b2AABB box={{-1,-1},{5,1}};
    b2TreeStats stats=b2World_OverlapAABB(id,origin,box,filter,CollectOverlap,&hits);ENSURE(hits.count==3 && hits.valid);ENSURE(stats.leafVisits>=3 && stats.nodeVisits>0);
    filter.categoryBits=2;filter.maskBits=2;hits.count=0;b2World_OverlapAABB(id,origin,box,filter,CollectOverlap,&hits);
    ENSURE(hits.count==1 && hits.shapes[0].index1==shapes[1].index1);
    filter.categoryBits=UINT64_MAX;filter.maskBits=UINT64_MAX;hits.count=0;b2World_OverlapAABB(id,origin,box,filter,CollectOverlap,&hits);ENSURE(hits.count==4);
    hits.count=0;b2World_OverlapAABB(id,origin,b2AABB{{20,20},{21,21}},filter,CollectOverlap,&hits);ENSURE(hits.count==0);
    // Restrict to the static tree to exercise callback termination without depending on tree order.
    filter.maskBits=1|8;hits.count=0;hits.keepGoing=false;b2World_OverlapAABB(id,origin,box,filter,CollectOverlap,&hits);ENSURE(hits.count==1);
    world->locked=true;hits.count=0;stats=b2World_OverlapAABB(id,origin,box,filter,CollectOverlap,&hits);world->locked=false;ENSURE(hits.count==0 && stats.nodeVisits==0);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int WorldOverlapShapeTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);
    b2Pos origin={1000,2000};b2BodyDef bd=b2DefaultBodyDef();b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},0.25f};
    bd.position={origin.x+0.8f,origin.y};b2BodyId body=b2CreateBody(id,&bd);sd.isSensor=true;b2ShapeId near=b2CreateCircleShape(body,&sd,&c);
    bd.position={origin.x+0.9f,origin.y+0.9f};body=b2CreateBody(id,&bd);c.radius=0.05f;sd.isSensor=false;b2CreateCircleShape(body,&sd,&c);
    bd.position={origin.x+3,origin.y};body=b2CreateBody(id,&bd);c.radius=0.25f;sd.filter.categoryBits=2;b2ShapeId far=b2CreateCircleShape(body,&sd,&c);
    bd.position={origin.x,origin.y-2};bd.rotation=b2MakeRot(0.5f*B2_PI);body=b2CreateBody(id,&bd);sd.filter.categoryBits=1;
    b2Polygon bar=b2MakeBox(1,0.1f);b2ShapeId rotated=b2CreatePolygonShape(body,&sd,&bar);
    QueryHits hits={};hits.keepGoing=true;hits.valid=true;b2QueryFilter filter=b2DefaultQueryFilter();
    b2Vec2 center={0,0};b2ShapeProxy proxy=b2MakeProxy(&center,1,1);
    b2World_OverlapAABB(id,origin,b2AABB{{-1,-1},{1,1}},filter,CollectOverlap,&hits);ENSURE(hits.count==3);
    hits.count=0;b2TreeStats stats=b2World_OverlapShape(id,origin,&proxy,filter,CollectOverlap,&hits);ENSURE(hits.count==2 && hits.valid && stats.leafVisits>=2);
    bool foundNear=false,foundRotated=false;
    for(int i=0;i<hits.count;++i) { foundNear|=hits.shapes[i].index1==near.index1;foundRotated|=hits.shapes[i].index1==rotated.index1; }
    ENSURE(foundNear && foundRotated);
    b2Vec2 segment[2]={{0,0},{3,0}};proxy=b2MakeProxy(segment,2,0.1f);hits.count=0;
    b2World_OverlapShape(id,origin,&proxy,filter,CollectOverlap,&hits);ENSURE(hits.count==2);
    filter.maskBits=2;hits.count=0;b2World_OverlapShape(id,origin,&proxy,filter,CollectOverlap,&hits);ENSURE(hits.count==1 && hits.shapes[0].index1==far.index1);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

struct RayHits { int count;int mode;int skipIndex;bool valid;b2ShapeId shape;b2Pos point;b2Vec2 normal;float fraction; };
static float CollectRay(b2ShapeId shape,b2Pos point,b2Vec2 normal,float fraction,void* context)
{
    RayHits* hits=static_cast<RayHits*>(context);++hits->count;hits->valid&=b2Shape_IsValid(shape)&&b2IsValidVec2(point)&&b2IsValidVec2(normal)&&fraction>=0&&fraction<=1;
    if(hits->mode==3 && shape.index1==hits->skipIndex) { return -1; }
    hits->shape=shape;hits->point=point;hits->normal=normal;hits->fraction=fraction;
    if(hits->mode==0) { return 1; }if(hits->mode==2) { return 0; }return fraction;
}

int WorldRayCastTest(void)
{
    size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);
    b2World* world=b2_worlds+id.index1-1;b2Pos origin={1000,2000};b2BodyDef bd=b2DefaultBodyDef();bd.position={1002,2000};b2ShapeDef sd=b2DefaultShapeDef();
    b2BodyId body=b2CreateBody(id,&bd);b2Circle circle={{0,0},0.5f};b2ShapeId near=b2CreateCircleShape(body,&sd,&circle);
    bd.type=b2_dynamicBody;bd.position={1005,2000};body=b2CreateBody(id,&bd);sd.filter.categoryBits=2;b2Polygon box=b2MakeBox(0.5f,0.5f);b2ShapeId middle=b2CreatePolygonShape(body,&sd,&box);
    bd.type=b2_kinematicBody;bd.position={1008,2000};body=b2CreateBody(id,&bd);sd.filter.categoryBits=4;b2Capsule capsule={{0,-0.5f},{0,0.5f},0.5f};b2ShapeId far=b2CreateCapsuleShape(body,&sd,&capsule);
    b2QueryFilter filter=b2DefaultQueryFilter();RayHits hits={};hits.valid=true;hits.mode=0;
    b2TreeStats stats=b2World_CastRay(id,origin,{10,0},filter,CollectRay,&hits);ENSURE(hits.count==3 && hits.valid && stats.leafVisits>=3);
    hits={};hits.valid=true;hits.mode=1;b2World_CastRay(id,origin,{10,0},filter,CollectRay,&hits);
    ENSURE(hits.count==1 && hits.shape.index1==near.index1);ENSURE_SMALL(hits.fraction-0.15f,0.00001f);
    ENSURE_SMALL(hits.point.x-1001.5f,0.00001f);ENSURE(hits.point.y==2000 && hits.normal.x==-1 && hits.normal.y==0);
    hits={};hits.valid=true;hits.mode=2;b2World_CastRay(id,origin,{10,0},filter,CollectRay,&hits);ENSURE(hits.count==1);
    hits={};hits.valid=true;hits.mode=3;hits.skipIndex=near.index1;b2World_CastRay(id,origin,{10,0},filter,CollectRay,&hits);ENSURE(hits.shape.index1==middle.index1 && hits.count>=2 && hits.count<=3);
    ENSURE_SMALL(hits.fraction-0.45f,0.00001f);
    hits={};hits.valid=true;filter.maskBits=4;b2World_CastRay(id,origin,{10,0},filter,CollectRay,&hits);ENSURE(hits.count==1 && hits.shape.index1==far.index1);
    hits.count=0;b2World_CastRay(id,origin,{0,10},filter,CollectRay,&hits);ENSURE(hits.count==0);
    world->locked=true;stats=b2World_CastRay(id,origin,{10,0},filter,CollectRay,&hits);world->locked=false;ENSURE(hits.count==0 && stats.nodeVisits==0);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int WorldRayClosestTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);
 b2BodyDef bd=b2DefaultBodyDef();b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{0,0},0.5f};
 bd.position={105,200};b2BodyId body=b2CreateBody(id,&bd);b2CreateCircleShape(body,&sd,&circle);
 bd.type=b2_dynamicBody;bd.position={102,200};body=b2CreateBody(id,&bd);sd.filter.categoryBits=2;b2ShapeId near=b2CreateCircleShape(body,&sd,&circle);
 b2QueryFilter filter=b2DefaultQueryFilter();b2RayResult r=b2World_CastRayClosest(id,{100,200},{10,0},filter);
 ENSURE(r.hit && r.shapeId.index1==near.index1 && r.leafVisits>=2);ENSURE_SMALL(r.fraction-0.15f,0.00001f);
 ENSURE(r.point.x==101.5f && r.point.y==200 && r.normal.x==-1);
 filter.maskBits=1;r=b2World_CastRayClosest(id,{100,200},{10,0},filter);ENSURE(r.hit);ENSURE_SMALL(r.fraction-0.45f,0.00001f);
 r=b2World_CastRayClosest(id,{100,210},{10,0},filter);ENSURE(!r.hit && r.shapeId.index1==0);
 b2World* world=b2_worlds+id.index1-1;world->locked=true;r=b2World_CastRayClosest(id,{100,200},{10,0},filter);world->locked=false;ENSURE(!r.hit && r.leafVisits==0);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int WorldShapeCastTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);
 b2BodyDef bd=b2DefaultBodyDef();bd.position={103,200};b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{0,0},0.5f};b2ShapeId near=b2CreateCircleShape(body,&sd,&circle);
 bd.type=b2_dynamicBody;bd.position={107,200};bd.rotation=b2MakeRot(0.5f*B2_PI);body=b2CreateBody(id,&bd);sd.filter.categoryBits=2;b2Polygon box=b2MakeBox(1,0.5f);b2ShapeId far=b2CreatePolygonShape(body,&sd,&box);
 b2Vec2 point={0,0};b2ShapeProxy proxy=b2MakeProxy(&point,1,0.5f);b2QueryFilter filter=b2DefaultQueryFilter();RayHits hits={};hits.valid=true;
 b2TreeStats stats=b2World_CastShape(id,{100,200},&proxy,{10,0},filter,CollectRay,&hits);ENSURE(hits.count==2 && hits.valid && stats.leafVisits>=2);
 hits={};hits.mode=1;hits.valid=true;b2World_CastShape(id,{100,200},&proxy,{10,0},filter,CollectRay,&hits);ENSURE(hits.count==1 && hits.shape.index1==near.index1);ENSURE_SMALL(hits.fraction-0.2f,0.002f);ENSURE(hits.normal.x < -0.99f);ENSURE_SMALL(hits.point.x-102.5f,0.02f);
 hits={};hits.mode=3;hits.skipIndex=near.index1;hits.valid=true;b2World_CastShape(id,{100,200},&proxy,{10,0},filter,CollectRay,&hits);ENSURE(hits.shape.index1==far.index1 && hits.count==2);ENSURE_SMALL(hits.fraction-0.6f,0.002f);
 hits={};hits.mode=2;hits.valid=true;b2World_CastShape(id,{100,200},&proxy,{10,0},filter,CollectRay,&hits);ENSURE(hits.count==1);
 filter.maskBits=2;hits={};hits.valid=true;b2World_CastShape(id,{100,200},&proxy,{10,0},filter,CollectRay,&hits);ENSURE(hits.count==1 && hits.shape.index1==far.index1);
 proxy.count=0;hits.count=0;b2World_CastShape(id,{100,200},&proxy,{10,0},filter,CollectRay,&hits);ENSURE(hits.count==0);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int AttachedShapePointTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);
 b2BodyDef bd=b2DefaultBodyDef();bd.position={100,200};bd.rotation={0,1};b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();
 b2Circle c={{2,0},0.5f};b2ShapeId circle=b2CreateCircleShape(body,&sd,&c);ENSURE(b2Shape_TestPoint(circle,{100,202}));ENSURE(!b2Shape_TestPoint(circle,{102,200}));
 b2Capsule cap={{-1,0},{1,0},0.5f};b2ShapeId capsule=b2CreateCapsuleShape(body,&sd,&cap);ENSURE(b2Shape_TestPoint(capsule,{100,201.25f}));ENSURE(!b2Shape_TestPoint(capsule,{100.75f,200}));
 b2Polygon box=b2MakeBox(2,0.25f);b2ShapeId polygon=b2CreatePolygonShape(body,&sd,&box);ENSURE(b2Shape_TestPoint(polygon,{100.1f,201.5f}));ENSURE(!b2Shape_TestPoint(polygon,{101,200}));
 b2Segment edge={{-1,0},{1,0}};b2ShapeId segment=b2CreateSegmentShape(body,&sd,&edge);ENSURE(!b2Shape_TestPoint(segment,{100,200}));
 b2Body_SetTransform(body,{20,30},b2Rot_identity);ENSURE(b2Shape_TestPoint(circle,{22,30}));ENSURE(!b2Shape_TestPoint(circle,{100,202}));
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int AttachedShapeRayTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);
 b2BodyDef bd=b2DefaultBodyDef();bd.position={100,200};bd.rotation={0,1};b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();
 b2Polygon box=b2MakeBox(2,0.5f);b2ShapeId shape=b2CreatePolygonShape(body,&sd,&box);b2WorldCastOutput r=b2Shape_CastRay(shape,{95,200},{10,0});
 ENSURE(r.hit && r.normal.x==-1 && r.normal.y==0);ENSURE_SMALL(r.fraction-0.45f,0.00001f);ENSURE(r.point.x==99.5f && r.point.y==200);
 r=b2Shape_CastRay(shape,{95,203},{10,0});ENSURE(!r.hit);
 b2Circle c={{3,0},0.5f};shape=b2CreateCircleShape(body,&sd,&c);r=b2Shape_CastRay(shape,{95,203},{10,0});ENSURE(r.hit);ENSURE_SMALL(r.fraction-0.45f,0.00001f);
 b2Segment edge={{-1,0},{1,0}};shape=b2CreateSegmentShape(body,&sd,&edge);
 r=b2Shape_CastRay(shape,{95,200},{10,0});ENSURE(r.hit && r.normal.x==-1);ENSURE_SMALL(r.fraction-0.5f,0.00001f);
 r=b2Shape_CastRay(shape,{105,200},{-10,0});ENSURE(r.hit && r.normal.x==1);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int ShapeClosestPointTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);
 b2BodyDef bd=b2DefaultBodyDef();bd.position={100,200};bd.rotation={0,1};b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();
 b2Circle c={{2,0},0.5f};b2ShapeId circle=b2CreateCircleShape(body,&sd,&c);b2Pos p=b2Shape_GetClosestPoint(circle,{103,202});ENSURE_SMALL(p.x-100.5f,0.00001f);ENSURE_SMALL(p.y-202,0.00001f);
 b2Polygon box=b2MakeBox(2,0.5f);b2ShapeId polygon=b2CreatePolygonShape(body,&sd,&box);p=b2Shape_GetClosestPoint(polygon,{103,201});ENSURE_SMALL(p.x-100.5f,0.00001f);ENSURE_SMALL(p.y-201,0.00001f);
 p=b2Shape_GetClosestPoint(polygon,{100,200});ENSURE_SMALL(p.x-100,0.00001f);ENSURE_SMALL(p.y-200,0.00001f);
 b2Segment edge={{-1,0},{1,0}};b2ShapeId segment=b2CreateSegmentShape(body,&sd,&edge);p=b2Shape_GetClosestPoint(segment,{103,204});ENSURE_SMALL(p.x-100,0.00001f);ENSURE_SMALL(p.y-201,0.00001f);
 b2Body_SetTransform(body,{20,30},b2Rot_identity);p=b2Shape_GetClosestPoint(circle,{25,30});ENSURE_SMALL(p.x-22.5f,0.00001f);ENSURE_SMALL(p.y-30,0.00001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int WorldExplosionTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);
 b2BodyId bodies[6];b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},0.5f};
 for(int i=0;i<6;++i) { b2BodyDef bd=b2DefaultBodyDef();bd.type=i==3?b2_staticBody:b2_dynamicBody;bd.position={100+(i==1?4.0f:i==2?8.0f:2.0f),200};bd.isAwake=i!=0;bd.isEnabled=i!=5;bodies[i]=b2CreateBody(id,&bd);sd.filter.categoryBits=i==4?2:1;b2CreateCircleShape(bodies[i],&sd,&c); }
 b2ExplosionDef e=b2DefaultExplosionDef();ENSURE(e.maskBits==UINT64_MAX && e.radius==0 && e.falloff==0 && e.impulsePerLength==0);
 e.maskBits=1;e.position={100,200};e.radius=2;e.falloff=2;e.impulsePerLength=2;b2World_Explode(id,&e);
 ENSURE(b2Body_IsAwake(bodies[0]));ENSURE_SMALL(b2Body_GetLinearVelocity(bodies[0]).x-8/B2_PI,0.00001f);ENSURE_SMALL(b2Body_GetLinearVelocity(bodies[1]).x-2/B2_PI,0.00001f);
 for(int i=2;i<6;++i) { ENSURE(b2LengthSquared(b2Body_GetLinearVelocity(bodies[i]))==0); }
 ENSURE(b2Body_GetAngularVelocity(bodies[0])==0);e.impulsePerLength=-2;b2World_Explode(id,&e);ENSURE_SMALL(b2Body_GetLinearVelocity(bodies[0]).x,0.00001f);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int ShapeWindTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
 b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.isAwake=false;b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},0.5f};b2ShapeId shape=b2CreateCircleShape(body,&sd,&c);
 b2Shape_ApplyWind(shape,{10,0},1,0,false);ENSURE(!b2Body_IsAwake(body));ENSURE(b2GetBodySim(world,b2GetBodyFullId(world,body))->force.x==0);
 b2Shape_ApplyWind(shape,{10,0},1,0,true);ENSURE(b2Body_IsAwake(body));b2BodySim* sim=b2GetBodySim(world,b2GetBodyFullId(world,body));ENSURE_SMALL(sim->force.x-61.25f,0.0001f);ENSURE(sim->force.y==0 && sim->torque==0);
 b2World_Step(id,0.01f,1);ENSURE_SMALL(b2Body_GetLinearVelocity(body).x-0.01f*61.25f*4/B2_PI,0.0001f);
 bd.isAwake=true;bd.position={10,0};body=b2CreateBody(id,&bd);b2Capsule cap={{-1,0},{1,0},0.5f};shape=b2CreateCapsuleShape(body,&sd,&cap);
 b2Shape_ApplyWind(shape,{10,0},0,0,false);sim=b2GetBodySim(world,b2GetBodyFullId(world,body));ENSURE_SMALL(sim->force.x-61.25f,0.0001f);sim->force=b2Vec2_zero;
 b2Body_SetTransform(body,{10,0},{0,1});b2Shape_ApplyWind(shape,{10,0},0,0,false);ENSURE_SMALL(sim->force.x-183.75f,0.0001f);
 sim->force=b2Vec2_zero;b2Shape_ApplyWind(shape,{10,10},0,1,false);ENSURE(b2AbsFloat(sim->force.x-sim->force.y)>1);
 bd.position={20,0};body=b2CreateBody(id,&bd);b2Polygon box=b2MakeBox(2,0.5f);shape=b2CreatePolygonShape(body,&sd,&box);b2Shape_ApplyWind(shape,{10,0},0,0,false);sim=b2GetBodySim(world,b2GetBodyFullId(world,body));ENSURE_SMALL(sim->force.x-61.25f,0.0001f);sim->force=b2Vec2_zero;
 b2Body_SetTransform(body,{20,0},{0,1});b2Shape_ApplyWind(shape,{10,0},0,0,false);ENSURE_SMALL(sim->force.x-245,0.0001f);
 bd.type=b2_staticBody;bd.position={30,0};body=b2CreateBody(id,&bd);shape=b2CreateCircleShape(body,&sd,&c);b2Shape_ApplyWind(shape,{10,0},0,0,true);ENSURE(b2GetBodySim(world,b2GetBodyFullId(world,body))->force.x==0);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int WorldMoverCastTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);b2ShapeDef sd=b2DefaultShapeDef();b2Polygon wall=b2MakeBox(0.1f,2);
 for(int i=0;i<3;++i) { b2BodyDef bd=b2DefaultBodyDef();bd.type=(b2BodyType)i;bd.position={1003.0f+2*i,2000};b2BodyId body=b2CreateBody(id,&bd);sd.filter.categoryBits=uint64_t(1)<<i;b2CreatePolygonShape(body,&sd,&wall); }
 b2BodyDef bd=b2DefaultBodyDef();bd.position={1000,2000};b2BodyId body=b2CreateBody(id,&bd);b2Circle c={{0,0},0.1f};sd.filter.categoryBits=8;b2CreateCircleShape(body,&sd,&c);
 b2Capsule mover={{0,-0.5f},{0,0.5f},0.5f};b2QueryFilter filter=b2DefaultQueryFilter();float f=b2World_CastMover(id,{1000,2000},&mover,{10,0},filter);ENSURE_SMALL(f-0.24f,0.002f);
 filter.maskBits=2;f=b2World_CastMover(id,{1000,2000},&mover,{10,0},filter);ENSURE_SMALL(f-0.44f,0.002f);
 filter.maskBits=4;f=b2World_CastMover(id,{1000,2000},&mover,{10,0},filter);ENSURE_SMALL(f-0.64f,0.002f);
 filter.maskBits=UINT64_MAX;f=b2World_CastMover(id,{1000,2000},&mover,{-10,0},filter);ENSURE(f==1);
 b2World* world=b2_worlds+id.index1-1;world->locked=true;f=b2World_CastMover(id,{1000,2000},&mover,{10,0},filter);world->locked=false;ENSURE(f==1);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

struct PlaneHits { b2PlaneResult planes[8];int count;bool keepGoing;bool valid; };
static bool CollectPlane(b2ShapeId shape,const b2PlaneResult* plane,void* context)
{
 PlaneHits* hits=static_cast<PlaneHits*>(context);hits->valid&=b2Shape_IsValid(shape)&&plane->hit&&b2IsNormalized(plane->plane.normal);
 if(hits->count<8) { hits->planes[hits->count]=*plane; }++hits->count;return hits->keepGoing;
}
int WorldMoverPlanesTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);
 b2BodyDef bd=b2DefaultBodyDef();bd.position={1000,2000};b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();b2Segment floor={{-10,0},{10,0}},wall={{0,-10},{0,10}};
 b2CreateSegmentShape(body,&sd,&floor);sd.filter.categoryBits=2;b2CreateSegmentShape(body,&sd,&wall);
 b2Capsule mover={{0.4f,0.25f},{0.4f,1.25f},0.5f};b2QueryFilter filter=b2DefaultQueryFilter();PlaneHits hits={};hits.keepGoing=true;hits.valid=true;
 b2World_CollideMover(id,{1000,2000},&mover,filter,CollectPlane,&hits);ENSURE(hits.count==2 && hits.valid);bool up=false,right=false;
 for(int i=0;i<2;++i) { b2Plane plane=hits.planes[i].plane;if(plane.normal.y>0.99f) { up=true;ENSURE_SMALL(plane.offset-0.25f,0.0001f); }if(plane.normal.x>0.99f) { right=true;ENSURE_SMALL(plane.offset-0.1f,0.0001f); } }ENSURE(up && right);
 filter.maskBits=2;hits.count=0;b2World_CollideMover(id,{1000,2000},&mover,filter,CollectPlane,&hits);ENSURE(hits.count==1 && hits.planes[0].plane.normal.x>0.99f);
 filter.maskBits=UINT64_MAX;hits.count=0;hits.keepGoing=false;b2World_CollideMover(id,{1000,2000},&mover,filter,CollectPlane,&hits);ENSURE(hits.count==1);
 hits.count=0;hits.keepGoing=true;b2World_CollideMover(id,{1100,2100},&mover,filter,CollectPlane,&hits);ENSURE(hits.count==0);
 b2Shape shape={};shape.type=b2_circleShape;shape.circle={{0,0},0.5f};b2Capsule testMover={{0,0.75f},{0,1.5f},0.5f};b2PlaneResult p=b2CollideMover(&testMover,&shape,b2Transform_identity);ENSURE(p.hit && p.plane.normal.y>0.99f);ENSURE_SMALL(p.plane.offset-0.25f,0.00001f);
 shape.type=b2_capsuleShape;shape.capsule={{-1,0},{1,0},0.5f};p=b2CollideMover(&testMover,&shape,b2Transform_identity);ENSURE(p.hit && p.plane.normal.y>0.99f);
 shape.type=b2_polygonShape;shape.polygon=b2MakeBox(1,0.5f);p=b2CollideMover(&testMover,&shape,b2Transform_identity);ENSURE(p.hit && p.plane.normal.y>0.99f);
 shape.type=b2_chainSegmentShape;shape.chainSegment.segment={{1,0},{-1,0}};testMover={{0,0.25f},{0,1.25f},0.5f};p=b2CollideMover(&testMover,&shape,b2Transform_identity);ENSURE(p.hit);testMover={{0,-0.25f},{0,-1.25f},0.5f};p=b2CollideMover(&testMover,&shape,b2Transform_identity);ENSURE(!p.hit);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int WorldBoundsTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();b2WorldId id=b2CreateWorld(&wd);b2AABB box=b2World_GetBounds(id);ENSURE(box.lowerBound.x==0 && box.lowerBound.y==0 && box.upperBound.x==0 && box.upperBound.y==0);
 b2BodyId bodies[3];b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},0.5f};
 for(int i=0;i<3;++i) { b2BodyDef bd=b2DefaultBodyDef();bd.type=(b2BodyType)i;bd.position={10.0f*i,5.0f*i};bodies[i]=b2CreateBody(id,&bd);b2CreateCircleShape(bodies[i],&sd,&c); }
 b2World* world=b2_worlds+id.index1-1;b2AABB expected=b2DynamicTree_GetRootBounds(world->broadPhase.trees);
 for(int i=1;i<3;++i) { b2AABB part=b2DynamicTree_GetRootBounds(world->broadPhase.trees+i);expected.lowerBound=b2Min(expected.lowerBound,part.lowerBound);expected.upperBound=b2Max(expected.upperBound,part.upperBound); }
 box=b2World_GetBounds(id);ENSURE(box.lowerBound.x==expected.lowerBound.x && box.upperBound.y==expected.upperBound.y);
 b2Body_SetTransform(bodies[2],{30,15},b2Rot_identity);box=b2World_GetBounds(id);ENSURE(box.upperBound.x>30);
 b2Body_Disable(bodies[2]);box=b2World_GetBounds(id);ENSURE(box.upperBound.x<20);
 world->locked=true;box=b2World_GetBounds(id);world->locked=false;ENSURE(box.upperBound.x==0);
 b2DestroyBody(bodies[0]);b2DestroyBody(bodies[1]);b2DestroyBody(bodies[2]);box=b2World_GetBounds(id);ENSURE(box.lowerBound.x==0 && box.upperBound.x==0);
 b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

struct DrawStats { int circles,polygons,capsules,lines,points,strings,bounds,transforms;bool valid;b2Pos center;b2HexColor color; };
static void DrawCircle(b2WorldTransform xf,b2Vec2 local,float radius,b2HexColor color,void* ctx) { DrawStats* s=(DrawStats*)ctx;++s->circles;s->center=b2TransformWorldPoint(xf,local);s->color=color;s->valid&=radius>0; }
static void DrawPolygon(b2WorldTransform,const b2Vec2*,int count,float,b2HexColor,void* ctx) { DrawStats* s=(DrawStats*)ctx;++s->polygons;s->valid&=count>=3; }
static void DrawCapsule(b2Pos,b2Pos,float,b2HexColor,void* ctx) { ++((DrawStats*)ctx)->capsules; }
static void DrawLine(b2Pos a,b2Pos b,b2HexColor,void* ctx) { DrawStats* s=(DrawStats*)ctx;++s->lines;s->valid&=b2IsValidVec2(a)&&b2IsValidVec2(b); }
static void DrawPoint(b2Pos,float,b2HexColor,void* ctx) { ++((DrawStats*)ctx)->points; }
static void DrawString(b2Pos,const char*,b2HexColor,void* ctx) { ++((DrawStats*)ctx)->strings; }
static void DrawBounds(b2AABB,b2HexColor,void* ctx) { ++((DrawStats*)ctx)->bounds; }
static void DrawTransform(b2WorldTransform,void* ctx) { ++((DrawStats*)ctx)->transforms; }
int WorldDebugDrawTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2ShapeDef sd=b2DefaultShapeDef();b2BodyDef bd=b2DefaultBodyDef();bd.position={100,200};b2BodyId body=b2CreateBody(id,&bd);b2Circle circle={{1,0},0.5f};sd.material.customColor=0x123456;b2CreateCircleShape(body,&sd,&circle);
 bd.position={103,200};bd.type=b2_dynamicBody;bd.name="box";body=b2CreateBody(id,&bd);sd.material.customColor=0;b2Polygon box=b2MakeBox(0.5f,0.5f);b2CreatePolygonShape(body,&sd,&box);circle.center={0,0};b2CreateCircleShape(body,&sd,&circle);
 bd.position={106,200};bd.type=b2_staticBody;bd.name=nullptr;body=b2CreateBody(id,&bd);b2Capsule cap={{0,-1},{0,1},0.5f};b2CreateCapsuleShape(body,&sd,&cap);
 bd.position={109,200};body=b2CreateBody(id,&bd);b2Segment edge={{0,-1},{0,1}};b2CreateSegmentShape(body,&sd,&edge);
 b2DebugDraw draw=b2DefaultDebugDraw();DrawStats stats={};stats.valid=true;draw.context=&stats;draw.DrawSolidCircleFcn=DrawCircle;draw.DrawSolidPolygonFcn=DrawPolygon;draw.DrawSolidCapsuleFcn=DrawCapsule;draw.DrawLineFcn=DrawLine;draw.DrawPointFcn=DrawPoint;draw.DrawStringFcn=DrawString;draw.DrawBoundsFcn=DrawBounds;draw.DrawTransformFcn=DrawTransform;
 draw.drawingBounds={{100,199},{102,201}};b2World_Draw(id,&draw);ENSURE(stats.circles==1 && stats.color==static_cast<b2HexColor>(0x123456));ENSURE(stats.center.x==101 && stats.center.y==200);
 stats={};stats.valid=true;draw.drawingBounds={{90,190},{120,210}};draw.drawBounds=true;draw.drawMass=true;draw.drawBodyNames=true;draw.drawIslands=true;b2World_Draw(id,&draw);ENSURE(stats.circles==2 && stats.polygons==1 && stats.capsules==1 && stats.lines>=2);ENSURE(stats.transforms==1 && stats.strings==2 && stats.bounds==6 && stats.valid);
 size_t afterDraw=b2GetByteCount();b2World_Draw(id,&draw);ENSURE(b2GetByteCount()==afterDraw);
 bd.type=b2_dynamicBody;bd.position={103,200.8f};body=b2CreateBody(id,&bd);b2CreatePolygonShape(body,&sd,&box);b2World_Step(id,0.01f,1);stats={};stats.valid=true;draw.drawContacts=true;draw.drawContactNormals=true;b2World_Draw(id,&draw);ENSURE(stats.points>0 && stats.valid);
 stats={};b2World* world=b2_worlds+id.index1-1;world->locked=true;b2World_Draw(id,&draw);world->locked=false;ENSURE(stats.circles==0);
 draw=b2DefaultDebugDraw();b2World_Draw(id,&draw);b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

int WorldStaticTreeRebuildTest(void)
{
 size_t before=b2GetByteCount();b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;b2WorldId id=b2CreateWorld(&wd);b2World_RebuildStaticTree(id);
 b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},0.5f};b2BodyId first={};
 for(int i=0;i<8;++i) { b2BodyDef bd=b2DefaultBodyDef();bd.position={3.0f*i,0};b2BodyId b=b2CreateBody(id,&bd);if(i==0) { first=b; }b2CreateCircleShape(b,&sd,&c); }
 b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;bd.position={10,0};b2BodyId dynamic=b2CreateBody(id,&bd);b2CreateCircleShape(dynamic,&sd,&c);b2World_Step(id,0.01f,1);b2Body_SetTransform(first,{10.6f,0},b2Rot_identity);
 b2World* world=b2_worlds+id.index1-1;b2DynamicTree* tree=world->broadPhase.trees+b2_staticBody;int moved[16];int count=b2DynamicTree_GatherMovedProxies(tree,moved);ENSURE(count>0);uint64_t movedBefore=0;for(int i=0;i<count;++i) { movedBefore|=uint64_t(1)<<moved[i]; }
 QueryHits hits={};hits.keepGoing=true;hits.valid=true;b2QueryFilter filter=b2DefaultQueryFilter();b2World_OverlapAABB(id,{0,0},{{-2,-2},{25,2}},filter,CollectOverlap,&hits);uint64_t foundBefore=0;for(int i=0;i<hits.count;++i) { foundBefore|=uint64_t(1)<<hits.shapes[i].index1; }ENSURE(hits.count==9);
 b2AABB bounds=b2World_GetBounds(id);b2World_RebuildStaticTree(id);b2DynamicTree_Validate(tree);uint64_t movedAfter=0;int after=b2DynamicTree_GatherMovedProxies(tree,moved);for(int i=0;i<after;++i) { movedAfter|=uint64_t(1)<<moved[i]; }ENSURE(movedBefore==movedAfter && count==after);
 hits.count=0;b2World_OverlapAABB(id,{0,0},{{-2,-2},{25,2}},filter,CollectOverlap,&hits);uint64_t foundAfter=0;for(int i=0;i<hits.count;++i) { foundAfter|=uint64_t(1)<<hits.shapes[i].index1; }ENSURE(foundBefore==foundAfter && hits.count==9);
 b2AABB result=b2World_GetBounds(id);ENSURE(bounds.lowerBound.x==result.lowerBound.x && bounds.upperBound.x==result.upperBound.x);
 b2World_Step(id,0.01f,1);ENSURE(b2Body_GetContactCapacity(dynamic)>0);
 world->locked=true;b2World_RebuildStaticTree(id);world->locked=false;b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

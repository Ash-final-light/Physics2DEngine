// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "test_macros.h"
#include "world.h"

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

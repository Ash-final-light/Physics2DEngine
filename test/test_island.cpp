// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "test_macros.h"
#include "world.h"

static int CheckIslands(b2World* world)
{
    int total=0;
    for(int i=0;i<world->islands.count;++i)
    {
        b2Island* island=world->islands.data+i;
        if(island->setIndex==B2_NULL_INDEX) { continue; }
        b2ValidateIsland(world,i);
        ENSURE(world->solverSets.data[island->setIndex].islandSims.data[island->localIndex].islandId==i);
        total+=island->bodies.count;
    }
    int expected=0;
    for(int i=0;i<world->bodies.count;++i)
    {
        b2Body* body=world->bodies.data+i;
        if(body->id==B2_NULL_INDEX) { continue; }
        bool hasIsland=body->setIndex==b2_awakeSet || body->setIndex>=b2_firstSleepingSet;
        ENSURE(hasIsland==(body->islandId!=B2_NULL_INDEX));
        expected+=hasIsland?1:0;
    }
    ENSURE(total==expected);return 0;
}

int ContactIslandTest(void)
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId id=b2CreateWorld(&wd);b2World* w=b2_worlds+id.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle c={{0,0},1};
    b2BodyId a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&c);
    bd.position={1.5f,0};b2BodyId b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    bd.position={3.0f,0};b2BodyId d=b2CreateBody(id,&bd);b2CreateCircleShape(d,&sd,&c);
    b2UpdateBroadPhaseContacts(w);b2UpdateContacts(w);
    ENSURE(b2GetIdCount(&w->islandIdPool)==1);
    ENSURE(w->islands.data[w->bodies.data[a.index1-1].islandId].contacts.count==2);
    ENSURE(CheckIslands(w)==0);
    b2Body_SetTransform(b,{20,0},b2Rot_identity);
    b2UpdateBroadPhaseContacts(w);b2UpdateContacts(w);b2SplitContactIslands(w);
    ENSURE(b2GetIdCount(&w->islandIdPool)==3);ENSURE(CheckIslands(w)==0);
    // Static ground can touch both independent bodies without bridging them.
    bd.type=b2_staticBody;bd.position={1.5f,-1.5f};b2BodyId ground=b2CreateBody(id,&bd);
    b2Polygon floor=b2MakeBox(10,0.6f);b2CreatePolygonShape(ground,&sd,&floor);
    b2UpdateBroadPhaseContacts(w);b2UpdateContacts(w);
    ENSURE(w->bodies.data[a.index1-1].islandId!=w->bodies.data[d.index1-1].islandId);
    ENSURE(w->bodies.data[ground.index1-1].islandId==B2_NULL_INDEX);ENSURE(CheckIslands(w)==0);
    b2Body_SetType(a,b2_staticBody);ENSURE(CheckIslands(w)==0);
    b2Body_SetType(a,b2_dynamicBody);ENSURE(CheckIslands(w)==0);
    b2DestroyBody(d);b2SplitContactIslands(w);ENSURE(CheckIslands(w)==0);
    bd.type=b2_dynamicBody;bd.isAwake=false;bd.position={40,0};
    b2BodyId sleep=b2CreateBody(id,&bd);b2CreateCircleShape(sleep,&sd,&c);
    int sleepSet=w->bodies.data[sleep.index1-1].setIndex;
    bd.isAwake=true;bd.position={41.5f,0};b2BodyId awake=b2CreateBody(id,&bd);b2CreateCircleShape(awake,&sd,&c);
    b2UpdateBroadPhaseContacts(w);b2UpdateContacts(w);
    ENSURE(w->solverSets.data[sleepSet].setIndex==B2_NULL_INDEX);
    ENSURE(w->bodies.data[sleep.index1-1].islandId==w->bodies.data[awake.index1-1].islandId);
    ENSURE(CheckIslands(w)==0);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // A larger component exercises island-array growth during splitting.
    id=b2CreateWorld(&wd);w=b2_worlds+id.index1-1;
    bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2BodyId line[48];
    for(int i=0;i<48;++i)
    {
        bd.position={1.5f*i,0};line[i]=b2CreateBody(id,&bd);b2CreateCircleShape(line[i],&sd,&c);
    }
    b2UpdateBroadPhaseContacts(w);b2UpdateContacts(w);
    ENSURE(b2GetIdCount(&w->islandIdPool)==1);ENSURE(CheckIslands(w)==0);
    b2DestroyBody(line[24]);b2SplitContactIslands(w);
    ENSURE(b2GetIdCount(&w->islandIdPool)==2);ENSURE(CheckIslands(w)==0);
    ENSURE(w->bodies.data[line[0].index1-1].islandId!=w->bodies.data[line[47].index1-1].islandId);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // Removing one redundant edge must retain a connected component.
    id=b2CreateWorld(&wd);w=b2_worlds+id.index1-1;
    bd.position={0,0};a=b2CreateBody(id,&bd);b2ShapeId sa=b2CreateCircleShape(a,&sd,&c);
    bd.position={1.2f,0};b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&c);
    bd.position={0.6f,1.0f};d=b2CreateBody(id,&bd);b2CreateCircleShape(d,&sd,&c);
    b2UpdateBroadPhaseContacts(w);b2UpdateContacts(w);
    ENSURE(b2GetIdCount(&w->islandIdPool)==1);
    b2Contact* edge=NULL;
    for(int i=0;i<w->contacts.count;++i)
    {
        b2Contact* candidate=w->contacts.data+i;
        if(candidate->shapeIdA==sa.index1-1 || candidate->shapeIdB==sa.index1-1) { edge=candidate;break; }
    }
    ENSURE(edge!=NULL);b2DestroyContact(w,edge);b2SplitContactIslands(w);
    ENSURE(b2GetIdCount(&w->islandIdPool)==1);ENSURE(CheckIslands(w)==0);
    ENSURE(w->islands.data[w->bodies.data[a.index1-1].islandId].constraintRemoveCount==0);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}

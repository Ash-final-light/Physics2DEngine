// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include <algorithm>
#include <atomic>
#include <thread>
#include <vector>
#include "world.h"
#include "box2d/types.h"
#include "box2d/math_functions.h"

struct PairThreadHost
{
    std::atomic<int> enqueued{0};
    std::atomic<int> executed{0};
};

static void* EnqueuePairThread(b2TaskCallbackFcn* task,void* context,void* user)
{
    auto host=static_cast<PairThreadHost*>(user);
    ++host->enqueued;
    return new std::thread([=] { ++host->executed;task(context); });
}

static void FinishPairThread(void* handle,void*)
{
    auto thread=static_cast<std::thread*>(handle);
    thread->join();delete thread;
}

static b2WorldId MakePairWorld(PairThreadHost* host)
{
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;wd.enableSleep=false;
    if(host) { wd.workerCount=4;wd.enqueueTask=EnqueuePairThread;wd.finishTask=FinishPairThread;wd.userTaskContext=host; }
    b2WorldId id=b2CreateWorld(&wd);
    b2ShapeDef sd=b2DefaultShapeDef();sd.enableContactEvents=true;
    b2Circle disk={{0,0},0.4f};
    for(int i=0;i<180;++i)
    {
        b2BodyDef bd=b2DefaultBodyDef();bd.type=(b2BodyType)(i%3);
        bd.position={float(i%15)*0.6f,float(i/15)*0.6f};
        b2CreateCircleShape(b2CreateBody(id,&bd),&sd,&disk);
    }
    return id;
}

static std::vector<uint64_t> PairContactKeys(b2WorldId id)
{
    b2World* world=b2_worlds+id.index1-1;std::vector<uint64_t> keys;
    for(int i=0;i<world->contacts.count;++i)
    {
        const b2Contact& c=world->contacts.data[i];
        if(c.contactId!=B2_NULL_INDEX)
        {
            uint32_t a=(uint32_t)b2MinInt(c.shapeIdA,c.shapeIdB),b=(uint32_t)b2MaxInt(c.shapeIdA,c.shapeIdB);
            keys.push_back((uint64_t)a<<32|b);
        }
    }
    return keys;
}

int ParallelBroadPhaseTest(void)
{
    size_t before=b2GetByteCount();PairThreadHost host;
    b2WorldId serial=MakePairWorld(nullptr),threaded=MakePairWorld(&host),scheduled=MakePairWorld(nullptr),reverse=MakePairWorld(nullptr);
    b2World_SetWorkerCount(scheduled,4);b2World_SetWorkerCount(reverse,4);
    std::vector<uint64_t> bruteForce;
    b2World* initial=b2_worlds+serial.index1-1;
    for(int a=0;a<initial->shapes.count;++a)
    {
        for(int b=a+1;b<initial->shapes.count;++b)
        {
            b2BodyType ta=initial->bodies.data[initial->shapes.data[a].bodyId].type;
            b2BodyType tb=initial->bodies.data[initial->shapes.data[b].bodyId].type;
            if((ta==b2_dynamicBody || tb==b2_dynamicBody) && b2AABB_Overlaps(initial->fatAABBs.data[a],initial->fatAABBs.data[b]))
            {
                bruteForce.push_back((uint64_t)(uint32_t)a<<32|(uint32_t)b);
            }
        }
    }
    for(int step=0;step<20;++step)
    {
        b2World_Step(serial,1.0f/60,4);b2World_Step(threaded,1.0f/60,4);b2World_Step(scheduled,1.0f/60,4);
        b2TaskBatch batch=b2World_BeginStep(reverse,1.0f/60,4);
        while(batch.count>0)
        {
            for(int i=batch.count-1;i>=0;--i) { batch.tasks[i].fcn(batch.tasks[i].context); }
            batch=b2World_ContinueStep(reverse);
        }
        auto expected=PairContactKeys(serial);ENSURE(!expected.empty());
        if(step==0) { ENSURE(expected==bruteForce); }
        ENSURE(expected==PairContactKeys(threaded));ENSURE(expected==PairContactKeys(scheduled));ENSURE(expected==PairContactKeys(reverse));
        auto ordered=expected;std::sort(ordered.begin(),ordered.end());ENSURE(std::adjacent_find(ordered.begin(),ordered.end())==ordered.end());
        b2World* a=b2_worlds+serial.index1-1;
        for(b2WorldId other:{threaded,scheduled,reverse})
        {
            b2World* b=b2_worlds+other.index1-1;
            ENSURE(a->bodyMoveEvents.count==b->bodyMoveEvents.count);
            for(int i=0;i<a->bodyMoveEvents.count;++i)
            {
                ENSURE(a->bodyMoveEvents.data[i].transform.p.x==b->bodyMoveEvents.data[i].transform.p.x);
                ENSURE(a->bodyMoveEvents.data[i].transform.p.y==b->bodyMoveEvents.data[i].transform.p.y);
            }
        }
        if(step==9) { b2World_SetWorkerCount(scheduled,2);b2World_SetWorkerCount(reverse,2); }
    }
    ENSURE(host.enqueued>0 && host.executed==host.enqueued);
    for(b2WorldId id:{serial,threaded,scheduled,reverse})
    {
        b2World* w=b2_worlds+id.index1-1;ENSURE(w->stack.allocation==0);b2DestroyWorld(id);
    }
    ENSURE(b2GetByteCount()==before);return 0;
}

int BroadPhaseTreeTypesTest( void )
{
    size_t before = b2GetByteCount();
    b2WorldDef def = b2DefaultWorldDef();
    def.capacity.staticShapeCount = 128;
    def.capacity.dynamicShapeCount = 256;
    b2WorldId id = b2CreateWorld( &def );
    b2BroadPhase* bp = &b2_worlds[id.index1-1].broadPhase;
    ENSURE( bp->trees[b2_staticBody].proxyCapacity >= 128 );
    ENSURE( bp->trees[b2_dynamicBody].proxyCapacity >= 256 );
    ENSURE( bp->trees[b2_kinematicBody].proxyCapacity >= 16 );
    for ( int i = 0; i < b2_bodyTypeCount; ++i )
    {
        ENSURE( b2DynamicTree_GetProxyCount( bp->trees+i ) == 0 );
        ENSURE( bp->trees[i].nodes != bp->trees[(i+1)%b2_bodyTypeCount].nodes );
    }
    b2WorldId other = b2CreateWorld( &def );
    ENSURE( bp->trees[0].nodes != b2_worlds[other.index1-1].broadPhase.trees[0].nodes );
    b2DestroyWorld( id );
    b2DestroyWorld( other );
    ENSURE( b2GetByteCount() == before );
    return 0;
}


int BroadPhaseProxyLifecycleTest( void )
{
    size_t before = b2GetByteCount();
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId worldId = b2CreateWorld( &wd );
    b2World* world = b2_worlds + worldId.index1-1;
    b2BodyDef bd = b2DefaultBodyDef();
    b2ShapeDef sd = b2DefaultShapeDef();
    sd.filter.categoryBits = 8;
    sd.invokeContactCreation = false;
    b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
    for ( int type = 0; type < b2_bodyTypeCount; ++type )
    {
        bd.type = (b2BodyType)type;
        b2BodyId body = b2CreateBody( worldId, &bd );
        b2ShapeId id = b2CreateCircleShape( body, &sd, &circle );
        b2Shape* shape = world->shapes.data + id.index1-1;
        ENSURE( B2_PROXY_TYPE( shape->proxyKey ) == type );
        b2DynamicTree* tree = world->broadPhase.trees + type;
        int proxyId = B2_PROXY_ID( shape->proxyKey );
        ENSURE( b2DynamicTree_GetProxyCount( tree ) == 1 );
        ENSURE( b2DynamicTree_GetUserData( tree, proxyId ) == (uint64_t)(id.index1-1) );
        ENSURE( b2DynamicTree_GetCategoryBits( tree, proxyId ) == 8 );
        ENSURE( b2IsNodeMoved( tree->nodes + tree->proxies[proxyId].node ) == (type != b2_staticBody) );
        b2AABB fat = b2DynamicTree_GetProxyBounds( tree, proxyId );
        ENSURE( b2AABB_Contains( fat, shape->aabb ) );
        b2DestroyShape( id, true );
        ENSURE( b2DynamicTree_GetProxyCount( tree ) == 0 );
        b2CreateCircleShape( body, &sd, &circle );
        b2DestroyBody( body );
        ENSURE( b2DynamicTree_GetProxyCount( tree ) == 0 );
    }
    bd.isEnabled = false;
    b2BodyId disabled = b2CreateBody( worldId, &bd );
    b2ShapeId id = b2CreateCircleShape( disabled, &sd, &circle );
    ENSURE( world->shapes.data[id.index1-1].proxyKey == B2_NULL_INDEX );
    b2DestroyWorld( worldId );
    ENSURE( b2GetByteCount() == before );
    return 0;
}


int BroadPhaseMoveTest( void )
{
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId worldId = b2CreateWorld( &wd );
    b2World* world = b2_worlds + worldId.index1-1;
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    b2BodyId body = b2CreateBody( worldId, &bd );
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Circle circle = { { 2.0f, 0.0f }, 1.0f };
    b2ShapeId id = b2CreateCircleShape( body, &sd, &circle );
    b2Shape* shape = world->shapes.data + id.index1-1;
    b2DynamicTree* tree = world->broadPhase.trees+b2_dynamicBody;
    int proxyId = B2_PROXY_ID( shape->proxyKey );
    b2AABB original = b2DynamicTree_GetProxyBounds( tree, proxyId );
    b2Body_SetTransform( body, { 0.001f, 0.0f }, b2Rot_identity );
    b2AABB unchanged = b2DynamicTree_GetProxyBounds( tree, proxyId );
    ENSURE( original.lowerBound.x == unchanged.lowerBound.x && original.upperBound.x == unchanged.upperBound.x );
    b2Body_SetTransform( body, { 100.0f, 20.0f }, { 0.0f, 1.0f } );
    b2AABB moved = b2DynamicTree_GetProxyBounds( tree, proxyId );
    ENSURE( moved.lowerBound.x > 90.0f );
    ENSURE( b2AABB_Contains( moved, shape->aabb ) );
    ENSURE( b2IsNodeMoved( tree->nodes + tree->proxies[proxyId].node ) );
    b2DynamicTree_Validate( tree );
    circle.radius = 10.0f;
    b2Shape_SetCircle( id, &circle );
    proxyId = B2_PROXY_ID( shape->proxyKey );
    moved = b2DynamicTree_GetProxyBounds( tree, proxyId );
    ENSURE( moved.lowerBound.x < 90.0f && b2AABB_Contains( moved, shape->aabb ) );
    ENSURE( b2DynamicTree_GetProxyCount( tree ) == 1 );
    bd.isEnabled = false;
    b2BodyId disabled = b2CreateBody( worldId, &bd );
    b2ShapeId disabledShape = b2CreateCircleShape( disabled, &sd, &circle );
    b2Body_SetTransform( disabled, { 500.0f, 500.0f }, b2Rot_identity );
    ENSURE( world->shapes.data[disabledShape.index1-1].proxyKey == B2_NULL_INDEX );
    ENSURE( b2Body_ComputeBounds( disabled ).lowerBound.x > 480.0f );
    b2DestroyWorld( worldId );
    return 0;
}


int BroadPhaseCandidatesTest( void )
{
    size_t before = b2GetByteCount();
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId worldId = b2CreateWorld( &wd );
    b2World* world = b2_worlds + worldId.index1-1;
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Circle circle = { { 0.0f, 0.0f }, 0.75f };
    uint32_t random = 1234567;
    for ( int i = 0; i < 45; ++i )
    {
        b2BodyDef bd = b2DefaultBodyDef();
        bd.type = (b2BodyType)(i%3);
        random = 1664525*random+1013904223;
        bd.position.x = float((random>>16)%40)*0.1f;
        random = 1664525*random+1013904223;
        bd.position.y = float((random>>16)%40)*0.1f;
        b2CreateCircleShape( b2CreateBody( worldId, &bd ), &sd, &circle );
    }
    b2DynamicArray<uint64_t> keys = {};
    b2CollectCandidatePairs( world, &keys );
    int expected = 0;
    for ( int a = 0; a < world->shapes.count; ++a )
    {
        for ( int b = a+1; b < world->shapes.count; ++b )
        {
            b2BodyType typeA = world->bodies.data[world->shapes.data[a].bodyId].type;
            b2BodyType typeB = world->bodies.data[world->shapes.data[b].bodyId].type;
            if ( typeA != b2_dynamicBody && typeB != b2_dynamicBody )
            {
                continue;
            }
            if ( b2AABB_Overlaps( world->fatAABBs.data[a], world->fatAABBs.data[b] ) )
            {
                ++expected;
                uint64_t key = (uint64_t)(uint32_t)a<<32 | (uint32_t)b;
                int occurrences = 0;
                for ( int i = 0; i < keys.count; ++i )
                {
                    occurrences += keys.data[i] == key;
                }
                ENSURE( occurrences == 1 );
            }
        }
    }
    ENSURE( expected > 32 && keys.count == expected );
    b2CollectCandidatePairs( world, &keys );
    ENSURE( keys.count == 0 );
    b2Array_Destroy( keys );
    b2DestroyWorld( worldId );
    ENSURE( b2GetByteCount() == before );
    return 0;
}


static void MarkAllBroadPhaseProxies( b2World* world )
{
    for ( int i = 0; i < world->shapes.count; ++i )
    {
        b2Shape* shape = world->shapes.data+i;
        if ( shape->id != B2_NULL_INDEX && shape->proxyKey != B2_NULL_INDEX )
        {
            b2BroadPhase_MoveProxy( &world->broadPhase, shape->proxyKey, world->fatAABBs.data[i] );
        }
    }
}

int BroadPhaseExistingPairsTest( void )
{
    size_t before = b2GetByteCount();
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId worldId = b2CreateWorld( &wd );
    b2World* world = b2_worlds+worldId.index1-1;
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
    for ( int i = 0; i < 4; ++i )
    {
        b2BodyDef bd = b2DefaultBodyDef();
        bd.type = b2_dynamicBody;
        b2CreateCircleShape( b2CreateBody( worldId, &bd ), &sd, &circle );
    }
    b2DynamicArray<uint64_t> keys = {};
    b2CollectCandidatePairs( world, &keys );
    ENSURE( keys.count == 6 );
    uint64_t existing = keys.data[0];
    ENSURE( b2AddKey( &world->broadPhase.pairSet, existing ) == false );
    MarkAllBroadPhaseProxies( world );
    b2CollectCandidatePairs( world, &keys );
    ENSURE( keys.count == 5 );
    for ( int i = 0; i < keys.count; ++i )
    {
        ENSURE( keys.data[i] != existing );
    }
    ENSURE( b2RemoveKey( &world->broadPhase.pairSet, existing ) );
    MarkAllBroadPhaseProxies( world );
    b2CollectCandidatePairs( world, &keys );
    ENSURE( keys.count == 6 );
    b2Array_Destroy( keys );
    b2DestroyWorld( worldId );
    ENSURE( b2GetByteCount() == before );
    return 0;
}


int BroadPhaseSortTest( void )
{
    uint64_t values[513], expected[513], temporary[513];
    uint64_t random = 0x123456789abcdef0ULL;
    for ( int i = 0; i < 513; ++i )
    {
        random ^= random << 13;
        random ^= random >> 7;
        random ^= random << 17;
        values[i] = i%11 == 0 ? UINT64_MAX : random;
        expected[i] = values[i];
    }
    std::sort( expected, expected+513 );
    b2RadixSortKeys( values, temporary, 513 );
    for ( int i = 0; i < 513; ++i )
    {
        ENSURE( values[i] == expected[i] );
    }
    b2RadixSortKeys( NULL, NULL, 0 );
    uint64_t one = 42;
    b2RadixSortKeys( &one, temporary, 1 );
    ENSURE( one == 42 );
    for ( int i = 0; i < 513; ++i )
    {
        values[i] = 123;
    }
    b2RadixSortKeys( values, temporary, 513 );
    ENSURE( values[0] == 123 && values[512] == 123 );
    return 0;
}


static int FilteredPairCount( b2Filter a, b2Filter b, bool sameBody, bool segments )
{
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId worldId = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    b2BodyId bodyA = b2CreateBody( worldId, &bd );
    b2BodyId bodyB = sameBody ? bodyA : b2CreateBody( worldId, &bd );
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
    b2Segment segment = { { -1.0f, 0.0f }, { 1.0f, 0.0f } };
    sd.filter = a;
    if ( segments ) { b2CreateSegmentShape( bodyA, &sd, &segment ); }
    else { b2CreateCircleShape( bodyA, &sd, &circle ); }
    sd.filter = b;
    if ( segments ) { b2CreateSegmentShape( bodyB, &sd, &segment ); }
    else { b2CreateCircleShape( bodyB, &sd, &circle ); }
    b2DynamicArray<uint64_t> keys = {};
    b2CollectCandidatePairs( b2_worlds+worldId.index1-1, &keys );
    int count = keys.count;
    b2Array_Destroy( keys );
    b2DestroyWorld( worldId );
    return count;
}

int BroadPhaseFilterTest( void )
{
    size_t before = b2GetByteCount();
    b2Filter a = b2DefaultFilter(), b = b2DefaultFilter();
    ENSURE( FilteredPairCount( a, b, false, false ) == 1 );
    ENSURE( FilteredPairCount( a, b, true, false ) == 0 );
    ENSURE( FilteredPairCount( a, b, false, true ) == 0 );
    a.maskBits = 0;
    ENSURE( FilteredPairCount( a, b, false, false ) == 0 );
    a.groupIndex = b.groupIndex = 3;
    b.maskBits = 0;
    ENSURE( FilteredPairCount( a, b, false, false ) == 1 );
    a.groupIndex = b.groupIndex = -3;
    ENSURE( FilteredPairCount( a, b, false, false ) == 0 );
    a = b2DefaultFilter(); b = b2DefaultFilter();
    a.categoryBits = 2; a.maskBits = 4;
    b.categoryBits = 4; b.maskBits = 2;
    ENSURE( FilteredPairCount( a, b, false, false ) == 1 );
    b.maskBits = 1;
    ENSURE( FilteredPairCount( a, b, false, false ) == 0 );
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId worldId = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
    b2ShapeId shapeId = b2CreateCircleShape( b2CreateBody( worldId, &bd ), &sd, &circle );
    b2Shape_SetFilter( shapeId, a );
    ENSURE( b2Shape_GetFilter( shapeId ).categoryBits == 2 );
    b2World* world = b2_worlds+worldId.index1-1;
    b2Shape* shape = world->shapes.data+shapeId.index1-1;
    ENSURE( b2DynamicTree_GetCategoryBits( world->broadPhase.trees+b2_dynamicBody, B2_PROXY_ID(shape->proxyKey) ) == 2 );
    b2DestroyWorld( worldId );
    ENSURE( b2GetByteCount() == before );
    return 0;
}

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "world.h"
#include "box2d/types.h"

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

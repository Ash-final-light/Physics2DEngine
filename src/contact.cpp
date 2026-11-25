// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact.h"
#include "world.h"
#include "body.h"
#include "shape.h"

static constexpr bool b2_primary[5][5] = {
    { true, false, false, false, false },
    { true, true, false, false, false },
    { true, true, false, true, false },
    { true, true, false, true, false },
    { true, true, false, true, false }
};

bool b2CanCollide( b2ShapeType a, b2ShapeType b )
{
    bool edgeA = a == b2_segmentShape || a == b2_chainSegmentShape;
    bool edgeB = b == b2_segmentShape || b == b2_chainSegmentShape;
    return edgeA == false || edgeB == false;
}

bool b2Contact_IsValid( b2ContactId id )
{
    if ( id.world0 >= B2_MAX_WORLDS )
    {
        return false;
    }
    b2World* world = b2_worlds+id.world0;
    if ( world->inUse == false || id.index1 < 1 || id.index1 > world->contacts.count )
    {
        return false;
    }
    const b2Contact* contact = world->contacts.data+id.index1-1;
    return contact->contactId == id.index1-1 && contact->generation == id.generation;
}

void b2CreateContact( b2World* world, b2Shape* a, b2Shape* b )
{
    if ( b2CanCollide(a->type,b->type) == false )
    {
        return;
    }
    if ( b2_primary[a->type][b->type] == false )
    {
        b2CreateContact(world,b,a);
        return;
    }
    int id = b2AllocId(&world->contactIdPool);
    if ( id == world->contacts.count )
    {
        b2Array_Push(world->contacts,b2Contact{});
    }
    b2Contact* contact = world->contacts.data+id;
    uint32_t generation = contact->generation+1;
    *contact = b2Contact{};
    contact->contactId = id;
    contact->generation = generation;
    contact->shapeIdA = a->id;
    contact->shapeIdB = b->id;
    contact->islandId = B2_NULL_INDEX;
    contact->islandIndex = B2_NULL_INDEX;
    contact->setIndex = B2_NULL_INDEX;
    contact->colorIndex = B2_NULL_INDEX;
    contact->localIndex = B2_NULL_INDEX;
    int bodyIds[2] = { a->bodyId,b->bodyId };
    for ( int edge = 0; edge < 2; ++edge )
    {
        b2Body* body = world->bodies.data+bodyIds[edge];
        b2ContactEdge* link = contact->edges+edge;
        link->bodyId = body->id;
        link->prevKey = B2_NULL_INDEX;
        link->nextKey = body->headContactKey;
        int key = (id<<1)|edge;
        if ( body->headContactKey != B2_NULL_INDEX )
        {
            b2Contact* head = world->contacts.data+(body->headContactKey>>1);
            head->edges[body->headContactKey&1].prevKey = key;
        }
        body->headContactKey = key;
        body->contactCount += 1;
    }
    uint64_t key = B2_SHAPE_PAIR_KEY(a->id,b->id);
    B2_ASSERT(b2ContainsKey(&world->broadPhase.pairSet,key)==false);
    b2AddKey(&world->broadPhase.pairSet,key);
}

void b2DestroyContact( b2World* world, b2Contact* contact )
{
    uint64_t key = B2_SHAPE_PAIR_KEY(contact->shapeIdA,contact->shapeIdB);
    b2RemoveKey(&world->broadPhase.pairSet,key);
    for ( int edge = 0; edge < 2; ++edge )
    {
        b2ContactEdge* link = contact->edges+edge;
        b2Body* body = world->bodies.data+link->bodyId;
        if ( link->prevKey != B2_NULL_INDEX )
        {
            world->contacts.data[link->prevKey>>1].edges[link->prevKey&1].nextKey = link->nextKey;
        }
        if ( link->nextKey != B2_NULL_INDEX )
        {
            world->contacts.data[link->nextKey>>1].edges[link->nextKey&1].prevKey = link->prevKey;
        }
        if ( body->headContactKey == ((contact->contactId<<1)|edge) )
        {
            body->headContactKey = link->nextKey;
        }
        body->contactCount -= 1;
    }
    b2FreeId(&world->contactIdPool,contact->contactId);
    uint32_t generation = contact->generation;
    *contact = b2Contact{};
    contact->contactId = B2_NULL_INDEX;
    contact->generation = generation;
}

void b2DestroyBodyContacts( b2World* world, b2Body* body )
{
    while ( body->headContactKey != B2_NULL_INDEX )
    {
        b2DestroyContact(world,world->contacts.data+(body->headContactKey>>1));
    }
}

void b2DestroyShapeContacts( b2World* world, const b2Shape* shape )
{
    int key = world->bodies.data[shape->bodyId].headContactKey;
    while ( key != B2_NULL_INDEX )
    {
        b2Contact* contact = world->contacts.data+(key>>1);
        key = contact->edges[key&1].nextKey;
        if ( contact->shapeIdA == shape->id || contact->shapeIdB == shape->id )
        {
            b2DestroyContact(world,contact);
        }
    }
}

void b2UpdateBroadPhaseContacts( b2World* world )
{
    for ( int i = 0; i < world->contacts.count; ++i )
    {
        b2Contact* contact = world->contacts.data+i;
        if ( contact->contactId != B2_NULL_INDEX &&
             b2AABB_Overlaps(world->fatAABBs.data[contact->shapeIdA],world->fatAABBs.data[contact->shapeIdB]) == false )
        {
            b2DestroyContact(world,contact);
        }
    }
    b2DynamicArray<uint64_t> keys = {};
    b2CollectCandidatePairs(world,&keys);
    for ( int i = 0; i < keys.count; ++i )
    {
        int a = (int)(keys.data[i]>>32);
        int b = (int)(keys.data[i]&0xFFFFFFFF);
        b2CreateContact(world,world->shapes.data+a,world->shapes.data+b);
    }
    b2Array_Destroy(keys);
}

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"

#include "core.h"
#include "world.h"
#include "island.h"
#include "shape.h"
#include "contact.h"

#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <string.h>

b2Body* b2GetBodyFullId( b2World* world, b2BodyId bodyId )
{
	if ( world == NULL || bodyId.index1 < 1 || bodyId.index1 > world->bodies.count )
	{
		return NULL;
	}
	b2Body* body = world->bodies.data + ( bodyId.index1 - 1 );
	if ( body->id != bodyId.index1 - 1 || body->generation != bodyId.generation )
	{
		return NULL;
	}
	return body;
}

static void b2CreateIslandForBody( b2World* world, int setIndex, b2Body* body )
{
	B2_ASSERT( body->islandId == B2_NULL_INDEX );
	B2_ASSERT( setIndex != b2_disabledSet );

	b2Island* island = b2CreateIsland( world, setIndex );
	b2Array_Push( island->bodies, body->id );
	body->islandId = island->islandId;
	body->islandIndex = 0;

	b2ValidateIsland( world, island->islandId );
}

static void b2RemoveBodyFromIsland( b2World* world, b2Body* body )
{
	if ( body->islandId == B2_NULL_INDEX )
	{
		B2_ASSERT( body->islandIndex == B2_NULL_INDEX );
		return;
	}

	int islandId = body->islandId;
	b2Island* island = b2Array_Get( world->islands, islandId );
	{
		int localIndex = body->islandIndex;
		int movedBodyId = island->bodies.data[island->bodies.count - 1];
		island->bodies.data[localIndex] = movedBodyId;
		B2_VALIDATE( world->bodies.data[movedBodyId].islandIndex == island->bodies.count - 1 );
		world->bodies.data[movedBodyId].islandIndex = localIndex;
		island->bodies.count -= 1;
	}

	if ( island->bodies.count == 0 )
	{
		// 销毁空岛
		B2_ASSERT( island->contacts.count == 0 );

		// 释放岛
		b2DestroyIsland( world, island->islandId );
	}
	else
	{
		b2ValidateIsland( world, islandId );
	}

	body->islandId = B2_NULL_INDEX;
	body->islandIndex = B2_NULL_INDEX;
}


b2BodyId b2CreateBody( b2WorldId worldId, const b2BodyDef* def )
{
    B2_CHECK_DEF(def);
    if(def->type<b2_staticBody || def->type>=b2_bodyTypeCount || b2IsValidVec2(def->position)==false ||
       b2IsValidRot(def->rotation)==false || b2IsValidVec2(def->linearVelocity)==false ||
       b2IsValidFloat(def->angularVelocity)==false || b2IsValidFloat(def->linearDamping)==false || def->linearDamping<0.0f ||
       b2IsValidFloat(def->angularDamping)==false || def->angularDamping<0.0f || b2IsValidFloat(def->gravityScale)==false ||
       b2IsValidFloat(def->sleepThreshold)==false || def->sleepThreshold<0.0f || b2IsValidFloat(def->safetyFactor)==false || def->safetyFactor<0.0f ||
       b2World_IsValid(worldId)==false)
    { return b2_nullBodyId; }
    b2World* world=b2_worlds+worldId.index1-1;
    if(world->locked || world->inUse==false) { return b2_nullBodyId; }
    int id=b2AllocId(&world->bodyIdPool);
    if(id==world->bodies.count) { b2Array_Push(world->bodies,b2Body{}); }
    b2Body* body=world->bodies.data+id;
    uint16_t generation=(uint16_t)(body->generation+1);
    *body=b2Body{};
    body->id=id;body->generation=generation;body->type=def->type;
    body->headShapeId=B2_NULL_INDEX;body->headChainId=B2_NULL_INDEX;
    body->headContactKey=B2_NULL_INDEX;body->headJointKey=B2_NULL_INDEX;
    body->islandId=B2_NULL_INDEX;body->islandIndex=B2_NULL_INDEX;
    body->bodyMoveIndex=B2_NULL_INDEX;
    body->sleepThreshold=def->sleepThreshold;body->safetyFactor=def->safetyFactor;
    body->userData=def->userData;
    body->flags=(def->motionLocks.linearX?b2_lockLinearX:0) |
                (def->motionLocks.linearY?b2_lockLinearY:0) |
                (def->motionLocks.angularZ?b2_lockAngularZ:0) |
                (def->isBullet?b2_isBullet:0) |
                (def->allowFastRotation?b2_allowFastRotation:0) |
                (def->enableSleep?b2_enableSleep:0) |
                (def->enableContactRecycling?b2_bodyEnableContactRecycling:0) |
                (def->type==b2_dynamicBody?b2_dynamicFlag:0);
    int setIndex;
    if(def->isEnabled==false) { setIndex=b2_disabledSet; }
    else if(def->type==b2_staticBody) { setIndex=b2_staticSet; }
    else if(def->type==b2_dynamicBody && def->isAwake==false && def->enableSleep) { setIndex=b2CreateSolverSet(world); }
    else { setIndex=b2_awakeSet; }
    b2SolverSet* set=world->solverSets.data+setIndex;
    body->setIndex=setIndex;body->localIndex=set->bodySims.count;
    b2BodySim* sim=b2Array_Emplace(set->bodySims);*sim=b2BodySim{};
    sim->bodyId=id;sim->transform={def->position,def->rotation};sim->center=def->position;
    sim->center0=sim->center;sim->rotation0=def->rotation;sim->minExtent=B2_HUGE;
    sim->linearDamping=def->linearDamping;sim->angularDamping=def->angularDamping;
    sim->gravityScale=def->gravityScale;sim->flags=body->flags;
    if(setIndex==b2_awakeSet)
    {
        b2BodyState state=b2_identityBodyState;
        state.linearVelocity=def->linearVelocity;state.angularVelocity=def->angularVelocity;state.flags=body->flags;
        b2Array_Push(set->bodyStates,state);
    }
    if(setIndex==b2_awakeSet || setIndex>=b2_firstSleepingSet) { b2CreateIslandForBody(world,setIndex,body); }
    if(def->name!=NULL) { strncpy_s(body->name,sizeof(body->name),def->name,B2_NAME_LENGTH); }
    return b2BodyId{id+1,world->worldId,generation};
}

void b2DestroyBody( b2BodyId bodyId )
{
	if ( bodyId.world0 >= B2_MAX_WORLDS || b2_worlds[bodyId.world0].inUse == false )
	{
		return;
	}

	b2World* world = b2_worlds + bodyId.world0;
	b2Body* body = b2GetBodyFullId( world, bodyId );
	if ( body == NULL )
	{
		return;
	}


    b2DestroyBodyContacts( world, body );
    while ( body->headChainId != B2_NULL_INDEX )
    {
        b2ChainShape* chain = world->chainShapes.data + body->headChainId;
        b2DestroyChain( b2ChainId{ chain->id + 1, world->worldId, chain->generation } );
    }
    while ( body->headShapeId != B2_NULL_INDEX )
    {
        b2Shape* shape = world->shapes.data + body->headShapeId;
        b2DestroyShape( b2ShapeId{ shape->id + 1, world->worldId, shape->generation }, false );
    }
    b2RemoveBodyFromIsland(world,body);
    b2RemoveBodyFromSolverSet(world,body);
	b2FreeId( &world->bodyIdPool, body->id );
	uint16_t generation = body->generation;
	*body = b2Body{};
	body->generation = generation;
	body->id = B2_NULL_INDEX;
}

bool b2Body_IsValid( b2BodyId id )
{
	if ( id.world0 >= B2_MAX_WORLDS )
	{
		return false;
	}
	b2World* world = b2_worlds + id.world0;
	if ( world->inUse == false || world->worldId != id.world0 )
	{
		return false;
	}
	if ( id.index1 < 1 || world->bodies.count < id.index1 )
	{
		return false;
	}
	b2Body* body = world->bodies.data + ( id.index1 - 1 );
	if ( body->id == B2_NULL_INDEX )
	{
		return false;
	}
	return body->generation == id.generation;
}

void b2Body_SetName( b2BodyId bodyId, const char* name )
{
	if ( bodyId.world0 >= B2_MAX_WORLDS || b2_worlds[bodyId.world0].inUse == false )
	{
		return;
	}
	b2Body* body = b2GetBodyFullId( b2_worlds + bodyId.world0, bodyId );
	if ( body == NULL )
	{
		return;
	}
	if ( name != NULL )
	{
		strncpy_s( body->name, sizeof( body->name ), name, B2_NAME_LENGTH );
	}
	else
	{
		memset( body->name, 0, sizeof( body->name ) );
	}
}

const char* b2Body_GetName( b2BodyId bodyId )
{
	static const char empty[] = "";
	if ( bodyId.world0 >= B2_MAX_WORLDS || b2_worlds[bodyId.world0].inUse == false )
	{
		return empty;
	}
	b2Body* body = b2GetBodyFullId( b2_worlds + bodyId.world0, bodyId );
	if ( body == NULL )
	{
		return empty;
	}
	return body->name;
}

void b2Body_SetUserData( b2BodyId bodyId, void* userData )
{
	if ( bodyId.world0 >= B2_MAX_WORLDS || b2_worlds[bodyId.world0].inUse == false )
	{
		return;
	}
	b2Body* body = b2GetBodyFullId( b2_worlds + bodyId.world0, bodyId );
	if ( body == NULL )
	{
		return;
	}
	body->userData = userData;
}

void* b2Body_GetUserData( b2BodyId bodyId )
{
	if ( bodyId.world0 >= B2_MAX_WORLDS || b2_worlds[bodyId.world0].inUse == false )
	{
		return NULL;
	}
	b2Body* body = b2GetBodyFullId( b2_worlds + bodyId.world0, bodyId );
	if ( body == NULL )
	{
		return NULL;
	}
	return body->userData;
}

static b2WorldTransform b2GetBodyTransformFromId( b2BodyId bodyId )
{
	B2_ASSERT( b2Body_IsValid( bodyId ) );
	b2Body* body = b2GetBodyFullId( b2_worlds + bodyId.world0, bodyId );
	return b2GetBodySim(b2_worlds+bodyId.world0,body)->transform;
}

b2Pos b2Body_GetPosition( b2BodyId bodyId )
{
	return b2GetBodyTransformFromId( bodyId ).p;
}

b2Rot b2Body_GetRotation( b2BodyId bodyId )
{
	return b2GetBodyTransformFromId( bodyId ).q;
}

b2WorldTransform b2Body_GetTransform( b2BodyId bodyId )
{
	return b2GetBodyTransformFromId( bodyId );
}

b2Vec2 b2Body_GetLocalPoint( b2BodyId bodyId, b2Pos worldPoint )
{
	return b2InvTransformPoint( b2GetBodyTransformFromId( bodyId ), worldPoint );
}

b2Pos b2Body_GetWorldPoint( b2BodyId bodyId, b2Vec2 localPoint )
{
	return b2TransformPoint( b2GetBodyTransformFromId( bodyId ), localPoint );
}

b2Vec2 b2Body_GetLocalVector( b2BodyId bodyId, b2Vec2 worldVector )
{
	return b2InvRotateVector( b2GetBodyTransformFromId( bodyId ).q, worldVector );
}

b2Vec2 b2Body_GetWorldVector( b2BodyId bodyId, b2Vec2 localVector )
{
	return b2RotateVector( b2GetBodyTransformFromId( bodyId ).q, localVector );
}

void b2Body_SetTransform( b2BodyId bodyId, b2Pos position, b2Rot rotation )
{
	if ( b2IsValidVec2( position ) == false || b2IsValidRot( rotation ) == false ||
		 b2Body_IsValid( bodyId ) == false )
	{
		return;
	}
	b2World* world = b2_worlds + bodyId.world0;
	if ( world->locked )
	{
		return;
	}
	b2Body* body = b2GetBodyFullId( world, bodyId );
	b2GetBodySim(b2_worlds+bodyId.world0,body)->transform.p = position;
	b2GetBodySim(b2_worlds+bodyId.world0,body)->transform.q = rotation;
	b2GetBodySim(b2_worlds+bodyId.world0,body)->center = b2TransformPoint( b2GetBodySim(b2_worlds+bodyId.world0,body)->transform, b2GetBodySim(b2_worlds+bodyId.world0,body)->localCenter );
    for ( int id = body->headShapeId; id != B2_NULL_INDEX; id = world->shapes.data[id].nextShapeId )
    {
        b2MoveShapeProxy( world, world->shapes.data + id, b2GetBodySim(b2_worlds+bodyId.world0,body)->transform );
    }
}

b2AABB b2Body_ComputeBounds( b2BodyId bodyId )
{
	if ( b2Body_IsValid( bodyId ) == false || b2_worlds[bodyId.world0].locked )
	{
		return b2AABB{};
	}
    b2World* world = b2_worlds + bodyId.world0;
    b2Body* body = b2GetBodyFullId( world, bodyId );
    if ( body->headShapeId == B2_NULL_INDEX )
    {
        b2Pos position = b2GetBodySim(b2_worlds+bodyId.world0,body)->transform.p;
        return b2AABB{ position, position };
    }
    b2Shape* shape = world->shapes.data + body->headShapeId;
    b2AABB bounds = shape->aabb;
    while ( shape->nextShapeId != B2_NULL_INDEX )
    {
        shape = world->shapes.data + shape->nextShapeId;
        bounds = b2AABB_Union( bounds, shape->aabb );
    }
    return bounds;
}


void b2UpdateBodyMassData( b2World* world, b2Body* body )
{
    body->flags &= ~b2_dirtyMass;
    b2SyncBodyFlags(world,body);
    body->mass = 0.0f;
    body->inertia = 0.0f;
    b2GetBodySim(world,body)->invMass = 0.0f;
    b2GetBodySim(world,body)->invInertia = 0.0f;
    b2GetBodySim(world,body)->localCenter = b2Vec2_zero;
    b2GetBodySim(world,body)->minExtent = B2_HUGE;
    b2GetBodySim(world,body)->maxExtent = 0.0f;
    if ( body->type != b2_dynamicBody )
    {
        b2GetBodySim(world,body)->center = b2GetBodySim(world,body)->transform.p;
        if ( body->type == b2_kinematicBody )
        {
            for ( int id = body->headShapeId; id != B2_NULL_INDEX; id = world->shapes.data[id].nextShapeId )
            {
                b2ShapeExtent extent = b2ComputeShapeExtent( world->shapes.data + id, b2Vec2_zero );
                b2GetBodySim(world,body)->minExtent = b2MinFloat( b2GetBodySim(world,body)->minExtent, extent.minExtent );
                b2GetBodySim(world,body)->maxExtent = b2MaxFloat( b2GetBodySim(world,body)->maxExtent, extent.maxExtent );
            }
        }
        return;
    }
    b2MassData* masses = static_cast<b2MassData*>( b2StackAlloc( &world->stack, body->shapeCount * sizeof( b2MassData ), "mass data" ) );
    b2Vec2 localCenter = b2Vec2_zero;
    int index = 0;
    for ( int id = body->headShapeId; id != B2_NULL_INDEX; id = world->shapes.data[id].nextShapeId )
    {
        const b2Shape* shape = world->shapes.data + id;
        if ( shape->density == 0.0f )
        {
            masses[index++] = b2MassData{};
            continue;
        }
        b2MassData data = b2ComputeShapeMass( shape );
        body->mass += data.mass;
        localCenter = b2MulAdd( localCenter, data.mass, data.center );
        masses[index++] = data;
    }
    if ( body->mass > 0.0f )
    {
        b2GetBodySim(world,body)->invMass = 1.0f / body->mass;
        localCenter = b2MulSV( b2GetBodySim(world,body)->invMass, localCenter );
    }
    for ( int i = 0; i < body->shapeCount; ++i )
    {
        b2MassData data = masses[i];
        if ( data.mass == 0.0f )
        {
            continue;
        }
        b2Vec2 offset = b2Sub( localCenter, data.center );
        body->inertia += data.rotationalInertia + data.mass * b2Dot( offset, offset );
    }
    b2StackFree( &world->stack, masses );
    if ( body->inertia > 0.0f && ((body->flags & b2_lockAngularZ) != 0) == false )
    {
        b2GetBodySim(world,body)->invInertia = 1.0f / body->inertia;
    }
    else
    {
        body->inertia = 0.0f;
    }
    b2Pos oldCenter = b2GetBodySim(world,body)->center;
    b2GetBodySim(world,body)->localCenter = localCenter;
    b2GetBodySim(world,body)->center = b2TransformPoint( b2GetBodySim(world,body)->transform, localCenter );
    if ( (body->setIndex == b2_awakeSet) && (body->setIndex != b2_disabledSet) )
    {
        b2Vec2 delta = b2CrossSV( b2GetBodyState(world,body)->angularVelocity, b2Sub( b2GetBodySim(world,body)->center, oldCenter ) );
        b2GetBodyState(world,body)->linearVelocity = b2Add( b2GetBodyState(world,body)->linearVelocity, delta );
    }
    for ( int id = body->headShapeId; id != B2_NULL_INDEX; id = world->shapes.data[id].nextShapeId )
    {
        b2ShapeExtent extent = b2ComputeShapeExtent( world->shapes.data + id, localCenter );
        b2GetBodySim(world,body)->minExtent = b2MinFloat( b2GetBodySim(world,body)->minExtent, extent.minExtent );
        b2GetBodySim(world,body)->maxExtent = b2MaxFloat( b2GetBodySim(world,body)->maxExtent, extent.maxExtent );
    }
}

b2MassData b2Body_GetMassData( b2BodyId id )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    b2Body* body = b2GetBodyFullId( b2_worlds + id.world0, id );
    return b2MassData{ body->mass, b2GetBodySim(b2_worlds+id.world0,body)->localCenter, body->inertia };
}

float b2Body_GetMass( b2BodyId id )
{
    return b2Body_GetMassData( id ).mass;
}

float b2Body_GetRotationalInertia( b2BodyId id )
{
    return b2Body_GetMassData( id ).rotationalInertia;
}

b2Vec2 b2Body_GetLocalCenter( b2BodyId id )
{
    return b2Body_GetMassData( id ).center;
}

b2Pos b2Body_GetWorldCenter( b2BodyId id )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    return b2GetBodySim(b2_worlds+id.world0,b2GetBodyFullId( b2_worlds + id.world0, id ))->center;
}

void b2Body_UpdateMassFromShapes( b2BodyId id )
{
    if ( b2Body_IsValid( id ) && b2_worlds[id.world0].locked == false )
    {
        b2UpdateBodyMassData( b2_worlds + id.world0, b2GetBodyFullId( b2_worlds + id.world0, id ) );
    }
}


void b2Body_SetMassData( b2BodyId id, b2MassData data )
{
    if ( b2IsValidFloat( data.mass ) == false || data.mass < 0.0f ||
         b2IsValidFloat( data.rotationalInertia ) == false || data.rotationalInertia < 0.0f ||
         b2IsValidVec2( data.center ) == false || b2Body_IsValid( id ) == false || b2_worlds[id.world0].locked )
    {
        return;
    }
    b2World* world = b2_worlds + id.world0;
    b2Body* body = b2GetBodyFullId( world, id );
    body->mass = data.mass;
    body->inertia = data.rotationalInertia;
    b2GetBodySim(b2_worlds+id.world0,body)->localCenter = data.center;
    b2Pos oldCenter = b2GetBodySim(b2_worlds+id.world0,body)->center;
    b2GetBodySim(b2_worlds+id.world0,body)->center = b2TransformPoint( b2GetBodySim(b2_worlds+id.world0,body)->transform, data.center );
    if ( body->type != b2_staticBody && (body->setIndex == b2_awakeSet) && (body->setIndex != b2_disabledSet) )
    {
        b2Vec2 delta = b2CrossSV( b2GetBodyState(b2_worlds+id.world0,body)->angularVelocity, b2Sub( b2GetBodySim(b2_worlds+id.world0,body)->center, oldCenter ) );
        b2GetBodyState(b2_worlds+id.world0,body)->linearVelocity = b2Add( b2GetBodyState(b2_worlds+id.world0,body)->linearVelocity, delta );
    }
    b2GetBodySim(b2_worlds+id.world0,body)->invMass = body->mass > 0.0f ? 1.0f / body->mass : 0.0f;
    b2GetBodySim(b2_worlds+id.world0,body)->invInertia = body->inertia > 0.0f ? 1.0f / body->inertia : 0.0f;
    b2GetBodySim(b2_worlds+id.world0,body)->minExtent = B2_HUGE;
    b2GetBodySim(b2_worlds+id.world0,body)->maxExtent = 0.0f;
    for ( int shapeId = body->headShapeId; shapeId != B2_NULL_INDEX; shapeId = world->shapes.data[shapeId].nextShapeId )
    {
        b2ShapeExtent extent = b2ComputeShapeExtent( world->shapes.data + shapeId, data.center );
        b2GetBodySim(b2_worlds+id.world0,body)->minExtent = b2MinFloat( b2GetBodySim(b2_worlds+id.world0,body)->minExtent, extent.minExtent );
        b2GetBodySim(b2_worlds+id.world0,body)->maxExtent = b2MaxFloat( b2GetBodySim(b2_worlds+id.world0,body)->maxExtent, extent.maxExtent );
    }
    if ( ((body->flags & b2_lockAngularZ) != 0) )
    {
        body->inertia = 0.0f;
        b2GetBodySim(b2_worlds+id.world0,body)->invInertia = 0.0f;
    }
    body->flags &= ~b2_dirtyMass;
    b2SyncBodyFlags(b2_worlds+id.world0,body);
}


b2WorldId b2Body_GetWorld( b2BodyId id )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    return b2WorldId{ (uint16_t)(id.world0+1), b2_worlds[id.world0].generation };
}

int b2Body_GetShapeCount( b2BodyId id )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    return b2GetBodyFullId( b2_worlds+id.world0, id )->shapeCount;
}

int b2Body_GetShapes( b2BodyId id, b2ShapeId* output, int capacity )
{
    if ( b2Body_IsValid( id ) == false || capacity <= 0 || output == NULL )
    {
        return 0;
    }
    b2World* world = b2_worlds + id.world0;
    b2Body* body = b2GetBodyFullId( world, id );
    int count = 0;
    for ( int shapeId = body->headShapeId; shapeId != B2_NULL_INDEX && count < capacity; shapeId = world->shapes.data[shapeId].nextShapeId )
    {
        output[count++] = b2ShapeId{ shapeId+1, id.world0, world->shapes.data[shapeId].generation };
    }
    return count;
}


static bool b2HasBodyMotionState( const b2Body* body )
{
    return body->setIndex==b2_awakeSet;
}

b2Vec2 b2Body_GetLinearVelocity( b2BodyId id )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    b2Body* body = b2GetBodyFullId( b2_worlds+id.world0, id );
    return b2HasBodyMotionState( body ) ? b2GetBodyState(b2_worlds+id.world0,body)->linearVelocity : b2Vec2_zero;
}

float b2Body_GetAngularVelocity( b2BodyId id )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    b2Body* body = b2GetBodyFullId( b2_worlds+id.world0, id );
    return b2HasBodyMotionState( body ) ? b2GetBodyState(b2_worlds+id.world0,body)->angularVelocity : 0.0f;
}

void b2Body_SetLinearVelocity( b2BodyId id, b2Vec2 velocity )
{
    if ( b2Body_IsValid( id ) == false || b2IsValidVec2( velocity ) == false )
    {
        return;
    }
    b2Body* body = b2GetBodyFullId( b2_worlds+id.world0, id );
    if ( body->type == b2_staticBody || (body->setIndex != b2_disabledSet) == false )
    {
        return;
    }
    if ( b2LengthSquared( velocity ) > 0.0f )
    {
        b2WakeBody(b2_worlds+id.world0,body);
    }
    if ( (body->setIndex == b2_awakeSet) )
    {
        b2GetBodyState(b2_worlds+id.world0,body)->linearVelocity = velocity;
    }
}

void b2Body_SetAngularVelocity( b2BodyId id, float velocity )
{
    if ( b2Body_IsValid( id ) == false || b2IsValidFloat( velocity ) == false )
    {
        return;
    }
    b2Body* body = b2GetBodyFullId( b2_worlds+id.world0, id );
    if ( body->type == b2_staticBody || (body->setIndex != b2_disabledSet) == false || ((body->flags & b2_lockAngularZ) != 0) )
    {
        return;
    }
    if ( velocity != 0.0f )
    {
        b2WakeBody(b2_worlds+id.world0,body);
    }
    if ( (body->setIndex == b2_awakeSet) )
    {
        b2GetBodyState(b2_worlds+id.world0,body)->angularVelocity = velocity;
    }
}

b2Vec2 b2Body_GetLocalPointVelocity( b2BodyId id, b2Vec2 point )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    b2Body* body = b2GetBodyFullId( b2_worlds+id.world0, id );
    if ( b2HasBodyMotionState( body ) == false )
    {
        return b2Vec2_zero;
    }
    b2Vec2 offset = b2RotateVector( b2GetBodySim(b2_worlds+id.world0,body)->transform.q, b2Sub( point, b2GetBodySim(b2_worlds+id.world0,body)->localCenter ) );
    return b2Add( b2GetBodyState(b2_worlds+id.world0,body)->linearVelocity, b2CrossSV( b2GetBodyState(b2_worlds+id.world0,body)->angularVelocity, offset ) );
}

b2Vec2 b2Body_GetWorldPointVelocity( b2BodyId id, b2Pos point )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    b2Body* body = b2GetBodyFullId( b2_worlds+id.world0, id );
    if ( b2HasBodyMotionState( body ) == false )
    {
        return b2Vec2_zero;
    }
    return b2Add( b2GetBodyState(b2_worlds+id.world0,body)->linearVelocity, b2CrossSV( b2GetBodyState(b2_worlds+id.world0,body)->angularVelocity, b2Sub( point, b2GetBodySim(b2_worlds+id.world0,body)->center ) ) );
}


void b2Body_SetGravityScale( b2BodyId id, float scale )
{
    if ( b2Body_IsValid( id ) && b2IsValidFloat( scale ) )
    {
        b2GetBodySim(b2_worlds+id.world0,b2GetBodyFullId( b2_worlds+id.world0, id ))->gravityScale = scale;
    }
}

float b2Body_GetGravityScale( b2BodyId id )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    return b2GetBodySim(b2_worlds+id.world0,b2GetBodyFullId( b2_worlds+id.world0, id ))->gravityScale;
}


void b2Body_SetLinearDamping( b2BodyId id, float damping )
{
    if ( b2Body_IsValid( id ) && b2IsValidFloat( damping ) && damping >= 0.0f )
    {
        b2GetBodySim(b2_worlds+id.world0,b2GetBodyFullId( b2_worlds+id.world0, id ))->linearDamping = damping;
    }
}

float b2Body_GetLinearDamping( b2BodyId id )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    return b2GetBodySim(b2_worlds+id.world0,b2GetBodyFullId( b2_worlds+id.world0, id ))->linearDamping;
}

void b2Body_SetAngularDamping( b2BodyId id, float damping )
{
    if ( b2Body_IsValid( id ) && b2IsValidFloat( damping ) && damping >= 0.0f )
    {
        b2GetBodySim(b2_worlds+id.world0,b2GetBodyFullId( b2_worlds+id.world0, id ))->angularDamping = damping;
    }
}

float b2Body_GetAngularDamping( b2BodyId id )
{
    B2_ASSERT( b2Body_IsValid( id ) );
    return b2GetBodySim(b2_worlds+id.world0,b2GetBodyFullId( b2_worlds+id.world0, id ))->angularDamping;
}


static b2Body* b2GetBodyForExternalAction( b2BodyId id, bool wake )
{
    if ( b2Body_IsValid( id ) == false )
    {
        return NULL;
    }
    b2Body* body = b2GetBodyFullId( b2_worlds+id.world0, id );
    if ( body->type != b2_dynamicBody || (body->setIndex != b2_disabledSet) == false )
    {
        return NULL;
    }
    if ( wake )
    {
        b2WakeBody(b2_worlds+id.world0,body);
    }
    return (body->setIndex == b2_awakeSet) ? body : NULL;
}

void b2Body_ApplyForce( b2BodyId id, b2Vec2 force, b2Pos point, bool wake )
{
    if ( b2IsValidVec2( force ) == false || b2IsValidVec2( point ) == false )
    {
        return;
    }
    b2Body* body = b2GetBodyForExternalAction( id, wake );
    if ( body != NULL )
    {
        b2GetBodySim(b2_worlds+id.world0,body)->force = b2Add( b2GetBodySim(b2_worlds+id.world0,body)->force, force );
        b2GetBodySim(b2_worlds+id.world0,body)->torque += b2Cross( b2Sub( point, b2GetBodySim(b2_worlds+id.world0,body)->center ), force );
    }
}

void b2Body_ApplyForceToCenter( b2BodyId id, b2Vec2 force, bool wake )
{
    if ( b2IsValidVec2( force ) == false )
    {
        return;
    }
    b2Body* body = b2GetBodyForExternalAction( id, wake );
    if ( body != NULL )
    {
        b2GetBodySim(b2_worlds+id.world0,body)->force = b2Add( b2GetBodySim(b2_worlds+id.world0,body)->force, force );
    }
}

void b2Body_ClearForces( b2BodyId id )
{
    if ( b2Body_IsValid( id ) )
    {
        b2Body* body = b2GetBodyFullId( b2_worlds+id.world0, id );
        b2GetBodySim(b2_worlds+id.world0,body)->force = b2Vec2_zero;
        b2GetBodySim(b2_worlds+id.world0,body)->torque = 0.0f;
    }
}


void b2Body_ApplyTorque( b2BodyId id, float torque, bool wake )
{
    if ( b2IsValidFloat( torque ) == false )
    {
        return;
    }
    b2Body* body = b2GetBodyForExternalAction( id, wake );
    if ( body != NULL )
    {
        b2GetBodySim(b2_worlds+id.world0,body)->torque += torque;
    }
}


static void b2LimitBodyVelocity( b2BodyState* state, float maximum )
{
    float squared=b2LengthSquared(state->linearVelocity);
    if(squared>maximum*maximum)
    { state->linearVelocity=b2MulSV(maximum/sqrtf(squared),state->linearVelocity); }
}

void b2Body_ApplyLinearImpulse( b2BodyId id, b2Vec2 impulse, b2Pos point, bool wake )
{
    if ( b2IsValidVec2( impulse ) == false || b2IsValidVec2( point ) == false )
    {
        return;
    }
    b2Body* body = b2GetBodyForExternalAction( id, wake );
    if ( body != NULL )
    {
        b2GetBodyState(b2_worlds+id.world0,body)->linearVelocity = b2MulAdd( b2GetBodyState(b2_worlds+id.world0,body)->linearVelocity, b2GetBodySim(b2_worlds+id.world0,body)->invMass, impulse );
        b2GetBodyState(b2_worlds+id.world0,body)->angularVelocity += b2GetBodySim(b2_worlds+id.world0,body)->invInertia*b2Cross( b2Sub(point,b2GetBodySim(b2_worlds+id.world0,body)->center), impulse );
        b2LimitBodyVelocity( b2GetBodyState(b2_worlds+id.world0,body), b2_worlds[id.world0].maxLinearSpeed );
    }
}

void b2Body_ApplyLinearImpulseToCenter( b2BodyId id, b2Vec2 impulse, bool wake )
{
    if ( b2IsValidVec2( impulse ) == false )
    {
        return;
    }
    b2Body* body = b2GetBodyForExternalAction( id, wake );
    if ( body != NULL )
    {
        b2GetBodyState(b2_worlds+id.world0,body)->linearVelocity = b2MulAdd( b2GetBodyState(b2_worlds+id.world0,body)->linearVelocity, b2GetBodySim(b2_worlds+id.world0,body)->invMass, impulse );
        b2LimitBodyVelocity( b2GetBodyState(b2_worlds+id.world0,body), b2_worlds[id.world0].maxLinearSpeed );
    }
}


void b2Body_ApplyAngularImpulse( b2BodyId id, float impulse, bool wake )
{
    if ( b2IsValidFloat( impulse ) == false )
    {
        return;
    }
    b2Body* body = b2GetBodyForExternalAction( id, wake );
    if ( body != NULL )
    {
        b2GetBodyState(b2_worlds+id.world0,body)->angularVelocity += b2GetBodySim(b2_worlds+id.world0,body)->invInertia*impulse;
    }
}


void b2Body_SetTargetTransform( b2BodyId id, b2WorldTransform target, float timeStep, bool wake )
{
    if ( b2Body_IsValid(id)==false || b2IsValidTransform(target)==false || b2IsValidFloat(timeStep)==false || timeStep<=0.0f )
    {
        return;
    }
    b2Body* body=b2GetBodyFullId(b2_worlds+id.world0,id);
    if ( (body->setIndex != b2_disabledSet)==false || body->type==b2_staticBody || ((body->setIndex == b2_awakeSet)==false && wake==false) )
    {
        return;
    }
    b2Vec2 delta=b2Sub(b2TransformPoint(target,b2GetBodySim(b2_worlds+id.world0,body)->localCenter),b2GetBodySim(b2_worlds+id.world0,body)->center);
    float inverseTime=1.0f/timeStep;
    b2Vec2 velocity=b2MulSV(inverseTime,delta);
    float angularVelocity=inverseTime*b2RelativeAngle(b2GetBodySim(b2_worlds+id.world0,body)->transform.q,target.q);
    if ( (body->setIndex == b2_awakeSet)==false )
    {
        float maximum=b2Length(velocity)+b2AbsFloat(angularVelocity)*b2GetBodySim(b2_worlds+id.world0,body)->maxExtent;
        if ( maximum<body->sleepThreshold )
        {
            return;
        }
        b2WakeBody(b2_worlds+id.world0,body);
    }
    b2GetBodyState(b2_worlds+id.world0,body)->linearVelocity=velocity;
    b2GetBodyState(b2_worlds+id.world0,body)->angularVelocity=angularVelocity;
}


b2BodyType b2Body_GetType( b2BodyId id )
{
    B2_ASSERT( b2Body_IsValid(id) );
    return b2GetBodyFullId(b2_worlds+id.world0,id)->type;
}


void b2Body_SetType( b2BodyId id, b2BodyType type )
{
    if(b2Body_IsValid(id)==false || type<b2_staticBody || type>=b2_bodyTypeCount || b2_worlds[id.world0].locked)
    { return; }
    b2World* world=b2_worlds+id.world0;
    b2Body* body=b2GetBodyFullId(world,id);
    if(body->type==type) { return; }
    b2WakeBody(world,body);
    b2DestroyBodyContacts(world,body);
    b2RemoveBodyFromIsland(world,body);
    body->type=type;
    if(type==b2_dynamicBody) { body->flags|=b2_dynamicFlag; }
    else { body->flags&=~b2_dynamicFlag; }
    if(body->setIndex!=b2_disabledSet)
    {
        int target=type==b2_staticBody?b2_staticSet:b2_awakeSet;
        b2TransferBody(world,world->solverSets.data+target,world->solverSets.data+body->setIndex,body);
    }
    if(body->setIndex==b2_awakeSet) { b2CreateIslandForBody(world,b2_awakeSet,body); }
    b2SyncBodyFlags(world,body);
    b2UpdateBodyMassData(world,body);
    if(body->setIndex!=b2_disabledSet)
    {
        for(int shapeId=body->headShapeId;shapeId!=B2_NULL_INDEX;shapeId=world->shapes.data[shapeId].nextShapeId)
        {
            b2Shape* shape=world->shapes.data+shapeId;
            b2DestroyShapeProxy(shape,&world->broadPhase);
            b2CreateShapeProxy(world,shape,type,b2GetBodySim(b2_worlds+id.world0,body)->transform,true);
        }
    }
}


int b2Body_GetContactCapacity( b2BodyId id )
{
    if(b2Body_IsValid(id)==false || b2_worlds[id.world0].locked)
    {
        return 0;
    }
    return b2GetBodyFullId(b2_worlds+id.world0,id)->contactCount;
}

int b2Body_GetContactData( b2BodyId id, b2ContactData* output, int capacity )
{
    if(b2Body_IsValid(id)==false || output==NULL || capacity<=0 || b2_worlds[id.world0].locked)
    {
        return 0;
    }
    b2World* world=b2_worlds+id.world0;
    b2Body* body=b2GetBodyFullId(world,id);
    int count=0;
    int key=body->headContactKey;
    while(key!=B2_NULL_INDEX && count<capacity)
    {
        b2Contact* contact=world->contacts.data+(key>>1);
        if((contact->flags&b2_contactTouchingFlag)!=0)
        {
            output[count++]=b2Contact_GetData(b2ContactId{contact->contactId+1,id.world0,0,contact->generation});
        }
        key=contact->edges[key&1].nextKey;
    }
    return count;
}

void b2Body_EnableContactRecycling(b2BodyId id,bool flag)
{
    if(!b2Body_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    b2World* world=b2_worlds+id.world0;b2Body* body=b2GetBodyFullId(world,id);
    uint32_t newFlag=flag?b2_bodyEnableContactRecycling:0;
    if((body->flags&b2_bodyEnableContactRecycling)==newFlag) { return; }
    body->flags=(body->flags&~b2_bodyEnableContactRecycling)|newFlag;b2SyncBodyFlags(world,body);
}

bool b2Body_IsContactRecyclingEnabled(b2BodyId id)
{
    return b2Body_IsValid(id) && (b2GetBodyFullId(b2_worlds+id.world0,id)->flags&b2_bodyEnableContactRecycling)!=0;
}

void b2Body_SetMotionLocks(b2BodyId id,b2MotionLocks locks)
{
    if(!b2Body_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    b2World* world=b2_worlds+id.world0;b2Body* body=b2GetBodyFullId(world,id);
    uint32_t newFlags=(locks.linearX?b2_lockLinearX:0)|(locks.linearY?b2_lockLinearY:0)|(locks.angularZ?b2_lockAngularZ:0);
    if((body->flags&b2_allLocks)==newFlags) { return; }
    body->flags=(body->flags&~b2_allLocks)|newFlags;b2SyncBodyFlags(world,body);
    b2BodyState* state=b2GetBodyState(world,body);
    if(state!=NULL)
    {
        if(locks.linearX) { state->linearVelocity.x=0; }
        if(locks.linearY) { state->linearVelocity.y=0; }
        if(locks.angularZ) { state->angularVelocity=0; }
    }
    b2UpdateBodyMassData(world,body);
}

b2MotionLocks b2Body_GetMotionLocks(b2BodyId id)
{
    B2_ASSERT(b2Body_IsValid(id));uint32_t flags=b2GetBodyFullId(b2_worlds+id.world0,id)->flags;
    return b2MotionLocks{(flags&b2_lockLinearX)!=0,(flags&b2_lockLinearY)!=0,(flags&b2_lockAngularZ)!=0};
}

void b2Body_Disable(b2BodyId id)
{
    if(!b2Body_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    b2World* world=b2_worlds+id.world0;b2Body* body=b2GetBodyFullId(world,id);
    if(body->setIndex==b2_disabledSet) { return; }
    // Removing support must wake connected bodies, including when this body is static.
    int key=body->headContactKey;
    while(key!=B2_NULL_INDEX)
    {
        b2Contact* contact=world->contacts.data+(key>>1);int edge=key&1;
        key=contact->edges[edge].nextKey;
        b2WakeBody(world,world->bodies.data+contact->edges[edge^1].bodyId);
    }
    b2DestroyBodyContacts(world,body);
    for(int shapeId=body->headShapeId;shapeId!=B2_NULL_INDEX;shapeId=world->shapes.data[shapeId].nextShapeId)
    { b2DestroyShapeProxy(world->shapes.data+shapeId,&world->broadPhase); }
    b2RemoveBodyFromIsland(world,body);
    b2TransferBody(world,world->solverSets.data+b2_disabledSet,world->solverSets.data+body->setIndex,body);
}

void b2Body_Enable(b2BodyId id)
{
    if(!b2Body_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    b2World* world=b2_worlds+id.world0;b2Body* body=b2GetBodyFullId(world,id);
    if(body->setIndex!=b2_disabledSet) { return; }
    int target=body->type==b2_staticBody?b2_staticSet:b2_awakeSet;
    b2TransferBody(world,world->solverSets.data+target,world->solverSets.data+b2_disabledSet,body);
    b2WorldTransform transform=b2GetBodySim(world,body)->transform;
    for(int shapeId=body->headShapeId;shapeId!=B2_NULL_INDEX;shapeId=world->shapes.data[shapeId].nextShapeId)
    { b2CreateShapeProxy(world,world->shapes.data+shapeId,body->type,transform,true); }
    if(target!=b2_staticSet) { b2CreateIslandForBody(world,target,body); }
}

bool b2Body_IsEnabled(b2BodyId id)
{
    return b2Body_IsValid(id) && b2GetBodyFullId(b2_worlds+id.world0,id)->setIndex!=b2_disabledSet;
}

bool b2Body_IsAwake(b2BodyId id)
{
    return b2Body_IsValid(id) && b2GetBodyFullId(b2_worlds+id.world0,id)->setIndex==b2_awakeSet;
}

void b2Body_SetAwake(b2BodyId id,bool awake)
{
    if(!b2Body_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    b2World* world=b2_worlds+id.world0;b2Body* body=b2GetBodyFullId(world,id);
    if(awake && body->setIndex>=b2_firstSleepingSet) { b2WakeBody(world,body); }
    else if(!awake && body->setIndex==b2_awakeSet)
    {
        if(world->islands.data[body->islandId].constraintRemoveCount>0) { b2SplitIsland(world,body->islandId); }
        b2TrySleepIsland(world,body->islandId);
    }
}

void b2Body_WakeTouching(b2BodyId id)
{
    if(!b2Body_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    b2World* world=b2_worlds+id.world0;b2Body* body=b2GetBodyFullId(world,id);
    int key=body->headContactKey;
    while(key!=B2_NULL_INDEX)
    {
        b2Contact* contact=world->contacts.data+(key>>1);int edge=key&1;
        key=contact->edges[edge].nextKey;
        b2WakeBody(world,world->bodies.data+contact->edges[edge^1].bodyId);
    }
}

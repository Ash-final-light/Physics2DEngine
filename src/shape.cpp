// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "shape.h"
#include "body.h"
#include "core.h"
#include "world.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

void b2UpdateShapeAABB( b2Shape* shape, b2WorldTransform transform )
{
    B2_ASSERT( shape->type == b2_circleShape );
    b2Circle expanded = shape->circle;
    expanded.radius += B2_SPECULATIVE_DISTANCE;
    shape->aabb = b2ComputeCircleBounds( &expanded, transform );
}

bool b2Shape_IsValid( b2ShapeId id )
{
    if ( id.world0 >= B2_MAX_WORLDS )
    {
        return false;
    }
    b2World* world = b2_worlds + id.world0;
    if ( world->inUse == false || id.index1 < 1 || id.index1 > world->shapes.count )
    {
        return false;
    }
    b2Shape* shape = world->shapes.data + id.index1 - 1;
    return shape->id == id.index1 - 1 && shape->generation == id.generation;
}

static bool b2IsValidMaterial( const b2SurfaceMaterial* material )
{
    return b2IsValidFloat( material->friction ) && material->friction >= 0.0f &&
           b2IsValidFloat( material->restitution ) && material->restitution >= 0.0f &&
           b2IsValidFloat( material->rollingResistance ) && material->rollingResistance >= 0.0f &&
           b2IsValidFloat( material->tangentSpeed );
}

b2ShapeId b2CreateCircleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle )
{
    B2_CHECK_DEF( def );
    if ( circle == NULL || b2IsValidVec2( circle->center ) == false ||
         b2IsValidFloat( circle->radius ) == false || circle->radius < 0.0f ||
         b2IsValidFloat( def->density ) == false || def->density < 0.0f ||
         b2IsValidMaterial( &def->material ) == false || b2Body_IsValid( bodyId ) == false )
    {
        return b2_nullShapeId;
    }
    b2World* world = b2_worlds + bodyId.world0;
    if ( world->locked )
    {
        return b2_nullShapeId;
    }
    b2Body* body = b2GetBodyFullId( world, bodyId );
    int id = b2AllocId( &world->shapeIdPool );
    if ( id == world->shapes.count )
    {
        b2Array_Push( world->shapes, b2Shape{} );
    }
    b2Shape* shape = world->shapes.data + id;
    uint16_t generation = (uint16_t)( shape->generation + 1 );
    *shape = b2Shape{};
    shape->id = id;
    shape->generation = generation;
    shape->bodyId = body->id;
    shape->type = b2_circleShape;
    shape->circle = *circle;
    shape->density = def->density;
    shape->material = def->material;
    shape->filter = def->filter;
    shape->userData = def->userData;
    shape->prevShapeId = B2_NULL_INDEX;
    shape->nextShapeId = body->headShapeId;
    if ( body->headShapeId != B2_NULL_INDEX )
    {
        world->shapes.data[body->headShapeId].prevShapeId = id;
    }
    body->headShapeId = id;
    body->shapeCount += 1;
    body->dirtyMass = true;
    b2UpdateShapeAABB( shape, body->transform );
    return b2ShapeId{ id + 1, world->worldId, generation };
}

void b2DestroyShape( b2ShapeId shapeId, bool updateBodyMass )
{
    if ( b2Shape_IsValid( shapeId ) == false )
    {
        return;
    }
    b2World* world = b2_worlds + shapeId.world0;
    if ( world->locked )
    {
        return;
    }
    b2Shape* shape = world->shapes.data + shapeId.index1 - 1;
    b2Body* body = world->bodies.data + shape->bodyId;
    if ( shape->prevShapeId != B2_NULL_INDEX )
    {
        world->shapes.data[shape->prevShapeId].nextShapeId = shape->nextShapeId;
    }
    if ( shape->nextShapeId != B2_NULL_INDEX )
    {
        world->shapes.data[shape->nextShapeId].prevShapeId = shape->prevShapeId;
    }
    if ( body->headShapeId == shape->id )
    {
        body->headShapeId = shape->nextShapeId;
    }
    body->shapeCount -= 1;
    body->dirtyMass = body->dirtyMass || updateBodyMass;
    b2FreeId( &world->shapeIdPool, shape->id );
    uint16_t generation = shape->generation;
    *shape = b2Shape{};
    shape->generation = generation;
    shape->id = B2_NULL_INDEX;
}

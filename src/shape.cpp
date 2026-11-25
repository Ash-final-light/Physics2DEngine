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
    switch ( shape->type )
    {
        case b2_circleShape:
        {
            b2Circle expanded = shape->circle;
            expanded.radius += B2_SPECULATIVE_DISTANCE;
            shape->aabb = b2ComputeCircleBounds( &expanded, transform );
        }
        break;
        case b2_capsuleShape:
        {
            b2Capsule expanded = shape->capsule;
            expanded.radius += B2_SPECULATIVE_DISTANCE;
            shape->aabb = b2ComputeCapsuleBounds( &expanded, transform );
        }
        break;
        case b2_polygonShape:
        {
            b2Polygon expanded = shape->polygon;
            expanded.radius += B2_SPECULATIVE_DISTANCE;
            shape->aabb = b2ComputePolygonBounds( &expanded, transform );
        }
        break;
        case b2_segmentShape:
        {
            b2AABB bounds = b2ComputeSegmentBounds( &shape->segment, transform );
            b2Vec2 margin = { B2_SPECULATIVE_DISTANCE, B2_SPECULATIVE_DISTANCE };
            shape->aabb = { b2Sub( bounds.lowerBound, margin ), b2Add( bounds.upperBound, margin ) };
        }
        break;
        default:
            B2_ASSERT( false );
    }
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

static b2ShapeId b2CreateShape( b2BodyId bodyId, const b2ShapeDef* def, const void* geometry, b2ShapeType type )
{
    B2_CHECK_DEF( def );
    if ( b2IsValidFloat( def->density ) == false || def->density < 0.0f ||
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
    shape->type = type;
    switch ( type )
    {
        case b2_circleShape: shape->circle = *(const b2Circle*)geometry; break;
        case b2_capsuleShape: shape->capsule = *(const b2Capsule*)geometry; break;
        case b2_polygonShape: shape->polygon = *(const b2Polygon*)geometry; break;
        case b2_segmentShape: shape->segment = *(const b2Segment*)geometry; break;
        default: B2_ASSERT( false ); break;
    }
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
    if ( def->updateBodyMass )
    {
        b2UpdateBodyMassData( world, body );
    }
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
    b2FreeId( &world->shapeIdPool, shape->id );
    uint16_t generation = shape->generation;
    *shape = b2Shape{};
    shape->generation = generation;
    shape->id = B2_NULL_INDEX;
    if ( updateBodyMass )
    {
        b2UpdateBodyMassData( world, body );
    }
}


b2ShapeId b2CreateCircleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle )
{
    if ( circle == NULL || b2IsValidVec2( circle->center ) == false ||
         b2IsValidFloat( circle->radius ) == false || circle->radius < 0.0f )
    {
        return b2_nullShapeId;
    }
    return b2CreateShape( bodyId, def, circle, b2_circleShape );
}

b2ShapeId b2CreateCapsuleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule )
{
    if ( capsule == NULL || b2IsValidVec2( capsule->center1 ) == false ||
         b2IsValidVec2( capsule->center2 ) == false || b2IsValidFloat( capsule->radius ) == false ||
         capsule->radius < 0.0f || b2DistanceSquared( capsule->center1, capsule->center2 ) <= B2_LINEAR_SLOP * B2_LINEAR_SLOP )
    {
        return b2_nullShapeId;
    }
    return b2CreateShape( bodyId, def, capsule, b2_capsuleShape );
}


static bool b2IsValidPolygon( const b2Polygon* polygon )
{
    if ( polygon == NULL || polygon->count < 1 || polygon->count > B2_MAX_POLYGON_VERTICES ||
         b2IsValidFloat( polygon->radius ) == false || polygon->radius < 0.0f ||
         b2IsValidVec2( polygon->centroid ) == false )
    {
        return false;
    }
    for ( int i = 0; i < polygon->count; ++i )
    {
        if ( b2IsValidVec2( polygon->vertices[i] ) == false || b2IsValidVec2( polygon->normals[i] ) == false )
        {
            return false;
        }
    }
    return true;
}

b2ShapeId b2CreatePolygonShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon )
{
    if ( b2IsValidPolygon( polygon ) == false )
    {
        return b2_nullShapeId;
    }
    return b2CreateShape( bodyId, def, polygon, b2_polygonShape );
}


b2ShapeId b2CreateSegmentShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment )
{
    if ( segment == NULL || b2IsValidVec2( segment->point1 ) == false || b2IsValidVec2( segment->point2 ) == false ||
         b2DistanceSquared( segment->point1, segment->point2 ) <= B2_LINEAR_SLOP * B2_LINEAR_SLOP )
    {
        return b2_nullShapeId;
    }
    return b2CreateShape( bodyId, def, segment, b2_segmentShape );
}


static b2Shape* b2GetShapeFromId( b2ShapeId id )
{
    if ( b2Shape_IsValid( id ) == false )
    {
        return NULL;
    }
    return b2_worlds[id.world0].shapes.data + id.index1 - 1;
}

void b2Shape_SetUserData( b2ShapeId id, void* userData )
{
    b2Shape* shape = b2GetShapeFromId( id );
    if ( shape != NULL && b2_worlds[id.world0].locked == false )
    {
        shape->userData = userData;
    }
}

void* b2Shape_GetUserData( b2ShapeId id )
{
    b2Shape* shape = b2GetShapeFromId( id );
    return shape != NULL ? shape->userData : NULL;
}


b2ShapeType b2Shape_GetType( b2ShapeId id )
{
    b2Shape* shape = b2GetShapeFromId( id );
    B2_ASSERT( shape != NULL );
    return shape->type;
}

static b2Shape* b2GetShapeWithType( b2ShapeId id, b2ShapeType type )
{
    b2Shape* shape = b2GetShapeFromId( id );
    B2_ASSERT( shape != NULL && shape->type == type );
    return shape;
}

static void b2SetShapeGeometry( b2ShapeId id, const void* geometry, b2ShapeType type )
{
    b2Shape* shape = b2GetShapeFromId( id );
    if ( shape == NULL || b2_worlds[id.world0].locked )
    {
        return;
    }
    switch ( type )
    {
        case b2_circleShape: shape->circle = *(const b2Circle*)geometry; break;
        case b2_capsuleShape: shape->capsule = *(const b2Capsule*)geometry; break;
        case b2_polygonShape: shape->polygon = *(const b2Polygon*)geometry; break;
        case b2_segmentShape: shape->segment = *(const b2Segment*)geometry; break;
        default: B2_ASSERT( false ); return;
    }
    shape->type = type;
    b2Body* body = b2_worlds[id.world0].bodies.data + shape->bodyId;
    b2UpdateShapeAABB( shape, body->transform );
}

b2Circle b2Shape_GetCircle( b2ShapeId id )
{
    return b2GetShapeWithType( id, b2_circleShape )->circle;
}

b2Capsule b2Shape_GetCapsule( b2ShapeId id )
{
    return b2GetShapeWithType( id, b2_capsuleShape )->capsule;
}

b2Polygon b2Shape_GetPolygon( b2ShapeId id )
{
    return b2GetShapeWithType( id, b2_polygonShape )->polygon;
}

b2Segment b2Shape_GetSegment( b2ShapeId id )
{
    return b2GetShapeWithType( id, b2_segmentShape )->segment;
}


void b2Shape_SetCircle( b2ShapeId id, const b2Circle* circle )
{
    if ( circle != NULL && b2IsValidVec2( circle->center ) && b2IsValidFloat( circle->radius ) && circle->radius >= 0.0f )
    {
        b2SetShapeGeometry( id, circle, b2_circleShape );
    }
}

void b2Shape_SetCapsule( b2ShapeId id, const b2Capsule* capsule )
{
    if ( capsule != NULL && b2IsValidVec2( capsule->center1 ) && b2IsValidVec2( capsule->center2 ) &&
         b2IsValidFloat( capsule->radius ) && capsule->radius >= 0.0f &&
         b2DistanceSquared( capsule->center1, capsule->center2 ) > B2_LINEAR_SLOP * B2_LINEAR_SLOP )
    {
        b2SetShapeGeometry( id, capsule, b2_capsuleShape );
    }
}

void b2Shape_SetPolygon( b2ShapeId id, const b2Polygon* polygon )
{
    if ( b2IsValidPolygon( polygon ) )
    {
        b2SetShapeGeometry( id, polygon, b2_polygonShape );
    }
}

void b2Shape_SetSegment( b2ShapeId id, const b2Segment* segment )
{
    if ( segment != NULL && b2IsValidVec2( segment->point1 ) && b2IsValidVec2( segment->point2 ) )
    {
        b2SetShapeGeometry( id, segment, b2_segmentShape );
    }
}


b2MassData b2ComputeShapeMass( const b2Shape* shape )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
			return b2ComputeCapsuleMass( &shape->capsule, shape->density );
		case b2_circleShape:
			return b2ComputeCircleMass( &shape->circle, shape->density );
		case b2_polygonShape:
			return b2ComputePolygonMass( &shape->polygon, shape->density );
		default:
			return b2MassData{};
	}
}

b2ShapeExtent b2ComputeShapeExtent( const b2Shape* shape, b2Vec2 localCenter )
{
	b2ShapeExtent extent = {};

	switch ( shape->type )
	{
		case b2_capsuleShape:
		{
			float radius = shape->capsule.radius;
			extent.minExtent = radius;
			b2Vec2 c1 = b2Sub( shape->capsule.center1, localCenter );
			b2Vec2 c2 = b2Sub( shape->capsule.center2, localCenter );
			extent.maxExtent =
				sqrtf( b2MaxFloat( b2LengthSquared( c1 ), b2LengthSquared( c2 ) ) ) + radius;
		}
		break;

		case b2_circleShape:
		{
			float radius = shape->circle.radius;
			extent.minExtent = radius;
			extent.maxExtent = b2Length( b2Sub( shape->circle.center, localCenter ) ) + radius;
		}
		break;

		case b2_polygonShape:
		{
			const b2Polygon* poly = &shape->polygon;
			float minExtent = B2_HUGE;
			float maxExtentSqr = 0.0f;
			int count = poly->count;
			for ( int i = 0; i < count; ++i )
			{
				b2Vec2 v = poly->vertices[i];
				float planeOffset = b2Dot( poly->normals[i], b2Sub( v, poly->centroid ) );
				minExtent = b2MinFloat( minExtent, planeOffset );

				float distanceSqr = b2LengthSquared( b2Sub( v, localCenter ) );
				maxExtentSqr = b2MaxFloat( maxExtentSqr, distanceSqr );
			}

			extent.minExtent = minExtent + poly->radius;
			extent.maxExtent = sqrtf( maxExtentSqr ) + poly->radius;
		}
		break;

		case b2_segmentShape:
		{
			extent.minExtent = 0.0f;
			b2Vec2 c1 = b2Sub( shape->segment.point1, localCenter );
			b2Vec2 c2 = b2Sub( shape->segment.point2, localCenter );
			extent.maxExtent = sqrtf( b2MaxFloat( b2LengthSquared( c1 ), b2LengthSquared( c2 ) ) );
		}
		break;

		default:
			break;
	}

	return extent;
}


void b2Shape_SetDensity( b2ShapeId id, float density, bool updateBodyMass )
{
    b2Shape* shape = b2GetShapeFromId( id );
    if ( shape == NULL || b2_worlds[id.world0].locked || b2IsValidFloat( density ) == false || density < 0.0f || shape->density == density )
    {
        return;
    }
    shape->density = density;
    if ( updateBodyMass )
    {
        b2World* world = b2_worlds + id.world0;
        b2UpdateBodyMassData( world, world->bodies.data + shape->bodyId );
    }
}

float b2Shape_GetDensity( b2ShapeId id )
{
    b2Shape* shape = b2GetShapeFromId( id );
    B2_ASSERT( shape != NULL );
    return shape->density;
}

b2MassData b2Shape_ComputeMassData( b2ShapeId id )
{
    b2Shape* shape = b2GetShapeFromId( id );
    B2_ASSERT( shape != NULL );
    return b2ComputeShapeMass( shape );
}

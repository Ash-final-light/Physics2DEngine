// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "shape.h"
#include "recording.h"
#include "body.h"
#include "core.h"
#include "contact.h"
#include "world.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

static float b2ComputeShapeMargin( b2Shape* shape )
{
	float margin = 0.0f;

	switch ( shape->type )
	{
		case b2_capsuleShape:
		{
			margin = 0.5f * b2Distance( shape->capsule.center2, shape->capsule.center1 ) +
					 shape->capsule.radius;
		}
		break;

		case b2_circleShape:
		{
			margin = shape->circle.radius;
		}
		break;

		case b2_polygonShape:
		{
			const b2Polygon* poly = &shape->polygon;
			float maxExtentSqr = 0.0f;
			int count = poly->count;
			for ( int i = 0; i < count; ++i )
			{
				float distanceSqr = b2DistanceSquared( poly->vertices[i], poly->centroid );
				maxExtentSqr = b2MaxFloat( maxExtentSqr, distanceSqr );
			}

			margin = sqrtf( maxExtentSqr );
		}
		break;

		case b2_segmentShape:
		{
			margin = 0.5f * b2Distance( shape->segment.point1, shape->segment.point2 );
		}
		break;

		case b2_chainSegmentShape:
		{
			margin = 0.5f * b2Distance( shape->chainSegment.segment.point1,
										shape->chainSegment.segment.point2 );
		}
		break;

		default:
			B2_VALIDATE( false );
			return B2_MAX_AABB_MARGIN;
	}

	return b2MinFloat( B2_MAX_AABB_MARGIN, B2_AABB_MARGIN_FRACTION * margin );
}

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
        case b2_chainSegmentShape:
        {
            const b2Segment* segment = shape->type == b2_segmentShape ? &shape->segment : &shape->chainSegment.segment;
            b2AABB bounds = b2ComputeSegmentBounds( segment, transform );
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
        b2Array_Push( world->fatAABBs, b2AABB{} );
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
        case b2_chainSegmentShape: shape->chainSegment = *(const b2ChainSegment*)geometry; break;
        default: B2_ASSERT( false ); break;
    }
    shape->density = def->density;
    shape->proxyKey = B2_NULL_INDEX;
    shape->aabbMargin = b2ComputeShapeMargin( shape );
    shape->material = def->material;
    shape->filter = def->filter;
    shape->userData = def->userData;
    shape->enableContactEvents=def->enableContactEvents;
    shape->enableSensorEvents=def->enableSensorEvents;
    shape->enableCustomFiltering=def->enableCustomFiltering;
    shape->sensorIndex=B2_NULL_INDEX;
    shape->enableHitEvents=def->enableHitEvents;
    shape->enablePreSolveEvents=def->enablePreSolveEvents;
    shape->prevShapeId = B2_NULL_INDEX;
    shape->nextShapeId = body->headShapeId;
    if ( body->headShapeId != B2_NULL_INDEX )
    {
        world->shapes.data[body->headShapeId].prevShapeId = id;
    }
    body->headShapeId = id;
    body->shapeCount += 1;
    if ( type != b2_chainSegmentShape )
    {
        body->flags |= b2_dirtyMass;
        b2SyncBodyFlags(b2_worlds+bodyId.world0,body);
        if ( def->updateBodyMass )
        {
            b2UpdateBodyMassData( world, body );
        }
    }
    b2UpdateShapeAABB( shape, b2GetBodySim(world,body)->transform );
    if ( (body->setIndex != b2_disabledSet) )
    {
        b2CreateShapeProxy( world, shape, body->type, b2GetBodySim(world,body)->transform, def->invokeContactCreation );
    }
    if(def->isSensor)
    {
        shape->sensorIndex=world->sensors.count;b2Sensor sensor={};sensor.shapeId=id;
        b2Array_CreateN(sensor.hits,4);b2Array_CreateN(sensor.overlaps1,16);b2Array_CreateN(sensor.overlaps2,16);
        b2Array_Push(world->sensors,sensor);
    }
    return b2ShapeId{ id + 1, world->worldId, generation };
}

static void b2DestroyShapeInternal( b2World* world, b2Shape* shape, b2Body* body )
{
    b2DestroyShapeContacts(world,shape);
    if(shape->sensorIndex!=B2_NULL_INDEX) { b2DestroySensor(world,shape); }
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
    b2DestroyShapeProxy( shape, &world->broadPhase );
    b2FreeId( &world->shapeIdPool, shape->id );
    uint16_t generation = shape->generation;
    *shape = b2Shape{};
    shape->generation = generation;
    shape->id = B2_NULL_INDEX;
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
    if ( shape->type == b2_chainSegmentShape && shape->chainSegment.chainId != B2_NULL_INDEX )
    {
        return;
    }
    B2_REC( world, DestroyShape, shapeId, updateBodyMass );
    b2Body* body = world->bodies.data + shape->bodyId;
    b2DestroyShapeInternal( world, shape, body );
    if ( updateBodyMass )
    {
        b2UpdateBodyMassData( world, body );
    }
}



static bool b2IsValidChainSegment( const b2ChainSegment* segment )
{
    return segment != NULL && b2IsValidVec2( segment->ghost1 ) &&
           b2IsValidVec2( segment->segment.point1 ) && b2IsValidVec2( segment->segment.point2 ) &&
           b2IsValidVec2( segment->ghost2 );
}

static void b2FixGhostVertices( b2ChainSegment* chainSegment )
{
	b2Vec2 p1 = chainSegment->segment.point1;
	b2Vec2 p2 = chainSegment->segment.point2;
	float tolSqr = B2_LINEAR_SLOP * B2_LINEAR_SLOP;

	// 最坏情况下法线会计算两次，但正常情况两者都不应发生。

	if ( b2DistanceSquared( chainSegment->ghost1, p1 ) <= tolSqr )
	{
		b2Vec2 normal = b2RightPerp( b2Normalize( b2Sub( p2, p1 ) ) );
		chainSegment->ghost1 = b2Sub( p1, normal );
	}

	if ( b2DistanceSquared( chainSegment->ghost2, p2 ) <= tolSqr )
	{
		b2Vec2 normal = b2RightPerp( b2Normalize( b2Sub( p2, p1 ) ) );
		chainSegment->ghost2 = b2Sub( p2, normal );
	}
}

b2ShapeId b2CreateCircleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle )
{
    if ( circle == NULL || b2IsValidVec2( circle->center ) == false ||
         b2IsValidFloat( circle->radius ) == false || circle->radius < 0.0f )
    {
        return b2_nullShapeId;
    }
    b2ShapeId id = b2CreateShape( bodyId, def, circle, b2_circleShape );
    if ( id.index1 != 0 )
    {
        B2_REC_CREATE( ( b2_worlds + bodyId.world0 ), CreateCircleShape, id, bodyId, *def, *circle );
    }
    return id;
}

b2ShapeId b2CreateCapsuleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule )
{
    if ( capsule == NULL || b2IsValidVec2( capsule->center1 ) == false ||
         b2IsValidVec2( capsule->center2 ) == false || b2IsValidFloat( capsule->radius ) == false ||
         capsule->radius < 0.0f || b2DistanceSquared( capsule->center1, capsule->center2 ) <= B2_LINEAR_SLOP * B2_LINEAR_SLOP )
    {
        return b2_nullShapeId;
    }
    b2ShapeId id = b2CreateShape( bodyId, def, capsule, b2_capsuleShape );
    if ( id.index1 != 0 )
    {
        B2_REC_CREATE( ( b2_worlds + bodyId.world0 ), CreateCapsuleShape, id, bodyId, *def, *capsule );
    }
    return id;
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
    b2ShapeId id = b2CreateShape( bodyId, def, polygon, b2_polygonShape );
    if ( id.index1 != 0 )
    {
        B2_REC_CREATE( ( b2_worlds + bodyId.world0 ), CreatePolygonShape, id, bodyId, *def, *polygon );
    }
    return id;
}


b2ShapeId b2CreateSegmentShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment )
{
    if ( segment == NULL || b2IsValidVec2( segment->point1 ) == false || b2IsValidVec2( segment->point2 ) == false ||
         b2DistanceSquared( segment->point1, segment->point2 ) <= B2_LINEAR_SLOP * B2_LINEAR_SLOP )
    {
        return b2_nullShapeId;
    }
    b2ShapeId id = b2CreateShape( bodyId, def, segment, b2_segmentShape );
    if ( id.index1 != 0 )
    {
        B2_REC_CREATE( ( b2_worlds + bodyId.world0 ), CreateSegmentShape, id, bodyId, *def, *segment );
    }
    return id;
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
    b2World* world = b2_worlds + id.world0;
    b2DestroyShapeContacts(world,shape);
    if(shape->sensorIndex!=B2_NULL_INDEX) { b2DestroySensor(world,shape); }
    b2Body* body = world->bodies.data + shape->bodyId;
    shape->aabbMargin = b2ComputeShapeMargin( shape );
    b2DestroyShapeProxy( shape, &world->broadPhase );
    if ( (body->setIndex != b2_disabledSet) )
    {
        b2CreateShapeProxy( world, shape, body->type, b2GetBodySim(world,body)->transform, true );
    }
    else
    {
        b2UpdateShapeAABB( shape, b2GetBodySim(world,body)->transform );
    }
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
        if ( b2Shape_IsValid( id ) && !b2_worlds[id.world0].locked )
        {
            B2_REC( ( b2_worlds + id.world0 ), ShapeSetCircle, id, *circle );
        }
        b2SetShapeGeometry( id, circle, b2_circleShape );
    }
}

void b2Shape_SetCapsule( b2ShapeId id, const b2Capsule* capsule )
{
    if ( capsule != NULL && b2IsValidVec2( capsule->center1 ) && b2IsValidVec2( capsule->center2 ) &&
         b2IsValidFloat( capsule->radius ) && capsule->radius >= 0.0f &&
         b2DistanceSquared( capsule->center1, capsule->center2 ) > B2_LINEAR_SLOP * B2_LINEAR_SLOP )
    {
        if ( b2Shape_IsValid( id ) && !b2_worlds[id.world0].locked )
        {
            B2_REC( ( b2_worlds + id.world0 ), ShapeSetCapsule, id, *capsule );
        }
        b2SetShapeGeometry( id, capsule, b2_capsuleShape );
    }
}

void b2Shape_SetPolygon( b2ShapeId id, const b2Polygon* polygon )
{
    if ( b2IsValidPolygon( polygon ) )
    {
        if ( b2Shape_IsValid( id ) && !b2_worlds[id.world0].locked )
        {
            B2_REC( ( b2_worlds + id.world0 ), ShapeSetPolygon, id, *polygon );
        }
        b2SetShapeGeometry( id, polygon, b2_polygonShape );
    }
}

void b2Shape_SetSegment( b2ShapeId id, const b2Segment* segment )
{
    if ( segment != NULL && b2IsValidVec2( segment->point1 ) && b2IsValidVec2( segment->point2 ) )
    {
        if ( b2Shape_IsValid( id ) && !b2_worlds[id.world0].locked )
        {
            B2_REC( ( b2_worlds + id.world0 ), ShapeSetSegment, id, *segment );
        }
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

        case b2_chainSegmentShape:
        {
            b2Vec2 c1 = b2Sub( shape->chainSegment.segment.point1, localCenter );
            b2Vec2 c2 = b2Sub( shape->chainSegment.segment.point2, localCenter );
            extent.minExtent = 0.0f;
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
    B2_REC( ( b2_worlds + id.world0 ), ShapeSetDensity, id, density, updateBodyMass );
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


bool b2Chain_IsValid( b2ChainId id )
{
    if ( id.world0 >= B2_MAX_WORLDS )
    {
        return false;
    }
    b2World* world = b2_worlds + id.world0;
    if ( world->inUse == false || id.index1 < 1 || id.index1 > world->chainShapes.count )
    {
        return false;
    }
    const b2ChainShape* chain = world->chainShapes.data + id.index1 - 1;
    return chain->id == id.index1 - 1 && chain->generation == id.generation;
}

b2ChainId b2CreateChain( b2BodyId bodyId, const b2ChainDef* def )
{
    B2_CHECK_DEF( def );
    if ( b2Body_IsValid( bodyId ) == false || def->points == NULL ||
         def->pointCount < (def->isLoop ? 3 : 2) || b2_worlds[bodyId.world0].locked )
    {
        return b2_nullChainId;
    }
    int n = def->isLoop ? def->pointCount : def->pointCount - 1;
    if ( def->materials == NULL || (def->materialCount != 1 && def->materialCount != n) )
    {
        return b2_nullChainId;
    }
    for ( int i = 0; i < def->materialCount; ++i )
    {
        if ( b2IsValidMaterial( def->materials + i ) == false )
        {
            return b2_nullChainId;
        }
    }
    if ( def->isLoop == false && (b2IsValidVec2( def->ghost1 ) == false || b2IsValidVec2( def->ghost2 ) == false) )
    {
        return b2_nullChainId;
    }
    for ( int i = 0; i < def->pointCount; ++i )
    {
        if ( b2IsValidVec2( def->points[i] ) == false )
        {
            return b2_nullChainId;
        }
    }
    float tolerance = B2_LINEAR_SLOP * B2_LINEAR_SLOP;
    for ( int i = 0; i < n; ++i )
    {
        if ( b2DistanceSquared( def->points[i], def->points[(i+1)%def->pointCount] ) <= tolerance )
        {
            return b2_nullChainId;
        }
    }
    if ( def->isLoop == false &&
         (b2DistanceSquared( def->ghost1, def->points[0] ) <= tolerance ||
          b2DistanceSquared( def->ghost2, def->points[def->pointCount-1] ) <= tolerance) )
    {
        return b2_nullChainId;
    }
    b2World* world = b2_worlds + bodyId.world0;
    b2Body* body = b2GetBodyFullId( world, bodyId );
    int id = b2AllocId( &world->chainIdPool );
    if ( id == world->chainShapes.count )
    {
        b2Array_Push( world->chainShapes, b2ChainShape{} );
    }
    b2ChainShape* chain = world->chainShapes.data + id;
    uint16_t generation = (uint16_t)( chain->generation + 1 );
    *chain = b2ChainShape{};
    chain->id = id;
    chain->generation = generation;
    chain->bodyId = body->id;
    chain->nextChainId = body->headChainId;
    chain->segmentCount = n;
    chain->shapeIndices = static_cast<int*>( b2Alloc( n * sizeof( int ) ) );
    body->headChainId = id;
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.userData = def->userData;
    shapeDef.filter = def->filter;
    shapeDef.updateBodyMass = false;
    shapeDef.enableSensorEvents=def->enableSensorEvents;
    for ( int i = 0; i < n; ++i )
    {
        b2ChainSegment segment;
        segment.ghost1 = i == 0 ? (def->isLoop ? def->points[n-1] : def->ghost1) : def->points[i-1];
        segment.segment.point1 = def->points[i];
        segment.segment.point2 = def->points[(i+1)%def->pointCount];
        segment.ghost2 = def->isLoop ? def->points[(i+2)%n] : (i == n-1 ? def->ghost2 : def->points[i+2]);
        segment.chainId = id;
        shapeDef.material = def->materials[def->materialCount == 1 ? 0 : i];
        b2ShapeId shape = b2CreateShape( bodyId, &shapeDef, &segment, b2_chainSegmentShape );
        B2_ASSERT( shape.index1 != 0 );
        chain->shapeIndices[i] = shape.index1 - 1;
    }
    b2ChainId result = { id + 1, world->worldId, generation };
    B2_REC_CREATE( world, CreateChain, result, bodyId, *def );
    return result;
}

void b2DestroyChain( b2ChainId id )
{
    if ( b2Chain_IsValid( id ) == false || b2_worlds[id.world0].locked )
    {
        return;
    }
    b2World* world = b2_worlds + id.world0;
    B2_REC( world, DestroyChain, id );
    b2ChainShape* chain = world->chainShapes.data + id.index1 - 1;
    b2Body* body = world->bodies.data + chain->bodyId;
    int* link = &body->headChainId;
    while ( *link != B2_NULL_INDEX && *link != chain->id )
    {
        link = &world->chainShapes.data[*link].nextChainId;
    }
    B2_ASSERT( *link == chain->id );
    *link = chain->nextChainId;
    for ( int i = 0; i < chain->segmentCount; ++i )
    {
        b2DestroyShapeInternal( world, world->shapes.data + chain->shapeIndices[i], body );
    }
    b2FreeChainData( chain );
    b2FreeId( &world->chainIdPool, chain->id );
    uint16_t generation = chain->generation;
    *chain = b2ChainShape{};
    chain->generation = generation;
    chain->id = B2_NULL_INDEX;
}


static b2ChainShape* b2GetChainFromId( b2ChainId id )
{
    if ( b2Chain_IsValid( id ) == false )
    {
        return NULL;
    }
    return b2_worlds[id.world0].chainShapes.data + id.index1 - 1;
}

b2WorldId b2Chain_GetWorld( b2ChainId id )
{
    B2_ASSERT( b2Chain_IsValid( id ) );
    return b2WorldId{ (uint16_t)(id.world0+1), b2_worlds[id.world0].generation };
}

int b2Chain_GetSegmentCount( b2ChainId id )
{
    b2ChainShape* chain = b2GetChainFromId( id );
    return chain != NULL && b2_worlds[id.world0].locked == false ? chain->segmentCount : 0;
}

int b2Chain_GetSegments( b2ChainId id, b2ShapeId* output, int capacity )
{
    b2ChainShape* chain = b2GetChainFromId( id );
    if ( chain == NULL || capacity <= 0 || output == NULL || b2_worlds[id.world0].locked )
    {
        return 0;
    }
    int count = b2MinInt( chain->segmentCount, capacity );
    for ( int i = 0; i < count; ++i )
    {
        int shapeId = chain->shapeIndices[i];
        output[i] = b2ShapeId{ shapeId+1, id.world0, b2_worlds[id.world0].shapes.data[shapeId].generation };
    }
    return count;
}

b2ChainSegment b2Shape_GetChainSegment( b2ShapeId id )
{
    return b2GetShapeWithType( id, b2_chainSegmentShape )->chainSegment;
}

b2ChainId b2Shape_GetParentChain( b2ShapeId id )
{
    b2Shape* shape = b2GetShapeFromId( id );
    if ( shape == NULL || shape->type != b2_chainSegmentShape || shape->chainSegment.chainId == B2_NULL_INDEX )
    {
        return b2_nullChainId;
    }
    int chainId = shape->chainSegment.chainId;
    return b2ChainId{ chainId+1, id.world0, b2_worlds[id.world0].chainShapes.data[chainId].generation };
}


void b2Chain_SetSurfaceMaterial( b2ChainId id, const b2SurfaceMaterial* material, int segmentIndex )
{
    b2ChainShape* chain = b2GetChainFromId( id );
    if ( chain == NULL || material == NULL || b2IsValidMaterial( material ) == false ||
         segmentIndex < 0 || segmentIndex >= chain->segmentCount || b2_worlds[id.world0].locked )
    {
        return;
    }
    B2_REC( ( b2_worlds + id.world0 ), ChainSetSurfaceMaterial, id, *material, segmentIndex );
    b2_worlds[id.world0].shapes.data[chain->shapeIndices[segmentIndex]].material = *material;
}

void b2Chain_SetAllSurfaceMaterials( b2ChainId id, const b2SurfaceMaterial* material )
{
    b2ChainShape* chain = b2GetChainFromId( id );
    if ( chain == NULL || material == NULL || b2IsValidMaterial( material ) == false || b2_worlds[id.world0].locked )
    {
        return;
    }
    B2_REC( ( b2_worlds + id.world0 ), ChainSetAllSurfaceMaterials, id, *material );
    for ( int i = 0; i < chain->segmentCount; ++i )
    {
        b2_worlds[id.world0].shapes.data[chain->shapeIndices[i]].material = *material;
    }
}

b2SurfaceMaterial b2Chain_GetSurfaceMaterial( b2ChainId id, int segmentIndex )
{
    b2ChainShape* chain = b2GetChainFromId( id );
    B2_ASSERT( chain != NULL && segmentIndex >= 0 && segmentIndex < chain->segmentCount );
    return b2_worlds[id.world0].shapes.data[chain->shapeIndices[segmentIndex]].material;
}


b2BodyId b2Shape_GetBody( b2ShapeId id )
{
    b2Shape* shape = b2GetShapeFromId( id );
    B2_ASSERT( shape != NULL );
    b2Body* body = b2_worlds[id.world0].bodies.data + shape->bodyId;
    return b2BodyId{ body->id+1, id.world0, body->generation };
}

b2WorldId b2Shape_GetWorld( b2ShapeId id )
{
    B2_ASSERT( b2Shape_IsValid( id ) );
    return b2WorldId{ (uint16_t)(id.world0+1), b2_worlds[id.world0].generation };
}


void b2CreateShapeProxy( b2World* world, b2Shape* shape, b2BodyType type, b2WorldTransform transform, bool forcePairCreation )
{
    B2_ASSERT( shape->proxyKey == B2_NULL_INDEX );
    b2UpdateShapeAABB( shape, transform );
    float margin = type == b2_staticBody ? B2_SPECULATIVE_DISTANCE : shape->aabbMargin;
    b2Vec2 extra = { margin, margin };
    b2AABB fat = { b2Sub( shape->aabb.lowerBound, extra ), b2Add( shape->aabb.upperBound, extra ) };
    world->fatAABBs.data[shape->id] = fat;
    shape->proxyKey = b2BroadPhase_CreateProxy( &world->broadPhase, type, fat, shape->filter.categoryBits,
                                               shape->id, forcePairCreation );
}

void b2DestroyShapeProxy( b2Shape* shape, b2BroadPhase* bp )
{
    if ( shape->proxyKey != B2_NULL_INDEX )
    {
        b2BroadPhase_DestroyProxy( bp, shape->proxyKey );
        shape->proxyKey = B2_NULL_INDEX;
    }
}


void b2MoveShapeProxy( b2World* world, b2Shape* shape, b2WorldTransform transform )
{
    b2UpdateShapeAABB( shape, transform );
    b2AABB* fat = world->fatAABBs.data + shape->id;
    if ( b2AABB_Contains( *fat, shape->aabb ) == false )
    {
        b2Vec2 margin = { shape->aabbMargin, shape->aabbMargin };
        *fat = b2AABB{ b2Sub( shape->aabb.lowerBound, margin ), b2Add( shape->aabb.upperBound, margin ) };
        if ( shape->proxyKey != B2_NULL_INDEX )
        {
            b2BroadPhase_MoveProxy( &world->broadPhase, shape->proxyKey, *fat );
        }
    }
}


b2Filter b2Shape_GetFilter( b2ShapeId id )
{
    b2Shape* shape = b2GetShapeFromId( id );
    B2_ASSERT( shape != NULL );
    return shape->filter;
}

void b2Shape_SetFilter( b2ShapeId id, b2Filter filter )
{
    b2Shape* shape = b2GetShapeFromId( id );
    if ( shape == NULL || b2_worlds[id.world0].locked ||
         (shape->filter.categoryBits == filter.categoryBits && shape->filter.maskBits == filter.maskBits && shape->filter.groupIndex == filter.groupIndex) )
    {
        return;
    }
    b2World* world = b2_worlds+id.world0;
    B2_REC( world, ShapeSetFilter, id, filter );
    b2Body* body = world->bodies.data+shape->bodyId;
    b2DestroyShapeContacts(world,shape);
    if(shape->sensorIndex!=B2_NULL_INDEX) { b2DestroySensor(world,shape); }
    shape->filter = filter;
    b2DestroyShapeProxy( shape, &world->broadPhase );
    if ( (body->setIndex != b2_disabledSet) )
    {
        b2CreateShapeProxy( world, shape, body->type, b2GetBodySim(world,body)->transform, true );
    }
}


int b2Shape_GetContactCapacity( b2ShapeId id )
{
    b2Shape* shape=b2GetShapeFromId(id);
    if(shape==NULL || b2_worlds[id.world0].locked)
    {
        return 0;
    }
    return b2_worlds[id.world0].bodies.data[shape->bodyId].contactCount;
}

int b2Shape_GetContactData( b2ShapeId id, b2ContactData* output, int capacity )
{
    b2Shape* shape=b2GetShapeFromId(id);
    if(shape==NULL || output==NULL || capacity<=0 || b2_worlds[id.world0].locked)
    {
        return 0;
    }
    b2World* world=b2_worlds+id.world0;
    int key=world->bodies.data[shape->bodyId].headContactKey;
    int count=0;
    while(key!=B2_NULL_INDEX && count<capacity)
    {
        b2Contact* contact=world->contacts.data+(key>>1);
        if((contact->flags&b2_contactTouchingFlag)!=0 && (contact->shapeIdA==shape->id || contact->shapeIdB==shape->id))
        {
            output[count++]=b2Contact_GetData(b2ContactId{contact->contactId+1,id.world0,0,contact->generation});
        }
        key=contact->edges[key&1].nextKey;
    }
    return count;
}

void b2Shape_SetFriction(b2ShapeId id,float friction)
{
    if(!b2IsValidFloat(friction) || friction<0.0f || !b2Shape_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    B2_REC( ( b2_worlds + id.world0 ), ShapeSetFriction, id, friction );
    b2GetShapeFromId(id)->material.friction=friction;
}

float b2Shape_GetFriction(b2ShapeId id)
{
    b2Shape* shape=b2GetShapeFromId(id);return shape!=NULL?shape->material.friction:0.0f;
}

void b2Shape_SetSurfaceMaterial(b2ShapeId id,const b2SurfaceMaterial* material)
{
    if(material==NULL || !b2IsValidMaterial(material) || !b2Shape_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    B2_REC( ( b2_worlds + id.world0 ), ShapeSetSurfaceMaterial, id, *material );
    b2GetShapeFromId(id)->material=*material;
}

b2SurfaceMaterial b2Shape_GetSurfaceMaterial(b2ShapeId id)
{
    b2Shape* shape=b2GetShapeFromId(id);return shape!=NULL?shape->material:b2SurfaceMaterial{};
}

void b2Shape_SetRestitution(b2ShapeId id,float restitution)
{
    if(!b2IsValidFloat(restitution) || restitution<0.0f || !b2Shape_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    B2_REC( ( b2_worlds + id.world0 ), ShapeSetRestitution, id, restitution );
    b2GetShapeFromId(id)->material.restitution=restitution;
}

float b2Shape_GetRestitution(b2ShapeId id)
{
    b2Shape* shape=b2GetShapeFromId(id);return shape!=NULL?shape->material.restitution:0.0f;
}

void b2Shape_SetUserMaterial(b2ShapeId id,uint64_t material)
{
    if(!b2Shape_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    B2_REC( ( b2_worlds + id.world0 ), ShapeSetUserMaterial, id, material );
    b2GetShapeFromId(id)->material.userMaterialId=material;
}

uint64_t b2Shape_GetUserMaterial(b2ShapeId id)
{
    b2Shape* shape=b2GetShapeFromId(id);return shape!=NULL?shape->material.userMaterialId:0;
}

b2AABB b2ComputeShapeAABB( const b2Shape* shape, b2WorldTransform xf )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
			return b2ComputeCapsuleBounds( &shape->capsule, xf );
		case b2_circleShape:
			return b2ComputeCircleBounds( &shape->circle, xf );
		case b2_polygonShape:
			return b2ComputePolygonBounds( &shape->polygon, xf );
		case b2_segmentShape:
			return b2ComputeSegmentBounds( &shape->segment, xf );
		case b2_chainSegmentShape:
			return b2ComputeSegmentBounds( &shape->chainSegment.segment, xf );
		default:
		{
			B2_ASSERT( false );
			b2AABB empty = { { 0.0f, 0.0f }, { 0.0f, 0.0f } };
			return empty;
		}
	}
}

b2Vec2 b2GetShapeCentroid( const b2Shape* shape )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
			return b2Lerp( shape->capsule.center1, shape->capsule.center2, 0.5f );
		case b2_circleShape:
			return shape->circle.center;
		case b2_polygonShape:
			return shape->polygon.centroid;
		case b2_segmentShape:
			return b2Lerp( shape->segment.point1, shape->segment.point2, 0.5f );
		case b2_chainSegmentShape:
			return b2Lerp( shape->chainSegment.segment.point1, shape->chainSegment.segment.point2,
						   0.5f );
		default:
			return b2Vec2_zero;
	}
}

b2ShapeProxy b2MakeShapeDistanceProxy( const b2Shape* shape )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
			return b2MakeProxy( &shape->capsule.center1, 2, shape->capsule.radius );
		case b2_circleShape:
			return b2MakeProxy( &shape->circle.center, 1, shape->circle.radius );
		case b2_polygonShape:
			return b2MakeProxy( shape->polygon.vertices, shape->polygon.count,
								shape->polygon.radius );
		case b2_segmentShape:
			return b2MakeProxy( &shape->segment.point1, 2, 0.0f );
		case b2_chainSegmentShape:
			return b2MakeProxy( &shape->chainSegment.segment.point1, 2, 0.0f );
		default:
		{
			B2_ASSERT( false );
			b2ShapeProxy empty = {};
			return empty;
		}
	}
}

void b2Shape_EnableContactEvents(b2ShapeId id,bool flag)
{
    if(!b2Shape_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    B2_REC( ( b2_worlds + id.world0 ), ShapeEnableContactEvents, id, flag );
    b2GetShapeFromId(id)->enableContactEvents=flag;
}

bool b2Shape_AreContactEventsEnabled(b2ShapeId id)
{
    b2Shape* shape=b2GetShapeFromId(id);return shape!=NULL && shape->enableContactEvents;
}

void b2Shape_EnableHitEvents(b2ShapeId id,bool flag)
{
    if(!b2Shape_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    B2_REC( ( b2_worlds + id.world0 ), ShapeEnableHitEvents, id, flag );
    b2GetShapeFromId(id)->enableHitEvents=flag;
}

bool b2Shape_AreHitEventsEnabled(b2ShapeId id)
{
    b2Shape* shape=b2GetShapeFromId(id);return shape!=NULL && shape->enableHitEvents;
}

bool b2Shape_IsSensor(b2ShapeId id)
{
    b2Shape* shape=b2GetShapeFromId(id);return shape!=NULL && shape->sensorIndex!=B2_NULL_INDEX;
}

void b2Shape_EnableSensorEvents(b2ShapeId id,bool flag)
{
    if(!b2Shape_IsValid(id) || b2_worlds[id.world0].locked) { return; }
    B2_REC( ( b2_worlds + id.world0 ), ShapeEnableSensorEvents, id, flag );
    b2GetShapeFromId(id)->enableSensorEvents=flag;
}

bool b2Shape_AreSensorEventsEnabled(b2ShapeId id)
{
    b2Shape* shape=b2GetShapeFromId(id);return shape!=NULL && shape->enableSensorEvents;
}

int b2Shape_GetSensorCapacity(b2ShapeId id)
{
    b2Shape* shape=b2GetShapeFromId(id);if(shape==NULL || shape->sensorIndex==B2_NULL_INDEX || b2_worlds[id.world0].locked) { return 0; }
    return b2_worlds[id.world0].sensors.data[shape->sensorIndex].overlaps2.count;
}

int b2Shape_GetSensorData(b2ShapeId id,b2ShapeId* output,int capacity)
{
    b2Shape* shape=b2GetShapeFromId(id);
    if(shape==NULL || shape->sensorIndex==B2_NULL_INDEX || b2_worlds[id.world0].locked || output==NULL || capacity<=0) { return 0; }
    const b2Sensor* sensor=b2_worlds[id.world0].sensors.data+shape->sensorIndex;
    int count=b2MinInt(sensor->overlaps2.count,capacity);
    for(int i=0;i<count;++i) { output[i]={sensor->overlaps2.data[i].shapeId+1,id.world0,sensor->overlaps2.data[i].generation}; }
    return count;
}

void b2Shape_EnablePreSolveEvents( b2ShapeId shapeId, bool flag )
{
	b2World* world = b2Shape_IsValid(shapeId)?b2_worlds+shapeId.world0:NULL;
	if ( world == NULL || world->locked )
	{
		return;
	}


	B2_REC( world, ShapeEnablePreSolveEvents, shapeId, flag );
	b2Shape* shape = b2GetShapeFromId(shapeId);
	if ( shape->enablePreSolveEvents == flag )
	{
		return;
	}

	shape->enablePreSolveEvents = flag;

	// 刷新接触上的缓存标志。
	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	int contactKey = body->headContactKey;
	while ( contactKey != B2_NULL_INDEX )
	{
		int contactId = contactKey >> 1;
		int edgeIndex = contactKey & 1;

		b2Contact* contact = b2Array_Get( world->contacts, contactId );

		if ( contact->shapeIdA == shape->id || contact->shapeIdB == shape->id )
		{
			b2Shape* shapeA = b2Array_Get( world->shapes, contact->shapeIdA );
			b2Shape* shapeB = b2Array_Get( world->shapes, contact->shapeIdB );
			b2Body* bodyA = b2Array_Get( world->bodies, shapeA->bodyId );
			b2Body* bodyB = b2Array_Get( world->bodies, shapeB->bodyId );

			bool recycle = ( bodyA->flags & b2_bodyEnableContactRecycling ) != 0 &&
						   ( bodyB->flags & b2_bodyEnableContactRecycling ) != 0 &&
						   shapeA->enablePreSolveEvents == false &&
						   shapeB->enablePreSolveEvents == false;

			if ( recycle )
			{
				contact->flags |= b2_contactRecycleFlag;
			}
			else
			{
				contact->flags &= ~b2_contactRecycleFlag;
			}

			b2ContactSim* contactSim = b2GetContactSim( world, contact );

			if ( recycle )
			{
				contactSim->simFlags |= b2_contactRecycleFlag;
			}
			else
			{
				contactSim->simFlags &= ~b2_contactRecycleFlag;
			}

			if ( shapeA->enablePreSolveEvents || shapeB->enablePreSolveEvents )
			{
				contactSim->simFlags |= b2_simEnablePreSolveEvents;
			}
			else
			{
				contactSim->simFlags &= ~b2_simEnablePreSolveEvents;
			}
		}

		contactKey = contact->edges[edgeIndex].nextKey;
	}
}

bool b2Shape_ArePreSolveEventsEnabled( b2ShapeId shapeId )
{
	b2World* world=b2_worlds+shapeId.world0;
	b2Shape* shape = b2GetShapeFromId(shapeId);
	return shape->enablePreSolveEvents;
}

b2CastOutput b2RayCastShape( const b2RayCastInput* input, const b2Shape* shape,
							 b2Transform transform )
{
	b2RayCastInput localInput = *input;
	localInput.origin = b2InvTransformPoint( transform, input->origin );
	localInput.translation = b2InvRotateVector( transform.q, input->translation );

	b2CastOutput output = {};
	switch ( shape->type )
	{
		case b2_capsuleShape:
			output = b2CastRayCapsule( &shape->capsule, &localInput );
			break;
		case b2_circleShape:
			output = b2CastRayCircle( &shape->circle, &localInput );
			break;
		case b2_polygonShape:
			output = b2CastRayPolygon( &shape->polygon, &localInput );
			break;
		case b2_segmentShape:
			output = b2CastRaySegment( &shape->segment, &localInput, false );
			break;
		case b2_chainSegmentShape:
			output = b2CastRaySegment( &shape->chainSegment.segment, &localInput, true );
			break;
		default:
			return output;
	}

	// 输出点留在输入变换的坐标系中，这是调用者选择的坐标系，通常重定心到原点附近。
	output.point = b2TransformPoint( transform, output.point );
	output.normal = b2RotateVector( transform.q, output.normal );
	return output;
}

b2CastOutput b2ShapeCastShape( const b2ShapeCastInput* input, const b2Shape* shape,
							   b2Transform transform )
{
	b2CastOutput output = {};

	if ( input->proxy.count == 0 )
	{
		return output;
	}

	b2ShapeCastInput localInput = *input;

	for ( int i = 0; i < localInput.proxy.count; ++i )
	{
		localInput.proxy.points[i] = b2InvTransformPoint( transform, input->proxy.points[i] );
	}

	localInput.translation = b2InvRotateVector( transform.q, input->translation );

	switch ( shape->type )
	{
		case b2_capsuleShape:
			output = b2CastShapeCapsule( &shape->capsule, &localInput );
			break;
		case b2_circleShape:
			output = b2CastShapeCircle( &shape->circle, &localInput );
			break;
		case b2_polygonShape:
			output = b2CastShapePolygon( &shape->polygon, &localInput );
			break;
		case b2_segmentShape:
			output = b2CastShapeSegment( &shape->segment, &localInput );
			break;
		case b2_chainSegmentShape:
		{
			// 检查背面碰撞
			b2Vec2 approximateCentroid = localInput.proxy.points[0];
			for ( int i = 1; i < localInput.proxy.count; ++i )
			{
				approximateCentroid = b2Add( approximateCentroid, localInput.proxy.points[i] );
			}

			approximateCentroid = b2MulSV( 1.0f / localInput.proxy.count, approximateCentroid );

			b2Vec2 edge =
				b2Sub( shape->chainSegment.segment.point2, shape->chainSegment.segment.point1 );
			b2Vec2 r = b2Sub( approximateCentroid, shape->chainSegment.segment.point1 );

			if ( b2Cross( r, edge ) < 0.0f )
			{
				// 形状投射从后方开始
				return output;
			}

			output = b2CastShapeSegment( &shape->chainSegment.segment, &localInput );
		}
		break;
		default:
			return output;
	}

	// 与 b2RayCastShape 相同的坐标系约定，点留在输入变换坐标系中
	output.point = b2TransformPoint( transform, output.point );
	output.normal = b2RotateVector( transform.q, output.normal );
	return output;
}

bool b2Shape_TestPoint( b2ShapeId shapeId, b2Pos point )
{
	b2World* world = b2_worlds + shapeId.world0;
	b2Shape* shape = b2GetShapeFromId(shapeId);

	b2WorldTransform transform = b2GetBodySim(world,b2Array_Get(world->bodies,shape->bodyId))->transform;
	b2Vec2 localPoint = b2InvTransformWorldPoint( transform, point );

	bool result;
	switch ( shape->type )
	{
		case b2_capsuleShape:
			result = b2PointInCapsule( &shape->capsule, localPoint );
			break;

		case b2_circleShape:
			result = b2PointInCircle( &shape->circle, localPoint );
			break;

		case b2_polygonShape:
			result = b2PointInPolygon( &shape->polygon, localPoint );
			break;

		default:
			result = false;
			break;
	}

	return result;
}

b2WorldCastOutput b2Shape_CastRay( b2ShapeId shapeId, b2Pos origin, b2Vec2 translation )
{
	B2_ASSERT( b2IsValidVec2( origin ) );
	B2_ASSERT( b2IsValidVec2( translation ) );

	b2World* world = b2_worlds + shapeId.world0;
	b2Shape* shape = b2GetShapeFromId(shapeId);

	// 以原点重定心，使投射在 float 精度下运行
	b2Transform transform =
		b2ToRelativeTransform( b2GetBodySim(world,b2Array_Get(world->bodies,shape->bodyId))->transform, origin );

	// 射线从原点出发，因此在重定心坐标系中原点为零
	b2RayCastInput input = { b2Vec2_zero, translation, 1.0f };

	// 将重定心的 float 结果提升回世界位置
	b2CastOutput local = b2RayCastShape( &input, shape, transform );
	b2WorldCastOutput output;
	output.normal = local.normal;
	output.point = b2OffsetPos( origin, local.point );
	output.fraction = local.fraction;
	output.iterations = local.iterations;
	output.hit = local.hit;

	return output;
}

b2Pos b2Shape_GetClosestPoint( b2ShapeId shapeId, b2Pos target )
{
	b2World* world = b2_worlds + shapeId.world0;
	if ( world == NULL )
	{
		return b2Pos_zero;
	}

	b2Shape* shape = b2GetShapeFromId(shapeId);
	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );
	b2WorldTransform transform = b2GetBodySim(world,body)->transform;

	// 目标作为代理 B 的坐标系传入，因此相对位姿用 double 做差，远离原点时结果仍精确
	b2Vec2 zero = b2Vec2_zero;
	b2WorldTransform targetTransform = { target, b2Rot_identity };

	b2DistanceInput input;
	input.proxyA = b2MakeShapeDistanceProxy( shape );
	input.proxyB = b2MakeProxy( &zero, 1, 0.0f );
	input.transform = b2InvMulWorldTransforms( transform, targetTransform );
	input.useRadii = true;

	b2SimplexCache cache = {};
	b2DistanceOutput output = b2ComputeShapeDistance( &input, &cache, NULL, 0 );

	return b2TransformWorldPoint( transform, output.pointA );
}

// 将形状周长投影到无限直线上
float b2GetShapeProjectedPerimeter( const b2Shape* shape, b2Vec2 line )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
		{
			b2Vec2 axis = b2Sub( shape->capsule.center2, shape->capsule.center1 );
			float projectedLength = b2AbsFloat( b2Dot( axis, line ) );
			return projectedLength + 2.0f * shape->capsule.radius;
		}

		case b2_circleShape:
			return 2.0f * shape->circle.radius;

		case b2_polygonShape:
		{
			const b2Vec2* points = shape->polygon.vertices;
			int count = shape->polygon.count;
			B2_ASSERT( count > 0 );
			float value = b2Dot( points[0], line );
			float lower = value;
			float upper = value;
			for ( int i = 1; i < count; ++i )
			{
				value = b2Dot( points[i], line );
				lower = b2MinFloat( lower, value );
				upper = b2MaxFloat( upper, value );
			}

			return ( upper - lower ) + 2.0f * shape->polygon.radius;
		}

		case b2_segmentShape:
		{
			float value1 = b2Dot( shape->segment.point1, line );
			float value2 = b2Dot( shape->segment.point2, line );
			return b2AbsFloat( value2 - value1 );
		}

		case b2_chainSegmentShape:
		{
			float value1 = b2Dot( shape->chainSegment.segment.point1, line );
			float value2 = b2Dot( shape->chainSegment.segment.point2, line );
			return b2AbsFloat( value2 - value1 );
		}

		default:
			return 0.0f;
	}
}

// https://en.wikipedia.org/wiki/Density_of_air
// https://www.engineeringtoolbox.com/wind-load-d_1775.html
// 力 = 0.5 * 空气密度 * 速度^2 * 面积
// https://en.wikipedia.org/wiki/Lift_(force)
void b2Shape_ApplyWind( b2ShapeId shapeId, b2Vec2 wind, float drag, float lift, bool wake )
{
	if (!(b2IsValidVec2( wind ))) { return; }
	if (!(b2IsValidFloat( drag ))) { return; }
	if (!(b2IsValidFloat( lift ))) { return; }

	b2World* world = b2_worlds + shapeId.world0;
	if ( world == NULL )
	{
		return;
	}



	B2_REC( world, ShapeApplyWind, shapeId, wind, drag, lift, wake );
	b2Shape* shape = b2GetShapeFromId(shapeId);

	b2ShapeType shapeType = shape->type;
	if ( shapeType != b2_circleShape && shapeType != b2_capsuleShape &&
		 shapeType != b2_polygonShape )
	{
		return;
	}

	b2Body* body = b2Array_Get( world->bodies, shape->bodyId );

	if ( body->type != b2_dynamicBody )
	{
		return;
	}

	if ( body->setIndex >= b2_firstSleepingSet && wake == false )
	{
		return;
	}

	b2BodySim* sim = b2GetBodySim( world, body );

	if ( body->setIndex != b2_awakeSet )
	{
		// 必须唤醒才有状态
		b2WakeBody( world, body );
	}

	B2_ASSERT( body->setIndex == b2_awakeSet );

	sim = b2GetBodySim(world, body);
	b2BodyState* state = b2GetBodyState( world, body );
	b2WorldTransform transform = sim->transform;

	float lengthUnits = b2GetLengthUnitsPerMeter();
	float volumeUnits = lengthUnits * lengthUnits * lengthUnits;

	// 在 2D 中假设单位深度
	float airDensity = 1.2250f / ( volumeUnits );

	b2Vec2 force = {};
	float torque = 0.0f;

	switch ( shape->type )
	{
		case b2_circleShape:
		{
			float radius = shape->circle.radius;
			b2Vec2 centroid = b2GetShapeCentroid(shape);
			b2Vec2 lever = b2RotateVector( transform.q, b2Sub( centroid, sim->localCenter ) );
			b2Vec2 shapeVelocity =
				b2Add( state->linearVelocity, b2CrossSV( state->angularVelocity, lever ) );
			b2Vec2 relativeVelocity = b2MulSub( wind, drag, shapeVelocity );
			float speed;
			b2Vec2 direction = b2GetLengthAndNormalize( &speed, relativeVelocity );
			float projectedArea = 2.0f * radius;
			force = b2MulSV( 0.5f * airDensity * projectedArea * speed * speed, direction );
			torque = b2Cross( lever, force );
		}
		break;

		case b2_capsuleShape:
		{
			b2Vec2 centroid = b2GetShapeCentroid(shape);
			b2Vec2 lever = b2RotateVector( transform.q, b2Sub( centroid, sim->localCenter ) );
			b2Vec2 shapeVelocity =
				b2Add( state->linearVelocity, b2CrossSV( state->angularVelocity, lever ) );
			b2Vec2 relativeVelocity = b2MulSub( wind, drag, shapeVelocity );
			float speed;
			b2Vec2 direction = b2GetLengthAndNormalize( &speed, relativeVelocity );

			b2Vec2 d = b2Sub( shape->capsule.center2, shape->capsule.center1 );
			d = b2RotateVector( transform.q, d );

			float radius = shape->capsule.radius;
			float projectedArea = 2.0f * radius + b2AbsFloat( b2Cross( d, direction ) );

			// 与风相反的法线
			b2Vec2 normal = b2LeftPerp( b2Normalize( d ) );
			normal = b2Dot( normal, direction ) > 0.0f ? b2Neg( normal ) : normal;

			// 垂直于表面的风分量
			b2Vec2 liftDirection = b2CrossSV( b2Cross( normal, direction ), direction );

			float forceMagnitude = 0.5f * airDensity * projectedArea * speed * speed;
			force = b2MulSV( forceMagnitude, b2MulAdd( direction, lift, liftDirection ) );

			b2Vec2 edgeLever = b2MulAdd( lever, radius, normal );
			torque = b2Cross( edgeLever, force );
		}
		break;

		case b2_polygonShape:
		{
			b2Vec2 centroid = b2GetShapeCentroid(shape);
			b2Vec2 lever = b2RotateVector( transform.q, b2Sub( centroid, sim->localCenter ) );
			b2Vec2 shapeVelocity =
				b2Add( state->linearVelocity, b2CrossSV( state->angularVelocity, lever ) );
			b2Vec2 relativeVelocity = b2MulSub( wind, drag, shapeVelocity );
			float speed;
			b2Vec2 direction = b2GetLengthAndNormalize( &speed, relativeVelocity );

			// 为简化忽略多边形半径
			int count = shape->polygon.count;
			b2Vec2* vertices = shape->polygon.vertices;

			b2Vec2 v1 = vertices[count - 1];
			for ( int i = 0; i < count; ++i )
			{
				b2Vec2 v2 = vertices[i];
				b2Vec2 d = b2Sub( v2, v1 );
				b2Vec2 edgeCenter = b2Lerp( v1, v2, 0.5f );
				v1 = v2;

				d = b2RotateVector( transform.q, d );

				float projectedArea = b2Cross( d, direction );
				if ( projectedArea <= 0.0f )
				{
					// 背向
					continue;
				}

				b2Vec2 normal = b2RightPerp( b2Normalize( d ) );

				// 垂直于表面的风分量
				b2Vec2 liftDirection = b2CrossSV( b2Cross( normal, direction ), direction );

				float forceMagnitude = 0.5f * airDensity * projectedArea * speed * speed;
				b2Vec2 f = b2MulSV( forceMagnitude, b2MulAdd( direction, lift, liftDirection ) );

				b2Vec2 edgeLever =
					b2RotateVector( transform.q, b2Sub( edgeCenter, sim->localCenter ) );

				force = b2Add( force, f );
				torque += b2Cross( edgeLever, f );
			}
		}
		break;

		default:
			break;
	}

	sim->force = b2Add( sim->force, force );
	sim->torque += torque;
}

b2PlaneResult b2CollideMover( const b2Capsule* mover, const b2Shape* shape, b2Transform transform )
{
	b2Capsule localMover;
	localMover.center1 = b2InvTransformPoint( transform, mover->center1 );
	localMover.center2 = b2InvTransformPoint( transform, mover->center2 );
	localMover.radius = mover->radius;

	b2PlaneResult result = {};
	switch ( shape->type )
	{
		case b2_capsuleShape:
			result = b2CollideMoverAndCapsule( &localMover, &shape->capsule );
			break;
		case b2_circleShape:
			result = b2CollideMoverAndCircle( &localMover, &shape->circle );
			break;
		case b2_polygonShape:
			result = b2CollideMoverAndPolygon( &localMover, &shape->polygon );
			break;
		case b2_segmentShape:
			result = b2CollideMoverAndSegment( &localMover, &shape->segment );
			break;
		case b2_chainSegmentShape:
		{
			b2Vec2 centroid = b2Lerp( localMover.center1, localMover.center2, 0.5f );
			b2Vec2 edge =
				b2Sub( shape->chainSegment.segment.point2, shape->chainSegment.segment.point1 );
			b2Vec2 r = b2Sub( centroid, shape->chainSegment.segment.point1 );

			// 跳过背面碰撞。
			if ( b2Cross( r, edge ) < 0.0f )
			{
				return result;
			}

			result = b2CollideMoverAndSegment( &localMover, &shape->chainSegment.segment );
		}
		break;
		default:
			return result;
	}

	if ( result.hit == false )
	{
		return result;
	}

	result.plane.normal = b2RotateVector( transform.q, result.plane.normal );
	return result;
}

b2ShapeId b2CreateChainSegmentShape( b2BodyId bodyId, const b2ShapeDef* def,
									 const b2ChainSegment* chainSegment )
{
	if ( !b2IsValidChainSegment( chainSegment ) ) { return b2_nullShapeId; }

	float lengthSqr =
		b2DistanceSquared( chainSegment->segment.point1, chainSegment->segment.point2 );
	if ( lengthSqr <= B2_LINEAR_SLOP * B2_LINEAR_SLOP )
	{
		return b2_nullShapeId;
	}

	// 没有父链条形状
	b2ChainSegment local = *chainSegment;
	local.chainId = B2_NULL_INDEX;
	b2FixGhostVertices( &local );

	b2ShapeId id = b2CreateShape( bodyId, def, &local, b2_chainSegmentShape );

	if ( id.index1 != 0 )
	{
		B2_REC_CREATE( ( b2_worlds + bodyId.world0 ), CreateChainSegmentShape, id, bodyId, *def, local );
	}

	return id;
}

void b2Shape_SetChainSegment( b2ShapeId shapeId, const b2ChainSegment* chainSegment )
{
    if ( !b2IsValidChainSegment( chainSegment ) || !b2Shape_IsValid( shapeId ) ) { return; }
    b2World* world = b2_worlds + shapeId.world0;
    if ( world->locked ) { return; }
    b2Shape* shape = b2GetShapeFromId( shapeId );
    if ( shape->type == b2_chainSegmentShape && shape->chainSegment.chainId != B2_NULL_INDEX ) { return; }
    float lengthSqr = b2DistanceSquared( chainSegment->segment.point1, chainSegment->segment.point2 );
    if ( lengthSqr <= B2_LINEAR_SLOP * B2_LINEAR_SLOP ) { return; }

    B2_REC( world, ShapeSetChainSegment, shapeId, *chainSegment );
    shape->chainSegment = *chainSegment;
    shape->chainSegment.chainId = B2_NULL_INDEX;
    b2FixGhostVertices( &shape->chainSegment );
    shape->type = b2_chainSegmentShape;
    shape->aabbMargin = b2ComputeShapeMargin( shape );

    b2DestroyShapeContacts( world, shape );
    b2Body* body = world->bodies.data + shape->bodyId;
    b2WorldTransform transform = b2GetBodySim( world, body )->transform;
    if ( shape->proxyKey != B2_NULL_INDEX )
    {
        b2DestroyShapeProxy( shape, &world->broadPhase );
        b2CreateShapeProxy( world, shape, body->type, transform, true );
    }
    else
    {
        b2UpdateShapeAABB( shape, transform );
        float margin = body->type == b2_staticBody ? B2_SPECULATIVE_DISTANCE : shape->aabbMargin;
        b2Vec2 extra = { margin, margin };
        world->fatAABBs.data[shape->id] = { b2Sub( shape->aabb.lowerBound, extra ), b2Add( shape->aabb.upperBound, extra ) };
    }
}

void b2FreeChainData( b2ChainShape* chain )
{
	b2Free( chain->shapeIndices, chain->segmentCount * sizeof( int ) );
	chain->shapeIndices = NULL;
}

b2AABB b2Shape_GetBounds( b2ShapeId shapeId )
{
    b2Shape* shape = b2GetShapeFromId( shapeId );
    return shape != NULL ? shape->aabb : b2AABB{};
}

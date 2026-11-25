// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "shape.h"
#include "body.h"
#include "core.h"
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
        body->dirtyMass = true;
        if ( def->updateBodyMass )
        {
            b2UpdateBodyMassData( world, body );
        }
    }
    b2UpdateShapeAABB( shape, body->transform );
    if ( body->isEnabled )
    {
        b2CreateShapeProxy( world, shape, body->type, body->transform, def->invokeContactCreation );
    }
    return b2ShapeId{ id + 1, world->worldId, generation };
}

static void b2DestroyShapeInternal( b2World* world, b2Shape* shape, b2Body* body )
{
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
    b2Body* body = world->bodies.data + shape->bodyId;
    b2DestroyShapeInternal( world, shape, body );
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
    b2World* world = b2_worlds + id.world0;
    b2Body* body = world->bodies.data + shape->bodyId;
    shape->aabbMargin = b2ComputeShapeMargin( shape );
    b2DestroyShapeProxy( shape, &world->broadPhase );
    if ( body->isEnabled )
    {
        b2CreateShapeProxy( world, shape, body->type, body->transform, true );
    }
    else
    {
        b2UpdateShapeAABB( shape, body->transform );
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
    return b2ChainId{ id+1, world->worldId, generation };
}

void b2DestroyChain( b2ChainId id )
{
    if ( b2Chain_IsValid( id ) == false || b2_worlds[id.world0].locked )
    {
        return;
    }
    b2World* world = b2_worlds + id.world0;
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
    b2Free( chain->shapeIndices, chain->segmentCount * sizeof( int ) );
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
    b2_worlds[id.world0].shapes.data[chain->shapeIndices[segmentIndex]].material = *material;
}

void b2Chain_SetAllSurfaceMaterials( b2ChainId id, const b2SurfaceMaterial* material )
{
    b2ChainShape* chain = b2GetChainFromId( id );
    if ( chain == NULL || material == NULL || b2IsValidMaterial( material ) == false || b2_worlds[id.world0].locked )
    {
        return;
    }
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
    b2Body* body = world->bodies.data+shape->bodyId;
    shape->filter = filter;
    b2DestroyShapeProxy( shape, &world->broadPhase );
    if ( body->isEnabled )
    {
        b2CreateShapeProxy( world, shape, body->type, body->transform, true );
    }
}

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

struct b2Shape
{
    int id;
    int bodyId;
    int prevShapeId;
    int nextShapeId;
    b2ShapeType type;
    b2SurfaceMaterial material;
    float density;
    float aabbMargin;
    int proxyKey;
    int sensorIndex;
    b2AABB aabb;
    b2Filter filter;
    void* userData;
    bool enableContactEvents;
    bool enableSensorEvents;
    bool enableCustomFiltering;
    bool enableHitEvents;
    bool enablePreSolveEvents;
    union
    {
        b2Circle circle;
        b2Capsule capsule;
        b2Polygon polygon;
        b2Segment segment;
        b2ChainSegment chainSegment;
    };
    uint16_t generation;
};

void b2UpdateShapeAABB( b2Shape* shape, b2WorldTransform transform );


struct b2ShapeExtent
{
    float minExtent;
    float maxExtent;
};
b2MassData b2ComputeShapeMass( const b2Shape* shape );
b2ShapeExtent b2ComputeShapeExtent( const b2Shape* shape, b2Vec2 localCenter );


struct b2ChainShape
{
    int id;
    int bodyId;
    int nextChainId;
    int segmentCount;
    uint16_t generation;
    int* shapeIndices;
};


struct b2World;
struct b2BroadPhase;
void b2CreateShapeProxy( b2World* world, b2Shape* shape, b2BodyType type, b2WorldTransform transform, bool forcePairCreation );
void b2DestroyShapeProxy( b2Shape* shape, b2BroadPhase* broadPhase );

void b2MoveShapeProxy( b2World* world, b2Shape* shape, b2WorldTransform transform );


inline bool b2ShouldShapesCollide( b2Filter a, b2Filter b )
{
    if ( a.groupIndex == b.groupIndex && a.groupIndex != 0 )
    {
        return a.groupIndex > 0;
    }
    return (a.maskBits & b.categoryBits) != 0 && (a.categoryBits & b.maskBits) != 0;
}

static inline float b2GetShapeRadius( const b2Shape* shape )
{
	switch ( shape->type )
	{
		case b2_capsuleShape:
			return shape->capsule.radius;
		case b2_circleShape:
			return shape->circle.radius;
		case b2_polygonShape:
			return shape->polygon.radius;
		default:
			return 0.0f;
	}
}

b2AABB b2ComputeShapeAABB(const b2Shape* shape,b2WorldTransform xf);
b2Vec2 b2GetShapeCentroid(const b2Shape* shape);
b2ShapeProxy b2MakeShapeDistanceProxy(const b2Shape* shape);

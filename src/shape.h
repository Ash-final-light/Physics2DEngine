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
    b2AABB aabb;
    b2Filter filter;
    void* userData;
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

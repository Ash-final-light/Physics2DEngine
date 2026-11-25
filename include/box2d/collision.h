// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "math_types.h"

/// 凸多边形最多使用的顶点数。
#define B2_MAX_POLYGON_VERTICES 8

/// 射线检测输出。未命中时 hit 为 false，其余字段为零。
typedef struct b2CastOutput
{
	b2Vec2 normal;
	b2Vec2 point;
	float fraction;
	int iterations;
	bool hit;
} b2CastOutput;

/// 形状的质量数据。
typedef struct b2MassData
{
	float mass;
	b2Vec2 center;
	float rotationalInertia;
} b2MassData;

/// 实心圆。
typedef struct b2Circle
{
	b2Vec2 center;
	float radius;
} b2Circle;

/// 具有双侧碰撞的线段。
typedef struct b2Segment
{
	b2Vec2 point1;
	b2Vec2 point2;
} b2Segment;

/// 实心胶囊，由两个半圆和中间矩形组成。
typedef struct b2Capsule
{
	b2Vec2 center1;
	b2Vec2 center2;
	float radius;
} b2Capsule;

/// 凸包。点按逆时针排列。
typedef struct b2Hull
{
	b2Vec2 points[B2_MAX_POLYGON_VERTICES];
	int count;
} b2Hull;

/// 计算点集的凸包。点数不足、超过 8 个、全部过近或共线时返回空凸包。
B2_API b2Hull b2ComputeHull( const b2Vec2* points, int count );

/// 检查凸包是否严格凸、逆时针，且没有过近的共线点。
B2_API bool b2ValidateHull( const b2Hull* hull );

/// 实心凸多边形。顶点按逆时针排列，法线指向外侧。
typedef struct b2Polygon
{
	b2Vec2 vertices[B2_MAX_POLYGON_VERTICES];
	b2Vec2 normals[B2_MAX_POLYGON_VERTICES];
	b2Vec2 centroid;
	float radius;
	int count;
} b2Polygon;

B2_API b2Polygon b2MakePolygon( const b2Hull* hull, float radius );
B2_API b2Polygon b2MakeOffsetPolygon( const b2Hull* hull, b2Vec2 position, b2Rot rotation );
B2_API b2Polygon b2MakeOffsetRoundedPolygon( const b2Hull* hull, b2Vec2 position, b2Rot rotation, float radius );
B2_API b2Polygon b2MakeBox( float halfWidth, float halfHeight );
B2_API b2Polygon b2MakeSquare( float halfWidth );
B2_API b2Polygon b2MakeRoundedBox( float halfWidth, float halfHeight, float radius );
B2_API b2Polygon b2MakeOffsetBox( float halfWidth, float halfHeight, b2Vec2 center, b2Rot rotation );
B2_API b2Polygon b2MakeOffsetRoundedBox( float halfWidth, float halfHeight, b2Vec2 center, b2Rot rotation, float radius );
B2_API b2MassData b2ComputeCircleMass( const b2Circle* shape, float density );

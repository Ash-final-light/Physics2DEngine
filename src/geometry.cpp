// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "geometry.h"

#include "box2d/math_functions.h"

bool b2IsValidCircle( const b2Circle* circle )
{
	return b2IsValidVec2( circle->center ) && b2IsValidFloat( circle->radius ) && circle->radius >= 0.0f;
}

bool b2IsValidSegment( const b2Segment* segment )
{
	return b2IsValidVec2( segment->point1 ) && b2IsValidVec2( segment->point2 );
}

bool b2IsValidCapsule( const b2Capsule* capsule )
{
	return b2IsValidVec2( capsule->center1 ) && b2IsValidVec2( capsule->center2 ) && b2IsValidFloat( capsule->radius ) &&
		   capsule->radius >= 0.0f;
}

#include <float.h>

static_assert( B2_MAX_POLYGON_VERTICES > 2, "must be 3 or more" );

static b2Vec2 b2ComputePolygonCentroid( const b2Vec2* vertices, int count )
{
	b2Vec2 center = { 0.0f, 0.0f };
	float area = 0.0f;
	b2Vec2 origin = vertices[0];
	const float inv3 = 1.0f / 3.0f;

	for ( int i = 1; i < count - 1; ++i )
	{
		b2Vec2 e1 = b2Sub( vertices[i], origin );
		b2Vec2 e2 = b2Sub( vertices[i + 1], origin );
		float a = 0.5f * b2Cross( e1, e2 );
		center = b2MulAdd( center, a * inv3, b2Add( e1, e2 ) );
		area += a;
	}

	B2_ASSERT( area > FLT_EPSILON );
	float invArea = 1.0f / area;
	center.x *= invArea;
	center.y *= invArea;
	return b2Add( origin, center );
}

b2Polygon b2MakeBox( float halfWidth, float halfHeight )
{
	B2_ASSERT( b2IsValidFloat( halfWidth ) && halfWidth > 0.0f );
	B2_ASSERT( b2IsValidFloat( halfHeight ) && halfHeight > 0.0f );

	b2Polygon shape = {};
	shape.count = 4;
	shape.vertices[0] = { -halfWidth, -halfHeight };
	shape.vertices[1] = { halfWidth, -halfHeight };
	shape.vertices[2] = { halfWidth, halfHeight };
	shape.vertices[3] = { -halfWidth, halfHeight };
	shape.normals[0] = { 0.0f, -1.0f };
	shape.normals[1] = { 1.0f, 0.0f };
	shape.normals[2] = { 0.0f, 1.0f };
	shape.normals[3] = { -1.0f, 0.0f };
	shape.radius = 0.0f;
	shape.centroid = b2Vec2_zero;
	return shape;
}

b2Polygon b2MakeSquare( float halfWidth )
{
	return b2MakeBox( halfWidth, halfWidth );
}

b2Polygon b2MakePolygon( const b2Hull* hull, float radius )
{
	B2_ASSERT( b2ValidateHull( hull ) );

	if ( hull->count < 3 )
	{
		return b2MakeSquare( 0.5f );
	}

	b2Polygon shape = {};
	shape.count = hull->count;
	shape.radius = radius;
	for ( int i = 0; i < shape.count; ++i )
	{
		shape.vertices[i] = hull->points[i];
	}

	for ( int i = 0; i < shape.count; ++i )
	{
		int i2 = i + 1 < shape.count ? i + 1 : 0;
		b2Vec2 edge = b2Sub( shape.vertices[i2], shape.vertices[i] );
		B2_ASSERT( b2Dot( edge, edge ) > FLT_EPSILON * FLT_EPSILON );
		shape.normals[i] = b2Normalize( b2CrossVS( edge, 1.0f ) );
	}

	shape.centroid = b2ComputePolygonCentroid( shape.vertices, shape.count );
	return shape;
}

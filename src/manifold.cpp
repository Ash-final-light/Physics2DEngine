// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>

b2LocalManifold b2CollideCircles( const b2Circle* circleA, const b2Circle* circleB, b2Transform xf )
{
	b2LocalManifold manifold = {};

	b2Vec2 pointA = circleA->center;
	b2Vec2 pointB = b2TransformPoint( xf, circleB->center );

	float distance;
	b2Vec2 normal = b2GetLengthAndNormalize( &distance, b2Sub( pointB, pointA ) );

	float separation = distance - circleA->radius - circleB->radius;
	if ( separation > B2_SPECULATIVE_DISTANCE )
	{
		return manifold;
	}

	b2Vec2 cA = b2MulAdd( pointA, circleA->radius, normal );
	b2Vec2 cB = b2MulAdd( pointB, -circleB->radius, normal );
	manifold.normal = normal;
	manifold.points[0].point = b2Lerp( cA, cB, 0.5f );
	manifold.points[0].separation = separation;
	manifold.points[0].id = 0;
	manifold.pointCount = 1;
	return manifold;
}

b2LocalManifold b2CollideCapsuleAndCircle( const b2Capsule* capsuleA, const b2Circle* circleB, b2Transform xf )
{
	b2LocalManifold manifold = {};
	b2Vec2 pB = b2TransformPoint( xf, circleB->center );
	b2Vec2 p1 = capsuleA->center1;
	b2Vec2 p2 = capsuleA->center2;
	b2Vec2 e = b2Sub( p2, p1 );

	b2Vec2 pA;
	float s1 = b2Dot( b2Sub( pB, p1 ), e );
	float s2 = b2Dot( b2Sub( p2, pB ), e );
	if ( s1 < 0.0f )
	{
		pA = p1;
	}
	else if ( s2 < 0.0f )
	{
		pA = p2;
	}
	else
	{
		float s = s1 / b2Dot( e, e );
		pA = b2MulAdd( p1, s, e );
	}

	float distance;
	b2Vec2 normal = b2GetLengthAndNormalize( &distance, b2Sub( pB, pA ) );
	float separation = distance - capsuleA->radius - circleB->radius;
	if ( separation > B2_SPECULATIVE_DISTANCE )
	{
		return manifold;
	}

	b2Vec2 cA = b2MulAdd( pA, capsuleA->radius, normal );
	b2Vec2 cB = b2MulAdd( pB, -circleB->radius, normal );
	manifold.normal = normal;
	manifold.points[0].point = b2Lerp( cA, cB, 0.5f );
	manifold.points[0].separation = separation;
	manifold.points[0].id = 0;
	manifold.pointCount = 1;
	return manifold;
}

b2LocalManifold b2CollidePolygonAndCircle( const b2Polygon* polygonA, const b2Circle* circleB, b2Transform xf )
{
	b2LocalManifold manifold = {};
	const float speculativeDistance = B2_SPECULATIVE_DISTANCE;
	b2Vec2 center = b2TransformPoint( xf, circleB->center );
	float radiusA = polygonA->radius;
	float radiusB = circleB->radius;
	float radius = radiusA + radiusB;

	int normalIndex = 0;
	float separation = -FLT_MAX;
	int vertexCount = polygonA->count;
	const b2Vec2* vertices = polygonA->vertices;
	const b2Vec2* normals = polygonA->normals;

	for ( int i = 0; i < vertexCount; ++i )
	{
		float s = b2Dot( normals[i], b2Sub( center, vertices[i] ) );
		if ( s > separation )
		{
			separation = s;
			normalIndex = i;
		}
	}

	if ( separation > radius + speculativeDistance )
	{
		return manifold;
	}

	int vertIndex1 = normalIndex;
	int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
	b2Vec2 v1 = vertices[vertIndex1];
	b2Vec2 v2 = vertices[vertIndex2];
	float u1 = b2Dot( b2Sub( center, v1 ), b2Sub( v2, v1 ) );
	float u2 = b2Dot( b2Sub( center, v2 ), b2Sub( v1, v2 ) );

	if ( u1 < 0.0f && separation > FLT_EPSILON )
	{
		b2Vec2 normal = b2Normalize( b2Sub( center, v1 ) );
		separation = b2Dot( b2Sub( center, v1 ), normal );
		if ( separation > radius + speculativeDistance )
		{
			return manifold;
		}

		b2Vec2 cA = b2MulAdd( v1, radiusA, normal );
		b2Vec2 cB = b2MulSub( center, radiusB, normal );
		manifold.normal = normal;
		manifold.points[0].point = b2Lerp( cA, cB, 0.5f );
		manifold.points[0].separation = b2Dot( b2Sub( cB, cA ), normal );
		manifold.points[0].id = 0;
		manifold.pointCount = 1;
	}
	else if ( u2 < 0.0f && separation > FLT_EPSILON )
	{
		b2Vec2 normal = b2Normalize( b2Sub( center, v2 ) );
		separation = b2Dot( b2Sub( center, v2 ), normal );
		if ( separation > radius + speculativeDistance )
		{
			return manifold;
		}

		b2Vec2 cA = b2MulAdd( v2, radiusA, normal );
		b2Vec2 cB = b2MulSub( center, radiusB, normal );
		manifold.normal = normal;
		manifold.points[0].point = b2Lerp( cA, cB, 0.5f );
		manifold.points[0].separation = b2Dot( b2Sub( cB, cA ), normal );
		manifold.points[0].id = 0;
		manifold.pointCount = 1;
	}
	else
	{
		b2Vec2 normal = normals[normalIndex];
		manifold.normal = normal;
		b2Vec2 cA = b2MulAdd( center, radiusA - b2Dot( b2Sub( center, v1 ), normal ), normal );
		b2Vec2 cB = b2MulSub( center, radiusB, normal );
		manifold.points[0].point = b2Lerp( cA, cB, 0.5f );
		manifold.points[0].separation = separation - radius;
		manifold.points[0].id = 0;
		manifold.pointCount = 1;
	}

	return manifold;
}

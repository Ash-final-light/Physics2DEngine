// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>

#define B2_MAKE_ID( A, B ) ( (uint8_t)( A ) << 8 | (uint8_t)( B ) )

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

b2LocalManifold b2CollideCapsules( const b2Capsule* capsuleA, const b2Capsule* capsuleB, b2Transform xf )
{
	b2Vec2 origin = capsuleA->center1;
	b2Transform xfs = { b2Sub( xf.p, origin ), xf.q };
	b2Vec2 p1 = b2Vec2_zero;
	b2Vec2 q1 = b2Sub( capsuleA->center2, origin );
	b2Vec2 p2 = b2TransformPoint( xfs, capsuleB->center1 );
	b2Vec2 q2 = b2TransformPoint( xfs, capsuleB->center2 );
	b2Vec2 d1 = b2Sub( q1, p1 );
	b2Vec2 d2 = b2Sub( q2, p2 );
	float dd1 = b2Dot( d1, d1 );
	float dd2 = b2Dot( d2, d2 );
	const float epsSqr = FLT_EPSILON * FLT_EPSILON;
	B2_ASSERT( dd1 > epsSqr && dd2 > epsSqr );

	b2Vec2 r = b2Sub( p1, p2 );
	float rd1 = b2Dot( r, d1 );
	float rd2 = b2Dot( r, d2 );
	float d12 = b2Dot( d1, d2 );
	float denom = dd1 * dd2 - d12 * d12;
	float f1 = 0.0f;
	if ( denom != 0.0f )
	{
		f1 = b2ClampFloat( ( d12 * rd2 - rd1 * dd2 ) / denom, 0.0f, 1.0f );
	}

	float f2 = ( d12 * f1 + rd2 ) / dd2;
	if ( f2 < 0.0f )
	{
		f2 = 0.0f;
		f1 = b2ClampFloat( -rd1 / dd1, 0.0f, 1.0f );
	}
	else if ( f2 > 1.0f )
	{
		f2 = 1.0f;
		f1 = b2ClampFloat( ( d12 - rd1 ) / dd1, 0.0f, 1.0f );
	}

	b2Vec2 closest1 = b2MulAdd( p1, f1, d1 );
	b2Vec2 closest2 = b2MulAdd( p2, f2, d2 );
	float distanceSquared = b2DistanceSquared( closest1, closest2 );

	b2LocalManifold manifold = {};
	float radiusA = capsuleA->radius;
	float radiusB = capsuleB->radius;
	float radius = radiusA + radiusB;
	float maxDistance = radius + B2_SPECULATIVE_DISTANCE;
	if ( distanceSquared > maxDistance * maxDistance )
	{
		return manifold;
	}

	float distance = sqrtf( distanceSquared );
	float length1, length2;
	b2Vec2 u1 = b2GetLengthAndNormalize( &length1, d1 );
	b2Vec2 u2 = b2GetLengthAndNormalize( &length2, d2 );
	float fp2 = b2Dot( b2Sub( p2, p1 ), u1 );
	float fq2 = b2Dot( b2Sub( q2, p1 ), u1 );
	bool outsideA = ( fp2 <= 0.0f && fq2 <= 0.0f ) || ( fp2 >= length1 && fq2 >= length1 );
	float fp1 = b2Dot( b2Sub( p1, p2 ), u2 );
	float fq1 = b2Dot( b2Sub( q1, p2 ), u2 );
	bool outsideB = ( fp1 <= 0.0f && fq1 <= 0.0f ) || ( fp1 >= length2 && fq1 >= length2 );

	if ( outsideA == false && outsideB == false )
	{
		b2Vec2 normalA;
		float separationA;
		{
			normalA = b2LeftPerp( u1 );
			float ss1 = b2Dot( b2Sub( p2, p1 ), normalA );
			float ss2 = b2Dot( b2Sub( q2, p1 ), normalA );
			float s1p = ss1 < ss2 ? ss1 : ss2;
			float s1n = -ss1 < -ss2 ? -ss1 : -ss2;
			if ( s1p > s1n )
			{
				separationA = s1p;
			}
			else
			{
				separationA = s1n;
				normalA = b2Neg( normalA );
			}
		}

		b2Vec2 normalB;
		float separationB;
		{
			normalB = b2LeftPerp( u2 );
			float ss1 = b2Dot( b2Sub( p1, p2 ), normalB );
			float ss2 = b2Dot( b2Sub( q1, p2 ), normalB );
			float s1p = ss1 < ss2 ? ss1 : ss2;
			float s1n = -ss1 < -ss2 ? -ss1 : -ss2;
			if ( s1p > s1n )
			{
				separationB = s1p;
			}
			else
			{
				separationB = s1n;
				normalB = b2Neg( normalB );
			}
		}

		if ( separationA + 0.1f * B2_LINEAR_SLOP >= separationB )
		{
			manifold.normal = normalA;
			b2Vec2 cp = p2;
			b2Vec2 cq = q2;
			if ( fp2 < 0.0f && fq2 > 0.0f )
			{
				cp = b2Lerp( p2, q2, ( 0.0f - fp2 ) / ( fq2 - fp2 ) );
			}
			else if ( fq2 < 0.0f && fp2 > 0.0f )
			{
				cq = b2Lerp( q2, p2, ( 0.0f - fq2 ) / ( fp2 - fq2 ) );
			}

			if ( fp2 > length1 && fq2 < length1 )
			{
				cp = b2Lerp( p2, q2, ( fp2 - length1 ) / ( fp2 - fq2 ) );
			}
			else if ( fq2 > length1 && fp2 < length1 )
			{
				cq = b2Lerp( q2, p2, ( fq2 - length1 ) / ( fq2 - fp2 ) );
			}

			float sp = b2Dot( b2Sub( cp, p1 ), normalA );
			float sq = b2Dot( b2Sub( cq, p1 ), normalA );
			if ( sp <= distance + B2_LINEAR_SLOP || sq <= distance + B2_LINEAR_SLOP )
			{
				manifold.points[0].point = b2MulAdd( cp, 0.5f * ( radiusA - radiusB - sp ), normalA );
				manifold.points[0].separation = sp - radius;
				manifold.points[0].id = B2_MAKE_ID( 0, 0 );
				manifold.points[1].point = b2MulAdd( cq, 0.5f * ( radiusA - radiusB - sq ), normalA );
				manifold.points[1].separation = sq - radius;
				manifold.points[1].id = B2_MAKE_ID( 0, 1 );
				manifold.pointCount = 2;
			}
		}
		else
		{
			manifold.normal = b2Neg( normalB );
			b2Vec2 cp = p1;
			b2Vec2 cq = q1;
			if ( fp1 < 0.0f && fq1 > 0.0f )
			{
				cp = b2Lerp( p1, q1, ( 0.0f - fp1 ) / ( fq1 - fp1 ) );
			}
			else if ( fq1 < 0.0f && fp1 > 0.0f )
			{
				cq = b2Lerp( q1, p1, ( 0.0f - fq1 ) / ( fp1 - fq1 ) );
			}

			if ( fp1 > length2 && fq1 < length2 )
			{
				cp = b2Lerp( p1, q1, ( fp1 - length2 ) / ( fp1 - fq1 ) );
			}
			else if ( fq1 > length2 && fp1 < length2 )
			{
				cq = b2Lerp( q1, p1, ( fq1 - length2 ) / ( fq1 - fp1 ) );
			}

			float sp = b2Dot( b2Sub( cp, p2 ), normalB );
			float sq = b2Dot( b2Sub( cq, p2 ), normalB );
			if ( sp <= distance + B2_LINEAR_SLOP || sq <= distance + B2_LINEAR_SLOP )
			{
				manifold.points[0].point = b2MulAdd( cp, 0.5f * ( radiusB - radiusA - sp ), normalB );
				manifold.points[0].separation = sp - radius;
				manifold.points[0].id = B2_MAKE_ID( 0, 0 );
				manifold.points[1].point = b2MulAdd( cq, 0.5f * ( radiusB - radiusA - sq ), normalB );
				manifold.points[1].separation = sq - radius;
				manifold.points[1].id = B2_MAKE_ID( 1, 0 );
				manifold.pointCount = 2;
			}
		}
	}

	if ( manifold.pointCount == 0 )
	{
		b2Vec2 normal = b2Sub( closest2, closest1 );
		if ( b2Dot( normal, normal ) > epsSqr )
		{
			normal = b2Normalize( normal );
		}
		else
		{
			normal = b2LeftPerp( u1 );
		}

		b2Vec2 c1 = b2MulAdd( closest1, radiusA, normal );
		b2Vec2 c2 = b2MulAdd( closest2, -radiusB, normal );
		int i1 = f1 == 0.0f ? 0 : 1;
		int i2 = f2 == 0.0f ? 0 : 1;
		manifold.normal = normal;
		manifold.points[0].point = b2Lerp( c1, c2, 0.5f );
		manifold.points[0].separation = sqrtf( distanceSquared ) - radius;
		manifold.points[0].id = B2_MAKE_ID( i1, i2 );
		manifold.pointCount = 1;
	}

	for ( int i = 0; i < manifold.pointCount; ++i )
	{
		manifold.points[i].point = b2Add( manifold.points[i].point, origin );
	}

	return manifold;
}

b2LocalManifold b2CollideSegmentAndCircle( const b2Segment* segmentA, const b2Circle* circleB, b2Transform xf )
{
	b2Capsule capsuleA = { segmentA->point1, segmentA->point2, 0.0f };
	return b2CollideCapsuleAndCircle( &capsuleA, circleB, xf );
}

b2LocalManifold b2CollideSegmentAndCapsule( const b2Segment* segmentA, const b2Capsule* capsuleB, b2Transform xf )
{
	b2Capsule capsuleA = { segmentA->point1, segmentA->point2, 0.0f };
	return b2CollideCapsules( &capsuleA, capsuleB, xf );
}

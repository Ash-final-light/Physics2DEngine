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

static b2Polygon b2MakeCapsule( b2Vec2 p1, b2Vec2 p2, float radius )
{
	b2Polygon shape = {};
	shape.vertices[0] = p1;
	shape.vertices[1] = p2;
	shape.centroid = b2Lerp( p1, p2, 0.5f );
	b2Vec2 d = b2Sub( p2, p1 );
	B2_ASSERT( b2LengthSquared( d ) > FLT_EPSILON );
	b2Vec2 axis = b2Normalize( d );
	b2Vec2 normal = b2RightPerp( axis );
	shape.normals[0] = normal;
	shape.normals[1] = b2Neg( normal );
	shape.count = 2;
	shape.radius = radius;
	return shape;
}

static b2LocalManifold b2ClipPolygons( const b2Polygon* polyA, const b2Polygon* polyB, int edgeA, int edgeB, bool flip )
{
	b2LocalManifold manifold = {};
	const b2Polygon* poly1;
	const b2Polygon* poly2;
	int i11, i12, i21, i22;
	if ( flip )
	{
		poly1 = polyB;
		poly2 = polyA;
		i11 = edgeB;
		i12 = edgeB + 1 < polyB->count ? edgeB + 1 : 0;
		i21 = edgeA;
		i22 = edgeA + 1 < polyA->count ? edgeA + 1 : 0;
	}
	else
	{
		poly1 = polyA;
		poly2 = polyB;
		i11 = edgeA;
		i12 = edgeA + 1 < polyA->count ? edgeA + 1 : 0;
		i21 = edgeB;
		i22 = edgeB + 1 < polyB->count ? edgeB + 1 : 0;
	}

	b2Vec2 normal = poly1->normals[i11];
	b2Vec2 v11 = poly1->vertices[i11];
	b2Vec2 v12 = poly1->vertices[i12];
	b2Vec2 v21 = poly2->vertices[i21];
	b2Vec2 v22 = poly2->vertices[i22];
	b2Vec2 tangent = b2CrossSV( 1.0f, normal );
	float lower1 = 0.0f;
	float upper1 = b2Dot( b2Sub( v12, v11 ), tangent );
	float upper2 = b2Dot( b2Sub( v21, v11 ), tangent );
	float lower2 = b2Dot( b2Sub( v22, v11 ), tangent );
	if ( upper2 < lower1 || upper1 < lower2 )
	{
		return manifold;
	}

	b2Vec2 vLower;
	if ( lower2 < lower1 && upper2 - lower2 > FLT_EPSILON )
	{
		vLower = b2Lerp( v22, v21, ( lower1 - lower2 ) / ( upper2 - lower2 ) );
	}
	else
	{
		vLower = v22;
	}

	b2Vec2 vUpper;
	if ( upper2 > upper1 && upper2 - lower2 > FLT_EPSILON )
	{
		vUpper = b2Lerp( v22, v21, ( upper1 - lower2 ) / ( upper2 - lower2 ) );
	}
	else
	{
		vUpper = v21;
	}

	float separationLower = b2Dot( b2Sub( vLower, v11 ), normal );
	float separationUpper = b2Dot( b2Sub( vUpper, v11 ), normal );
	float r1 = poly1->radius;
	float r2 = poly2->radius;
	vLower = b2MulAdd( vLower, 0.5f * ( r1 - r2 - separationLower ), normal );
	vUpper = b2MulAdd( vUpper, 0.5f * ( r1 - r2 - separationUpper ), normal );
	float radius = r1 + r2;
	if ( flip == false )
	{
		manifold.normal = normal;
		manifold.points[0].point = vLower;
		manifold.points[0].separation = separationLower - radius;
		manifold.points[0].id = B2_MAKE_ID( i11, i22 );
		manifold.points[1].point = vUpper;
		manifold.points[1].separation = separationUpper - radius;
		manifold.points[1].id = B2_MAKE_ID( i12, i21 );
		manifold.pointCount = 2;
	}
	else
	{
		manifold.normal = b2Neg( normal );
		manifold.points[0].point = vUpper;
		manifold.points[0].separation = separationUpper - radius;
		manifold.points[0].id = B2_MAKE_ID( i21, i12 );
		manifold.points[1].point = vLower;
		manifold.points[1].separation = separationLower - radius;
		manifold.points[1].id = B2_MAKE_ID( i22, i11 );
		manifold.pointCount = 2;
	}

	return manifold;
}

static float b2FindMaxSeparation( int* edgeIndex, const b2Polygon* poly1, const b2Polygon* poly2 )
{
	int count1 = poly1->count;
	int count2 = poly2->count;
	const b2Vec2* n1s = poly1->normals;
	const b2Vec2* v1s = poly1->vertices;
	const b2Vec2* v2s = poly2->vertices;
	int bestIndex = 0;
	float maxSeparation = -FLT_MAX;
	for ( int i = 0; i < count1; ++i )
	{
		b2Vec2 n = n1s[i];
		b2Vec2 v1 = v1s[i];
		float si = FLT_MAX;
		for ( int j = 0; j < count2; ++j )
		{
			float sij = b2Dot( n, b2Sub( v2s[j], v1 ) );
			if ( sij < si )
			{
				si = sij;
			}
		}

		if ( si > maxSeparation )
		{
			maxSeparation = si;
			bestIndex = i;
		}
	}

	*edgeIndex = bestIndex;
	return maxSeparation;
}

b2LocalManifold b2CollidePolygons( const b2Polygon* polygonA, const b2Polygon* polygonB, b2Transform xf )
{
	b2Vec2 origin = polygonA->vertices[0];
	float linearSlop = B2_LINEAR_SLOP;
	float speculativeDistance = B2_SPECULATIVE_DISTANCE;
	b2Transform xfs = { b2Sub( xf.p, origin ), xf.q };

	b2Polygon localPolyA = {};
	localPolyA.count = polygonA->count;
	localPolyA.radius = polygonA->radius;
	localPolyA.vertices[0] = b2Vec2_zero;
	localPolyA.normals[0] = polygonA->normals[0];
	for ( int i = 1; i < localPolyA.count; ++i )
	{
		localPolyA.vertices[i] = b2Sub( polygonA->vertices[i], origin );
		localPolyA.normals[i] = polygonA->normals[i];
	}

	b2Polygon localPolyB = {};
	localPolyB.count = polygonB->count;
	localPolyB.radius = polygonB->radius;
	for ( int i = 0; i < localPolyB.count; ++i )
	{
		localPolyB.vertices[i] = b2TransformPoint( xfs, polygonB->vertices[i] );
		localPolyB.normals[i] = b2RotateVector( xfs.q, polygonB->normals[i] );
	}

	float radius = localPolyA.radius + localPolyB.radius;
	int edgeA = 0;
	float separationA = b2FindMaxSeparation( &edgeA, &localPolyA, &localPolyB );
	if ( separationA > speculativeDistance + radius )
	{
		return {};
	}

	int edgeB = 0;
	float separationB = b2FindMaxSeparation( &edgeB, &localPolyB, &localPolyA );
	if ( separationB > speculativeDistance + radius )
	{
		return {};
	}

	bool flip;
	if ( separationA >= separationB )
	{
		flip = false;
		b2Vec2 searchDirection = localPolyA.normals[edgeA];
		int count = localPolyB.count;
		const b2Vec2* normals = localPolyB.normals;
		edgeB = 0;
		float minDot = FLT_MAX;
		for ( int i = 0; i < count; ++i )
		{
			float dot = b2Dot( searchDirection, normals[i] );
			if ( dot < minDot )
			{
				minDot = dot;
				edgeB = i;
			}
		}
	}
	else
	{
		flip = true;
		b2Vec2 searchDirection = localPolyB.normals[edgeB];
		int count = localPolyA.count;
		const b2Vec2* normals = localPolyA.normals;
		edgeA = 0;
		float minDot = FLT_MAX;
		for ( int i = 0; i < count; ++i )
		{
			float dot = b2Dot( searchDirection, normals[i] );
			if ( dot < minDot )
			{
				minDot = dot;
				edgeA = i;
			}
		}
	}

	b2LocalManifold manifold = {};
	if ( separationA > 0.1f * linearSlop || separationB > 0.1f * linearSlop )
	{
		int i11 = edgeA;
		int i12 = edgeA + 1 < localPolyA.count ? edgeA + 1 : 0;
		int i21 = edgeB;
		int i22 = edgeB + 1 < localPolyB.count ? edgeB + 1 : 0;
		b2Vec2 v11 = localPolyA.vertices[i11];
		b2Vec2 v12 = localPolyA.vertices[i12];
		b2Vec2 v21 = localPolyB.vertices[i21];
		b2Vec2 v22 = localPolyB.vertices[i22];
		b2SegmentDistanceResult result = b2ComputeSegmentDistance( v11, v12, v21, v22 );
		B2_ASSERT( result.distanceSquared > 0.0f );
		float distance = sqrtf( result.distanceSquared );
		float separation = distance - radius;
		if ( distance - radius > speculativeDistance )
		{
			return manifold;
		}

		manifold = b2ClipPolygons( &localPolyA, &localPolyB, edgeA, edgeB, flip );
		float minSeparation = FLT_MAX;
		for ( int i = 0; i < manifold.pointCount; ++i )
		{
			minSeparation = b2MinFloat( minSeparation, manifold.points[i].separation );
		}

		if ( separation + 0.1f * linearSlop < minSeparation )
		{
			if ( result.fraction1 == 0.0f && result.fraction2 == 0.0f )
			{
				b2Vec2 normal = b2Sub( v21, v11 );
				float invDistance = 1.0f / distance;
				normal.x *= invDistance;
				normal.y *= invDistance;
				b2Vec2 c1 = b2MulAdd( v11, localPolyA.radius, normal );
				b2Vec2 c2 = b2MulAdd( v21, -localPolyB.radius, normal );
				manifold.normal = normal;
				manifold.points[0].point = b2Lerp( c1, c2, 0.5f );
				manifold.points[0].separation = distance - radius;
				manifold.points[0].id = B2_MAKE_ID( i11, i21 );
				manifold.pointCount = 1;
			}
			else if ( result.fraction1 == 0.0f && result.fraction2 == 1.0f )
			{
				b2Vec2 normal = b2Sub( v22, v11 );
				float invDistance = 1.0f / distance;
				normal.x *= invDistance;
				normal.y *= invDistance;
				b2Vec2 c1 = b2MulAdd( v11, localPolyA.radius, normal );
				b2Vec2 c2 = b2MulAdd( v22, -localPolyB.radius, normal );
				manifold.normal = normal;
				manifold.points[0].point = b2Lerp( c1, c2, 0.5f );
				manifold.points[0].separation = distance - radius;
				manifold.points[0].id = B2_MAKE_ID( i11, i22 );
				manifold.pointCount = 1;
			}
			else if ( result.fraction1 == 1.0f && result.fraction2 == 0.0f )
			{
				b2Vec2 normal = b2Sub( v21, v12 );
				float invDistance = 1.0f / distance;
				normal.x *= invDistance;
				normal.y *= invDistance;
				b2Vec2 c1 = b2MulAdd( v12, localPolyA.radius, normal );
				b2Vec2 c2 = b2MulAdd( v21, -localPolyB.radius, normal );
				manifold.normal = normal;
				manifold.points[0].point = b2Lerp( c1, c2, 0.5f );
				manifold.points[0].separation = distance - radius;
				manifold.points[0].id = B2_MAKE_ID( i12, i21 );
				manifold.pointCount = 1;
			}
			else if ( result.fraction1 == 1.0f && result.fraction2 == 1.0f )
			{
				b2Vec2 normal = b2Sub( v22, v12 );
				float invDistance = 1.0f / distance;
				normal.x *= invDistance;
				normal.y *= invDistance;
				b2Vec2 c1 = b2MulAdd( v12, localPolyA.radius, normal );
				b2Vec2 c2 = b2MulAdd( v22, -localPolyB.radius, normal );
				manifold.normal = normal;
				manifold.points[0].point = b2Lerp( c1, c2, 0.5f );
				manifold.points[0].separation = distance - radius;
				manifold.points[0].id = B2_MAKE_ID( i12, i22 );
				manifold.pointCount = 1;
			}
		}
	}
	else
	{
		manifold = b2ClipPolygons( &localPolyA, &localPolyB, edgeA, edgeB, flip );
	}

	for ( int i = 0; i < manifold.pointCount; ++i )
	{
		manifold.points[i].point = b2Add( manifold.points[i].point, origin );
	}

	return manifold;
}

b2LocalManifold b2CollidePolygonAndCapsule( const b2Polygon* polygonA, const b2Capsule* capsuleB, b2Transform xf )
{
	b2Polygon polyB = b2MakeCapsule( capsuleB->center1, capsuleB->center2, capsuleB->radius );
	return b2CollidePolygons( polygonA, &polyB, xf );
}

b2LocalManifold b2CollideSegmentAndPolygon( const b2Segment* segmentA, const b2Polygon* polygonB, b2Transform xf )
{
	b2Polygon polygonA = b2MakeCapsule( segmentA->point1, segmentA->point2, 0.0f );
	return b2CollidePolygons( &polygonA, polygonB, xf );
}

b2LocalManifold b2CollideChainSegmentAndCircle( const b2ChainSegment* segmentA, const b2Circle* circleB, b2Transform xf )
{
	b2LocalManifold manifold = {};
	b2Vec2 pB = b2TransformPoint( xf, circleB->center );
	b2Vec2 p1 = segmentA->segment.point1;
	b2Vec2 p2 = segmentA->segment.point2;
	b2Vec2 e = b2Sub( p2, p1 );
	float offset = b2Dot( b2RightPerp( e ), b2Sub( pB, p1 ) );
	if ( offset < 0.0f )
	{
		return manifold;
	}

	float u = b2Dot( e, b2Sub( p2, pB ) );
	float v = b2Dot( e, b2Sub( pB, p1 ) );
	b2Vec2 pA;
	if ( v <= 0.0f )
	{
		b2Vec2 prevEdge = b2Sub( p1, segmentA->ghost1 );
		float uPrev = b2Dot( prevEdge, b2Sub( pB, p1 ) );
		if ( uPrev <= 0.0f )
		{
			return manifold;
		}

		pA = p1;
	}
	else if ( u <= 0.0f )
	{
		b2Vec2 nextEdge = b2Sub( segmentA->ghost2, p2 );
		float vNext = b2Dot( nextEdge, b2Sub( pB, p2 ) );
		if ( vNext > 0.0f )
		{
			return manifold;
		}

		pA = p2;
	}
	else
	{
		float ee = b2Dot( e, e );
		pA = b2Vec2{ u * p1.x + v * p2.x, u * p1.y + v * p2.y };
		pA = ee > 0.0f ? b2MulSV( 1.0f / ee, pA ) : p1;
	}

	float distance;
	b2Vec2 normal = b2GetLengthAndNormalize( &distance, b2Sub( pB, pA ) );
	float separation = distance - circleB->radius;
	if ( separation > B2_SPECULATIVE_DISTANCE )
	{
		return manifold;
	}

	b2Vec2 cB = b2MulAdd( pB, -circleB->radius, normal );
	manifold.normal = normal;
	manifold.points[0].point = b2Lerp( pA, cB, 0.5f );
	manifold.points[0].separation = separation;
	manifold.points[0].id = 0;
	manifold.pointCount = 1;
	return manifold;
}

static b2LocalManifold b2ClipSegments( b2Vec2 a1, b2Vec2 a2, b2Vec2 b1, b2Vec2 b2, b2Vec2 normal, float ra, float rb, uint16_t id1,
									   uint16_t id2 )
{
	b2LocalManifold manifold = {};
	b2Vec2 tangent = b2LeftPerp( normal );
	float lower1 = 0.0f;
	float upper1 = b2Dot( b2Sub( a2, a1 ), tangent );
	float upper2 = b2Dot( b2Sub( b1, a1 ), tangent );
	float lower2 = b2Dot( b2Sub( b2, a1 ), tangent );
	if ( upper2 < lower1 || upper1 < lower2 )
	{
		return manifold;
	}

	b2Vec2 vLower;
	if ( lower2 < lower1 && upper2 - lower2 > FLT_EPSILON )
	{
		vLower = b2Lerp( b2, b1, ( lower1 - lower2 ) / ( upper2 - lower2 ) );
	}
	else
	{
		vLower = b2;
	}

	b2Vec2 vUpper;
	if ( upper2 > upper1 && upper2 - lower2 > FLT_EPSILON )
	{
		vUpper = b2Lerp( b2, b1, ( upper1 - lower2 ) / ( upper2 - lower2 ) );
	}
	else
	{
		vUpper = b1;
	}

	float separationLower = b2Dot( b2Sub( vLower, a1 ), normal );
	float separationUpper = b2Dot( b2Sub( vUpper, a1 ), normal );
	vLower = b2MulAdd( vLower, 0.5f * ( ra - rb - separationLower ), normal );
	vUpper = b2MulAdd( vUpper, 0.5f * ( ra - rb - separationUpper ), normal );
	float radius = ra + rb;
	manifold.normal = normal;
	manifold.points[0].point = vLower;
	manifold.points[0].separation = separationLower - radius;
	manifold.points[0].id = id1;
	manifold.points[1].point = vUpper;
	manifold.points[1].separation = separationUpper - radius;
	manifold.points[1].id = id2;
	manifold.pointCount = 2;
	return manifold;
}

enum b2NormalType
{
	b2_normalSkip,
	b2_normalAdmit,
	b2_normalSnap
};

struct b2ChainSegmentParams
{
	b2Vec2 edge1;
	b2Vec2 normal0;
	b2Vec2 normal2;
	bool convex1;
	bool convex2;
};

static b2NormalType b2ClassifyNormal( b2ChainSegmentParams params, b2Vec2 normal )
{
	const float sinTol = 0.01f;
	if ( b2Dot( normal, params.edge1 ) <= 0.0f )
	{
		if ( params.convex1 )
		{
			if ( b2Cross( normal, params.normal0 ) > sinTol )
			{
				return b2_normalSkip;
			}

			return b2_normalAdmit;
		}

		return b2_normalSnap;
	}

	if ( params.convex2 )
	{
		if ( b2Cross( params.normal2, normal ) > sinTol )
		{
			return b2_normalSkip;
		}

		return b2_normalAdmit;
	}

	return b2_normalSnap;
}

static b2LocalManifold b2CollideChainSegmentAndPolygon( const b2ChainSegment* segmentA, const b2Polygon* polygonB, b2Transform xf,
													   b2SimplexCache* cache )
{
	b2LocalManifold manifold = {};
	b2Vec2 centroidB = b2TransformPoint( xf, polygonB->centroid );
	float radiusB = polygonB->radius;
	b2Vec2 p1 = segmentA->segment.point1;
	b2Vec2 p2 = segmentA->segment.point2;
	b2Vec2 edge1 = b2Normalize( b2Sub( p2, p1 ) );
	b2ChainSegmentParams smoothParams = {};
	smoothParams.edge1 = edge1;
	const float convexTol = 0.01f;
	b2Vec2 edge0 = b2Normalize( b2Sub( p1, segmentA->ghost1 ) );
	smoothParams.normal0 = b2RightPerp( edge0 );
	smoothParams.convex1 = b2Cross( edge0, edge1 ) >= convexTol;
	b2Vec2 edge2 = b2Normalize( b2Sub( segmentA->ghost2, p2 ) );
	smoothParams.normal2 = b2RightPerp( edge2 );
	smoothParams.convex2 = b2Cross( edge1, edge2 ) >= convexTol;
	b2Vec2 normal1 = b2RightPerp( edge1 );
	bool behind1 = b2Dot( normal1, b2Sub( centroidB, p1 ) ) < 0.0f;
	bool behind0 = true;
	bool behind2 = true;
	if ( smoothParams.convex1 )
	{
		behind0 = b2Dot( smoothParams.normal0, b2Sub( centroidB, p1 ) ) < 0.0f;
	}

	if ( smoothParams.convex2 )
	{
		behind2 = b2Dot( smoothParams.normal2, b2Sub( centroidB, p2 ) ) < 0.0f;
	}

	if ( behind1 && behind0 && behind2 )
	{
		return manifold;
	}

	int count = polygonB->count;
	b2Vec2 vertices[B2_MAX_POLYGON_VERTICES];
	b2Vec2 normals[B2_MAX_POLYGON_VERTICES];
	for ( int i = 0; i < count; ++i )
	{
		vertices[i] = b2TransformPoint( xf, polygonB->vertices[i] );
		normals[i] = b2RotateVector( xf.q, polygonB->normals[i] );
	}

	b2DistanceInput input;
	input.proxyA = b2MakeProxy( &segmentA->segment.point1, 2, 0.0f );
	input.proxyB = b2MakeProxy( vertices, count, 0.0f );
	input.transform = b2Transform_identity;
	input.useRadii = false;
	b2DistanceOutput output = b2ComputeShapeDistance( &input, cache, NULL, 0 );
	if ( output.distance > radiusB + B2_SPECULATIVE_DISTANCE )
	{
		return manifold;
	}

	b2Vec2 n0 = smoothParams.convex1 ? smoothParams.normal0 : normal1;
	b2Vec2 n2 = smoothParams.convex2 ? smoothParams.normal2 : normal1;
	int incidentIndex = -1;
	int incidentNormal = -1;
	if ( behind1 == false && output.distance > 0.1f * B2_LINEAR_SLOP )
	{
		if ( cache->count == 1 )
		{
			b2Vec2 pA = output.pointA;
			b2Vec2 pB = output.pointB;
			b2Vec2 normal = b2Normalize( b2Sub( pB, pA ) );
			b2NormalType type = b2ClassifyNormal( smoothParams, normal );
			if ( type == b2_normalSkip )
			{
				return manifold;
			}

			if ( type == b2_normalAdmit )
			{
				manifold.normal = normal;
				manifold.points[0].point = pA;
				manifold.points[0].separation = output.distance - radiusB;
				manifold.points[0].id = B2_MAKE_ID( cache->indexA[0], cache->indexB[0] );
				manifold.pointCount = 1;
				return manifold;
			}

			incidentIndex = cache->indexB[0];
		}
		else
		{
			B2_ASSERT( cache->count == 2 );
			int ia1 = cache->indexA[0];
			int ia2 = cache->indexA[1];
			int ib1 = cache->indexB[0];
			int ib2 = cache->indexB[1];
			if ( ia1 == ia2 )
			{
				B2_ASSERT( ib1 != ib2 );
				b2Vec2 normalB = b2Sub( output.pointA, output.pointB );
				float dot1 = b2Dot( normalB, normals[ib1] );
				float dot2 = b2Dot( normalB, normals[ib2] );
				int ib = dot1 > dot2 ? ib1 : ib2;
				normalB = normals[ib];
				b2NormalType type = b2ClassifyNormal( smoothParams, b2Neg( normalB ) );
				if ( type == b2_normalSkip )
				{
					return manifold;
				}

				if ( type == b2_normalAdmit )
				{
					ib1 = ib;
					ib2 = ib < count - 1 ? ib + 1 : 0;
					b2Vec2 b1 = vertices[ib1];
					b2Vec2 b2 = vertices[ib2];
					dot1 = b2Dot( normalB, b2Sub( p1, b1 ) );
					dot2 = b2Dot( normalB, b2Sub( p2, b1 ) );
					if ( dot1 < dot2 )
					{
						if ( b2Dot( n0, normalB ) < b2Dot( normal1, normalB ) )
						{
							return manifold;
						}
					}
					else
					{
						if ( b2Dot( n2, normalB ) < b2Dot( normal1, normalB ) )
						{
							return manifold;
						}
					}

					manifold = b2ClipSegments( b1, b2, p1, p2, normalB, radiusB, 0.0f, B2_MAKE_ID( ib1, 1 ), B2_MAKE_ID( ib2, 0 ) );
					B2_ASSERT( manifold.pointCount == 0 || manifold.pointCount == 2 );
					if ( manifold.pointCount == 2 )
					{
						manifold.normal = b2Neg( normalB );
					}

					return manifold;
				}

				incidentNormal = ib;
			}
			else
			{
				float dot1 = b2Dot( normal1, b2Sub( vertices[ib1], p1 ) );
				float dot2 = b2Dot( normal1, b2Sub( vertices[ib2], p2 ) );
				incidentIndex = dot1 < dot2 ? ib1 : ib2;
			}
		}
	}
	else
	{
		float edgeSeparation = FLT_MAX;
		for ( int i = 0; i < count; ++i )
		{
			float s = b2Dot( normal1, b2Sub( vertices[i], p1 ) );
			if ( s < edgeSeparation )
			{
				edgeSeparation = s;
				incidentIndex = i;
			}
		}

		if ( smoothParams.convex1 )
		{
			float s0 = FLT_MAX;
			for ( int i = 0; i < count; ++i )
			{
				float s = b2Dot( smoothParams.normal0, b2Sub( vertices[i], p1 ) );
				if ( s < s0 )
				{
					s0 = s;
				}
			}

			if ( s0 > edgeSeparation )
			{
				edgeSeparation = s0;
				incidentIndex = -1;
			}
		}

		if ( smoothParams.convex2 )
		{
			float s2 = FLT_MAX;
			for ( int i = 0; i < count; ++i )
			{
				float s = b2Dot( smoothParams.normal2, b2Sub( vertices[i], p2 ) );
				if ( s < s2 )
				{
					s2 = s;
				}
			}

			if ( s2 > edgeSeparation )
			{
				edgeSeparation = s2;
				incidentIndex = -1;
			}
		}

		float polygonSeparation = -FLT_MAX;
		int referenceIndex = -1;
		for ( int i = 0; i < count; ++i )
		{
			b2Vec2 n = normals[i];
			b2NormalType type = b2ClassifyNormal( smoothParams, b2Neg( n ) );
			if ( type != b2_normalAdmit )
			{
				continue;
			}

			b2Vec2 p = vertices[i];
			float s = b2MinFloat( b2Dot( n, b2Sub( p2, p ) ), b2Dot( n, b2Sub( p1, p ) ) );
			if ( s > polygonSeparation )
			{
				polygonSeparation = s;
				referenceIndex = i;
			}
		}

		if ( polygonSeparation > edgeSeparation )
		{
			int ia1 = referenceIndex;
			int ia2 = ia1 < count - 1 ? ia1 + 1 : 0;
			b2Vec2 a1 = vertices[ia1];
			b2Vec2 a2 = vertices[ia2];
			b2Vec2 n = normals[ia1];
			float dot1 = b2Dot( n, b2Sub( p1, a1 ) );
			float dot2 = b2Dot( n, b2Sub( p2, a1 ) );
			if ( dot1 < dot2 )
			{
				if ( b2Dot( n0, n ) < b2Dot( normal1, n ) )
				{
					return manifold;
				}
			}
			else
			{
				if ( b2Dot( n2, n ) < b2Dot( normal1, n ) )
				{
					return manifold;
				}
			}

			manifold = b2ClipSegments( a1, a2, p1, p2, normals[ia1], radiusB, 0.0f, B2_MAKE_ID( ia1, 1 ), B2_MAKE_ID( ia2, 0 ) );
			B2_ASSERT( manifold.pointCount == 0 || manifold.pointCount == 2 );
			if ( manifold.pointCount == 2 )
			{
				manifold.normal = b2Neg( normals[ia1] );
			}

			return manifold;
		}

		if ( incidentIndex == -1 )
		{
			return manifold;
		}
	}

	B2_ASSERT( incidentNormal != -1 || incidentIndex != -1 );
	b2Vec2 b1, b2;
	int ib1, ib2;
	if ( incidentNormal != -1 )
	{
		ib1 = incidentNormal;
		ib2 = ib1 < count - 1 ? ib1 + 1 : 0;
		b1 = vertices[ib1];
		b2 = vertices[ib2];
	}
	else
	{
		int i2 = incidentIndex;
		int i1 = i2 > 0 ? i2 - 1 : count - 1;
		float d1 = b2Dot( normal1, normals[i1] );
		float d2 = b2Dot( normal1, normals[i2] );
		if ( d1 < d2 )
		{
			ib1 = i1;
			ib2 = i2;
			b1 = vertices[ib1];
			b2 = vertices[ib2];
		}
		else
		{
			ib1 = i2;
			ib2 = i2 < count - 1 ? i2 + 1 : 0;
			b1 = vertices[ib1];
			b2 = vertices[ib2];
		}
	}

	manifold = b2ClipSegments( p1, p2, b1, b2, normal1, 0.0f, radiusB, B2_MAKE_ID( 0, ib2 ), B2_MAKE_ID( 1, ib1 ) );
	B2_ASSERT( manifold.pointCount == 0 || manifold.pointCount == 2 );
	return manifold;
}

b2LocalManifold b2CollideChainSegmentAndCapsule( const b2ChainSegment* segmentA, const b2Capsule* capsuleB, b2Transform xf,
												 b2SimplexCache* cache )
{
	b2Polygon polyB = b2MakeCapsule( capsuleB->center1, capsuleB->center2, capsuleB->radius );
	return b2CollideChainSegmentAndPolygon( segmentA, &polyB, xf, cache );
}

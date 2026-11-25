// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>

b2SegmentDistanceResult b2ComputeSegmentDistance( b2Vec2 p1, b2Vec2 q1, b2Vec2 p2, b2Vec2 q2 )
{
	b2SegmentDistanceResult result = {};

	b2Vec2 d1 = b2Sub( q1, p1 );
	b2Vec2 d2 = b2Sub( q2, p2 );
	b2Vec2 r = b2Sub( p1, p2 );
	float dd1 = b2Dot( d1, d1 );
	float dd2 = b2Dot( d2, d2 );
	float rd1 = b2Dot( r, d1 );
	float rd2 = b2Dot( r, d2 );

	const float epsSqr = FLT_EPSILON * FLT_EPSILON;

	if ( dd1 < epsSqr || dd2 < epsSqr )
	{
		if ( dd1 >= epsSqr )
		{
			result.fraction1 = b2ClampFloat( -rd1 / dd1, 0.0f, 1.0f );
			result.fraction2 = 0.0f;
		}
		else if ( dd2 >= epsSqr )
		{
			result.fraction1 = 0.0f;
			result.fraction2 = b2ClampFloat( rd2 / dd2, 0.0f, 1.0f );
		}
		else
		{
			result.fraction1 = 0.0f;
			result.fraction2 = 0.0f;
		}
	}
	else
	{
		float d12 = b2Dot( d1, d2 );
		float denominator = dd1 * dd2 - d12 * d12;
		float f1 = 0.0f;
		if ( denominator != 0.0f )
		{
			f1 = b2ClampFloat( ( d12 * rd2 - rd1 * dd2 ) / denominator, 0.0f, 1.0f );
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

		result.fraction1 = f1;
		result.fraction2 = f2;
	}

	result.closest1 = b2MulAdd( p1, result.fraction1, d1 );
	result.closest2 = b2MulAdd( p2, result.fraction2, d2 );
	result.distanceSquared = b2DistanceSquared( result.closest1, result.closest2 );
	return result;
}

b2ShapeProxy b2MakeProxy( const b2Vec2* points, int count, float radius )
{
	count = b2MinInt( count, B2_MAX_POLYGON_VERTICES );
	b2ShapeProxy proxy;
	for ( int i = 0; i < count; ++i )
	{
		proxy.points[i] = points[i];
	}
	proxy.count = count;
	proxy.radius = radius;
	return proxy;
}

b2ShapeProxy b2MakeOffsetProxy( const b2Vec2* points, int count, float radius, b2Vec2 position, b2Rot rotation )
{
	count = b2MinInt( count, B2_MAX_POLYGON_VERTICES );
	b2Transform transform = { position, rotation };
	b2ShapeProxy proxy;
	for ( int i = 0; i < count; ++i )
	{
		proxy.points[i] = b2TransformPoint( transform, points[i] );
	}
	proxy.count = count;
	proxy.radius = radius;
	return proxy;
}

static inline b2Vec2 b2Weight2( float a1, b2Vec2 w1, float a2, b2Vec2 w2 )
{
	return b2Vec2{ a1 * w1.x + a2 * w2.x, a1 * w1.y + a2 * w2.y };
}

static inline b2Vec2 b2Weight3( float a1, b2Vec2 w1, float a2, b2Vec2 w2, float a3, b2Vec2 w3 )
{
	return b2Vec2{ a1 * w1.x + a2 * w2.x + a3 * w3.x, a1 * w1.y + a2 * w2.y + a3 * w3.y };
}

static inline int b2FindSupport( const b2ShapeProxy* proxy, b2Vec2 direction )
{
	const b2Vec2* points = proxy->points;
	int count = proxy->count;
	int bestIndex = 0;
	float bestValue = b2Dot( points[0], direction );
	for ( int i = 1; i < count; ++i )
	{
		float value = b2Dot( points[i], direction );
		if ( value > bestValue )
		{
			bestIndex = i;
			bestValue = value;
		}
	}
	return bestIndex;
}

static b2Simplex b2MakeSimplexFromCache( b2SimplexCache cache, const b2ShapeProxy* proxyA, const b2ShapeProxy* proxyB )
{
	B2_ASSERT( cache.count <= 3 );
	b2Simplex s;
	s.count = cache.count;

	b2SimplexVertex* vertices[] = { &s.v1, &s.v2, &s.v3 };
	for ( int i = 0; i < s.count; ++i )
	{
		b2SimplexVertex* v = vertices[i];
		v->indexA = cache.indexA[i];
		v->indexB = cache.indexB[i];
		v->wA = proxyA->points[v->indexA];
		v->wB = proxyB->points[v->indexB];
		v->w = b2Sub( v->wA, v->wB );
		v->a = -1.0f;
	}

	if ( s.count == 0 )
	{
		b2SimplexVertex* v = vertices[0];
		v->indexA = 0;
		v->indexB = 0;
		v->wA = proxyA->points[0];
		v->wB = proxyB->points[0];
		v->w = b2Sub( v->wA, v->wB );
		v->a = 1.0f;
		s.count = 1;
	}

	return s;
}

static b2SimplexCache b2MakeSimplexCache( const b2Simplex* simplex )
{
	b2SimplexCache cache = {};
	cache.count = (uint16_t)simplex->count;
	const b2SimplexVertex* vertices[] = { &simplex->v1, &simplex->v2, &simplex->v3 };
	for ( int i = 0; i < simplex->count; ++i )
	{
		cache.indexA[i] = (uint8_t)vertices[i]->indexA;
		cache.indexB[i] = (uint8_t)vertices[i]->indexB;
	}
	return cache;
}

static void b2ComputeWitnessPoints( const b2Simplex* s, b2Vec2* a, b2Vec2* b )
{
	switch ( s->count )
	{
		case 1:
			*a = s->v1.wA;
			*b = s->v1.wB;
			break;

		case 2:
			*a = b2Weight2( s->v1.a, s->v1.wA, s->v2.a, s->v2.wA );
			*b = b2Weight2( s->v1.a, s->v1.wB, s->v2.a, s->v2.wB );
			break;

		case 3:
			*a = b2Weight3( s->v1.a, s->v1.wA, s->v2.a, s->v2.wA, s->v3.a, s->v3.wA );
			*b = *a;
			break;

		default:
			*a = b2Vec2_zero;
			*b = b2Vec2_zero;
			B2_ASSERT( false );
			break;
	}
}

static b2Vec2 b2SolveSimplex2( b2Simplex* s )
{
	b2Vec2 w1 = s->v1.w;
	b2Vec2 w2 = s->v2.w;
	b2Vec2 e12 = b2Sub( w2, w1 );

	float d12_2 = -b2Dot( w1, e12 );
	if ( d12_2 <= 0.0f )
	{
		s->v1.a = 1.0f;
		s->count = 1;
		return b2Neg( w1 );
	}

	float d12_1 = b2Dot( w2, e12 );
	if ( d12_1 <= 0.0f )
	{
		s->v2.a = 1.0f;
		s->count = 1;
		s->v1 = s->v2;
		return b2Neg( w2 );
	}

	float inv_d12 = 1.0f / ( d12_1 + d12_2 );
	s->v1.a = d12_1 * inv_d12;
	s->v2.a = d12_2 * inv_d12;
	s->count = 2;
	return b2CrossSV( b2Cross( b2Add( w1, w2 ), e12 ), e12 );
}

static b2Vec2 b2SolveSimplex3( b2Simplex* s )
{
	b2Vec2 w1 = s->v1.w;
	b2Vec2 w2 = s->v2.w;
	b2Vec2 w3 = s->v3.w;

	b2Vec2 e12 = b2Sub( w2, w1 );
	float d12_1 = b2Dot( w2, e12 );
	float d12_2 = -b2Dot( w1, e12 );

	b2Vec2 e13 = b2Sub( w3, w1 );
	float d13_1 = b2Dot( w3, e13 );
	float d13_2 = -b2Dot( w1, e13 );

	b2Vec2 e23 = b2Sub( w3, w2 );
	float d23_1 = b2Dot( w3, e23 );
	float d23_2 = -b2Dot( w2, e23 );

	float n123 = b2Cross( e12, e13 );
	float d123_1 = n123 * b2Cross( w2, w3 );
	float d123_2 = n123 * b2Cross( w3, w1 );
	float d123_3 = n123 * b2Cross( w1, w2 );

	if ( d12_2 <= 0.0f && d13_2 <= 0.0f )
	{
		s->v1.a = 1.0f;
		s->count = 1;
		return b2Neg( w1 );
	}

	if ( d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f )
	{
		float inv_d12 = 1.0f / ( d12_1 + d12_2 );
		s->v1.a = d12_1 * inv_d12;
		s->v2.a = d12_2 * inv_d12;
		s->count = 2;
		return b2CrossSV( b2Cross( b2Add( w1, w2 ), e12 ), e12 );
	}

	if ( d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f )
	{
		float inv_d13 = 1.0f / ( d13_1 + d13_2 );
		s->v1.a = d13_1 * inv_d13;
		s->v3.a = d13_2 * inv_d13;
		s->count = 2;
		s->v2 = s->v3;
		return b2CrossSV( b2Cross( b2Add( w1, w3 ), e13 ), e13 );
	}

	if ( d12_1 <= 0.0f && d23_2 <= 0.0f )
	{
		s->v2.a = 1.0f;
		s->count = 1;
		s->v1 = s->v2;
		return b2Neg( w2 );
	}

	if ( d13_1 <= 0.0f && d23_1 <= 0.0f )
	{
		s->v3.a = 1.0f;
		s->count = 1;
		s->v1 = s->v3;
		return b2Neg( w3 );
	}

	if ( d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f )
	{
		float inv_d23 = 1.0f / ( d23_1 + d23_2 );
		s->v2.a = d23_1 * inv_d23;
		s->v3.a = d23_2 * inv_d23;
		s->count = 2;
		s->v1 = s->v3;
		return b2CrossSV( b2Cross( b2Add( w2, w3 ), e23 ), e23 );
	}

	float inv_d123 = 1.0f / ( d123_1 + d123_2 + d123_3 );
	s->v1.a = d123_1 * inv_d123;
	s->v2.a = d123_2 * inv_d123;
	s->v3.a = d123_3 * inv_d123;
	s->count = 3;
	return b2Vec2_zero;
}

b2DistanceOutput b2ComputeShapeDistance( const b2DistanceInput* input, b2SimplexCache* cache, b2Simplex* simplexes, int simplexCapacity )
{
	B2_ASSERT( input->proxyA.count > 0 && input->proxyB.count > 0 );
	B2_ASSERT( input->proxyA.radius >= 0.0f );
	B2_ASSERT( input->proxyB.radius >= 0.0f );

	b2DistanceOutput output = {};
	const b2ShapeProxy* proxyA = &input->proxyA;

	b2ShapeProxy localProxyB;
	localProxyB.count = input->proxyB.count;
	localProxyB.radius = input->proxyB.radius;
	for ( int i = 0; i < localProxyB.count; ++i )
	{
		localProxyB.points[i] = b2TransformPoint( input->transform, input->proxyB.points[i] );
	}

	b2Simplex simplex = b2MakeSimplexFromCache( *cache, proxyA, &localProxyB );
	int simplexIndex = 0;
	if ( simplexes != NULL && simplexIndex < simplexCapacity )
	{
		simplexes[simplexIndex] = simplex;
		simplexIndex += 1;
	}

	b2SimplexVertex* vertices[] = { &simplex.v1, &simplex.v2, &simplex.v3 };
	b2Vec2 nonUnitNormal = b2Vec2_zero;
	int saveA[3], saveB[3];
	const int maxIterations = 20;
	int iteration = 0;
	while ( iteration < maxIterations )
	{
		int saveCount = simplex.count;
		for ( int i = 0; i < saveCount; ++i )
		{
			saveA[i] = vertices[i]->indexA;
			saveB[i] = vertices[i]->indexB;
		}

		b2Vec2 d = {};
		switch ( simplex.count )
		{
			case 1:
				d = b2Neg( simplex.v1.w );
				break;

			case 2:
				d = b2SolveSimplex2( &simplex );
				break;

			case 3:
				d = b2SolveSimplex3( &simplex );
				break;

			default:
				B2_ASSERT( false );
				break;
		}

		if ( simplex.count == 3 )
		{
			*cache = b2MakeSimplexCache( &simplex );
			b2Vec2 localPointA, localPointB;
			b2ComputeWitnessPoints( &simplex, &localPointA, &localPointB );
			output.distance = 0.0f;
			output.pointA = localPointA;
			output.pointB = localPointB;
			return output;
		}

#ifndef NDEBUG
		if ( simplexes != NULL && simplexIndex < simplexCapacity )
		{
			simplexes[simplexIndex] = simplex;
			simplexIndex += 1;
		}
#endif

		if ( b2Dot( d, d ) < 1000.0f * FLT_MIN )
		{
			*cache = b2MakeSimplexCache( &simplex );
			b2Vec2 localPointA, localPointB;
			b2ComputeWitnessPoints( &simplex, &localPointA, &localPointB );
			output.distance = 0.0f;
			output.pointA = localPointA;
			output.pointB = localPointB;
			return output;
		}

		nonUnitNormal = d;
		b2SimplexVertex* vertex = vertices[simplex.count];
		vertex->indexA = b2FindSupport( proxyA, d );
		vertex->wA = proxyA->points[vertex->indexA];
		vertex->indexB = b2FindSupport( &localProxyB, b2Neg( d ) );
		vertex->wB = localProxyB.points[vertex->indexB];
		vertex->w = b2Sub( vertex->wA, vertex->wB );
		++iteration;

		bool duplicate = false;
		for ( int i = 0; i < saveCount; ++i )
		{
			if ( vertex->indexA == saveA[i] && vertex->indexB == saveB[i] )
			{
				duplicate = true;
				break;
			}
		}

		if ( duplicate )
		{
			break;
		}

		simplex.count += 1;
	}

#ifndef NDEBUG
	if ( simplexes != NULL && simplexIndex < simplexCapacity )
	{
		simplexes[simplexIndex] = simplex;
		simplexIndex += 1;
	}
#endif

	b2Vec2 normal = b2Normalize( nonUnitNormal );
	b2Vec2 localPointA, localPointB;
	b2ComputeWitnessPoints( &simplex, &localPointA, &localPointB );
	output.normal = normal;
	output.distance = b2Distance( localPointA, localPointB );
	output.pointA = localPointA;
	output.pointB = localPointB;
	output.iterations = iteration;
	output.simplexCount = simplexIndex;
	*cache = b2MakeSimplexCache( &simplex );

	if ( input->useRadii )
	{
		float radiusA = input->proxyA.radius;
		float radiusB = input->proxyB.radius;
		output.distance = b2MaxFloat( 0.0f, output.distance - radiusA - radiusB );
		output.pointA = b2MulAdd( output.pointA, radiusA, normal );
		output.pointB = b2MulSub( output.pointB, radiusB, normal );
	}

	return output;
}

b2CastOutput b2CastShape( const b2ShapeCastPairInput* input )
{
	float linearSlop = B2_LINEAR_SLOP;
	float totalRadius = input->proxyA.radius + input->proxyB.radius;
	float target = b2MaxFloat( linearSlop, totalRadius - linearSlop );
	float tolerance = 0.25f * linearSlop;

	B2_ASSERT( target > tolerance );

	b2SimplexCache cache = {};
	float fraction = 0.0f;

	b2DistanceInput distanceInput = {};
	distanceInput.proxyA = input->proxyA;
	distanceInput.proxyB = input->proxyB;
	distanceInput.useRadii = false;
	distanceInput.transform = input->transform;

	b2Vec2 delta2 = input->translationB;
	b2CastOutput output = {};
	int iteration = 0;
	const int maxIterations = 20;

	for ( ; iteration < maxIterations; ++iteration )
	{
		output.iterations += 1;
		b2DistanceOutput distanceOutput = b2ComputeShapeDistance( &distanceInput, &cache, NULL, 0 );

		if ( distanceOutput.distance < target + tolerance )
		{
			if ( iteration == 0 )
			{
				if ( input->canEncroach && distanceOutput.distance > 2.0f * linearSlop )
				{
					target = distanceOutput.distance - linearSlop;
				}
				else
				{
					output.hit = true;
					b2Vec2 c1 = b2MulAdd( distanceOutput.pointA, input->proxyA.radius, distanceOutput.normal );
					b2Vec2 c2 = b2MulAdd( distanceOutput.pointB, -input->proxyB.radius, distanceOutput.normal );
					output.point = b2Lerp( c1, c2, 0.5f );
					return output;
				}
			}
			else
			{
				output.fraction = fraction;
				output.point = b2MulAdd( distanceOutput.pointA, input->proxyA.radius, distanceOutput.normal );
				output.normal = distanceOutput.normal;
				output.hit = true;
				return output;
			}
		}

		float denominator = b2Dot( delta2, distanceOutput.normal );
		if ( denominator >= 0.0f )
		{
			return output;
		}

		fraction += ( target - distanceOutput.distance ) / denominator;
		if ( fraction >= input->maxFraction )
		{
			return output;
		}

		distanceInput.transform.p = b2MulAdd( input->transform.p, fraction, delta2 );
	}

	return output;
}

b2Transform b2GetSweepTransform( const b2Sweep* sweep, float time )
{
	b2Transform xf;
	xf.p = b2Add( b2MulSV( 1.0f - time, sweep->c1 ), b2MulSV( time, sweep->c2 ) );

	b2Rot q = {
		( 1.0f - time ) * sweep->q1.c + time * sweep->q2.c,
		( 1.0f - time ) * sweep->q1.s + time * sweep->q2.s,
	};
	xf.q = b2NormalizeRot( q );
	xf.p = b2Sub( xf.p, b2RotateVector( xf.q, sweep->localCenter ) );
	return xf;
}

typedef enum b2SeparationType
{
	b2_pointsType,
	b2_faceAType,
	b2_faceBType
} b2SeparationType;

typedef struct b2SeparationFunction
{
	const b2ShapeProxy* proxyA;
	const b2ShapeProxy* proxyB;
	b2Sweep sweepA, sweepB;
	b2Vec2 localPoint;
	b2Vec2 axis;
	b2SeparationType type;
} b2SeparationFunction;

static b2SeparationFunction b2MakeSeparationFunction( b2SimplexCache cache, const b2ShapeProxy* proxyA, const b2Sweep* sweepA,
													  const b2ShapeProxy* proxyB, const b2Sweep* sweepB, float t1 )
{
	b2SeparationFunction f;
	f.proxyA = proxyA;
	f.proxyB = proxyB;
	int count = cache.count;
	B2_ASSERT( 0 < count && count < 3 );

	f.sweepA = *sweepA;
	f.sweepB = *sweepB;

	b2Transform xfA = b2GetSweepTransform( sweepA, t1 );
	b2Transform xfB = b2GetSweepTransform( sweepB, t1 );

	if ( count == 1 )
	{
		f.type = b2_pointsType;
		b2Vec2 localPointA = proxyA->points[cache.indexA[0]];
		b2Vec2 localPointB = proxyB->points[cache.indexB[0]];
		b2Vec2 pointA = b2TransformPoint( xfA, localPointA );
		b2Vec2 pointB = b2TransformPoint( xfB, localPointB );
		f.axis = b2Normalize( b2Sub( pointB, pointA ) );
		f.localPoint = b2Vec2_zero;
		return f;
	}

	if ( cache.indexA[0] == cache.indexA[1] )
	{
		f.type = b2_faceBType;
		b2Vec2 localPointB1 = proxyB->points[cache.indexB[0]];
		b2Vec2 localPointB2 = proxyB->points[cache.indexB[1]];
		f.axis = b2CrossVS( b2Sub( localPointB2, localPointB1 ), 1.0f );
		f.axis = b2Normalize( f.axis );
		b2Vec2 normal = b2RotateVector( xfB.q, f.axis );
		f.localPoint = b2Vec2{ 0.5f * ( localPointB1.x + localPointB2.x ), 0.5f * ( localPointB1.y + localPointB2.y ) };
		b2Vec2 pointB = b2TransformPoint( xfB, f.localPoint );
		b2Vec2 localPointA = proxyA->points[cache.indexA[0]];
		b2Vec2 pointA = b2TransformPoint( xfA, localPointA );
		float s = b2Dot( b2Sub( pointA, pointB ), normal );
		if ( s < 0.0f )
		{
			f.axis = b2Neg( f.axis );
		}
		return f;
	}

	f.type = b2_faceAType;
	b2Vec2 localPointA1 = proxyA->points[cache.indexA[0]];
	b2Vec2 localPointA2 = proxyA->points[cache.indexA[1]];
	f.axis = b2CrossVS( b2Sub( localPointA2, localPointA1 ), 1.0f );
	f.axis = b2Normalize( f.axis );
	b2Vec2 normal = b2RotateVector( xfA.q, f.axis );
	f.localPoint = b2Vec2{ 0.5f * ( localPointA1.x + localPointA2.x ), 0.5f * ( localPointA1.y + localPointA2.y ) };
	b2Vec2 pointA = b2TransformPoint( xfA, f.localPoint );
	b2Vec2 localPointB = proxyB->points[cache.indexB[0]];
	b2Vec2 pointB = b2TransformPoint( xfB, localPointB );
	float s = b2Dot( b2Sub( pointB, pointA ), normal );
	if ( s < 0.0f )
	{
		f.axis = b2Neg( f.axis );
	}
	return f;
}

static float b2FindMinSeparation( const b2SeparationFunction* f, int* indexA, int* indexB, float t )
{
	b2Transform xfA = b2GetSweepTransform( &f->sweepA, t );
	b2Transform xfB = b2GetSweepTransform( &f->sweepB, t );

	switch ( f->type )
	{
		case b2_pointsType:
		{
			b2Vec2 axisA = b2InvRotateVector( xfA.q, f->axis );
			b2Vec2 axisB = b2InvRotateVector( xfB.q, b2Neg( f->axis ) );
			*indexA = b2FindSupport( f->proxyA, axisA );
			*indexB = b2FindSupport( f->proxyB, axisB );
			b2Vec2 pointA = b2TransformPoint( xfA, f->proxyA->points[*indexA] );
			b2Vec2 pointB = b2TransformPoint( xfB, f->proxyB->points[*indexB] );
			return b2Dot( b2Sub( pointB, pointA ), f->axis );
		}

		case b2_faceAType:
		{
			b2Vec2 normal = b2RotateVector( xfA.q, f->axis );
			b2Vec2 pointA = b2TransformPoint( xfA, f->localPoint );
			b2Vec2 axisB = b2InvRotateVector( xfB.q, b2Neg( normal ) );
			*indexA = -1;
			*indexB = b2FindSupport( f->proxyB, axisB );
			b2Vec2 pointB = b2TransformPoint( xfB, f->proxyB->points[*indexB] );
			return b2Dot( b2Sub( pointB, pointA ), normal );
		}

		case b2_faceBType:
		{
			b2Vec2 normal = b2RotateVector( xfB.q, f->axis );
			b2Vec2 pointB = b2TransformPoint( xfB, f->localPoint );
			b2Vec2 axisA = b2InvRotateVector( xfA.q, b2Neg( normal ) );
			*indexB = -1;
			*indexA = b2FindSupport( f->proxyA, axisA );
			b2Vec2 pointA = b2TransformPoint( xfA, f->proxyA->points[*indexA] );
			return b2Dot( b2Sub( pointA, pointB ), normal );
		}

		default:
			B2_ASSERT( false );
			*indexA = -1;
			*indexB = -1;
			return 0.0f;
	}
}

static float b2EvaluateSeparation( const b2SeparationFunction* f, int indexA, int indexB, float t )
{
	b2Transform xfA = b2GetSweepTransform( &f->sweepA, t );
	b2Transform xfB = b2GetSweepTransform( &f->sweepB, t );

	switch ( f->type )
	{
		case b2_pointsType:
		{
			b2Vec2 pointA = b2TransformPoint( xfA, f->proxyA->points[indexA] );
			b2Vec2 pointB = b2TransformPoint( xfB, f->proxyB->points[indexB] );
			return b2Dot( b2Sub( pointB, pointA ), f->axis );
		}

		case b2_faceAType:
		{
			b2Vec2 normal = b2RotateVector( xfA.q, f->axis );
			b2Vec2 pointA = b2TransformPoint( xfA, f->localPoint );
			b2Vec2 pointB = b2TransformPoint( xfB, f->proxyB->points[indexB] );
			return b2Dot( b2Sub( pointB, pointA ), normal );
		}

		case b2_faceBType:
		{
			b2Vec2 normal = b2RotateVector( xfB.q, f->axis );
			b2Vec2 pointB = b2TransformPoint( xfB, f->localPoint );
			b2Vec2 pointA = b2TransformPoint( xfA, f->proxyA->points[indexA] );
			return b2Dot( b2Sub( pointA, pointB ), normal );
		}

		default:
			B2_ASSERT( false );
			return 0.0f;
	}
}

b2TOIOutput b2ComputeTimeOfImpact( const b2TOIInput* input )
{
	b2TOIOutput output = {};
	output.state = b2_toiStateUnknown;
	output.fraction = input->maxFraction;

	b2Sweep sweepA = input->sweepA;
	b2Sweep sweepB = input->sweepB;
	B2_ASSERT( b2IsNormalizedRot( sweepA.q1 ) && b2IsNormalizedRot( sweepA.q2 ) );
	B2_ASSERT( b2IsNormalizedRot( sweepB.q1 ) && b2IsNormalizedRot( sweepB.q2 ) );

	const b2ShapeProxy* proxyA = &input->proxyA;
	const b2ShapeProxy* proxyB = &input->proxyB;
	float tMax = input->maxFraction;
	float totalRadius = proxyA->radius + proxyB->radius;
	float target = b2MaxFloat( B2_LINEAR_SLOP, totalRadius - B2_LINEAR_SLOP );
	float tolerance = 0.25f * B2_LINEAR_SLOP;
	B2_ASSERT( target > tolerance );

	float t1 = 0.0f;
	const int k_maxIterations = 20;
	int distanceIterations = 0;
	b2SimplexCache cache = {};
	b2DistanceInput distanceInput = {};
	distanceInput.proxyA = input->proxyA;
	distanceInput.proxyB = input->proxyB;
	distanceInput.useRadii = false;

	for ( ;; )
	{
		b2Transform xfA = b2GetSweepTransform( &sweepA, t1 );
		b2Transform xfB = b2GetSweepTransform( &sweepB, t1 );
		distanceInput.transform = b2InvMulTransforms( xfA, xfB );
		b2DistanceOutput distanceOutput = b2ComputeShapeDistance( &distanceInput, &cache, NULL, 0 );

		b2Vec2 worldNormal = b2RotateVector( xfA.q, distanceOutput.normal );
		b2Vec2 worldPointA = b2TransformPoint( xfA, distanceOutput.pointA );
		b2Vec2 worldPointB = b2TransformPoint( xfA, distanceOutput.pointB );

		distanceIterations += 1;

		if ( distanceOutput.distance <= 0.0f )
		{
			output.state = b2_toiStateOverlapped;
			output.fraction = 0.0f;
			break;
		}

		if ( distanceOutput.distance <= target + tolerance )
		{
			output.state = b2_toiStateHit;
			b2Vec2 pA = b2MulAdd( worldPointA, proxyA->radius, worldNormal );
			b2Vec2 pB = b2MulAdd( worldPointB, -proxyB->radius, worldNormal );
			output.point = b2Lerp( pA, pB, 0.5f );
			output.normal = worldNormal;
			output.fraction = t1;
			break;
		}

		b2SeparationFunction fcn = b2MakeSeparationFunction( cache, proxyA, &sweepA, proxyB, &sweepB, t1 );
		bool done = false;
		float t2 = tMax;
		int pushBackIterations = 0;
		for ( ;; )
		{
			int indexA, indexB;
			float s2 = b2FindMinSeparation( &fcn, &indexA, &indexB, t2 );
			if ( s2 > target + tolerance )
			{
				output.state = b2_toiStateSeparated;
				output.fraction = tMax;
				done = true;
				break;
			}

			if ( s2 > target - tolerance )
			{
				t1 = t2;
				break;
			}

			float s1 = b2EvaluateSeparation( &fcn, indexA, indexB, t1 );
			if ( s1 < target - tolerance )
			{
				output.state = b2_toiStateFailed;
				output.fraction = t1;
				done = true;
				break;
			}

			if ( s1 <= target + tolerance )
			{
				output.state = b2_toiStateHit;
				b2Vec2 pA = b2MulAdd( worldPointA, proxyA->radius, worldNormal );
				b2Vec2 pB = b2MulAdd( worldPointB, -proxyB->radius, worldNormal );
				output.point = b2Lerp( pA, pB, 0.5f );
				output.normal = worldNormal;
				output.fraction = t1;
				done = true;
				break;
			}

			int rootIterationCount = 0;
			float a1 = t1, a2 = t2;
			for ( ;; )
			{
				float t;
				if ( rootIterationCount & 1 )
				{
					t = a1 + ( target - s1 ) * ( a2 - a1 ) / ( s2 - s1 );
				}
				else
				{
					t = 0.5f * ( a1 + a2 );
				}

				rootIterationCount += 1;
				float s = b2EvaluateSeparation( &fcn, indexA, indexB, t );
				if ( b2AbsFloat( s - target ) < tolerance )
				{
					t2 = t;
					break;
				}

				if ( s > target )
				{
					a1 = t;
					s1 = s;
				}
				else
				{
					a2 = t;
					s2 = s;
				}

				if ( rootIterationCount == 50 )
				{
					break;
				}
			}

			pushBackIterations += 1;
			if ( pushBackIterations == B2_MAX_POLYGON_VERTICES )
			{
				break;
			}
		}

		if ( done )
		{
			break;
		}

		if ( distanceIterations == k_maxIterations )
		{
			output.state = b2_toiStateFailed;
			b2Vec2 pA = b2MulAdd( worldPointA, proxyA->radius, worldNormal );
			b2Vec2 pB = b2MulAdd( worldPointB, -proxyB->radius, worldNormal );
			output.point = b2Lerp( pA, pB, 0.5f );
			output.normal = worldNormal;
			output.fraction = t1;
			break;
		}
	}

	return output;
}

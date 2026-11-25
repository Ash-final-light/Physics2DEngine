// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>

int SegmentDistanceTest( void )
{
	b2Vec2 p1 = { -1.0f, -1.0f };
	b2Vec2 q1 = { -1.0f, 1.0f };
	b2Vec2 p2 = { 2.0f, 0.0f };
	b2Vec2 q2 = { 1.0f, 0.0f };
	b2SegmentDistanceResult separated = b2ComputeSegmentDistance( p1, q1, p2, q2 );
	ENSURE_SMALL( separated.fraction1 - 0.5f, FLT_EPSILON );
	ENSURE_SMALL( separated.fraction2 - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( separated.closest1.x + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( separated.closest1.y, FLT_EPSILON );
	ENSURE_SMALL( separated.closest2.x - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( separated.closest2.y, FLT_EPSILON );
	ENSURE_SMALL( separated.distanceSquared - 4.0f, FLT_EPSILON );

	b2Vec2 a1 = { 0.0f, 0.0f };
	b2Vec2 a2 = { 2.0f, 0.0f };
	b2Vec2 b1 = { 1.0f, -1.0f };
	b2Vec2 b2 = { 1.0f, 1.0f };
	b2SegmentDistanceResult crossed = b2ComputeSegmentDistance( a1, a2, b1, b2 );
	ENSURE_SMALL( crossed.distanceSquared, FLT_EPSILON );
	ENSURE_SMALL( crossed.fraction1 - 0.5f, 0.0001f );
	ENSURE_SMALL( crossed.fraction2 - 0.5f, 0.0001f );
	ENSURE_SMALL( crossed.closest1.x - 1.0f, 0.0001f );
	ENSURE_SMALL( crossed.closest2.y, 0.0001f );

	b2Vec2 c1 = { 0.0f, 1.0f };
	b2Vec2 c2 = { 2.0f, 1.0f };
	b2SegmentDistanceResult parallel = b2ComputeSegmentDistance( a1, a2, c1, c2 );
	ENSURE_SMALL( parallel.distanceSquared - 1.0f, 0.0001f );
	ENSURE_SMALL( parallel.closest1.y, 0.0001f );
	ENSURE_SMALL( parallel.closest2.y - 1.0f, 0.0001f );
	ENSURE_SMALL( parallel.closest1.x - parallel.closest2.x, 0.0001f );

	b2Vec2 d1 = { 3.0f, 1.0f };
	b2Vec2 d2 = { 4.0f, 1.0f };
	b2SegmentDistanceResult ends = b2ComputeSegmentDistance( a1, a2, d1, d2 );
	ENSURE_SMALL( ends.fraction1 - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( ends.fraction2, FLT_EPSILON );
	ENSURE_SMALL( ends.closest1.x - 2.0f, FLT_EPSILON );
	ENSURE_SMALL( ends.closest2.x - 3.0f, FLT_EPSILON );
	ENSURE_SMALL( ends.distanceSquared - 2.0f, 0.0001f );

	b2Vec2 point = { 1.0f, 2.0f };
	b2SegmentDistanceResult collapsed = b2ComputeSegmentDistance( point, point, a1, a2 );
	ENSURE_SMALL( collapsed.fraction1, FLT_EPSILON );
	ENSURE_SMALL( collapsed.closest1.x - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( collapsed.closest1.y - 2.0f, FLT_EPSILON );
	ENSURE_SMALL( collapsed.closest2.x - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( collapsed.closest2.y, FLT_EPSILON );
	ENSURE_SMALL( collapsed.distanceSquared - 4.0f, 0.0001f );

	b2SegmentDistanceResult both = b2ComputeSegmentDistance( point, point, d1, d1 );
	ENSURE_SMALL( both.fraction1, FLT_EPSILON );
	ENSURE_SMALL( both.fraction2, FLT_EPSILON );
	ENSURE_SMALL( both.distanceSquared - b2DistanceSquared( point, d1 ), FLT_EPSILON );
	return 0;
}

int ShapeProxyTest( void )
{
	b2Vec2 points[9];
	for ( int i = 0; i < 9; ++i )
	{
		points[i].x = (float)i;
		points[i].y = (float)( i * 2 );
	}

	b2ShapeProxy proxy = b2MakeProxy( points, 3, 0.25f );
	ENSURE( proxy.count == 3 );
	ENSURE( proxy.radius == 0.25f );
	ENSURE( proxy.points[0].x == 0.0f && proxy.points[0].y == 0.0f );
	ENSURE( proxy.points[2].x == 2.0f && proxy.points[2].y == 4.0f );

	points[0].x = 9.0f;
	ENSURE( proxy.points[0].x == 0.0f );

	b2ShapeProxy capped = b2MakeProxy( points, 9, 0.0f );
	ENSURE( capped.count == B2_MAX_POLYGON_VERTICES );
	ENSURE( capped.radius == 0.0f );
	ENSURE( capped.points[7].x == 7.0f && capped.points[7].y == 14.0f );

	b2Vec2 local = { 1.0f, 0.0f };
	b2Vec2 position = { 2.0f, 3.0f };
	b2Rot quarter = { 0.0f, 1.0f };
	b2ShapeProxy moved = b2MakeOffsetProxy( &local, 1, 0.4f, position, quarter );
	ENSURE( moved.count == 1 && moved.radius == 0.4f );
	b2Vec2 expected = b2TransformPoint( { position, quarter }, local );
	ENSURE_SMALL( moved.points[0].x - expected.x, FLT_EPSILON );
	ENSURE_SMALL( moved.points[0].y - expected.y, FLT_EPSILON );

	b2ShapeProxy plain = b2MakeOffsetProxy( &local, 1, 0.4f, b2Vec2_zero, b2Rot_identity );
	ENSURE( plain.points[0].x == 1.0f && plain.points[0].y == 0.0f );
	return 0;
}

int ShapeDistanceTest( void )
{
	b2Vec2 box[4] = { { -1.0f, -1.0f }, { 1.0f, -1.0f }, { 1.0f, 1.0f }, { -1.0f, 1.0f } };
	b2Vec2 edge[2] = { { 2.0f, -1.0f }, { 2.0f, 1.0f } };

	b2DistanceInput input;
	input.proxyA = b2MakeProxy( box, 4, 0.0f );
	input.proxyB = b2MakeProxy( edge, 2, 0.0f );
	input.transform = b2Transform_identity;
	input.useRadii = false;

	b2SimplexCache cache = b2_emptySimplexCache;
	b2DistanceOutput output = b2ComputeShapeDistance( &input, &cache, NULL, 0 );
	ENSURE_SMALL( output.distance - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( output.pointA.x - 1.0f, 0.0001f );
	ENSURE_SMALL( output.pointB.x - 2.0f, 0.0001f );
	ENSURE( output.iterations > 0 );
	ENSURE( cache.count > 0 );

	b2DistanceOutput again = b2ComputeShapeDistance( &input, &cache, NULL, 0 );
	ENSURE_SMALL( again.distance - 1.0f, FLT_EPSILON );
	ENSURE( again.iterations <= output.iterations );

	b2DistanceInput overlap = input;
	overlap.proxyB = b2MakeProxy( box, 4, 0.0f );
	b2SimplexCache overlapCache = b2_emptySimplexCache;
	b2DistanceOutput hit = b2ComputeShapeDistance( &overlap, &overlapCache, NULL, 0 );
	ENSURE_SMALL( hit.distance, FLT_EPSILON );

	b2Vec2 origin = { 0.0f, 0.0f };
	b2Vec2 other = { 3.0f, 0.0f };
	b2DistanceInput radii;
	radii.proxyA = b2MakeProxy( &origin, 1, 1.0f );
	radii.proxyB = b2MakeProxy( &other, 1, 1.0f );
	radii.transform = b2Transform_identity;
	radii.useRadii = true;
	b2SimplexCache radiiCache = b2_emptySimplexCache;
	b2DistanceOutput round = b2ComputeShapeDistance( &radii, &radiiCache, NULL, 0 );
	ENSURE_SMALL( round.distance - 1.0f, 0.0001f );
	ENSURE_SMALL( round.pointA.x - 1.0f, 0.0001f );
	ENSURE_SMALL( round.pointB.x - 2.0f, 0.0001f );
	ENSURE_SMALL( round.normal.x - 1.0f, 0.0001f );

	b2Vec2 local = { 0.0f, 0.0f };
	b2DistanceInput shifted;
	shifted.proxyA = b2MakeProxy( box, 4, 0.0f );
	shifted.proxyB = b2MakeProxy( &local, 1, 0.0f );
	shifted.transform = { { 3.0f, 0.0f }, b2Rot_identity };
	shifted.useRadii = false;
	b2SimplexCache shiftedCache = b2_emptySimplexCache;
	b2DistanceOutput moved = b2ComputeShapeDistance( &shifted, &shiftedCache, NULL, 0 );
	ENSURE_SMALL( moved.distance - 2.0f, 0.0001f );
	ENSURE_SMALL( moved.pointB.x - 3.0f, 0.0001f );
	return 0;
}

int ShapeCastTest( void )
{
	b2Vec2 box[4] = { { -1.0f, -1.0f }, { 1.0f, -1.0f }, { 1.0f, 1.0f }, { -1.0f, 1.0f } };
	b2Vec2 edge[2] = { { 2.0f, -1.0f }, { 2.0f, 1.0f } };

	b2ShapeCastPairInput input = {};
	input.proxyA = b2MakeProxy( box, 4, 0.0f );
	input.proxyB = b2MakeProxy( edge, 2, 0.0f );
	input.transform = b2Transform_identity;
	input.translationB = { -2.0f, 0.0f };
	input.maxFraction = 1.0f;
	input.canEncroach = false;

	b2CastOutput output = b2CastShape( &input );
	ENSURE( output.hit == true );
	ENSURE_SMALL( output.fraction - 0.5f, 0.005f );
	ENSURE( output.normal.x > 0.0f );

	input.translationB = { 2.0f, 0.0f };
	b2CastOutput miss = b2CastShape( &input );
	ENSURE( miss.hit == false );

	input.translationB = { -2.0f, 0.0f };
	input.maxFraction = 0.25f;
	ENSURE( b2CastShape( &input ).hit == false );

	b2ShapeCastPairInput overlap = {};
	overlap.proxyA = b2MakeProxy( box, 4, 0.0f );
	overlap.proxyB = b2MakeProxy( box, 4, 0.0f );
	overlap.transform = b2Transform_identity;
	overlap.translationB = { -1.0f, 0.0f };
	overlap.maxFraction = 1.0f;
	overlap.canEncroach = false;
	b2CastOutput already = b2CastShape( &overlap );
	ENSURE( already.hit == true );
	ENSURE_SMALL( already.fraction, FLT_EPSILON );

	float halfThickness = 0.0016f;
	b2Vec2 corners[4] = {
		{ -0.5f, -halfThickness },
		{ 0.5f, -halfThickness },
		{ 0.5f, halfThickness },
		{ -0.5f, halfThickness },
	};
	b2Vec2 point = { 0.5f + B2_LINEAR_SLOP, 0.0f };
	b2DistanceInput thin;
	thin.proxyA = b2MakeProxy( corners, 4, 0.0f );
	thin.proxyB = b2MakeProxy( &point, 1, 0.0f );
	thin.transform = b2Transform_identity;
	thin.useRadii = false;
	b2SimplexCache thinCache = {};
	b2DistanceOutput thinDistance = b2ComputeShapeDistance( &thin, &thinCache, NULL, 0 );
	ENSURE( thinDistance.distance > 0.0f );
	ENSURE( b2IsNormalized( thinDistance.normal ) );
	ENSURE_SMALL( thinDistance.distance - B2_LINEAR_SLOP, 1.0e-6f );
	return 0;
}

int ShapeCastDispatchTest( void )
{
	b2Vec2 start = { 4.0f, 0.0f };
	b2ShapeCastInput input = {};
	input.proxy = b2MakeProxy( &start, 1, 0.0f );
	input.translation = { -6.0f, 0.0f };
	input.maxFraction = 1.0f;
	input.canEncroach = false;

	b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
	b2CastOutput circleHit = b2CastShapeCircle( &circle, &input );
	ENSURE( circleHit.hit == true );
	ENSURE( circleHit.fraction > 0.45f && circleHit.fraction < 0.55f );
	ENSURE( circleHit.normal.x > 0.0f );

	b2Capsule capsule = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 1.0f };
	b2CastOutput capsuleHit = b2CastShapeCapsule( &capsule, &input );
	ENSURE( capsuleHit.hit == true );
	ENSURE( capsuleHit.fraction > 0.25f && capsuleHit.fraction < 0.40f );

	b2Segment segment = { { 0.0f, -1.0f }, { 0.0f, 1.0f } };
	b2CastOutput segmentHit = b2CastShapeSegment( &segment, &input );
	ENSURE( segmentHit.hit == true );
	ENSURE( segmentHit.fraction > 0.60f && segmentHit.fraction < 0.75f );

	b2Polygon box = b2MakeBox( 1.0f, 1.0f );
	b2CastOutput boxHit = b2CastShapePolygon( &box, &input );
	ENSURE( boxHit.hit == true );
	ENSURE( boxHit.fraction > 0.45f && boxHit.fraction < 0.55f );
	ENSURE( boxHit.normal.x > 0.0f );

	input.translation = { 2.0f, 0.0f };
	ENSURE( b2CastShapeCircle( &circle, &input ).hit == false );
	ENSURE( b2CastShapeCapsule( &capsule, &input ).hit == false );
	ENSURE( b2CastShapeSegment( &segment, &input ).hit == false );
	ENSURE( b2CastShapePolygon( &box, &input ).hit == false );
	return 0;
}

int SweepTransformTest( void )
{
	b2Sweep sweep = {};
	sweep.localCenter = b2Vec2_zero;
	sweep.c1 = { 0.0f, 0.0f };
	sweep.c2 = { 4.0f, 2.0f };
	sweep.q1 = b2Rot_identity;
	sweep.q2 = { 0.0f, 1.0f };

	b2Transform start = b2GetSweepTransform( &sweep, 0.0f );
	ENSURE_SMALL( start.p.x, FLT_EPSILON );
	ENSURE_SMALL( start.p.y, FLT_EPSILON );
	ENSURE_SMALL( start.q.c - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( start.q.s, FLT_EPSILON );

	b2Transform end = b2GetSweepTransform( &sweep, 1.0f );
	ENSURE_SMALL( end.p.x - 4.0f, FLT_EPSILON );
	ENSURE_SMALL( end.p.y - 2.0f, FLT_EPSILON );
	ENSURE_SMALL( end.q.c, 0.0001f );
	ENSURE_SMALL( end.q.s - 1.0f, 0.0001f );

	b2Transform mid = b2GetSweepTransform( &sweep, 0.5f );
	ENSURE_SMALL( mid.p.x - 2.0f, 0.0001f );
	ENSURE_SMALL( mid.p.y - 1.0f, 0.0001f );
	float half = 0.70710678f;
	ENSURE_SMALL( mid.q.c - half, 0.0001f );
	ENSURE_SMALL( mid.q.s - half, 0.0001f );

	sweep.localCenter = { 1.0f, 0.0f };
	sweep.q2 = b2Rot_identity;
	b2Transform shifted = b2GetSweepTransform( &sweep, 0.0f );
	ENSURE_SMALL( shifted.p.x + 1.0f, 0.0001f );
	ENSURE_SMALL( shifted.p.y, 0.0001f );
	return 0;
}

int TimeOfImpactTest( void )
{
	b2Vec2 box[4] = { { -1.0f, -1.0f }, { 1.0f, -1.0f }, { 1.0f, 1.0f }, { -1.0f, 1.0f } };
	b2Vec2 edge[2] = { { 2.0f, -1.0f }, { 2.0f, 1.0f } };

	b2TOIInput input;
	input.proxyA = b2MakeProxy( box, 4, 0.0f );
	input.proxyB = b2MakeProxy( edge, 2, 0.0f );
	input.sweepA = { b2Vec2_zero, b2Vec2_zero, b2Vec2_zero, b2Rot_identity, b2Rot_identity };
	input.sweepB = { b2Vec2_zero, b2Vec2_zero, { -2.0f, 0.0f }, b2Rot_identity, b2Rot_identity };
	input.maxFraction = 1.0f;

	b2TOIOutput output = b2ComputeTimeOfImpact( &input );
	ENSURE( output.state == b2_toiStateHit );
	ENSURE_SMALL( output.fraction - 0.5f, 0.005f );
	ENSURE( output.normal.x > 0.0f );

	input.sweepB.c2 = { 2.0f, 0.0f };
	b2TOIOutput apart = b2ComputeTimeOfImpact( &input );
	ENSURE( apart.state == b2_toiStateSeparated );
	ENSURE_SMALL( apart.fraction - 1.0f, FLT_EPSILON );

	input.proxyB = b2MakeProxy( box, 4, 0.0f );
	input.sweepB.c1 = b2Vec2_zero;
	input.sweepB.c2 = b2Vec2_zero;
	b2TOIOutput overlap = b2ComputeTimeOfImpact( &input );
	ENSURE( overlap.state == b2_toiStateOverlapped );
	ENSURE_SMALL( overlap.fraction, FLT_EPSILON );
	return 0;
}

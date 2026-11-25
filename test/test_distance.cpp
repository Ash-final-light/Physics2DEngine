// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/collision.h"
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

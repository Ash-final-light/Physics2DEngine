// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>

static b2Hull b2RecurseHull( b2Vec2 p1, b2Vec2 p2, b2Vec2* ps, int count )
{
	b2Hull hull;
	hull.count = 0;

	if ( count == 0 )
	{
		return hull;
	}

	b2Vec2 e = b2Normalize( b2Sub( p2, p1 ) );

	b2Vec2 rightPoints[B2_MAX_POLYGON_VERTICES];
	int rightCount = 0;

	int bestIndex = 0;
	float bestDistance = b2Cross( b2Sub( ps[bestIndex], p1 ), e );
	if ( bestDistance > 0.0f )
	{
		rightPoints[rightCount++] = ps[bestIndex];
	}

	for ( int i = 1; i < count; ++i )
	{
		float distance = b2Cross( b2Sub( ps[i], p1 ), e );
		if ( distance > bestDistance )
		{
			bestIndex = i;
			bestDistance = distance;
		}

		if ( distance > 0.0f )
		{
			rightPoints[rightCount++] = ps[i];
		}
	}

	if ( bestDistance < 2.0f * B2_LINEAR_SLOP )
	{
		return hull;
	}

	b2Vec2 bestPoint = ps[bestIndex];
	b2Hull hull1 = b2RecurseHull( p1, bestPoint, rightPoints, rightCount );
	b2Hull hull2 = b2RecurseHull( bestPoint, p2, rightPoints, rightCount );

	for ( int i = 0; i < hull1.count; ++i )
	{
		hull.points[hull.count++] = hull1.points[i];
	}

	hull.points[hull.count++] = bestPoint;

	for ( int i = 0; i < hull2.count; ++i )
	{
		hull.points[hull.count++] = hull2.points[i];
	}

	B2_ASSERT( hull.count < B2_MAX_POLYGON_VERTICES );
	return hull;
}

b2Hull b2ComputeHull( const b2Vec2* points, int count )
{
	b2Hull hull = {};
	hull.count = 0;

	if ( count < 3 || count > B2_MAX_POLYGON_VERTICES )
	{
		return hull;
	}

	count = b2MinInt( count, B2_MAX_POLYGON_VERTICES );

	b2AABB aabb = { { FLT_MAX, FLT_MAX }, { -FLT_MAX, -FLT_MAX } };
	b2Vec2 ps[B2_MAX_POLYGON_VERTICES];
	int n = 0;
	const float linearSlop = B2_LINEAR_SLOP;
	const float tolSqr = 0.25f * linearSlop * linearSlop;
	for ( int i = 0; i < count; ++i )
	{
		aabb.lowerBound = b2Min( aabb.lowerBound, points[i] );
		aabb.upperBound = b2Max( aabb.upperBound, points[i] );

		b2Vec2 vi = points[i];
		bool unique = true;
		for ( int j = 0; j < n; ++j )
		{
			if ( b2DistanceSquared( vi, ps[j] ) < tolSqr )
			{
				unique = false;
				break;
			}
		}

		if ( unique )
		{
			ps[n++] = vi;
		}
	}

	if ( n < 3 )
	{
		return hull;
	}

	b2Vec2 c = b2AABB_Center( aabb );
	int f1 = 0;
	float dsq1 = b2DistanceSquared( c, ps[f1] );
	for ( int i = 1; i < n; ++i )
	{
		float dsq = b2DistanceSquared( c, ps[i] );
		if ( dsq > dsq1 )
		{
			f1 = i;
			dsq1 = dsq;
		}
	}

	b2Vec2 p1 = ps[f1];
	ps[f1] = ps[n - 1];
	n = n - 1;

	int f2 = 0;
	float dsq2 = b2DistanceSquared( p1, ps[f2] );
	for ( int i = 1; i < n; ++i )
	{
		float dsq = b2DistanceSquared( p1, ps[i] );
		if ( dsq > dsq2 )
		{
			f2 = i;
			dsq2 = dsq;
		}
	}

	b2Vec2 p2 = ps[f2];
	ps[f2] = ps[n - 1];
	n = n - 1;

	b2Vec2 rightPoints[B2_MAX_POLYGON_VERTICES - 2];
	int rightCount = 0;
	b2Vec2 leftPoints[B2_MAX_POLYGON_VERTICES - 2];
	int leftCount = 0;
	b2Vec2 e = b2Normalize( b2Sub( p2, p1 ) );

	for ( int i = 0; i < n; ++i )
	{
		float d = b2Cross( b2Sub( ps[i], p1 ), e );
		if ( d >= 2.0f * linearSlop )
		{
			rightPoints[rightCount++] = ps[i];
		}
		else if ( d <= -2.0f * linearSlop )
		{
			leftPoints[leftCount++] = ps[i];
		}
	}

	b2Hull hull1 = b2RecurseHull( p1, p2, rightPoints, rightCount );
	b2Hull hull2 = b2RecurseHull( p2, p1, leftPoints, leftCount );

	if ( hull1.count == 0 && hull2.count == 0 )
	{
		return hull;
	}

	hull.points[hull.count++] = p1;
	for ( int i = 0; i < hull1.count; ++i )
	{
		hull.points[hull.count++] = hull1.points[i];
	}
	hull.points[hull.count++] = p2;
	for ( int i = 0; i < hull2.count; ++i )
	{
		hull.points[hull.count++] = hull2.points[i];
	}

	B2_ASSERT( hull.count <= B2_MAX_POLYGON_VERTICES );

	bool searching = true;
	while ( searching && hull.count > 2 )
	{
		searching = false;
		for ( int i = 0; i < hull.count; ++i )
		{
			int i1 = i;
			int i2 = ( i + 1 ) % hull.count;
			int i3 = ( i + 2 ) % hull.count;
			b2Vec2 s1 = hull.points[i1];
			b2Vec2 s2 = hull.points[i2];
			b2Vec2 s3 = hull.points[i3];
			b2Vec2 r = b2Normalize( b2Sub( s3, s1 ) );
			float distance = b2Cross( b2Sub( s2, s1 ), r );
			if ( distance <= 2.0f * linearSlop )
			{
				for ( int j = i2; j < hull.count - 1; ++j )
				{
					hull.points[j] = hull.points[j + 1];
				}
				hull.count -= 1;
				searching = true;
				break;
			}
		}
	}

	if ( hull.count < 3 )
	{
		hull.count = 0;
	}

	return hull;
}

bool b2ValidateHull( const b2Hull* hull )
{
	if ( hull->count < 3 || B2_MAX_POLYGON_VERTICES < hull->count )
	{
		return false;
	}

	for ( int i = 0; i < hull->count; ++i )
	{
		int i1 = i;
		int i2 = i < hull->count - 1 ? i1 + 1 : 0;
		b2Vec2 p = hull->points[i1];
		b2Vec2 e = b2Normalize( b2Sub( hull->points[i2], p ) );

		for ( int j = 0; j < hull->count; ++j )
		{
			if ( j == i1 || j == i2 )
			{
				continue;
			}

			float distance = b2Cross( b2Sub( hull->points[j], p ), e );
			if ( distance >= 0.0f )
			{
				return false;
			}
		}
	}

	const float linearSlop = B2_LINEAR_SLOP;
	for ( int i = 0; i < hull->count; ++i )
	{
		int i1 = i;
		int i2 = ( i + 1 ) % hull->count;
		int i3 = ( i + 2 ) % hull->count;
		b2Vec2 p1 = hull->points[i1];
		b2Vec2 p2 = hull->points[i2];
		b2Vec2 p3 = hull->points[i3];
		b2Vec2 e = b2Normalize( b2Sub( p3, p1 ) );
		float distance = b2Cross( b2Sub( p2, p1 ), e );
		if ( distance <= linearSlop )
		{
			return false;
		}
	}

	return true;
}

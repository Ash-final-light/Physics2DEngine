// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/collision.h"
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

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "aabb.h"

#include <float.h>

b2CastOutput b2AABB_RayCast( b2AABB a, b2Vec2 p1, b2Vec2 p2 )
{
	b2CastOutput output = {};

	float tMin = -FLT_MAX;
	float tMax = FLT_MAX;

	b2Vec2 p = p1;
	b2Vec2 d = b2Sub( p2, p1 );
	b2Vec2 absD = b2Abs( d );
	b2Vec2 normal = b2Vec2_zero;

	if ( absD.x < FLT_EPSILON )
	{
		if ( p.x < a.lowerBound.x || a.upperBound.x < p.x )
		{
			return output;
		}
	}
	else
	{
		float invD = 1.0f / d.x;
		float t1 = ( a.lowerBound.x - p.x ) * invD;
		float t2 = ( a.upperBound.x - p.x ) * invD;
		float s = -1.0f;

		if ( t1 > t2 )
		{
			float tmp = t1;
			t1 = t2;
			t2 = tmp;
			s = 1.0f;
		}

		if ( t1 > tMin )
		{
			normal.y = 0.0f;
			normal.x = s;
			tMin = t1;
		}

		tMax = b2MinFloat( tMax, t2 );
		if ( tMin > tMax )
		{
			return output;
		}
	}

	if ( absD.y < FLT_EPSILON )
	{
		if ( p.y < a.lowerBound.y || a.upperBound.y < p.y )
		{
			return output;
		}
	}
	else
	{
		float invD = 1.0f / d.y;
		float t1 = ( a.lowerBound.y - p.y ) * invD;
		float t2 = ( a.upperBound.y - p.y ) * invD;
		float s = -1.0f;

		if ( t1 > t2 )
		{
			float tmp = t1;
			t1 = t2;
			t2 = tmp;
			s = 1.0f;
		}

		if ( t1 > tMin )
		{
			normal.x = 0.0f;
			normal.y = s;
			tMin = t1;
		}

		tMax = b2MinFloat( tMax, t2 );
		if ( tMin > tMax )
		{
			return output;
		}
	}

	if ( tMin < 0.0f || 1.0f < tMin )
	{
		return output;
	}

	output.fraction = tMin;
	output.normal = normal;
	output.point = b2Lerp( p1, p2, tMin );
	output.hit = true;
	return output;
}

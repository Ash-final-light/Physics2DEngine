// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/collision.h"
#include "box2d/math_functions.h"

b2CastOutput b2AABB_RayCast( b2AABB a, b2Vec2 p1, b2Vec2 p2 );


static inline b2AABB b2OffsetAABB( b2AABB box, b2Pos origin )
{
	b2AABB result;
	result.lowerBound.x = b2RoundDownFloat( origin.x + (double)box.lowerBound.x );
	result.lowerBound.y = b2RoundDownFloat( origin.y + (double)box.lowerBound.y );
	result.upperBound.x = b2RoundUpFloat( origin.x + (double)box.upperBound.x );
	result.upperBound.y = b2RoundUpFloat( origin.y + (double)box.upperBound.y );
	return result;
}

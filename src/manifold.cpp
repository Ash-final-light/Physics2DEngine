// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

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

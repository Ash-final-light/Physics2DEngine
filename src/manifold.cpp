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

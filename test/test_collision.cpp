// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>

int CircleManifoldTest( void )
{
	b2Circle circleA = { { 0.0f, 0.0f }, 1.0f };
	b2Circle circleB = { { 0.0f, 0.0f }, 1.0f };
	b2Transform xf = { { 1.5f, 0.0f }, b2Rot_identity };

	b2LocalManifold hit = b2CollideCircles( &circleA, &circleB, xf );
	ENSURE( hit.pointCount == 1 );
	ENSURE_SMALL( hit.normal.x - 1.0f, 0.0001f );
	ENSURE_SMALL( hit.normal.y, 0.0001f );
	ENSURE_SMALL( hit.points[0].separation - ( 1.5f - 2.0f ), 0.0001f );
	ENSURE_SMALL( hit.points[0].point.x - 0.75f, 0.0001f );
	ENSURE_SMALL( hit.points[0].point.y, 0.0001f );

	xf.p = { 2.0f, 0.0f };
	b2LocalManifold touch = b2CollideCircles( &circleA, &circleB, xf );
	ENSURE( touch.pointCount == 1 );
	ENSURE_SMALL( touch.points[0].separation, 0.0001f );

	xf.p = { 2.0f + B2_SPECULATIVE_DISTANCE * 0.5f, 0.0f };
	b2LocalManifold near = b2CollideCircles( &circleA, &circleB, xf );
	ENSURE( near.pointCount == 1 );
	ENSURE( near.points[0].separation > 0.0f );
	ENSURE( near.points[0].separation <= B2_SPECULATIVE_DISTANCE );

	xf.p = { 3.0f, 0.0f };
	b2LocalManifold miss = b2CollideCircles( &circleA, &circleB, xf );
	ENSURE( miss.pointCount == 0 );
	return 0;
}

int CapsuleCircleManifoldTest( void )
{
	b2Capsule capsule = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 1.0f };
	b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
	b2Transform side = { { 0.0f, 1.5f }, b2Rot_identity };
	b2LocalManifold flank = b2CollideCapsuleAndCircle( &capsule, &circle, side );
	ENSURE( flank.pointCount == 1 );
	ENSURE_SMALL( flank.normal.x, 0.0001f );
	ENSURE_SMALL( flank.normal.y - 1.0f, 0.0001f );
	ENSURE_SMALL( flank.points[0].separation - ( 1.5f - 2.0f ), 0.0001f );
	ENSURE_SMALL( flank.points[0].point.x, 0.0001f );
	ENSURE_SMALL( flank.points[0].point.y - 0.75f, 0.0001f );

	b2Transform cap = { { 3.0f, 0.0f }, b2Rot_identity };
	b2LocalManifold end = b2CollideCapsuleAndCircle( &capsule, &circle, cap );
	ENSURE( end.pointCount == 1 );
	ENSURE_SMALL( end.normal.x - 1.0f, 0.0001f );
	ENSURE_SMALL( end.points[0].separation, 0.0001f );
	ENSURE_SMALL( end.points[0].point.x - 2.0f, 0.0001f );

	b2Transform far = { { 0.0f, 4.0f }, b2Rot_identity };
	ENSURE( b2CollideCapsuleAndCircle( &capsule, &circle, far ).pointCount == 0 );
	return 0;
}

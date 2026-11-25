// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "geometry.h"
#include "test_macros.h"

#include <math.h>

int CircleSegmentTest( void )
{
	b2Circle circle = { { 1.0f, 2.0f }, 0.5f };
	ENSURE( circle.center.x == 1.0f && circle.center.y == 2.0f && circle.radius == 0.5f );
	ENSURE( b2IsValidCircle( &circle ) == true );

	b2Circle point = { { 0.0f, 0.0f }, 0.0f };
	ENSURE( b2IsValidCircle( &point ) == true );

	b2Circle negative = circle;
	negative.radius = -0.1f;
	ENSURE( b2IsValidCircle( &negative ) == false );

	b2Circle bad = circle;
	bad.radius = NAN;
	ENSURE( b2IsValidCircle( &bad ) == false );
	bad = circle;
	bad.center.x = INFINITY;
	ENSURE( b2IsValidCircle( &bad ) == false );

	b2Segment segment = { { -1.0f, 0.0f }, { 1.0f, 0.0f } };
	ENSURE( segment.point1.x == -1.0f && segment.point2.x == 1.0f );
	ENSURE( b2IsValidSegment( &segment ) == true );

	b2Segment degenerate = { { 3.0f, 4.0f }, { 3.0f, 4.0f } };
	ENSURE( b2IsValidSegment( &degenerate ) == true );

	b2Segment broken = segment;
	broken.point2.y = NAN;
	ENSURE( b2IsValidSegment( &broken ) == false );
	return 0;
}

int CapsuleTest( void )
{
	b2Capsule capsule = { { 0.0f, 0.0f }, { 0.0f, 2.0f }, 0.25f };
	ENSURE( capsule.center1.y == 0.0f && capsule.center2.y == 2.0f && capsule.radius == 0.25f );
	ENSURE( b2IsValidCapsule( &capsule ) == true );

	b2Capsule degenerate = { { 1.0f, 1.0f }, { 1.0f, 1.0f }, 0.5f };
	ENSURE( b2IsValidCapsule( &degenerate ) == true );

	b2Capsule point = { { -2.0f, 3.0f }, { -2.0f, 3.0f }, 0.0f };
	ENSURE( b2IsValidCapsule( &point ) == true );

	b2Capsule negative = capsule;
	negative.radius = -1.0f;
	ENSURE( b2IsValidCapsule( &negative ) == false );

	b2Capsule bad = capsule;
	bad.center2.x = NAN;
	ENSURE( b2IsValidCapsule( &bad ) == false );
	bad = capsule;
	bad.radius = INFINITY;
	ENSURE( b2IsValidCapsule( &bad ) == false );
	return 0;
}

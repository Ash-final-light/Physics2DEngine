// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/math_functions.h"

#include <math.h>

#define ATAN_TOL 0.00004f

int TrigTest( void )
{
	for ( float t = -10.0f; t < 10.0f; t += 0.01f )
	{
		float angle = B2_PI * t;
		b2Rot r = b2MakeRot( angle );
		float c = cosf( angle );
		float s = sinf( angle );
		ENSURE_SMALL( r.c - c, 0.002f );
		ENSURE_SMALL( r.s - s, 0.002f );

		float xn = b2UnwindAngle( angle );
		ENSURE( -B2_PI <= xn && xn <= B2_PI );

		float a = b2Atan2( s, c );
		ENSURE( b2IsValidFloat( a ) );
		float diff = b2AbsFloat( a - xn );
		if ( diff > B2_PI )
		{
			diff -= 2.0f * B2_PI;
		}
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	for ( float y = -1.0f; y <= 1.0f; y += 0.1f )
	{
		for ( float x = -1.0f; x <= 1.0f; x += 0.1f )
		{
			float a1 = b2Atan2( y, x );
			float a2 = atan2f( y, x );
			ENSURE( b2IsValidFloat( a1 ) );
			ENSURE_SMALL( b2AbsFloat( a1 - a2 ), ATAN_TOL );
		}
	}

	ENSURE_SMALL( b2AbsFloat( b2Atan2( 0.0f, 0.0f ) - atan2f( 0.0f, 0.0f ) ), ATAN_TOL );
	ENSURE_SMALL( b2AbsFloat( b2Atan2( 1.0f, 0.0f ) - atan2f( 1.0f, 0.0f ) ), ATAN_TOL );
	ENSURE_SMALL( b2AbsFloat( b2Atan2( -1.0f, 0.0f ) - atan2f( -1.0f, 0.0f ) ), ATAN_TOL );
	ENSURE_SMALL( b2AbsFloat( b2Atan2( 0.0f, -1.0f ) - atan2f( 0.0f, -1.0f ) ), ATAN_TOL );

	b2Rot identity = b2Rot_identity;
	b2Rot quarter = b2MakeRot( 0.5f * B2_PI );
	float relative = b2RelativeAngle( identity, quarter );
	ENSURE_SMALL( b2AbsFloat( relative - 0.5f * B2_PI ), 0.01f );
	ENSURE_SMALL( b2AbsFloat( b2Rot_GetAngle( quarter ) - 0.5f * B2_PI ), 0.01f );
	return 0;
}

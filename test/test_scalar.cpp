// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>

int ScalarTest( void )
{
	if ( b2IsValidFloat( 0.0f ) == false || b2IsValidFloat( -0.0f ) == false )
	{
		return 1;
	}
	if ( b2IsValidFloat( 1.0f ) == false || b2IsValidFloat( -42.5f ) == false )
	{
		return 1;
	}
	if ( b2IsValidFloat( FLT_MIN ) == false || b2IsValidFloat( -FLT_MIN ) == false )
	{
		return 1;
	}
	if ( b2IsValidFloat( FLT_MAX ) == false || b2IsValidFloat( -FLT_MAX ) == false )
	{
		return 1;
	}
	if ( b2IsValidFloat( NAN ) || b2IsValidFloat( INFINITY ) || b2IsValidFloat( -INFINITY ) )
	{
		return 1;
	}

	if ( b2MinInt( -3, 4 ) != -3 || b2MinInt( 5, 5 ) != 5 || b2MaxInt( -3, 4 ) != 4 )
	{
		return 1;
	}
	if ( b2AbsInt( 0 ) != 0 || b2AbsInt( -7 ) != 7 || b2AbsInt( 7 ) != 7 )
	{
		return 1;
	}
	if ( b2ClampInt( -1, 0, 3 ) != 0 || b2ClampInt( 2, 0, 3 ) != 2 || b2ClampInt( 9, 0, 3 ) != 3 )
	{
		return 1;
	}
	if ( b2ClampInt( 1, 1, 1 ) != 1 )
	{
		return 1;
	}
	if ( b2CeilingInt( 0, 1 ) != 0 || b2CeilingInt( 1, 1 ) != 1 || b2CeilingInt( 4, 2 ) != 2 || b2CeilingInt( 5, 2 ) != 3 )
	{
		return 1;
	}

	if ( b2MinFloat( -1.5f, 2.0f ) != -1.5f || b2MaxFloat( -1.5f, 2.0f ) != 2.0f )
	{
		return 1;
	}
	if ( b2AbsFloat( 0.0f ) != 0.0f || b2AbsFloat( -3.25f ) != 3.25f )
	{
		return 1;
	}
	if ( b2ClampFloat( -1.0f, 0.0f, 2.0f ) != 0.0f || b2ClampFloat( 1.0f, 0.0f, 2.0f ) != 1.0f )
	{
		return 1;
	}
	if ( b2ClampFloat( 3.0f, 0.0f, 2.0f ) != 2.0f || b2ClampFloat( 1.0f, 1.0f, 1.0f ) != 1.0f )
	{
		return 1;
	}

	return 0;
}

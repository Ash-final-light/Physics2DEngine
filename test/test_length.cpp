// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math_functions.h"

#include <cmath>
#include <cstring>

static int s_logHits;
static int s_assertHits;

static void CaptureLog( const char* message )
{
	(void)message;
	s_logHits += 1;
}

static int SilentAssert( const char* condition, const char* fileName, int lineNumber )
{
	(void)condition;
	(void)fileName;
	(void)lineNumber;
	s_assertHits += 1;
	return 0;
}

int LengthUnitsTest( void )
{
	b2SetAssertCallback( SilentAssert );
	b2SetLogCallback( CaptureLog );

	if ( b2GetLengthUnitsPerMeter() != 1.0f || b2_lengthUnitsPerMeter != 1.0f )
	{
		return 1;
	}

	b2SetLengthUnitsPerMeter( 32.0f );
	if ( b2GetLengthUnitsPerMeter() != 32.0f )
	{
		return 1;
	}

	s_logHits = 0;
	s_assertHits = 0;
	float rejected[] = { 0.0f, -2.0f, NAN, INFINITY };
	for ( float value : rejected )
	{
		b2SetLengthUnitsPerMeter( value );
		if ( b2GetLengthUnitsPerMeter() != 32.0f )
		{
			return 1;
		}
	}

	if ( s_logHits != 4 )
	{
		return 1;
	}
#if !defined( NDEBUG ) || defined( B2_ENABLE_ASSERT )
	if ( s_assertHits != 4 )
	{
		return 1;
	}
#endif

	b2SetLengthUnitsPerMeter( 0.5f );
	if ( b2GetLengthUnitsPerMeter() != 0.5f )
	{
		return 1;
	}

	b2SetLengthUnitsPerMeter( 1.0f );
	return 0;
}

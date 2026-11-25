// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/base.h"

#include <stdio.h>
#include <string.h>

static int s_captureHits;
static int s_restoredHits;
static char s_condition[64];

static int CaptureAssert( const char* condition, const char* fileName, int lineNumber )
{
	(void)fileName;
	(void)lineNumber;
	s_captureHits += 1;
	strncpy( s_condition, condition, sizeof( s_condition ) - 1 );
	s_condition[sizeof( s_condition ) - 1] = '\0';
	return 0;
}

static int RestoredAssert( const char* condition, const char* fileName, int lineNumber )
{
	(void)condition;
	(void)fileName;
	(void)lineNumber;
	s_restoredHits += 1;
	return 0;
}

int AssertCallbackTest( void )
{
	s_captureHits = 0;
	s_restoredHits = 0;
	s_condition[0] = '\0';

	b2SetAssertCallback( CaptureAssert );
#if !defined( NDEBUG ) || defined( B2_ENABLE_ASSERT )
	B2_ASSERT( 1 == 0 );
	if ( s_captureHits != 1 )
	{
		return 1;
	}
	if ( strcmp( s_condition, "1 == 0" ) != 0 )
	{
		return 1;
	}
#else
	printf( "assert macro disabled in this configuration\n" );
#endif

	b2SetAssertCallback( RestoredAssert );
#if !defined( NDEBUG ) || defined( B2_ENABLE_ASSERT )
	B2_ASSERT( 2 == 1 );
	if ( s_captureHits != 1 || s_restoredHits != 1 )
	{
		return 1;
	}
	printf( "assert callback triggered and restored\n" );
#endif

	return 0;
}

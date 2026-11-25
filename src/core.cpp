// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include "box2d/math_functions.h"

#include <cmath>
#include <stdarg.h>
#include <stdio.h>

b2Version b2GetVersion( void )
{
	return b2Version{
		.major = B2_VERSION_MAJOR,
		.minor = B2_VERSION_MINOR,
		.revision = B2_VERSION_REVISION,
	};
}

bool b2IsDoublePrecision( void )
{
#if defined( BOX2D_DOUBLE_PRECISION )
	return true;
#else
	return false;
#endif
}

static int b2DefaultAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	fprintf( stderr, "BOX2D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );
	fflush( stderr );
	return 1;
}

static b2AssertCallbackFcn* b2AssertHandler = b2DefaultAssertFcn;

void b2SetAssertCallback( b2AssertCallbackFcn* assertFcn )
{
	B2_ASSERT( assertFcn != NULL );
	b2AssertHandler = assertFcn;
}

#if !defined( NDEBUG ) || defined( B2_ENABLE_ASSERT )
int b2InternalAssert( const char* condition, const char* fileName, int lineNumber )
{
	int result = b2AssertHandler( condition, fileName, lineNumber );
	if ( result )
	{
		B2_BREAKPOINT;
	}
	return result;
}
#endif

static void b2DefaultLogFcn( const char* message )
{
	printf( "Box2D: %s\n", message );
}

static b2LogCallbackFcn* b2LogHandler = b2DefaultLogFcn;

void b2SetLogCallback( b2LogCallbackFcn* logFcn )
{
	B2_ASSERT( logFcn != NULL );
	b2LogHandler = logFcn;
}

void b2Log( const char* format, ... )
{
	va_list args;
	va_start( args, format );
	char buffer[512];
	vsnprintf( buffer, sizeof( buffer ), format, args );
	b2LogHandler( buffer );
	va_end( args );
}

float b2_lengthUnitsPerMeter = 1.0f;

void b2SetLengthUnitsPerMeter( float lengthUnits )
{
	if ( std::isfinite( lengthUnits ) == false || lengthUnits <= 0.0f )
	{
		b2Log( "invalid input: lengthUnits in %s\n", __func__ );
		B2_ASSERT( std::isfinite( lengthUnits ) && lengthUnits > 0.0f );
		return;
	}

	b2_lengthUnitsPerMeter = lengthUnits;
}

float b2GetLengthUnitsPerMeter( void )
{
	return b2_lengthUnitsPerMeter;
}

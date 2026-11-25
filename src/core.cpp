// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/base.h"

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

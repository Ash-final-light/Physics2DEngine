// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include <stdio.h>
#include <string.h>

extern int ToolchainTest( void );
extern "C" int VersionCTest( void );
extern "C" int AssertCallbackTest( void );
int LogCallbackTest( void );
int LengthUnitsTest( void );

int main( int argc, char** argv )
{
	const char* filter = NULL;
	int ran = 0;
	if ( argc > 1 )
	{
		filter = argv[1];
	}

	printf( "Starting unit tests\n" );

	if ( filter == NULL || strcmp( filter, "ToolchainTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ToolchainTest );
	}

	if ( filter == NULL || strcmp( filter, "VersionCTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( VersionCTest );
	}

	if ( filter == NULL || strcmp( filter, "AssertCallbackTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( AssertCallbackTest );
	}

	if ( filter == NULL || strcmp( filter, "LogCallbackTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( LogCallbackTest );
	}

	if ( filter == NULL || strcmp( filter, "LengthUnitsTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( LengthUnitsTest );
	}

	if ( ran == 0 )
	{
		printf( "no tests matched\n" );
		return 1;
	}

	printf( "All tests passed\n" );
	return 0;
}

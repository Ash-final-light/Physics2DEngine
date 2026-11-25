// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include <stdio.h>
#include <string.h>

extern int ToolchainTest( void );

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

	if ( ran == 0 )
	{
		printf( "no tests matched\n" );
		return 1;
	}

	printf( "All tests passed\n" );
	return 0;
}

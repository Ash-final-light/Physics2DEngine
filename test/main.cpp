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
int MemoryTest( void );
int GrowAllocTest( void );
int ZeroAllocTest( void );
int ScalarTest( void );
int VectorTest( void );
int RotationTest( void );
int TransformTest( void );
int MatrixTest( void );
int TrigTest( void );
int ArrayCreateTest( void );
int ArrayReserveTest( void );
int ArrayPushTest( void );

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

	if ( filter == NULL || strcmp( filter, "MemoryTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( MemoryTest );
	}

	if ( filter == NULL || strcmp( filter, "GrowAllocTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( GrowAllocTest );
	}

	if ( filter == NULL || strcmp( filter, "ZeroAllocTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ZeroAllocTest );
	}

	if ( filter == NULL || strcmp( filter, "ScalarTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ScalarTest );
	}

	if ( filter == NULL || strcmp( filter, "VectorTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( VectorTest );
	}

	if ( filter == NULL || strcmp( filter, "RotationTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( RotationTest );
	}

	if ( filter == NULL || strcmp( filter, "TransformTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( TransformTest );
	}

	if ( filter == NULL || strcmp( filter, "MatrixTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( MatrixTest );
	}

	if ( filter == NULL || strcmp( filter, "TrigTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( TrigTest );
	}

	if ( filter == NULL || strcmp( filter, "ArrayCreateTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ArrayCreateTest );
	}

	if ( filter == NULL || strcmp( filter, "ArrayReserveTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ArrayReserveTest );
	}

	if ( filter == NULL || strcmp( filter, "ArrayPushTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ArrayPushTest );
	}

	if ( ran == 0 )
	{
		printf( "no tests matched\n" );
		return 1;
	}

	printf( "All tests passed\n" );
	return 0;
}

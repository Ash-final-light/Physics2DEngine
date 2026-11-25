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
int ArrayIndexTest( void );
int ArrayPopTest( void );
int ArrayRemoveSwapTest( void );
int ArrayResizeTest( void );
int ArrayConstraintTest( void );
int SwapValuesTest( void );
int BitSetTest( void );
int BitSetGrowTest( void );
int IdPoolTest( void );
int HashSetInsertTest( void );
int HashSetRemoveTest( void );
int HashSetGrowTest( void );
int StackAllocatorTest( void );
int TimerTest( void );
int AABBTest( void );
int AABBRayCastTest( void );
int CircleSegmentTest( void );
int CapsuleTest( void );
int PolygonTest( void );
int OffsetPolygonTest( void );
int CircleMassTest( void );
int CapsuleMassTest( void );
int PolygonMassTest( void );
int ShapeBoundsTest( void );
int PointInCircleTest( void );
int PointInCapsuleTest( void );
int PointInPolygonTest( void );
int RayCastCircleTest( void );
int RayCastCapsuleTest( void );
int RayCastSegmentTest( void );
int RayCastPolygonTest( void );
int SegmentDistanceTest( void );
int ShapeProxyTest( void );
int ShapeDistanceTest( void );
int ShapeCastTest( void );
int ShapeCastDispatchTest( void );
int SweepTransformTest( void );
int TimeOfImpactTest( void );
int CircleManifoldTest( void );
int CapsuleCircleManifoldTest( void );
int PolygonCircleManifoldTest( void );
int CapsuleManifoldTest( void );
int SegmentCircleManifoldTest( void );
int SegmentCapsuleManifoldTest( void );
int PolygonCapsuleManifoldTest( void );
int PolygonManifoldTest( void );
int SegmentPolygonManifoldTest( void );
int ChainCircleManifoldTest( void );
int ChainCapsuleManifoldTest( void );
int HullTest( void );

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

	if ( filter == NULL || strcmp( filter, "ArrayIndexTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ArrayIndexTest );
	}

	if ( filter == NULL || strcmp( filter, "ArrayPopTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ArrayPopTest );
	}

	if ( filter == NULL || strcmp( filter, "ArrayRemoveSwapTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ArrayRemoveSwapTest );
	}

	if ( filter == NULL || strcmp( filter, "ArrayResizeTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ArrayResizeTest );
	}

	if ( filter == NULL || strcmp( filter, "ArrayConstraintTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ArrayConstraintTest );
	}

	if ( filter == NULL || strcmp( filter, "SwapValuesTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( SwapValuesTest );
	}

	if ( filter == NULL || strcmp( filter, "BitSetTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BitSetTest );
	}

	if ( filter == NULL || strcmp( filter, "BitSetGrowTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BitSetGrowTest );
	}

	if ( filter == NULL || strcmp( filter, "IdPoolTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( IdPoolTest );
	}

	if ( filter == NULL || strcmp( filter, "HashSetInsertTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( HashSetInsertTest );
	}

	if ( filter == NULL || strcmp( filter, "HashSetRemoveTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( HashSetRemoveTest );
	}

	if ( filter == NULL || strcmp( filter, "HashSetGrowTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( HashSetGrowTest );
	}

	if ( filter == NULL || strcmp( filter, "StackAllocatorTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( StackAllocatorTest );
	}

	if ( filter == NULL || strcmp( filter, "TimerTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( TimerTest );
	}

	if ( filter == NULL || strcmp( filter, "AABBTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( AABBTest );
	}

	if ( filter == NULL || strcmp( filter, "AABBRayCastTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( AABBRayCastTest );
	}

	if ( filter == NULL || strcmp( filter, "CircleSegmentTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( CircleSegmentTest );
	}

	if ( filter == NULL || strcmp( filter, "CapsuleTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( CapsuleTest );
	}

	if ( filter == NULL || strcmp( filter, "PolygonTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( PolygonTest );
	}

	if ( filter == NULL || strcmp( filter, "OffsetPolygonTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( OffsetPolygonTest );
	}

	if ( filter == NULL || strcmp( filter, "CircleMassTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( CircleMassTest );
	}

	if ( filter == NULL || strcmp( filter, "CapsuleMassTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( CapsuleMassTest );
	}

	if ( filter == NULL || strcmp( filter, "PolygonMassTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( PolygonMassTest );
	}

	if ( filter == NULL || strcmp( filter, "ShapeBoundsTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ShapeBoundsTest );
	}

	if ( filter == NULL || strcmp( filter, "PointInCircleTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( PointInCircleTest );
	}

	if ( filter == NULL || strcmp( filter, "PointInCapsuleTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( PointInCapsuleTest );
	}

	if ( filter == NULL || strcmp( filter, "PointInPolygonTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( PointInPolygonTest );
	}

	if ( filter == NULL || strcmp( filter, "RayCastCircleTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( RayCastCircleTest );
	}

	if ( filter == NULL || strcmp( filter, "RayCastCapsuleTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( RayCastCapsuleTest );
	}

	if ( filter == NULL || strcmp( filter, "RayCastSegmentTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( RayCastSegmentTest );
	}

	if ( filter == NULL || strcmp( filter, "RayCastPolygonTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( RayCastPolygonTest );
	}

	if ( filter == NULL || strcmp( filter, "SegmentDistanceTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( SegmentDistanceTest );
	}

	if ( filter == NULL || strcmp( filter, "ShapeProxyTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ShapeProxyTest );
	}

	if ( filter == NULL || strcmp( filter, "ShapeDistanceTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ShapeDistanceTest );
	}

	if ( filter == NULL || strcmp( filter, "ShapeCastTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ShapeCastTest );
	}

	if ( filter == NULL || strcmp( filter, "ShapeCastDispatchTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ShapeCastDispatchTest );
	}

	if ( filter == NULL || strcmp( filter, "SweepTransformTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( SweepTransformTest );
	}

	if ( filter == NULL || strcmp( filter, "TimeOfImpactTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( TimeOfImpactTest );
	}

	if ( filter == NULL || strcmp( filter, "CircleManifoldTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( CircleManifoldTest );
	}

	if ( filter == NULL || strcmp( filter, "CapsuleCircleManifoldTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( CapsuleCircleManifoldTest );
	}

	if ( filter == NULL || strcmp( filter, "PolygonCircleManifoldTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( PolygonCircleManifoldTest );
	}

	if ( filter == NULL || strcmp( filter, "CapsuleManifoldTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( CapsuleManifoldTest );
	}

	if ( filter == NULL || strcmp( filter, "SegmentCircleManifoldTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( SegmentCircleManifoldTest );
	}

	if ( filter == NULL || strcmp( filter, "SegmentCapsuleManifoldTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( SegmentCapsuleManifoldTest );
	}

	if ( filter == NULL || strcmp( filter, "PolygonCapsuleManifoldTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( PolygonCapsuleManifoldTest );
	}

	if ( filter == NULL || strcmp( filter, "PolygonManifoldTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( PolygonManifoldTest );
	}

	if ( filter == NULL || strcmp( filter, "SegmentPolygonManifoldTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( SegmentPolygonManifoldTest );
	}

	if ( filter == NULL || strcmp( filter, "ChainCircleManifoldTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ChainCircleManifoldTest );
	}

	if ( filter == NULL || strcmp( filter, "ChainCapsuleManifoldTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ChainCapsuleManifoldTest );
	}

	if ( filter == NULL || strcmp( filter, "HullTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( HullTest );
	}

	if ( ran == 0 )
	{
		printf( "no tests matched\n" );
		return 1;
	}

	printf( "All tests passed\n" );
	return 0;
}

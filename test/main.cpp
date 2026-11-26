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
int ChainPolygonManifoldTest( void );
int ChainBuildTest( void );
int DynamicTreeCreateTest( void );
int DynamicTreeInsertTest( void );
int DynamicTreeRemoveTest( void );
int DynamicTreeMoveTest( void );
int DynamicTreeCategoryTest( void );
int DynamicTreeRebuildTest( void );
int DynamicTreeRefitTest( void );
int DynamicTreeOverlapTest( void );
int DynamicTreeRayTest( void );
int DynamicTreeBoxCastTest( void );
int DynamicTreeStatsTest( void );
int WorldCreateTest( void );
int WorldIdTest( void );
int WorldUserDataTest( void );
int WorldCountersTest( void );
int WorldProfileMemoryTest( void );
int BodyMoveEventsTest( void );
int BodyCreateTest( void );
int BodyIdTest( void );
int BodyNameTest( void );
int BodyTransformQueryTest( void );
int BodySetTransformTest( void );
int BodyBoundsTest( void );
int FreeBodyStepTest( void );
int SolverSetOrganizationTest( void );
int IslandSleepTest( void );
int IslandWakeTest( void );
int SleepControlTest( void );
int ContactIslandTest( void );
int ConstraintGraphTest( void );
int ContactPrepareTest( void );
int ContactNormalSolveTest( void );
int ContactImpulseStoreTest( void );
int ContactSubstepCorrectionTest( void );
int ContactWarmStartTest( void );
int ContactTuningTest( void );
int ContactRecycleTest( void );
int ContactTouchEventsTest( void );
int ContactHitEventsTest( void );
int CustomCollisionFilterTest( void );
int PreSolveCallbackTest( void );
int WorldOverlapAABBTest( void );
int WorldOverlapShapeTest( void );
int WorldRayClosestTest( void );
int WorldShapeCastTest( void );
int AttachedShapePointTest( void );
int AttachedShapeRayTest( void );
int ShapeClosestPointTest( void );
int WorldExplosionTest( void );
int ShapeWindTest( void );
int MoverPlanesTest( void );
int MoverClipVectorTest( void );
int WorldMoverCastTest( void );
int WorldMoverPlanesTest( void );
int WorldBoundsTest( void );
int WorldDebugDrawTest( void );
int WorldStaticTreeRebuildTest( void );
int JointLifecycleTest( void );
int JointDefinitionTest( void );
int JointOrganizationTest( void );
int DistanceJointBasicTest( void );
int DistanceJointSpringTest( void );
int DistanceJointLimitTest( void );
int DistanceJointMotorTest( void );
int RevoluteJointBasicTest( void );
int RevoluteJointSpringTest( void );
int RevoluteJointLimitTest( void );
int RevoluteJointMotorTest( void );
int PrismaticJointBasicTest( void );
int PrismaticJointSpringTest( void );
int PrismaticJointLimitTest( void );
int PrismaticJointMotorTest( void );
int WheelJointSuspensionTest( void );
int WheelJointLimitTest( void );
int WheelJointMotorTest( void );
int WeldJointTest( void );
int MotorJointVelocityTest( void );
int MotorJointSpringTest( void );
int MoverJointTest( void );
int PogoJointTest( void );
int FilterJointTest( void );
int JointCollideConnectedTest( void );
int JointLocalFrameTest( void );
int JointConstraintTuningTest( void );
int JointReactionTest( void );
int JointEventTest( void );
int JointEventOverflowTest( void );
int JointDrawTest( void );
int WorldRayCastTest( void );
int SensorOverlapEventsTest( void );
int SensorEventOptionsDataTest( void );
int ContactFrictionTest( void );
int ContactRollingResistanceTest( void );
int ContactTangentSpeedTest( void );
int ContactRestitutionTest( void );
int ContactRestitutionThresholdTest( void );
int ContactRestitutionControlTest( void );
int ContactMaterialCallbackTest( void );
int SpeculativeContactTest( void );
int ContinuousCollisionTest( void );
int WorldSolverSwitchTest( void );
int BulletCollisionTest( void );
int ContactLifecycleTest( void );
int ContactDataQueryTest( void );
int ContactPersistenceTest( void );
int ContactManifoldUpdateTest( void );
int BodyTypeChangeTest( void );
int BodyMotionLocksTest( void );
int BodyEnableDisableTest( void );
int BodySafetyAndSpeedTest( void );
int WorldGravityControlTest( void );
int BodyTypesTest( void );
int BodyTargetTransformTest( void );
int BodyAngularImpulseTest( void );
int BodyLinearImpulseTest( void );
int BodyTorqueTest( void );
int BodyForceTest( void );
int BodyDampingTest( void );
int BodyGravityTest( void );
int BodyVelocityTest( void );
int BroadPhaseTreeTypesTest( void );
int BroadPhaseFilterTest( void );
int BroadPhaseSortTest( void );
int BroadPhaseExistingPairsTest( void );
int BroadPhaseCandidatesTest( void );
int BroadPhaseMoveTest( void );
int BroadPhaseProxyLifecycleTest( void );
int ObjectRelationshipTest( void );
int ChainMaterialTest( void );
int ChainEnumerationTest( void );
int ChainLifecycleTest( void );
int BodyExplicitMassTest( void );
int BodyMassFromDensityTest( void );
int CircleShapeLifecycleTest( void );
int ShapeGeometryEditTest( void );
int ShapeUserDataTest( void );
int SegmentShapeLifecycleTest( void );
int PolygonShapeLifecycleTest( void );
int CapsuleShapeLifecycleTest( void );
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

	if ( filter == NULL || strcmp( filter, "ChainPolygonManifoldTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ChainPolygonManifoldTest );
	}

	if ( filter == NULL || strcmp( filter, "ChainBuildTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ChainBuildTest );
	}

	if ( filter == NULL || strcmp( filter, "DynamicTreeCreateTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( DynamicTreeCreateTest );
	}

	if ( filter == NULL || strcmp( filter, "DynamicTreeInsertTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( DynamicTreeInsertTest );
	}

	if ( filter == NULL || strcmp( filter, "DynamicTreeRemoveTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( DynamicTreeRemoveTest );
	}

	if ( filter == NULL || strcmp( filter, "DynamicTreeMoveTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( DynamicTreeMoveTest );
	}

	if ( filter == NULL || strcmp( filter, "DynamicTreeCategoryTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( DynamicTreeCategoryTest );
	}

	if ( filter == NULL || strcmp( filter, "DynamicTreeRebuildTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( DynamicTreeRebuildTest );
	}

	if ( filter == NULL || strcmp( filter, "DynamicTreeRefitTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( DynamicTreeRefitTest );
	}

	if ( filter == NULL || strcmp( filter, "DynamicTreeOverlapTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( DynamicTreeOverlapTest );
	}

	if ( filter == NULL || strcmp( filter, "DynamicTreeRayTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( DynamicTreeRayTest );
	}

	if ( filter == NULL || strcmp( filter, "DynamicTreeBoxCastTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( DynamicTreeBoxCastTest );
	}

	if ( filter == NULL || strcmp( filter, "DynamicTreeStatsTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( DynamicTreeStatsTest );
	}

	if ( filter == NULL || strcmp( filter, "WorldCreateTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( WorldCreateTest );
	}

	if ( filter == NULL || strcmp( filter, "WorldIdTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( WorldIdTest );
	}

	if ( filter == NULL || strcmp( filter, "WorldUserDataTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( WorldUserDataTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyCreateTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyCreateTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyIdTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyIdTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyNameTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyNameTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyTransformQueryTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyTransformQueryTest );
	}

	if ( filter == NULL || strcmp( filter, "BodySetTransformTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodySetTransformTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyBoundsTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyBoundsTest );
	}

	if ( filter == NULL || strcmp( filter, "CircleShapeLifecycleTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( CircleShapeLifecycleTest );
	}

	if ( filter == NULL || strcmp( filter, "CapsuleShapeLifecycleTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( CapsuleShapeLifecycleTest );
	}

	if ( filter == NULL || strcmp( filter, "PolygonShapeLifecycleTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( PolygonShapeLifecycleTest );
	}

	if ( filter == NULL || strcmp( filter, "SegmentShapeLifecycleTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( SegmentShapeLifecycleTest );
	}

	if ( filter == NULL || strcmp( filter, "ShapeUserDataTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ShapeUserDataTest );
	}

	if ( filter == NULL || strcmp( filter, "ShapeGeometryEditTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ShapeGeometryEditTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyMassFromDensityTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyMassFromDensityTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyExplicitMassTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyExplicitMassTest );
	}

	if ( filter == NULL || strcmp( filter, "ChainLifecycleTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ChainLifecycleTest );
	}

	if ( filter == NULL || strcmp( filter, "ChainEnumerationTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ChainEnumerationTest );
	}

	if ( filter == NULL || strcmp( filter, "ChainMaterialTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ChainMaterialTest );
	}

	if ( filter == NULL || strcmp( filter, "ObjectRelationshipTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ObjectRelationshipTest );
	}

	if ( filter == NULL || strcmp( filter, "BroadPhaseTreeTypesTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BroadPhaseTreeTypesTest );
	}

	if ( filter == NULL || strcmp( filter, "BroadPhaseProxyLifecycleTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BroadPhaseProxyLifecycleTest );
	}

	if ( filter == NULL || strcmp( filter, "BroadPhaseMoveTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BroadPhaseMoveTest );
	}

	if ( filter == NULL || strcmp( filter, "BroadPhaseCandidatesTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BroadPhaseCandidatesTest );
	}

	if ( filter == NULL || strcmp( filter, "BroadPhaseExistingPairsTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BroadPhaseExistingPairsTest );
	}

	if ( filter == NULL || strcmp( filter, "BroadPhaseSortTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BroadPhaseSortTest );
	}

	if ( filter == NULL || strcmp( filter, "BroadPhaseFilterTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BroadPhaseFilterTest );
	}

	if ( filter == NULL || strcmp( filter, "FreeBodyStepTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( FreeBodyStepTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyVelocityTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyVelocityTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyGravityTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyGravityTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyDampingTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyDampingTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyForceTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyForceTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyTorqueTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyTorqueTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyLinearImpulseTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyLinearImpulseTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyAngularImpulseTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyAngularImpulseTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyTargetTransformTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyTargetTransformTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyTypesTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyTypesTest );
	}

	if ( filter == NULL || strcmp( filter, "BodyTypeChangeTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( BodyTypeChangeTest );
	}

	if ( filter == NULL || strcmp( filter, "ContactLifecycleTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ContactLifecycleTest );
	}

	if ( filter == NULL || strcmp( filter, "ContactManifoldUpdateTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ContactManifoldUpdateTest );
	}

	if ( filter == NULL || strcmp( filter, "ContactPersistenceTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ContactPersistenceTest );
	}

	if ( filter == NULL || strcmp( filter, "ContactDataQueryTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( ContactDataQueryTest );
	}

	if ( filter == NULL || strcmp( filter, "SolverSetOrganizationTest" ) == 0 )
	{
		ran = 1;
		RUN_TEST( SolverSetOrganizationTest );
	}

if(filter==NULL || strcmp(filter,"ContactIslandTest")==0) { ran=1; RUN_TEST(ContactIslandTest); }
	if(filter==NULL || strcmp(filter,"ConstraintGraphTest")==0) { ran=1; RUN_TEST(ConstraintGraphTest); }
	if(filter==NULL || strcmp(filter,"ContactPrepareTest")==0) { ran=1; RUN_TEST(ContactPrepareTest); }
	if(filter==NULL || strcmp(filter,"ContactNormalSolveTest")==0) { ran=1; RUN_TEST(ContactNormalSolveTest); }
	if(filter==NULL || strcmp(filter,"ContactImpulseStoreTest")==0) { ran=1; RUN_TEST(ContactImpulseStoreTest); }
	if(filter==NULL || strcmp(filter,"ContactSubstepCorrectionTest")==0) { ran=1; RUN_TEST(ContactSubstepCorrectionTest); }
	if(filter==NULL || strcmp(filter,"ContactWarmStartTest")==0) { ran=1; RUN_TEST(ContactWarmStartTest); }
	if(filter==NULL || strcmp(filter,"ContactTuningTest")==0) { ran=1; RUN_TEST(ContactTuningTest); }
	if(filter==NULL || strcmp(filter,"ContactRecycleTest")==0) { ran=1; RUN_TEST(ContactRecycleTest); }
	if(filter==NULL || strcmp(filter,"ContactFrictionTest")==0) { ran=1; RUN_TEST(ContactFrictionTest); }
	if(filter==NULL || strcmp(filter,"ContactRollingResistanceTest")==0) { ran=1; RUN_TEST(ContactRollingResistanceTest); }
	if(filter==NULL || strcmp(filter,"ContactTangentSpeedTest")==0) { ran=1; RUN_TEST(ContactTangentSpeedTest); }
	if(filter==NULL || strcmp(filter,"ContactRestitutionTest")==0) { ran=1; RUN_TEST(ContactRestitutionTest); }
	if(filter==NULL || strcmp(filter,"ContactRestitutionThresholdTest")==0) { ran=1; RUN_TEST(ContactRestitutionThresholdTest); }
	if(filter==NULL || strcmp(filter,"ContactRestitutionControlTest")==0) { ran=1; RUN_TEST(ContactRestitutionControlTest); }
	if(filter==NULL || strcmp(filter,"ContactMaterialCallbackTest")==0) { ran=1; RUN_TEST(ContactMaterialCallbackTest); }
	if(filter==NULL || strcmp(filter,"BodyMotionLocksTest")==0) { ran=1; RUN_TEST(BodyMotionLocksTest); }
	if(filter==NULL || strcmp(filter,"BodyEnableDisableTest")==0) { ran=1; RUN_TEST(BodyEnableDisableTest); }
	if(filter==NULL || strcmp(filter,"IslandSleepTest")==0) { ran=1; RUN_TEST(IslandSleepTest); }
	if(filter==NULL || strcmp(filter,"IslandWakeTest")==0) { ran=1; RUN_TEST(IslandWakeTest); }
	if(filter==NULL || strcmp(filter,"SleepControlTest")==0) { ran=1; RUN_TEST(SleepControlTest); }
	if(filter==NULL || strcmp(filter,"SpeculativeContactTest")==0) { ran=1; RUN_TEST(SpeculativeContactTest); }
	if(filter==NULL || strcmp(filter,"ContinuousCollisionTest")==0) { ran=1; RUN_TEST(ContinuousCollisionTest); }
	if(filter==NULL || strcmp(filter,"BulletCollisionTest")==0) { ran=1; RUN_TEST(BulletCollisionTest); }
	if(filter==NULL || strcmp(filter,"BodySafetyAndSpeedTest")==0) { ran=1; RUN_TEST(BodySafetyAndSpeedTest); }
	if(filter==NULL || strcmp(filter,"WorldGravityControlTest")==0) { ran=1; RUN_TEST(WorldGravityControlTest); }
	if(filter==NULL || strcmp(filter,"WorldSolverSwitchTest")==0) { ran=1; RUN_TEST(WorldSolverSwitchTest); }
	if(filter==NULL || strcmp(filter,"WorldCountersTest")==0) { ran=1; RUN_TEST(WorldCountersTest); }
	if(filter==NULL || strcmp(filter,"WorldProfileMemoryTest")==0) { ran=1; RUN_TEST(WorldProfileMemoryTest); }
	if(filter==NULL || strcmp(filter,"BodyMoveEventsTest")==0) { ran=1; RUN_TEST(BodyMoveEventsTest); }
	if(filter==NULL || strcmp(filter,"ContactTouchEventsTest")==0) { ran=1; RUN_TEST(ContactTouchEventsTest); }
	if(filter==NULL || strcmp(filter,"ContactHitEventsTest")==0) { ran=1; RUN_TEST(ContactHitEventsTest); }
	if(filter==NULL || strcmp(filter,"SensorOverlapEventsTest")==0) { ran=1; RUN_TEST(SensorOverlapEventsTest); }
	if(filter==NULL || strcmp(filter,"SensorEventOptionsDataTest")==0) { ran=1; RUN_TEST(SensorEventOptionsDataTest); }
	if(filter==NULL || strcmp(filter,"CustomCollisionFilterTest")==0) { ran=1; RUN_TEST(CustomCollisionFilterTest); }
	if(filter==NULL || strcmp(filter,"PreSolveCallbackTest")==0) { ran=1; RUN_TEST(PreSolveCallbackTest); }
	if(filter==NULL || strcmp(filter,"WorldOverlapAABBTest")==0) { ran=1; RUN_TEST(WorldOverlapAABBTest); }
	if(filter==NULL || strcmp(filter,"WorldOverlapShapeTest")==0) { ran=1; RUN_TEST(WorldOverlapShapeTest); }
	if(filter==NULL || strcmp(filter,"WorldRayCastTest")==0) { ran=1; RUN_TEST(WorldRayCastTest); }
	if(filter==NULL || strcmp(filter,"WorldRayClosestTest")==0) { ran=1; RUN_TEST(WorldRayClosestTest); }

	if(filter==NULL || strcmp(filter,"WorldShapeCastTest")==0) { ran=1; RUN_TEST(WorldShapeCastTest); }

	if(filter==NULL || strcmp(filter,"AttachedShapePointTest")==0) { ran=1; RUN_TEST(AttachedShapePointTest); }

	if(filter==NULL || strcmp(filter,"AttachedShapeRayTest")==0) { ran=1; RUN_TEST(AttachedShapeRayTest); }

	if(filter==NULL || strcmp(filter,"ShapeClosestPointTest")==0) { ran=1; RUN_TEST(ShapeClosestPointTest); }

	if(filter==NULL || strcmp(filter,"WorldExplosionTest")==0) { ran=1; RUN_TEST(WorldExplosionTest); }

	if(filter==NULL || strcmp(filter,"ShapeWindTest")==0) { ran=1; RUN_TEST(ShapeWindTest); }

	if(filter==NULL || strcmp(filter,"MoverPlanesTest")==0) { ran=1; RUN_TEST(MoverPlanesTest); }

	if(filter==NULL || strcmp(filter,"MoverClipVectorTest")==0) { ran=1; RUN_TEST(MoverClipVectorTest); }

	if(filter==NULL || strcmp(filter,"WorldMoverCastTest")==0) { ran=1; RUN_TEST(WorldMoverCastTest); }

	if(filter==NULL || strcmp(filter,"WorldMoverPlanesTest")==0) { ran=1; RUN_TEST(WorldMoverPlanesTest); }

	if(filter==NULL || strcmp(filter,"WorldBoundsTest")==0) { ran=1; RUN_TEST(WorldBoundsTest); }

	if(filter==NULL || strcmp(filter,"WorldDebugDrawTest")==0) { ran=1; RUN_TEST(WorldDebugDrawTest); }

	if(filter==NULL || strcmp(filter,"WorldStaticTreeRebuildTest")==0) { ran=1; RUN_TEST(WorldStaticTreeRebuildTest); }

	if(filter==NULL || strcmp(filter,"JointLifecycleTest")==0) { ran=1; RUN_TEST(JointLifecycleTest); }

	if(filter==NULL || strcmp(filter,"JointDefinitionTest")==0) { ran=1; RUN_TEST(JointDefinitionTest); }

	if(filter==NULL || strcmp(filter,"JointOrganizationTest")==0) { ran=1; RUN_TEST(JointOrganizationTest); }

	if(filter==NULL || strcmp(filter,"DistanceJointBasicTest")==0) { ran=1; RUN_TEST(DistanceJointBasicTest); }

	if(filter==NULL || strcmp(filter,"DistanceJointSpringTest")==0) { ran=1; RUN_TEST(DistanceJointSpringTest); }

	if(filter==NULL || strcmp(filter,"DistanceJointLimitTest")==0) { ran=1; RUN_TEST(DistanceJointLimitTest); }

	if(filter==NULL || strcmp(filter,"DistanceJointMotorTest")==0) { ran=1; RUN_TEST(DistanceJointMotorTest); }

	if(filter==NULL || strcmp(filter,"RevoluteJointBasicTest")==0) { ran=1; RUN_TEST(RevoluteJointBasicTest); }

	if(filter==NULL || strcmp(filter,"RevoluteJointSpringTest")==0) { ran=1; RUN_TEST(RevoluteJointSpringTest); }

	if(filter==NULL || strcmp(filter,"RevoluteJointLimitTest")==0) { ran=1; RUN_TEST(RevoluteJointLimitTest); }

	if(filter==NULL || strcmp(filter,"RevoluteJointMotorTest")==0) { ran=1; RUN_TEST(RevoluteJointMotorTest); }

	if(filter==NULL || strcmp(filter,"PrismaticJointBasicTest")==0) { ran=1; RUN_TEST(PrismaticJointBasicTest); }

	if(filter==NULL || strcmp(filter,"PrismaticJointSpringTest")==0) { ran=1; RUN_TEST(PrismaticJointSpringTest); }

	if(filter==NULL || strcmp(filter,"PrismaticJointLimitTest")==0) { ran=1; RUN_TEST(PrismaticJointLimitTest); }

	if(filter==NULL || strcmp(filter,"PrismaticJointMotorTest")==0) { ran=1; RUN_TEST(PrismaticJointMotorTest); }

	if(filter==NULL || strcmp(filter,"WheelJointSuspensionTest")==0) { ran=1; RUN_TEST(WheelJointSuspensionTest); }

	if(filter==NULL || strcmp(filter,"WheelJointLimitTest")==0) { ran=1; RUN_TEST(WheelJointLimitTest); }

	if(filter==NULL || strcmp(filter,"WheelJointMotorTest")==0) { ran=1; RUN_TEST(WheelJointMotorTest); }

	if(filter==NULL || strcmp(filter,"WeldJointTest")==0) { ran=1; RUN_TEST(WeldJointTest); }

	if(filter==NULL || strcmp(filter,"MotorJointVelocityTest")==0) { ran=1; RUN_TEST(MotorJointVelocityTest); }

	if(filter==NULL || strcmp(filter,"MotorJointSpringTest")==0) { ran=1; RUN_TEST(MotorJointSpringTest); }

	if(filter==NULL || strcmp(filter,"MoverJointTest")==0) { ran=1; RUN_TEST(MoverJointTest); }

	if(filter==NULL || strcmp(filter,"PogoJointTest")==0) { ran=1; RUN_TEST(PogoJointTest); }

	if(filter==NULL || strcmp(filter,"FilterJointTest")==0) { ran=1; RUN_TEST(FilterJointTest); }

	if(filter==NULL || strcmp(filter,"JointCollideConnectedTest")==0) { ran=1; RUN_TEST(JointCollideConnectedTest); }

	if(filter==NULL || strcmp(filter,"JointLocalFrameTest")==0) { ran=1; RUN_TEST(JointLocalFrameTest); }

	if(filter==NULL || strcmp(filter,"JointConstraintTuningTest")==0) { ran=1; RUN_TEST(JointConstraintTuningTest); }

	if(filter==NULL || strcmp(filter,"JointReactionTest")==0) { ran=1; RUN_TEST(JointReactionTest); }

	if(filter==NULL || strcmp(filter,"JointEventTest")==0) { ran=1; RUN_TEST(JointEventTest); }

	if(filter==NULL || strcmp(filter,"JointEventOverflowTest")==0) { ran=1; RUN_TEST(JointEventOverflowTest); }

	if(filter==NULL || strcmp(filter,"JointDrawTest")==0) { ran=1; RUN_TEST(JointDrawTest); }

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

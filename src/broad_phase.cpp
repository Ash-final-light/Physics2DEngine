// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "broad_phase.h"
#include "core.h"
#include <cstring>

void b2CreateBroadPhase( b2BroadPhase* bp, const b2Capacity* capacity )
{
	static_assert( b2_bodyTypeCount == 3, "must be three body types" );

	bp->pairSet = b2CreateSet( b2MaxInt( 32, 2 * capacity->contactCount ) );

	int staticCapacity = b2MaxInt( 16, capacity->staticShapeCount );
	b2CreateDynamicTreeInternal( bp->trees + b2_staticBody, staticCapacity );

	int kinematicCapacity = 16;
	b2CreateDynamicTreeInternal( bp->trees + b2_kinematicBody, kinematicCapacity );

	int dynamicCapacity = b2MaxInt( 16, capacity->dynamicShapeCount );
	b2CreateDynamicTreeInternal( bp->trees + b2_dynamicBody, dynamicCapacity );

	bp->movedSiblings = NULL;
	bp->crossContext = NULL;
}

void b2DestroyBroadPhase( b2BroadPhase* bp )
{
	for ( int i = 0; i < b2_bodyTypeCount; ++i )
	{
		b2DestroyDynamicTreeInternal( bp->trees + i );
	}

	b2DestroySet( &bp->pairSet );

	memset( bp, 0, sizeof( b2BroadPhase ) );
}

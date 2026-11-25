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


int b2BroadPhase_CreateProxy( b2BroadPhase* bp, b2BodyType proxyType, b2AABB aabb, uint64_t categoryBits, int shapeIndex,
							  bool forcePairCreation )
{
	B2_ASSERT( 0 <= proxyType && proxyType < b2_bodyTypeCount );

	bool mark = ( proxyType != b2_staticBody || forcePairCreation );

	int proxyId = b2CreateTreeProxyInternal( bp->trees + proxyType, aabb, categoryBits, shapeIndex, mark );
	int proxyKey = B2_PROXY_KEY( proxyId, proxyType );
	return proxyKey;
}

void b2BroadPhase_DestroyProxy( b2BroadPhase* bp, int proxyKey )
{
	b2BodyType proxyType = B2_PROXY_TYPE( proxyKey );
	int proxyId = B2_PROXY_ID( proxyKey );

	B2_ASSERT( 0 <= proxyType && proxyType < b2_bodyTypeCount );
	b2DestroyTreeProxy( bp->trees + proxyType, proxyId );
}


void b2BroadPhase_MoveProxy( b2BroadPhase* bp, int proxyKey, b2AABB aabb )
{
    b2BodyType type = B2_PROXY_TYPE( proxyKey );
    int proxyId = B2_PROXY_ID( proxyKey );
    b2DynamicTree_MoveProxyInternal( bp->trees+type, proxyId, aabb, true );
}

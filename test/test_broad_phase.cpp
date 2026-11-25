// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "world.h"
#include "box2d/types.h"

int BroadPhaseTreeTypesTest( void )
{
    size_t before = b2GetByteCount();
    b2WorldDef def = b2DefaultWorldDef();
    def.capacity.staticShapeCount = 128;
    def.capacity.dynamicShapeCount = 256;
    b2WorldId id = b2CreateWorld( &def );
    b2BroadPhase* bp = &b2_worlds[id.index1-1].broadPhase;
    ENSURE( bp->trees[b2_staticBody].proxyCapacity >= 128 );
    ENSURE( bp->trees[b2_dynamicBody].proxyCapacity >= 256 );
    ENSURE( bp->trees[b2_kinematicBody].proxyCapacity >= 16 );
    for ( int i = 0; i < b2_bodyTypeCount; ++i )
    {
        ENSURE( b2DynamicTree_GetProxyCount( bp->trees+i ) == 0 );
        ENSURE( bp->trees[i].nodes != bp->trees[(i+1)%b2_bodyTypeCount].nodes );
    }
    b2WorldId other = b2CreateWorld( &def );
    ENSURE( bp->trees[0].nodes != b2_worlds[other.index1-1].broadPhase.trees[0].nodes );
    b2DestroyWorld( id );
    b2DestroyWorld( other );
    ENSURE( b2GetByteCount() == before );
    return 0;
}

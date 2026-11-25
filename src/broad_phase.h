// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "dynamic_tree.h"
#include "table.h"
#include "box2d/types.h"

#define B2_PROXY_TYPE( KEY ) ( (b2BodyType)( ( KEY ) & 3 ) )
#define B2_PROXY_ID( KEY ) ( ( KEY ) >> 2 )
#define B2_PROXY_KEY( ID, TYPE ) ( ( ( ID ) << 2 ) | ( TYPE ) )

struct b2BroadPhase
{
    b2DynamicTree trees[b2_bodyTypeCount];
    int* movedSiblings;
    struct b2CrossContext* crossContext;
    b2HashSet pairSet;
};

void b2CreateBroadPhase( b2BroadPhase* bp, const b2Capacity* capacity );
void b2DestroyBroadPhase( b2BroadPhase* bp );


int b2BroadPhase_CreateProxy( b2BroadPhase* bp, b2BodyType type, b2AABB aabb, uint64_t categoryBits, int shapeIndex, bool forcePairCreation );
void b2BroadPhase_DestroyProxy( b2BroadPhase* bp, int proxyKey );

void b2BroadPhase_MoveProxy( b2BroadPhase* bp, int proxyKey, b2AABB aabb );

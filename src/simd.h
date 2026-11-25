// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "dynamic_tree.h"

// Scalar path used until vector implementations are introduced.
typedef b2AABB b2AABBV;

B2_FORCE_INLINE b2AABBV b2LoadAABBV( const b2AABB* aabb )
{
	return *aabb;
}

B2_FORCE_INLINE bool b2OverlapNode( b2AABBV av, const b2TreeNode* node )
{
	const b2AABB* bv = &node->aabb;
	return av.lowerBound.x <= bv->upperBound.x && av.lowerBound.y <= bv->upperBound.y && bv->lowerBound.x <= av.upperBound.x &&
		   bv->lowerBound.y <= av.upperBound.y;
}

B2_FORCE_INLINE bool b2OverlapV( const b2AABB* a, const b2AABB* b )
{
	return a->lowerBound.x <= b->upperBound.x && a->lowerBound.y <= b->upperBound.y && b->lowerBound.x <= a->upperBound.x &&
		   b->lowerBound.y <= a->upperBound.y;
}

B2_FORCE_INLINE int b2OverlapPairMask( const b2TreeNode* pairA, const b2TreeNode* pairB )
{
	int mask = 0;
	mask |= b2OverlapV( &pairA[0].aabb, &pairB[0].aabb ) ? 1 : 0;
	mask |= b2OverlapV( &pairA[0].aabb, &pairB[1].aabb ) ? 2 : 0;
	mask |= b2OverlapV( &pairA[1].aabb, &pairB[0].aabb ) ? 4 : 0;
	mask |= b2OverlapV( &pairA[1].aabb, &pairB[1].aabb ) ? 8 : 0;
	return mask;
}

B2_FORCE_INLINE int b2OverlapNodePairMask( b2AABBV av, const b2TreeNode* pair )
{
	int mask = 0;
	mask |= b2OverlapNode( av, pair + 0 ) ? 1 : 0;
	mask |= b2OverlapNode( av, pair + 1 ) ? 2 : 0;
	return mask;
}

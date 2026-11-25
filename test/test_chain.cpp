// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

static float LoopArea( const b2ChainData* chain )
{
	float area = 0.0f;
	for ( int i = 0; i < chain->count; ++i )
	{
		b2Vec2 p1 = chain->points[i];
		b2Vec2 p2 = chain->points[i + 1 < chain->count ? i + 1 : 0];
		area += 0.5f * b2Cross( p1, p2 );
	}
	return area;
}

int ChainBuildTest( void )
{
	b2Polygon boxes[8];
	for ( int i = 0; i < 3; ++i )
	{
		float noise = 0.0001f * (float)( i - 1 );
		boxes[i] = b2MakeOffsetBox( 0.5f, 0.5f, { 1.0f * i + noise, noise }, b2Rot_identity );
	}

	b2ChainResult result = b2ComputeChains( boxes, 3, B2_LINEAR_SLOP );
	ENSURE( result.status == b2_chainStatusSuccess );
	ENSURE( result.chainCount == 1 );
	ENSURE( result.chains[0].count == 4 );
	ENSURE_SMALL( LoopArea( result.chains ) - 3.0f, 0.01f );
	b2FreeChains( &result );
	ENSURE( result.chains == NULL && result.chainCount == 0 );

	b2Polygon tee[3] = {
		b2MakeOffsetBox( 1.0f, 0.5f, { 1.0f, 0.5f }, b2Rot_identity ),
		b2MakeOffsetBox( 0.5f, 0.5f, { 0.5f, 1.5f }, b2Rot_identity ),
		b2MakeOffsetBox( 0.5f, 0.5f, { 1.5f, 1.5f }, b2Rot_identity ),
	};
	result = b2ComputeChains( tee, 3, B2_LINEAR_SLOP );
	ENSURE( result.status == b2_chainStatusSuccess );
	ENSURE( result.chainCount == 1 );
	ENSURE( result.chains[0].count == 4 );
	ENSURE_SMALL( LoopArea( result.chains ) - 4.0f, 0.01f );
	b2FreeChains( &result );

	b2Polygon corners[2] = {
		b2MakeOffsetBox( 0.5f, 0.5f, { 0.5f, 0.5f }, b2Rot_identity ),
		b2MakeOffsetBox( 0.5f, 0.5f, { 1.5f, 1.5f }, b2Rot_identity ),
	};
	result = b2ComputeChains( corners, 2, B2_LINEAR_SLOP );
	ENSURE( result.status == b2_chainStatusCornerTouch );
	ENSURE( result.chainCount == 1 );
	ENSURE( result.chains[0].count == 8 );
	ENSURE_SMALL( LoopArea( result.chains ) - 2.0f, 0.01f );
	b2Vec2 shared = { 1.0f, 1.0f };
	int visits = 0;
	for ( int i = 0; i < result.chains[0].count; ++i )
	{
		if ( b2DistanceSquared( result.chains[0].points[i], shared ) < 1.0e-6f )
		{
			b2Vec2 previous = result.chains[0].points[i > 0 ? i - 1 : result.chains[0].count - 1];
			b2Vec2 next = result.chains[0].points[i + 1 < result.chains[0].count ? i + 1 : 0];
			ENSURE( b2Cross( b2Sub( shared, previous ), b2Sub( next, shared ) ) < 0.0f );
			visits += 1;
		}
	}
	ENSURE( visits == 2 );
	b2FreeChains( &result );

	int count = 0;
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			if ( i == 1 && j == 1 )
			{
				continue;
			}
			boxes[count++] = b2MakeOffsetBox( 0.5f, 0.5f, { 1.0f * i, 1.0f * j }, b2Rot_identity );
		}
	}
	result = b2ComputeChains( boxes, count, B2_LINEAR_SLOP );
	ENSURE( result.status == b2_chainStatusSuccess );
	ENSURE( result.chainCount == 2 );
	ENSURE( result.chains[0].count == 4 && result.chains[1].count == 4 );
	float areas[2] = { LoopArea( result.chains + 0 ), LoopArea( result.chains + 1 ) };
	float outer = b2MaxFloat( areas[0], areas[1] );
	float inner = b2MinFloat( areas[0], areas[1] );
	ENSURE_SMALL( outer - 9.0f, 0.01f );
	ENSURE_SMALL( inner + 1.0f, 0.01f );
	b2FreeChains( &result );

	b2Polygon gap[2] = {
		b2MakeOffsetBox( 0.5f, 0.5f, { 0.0f, 0.0f }, b2Rot_identity ),
		b2MakeOffsetBox( 0.5f, 0.5f, { 1.05f, 0.0f }, b2Rot_identity ),
	};
	result = b2ComputeChains( gap, 2, B2_LINEAR_SLOP );
	ENSURE( result.status == b2_chainStatusSuccess );
	ENSURE( result.chainCount == 2 );
	b2FreeChains( &result );
	result = b2ComputeChains( gap, 2, 0.1f );
	ENSURE( result.status == b2_chainStatusSuccess );
	ENSURE( result.chainCount == 1 );
	ENSURE( result.chains[0].count == 4 );
	b2FreeChains( &result );

	b2Polygon overlap[2] = { b2MakeBox( 0.5f, 0.5f ), b2MakeBox( 0.5f, 0.5f ) };
	result = b2ComputeChains( overlap, 2, B2_LINEAR_SLOP );
	ENSURE( result.status == b2_chainStatusOverlap );
	b2FreeChains( &result );

	result = b2ComputeChains( NULL, 0, B2_LINEAR_SLOP );
	ENSURE( result.chains == NULL && result.chainCount == 0 && result.status == b2_chainStatusSuccess );
	b2FreeChains( &result );
	return 0;
}

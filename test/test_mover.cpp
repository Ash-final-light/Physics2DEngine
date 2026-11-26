// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>

static b2CollisionPlane MakePlane( b2Vec2 normal, float pushLimit )
{
	return b2CollisionPlane{
		.plane = { normal, 0.0f },
		.pushLimit = pushLimit,
		.push = 0.0f,
		.clipVelocity = true,
	};
}

static int NoPlanesTest( void )
{
	b2Vec2 targetDelta = { 0.5f, -0.25f };
	b2PlaneSolverResult result = b2SolvePlanes( targetDelta, NULL, 0 );

	ENSURE( result.delta.x == targetDelta.x );
	ENSURE( result.delta.y == targetDelta.y );
	ENSURE( result.iterationCount == 0 );

	return 0;
}

static int SolvePlaneTest( void )
{
	b2CollisionPlane plane = MakePlane( b2Vec2{ 0.0f, 1.0f }, FLT_MAX );
	b2PlaneSolverResult result = b2SolvePlanes( b2Vec2{ 0.0f, -1.0f }, &plane, 1 );

	// 过冲会把移动体从地板另一侧弹出去
	ENSURE( result.delta.y <= 0.0f );

	// 移动体停在表面下方一个容差处，而不是穿过去
	ENSURE( result.delta.y > -10.0f * B2_LINEAR_SLOP );
	ENSURE( result.delta.x == 0.0f );
	ENSURE( plane.push > 0.0f );

	return 0;
}

static int PushLimitTest( void )
{
	b2CollisionPlane plane = MakePlane( b2Vec2{ 0.0f, 1.0f }, 0.1f );
	b2PlaneSolverResult result = b2SolvePlanes( b2Vec2{ 0.0f, -1.0f }, &plane, 1 );

	// 软碰撞，限额用尽后移动体会下沉
	ENSURE_SMALL( plane.push - 0.1f, FLT_EPSILON );
	ENSURE_SMALL( result.delta.y + 0.9f, FLT_EPSILON );

	return 0;
}

static int CornerTest( void )
{
	b2CollisionPlane planes[2] = {
		MakePlane( b2Vec2{ 0.0f, 1.0f }, FLT_MAX ),
		MakePlane( b2Vec2{ 1.0f, 0.0f }, FLT_MAX ),
	};

	b2PlaneSolverResult result = b2SolvePlanes( b2Vec2{ -1.0f, -1.0f }, planes, (int)(sizeof(planes)/sizeof(planes[0])) );

	// 两个平面同时顶住，而不是第二个放松第一个
	for ( int i = 0; i < (int)(sizeof(planes)/sizeof(planes[0])); ++i )
	{
		float separation = b2PlaneSeparation( planes[i].plane, result.delta );
		ENSURE_SMALL( separation + B2_LINEAR_SLOP, B2_LINEAR_SLOP );
		ENSURE( planes[i].push > 0.0f );
	}

	return 0;
}

static int WedgeTest( void )
{
	// 锐角楔需要多轮才能稳定，不像直角拐角
	b2CollisionPlane planes[2] = {
		MakePlane( b2Vec2{ 0.0f, 1.0f }, FLT_MAX ),
		MakePlane( b2Vec2{ 0.8f, 0.6f }, FLT_MAX ),
	};

	b2PlaneSolverResult result = b2SolvePlanes( b2Vec2{ -1.0f, -1.0f }, planes, (int)(sizeof(planes)/sizeof(planes[0])) );

	// 稳定意味着同时靠在两个面上。过早停止会把移动体推离其中一面，
	// 表现为很大的正分离
	for ( int i = 0; i < (int)(sizeof(planes)/sizeof(planes[0])); ++i )
	{
		float separation = b2PlaneSeparation( planes[i].plane, result.delta );
		ENSURE_SMALL( separation + B2_LINEAR_SLOP, B2_LINEAR_SLOP );
	}

	return 0;
}

int MoverPlanesTest(void)
{
 if(NoPlanesTest() || SolvePlaneTest() || PushLimitTest() || CornerTest() || WedgeTest()) { return 1; }
 return 0;
}

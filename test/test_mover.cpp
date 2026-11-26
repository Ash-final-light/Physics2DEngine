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

static int ClipVectorTest( void )
{
	b2CollisionPlane plane = MakePlane( b2Vec2{ 0.0f, 1.0f }, FLT_MAX );
	plane.push = 0.5f;

	b2Vec2 v = b2ClipVector( b2Vec2{ 1.0f, -1.0f }, &plane, 1 );
	ENSURE_SMALL( v.x - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( v.y, FLT_EPSILON );

	// 分离速度原样穿过，不被改动
	v = b2ClipVector( b2Vec2{ 1.0f, 1.0f }, &plane, 1 );
	ENSURE_SMALL( v.x - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( v.y - 1.0f, FLT_EPSILON );

	plane.clipVelocity = false;
	v = b2ClipVector( b2Vec2{ 1.0f, -1.0f }, &plane, 1 );
	ENSURE_SMALL( v.y + 1.0f, FLT_EPSILON );

	// 零推力表示求解器从未碰到这个平面
	plane.clipVelocity = true;
	plane.push = 0.0f;
	v = b2ClipVector( b2Vec2{ 1.0f, -1.0f }, &plane, 1 );
	ENSURE_SMALL( v.y + 1.0f, FLT_EPSILON );

	return 0;
}

int MoverClipVectorTest(void)
{
 if(ClipVectorTest()) { return 1; }
 b2CollisionPlane planes[2]={MakePlane({0,1},FLT_MAX),MakePlane({1,0},FLT_MAX)};
 b2SolvePlanes({-1,-1},planes,2);b2Vec2 v=b2ClipVector({-3,-4},planes,2);ENSURE_SMALL(b2Length(v),FLT_EPSILON);
 v=b2ClipVector({3,4},planes,2);ENSURE(v.x==3 && v.y==4);return 0;
}

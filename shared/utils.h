// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/collision.h"
#include "box2d/math_functions.h"

#define RAND_LIMIT 32767
#define RAND_SEED 12345

// 简易随机数生成器的全局种子。

#ifdef __cplusplus
extern "C"
{
#endif

extern uint32_t g_randomSeed;
b2Polygon RandomPolygon( float extent );

int GetNumberOfCores( void );

#ifdef __cplusplus
}
#endif

// 简易随机数生成器。用它代替 rand()，以保证跨平台确定性。
B2_INLINE int RandomInt()
{
	// XorShift32 算法
	uint32_t x = g_randomSeed;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	g_randomSeed = x;

	// 将 32 位值映射到 0 到 RAND_LIMIT 区间
	return (int)( x % ( RAND_LIMIT + 1 ) );
}

// [lo, hi] 区间内的随机整数
B2_INLINE int RandomIntRange( int lo, int hi )
{
	return lo + RandomInt() % ( hi - lo + 1 );
}

// [-1, 1] 区间内的随机数
B2_INLINE float RandomFloat()
{
	float r = (float)( RandomInt() & ( RAND_LIMIT ) );
	r /= RAND_LIMIT;
	r = 2.0f * r - 1.0f;
	return r;
}

// [lo, hi] 区间内的随机浮点数
B2_INLINE float RandomFloatRange( float lo, float hi )
{
	float r = (float)( RandomInt() & ( RAND_LIMIT ) );
	r /= RAND_LIMIT;
	r = ( hi - lo ) * r + lo;
	return r;
}

// 坐标在 [lo, hi] 区间内的随机向量
B2_INLINE b2Vec2 RandomVec2( float lo, float hi )
{
	b2Vec2 v;
	v.x = RandomFloatRange( lo, hi );
	v.y = RandomFloatRange( lo, hi );
	return v;
}

// 坐标在 [lo, hi] 区间内的随机位置
B2_INLINE b2Pos RandomPos( float lo, float hi )
{
	b2Pos v;
	v.x = RandomFloatRange( lo, hi );
	v.y = RandomFloatRange( lo, hi );
	return v;
}

// 角度在 [-pi, pi] 区间内的随机旋转
B2_INLINE b2Rot RandomRot( void )
{
	float angle = RandomFloatRange( -B2_PI, B2_PI );
	return b2MakeRot( angle );
}

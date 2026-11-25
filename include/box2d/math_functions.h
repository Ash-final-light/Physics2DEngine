// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "math_types.h"

#include <float.h>
#include <math.h>

/**
 * @addtogroup math
 * @{
 */

/// 这是否为有效数字？非 NaN 或无穷。
B2_API bool b2IsValidFloat( float a );

/// 这是否为有效向量？非 NaN 或无穷。
B2_API bool b2IsValidVec2( b2Vec2 v );

/// @return 两个整数中的较小值
B2_INLINE int b2MinInt( int a, int b )
{
	return a < b ? a : b;
}

/// @return 两个整数中的较大值
B2_INLINE int b2MaxInt( int a, int b )
{
	return a > b ? a : b;
}

/// @return 整数的绝对值
B2_INLINE int b2AbsInt( int a )
{
	return a < 0 ? -a : a;
}

/// @return 夹紧在下界与上界之间的整数
B2_INLINE int b2ClampInt( int a, int lower, int upper )
{
	return a < lower ? lower : ( a > upper ? upper : a );
}

/// https://en.wikipedia.org/wiki/Floor_and_ceiling_functions
B2_INLINE int b2CeilingInt( int numerator, int denominator )
{
	B2_VALIDATE( denominator > 0 && numerator >= 0 );
	return ( numerator + denominator - 1 ) / denominator;
}

/// @return 两个浮点数中的较小值
B2_INLINE float b2MinFloat( float a, float b )
{
	return a < b ? a : b;
}

/// @return 两个浮点数中的较大值
B2_INLINE float b2MaxFloat( float a, float b )
{
	return a > b ? a : b;
}

/// @return 浮点数的绝对值
B2_INLINE float b2AbsFloat( float a )
{
	return a < 0 ? -a : a;
}

/// @return 夹紧在下界与上界之间的浮点数
B2_INLINE float b2ClampFloat( float a, float lower, float upper )
{
	return a < lower ? lower : ( a > upper ? upper : a );
}

/// 向量点积
B2_INLINE float b2Dot( b2Vec2 a, b2Vec2 b )
{
	return a.x * b.x + a.y * b.y;
}

/// 向量叉积。二维中得到标量。
B2_INLINE float b2Cross( b2Vec2 a, b2Vec2 b )
{
	return a.x * b.y - a.y * b.x;
}

/// 对向量与标量执行叉积。二维中得到向量。
B2_INLINE b2Vec2 b2CrossVS( b2Vec2 v, float s )
{
	return B2_LITERAL( b2Vec2 ){ s * v.y, -s * v.x };
}

/// 对标量与向量执行叉积。二维中得到向量。
B2_INLINE b2Vec2 b2CrossSV( float s, b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ -s * v.y, s * v.x };
}

/// 获取指向左侧的垂直向量。等价于 b2CrossSV(1.0f, v)
B2_INLINE b2Vec2 b2LeftPerp( b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ -v.y, v.x };
}

/// 获取指向右侧的垂直向量。等价于 b2CrossVS(v, 1.0f)
B2_INLINE b2Vec2 b2RightPerp( b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ v.y, -v.x };
}

/// 向量加法
B2_INLINE b2Vec2 b2Add( b2Vec2 a, b2Vec2 b )
{
	return B2_LITERAL( b2Vec2 ){ a.x + b.x, a.y + b.y };
}

/// 向量减法
B2_INLINE b2Vec2 b2Sub( b2Vec2 a, b2Vec2 b )
{
	return B2_LITERAL( b2Vec2 ){ a.x - b.x, a.y - b.y };
}

/// 向量取反
B2_INLINE b2Vec2 b2Neg( b2Vec2 a )
{
	return B2_LITERAL( b2Vec2 ){ -a.x, -a.y };
}

/// 获取此向量的长度（范数）
B2_INLINE float b2Length( b2Vec2 v )
{
	return sqrtf( v.x * v.x + v.y * v.y );
}

/// 获取此向量的长度平方
B2_INLINE float b2LengthSquared( b2Vec2 v )
{
	return v.x * v.x + v.y * v.y;
}

/// 若可能则将向量转换为单位向量，否则返回零向量。
B2_FORCE_INLINE b2Vec2 b2Normalize( b2Vec2 a )
{
	float lengthSquared = a.x * a.x + a.y * a.y;
	if ( lengthSquared > 1000.0f * FLT_MIN )
	{
		float s = 1.0f / sqrtf( lengthSquared );
		b2Vec2 u = { s * a.x, s * a.y };
		return u;
	}
	return B2_LITERAL( b2Vec2 ){ 0.0f, 0.0f };
}

/// 判断所提供向量是否已归一化（norm(a) == 1）。
B2_INLINE bool b2IsNormalized( b2Vec2 a )
{
	float aa = b2Dot( a, a );
	return b2AbsFloat( 1.0f - aa ) < 100.0f * FLT_EPSILON;
}

/// 设置每米对应的长度单位。只接受有限且大于 0 的值，非法值保持原值。
/// 默认值为 1。
B2_API void b2SetLengthUnitsPerMeter( float lengthUnits );

/// 获取当前每米的长度单位。
B2_API float b2GetLengthUnitsPerMeter( void );

#ifdef __cplusplus
/// 此值出于效率而导出。请勿直接修改。
extern "C" BOX2D_EXPORT float b2_lengthUnitsPerMeter;
#else
/// 此值出于效率而导出。请勿直接修改。
BOX2D_EXPORT extern float b2_lengthUnitsPerMeter;
#endif

/**@}*/

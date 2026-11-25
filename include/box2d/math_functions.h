// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"

#include <math.h>

/**
 * @addtogroup math
 * @{
 */

/// 这是否为有效数字？非 NaN 或无穷。
B2_API bool b2IsValidFloat( float a );

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

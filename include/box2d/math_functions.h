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

/// 这是否为有效旋转？非 NaN 或无穷。已归一化。
B2_API bool b2IsValidRot( b2Rot q );

/// 这是否为有效变换？非 NaN 或无穷。旋转已归一化。
B2_API bool b2IsValidTransform( b2Transform t );

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

/// 标量与向量相乘。
B2_INLINE b2Vec2 b2MulSV( float s, b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ s * v.x, s * v.y };
}

/// a + s * b
B2_INLINE b2Vec2 b2MulAdd( b2Vec2 a, float s, b2Vec2 b )
{
	return B2_LITERAL( b2Vec2 ){ a.x + s * b.x, a.y + s * b.y };
}

/// a - s * b
B2_INLINE b2Vec2 b2MulSub( b2Vec2 a, float s, b2Vec2 b )
{
	return B2_LITERAL( b2Vec2 ){ a.x - s * b.x, a.y - s * b.y };
}

/// 分量绝对值。
B2_INLINE b2Vec2 b2Abs( b2Vec2 a )
{
	return B2_LITERAL( b2Vec2 ){ b2AbsFloat( a.x ), b2AbsFloat( a.y ) };
}

/// 向量线性插值。
B2_INLINE b2Vec2 b2Lerp( b2Vec2 a, b2Vec2 b, float t )
{
	return B2_LITERAL( b2Vec2 ){ ( 1.0f - t ) * a.x + t * b.x, ( 1.0f - t ) * a.y + t * b.y };
}

/// 分量最小值。
B2_INLINE b2Vec2 b2Min( b2Vec2 a, b2Vec2 b )
{
	return B2_LITERAL( b2Vec2 ){ b2MinFloat( a.x, b.x ), b2MinFloat( a.y, b.y ) };
}

/// 分量最大值。
B2_INLINE b2Vec2 b2Max( b2Vec2 a, b2Vec2 b )
{
	return B2_LITERAL( b2Vec2 ){ b2MaxFloat( a.x, b.x ), b2MaxFloat( a.y, b.y ) };
}

/// 两点之间的距离。
B2_INLINE float b2Distance( b2Vec2 a, b2Vec2 b )
{
	float dx = b.x - a.x;
	float dy = b.y - a.y;
	return sqrtf( dx * dx + dy * dy );
}

/// 两点距离的平方。
B2_INLINE float b2DistanceSquared( b2Vec2 a, b2Vec2 b )
{
	b2Vec2 c = b2Sub( a, b );
	return c.x * c.x + c.y * c.y;
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

/// 若可能则将向量转换为单位向量，否则返回零向量。同时写出长度。
B2_INLINE b2Vec2 b2GetLengthAndNormalize( float* length, b2Vec2 a )
{
	float lengthSquared = a.x * a.x + a.y * a.y;
	if ( lengthSquared > 1000.0f * FLT_MIN )
	{
		*length = sqrtf( lengthSquared );
		float s = 1.0f / *length;
		b2Vec2 u = { s * a.x, s * a.y };
		return u;
	}

	*length = 0.0f;
	return B2_LITERAL( b2Vec2 ){ 0.0f, 0.0f };
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

/// 归一化旋转
B2_INLINE b2Rot b2NormalizeRot( b2Rot q )
{
	float mag = sqrtf( q.s * q.s + q.c * q.c );
	float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
	b2Rot qn = { q.c * invMag, q.s * invMag };
	return qn;
}

/// 此旋转是否已归一化？
B2_INLINE bool b2IsNormalizedRot( b2Rot q )
{
	float qq = q.s * q.s + q.c * q.c;
	return 1.0f - 0.0006f < qq && qq < 1.0f + 0.0006f;
}

/// 使用单位向量创建旋转
B2_INLINE b2Rot b2MakeRotFromUnitVector( b2Vec2 unitVector )
{
	B2_ASSERT( b2IsNormalized( unitVector ) );
	return B2_LITERAL( b2Rot ){ unitVector.x, unitVector.y };
}

/// 计算两个单位向量之间的旋转
B2_API b2Rot b2ComputeRotationBetweenUnitVectors( b2Vec2 v1, b2Vec2 v2 );

/// 计算范围 [-pi, pi] 内的近似反正切。
B2_API float b2Atan2( float y, float x );

/// 计算以弧度为单位的角的余弦与正弦。
B2_API b2CosSin b2ComputeCosSin( float radians );

/// 使用以弧度为单位的角创建旋转
B2_INLINE b2Rot b2MakeRot( float radians )
{
	b2CosSin cs = b2ComputeCosSin( radians );
	return B2_LITERAL( b2Rot ){ cs.cosine, cs.sine };
}

/// 获取范围 [-pi, pi] 内以弧度为单位的角
B2_INLINE float b2Rot_GetAngle( b2Rot q )
{
	return b2Atan2( q.s, q.c );
}

/// 旋转 b 相对旋转 a 的角。单位为弧度。
B2_INLINE float b2RelativeAngle( b2Rot a, b2Rot b )
{
	float s = a.c * b.s - a.s * b.c;
	float c = a.c * b.c + a.s * b.s;
	return b2Atan2( s, c );
}

/// 将任意角转换到区间 [-pi, pi]。
B2_INLINE float b2UnwindAngle( float radians )
{
	B2_VALIDATE( b2AbsFloat( radians ) < 1.0e4f );
	float x = b2ClampFloat( radians, -1.0e6f, 1.0e6f );
	double twoPi = 2.0f * B2_PI;
	double roundToNearest = 6755399441055744.0;
	double a = x;
	double k = ( a / twoPi + roundToNearest ) - roundToNearest;
	return (float)( a - k * twoPi );
}

/// 获取旋转的逆
B2_INLINE b2Rot b2InvertRot( b2Rot a )
{
	return B2_LITERAL( b2Rot ){ a.c, -a.s };
}

/// 两个旋转相乘：q * r
B2_INLINE b2Rot b2MulRot( b2Rot q, b2Rot r )
{
	b2Rot qr;
	qr.s = q.s * r.c + q.c * r.s;
	qr.c = q.c * r.c - q.s * r.s;
	return qr;
}

/// 转置相乘两个旋转：inv(a) * b
B2_INLINE b2Rot b2InvMulRot( b2Rot a, b2Rot b )
{
	b2Rot r;
	r.s = a.c * b.s - a.s * b.c;
	r.c = a.c * b.c + a.s * b.s;
	return r;
}

/// 旋转向量。
B2_INLINE b2Vec2 b2RotateVector( b2Rot q, b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y };
}

/// 逆旋转向量。
B2_INLINE b2Vec2 b2InvRotateVector( b2Rot q, b2Vec2 v )
{
	return B2_LITERAL( b2Vec2 ){ q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y };
}

/// 归一化线性插值
B2_INLINE b2Rot b2NLerp( b2Rot q1, b2Rot q2, float t )
{
	float omt = 1.0f - t;
	b2Rot q = {
		omt * q1.c + t * q2.c,
		omt * q1.s + t * q2.s,
	};

	float mag = sqrtf( q.s * q.s + q.c * q.c );
	float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
	b2Rot qn = { q.c * invMag, q.s * invMag };
	return qn;
}

/// 由角速度积分旋转
B2_INLINE b2Rot b2IntegrateRotation( b2Rot q1, float deltaAngle )
{
	b2Rot q2 = { q1.c - deltaAngle * q1.s, q1.s + deltaAngle * q1.c };
	float mag = sqrtf( q2.s * q2.s + q2.c * q2.c );
	float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
	b2Rot qn = { q2.c * invMag, q2.s * invMag };
	return qn;
}

/// 获取 x 轴
B2_INLINE b2Vec2 b2Rot_GetXAxis( b2Rot q )
{
	b2Vec2 v = { q.c, q.s };
	return v;
}

/// 获取 y 轴
B2_INLINE b2Vec2 b2Rot_GetYAxis( b2Rot q )
{
	b2Vec2 v = { -q.s, q.c };
	return v;
}

/// 变换点（例如从局部空间到世界空间）。
B2_INLINE b2Vec2 b2TransformPoint( b2Transform t, const b2Vec2 p )
{
	float x = ( t.q.c * p.x - t.q.s * p.y ) + t.p.x;
	float y = ( t.q.s * p.x + t.q.c * p.y ) + t.p.y;
	return B2_LITERAL( b2Vec2 ){ x, y };
}

/// 逆变换点（例如从世界空间到局部空间）。
B2_INLINE b2Vec2 b2InvTransformPoint( b2Transform t, const b2Vec2 p )
{
	float vx = p.x - t.p.x;
	float vy = p.y - t.p.y;
	return B2_LITERAL( b2Vec2 ){ t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy };
}

/// 两个变换相乘。
B2_INLINE b2Transform b2MulTransforms( b2Transform A, b2Transform B )
{
	b2Transform C;
	C.q = b2MulRot( A.q, B.q );
	C.p = b2Add( b2RotateVector( A.q, B.p ), A.p );
	return C;
}

/// 创建将坐标系 B 中的局部点转换为坐标系 A 中局部点的变换。
B2_INLINE b2Transform b2InvMulTransforms( b2Transform A, b2Transform B )
{
	b2Transform C;
	C.q = b2InvMulRot( A.q, B.q );
	C.p = b2InvRotateVector( A.q, b2Sub( B.p, A.p ) );
	return C;
}

/// 2×2 矩阵乘以二维向量
B2_INLINE b2Vec2 b2MulMV( b2Mat22 A, b2Vec2 v )
{
	b2Vec2 u = {
		A.cx.x * v.x + A.cy.x * v.y,
		A.cx.y * v.x + A.cy.y * v.y,
	};
	return u;
}

/// 获取 2×2 矩阵的逆。行列式为 0 时返回零矩阵。
B2_INLINE b2Mat22 b2GetInverse22( b2Mat22 A )
{
	float a = A.cx.x, b = A.cy.x, c = A.cx.y, d = A.cy.y;
	float det = a * d - b * c;
	if ( det != 0.0f )
	{
		det = 1.0f / det;
	}

	b2Mat22 B = {
		{ det * d, -det * c },
		{ -det * b, det * a },
	};
	return B;
}

/// 求解 A * x = b。行列式为 0 时返回零向量。
B2_INLINE b2Vec2 b2Solve22( b2Mat22 A, b2Vec2 b )
{
	float a11 = A.cx.x, a12 = A.cy.x, a21 = A.cx.y, a22 = A.cy.y;
	float det = a11 * a22 - a12 * a21;
	if ( det != 0.0f )
	{
		det = 1.0f / det;
	}
	b2Vec2 x = { det * ( a22 * b.x - a12 * b.y ), det * ( a11 * b.y - a21 * b.x ) };
	return x;
}

/// 上下界都是有限值，且上界不小于下界。
B2_API bool b2IsValidAABB( b2AABB a );

/// a 是否完全包含 b。
B2_INLINE bool b2AABB_Contains( b2AABB a, b2AABB b )
{
	bool s = true;
	s = s && a.lowerBound.x <= b.lowerBound.x;
	s = s && a.lowerBound.y <= b.lowerBound.y;
	s = s && b.upperBound.x <= a.upperBound.x;
	s = s && b.upperBound.y <= a.upperBound.y;
	return s;
}

/// 获取 AABB 的中心。
B2_INLINE b2Vec2 b2AABB_Center( b2AABB a )
{
	b2Vec2 b = { 0.5f * ( a.lowerBound.x + a.upperBound.x ), 0.5f * ( a.lowerBound.y + a.upperBound.y ) };
	return b;
}

/// 获取 AABB 的半宽半高。
B2_INLINE b2Vec2 b2AABB_Extents( b2AABB a )
{
	b2Vec2 b = { 0.5f * ( a.upperBound.x - a.lowerBound.x ), 0.5f * ( a.upperBound.y - a.lowerBound.y ) };
	return b;
}

/// 两个 AABB 的并集。
B2_INLINE b2AABB b2AABB_Union( b2AABB a, b2AABB b )
{
	b2AABB c;
	c.lowerBound.x = b2MinFloat( a.lowerBound.x, b.lowerBound.x );
	c.lowerBound.y = b2MinFloat( a.lowerBound.y, b.lowerBound.y );
	c.upperBound.x = b2MaxFloat( a.upperBound.x, b.upperBound.x );
	c.upperBound.y = b2MaxFloat( a.upperBound.y, b.upperBound.y );
	return c;
}

/// a 与 b 是否重叠。边界相接算重叠。
B2_INLINE bool b2AABB_Overlaps( b2AABB a, b2AABB b )
{
	return !( b.lowerBound.x > a.upperBound.x || b.lowerBound.y > a.upperBound.y || a.lowerBound.x > b.upperBound.x ||
			  a.lowerBound.y > b.upperBound.y );
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

B2_INLINE float b2RoundDownFloat( double x )
{
#if defined( BOX2D_DOUBLE_PRECISION )
	float f = (float)x;
	return (double)f > x ? nextafterf( f, -FLT_MAX ) : f;
#else
	return (float)x;
#endif
}

B2_INLINE float b2RoundUpFloat( double x )
{
#if defined( BOX2D_DOUBLE_PRECISION )
	float f = (float)x;
	return (double)f < x ? nextafterf( f, FLT_MAX ) : f;
#else
	return (float)x;
#endif
}

B2_INLINE b2AABB b2MakeAABB( const b2Vec2* points, int count, float radius )
{
	B2_ASSERT( count > 0 );
	b2AABB a = { points[0], points[0] };
	for ( int i = 1; i < count; ++i )
	{
		a.lowerBound = b2Min( a.lowerBound, points[i] );
		a.upperBound = b2Max( a.upperBound, points[i] );
	}

	b2Vec2 r = { radius, radius };
	a.lowerBound = b2Sub( a.lowerBound, r );
	a.upperBound = b2Add( a.upperBound, r );

	return a;
}

B2_INLINE b2Transform b2ToRelativeTransform( b2WorldTransform t, b2Pos base )
{
	b2Transform r;
	r.q = t.q;
	r.p = B2_LITERAL( b2Vec2 ){ (float)( t.p.x - base.x ), (float)( t.p.y - base.y ) };
	return r;
}

B2_INLINE b2Vec2 b2ToVec2( b2Pos p )
{
	return B2_LITERAL( b2Vec2 ){ (float)p.x, (float)p.y };
}

B2_INLINE b2Pos b2OffsetPos( b2Pos p, b2Vec2 d )
{
	return B2_LITERAL( b2Pos ){ p.x + d.x, p.y + d.y };
}

B2_INLINE b2Vec2 b2InvTransformWorldPoint( b2WorldTransform t, b2Pos p )
{
	float vx = (float)( p.x - t.p.x );
	float vy = (float)( p.y - t.p.y );
	return B2_LITERAL( b2Vec2 ){ t.q.c * vx + t.q.s * vy, -t.q.s * vx + t.q.c * vy };
}

B2_INLINE b2Pos b2TransformWorldPoint( b2WorldTransform t, b2Vec2 p )
{
	float rx = t.q.c * p.x - t.q.s * p.y;
	float ry = t.q.s * p.x + t.q.c * p.y;
	return B2_LITERAL( b2Pos ){ t.p.x + rx, t.p.y + ry };
}

B2_INLINE b2Transform b2InvMulWorldTransforms( b2WorldTransform A, b2WorldTransform B )
{
	b2Transform C;
	C.q = b2InvMulRot( A.q, B.q );
	b2Vec2 d = { (float)( B.p.x - A.p.x ), (float)( B.p.y - A.p.y ) };
	C.p = b2InvRotateVector( A.q, d );
	return C;
}

B2_INLINE float b2PlaneSeparation( b2Plane plane, b2Vec2 point )
{
	return b2Dot( plane.normal, point ) - plane.offset;
}

B2_INLINE float b2SpringDamper( float hertz, float dampingRatio, float position, float velocity, float timeStep )
{
	float omega = 2.0f * B2_PI * hertz;
	float omegaH = omega * timeStep;
	return ( velocity - omega * omegaH * position ) / ( 1.0f + 2.0f * dampingRatio * omegaH + omegaH * omegaH );
}

B2_INLINE b2Vec2 b2SubPos( b2Pos a, b2Pos b )
{
	return B2_LITERAL( b2Vec2 ){ (float)( a.x - b.x ), (float)( a.y - b.y ) };
}

B2_INLINE b2Pos b2LerpPos( b2Pos a, b2Pos b, float t )
{
	return B2_LITERAL( b2Pos ){ ( 1.0f - t ) * a.x + t * b.x, ( 1.0f - t ) * a.y + t * b.y };
}

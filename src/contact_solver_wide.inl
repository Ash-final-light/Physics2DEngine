// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "body.h"
#include "constraint_graph.h"
#include "contact.h"
#include "contact_solver.h"
#include "core.h"
#include "world.h"
#include "platform.h"
#include "solver_set.h"

#include <stddef.h>

#ifndef B2_WIDE
#define B2_WIDE( name ) name##W4
#endif
#define B2_SIMD_SHIFT 2

#define b2FloatW B2_WIDE( b2Float )
#define b2Vec2W B2_WIDE( b2Vec2 )
#define b2RotW B2_WIDE( b2Rot )
#define b2BodyStateW B2_WIDE( b2BodyState )
#define b2ContactConstraintWide B2_WIDE( b2ContactConstraint )

#if defined( B2_SIMD_NEON )

#include <arm_neon.h>

// 宽浮点保存 4 个数
typedef float32x4_t b2FloatW;

#elif defined( B2_SIMD_SSE2 )

#include <emmintrin.h>

// 宽浮点保存 4 个数
typedef __m128 b2FloatW;

#else

// 标量运算
typedef struct b2FloatW
{
	float x, y, z, w;
} b2FloatW;

#endif

// 宽 vec2
typedef struct b2Vec2W
{
	b2FloatW X, Y;
} b2Vec2W;

// 宽旋转
typedef struct b2RotW
{
	b2FloatW C, S;
} b2RotW;

#if defined( B2_SIMD_NEON )

static inline b2FloatW b2ZeroW( void )
{
	return vdupq_n_f32( 0.0f );
}

static inline b2FloatW b2SplatW( float scalar )
{
	return vdupq_n_f32( scalar );
}

static inline b2FloatW b2SetW( float a, float b, float c, float d )
{
	float32_t array[4] = { a, b, c, d };
	return vld1q_f32( array );
}

static inline b2FloatW b2NegW( b2FloatW a )
{
	return vnegq_f32( a );
}

static inline b2FloatW b2AddW( b2FloatW a, b2FloatW b )
{
	return vaddq_f32( a, b );
}

static inline b2FloatW b2SubW( b2FloatW a, b2FloatW b )
{
	return vsubq_f32( a, b );
}

static inline b2FloatW b2MulW( b2FloatW a, b2FloatW b )
{
	return vmulq_f32( a, b );
}

static inline b2FloatW b2DivW( b2FloatW a, b2FloatW b )
{
	return vdivq_f32( a, b );
}

static inline b2FloatW b2MulAddW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	return vaddq_f32( a, vmulq_f32( b, c ) );
}

static inline b2FloatW b2MulSubW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	return vsubq_f32( a, vmulq_f32( b, c ) );
}

static inline b2FloatW b2MinW( b2FloatW a, b2FloatW b )
{
	return vminq_f32( a, b );
}

static inline b2FloatW b2MaxW( b2FloatW a, b2FloatW b )
{
	return vmaxq_f32( a, b );
}

// a = clamp(a, -b, b)
static inline b2FloatW b2SymClampW( b2FloatW a, b2FloatW b )
{
	b2FloatW nb = vnegq_f32( b );
	return vmaxq_f32( nb, vminq_f32( a, b ) );
}

static inline b2FloatW b2AndW( b2FloatW a, b2FloatW b )
{
	return vreinterpretq_f32_u32( vandq_u32( vreinterpretq_u32_f32( a ), vreinterpretq_u32_f32( b ) ) );
}

static inline b2FloatW b2GreaterThanW( b2FloatW a, b2FloatW b )
{
	return vreinterpretq_f32_u32( vcgtq_f32( a, b ) );
}

static inline b2FloatW b2LessThanW( b2FloatW a, b2FloatW b )
{
	return vreinterpretq_f32_u32( vcltq_f32( a, b ) );
}

static inline bool b2AnyTrueW( b2FloatW mask )
{
	return vmaxvq_u32( vreinterpretq_u32_f32( mask ) ) != 0;
}

static inline bool b2AllZeroW( b2FloatW a )
{
	// 创建用于比较的零向量
	b2FloatW zero = vdupq_n_f32( 0.0f );

	// 将输入向量与零比较
	uint32x4_t cmp_result = vceqq_f32( a, zero );

	// 用 vminvq 检查比较结果是否全非零
	return vminvq_u32( cmp_result ) != 0;
}

// 按分量返回 mask ? b : a
static inline b2FloatW b2BlendW( b2FloatW a, b2FloatW b, b2FloatW mask )
{
	uint32x4_t mask32 = vreinterpretq_u32_f32( mask );
	return vbslq_f32( mask32, b, a );
}

static inline b2FloatW b2LoadU( const float32_t* data )
{
	return vld1q_f32( data );
}

static inline void b2StoreW( float32_t* data, b2FloatW a )
{
	vst1q_f32( data, a );
}

static inline b2FloatW b2UnpackLoW( b2FloatW a, b2FloatW b )
{
	return vzip1q_f32( a, b );
}

static inline b2FloatW b2UnpackHiW( b2FloatW a, b2FloatW b )
{
	return vzip2q_f32( a, b );
}

static inline b2FloatW b2SoftMaskW( const int* indexA, const int* indexB )
{
	int32x4_t zero = vdupq_n_s32( 0 );
	uint32x4_t a = vceqq_s32( vld1q_s32( indexA ), zero );
	uint32x4_t b = vceqq_s32( vld1q_s32( indexB ), zero );
	return vreinterpretq_f32_u32( vorrq_u32( a, b ) );
}

#elif defined( B2_SIMD_SSE2 )

static inline b2FloatW b2ZeroW( void )
{
	return _mm_setzero_ps();
}

static inline b2FloatW b2SplatW( float scalar )
{
	return _mm_set1_ps( scalar );
}

static inline b2FloatW b2SetW( float a, float b, float c, float d )
{
	return _mm_setr_ps( a, b, c, d );
}

static inline b2FloatW b2NegW( b2FloatW a )
{
	// 为每个元素创建符号位置位的掩码
	__m128 mask = _mm_set1_ps( -0.0f );

	// 与掩码异或以取反每个元素
	return _mm_xor_ps( a, mask );
}

static inline b2FloatW b2AddW( b2FloatW a, b2FloatW b )
{
	return _mm_add_ps( a, b );
}

static inline b2FloatW b2SubW( b2FloatW a, b2FloatW b )
{
	return _mm_sub_ps( a, b );
}

static inline b2FloatW b2MulW( b2FloatW a, b2FloatW b )
{
	return _mm_mul_ps( a, b );
}

static inline b2FloatW b2DivW( b2FloatW a, b2FloatW b )
{
	return _mm_div_ps( a, b );
}

static inline b2FloatW b2MulAddW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	return _mm_add_ps( a, _mm_mul_ps( b, c ) );
}

static inline b2FloatW b2MulSubW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	return _mm_sub_ps( a, _mm_mul_ps( b, c ) );
}

static inline b2FloatW b2MinW( b2FloatW a, b2FloatW b )
{
	return _mm_min_ps( a, b );
}

static inline b2FloatW b2MaxW( b2FloatW a, b2FloatW b )
{
	return _mm_max_ps( a, b );
}

// a = clamp(a, -b, b)
static inline b2FloatW b2SymClampW( b2FloatW a, b2FloatW b )
{
	// 为每个元素创建符号位置位的掩码
	__m128 mask = _mm_set1_ps( -0.0f );

	// 与掩码异或以取反每个元素
	__m128 nb = _mm_xor_ps( b, mask );

	return _mm_max_ps( nb, _mm_min_ps( a, b ) );
}

static inline b2FloatW b2AndW( b2FloatW a, b2FloatW b )
{
	return _mm_and_ps( a, b );
}

static inline b2FloatW b2GreaterThanW( b2FloatW a, b2FloatW b )
{
	return _mm_cmpgt_ps( a, b );
}

static inline b2FloatW b2LessThanW( b2FloatW a, b2FloatW b )
{
	return _mm_cmplt_ps( a, b );
}

static inline bool b2AnyTrueW( b2FloatW mask )
{
	return _mm_movemask_ps( mask ) != 0;
}

static inline bool b2AllZeroW( b2FloatW a )
{
	// 将每个元素与零比较
	b2FloatW zero = _mm_setzero_ps();
	b2FloatW cmp = _mm_cmpeq_ps( a, zero );

	// 由比较结果创建掩码
	int mask = _mm_movemask_ps( cmp );

	// 若所有元素为零，掩码为 0xF（二进制 1111）
	return mask == 0xF;
}

// 按分量返回 mask ? b : a
static inline b2FloatW b2BlendW( b2FloatW a, b2FloatW b, b2FloatW mask )
{
	return _mm_or_ps( _mm_and_ps( mask, b ), _mm_andnot_ps( mask, a ) );
}

static inline b2FloatW b2LoadU( const float* data )
{
	return _mm_loadu_ps( data );
}

static inline void b2StoreW( float* data, b2FloatW a )
{
	_mm_store_ps( data, a );
}

static inline b2FloatW b2UnpackLoW( b2FloatW a, b2FloatW b )
{
	return _mm_unpacklo_ps( a, b );
}

static inline b2FloatW b2UnpackHiW( b2FloatW a, b2FloatW b )
{
	return _mm_unpackhi_ps( a, b );
}

static inline b2FloatW b2SoftMaskW( const int* indexA, const int* indexB )
{
	__m128i zero = _mm_setzero_si128();
	__m128i a = _mm_cmpeq_epi32( _mm_load_si128( (const __m128i*)indexA ), zero );
	__m128i b = _mm_cmpeq_epi32( _mm_load_si128( (const __m128i*)indexB ), zero );
	return _mm_castsi128_ps( _mm_or_si128( a, b ) );
}

#else

static inline b2FloatW b2ZeroW( void )
{
	return b2FloatW{ 0.0f, 0.0f, 0.0f, 0.0f };
}

static inline b2FloatW b2SplatW( float scalar )
{
	return b2FloatW{ scalar, scalar, scalar, scalar };
}

static inline b2FloatW b2SetW( float a, float b, float c, float d )
{
	return b2FloatW{ a, b, c, d };
}

static inline b2FloatW b2NegW( b2FloatW a )
{
	return b2FloatW{ -a.x, -a.y, -a.z, -a.w };
}

static inline b2FloatW b2AddW( b2FloatW a, b2FloatW b )
{
	return b2FloatW{ a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w };
}

static inline b2FloatW b2SubW( b2FloatW a, b2FloatW b )
{
	return b2FloatW{ a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w };
}

static inline b2FloatW b2MulW( b2FloatW a, b2FloatW b )
{
	return b2FloatW{ a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w };
}

static inline b2FloatW b2DivW( b2FloatW a, b2FloatW b )
{
	return b2FloatW{ a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w };
}

static inline b2FloatW b2MulAddW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	return b2FloatW{ a.x + b.x * c.x, a.y + b.y * c.y, a.z + b.z * c.z, a.w + b.w * c.w };
}

static inline b2FloatW b2MulSubW( b2FloatW a, b2FloatW b, b2FloatW c )
{
	return b2FloatW{ a.x - b.x * c.x, a.y - b.y * c.y, a.z - b.z * c.z, a.w - b.w * c.w };
}

static inline b2FloatW b2MinW( b2FloatW a, b2FloatW b )
{
	b2FloatW r;
	r.x = a.x <= b.x ? a.x : b.x;
	r.y = a.y <= b.y ? a.y : b.y;
	r.z = a.z <= b.z ? a.z : b.z;
	r.w = a.w <= b.w ? a.w : b.w;
	return r;
}

static inline b2FloatW b2MaxW( b2FloatW a, b2FloatW b )
{
	b2FloatW r;
	r.x = a.x >= b.x ? a.x : b.x;
	r.y = a.y >= b.y ? a.y : b.y;
	r.z = a.z >= b.z ? a.z : b.z;
	r.w = a.w >= b.w ? a.w : b.w;
	return r;
}

// a = clamp(a, -b, b)
static inline b2FloatW b2SymClampW( b2FloatW a, b2FloatW b )
{
	b2FloatW r;
	r.x = b2ClampFloat( a.x, -b.x, b.x );
	r.y = b2ClampFloat( a.y, -b.y, b.y );
	r.z = b2ClampFloat( a.z, -b.z, b.z );
	r.w = b2ClampFloat( a.w, -b.w, b.w );
	return r;
}

static inline b2FloatW b2AndW( b2FloatW a, b2FloatW b )
{
	b2FloatW r;
	r.x = a.x != 0.0f && b.x != 0.0f ? 1.0f : 0.0f;
	r.y = a.y != 0.0f && b.y != 0.0f ? 1.0f : 0.0f;
	r.z = a.z != 0.0f && b.z != 0.0f ? 1.0f : 0.0f;
	r.w = a.w != 0.0f && b.w != 0.0f ? 1.0f : 0.0f;
	return r;
}

static inline b2FloatW b2GreaterThanW( b2FloatW a, b2FloatW b )
{
	b2FloatW r;
	r.x = a.x > b.x ? 1.0f : 0.0f;
	r.y = a.y > b.y ? 1.0f : 0.0f;
	r.z = a.z > b.z ? 1.0f : 0.0f;
	r.w = a.w > b.w ? 1.0f : 0.0f;
	return r;
}

static inline b2FloatW b2LessThanW( b2FloatW a, b2FloatW b )
{
	b2FloatW r;
	r.x = a.x < b.x ? 1.0f : 0.0f;
	r.y = a.y < b.y ? 1.0f : 0.0f;
	r.z = a.z < b.z ? 1.0f : 0.0f;
	r.w = a.w < b.w ? 1.0f : 0.0f;
	return r;
}

static inline bool b2AnyTrueW( b2FloatW mask )
{
	return mask.x != 0.0f || mask.y != 0.0f || mask.z != 0.0f || mask.w != 0.0f;
}

static inline bool b2AllZeroW( b2FloatW a )
{
	return a.x == 0.0f && a.y == 0.0f && a.z == 0.0f && a.w == 0.0f;
}

static inline b2FloatW b2LoadU( const float* data )
{
	return b2FloatW{ data[0], data[1], data[2], data[3] };
}

static inline void b2StoreW( float* data, b2FloatW a )
{
	data[0] = a.x;
	data[1] = a.y;
	data[2] = a.z;
	data[3] = a.w;
}

// 按分量返回 mask ? b : a
static inline b2FloatW b2BlendW( b2FloatW a, b2FloatW b, b2FloatW mask )
{
	b2FloatW r;
	r.x = mask.x != 0.0f ? b.x : a.x;
	r.y = mask.y != 0.0f ? b.y : a.y;
	r.z = mask.z != 0.0f ? b.z : a.z;
	r.w = mask.w != 0.0f ? b.w : a.w;
	return r;
}

static inline b2FloatW b2UnpackLoW( b2FloatW a, b2FloatW b )
{
	return b2FloatW{ a.x, b.x, a.y, b.y };
}

static inline b2FloatW b2UnpackHiW( b2FloatW a, b2FloatW b )
{
	return b2FloatW{ a.z, b.z, a.w, b.w };
}

static inline b2FloatW b2SoftMaskW( const int* indexA, const int* indexB )
{
	b2FloatW r;
	r.x = indexA[0] == 0 || indexB[0] == 0 ? 1.0f : 0.0f;
	r.y = indexA[1] == 0 || indexB[1] == 0 ? 1.0f : 0.0f;
	r.z = indexA[2] == 0 || indexB[2] == 0 ? 1.0f : 0.0f;
	r.w = indexA[3] == 0 || indexB[3] == 0 ? 1.0f : 0.0f;
	return r;
}
#endif

// a - b
static inline b2Vec2W b2SubVW( b2Vec2W a, b2Vec2W b )
{
	return b2Vec2W{
		.X = b2SubW( a.X, b.X ),
		.Y = b2SubW( a.Y, b.Y ),
	};
}

static inline b2Vec2W b2RightPerpW( b2Vec2W a )
{
	return b2Vec2W{ .X = a.Y, .Y = b2NegW( a.X ) };
}

static inline b2FloatW b2DotW( b2Vec2W a, b2Vec2W b )
{
	return b2AddW( b2MulW( a.X, b.X ), b2MulW( a.Y, b.Y ) );
}

static inline b2FloatW b2CrossW( b2Vec2W a, b2Vec2W b )
{
	return b2SubW( b2MulW( a.X, b.Y ), b2MulW( a.Y, b.X ) );
}

static inline b2Vec2W b2RotateVectorW( b2RotW q, b2Vec2W v )
{
	return b2Vec2W{ b2SubW( b2MulW( q.C, v.X ), b2MulW( q.S, v.Y ) ), b2AddW( b2MulW( q.S, v.X ), b2MulW( q.C, v.Y ) ) };
}

// 支持子步的软接触约束。
// Jacobian 使用固定锚点，滚动形状（圆和胶囊）表现更好。
// http://mmacklin.com/smallsteps.pdf
// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf

typedef struct b2ContactConstraintWide
{
	// 公共读取
	int indexA[B2_SIMD_WIDTH];
	int indexB[B2_SIMD_WIDTH];

	b2FloatW invMassA, invMassB;
	b2FloatW invIA, invIB;
	b2Vec2W normal;
	b2Vec2W anchorA1, anchorB1;
	b2Vec2W anchorA2, anchorB2;
	b2FloatW normalMass1, normalMass2;
	b2FloatW baseSeparation1;
	b2FloatW baseSeparation2;

	// 在 push 和 solve 中写入
	b2FloatW normalImpulse1, normalImpulse2;
	b2FloatW totalNormalImpulse1, totalNormalImpulse2;

	// 在 solve 中写入
	b2FloatW tangentImpulse1, tangentImpulse2;
	b2FloatW rollingImpulse;

	// 在 solve 中读取
	b2FloatW friction;
	b2FloatW tangentSpeed;
	b2FloatW rollingResistance;
	b2FloatW tangentMass1, tangentMass2;
	b2FloatW restitution;
	b2FloatW relativeVelocity1, relativeVelocity2;
	b2FloatW restitutionImpulse1, restitutionImpulse2;

} b2ContactConstraintWide;

int B2_WIDE( b2GetWideContactConstraintByteCount )( void )
{
	return sizeof( b2ContactConstraintWide );
}

// b2BodyState 的宽版本。
typedef struct b2BodyStateW
{
	b2Vec2W v;
	b2FloatW w;
	b2FloatW flags;
	b2Vec2W dp;
	b2RotW dq;
} b2BodyStateW;

// 每种 SIMD 类型的自定义 gather/scatter。
#if defined( B2_SIMD_NEON )

// 这是一次加载和转置
B2_FORCE_INLINE b2BodyStateW b2GatherBodies( const b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices )
{
	static_assert( sizeof( b2BodyState ) == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( (uintptr_t)states & 0x1F ) == 0 );

	// [vx vy w flags]
	b2FloatW identityA = b2ZeroW();

	// [dpx dpy dqc dqs]

	b2FloatW identityB = b2SetW( 0.0f, 0.0f, 1.0f, 0.0f );

	// 零表示空
	int i1 = indices[0] - 1;
	int i2 = indices[1] - 1;
	int i3 = indices[2] - 1;
	int i4 = indices[3] - 1;

	b2FloatW b1a = i1 == B2_NULL_INDEX ? identityA : vld1q_f32( (float*)( states + i1 ) + 0 );
	b2FloatW b1b = i1 == B2_NULL_INDEX ? identityB : vld1q_f32( (float*)( states + i1 ) + 4 );
	b2FloatW b2a = i2 == B2_NULL_INDEX ? identityA : vld1q_f32( (float*)( states + i2 ) + 0 );
	b2FloatW b2b = i2 == B2_NULL_INDEX ? identityB : vld1q_f32( (float*)( states + i2 ) + 4 );
	b2FloatW b3a = i3 == B2_NULL_INDEX ? identityA : vld1q_f32( (float*)( states + i3 ) + 0 );
	b2FloatW b3b = i3 == B2_NULL_INDEX ? identityB : vld1q_f32( (float*)( states + i3 ) + 4 );
	b2FloatW b4a = i4 == B2_NULL_INDEX ? identityA : vld1q_f32( (float*)( states + i4 ) + 0 );
	b2FloatW b4b = i4 == B2_NULL_INDEX ? identityB : vld1q_f32( (float*)( states + i4 ) + 4 );

	// [vx1 vx3 vy1 vy3]
	b2FloatW t1a = b2UnpackLoW( b1a, b3a );

	// [vx2 vx4 vy2 vy4]
	b2FloatW t2a = b2UnpackLoW( b2a, b4a );

	// [w1 w3 f1 f3]
	b2FloatW t3a = b2UnpackHiW( b1a, b3a );

	// [w2 w4 f2 f4]
	b2FloatW t4a = b2UnpackHiW( b2a, b4a );

	b2BodyStateW simdBody;
	simdBody.v.X = b2UnpackLoW( t1a, t2a );
	simdBody.v.Y = b2UnpackHiW( t1a, t2a );
	simdBody.w = b2UnpackLoW( t3a, t4a );
	simdBody.flags = b2UnpackHiW( t3a, t4a );

	b2FloatW t1b = b2UnpackLoW( b1b, b3b );
	b2FloatW t2b = b2UnpackLoW( b2b, b4b );
	b2FloatW t3b = b2UnpackHiW( b1b, b3b );
	b2FloatW t4b = b2UnpackHiW( b2b, b4b );

	simdBody.dp.X = b2UnpackLoW( t1b, t2b );
	simdBody.dp.Y = b2UnpackHiW( t1b, t2b );
	simdBody.dq.C = b2UnpackLoW( t3b, t4b );
	simdBody.dq.S = b2UnpackHiW( t3b, t4b );

	return simdBody;
}

// 只把速度写回求解刚体
// https://developer.arm.com/documentation/102107a/0100/Floating-point-4x4-matrix-transposition
B2_FORCE_INLINE void b2ScatterBodies( b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices,
									  const b2BodyStateW* B2_RESTRICT simdBody )
{
	static_assert( sizeof( b2BodyState ) == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( (uintptr_t)states & 0x1F ) == 0 );

	//	b2FloatW x = b2SetW(0.0f, 1.0f, 2.0f, 3.0f);
	//	b2FloatW y = b2SetW(4.0f, 5.0f, 6.0f, 7.0f);
	//	b2FloatW z = b2SetW(8.0f, 9.0f, 10.0f, 11.0f);
	//	b2FloatW w = b2SetW(12.0f, 13.0f, 14.0f, 15.0f);
	//
	//	float32x4x2_t rr1 = vtrnq_f32( x, y );
	//	float32x4x2_t rr2 = vtrnq_f32( z, w );
	//
	//	float32x4_t b1 = vcombine_f32(vget_low_f32(rr1.val[0]), vget_low_f32(rr2.val[0]));
	//	float32x4_t b2 = vcombine_f32(vget_low_f32(rr1.val[1]), vget_low_f32(rr2.val[1]));
	//	float32x4_t b3 = vcombine_f32(vget_high_f32(rr1.val[0]), vget_high_f32(rr2.val[0]));
	//	float32x4_t b4 = vcombine_f32(vget_high_f32(rr1.val[1]), vget_high_f32(rr2.val[1]));

	// 转置
	float32x4x2_t r1 = vtrnq_f32( simdBody->v.X, simdBody->v.Y );
	float32x4x2_t r2 = vtrnq_f32( simdBody->w, simdBody->flags );

	// 零表示空
	int i1 = indices[0] - 1;
	int i2 = indices[1] - 1;
	int i3 = indices[2] - 1;
	int i4 = indices[3] - 1;

	// 刚体数组中不用哑刚体，否则会导致多线程共享及相应的缓存刷新。
	if ( i1 != B2_NULL_INDEX && ( states[i1].flags & b2_dynamicFlag ) != 0 )
	{
		float32x4_t body1 = vcombine_f32( vget_low_f32( r1.val[0] ), vget_low_f32( r2.val[0] ) );
		b2StoreW( (float*)( states + i1 ), body1 );
	}

	if ( i2 != B2_NULL_INDEX && ( states[i2].flags & b2_dynamicFlag ) != 0 )
	{
		float32x4_t body2 = vcombine_f32( vget_low_f32( r1.val[1] ), vget_low_f32( r2.val[1] ) );
		b2StoreW( (float*)( states + i2 ), body2 );
	}

	if ( i3 != B2_NULL_INDEX && ( states[i3].flags & b2_dynamicFlag ) != 0 )
	{
		float32x4_t body3 = vcombine_f32( vget_high_f32( r1.val[0] ), vget_high_f32( r2.val[0] ) );
		b2StoreW( (float*)( states + i3 ), body3 );
	}

	if ( i4 != B2_NULL_INDEX && ( states[i4].flags & b2_dynamicFlag ) != 0 )
	{
		float32x4_t body4 = vcombine_f32( vget_high_f32( r1.val[1] ), vget_high_f32( r2.val[1] ) );
		b2StoreW( (float*)( states + i4 ), body4 );
	}
}

#elif defined( B2_SIMD_SSE2 )

// 这是一次加载和转置。强制内联对 MSVC 收益很大，Clang 已经内联。
B2_FORCE_INLINE b2BodyStateW b2GatherBodies( const b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices )
{
	static_assert( sizeof( b2BodyState ) == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( (uintptr_t)states & 0x1F ) == 0 );
	B2_VALIDATE( indices[0] >= 0 && indices[1] >= 0 && indices[2] >= 0 && indices[3] >= 0 );

	// [vx vy w flags]
	b2FloatW identityA = b2ZeroW();

	// [dpx dpy dqc dqs]
	b2FloatW identityB = b2SetW( 0.0f, 0.0f, 1.0f, 0.0f );

	// 零表示空
	int i1 = indices[0] - 1;
	int i2 = indices[1] - 1;
	int i3 = indices[2] - 1;
	int i4 = indices[3] - 1;

	b2FloatW b1a = i1 == B2_NULL_INDEX ? identityA : _mm_load_ps( (float*)( states + i1 ) + 0 );
	b2FloatW b1b = i1 == B2_NULL_INDEX ? identityB : _mm_load_ps( (float*)( states + i1 ) + 4 );
	b2FloatW b2a = i2 == B2_NULL_INDEX ? identityA : _mm_load_ps( (float*)( states + i2 ) + 0 );
	b2FloatW b2b = i2 == B2_NULL_INDEX ? identityB : _mm_load_ps( (float*)( states + i2 ) + 4 );
	b2FloatW b3a = i3 == B2_NULL_INDEX ? identityA : _mm_load_ps( (float*)( states + i3 ) + 0 );
	b2FloatW b3b = i3 == B2_NULL_INDEX ? identityB : _mm_load_ps( (float*)( states + i3 ) + 4 );
	b2FloatW b4a = i4 == B2_NULL_INDEX ? identityA : _mm_load_ps( (float*)( states + i4 ) + 0 );
	b2FloatW b4b = i4 == B2_NULL_INDEX ? identityB : _mm_load_ps( (float*)( states + i4 ) + 4 );

	// [vx1 vx3 vy1 vy3]
	b2FloatW t1a = b2UnpackLoW( b1a, b3a );

	// [vx2 vx4 vy2 vy4]
	b2FloatW t2a = b2UnpackLoW( b2a, b4a );

	// [w1 w3 f1 f3]
	b2FloatW t3a = b2UnpackHiW( b1a, b3a );

	// [w2 w4 f2 f4]
	b2FloatW t4a = b2UnpackHiW( b2a, b4a );

	b2BodyStateW simdBody;
	simdBody.v.X = b2UnpackLoW( t1a, t2a );
	simdBody.v.Y = b2UnpackHiW( t1a, t2a );
	simdBody.w = b2UnpackLoW( t3a, t4a );
	simdBody.flags = b2UnpackHiW( t3a, t4a );

	b2FloatW t1b = b2UnpackLoW( b1b, b3b );
	b2FloatW t2b = b2UnpackLoW( b2b, b4b );
	b2FloatW t3b = b2UnpackHiW( b1b, b3b );
	b2FloatW t4b = b2UnpackHiW( b2b, b4b );

	simdBody.dp.X = b2UnpackLoW( t1b, t2b );
	simdBody.dp.Y = b2UnpackHiW( t1b, t2b );
	simdBody.dq.C = b2UnpackLoW( t3b, t4b );
	simdBody.dq.S = b2UnpackHiW( t3b, t4b );

	return simdBody;
}

// 只把速度写回求解刚体。强制内联对 MSVC 收益很大，Clang 已经内联。
B2_FORCE_INLINE void b2ScatterBodies( b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices,
									  const b2BodyStateW* B2_RESTRICT simdBody )
{
	static_assert( sizeof( b2BodyState ) == 32, "b2BodyState not 32 bytes" );
	B2_ASSERT( ( (uintptr_t)states & 0x1F ) == 0 );
	B2_VALIDATE( indices[0] >= 0 && indices[1] >= 0 && indices[2] >= 0 && indices[3] >= 0 );

	// [vx1 vy1 vx2 vy2]
	b2FloatW t1 = b2UnpackLoW( simdBody->v.X, simdBody->v.Y );
	// [vx3 vy3 vx4 vy4]
	b2FloatW t2 = b2UnpackHiW( simdBody->v.X, simdBody->v.Y );
	// [w1 f1 w2 f2]
	b2FloatW t3 = b2UnpackLoW( simdBody->w, simdBody->flags );
	// [w3 f3 w4 f4]
	b2FloatW t4 = b2UnpackHiW( simdBody->w, simdBody->flags );

	static_assert( b2_dynamicFlag == ( 1 << 9 ), "dynamic flag must be bit 9" );
	int dynamicMask = _mm_movemask_ps( _mm_castsi128_ps( _mm_slli_epi32( _mm_castps_si128( simdBody->flags ), 31 - 9 ) ) );

	// 零表示空
	int i1 = indices[0] - 1;
	int i2 = indices[1] - 1;
	int i3 = indices[2] - 1;
	int i4 = indices[3] - 1;

	// 刚体数组中不用哑刚体，否则会导致多线程缓存一致性问题。
	if ( dynamicMask & 1 )
	{
		// [t1.x t1.y t3.x t3.y]
		b2StoreW( (float*)( states + i1 ), _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) ) );
	}

	if ( dynamicMask & 2 )
	{
		// [t1.z t1.w t3.z t3.w]
		b2StoreW( (float*)( states + i2 ), _mm_shuffle_ps( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) ) );
	}

	if ( dynamicMask & 4 )
	{
		// [t2.x t2.y t4.x t4.y]
		b2StoreW( (float*)( states + i3 ), _mm_shuffle_ps( t2, t4, _MM_SHUFFLE( 1, 0, 1, 0 ) ) );
	}

	if ( dynamicMask & 8 )
	{
		// [t2.z t2.w t4.z t4.w]
		b2StoreW( (float*)( states + i4 ), _mm_shuffle_ps( t2, t4, _MM_SHUFFLE( 3, 2, 3, 2 ) ) );
	}
}

#else

// 这是一次加载和转置
B2_FORCE_INLINE b2BodyStateW b2GatherBodies( const b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices )
{
	B2_VALIDATE( indices[0] >= 0 && indices[1] >= 0 && indices[2] >= 0 && indices[3] >= 0 );

	b2BodyState identity = b2_identityBodyState;

	// 零表示空
	int i1 = indices[0] - 1;
	int i2 = indices[1] - 1;
	int i3 = indices[2] - 1;
	int i4 = indices[3] - 1;

	b2BodyState s1 = i1 == B2_NULL_INDEX ? identity : states[i1];
	b2BodyState s2 = i2 == B2_NULL_INDEX ? identity : states[i2];
	b2BodyState s3 = i3 == B2_NULL_INDEX ? identity : states[i3];
	b2BodyState s4 = i4 == B2_NULL_INDEX ? identity : states[i4];

	b2BodyStateW simdBody;
	simdBody.v.X = b2FloatW{ s1.linearVelocity.x, s2.linearVelocity.x, s3.linearVelocity.x, s4.linearVelocity.x };
	simdBody.v.Y = b2FloatW{ s1.linearVelocity.y, s2.linearVelocity.y, s3.linearVelocity.y, s4.linearVelocity.y };
	simdBody.w = b2FloatW{ s1.angularVelocity, s2.angularVelocity, s3.angularVelocity, s4.angularVelocity };
	simdBody.flags = b2FloatW{ (float)s1.flags, (float)s2.flags, (float)s3.flags, (float)s4.flags };
	simdBody.dp.X = b2FloatW{ s1.deltaPosition.x, s2.deltaPosition.x, s3.deltaPosition.x, s4.deltaPosition.x };
	simdBody.dp.Y = b2FloatW{ s1.deltaPosition.y, s2.deltaPosition.y, s3.deltaPosition.y, s4.deltaPosition.y };
	simdBody.dq.C = b2FloatW{ s1.deltaRotation.c, s2.deltaRotation.c, s3.deltaRotation.c, s4.deltaRotation.c };
	simdBody.dq.S = b2FloatW{ s1.deltaRotation.s, s2.deltaRotation.s, s3.deltaRotation.s, s4.deltaRotation.s };

	return simdBody;
}

// 只把速度写回求解刚体
B2_FORCE_INLINE void b2ScatterBodies( b2BodyState* B2_RESTRICT states, int* B2_RESTRICT indices,
									  const b2BodyStateW* B2_RESTRICT simdBody )
{
	B2_VALIDATE( indices[0] >= 0 && indices[1] >= 0 && indices[2] >= 0 && indices[3] >= 0 );

	// 零表示空
	int i1 = indices[0] - 1;
	int i2 = indices[1] - 1;
	int i3 = indices[2] - 1;
	int i4 = indices[3] - 1;

	if ( i1 != B2_NULL_INDEX && ( states[i1].flags & b2_dynamicFlag ) != 0 )
	{
		b2BodyState* state = states + i1;
		state->linearVelocity.x = simdBody->v.X.x;
		state->linearVelocity.y = simdBody->v.Y.x;
		state->angularVelocity = simdBody->w.x;
	}

	if ( i2 != B2_NULL_INDEX && ( states[i2].flags & b2_dynamicFlag ) != 0 )
	{
		b2BodyState* state = states + i2;
		state->linearVelocity.x = simdBody->v.X.y;
		state->linearVelocity.y = simdBody->v.Y.y;
		state->angularVelocity = simdBody->w.y;
	}

	if ( i3 != B2_NULL_INDEX && ( states[i3].flags & b2_dynamicFlag ) != 0 )
	{
		b2BodyState* state = states + i3;
		state->linearVelocity.x = simdBody->v.X.z;
		state->linearVelocity.y = simdBody->v.Y.z;
		state->angularVelocity = simdBody->w.z;
	}

	if ( i4 != B2_NULL_INDEX && ( states[i4].flags & b2_dynamicFlag ) != 0 )
	{
		b2BodyState* state = states + i4;
		state->linearVelocity.x = simdBody->v.X.w;
		state->linearVelocity.y = simdBody->v.Y.w;
		state->angularVelocity = simdBody->w.w;
	}
}

#endif

static b2ContactSim b2_zeroContactSim = {};

// 跨通道收集接触数据的宏。
#define B2_GATHER_LANES( wide, scalar )                                                                                          \
	wide = b2SetW( contactLanes[0]->scalar, contactLanes[1]->scalar, contactLanes[2]->scalar, contactLanes[3]->scalar )

// 转置 gather 所需的布局
static_assert( offsetof( b2ManifoldPoint, anchorA ) == 0, "manifold point layout" );
static_assert( offsetof( b2ManifoldPoint, anchorB ) == 8, "manifold point layout" );
static_assert( offsetof( b2ManifoldPoint, separation ) == 16, "manifold point layout" );
static_assert( offsetof( b2ManifoldPoint, normalImpulse ) == 20, "manifold point layout" );
static_assert( offsetof( b2ManifoldPoint, tangentImpulse ) == 24, "manifold point layout" );
static_assert( sizeof( b2ManifoldPoint ) >= 32, "manifold point layout" );

// 准备宽接触约束。
// 潜在优化：对 SSE2 加载 4 个流形点并转置，类似刚体 gather/scatter。
// 需要重排流形点使约束数据在前。
void B2_WIDE( b2PrepareContacts_Wide )( b2SolverBlock block, b2StepContext* context )
{

	b2World* world = context->world;
	b2BodyState* states = context->states;
	b2ContactPrepareSpan* spans = context->contactPrepareSpans;
	b2ContactConstraintWide* wideBase = static_cast<b2ContactConstraintWide *>( context->wideContactConstraints );

#if B2_ENABLE_VALIDATION
	b2Body* bodies = world->bodies.data;
#endif

	b2FloatW zeroW = b2ZeroW();
	b2FloatW oneW = b2SplatW( 1.0f );
	b2FloatW warmStartScale = world->enableWarmStarting ? oneW : zeroW;
	bool anyRestitution = false;

	int wideIndex = block.startIndex;
	int endWideIndex = block.startIndex + block.count;

	// 为起始下标找颜色。线性搜索但很快。
	int colorIndex = 0;
	while ( spans[colorIndex + 1].start <= wideIndex )
	{
		colorIndex += 1;
	}

	// 遍历块
	while ( wideIndex < endWideIndex )
	{
		int colorWideStart = spans[colorIndex].start;
		int colorWideEndIndex = b2MinInt( spans[colorIndex + 1].start, endWideIndex );
		int colorContactCount = spans[colorIndex].count;
		b2ContactSim* contactSims = spans[colorIndex].contacts;

#if B2_ENABLE_VALIDATION
		int expectedWide = colorContactCount > 0 ? ( ( colorContactCount - 1 ) >> B2_SIMD_SHIFT ) + 1 : 0;
		B2_ASSERT( spans[colorIndex + 1].start - spans[colorIndex].start == expectedWide );
#endif

		// 遍历颜色
		for ( ; wideIndex < colorWideEndIndex; ++wideIndex )
		{
			b2ContactConstraintWide* cw = wideBase + wideIndex;
			int localWideIndex = wideIndex - colorWideStart;

			b2ContactSim* contactLanes[B2_SIMD_WIDTH];
			uint32_t hitEvents = 0;

			for ( int laneIndex = 0; laneIndex < B2_SIMD_WIDTH; ++laneIndex )
			{
				int contactIndex = B2_SIMD_WIDTH * localWideIndex + laneIndex;
				if ( contactIndex < colorContactCount )
				{
					b2ContactSim* c = contactSims + contactIndex;
					contactLanes[laneIndex] = c;

					// 下标从 1 起
					cw->indexA[laneIndex] = b2DecodeAwakeIndex( c->encodedBodySimA ) + 1;
					cw->indexB[laneIndex] = b2DecodeAwakeIndex( c->encodedBodySimB ) + 1;
					hitEvents |= ( c->simFlags & b2_simEnableHitEvent );

#if B2_ENABLE_VALIDATION
					b2Body* bodyA = bodies + c->bodyIdA;
					b2Body* bodyB = bodies + c->bodyIdB;
					B2_ASSERT( c->encodedBodySimA == b2EncodeBodySimIndex( bodyA ) );
					B2_ASSERT( c->encodedBodySimB == b2EncodeBodySimIndex( bodyB ) );
#endif
				}
				else
				{
					contactLanes[laneIndex] = &b2_zeroContactSim;
				}
			}

			B2_GATHER_LANES( cw->invMassA, invMassA );
			B2_GATHER_LANES( cw->invMassB, invMassB );
			B2_GATHER_LANES( cw->invIA, invIA );
			B2_GATHER_LANES( cw->invIB, invIB );
			B2_GATHER_LANES( cw->normal.X, manifold.normal.x );
			B2_GATHER_LANES( cw->normal.Y, manifold.normal.y );
			B2_GATHER_LANES( cw->friction, friction );
			B2_GATHER_LANES( cw->tangentSpeed, tangentSpeed );
			B2_GATHER_LANES( cw->rollingResistance, rollingResistance );
			B2_GATHER_LANES( cw->rollingImpulse, manifold.rollingImpulse );
			cw->rollingImpulse = b2MulW( warmStartScale, cw->rollingImpulse );

			B2_GATHER_LANES( cw->restitution, restitution );

			b2Vec2W tangent = b2RightPerpW( cw->normal );

			b2FloatW pointCounts;
			pointCounts = b2SetW( (float)contactLanes[0]->manifold.pointCount, (float)contactLanes[1]->manifold.pointCount,
								  (float)contactLanes[2]->manifold.pointCount, (float)contactLanes[3]->manifold.pointCount );
			b2FloatW twoPointMask = b2GreaterThanW( pointCounts, oneW );

			{
				// 优化的转置 gather。类似 b2GatherBodies。
				b2FloatW m1a = b2LoadU( (float*)( contactLanes[0]->manifold.points + 0 ) + 0 );
				b2FloatW m1b = b2LoadU( (float*)( contactLanes[0]->manifold.points + 0 ) + 4 );
				b2FloatW m2a = b2LoadU( (float*)( contactLanes[1]->manifold.points + 0 ) + 0 );
				b2FloatW m2b = b2LoadU( (float*)( contactLanes[1]->manifold.points + 0 ) + 4 );
				b2FloatW m3a = b2LoadU( (float*)( contactLanes[2]->manifold.points + 0 ) + 0 );
				b2FloatW m3b = b2LoadU( (float*)( contactLanes[2]->manifold.points + 0 ) + 4 );
				b2FloatW m4a = b2LoadU( (float*)( contactLanes[3]->manifold.points + 0 ) + 0 );
				b2FloatW m4b = b2LoadU( (float*)( contactLanes[3]->manifold.points + 0 ) + 4 );

				b2FloatW t1a = b2UnpackLoW( m1a, m3a );
				b2FloatW t2a = b2UnpackLoW( m2a, m4a );
				b2FloatW t3a = b2UnpackHiW( m1a, m3a );
				b2FloatW t4a = b2UnpackHiW( m2a, m4a );
				b2FloatW t1b = b2UnpackLoW( m1b, m3b );
				b2FloatW t2b = b2UnpackLoW( m2b, m4b );
				b2FloatW t3b = b2UnpackHiW( m1b, m3b );
				b2FloatW t4b = b2UnpackHiW( m2b, m4b );

				cw->anchorA1.X = b2UnpackLoW( t1a, t2a );
				cw->anchorA1.Y = b2UnpackHiW( t1a, t2a );
				cw->anchorB1.X = b2UnpackLoW( t3a, t4a );
				cw->anchorB1.Y = b2UnpackHiW( t3a, t4a );
				cw->baseSeparation1 = b2UnpackLoW( t1b, t2b );
				cw->normalImpulse1 = b2UnpackHiW( t1b, t2b );
				cw->tangentImpulse1 = b2UnpackLoW( t3b, t4b );

				// separation1 = cw->baseSeparation1;

				b2FloatW offset = b2DotW( b2SubVW( cw->anchorB1, cw->anchorA1 ), cw->normal );
				cw->baseSeparation1 = b2SubW( cw->baseSeparation1, offset );
				cw->normalImpulse1 = b2MulW( warmStartScale, cw->normalImpulse1 );
				cw->tangentImpulse1 = b2MulW( warmStartScale, cw->tangentImpulse1 );
				cw->totalNormalImpulse1 = zeroW;
				cw->relativeVelocity1 = zeroW;
				cw->restitutionImpulse1 = zeroW;

				// 沿法向的有效质量。
				{
					b2FloatW rnA = b2CrossW( cw->anchorA1, cw->normal );
					b2FloatW rnB = b2CrossW( cw->anchorB1, cw->normal );
					b2FloatW k = b2AddW( cw->invMassA, cw->invMassB );
					k = b2MulAddW( k, cw->invIA, b2MulW( rnA, rnA ) );
					k = b2MulAddW( k, cw->invIB, b2MulW( rnB, rnB ) );
					b2FloatW test = b2GreaterThanW( k, b2ZeroW() );
					cw->normalMass1 = b2BlendW( b2ZeroW(), b2DivW( oneW, k ), test );
				}

				// 沿切向的有效质量。
				{
					b2FloatW rtA = b2CrossW( cw->anchorA1, tangent );
					b2FloatW rtB = b2CrossW( cw->anchorB1, tangent );
					b2FloatW k = b2AddW( cw->invMassA, cw->invMassB );
					k = b2MulAddW( k, cw->invIA, b2MulW( rtA, rtA ) );
					k = b2MulAddW( k, cw->invIB, b2MulW( rtB, rtB ) );
					b2FloatW test = b2GreaterThanW( k, b2ZeroW() );
					cw->tangentMass1 = b2BlendW( b2ZeroW(), b2DivW( oneW, k ), test );
				}
			}

			{
				b2FloatW m1a = b2LoadU( (float*)( contactLanes[0]->manifold.points + 1 ) + 0 );
				b2FloatW m1b = b2LoadU( (float*)( contactLanes[0]->manifold.points + 1 ) + 4 );
				b2FloatW m2a = b2LoadU( (float*)( contactLanes[1]->manifold.points + 1 ) + 0 );
				b2FloatW m2b = b2LoadU( (float*)( contactLanes[1]->manifold.points + 1 ) + 4 );
				b2FloatW m3a = b2LoadU( (float*)( contactLanes[2]->manifold.points + 1 ) + 0 );
				b2FloatW m3b = b2LoadU( (float*)( contactLanes[2]->manifold.points + 1 ) + 4 );
				b2FloatW m4a = b2LoadU( (float*)( contactLanes[3]->manifold.points + 1 ) + 0 );
				b2FloatW m4b = b2LoadU( (float*)( contactLanes[3]->manifold.points + 1 ) + 4 );

				b2FloatW t1a = b2UnpackLoW( m1a, m3a );
				b2FloatW t2a = b2UnpackLoW( m2a, m4a );
				b2FloatW t3a = b2UnpackHiW( m1a, m3a );
				b2FloatW t4a = b2UnpackHiW( m2a, m4a );
				b2FloatW t1b = b2UnpackLoW( m1b, m3b );
				b2FloatW t2b = b2UnpackLoW( m2b, m4b );
				b2FloatW t3b = b2UnpackHiW( m1b, m3b );
				b2FloatW t4b = b2UnpackHiW( m2b, m4b );

				cw->anchorA2.X = b2UnpackLoW( t1a, t2a );
				cw->anchorA2.Y = b2UnpackHiW( t1a, t2a );
				cw->anchorB2.X = b2UnpackLoW( t3a, t4a );
				cw->anchorB2.Y = b2UnpackHiW( t3a, t4a );
				cw->baseSeparation2 = b2UnpackLoW( t1b, t2b );
				cw->normalImpulse2 = b2UnpackHiW( t1b, t2b );
				cw->tangentImpulse2 = b2UnpackLoW( t3b, t4b );

				// separation2 = cw->baseSeparation2;

				b2FloatW offset = b2DotW( b2SubVW( cw->anchorB2, cw->anchorA2 ), cw->normal );
				cw->baseSeparation2 = b2SubW( cw->baseSeparation2, offset );
				cw->normalImpulse2 = b2MulW( warmStartScale, cw->normalImpulse2 );
				cw->tangentImpulse2 = b2MulW( warmStartScale, cw->tangentImpulse2 );
				cw->totalNormalImpulse2 = zeroW;
				cw->relativeVelocity2 = zeroW;
				cw->restitutionImpulse2 = zeroW;

				// 沿法向的有效质量。
				{
					b2FloatW rnA = b2CrossW( cw->anchorA2, cw->normal );
					b2FloatW rnB = b2CrossW( cw->anchorB2, cw->normal );
					b2FloatW k = b2AddW( cw->invMassA, cw->invMassB );
					k = b2MulAddW( k, cw->invIA, b2MulW( rnA, rnA ) );
					k = b2MulAddW( k, cw->invIB, b2MulW( rnB, rnB ) );
					b2FloatW test = b2GreaterThanW( k, b2ZeroW() );
					cw->normalMass2 = b2BlendW( b2ZeroW(), b2DivW( oneW, k ), test );
				}

				// 沿切向的有效质量。
				{
					b2FloatW rtA = b2CrossW( cw->anchorA2, tangent );
					b2FloatW rtB = b2CrossW( cw->anchorB2, tangent );
					b2FloatW k = b2AddW( cw->invMassA, cw->invMassB );
					k = b2MulAddW( k, cw->invIA, b2MulW( rtA, rtA ) );
					k = b2MulAddW( k, cw->invIB, b2MulW( rtB, rtB ) );
					b2FloatW test = b2GreaterThanW( k, b2ZeroW() );
					cw->tangentMass2 = b2BlendW( b2ZeroW(), b2DivW( oneW, k ), test );
				}

				// 只有一个点的通道上有效质量为零。
				cw->normalMass2 = b2BlendW( zeroW, cw->normalMass2, twoPointMask );
				cw->tangentMass2 = b2BlendW( zeroW, cw->tangentMass2, twoPointMask );
			}

			// 仅在需要时采样接触点法向速度。
			b2FloatW restitutionMask = b2GreaterThanW( cw->restitution, zeroW );
			bool haveRestitution = b2AnyTrueW( restitutionMask );
			anyRestitution = anyRestitution || haveRestitution;

			if ( ( hitEvents & b2_simEnableHitEvent ) || haveRestitution )
			{
				b2BodyStateW bA = b2GatherBodies( states, cw->indexA );
				b2BodyStateW bB = b2GatherBodies( states, cw->indexB );

				{
					b2Vec2W rA = cw->anchorA1;
					b2Vec2W rB = cw->anchorB1;

					// 接触处的相对速度
					b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
					b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
					b2FloatW vn = b2AddW( b2MulW( dvx, cw->normal.X ), b2MulW( dvy, cw->normal.Y ) );
					cw->relativeVelocity1 = vn;

					alignas( 32 ) float normalVelocities[B2_SIMD_WIDTH];
					b2StoreW( normalVelocities, vn );

					for ( int lane = 0; lane < B2_SIMD_WIDTH; ++lane )
					{
						// 检查点数以避免写入 b2_zeroContactSim 时产生竞争。
						if ( contactLanes[lane]->manifold.pointCount > 0 )
						{
							contactLanes[lane]->manifold.points[0].normalVelocity = normalVelocities[lane];
						}
					}
				}

				{
					b2Vec2W rA = cw->anchorA2;
					b2Vec2W rB = cw->anchorB2;

					// 接触处的相对速度
					b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
					b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
					b2FloatW vn = b2AddW( b2MulW( dvx, cw->normal.X ), b2MulW( dvy, cw->normal.Y ) );
					cw->relativeVelocity2 = vn;

					alignas( 32 ) float normalVelocities[B2_SIMD_WIDTH];
					b2StoreW( normalVelocities, vn );

					for ( int lane = 0; lane < B2_SIMD_WIDTH; ++lane )
					{
						if ( contactLanes[lane]->manifold.pointCount > 1 )
						{
							contactLanes[lane]->manifold.points[1].normalVelocity = normalVelocities[lane];
						}
					}
				}
			}
		}

		// 前进到下一颜色
		colorIndex += 1;
	}

	if ( anyRestitution )
	{
		b2AtomicStoreInt( &context->anyRestitution, 1 );
	}

}

void B2_WIDE( b2WarmStartContacts_Wide )( b2SolverBlock block, b2StepContext* context )
{


	b2BodyState* states = context->states;
	b2ContactConstraintWide* constraints = static_cast<b2ContactConstraintWide *>( context->graph->colors[block.colorIndex].wideConstraints );

	for ( int i = block.startIndex; i < block.startIndex + block.count; ++i )
	{
		b2ContactConstraintWide* c = constraints + i;
		b2BodyStateW bA = b2GatherBodies( states, c->indexA );
		b2BodyStateW bB = b2GatherBodies( states, c->indexB );

		b2FloatW tangentX = c->normal.Y;
		b2FloatW tangentY = b2SubW( b2ZeroW(), c->normal.X );

		{
			// 固定锚点
			b2Vec2W rA = c->anchorA1;
			b2Vec2W rB = c->anchorB1;

			b2Vec2W P;
			P.X = b2AddW( b2MulW( c->normalImpulse1, c->normal.X ), b2MulW( c->tangentImpulse1, tangentX ) );
			P.Y = b2AddW( b2MulW( c->normalImpulse1, c->normal.Y ), b2MulW( c->tangentImpulse1, tangentY ) );
			bA.w = b2MulSubW( bA.w, c->invIA, b2CrossW( rA, P ) );
			bA.v.X = b2MulSubW( bA.v.X, c->invMassA, P.X );
			bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, P.Y );
			bB.w = b2MulAddW( bB.w, c->invIB, b2CrossW( rB, P ) );
			bB.v.X = b2MulAddW( bB.v.X, c->invMassB, P.X );
			bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, P.Y );

			c->totalNormalImpulse1 = b2AddW( c->totalNormalImpulse1, c->normalImpulse1 );
		}

		{
			// 固定锚点
			b2Vec2W rA = c->anchorA2;
			b2Vec2W rB = c->anchorB2;

			b2Vec2W P;
			P.X = b2AddW( b2MulW( c->normalImpulse2, c->normal.X ), b2MulW( c->tangentImpulse2, tangentX ) );
			P.Y = b2AddW( b2MulW( c->normalImpulse2, c->normal.Y ), b2MulW( c->tangentImpulse2, tangentY ) );
			bA.w = b2MulSubW( bA.w, c->invIA, b2CrossW( rA, P ) );
			bA.v.X = b2MulSubW( bA.v.X, c->invMassA, P.X );
			bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, P.Y );
			bB.w = b2MulAddW( bB.w, c->invIB, b2CrossW( rB, P ) );
			bB.v.X = b2MulAddW( bB.v.X, c->invMassB, P.X );
			bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, P.Y );

			c->totalNormalImpulse2 = b2AddW( c->totalNormalImpulse2, c->normalImpulse2 );
		}

		bA.w = b2MulSubW( bA.w, c->invIA, c->rollingImpulse );
		bB.w = b2MulAddW( bB.w, c->invIB, c->rollingImpulse );

		b2ScatterBodies( states, c->indexA, &bA );
		b2ScatterBodies( states, c->indexB, &bB );
	}

}

void B2_WIDE( b2PushContacts_Wide )( b2SolverBlock block, b2StepContext* context )
{


	b2BodyState* states = context->states;
	b2GraphColor* color = context->graph->colors + block.colorIndex;
	b2ContactConstraintWide* constraints = static_cast<b2ContactConstraintWide *>( color->wideConstraints );
	b2FloatW inv_h = b2SplatW( context->inv_h );
	b2FloatW contactSpeed = b2SplatW( -context->world->contactSpeed );
	b2FloatW oneW = b2SplatW( 1.0f );
	b2FloatW zeroW = b2ZeroW();

	b2FloatW dynamicBiasRate = b2SplatW( context->contactSoftness.massScale * context->contactSoftness.biasRate );
	b2FloatW dynamicMassScale = b2SplatW( context->contactSoftness.massScale );
	b2FloatW dynamicImpulseScale = b2SplatW( context->contactSoftness.impulseScale );
	b2FloatW staticBiasRate = b2SplatW( context->staticSoftness.massScale * context->staticSoftness.biasRate );
	b2FloatW staticMassScale = b2SplatW( context->staticSoftness.massScale );
	b2FloatW staticImpulseScale = b2SplatW( context->staticSoftness.impulseScale );

	for ( int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; ++wideIndex )
	{
		b2ContactConstraintWide* c = constraints + wideIndex;

		b2BodyStateW bA = b2GatherBodies( states, c->indexA );
		b2BodyStateW bB = b2GatherBodies( states, c->indexB );

		b2FloatW softMask = b2SoftMaskW( c->indexA, c->indexB );
		b2FloatW biasRate = b2BlendW( dynamicBiasRate, staticBiasRate, softMask );
		b2FloatW massScale = b2BlendW( dynamicMassScale, staticMassScale, softMask );
		b2FloatW impulseScale = b2BlendW( dynamicImpulseScale, staticImpulseScale, softMask );

		b2Vec2W normal = c->normal;
		b2Vec2W dp = { b2SubW( bB.dp.X, bA.dp.X ), b2SubW( bB.dp.Y, bA.dp.Y ) };

		b2FloatW velocityBias1, massScale1, impulseScale1;
		{
			// 用于当前分离的移动锚点
			b2Vec2W rsA = b2RotateVectorW( bA.dq, c->anchorA1 );
			b2Vec2W rsB = b2RotateVectorW( bB.dq, c->anchorB1 );

			// 计算当前分离
			// 锚点远离刚体质心时会受舍入误差影响
			b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			b2FloatW s = b2AddW( b2DotW( normal, ds ), c->baseSeparation1 );

			// 分离大于零时用预测性偏置，否则用软约束偏置
			// contactSpeed 用于限制刚度，而非增加。
			b2FloatW separated = b2GreaterThanW( s, zeroW );

			// 预测性偏置 - 正
			b2FloatW specBias = b2MulW( s, inv_h );

			// 重叠偏置 - 重叠时为负
			b2FloatW overlapBias = b2MaxW( b2MulW( biasRate, s ), contactSpeed );

			// velocity bias = s > 0 ? spec : overlap
			velocityBias1 = b2BlendW( overlapBias, specBias, separated );

			massScale1 = b2BlendW( massScale, oneW, separated );
			impulseScale1 = b2BlendW( impulseScale, zeroW, separated );
		}

		b2FloatW velocityBias2, massScale2, impulseScale2;
		{
			b2Vec2W rsA = b2RotateVectorW( bA.dq, c->anchorA2 );
			b2Vec2W rsB = b2RotateVectorW( bB.dq, c->anchorB2 );

			b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			b2FloatW s = b2AddW( b2DotW( normal, ds ), c->baseSeparation2 );

			b2FloatW separated = b2GreaterThanW( s, zeroW );
			b2FloatW specBias = b2MulW( s, inv_h );
			b2FloatW overlapBias = b2MaxW( b2MulW( biasRate, s ), contactSpeed );
			velocityBias2 = b2BlendW( overlapBias, specBias, separated );

			massScale2 = b2BlendW( massScale, oneW, separated );
			impulseScale2 = b2BlendW( impulseScale, zeroW, separated );
		}

		b2FloatW rnA1 = b2CrossW( c->anchorA1, normal );
		b2FloatW rnB1 = b2CrossW( c->anchorB1, normal );
		b2FloatW rnA2 = b2CrossW( c->anchorA2, normal );
		b2FloatW rnB2 = b2CrossW( c->anchorB2, normal );

		b2FloatW k21 = b2AddW( b2AddW( c->invMassA, c->invMassB ),
							   b2AddW( b2MulW( b2MulW( c->invIA, rnA1 ), rnA2 ), b2MulW( b2MulW( c->invIB, rnB1 ), rnB2 ) ) );

		b2FloatW dn = b2DotW( b2SubVW( bB.v, bA.v ), normal );
		b2FloatW vn1 = b2AddW( dn, b2SubW( b2MulW( bB.w, rnB1 ), b2MulW( bA.w, rnA1 ) ) );
		b2FloatW vn2 = b2AddW( dn, b2SubW( b2MulW( bB.w, rnB2 ), b2MulW( bA.w, rnA2 ) ) );

		b2FloatW negImpulse1 = b2AddW( b2MulW( c->normalMass1, b2AddW( b2MulW( massScale1, vn1 ), velocityBias1 ) ),
									   b2MulW( impulseScale1, c->normalImpulse1 ) );
		b2FloatW impulse1 = b2MaxW( b2NegW( negImpulse1 ), b2NegW( c->normalImpulse1 ) );
		c->normalImpulse1 = b2AddW( c->normalImpulse1, impulse1 );
		c->totalNormalImpulse1 = b2AddW( c->totalNormalImpulse1, impulse1 );

		vn2 = b2MulAddW( vn2, k21, impulse1 );

		b2FloatW negImpulse2 = b2AddW( b2MulW( c->normalMass2, b2AddW( b2MulW( massScale2, vn2 ), velocityBias2 ) ),
									   b2MulW( impulseScale2, c->normalImpulse2 ) );
		b2FloatW impulse2 = b2MaxW( b2NegW( negImpulse2 ), b2NegW( c->normalImpulse2 ) );
		c->normalImpulse2 = b2AddW( c->normalImpulse2, impulse2 );
		c->totalNormalImpulse2 = b2AddW( c->totalNormalImpulse2, impulse2 );

		b2FloatW impulse = b2AddW( impulse1, impulse2 );
		b2FloatW Px = b2MulW( impulse, normal.X );
		b2FloatW Py = b2MulW( impulse, normal.Y );
		b2FloatW angularA = b2AddW( b2MulW( impulse1, rnA1 ), b2MulW( impulse2, rnA2 ) );
		b2FloatW angularB = b2AddW( b2MulW( impulse1, rnB1 ), b2MulW( impulse2, rnB2 ) );

		bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
		bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
		bA.w = b2MulSubW( bA.w, c->invIA, angularA );

		bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
		bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
		bB.w = b2MulAddW( bB.w, c->invIB, angularB );

		b2ScatterBodies( states, c->indexA, &bA );
		b2ScatterBodies( states, c->indexB, &bB );
	}

}

// 求解法向约束、摩擦和滚动阻力。
void B2_WIDE( b2SolveContacts_Wide )( b2SolverBlock block, b2StepContext* context )
{


	b2BodyState* states = context->states;
	b2GraphColor* color = context->graph->colors + block.colorIndex;
	b2ContactConstraintWide* constraints = static_cast<b2ContactConstraintWide *>( color->wideConstraints );
	b2FloatW inv_h = b2SplatW( context->inv_h );
	b2FloatW zeroW = b2ZeroW();

	for ( int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; ++wideIndex )
	{
		b2ContactConstraintWide* c = constraints + wideIndex;

		b2BodyStateW bA = b2GatherBodies( states, c->indexA );
		b2BodyStateW bB = b2GatherBodies( states, c->indexB );

		b2Vec2W normal = c->normal;
		b2Vec2W tangent = b2RightPerpW( normal );
		b2Vec2W dp = { b2SubW( bB.dp.X, bA.dp.X ), b2SubW( bB.dp.Y, bA.dp.Y ) };

		b2FloatW velocityBias1, velocityBias2;
		{
			// 用于当前分离的移动锚点
			b2Vec2W rsA = b2RotateVectorW( bA.dq, c->anchorA1 );
			b2Vec2W rsB = b2RotateVectorW( bB.dq, c->anchorB1 );

			// 计算当前分离
			// 锚点远离刚体质心时会受舍入误差影响
			b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			b2FloatW s = b2AddW( b2DotW( normal, ds ), c->baseSeparation1 );

			// 预测性偏置，分离时为正，重叠时为零
			velocityBias1 = b2MaxW( zeroW, b2MulW( s, inv_h ) );
		}

		{
			b2Vec2W rsA = b2RotateVectorW( bA.dq, c->anchorA2 );
			b2Vec2W rsB = b2RotateVectorW( bB.dq, c->anchorB2 );

			b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			b2FloatW s = b2AddW( b2DotW( normal, ds ), c->baseSeparation2 );

			velocityBias2 = b2MaxW( zeroW, b2MulW( s, inv_h ) );
		}

		b2FloatW rnA1 = b2CrossW( c->anchorA1, normal );
		b2FloatW rnB1 = b2CrossW( c->anchorB1, normal );
		b2FloatW rnA2 = b2CrossW( c->anchorA2, normal );
		b2FloatW rnB2 = b2CrossW( c->anchorB2, normal );
		b2FloatW rtA1 = b2CrossW( c->anchorA1, tangent );
		b2FloatW rtB1 = b2CrossW( c->anchorB1, tangent );
		b2FloatW rtA2 = b2CrossW( c->anchorA2, tangent );
		b2FloatW rtB2 = b2CrossW( c->anchorB2, tangent );

		b2FloatW wnA1 = b2MulW( c->invIA, rnA1 );
		b2FloatW wnB1 = b2MulW( c->invIB, rnB1 );
		b2FloatW wnA2 = b2MulW( c->invIA, rnA2 );
		b2FloatW wnB2 = b2MulW( c->invIB, rnB2 );
		b2FloatW wtA1 = b2MulW( c->invIA, rtA1 );
		b2FloatW wtB1 = b2MulW( c->invIB, rtB1 );

		b2FloatW invMass = b2AddW( c->invMassA, c->invMassB );
		b2FloatW k21 = b2AddW( invMass, b2AddW( b2MulW( wnA1, rnA2 ), b2MulW( wnB1, rnB2 ) ) );
		b2FloatW k31 = b2AddW( b2MulW( wnA1, rtA1 ), b2MulW( wnB1, rtB1 ) );
		b2FloatW k32 = b2AddW( b2MulW( wnA2, rtA1 ), b2MulW( wnB2, rtB1 ) );
		b2FloatW k41 = b2AddW( b2MulW( wnA1, rtA2 ), b2MulW( wnB1, rtB2 ) );
		b2FloatW k42 = b2AddW( b2MulW( wnA2, rtA2 ), b2MulW( wnB2, rtB2 ) );
		b2FloatW k43 = b2AddW( invMass, b2AddW( b2MulW( wtA1, rtA2 ), b2MulW( wtB1, rtB2 ) ) );

		b2Vec2W dv = b2SubVW( bB.v, bA.v );
		b2FloatW dn = b2DotW( dv, normal );
		b2FloatW dt = b2SubW( b2DotW( dv, tangent ), c->tangentSpeed );

		b2FloatW vn1 = b2AddW( dn, b2SubW( b2MulW( bB.w, rnB1 ), b2MulW( bA.w, rnA1 ) ) );
		b2FloatW vn2 = b2AddW( dn, b2SubW( b2MulW( bB.w, rnB2 ), b2MulW( bA.w, rnA2 ) ) );
		b2FloatW vt1 = b2AddW( dt, b2SubW( b2MulW( bB.w, rtB1 ), b2MulW( bA.w, rtA1 ) ) );
		b2FloatW vt2 = b2AddW( dt, b2SubW( b2MulW( bB.w, rtB2 ), b2MulW( bA.w, rtA2 ) ) );

		b2FloatW normalImpulse1 = b2MaxW( b2NegW( b2MulW( c->normalMass1, b2AddW( vn1, velocityBias1 ) ) ),
										  b2NegW( c->normalImpulse1 ) );
		c->normalImpulse1 = b2AddW( c->normalImpulse1, normalImpulse1 );
		c->totalNormalImpulse1 = b2AddW( c->totalNormalImpulse1, normalImpulse1 );

		vn2 = b2MulAddW( vn2, k21, normalImpulse1 );

		b2FloatW normalImpulse2 = b2MaxW( b2NegW( b2MulW( c->normalMass2, b2AddW( vn2, velocityBias2 ) ) ),
										  b2NegW( c->normalImpulse2 ) );
		c->normalImpulse2 = b2AddW( c->normalImpulse2, normalImpulse2 );
		c->totalNormalImpulse2 = b2AddW( c->totalNormalImpulse2, normalImpulse2 );

		vt1 = b2MulAddW( b2MulAddW( vt1, k31, normalImpulse1 ), k32, normalImpulse2 );
		vt2 = b2MulAddW( b2MulAddW( vt2, k41, normalImpulse1 ), k42, normalImpulse2 );

		b2FloatW angularA = b2AddW( b2MulW( normalImpulse1, rnA1 ), b2MulW( normalImpulse2, rnA2 ) );
		b2FloatW angularB = b2AddW( b2MulW( normalImpulse1, rnB1 ), b2MulW( normalImpulse2, rnB2 ) );

		// 滚动阻力
		if ( b2AllZeroW( c->rollingResistance ) == false )
		{
			b2FloatW relativeW = b2SubW( b2SubW( bA.w, bB.w ), b2AddW( b2MulW( b2AddW( wnA1, wnB1 ), normalImpulse1 ),
																	  b2MulW( b2AddW( wnA2, wnB2 ), normalImpulse2 ) ) );
			b2FloatW k = b2AddW( c->invIA, c->invIB );
			b2FloatW mask = b2GreaterThanW( k, zeroW );
			b2FloatW deltaLambda = b2BlendW( zeroW, b2DivW( relativeW, k ), mask );
			b2FloatW lambda = c->rollingImpulse;
			b2FloatW maxLambda = b2MulW( c->rollingResistance, b2AddW( c->normalImpulse1, c->normalImpulse2 ) );
			c->rollingImpulse = b2SymClampW( b2AddW( lambda, deltaLambda ), maxLambda );
			deltaLambda = b2SubW( c->rollingImpulse, lambda );

			b2FloatW wtA2 = b2MulW( c->invIA, rtA2 );
			b2FloatW wtB2 = b2MulW( c->invIB, rtB2 );
			vt1 = b2MulAddW( vt1, b2AddW( wtA1, wtB1 ), deltaLambda );
			vt2 = b2MulAddW( vt2, b2AddW( wtA2, wtB2 ), deltaLambda );

			angularA = b2AddW( angularA, deltaLambda );
			angularB = b2AddW( angularB, deltaLambda );
		}

		b2FloatW maxFriction1 = b2MulW( c->friction, c->normalImpulse1 );
		b2FloatW tangentImpulse1 = b2NegW( b2MulW( c->tangentMass1, vt1 ) );
		tangentImpulse1 = b2MaxW( b2SubW( b2NegW( maxFriction1 ), c->tangentImpulse1 ),
								  b2MinW( tangentImpulse1, b2SubW( maxFriction1, c->tangentImpulse1 ) ) );
		c->tangentImpulse1 = b2AddW( c->tangentImpulse1, tangentImpulse1 );

		vt2 = b2MulAddW( vt2, k43, tangentImpulse1 );

		b2FloatW maxFriction2 = b2MulW( c->friction, c->normalImpulse2 );
		b2FloatW tangentImpulse2 = b2NegW( b2MulW( c->tangentMass2, vt2 ) );
		tangentImpulse2 = b2MaxW( b2SubW( b2NegW( maxFriction2 ), c->tangentImpulse2 ),
								  b2MinW( tangentImpulse2, b2SubW( maxFriction2, c->tangentImpulse2 ) ) );
		c->tangentImpulse2 = b2AddW( c->tangentImpulse2, tangentImpulse2 );

		angularA = b2AddW( angularA, b2AddW( b2MulW( tangentImpulse1, rtA1 ), b2MulW( tangentImpulse2, rtA2 ) ) );
		angularB = b2AddW( angularB, b2AddW( b2MulW( tangentImpulse1, rtB1 ), b2MulW( tangentImpulse2, rtB2 ) ) );

		b2FloatW normalImpulse = b2AddW( normalImpulse1, normalImpulse2 );
		b2FloatW tangentImpulse = b2AddW( tangentImpulse1, tangentImpulse2 );
		b2FloatW Px = b2AddW( b2MulW( normalImpulse, normal.X ), b2MulW( tangentImpulse, tangent.X ) );
		b2FloatW Py = b2AddW( b2MulW( normalImpulse, normal.Y ), b2MulW( tangentImpulse, tangent.Y ) );

		bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
		bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
		bA.w = b2MulSubW( bA.w, c->invIA, angularA );

		bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
		bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
		bB.w = b2MulAddW( bB.w, c->invIB, angularB );

		b2ScatterBodies( states, c->indexA, &bA );
		b2ScatterBodies( states, c->indexB, &bB );
	}

}

void B2_WIDE( b2ApplyRestitution_Wide )( b2SolverBlock block, b2StepContext* context )
{


	b2BodyState* states = context->states;
	b2GraphColor* color = context->graph->colors + block.colorIndex;
	b2ContactConstraintWide* constraints = static_cast<b2ContactConstraintWide *>( color->wideConstraints );
	b2FloatW inv_h = b2SplatW( context->inv_h );
	b2FloatW negRestitutionThreshold = b2SplatW( -context->world->restitutionThreshold );
	b2FloatW zeroW = b2ZeroW();
	bool propagate = context->world->enableRestitutionPropagation;

	for ( int wideIndex = block.startIndex; wideIndex < block.startIndex + block.count; ++wideIndex )
	{
		b2ContactConstraintWide* c = constraints + wideIndex;
		if ( propagate == false && b2AllZeroW( c->restitution ) )
		{
			continue;
		}

		b2BodyStateW bA = b2GatherBodies( states, c->indexA );
		b2BodyStateW bB = b2GatherBodies( states, c->indexB );

		b2FloatW restitutionMask = b2GreaterThanW( c->restitution, zeroW );
		b2FloatW normalMass1 = propagate ? c->normalMass1 : b2BlendW( zeroW, c->normalMass1, restitutionMask );
		b2FloatW normalMass2 = propagate ? c->normalMass2 : b2BlendW( zeroW, c->normalMass2, restitutionMask );
		b2Vec2W dp = { b2SubW( bB.dp.X, bA.dp.X ), b2SubW( bB.dp.Y, bA.dp.Y ) };

		{
			b2Vec2W rA = c->anchorA1;
			b2Vec2W rB = c->anchorB1;

			b2FloatW compressionImpulse = b2SubW( c->totalNormalImpulse1, c->restitutionImpulse1 );
			b2FloatW armed = b2AndW( b2AndW( restitutionMask, b2LessThanW( c->relativeVelocity1, negRestitutionThreshold ) ),
									 b2GreaterThanW( compressionImpulse, zeroW ) );

			b2Vec2W rsA = b2RotateVectorW( bA.dq, rA );
			b2Vec2W rsB = b2RotateVectorW( bB.dq, rB );
			b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			b2FloatW s = b2AddW( b2DotW( c->normal, ds ), c->baseSeparation1 );

			b2FloatW specBias = b2MaxW( zeroW, b2MulW( s, inv_h ) );
			b2FloatW velocityBias = b2BlendW( specBias, b2MulW( c->restitution, c->relativeVelocity1 ), armed );

			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vn = b2AddW( b2MulW( dvx, c->normal.X ), b2MulW( dvy, c->normal.Y ) );

			b2FloatW negImpulse = b2MulW( normalMass1, b2AddW( vn, velocityBias ) );

			b2FloatW newImpulse = b2MaxW( b2SubW( c->normalImpulse1, negImpulse ), zeroW );
			b2FloatW impulse = b2SubW( newImpulse, c->normalImpulse1 );

			b2FloatW approachImpulse = b2MinW( b2MaxW( b2NegW( b2MulW( normalMass1, vn ) ), zeroW ), b2MaxW( impulse, zeroW ) );
			b2FloatW allowance =
				b2SubW( b2MulW( c->restitution, b2AddW( compressionImpulse, approachImpulse ) ), c->restitutionImpulse1 );
			b2FloatW maxImpulse = b2AddW( approachImpulse, b2MaxW( allowance, zeroW ) );
			impulse = b2BlendW( impulse, b2MinW( impulse, maxImpulse ), armed );

			c->normalImpulse1 = b2AddW( c->normalImpulse1, impulse );
			c->restitutionImpulse1 = b2AddW( c->restitutionImpulse1, b2SubW( impulse, approachImpulse ) );
			c->totalNormalImpulse1 = b2AddW( c->totalNormalImpulse1, impulse );

			b2FloatW Px = b2MulW( impulse, c->normal.X );
			b2FloatW Py = b2MulW( impulse, c->normal.Y );

			bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		{
			b2Vec2W rA = c->anchorA2;
			b2Vec2W rB = c->anchorB2;

			b2FloatW compressionImpulse = b2SubW( c->totalNormalImpulse2, c->restitutionImpulse2 );
			b2FloatW armed = b2AndW( b2AndW( restitutionMask, b2LessThanW( c->relativeVelocity2, negRestitutionThreshold ) ),
									 b2GreaterThanW( compressionImpulse, zeroW ) );

			b2Vec2W rsA = b2RotateVectorW( bA.dq, rA );
			b2Vec2W rsB = b2RotateVectorW( bB.dq, rB );
			b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			b2FloatW s = b2AddW( b2DotW( c->normal, ds ), c->baseSeparation2 );

			b2FloatW specBias = b2MaxW( zeroW, b2MulW( s, inv_h ) );
			b2FloatW velocityBias = b2BlendW( specBias, b2MulW( c->restitution, c->relativeVelocity2 ), armed );

			b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			b2FloatW vn = b2AddW( b2MulW( dvx, c->normal.X ), b2MulW( dvy, c->normal.Y ) );

			b2FloatW negImpulse = b2MulW( normalMass2, b2AddW( vn, velocityBias ) );

			b2FloatW newImpulse = b2MaxW( b2SubW( c->normalImpulse2, negImpulse ), zeroW );
			b2FloatW impulse = b2SubW( newImpulse, c->normalImpulse2 );

			b2FloatW approachImpulse = b2MinW( b2MaxW( b2NegW( b2MulW( normalMass2, vn ) ), zeroW ), b2MaxW( impulse, zeroW ) );
			b2FloatW allowance =
				b2SubW( b2MulW( c->restitution, b2AddW( compressionImpulse, approachImpulse ) ), c->restitutionImpulse2 );
			b2FloatW maxImpulse = b2AddW( approachImpulse, b2MaxW( allowance, zeroW ) );
			impulse = b2BlendW( impulse, b2MinW( impulse, maxImpulse ), armed );

			c->normalImpulse2 = b2AddW( c->normalImpulse2, impulse );
			c->restitutionImpulse2 = b2AddW( c->restitutionImpulse2, b2SubW( impulse, approachImpulse ) );
			c->totalNormalImpulse2 = b2AddW( c->totalNormalImpulse2, impulse );

			b2FloatW Px = b2MulW( impulse, c->normal.X );
			b2FloatW Py = b2MulW( impulse, c->normal.Y );

			bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		}

		b2ScatterBodies( states, c->indexA, &bA );
		b2ScatterBodies( states, c->indexB, &bB );
	}

}

// 曾尝试把它加到最后几次松弛迭代，但更慢。
//
// 作为对整个宽约束范围的扁平 parallel-for 运行。每颜色的接触仿真
// 通过 prepareSpans 游标查找，而非块的 colorIndex，与 b2PrepareContacts_Wide 布局一致。
//
// 可以把流形指针存入 b2ContactConstraintWide 以简化此过程。
// 但保持约束结构体小很重要。
void B2_WIDE( b2StoreImpulses_Wide )( b2SolverBlock block, b2StepContext* context, int workerIndex )
{


	b2World* world = context->world;
	const b2ContactPrepareSpan* spans = context->contactPrepareSpans;
	const b2ContactConstraintWide* wideBase = static_cast<const b2ContactConstraintWide *>( context->wideContactConstraints );
	b2TaskContext* taskContext = world->taskContexts.data + workerIndex;
	b2BitSet* hitEventBitSet = &taskContext->hitEventBitSet;
	bool hasHitEvents = taskContext->hasHitEvents;
	float negHitThreshold = -world->hitEventThreshold;

	int wideIndex = block.startIndex;
	int endWideIndex = block.startIndex + block.count;

	// 为起始下标找颜色
	int colorIndex = 0;
	while ( spans[colorIndex + 1].start <= wideIndex )
	{
		colorIndex += 1;
	}

	while ( wideIndex < endWideIndex )
	{
		int colorWideEndIndex = b2MinInt( spans[colorIndex + 1].start, endWideIndex );
		int colorWideStart = spans[colorIndex].start;
		int colorContactCount = spans[colorIndex].count;
		b2ContactSim* contactSims = spans[colorIndex].contacts;

		for ( ; wideIndex < colorWideEndIndex; ++wideIndex )
		{
			const b2ContactConstraintWide* c = wideBase + wideIndex;
			const float* rollingImpulse = (float*)&c->rollingImpulse;
			const float* normalImpulse1 = (float*)&c->normalImpulse1;
			const float* normalImpulse2 = (float*)&c->normalImpulse2;
			const float* tangentImpulse1 = (float*)&c->tangentImpulse1;
			const float* tangentImpulse2 = (float*)&c->tangentImpulse2;
			const float* totalNormalImpulse1 = (float*)&c->totalNormalImpulse1;
			const float* totalNormalImpulse2 = (float*)&c->totalNormalImpulse2;

			int localWideIndex = wideIndex - colorWideStart;
			int baseIndex = B2_SIMD_WIDTH * localWideIndex;

			for ( int laneIndex = 0; laneIndex < B2_SIMD_WIDTH; ++laneIndex )
			{
				int contactIndex = baseIndex + laneIndex;
				if ( contactIndex >= colorContactCount )
				{
					break;
				}

				b2ContactSim* contactSim = contactSims + contactIndex;
				b2Manifold* m = &contactSim->manifold;
				m->rollingImpulse = rollingImpulse[laneIndex];

				m->points[0].normalImpulse = normalImpulse1[laneIndex];
				m->points[0].tangentImpulse = tangentImpulse1[laneIndex];
				m->points[0].totalNormalImpulse = totalNormalImpulse1[laneIndex];

				m->points[1].normalImpulse = normalImpulse2[laneIndex];
				m->points[1].tangentImpulse = tangentImpulse2[laneIndex];
				m->points[1].totalNormalImpulse = totalNormalImpulse2[laneIndex];

				// 检查命中事件，以加速本步稍后的串行处理
				if ( ( contactSim->simFlags & b2_simEnableHitEvent ) != 0 )
				{
					for ( int k = 0; k < contactSim->manifold.pointCount; ++k )
					{
						b2ManifoldPoint* mp = m->points + k;

						// 需检查总冲量，因为该点可能是预测性的且未碰撞
						if ( mp->normalVelocity < negHitThreshold && mp->totalNormalImpulse > 0.0f )
						{
							b2SetBit( hitEventBitSet, contactSim->contactId );
							hasHitEvents = true;
							break;
						}
					}
				}
			}
		}

		// 前进到下一颜色
		colorIndex += 1;
	}

	taskContext->hasHitEvents = hasHitEvents;

}

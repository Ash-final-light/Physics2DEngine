// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/math_types.h"

#include <stdint.h>
#if defined( _MSC_VER )
#include <vcruntime_string.h>
#else
#include <string.h>
#endif

// FNV-1a 64 位常量
#define B2_SNAP_FNV_INIT 14695981039346656037ull
#define B2_SNAP_FNV_PRIME 1099511628211ull

// 以全宽度混合世界位置，避免远距离位置只比较 float 精度。
static inline uint64_t b2FnvMixPosition( uint64_t hash, b2Pos p )
{
#if defined( BOX2D_DOUBLE_PRECISION )
	uint64_t bx, by;
	memcpy( &bx, &p.x, 8 );
	memcpy( &by, &p.y, 8 );
#else
	uint32_t fx, fy;
	memcpy( &fx, &p.x, 4 );
	memcpy( &fy, &p.y, 4 );
	uint64_t bx = fx, by = fy;
#endif
	hash = ( hash ^ bx ) * B2_SNAP_FNV_PRIME;
	hash = ( hash ^ by ) * B2_SNAP_FNV_PRIME;
	return hash;
}

struct b2World;

// 对活跃刚体的变换与速度做确定性哈希。
uint64_t b2HashWorldState( b2World* world );

// 对仿真状态做完整哈希，包含索引记账、接触与关节冲量、id 池和求解器集合计数。
uint64_t b2HashWorldStateDeep( b2World* world );

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

/// 二维向量。可用于表示点或自由向量。
typedef struct b2Vec2
{
	float x, y;
} b2Vec2;

static const b2Vec2 b2Vec2_zero = { 0.0f, 0.0f };

/// 余弦与正弦对。
typedef struct b2CosSin
{
	float cosine;
	float sine;
} b2CosSin;

#define B2_PI 3.14159265359f

/// 二维旋转。用余弦和正弦表示。
typedef struct b2Rot
{
	float c, s;
} b2Rot;

static const b2Rot b2Rot_identity = { 1.0f, 0.0f };

/// 二维刚体变换
typedef struct b2Transform
{
	b2Vec2 p;
	b2Rot q;
} b2Transform;

static const b2Transform b2Transform_identity = { { 0.0f, 0.0f }, { 1.0f, 0.0f } };

#if defined( BOX2D_DOUBLE_PRECISION )

/// 世界位置。大世界模式下为双精度，使坐标在远离原点时仍保持准确。
typedef struct b2Pos
{
	double x, y;
} b2Pos;

/// 世界变换，平移为双精度、旋转为单精度。旋转是局部的，不需要额外范围，
/// 与 Jolt 的 DMat44 划分方式相同。
typedef struct b2WorldTransform
{
	b2Pos p;
	b2Rot q;
} b2WorldTransform;

#else

/// 单精度下的别名。
typedef b2Vec2 b2Pos;

/// 单精度下的别名。
typedef b2Transform b2WorldTransform;

#endif

static const b2Pos b2Pos_zero = { 0.0f, 0.0f };
static const b2WorldTransform b2WorldTransform_identity = { { 0.0f, 0.0f }, { 1.0f, 0.0f } };

/// 2×2 矩阵，按列存储。
typedef struct b2Mat22
{
	b2Vec2 cx, cy;
} b2Mat22;

static const b2Mat22 b2Mat22_zero = { { 0.0f, 0.0f }, { 0.0f, 0.0f } };

/// 轴对齐包围盒。
typedef struct b2AABB
{
	b2Vec2 lowerBound;
	b2Vec2 upperBound;
} b2AABB;

typedef struct b2Plane
{
	b2Vec2 normal;
	float offset;
} b2Plane;

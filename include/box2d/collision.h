// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "math_types.h"

/// 射线检测输出。未命中时 hit 为 false，其余字段为零。
typedef struct b2CastOutput
{
	b2Vec2 normal;
	b2Vec2 point;
	float fraction;
	int iterations;
	bool hit;
} b2CastOutput;

/// 实心圆。
typedef struct b2Circle
{
	b2Vec2 center;
	float radius;
} b2Circle;

/// 具有双侧碰撞的线段。
typedef struct b2Segment
{
	b2Vec2 point1;
	b2Vec2 point2;
} b2Segment;

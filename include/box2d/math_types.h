// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

/// 二维向量。可用于表示点或自由向量。
typedef struct b2Vec2
{
	float x, y;
} b2Vec2;

static const b2Vec2 b2Vec2_zero = { 0.0f, 0.0f };

/// 二维旋转。用余弦和正弦表示。
typedef struct b2Rot
{
	float c, s;
} b2Rot;

static const b2Rot b2Rot_identity = { 1.0f, 0.0f };

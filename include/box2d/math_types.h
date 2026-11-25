// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

/// 二维向量。可用于表示点或自由向量。
typedef struct b2Vec2
{
	float x, y;
} b2Vec2;

static const b2Vec2 b2Vec2_zero = { 0.0f, 0.0f };

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "math_functions.h"

/// 碰撞与约束使用的小长度。默认长度单位下为 0.005。
#define B2_LINEAR_SLOP ( 0.005f * b2_lengthUnitsPerMeter )

/// 超过该范围的长度视为无效。单精度下为 100000 倍长度单位。
#if defined( BOX2D_DOUBLE_PRECISION )
#define B2_HUGE ( ( 100000.0f * 1000.0f ) * b2_lengthUnitsPerMeter )
#else
#define B2_HUGE ( ( 100.0f * 1000.0f ) * b2_lengthUnitsPerMeter )
#endif

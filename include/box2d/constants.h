// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "math_functions.h"

/// 碰撞与约束使用的小长度。默认长度单位下为 0.005。
#define B2_LINEAR_SLOP ( 0.005f * b2_lengthUnitsPerMeter )

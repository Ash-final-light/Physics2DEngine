// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"

/**
 * @addtogroup math
 * @{
 */

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

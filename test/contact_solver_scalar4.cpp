// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT
#include "core.h"
#undef B2_SIMD_SSE2
#undef B2_SIMD_NEON
#undef B2_SIMD_AVX2
#define B2_SIMD_WIDTH 4
#define B2_WIDE(name) name##Scalar4
#include "contact_solver_wide.inl"

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

struct b2World;
struct b2SnapBuffer;

// 将世界的完整仿真状态序列化到 buf。支撑公开 API b2World_GetSnapshot。
// 必须在步边界（两次步进之间）调用。
void b2SerializeWorld( b2World* world, b2SnapBuffer* buf );

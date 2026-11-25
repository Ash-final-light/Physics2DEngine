// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

// Box2D 编译期选项。
//
// 通常由 CMake 的 BOX2D_* 选项设置。若在不使用其 CMake 的情况下构建 Box2D
//（将源码放入其他项目），请在此设置，使库与你的代码保持一致。可直接编辑本文件，
// 或将设置放在别处，并在构建时让 Box2D 指向它们：
//
//   #define BOX2D_USER_CONFIG "my_box2d_config.h"
//
// 通过编译器命令行传入的宏定义仍优先于本文件。

// 大世界模式。以双精度存储世界位置。影响 ABI。
// #define BOX2D_DOUBLE_PRECISION

// 构建标量回退实现，而不使用 AVX2/SSE2/NEON。
// #define BOX2D_DISABLE_SIMD

// 在调试构建中启用内部校验。
// #define BOX2D_VALIDATE

// 用你自己的导出宏装饰公共 API，而不是 Box2D 的
// box2d_EXPORTS/BOX2D_DLL 方案，例如将 Box2D 编译进另一个
// 共享库时。单个取值无法在 dllexport 与 dllimport 之间切换，
// 因此更适合嵌入，而不是把 Box2D 作为独立 DLL 发布。
// #define BOX2D_EXPORT MYENGINE_API

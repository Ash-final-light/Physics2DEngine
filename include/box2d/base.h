// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>

#ifdef BOX2D_USER_CONFIG
#include BOX2D_USER_CONFIG
#endif
#include "config.h"

#ifndef BOX2D_EXPORT
#if defined( _WIN32 ) && defined( box2d_EXPORTS )
#define BOX2D_EXPORT __declspec( dllexport )
#elif defined( _WIN32 ) && defined( BOX2D_DLL )
#define BOX2D_EXPORT __declspec( dllimport )
#elif defined( box2d_EXPORTS )
#define BOX2D_EXPORT __attribute__( ( visibility( "default" ) ) )
#else
#define BOX2D_EXPORT
#endif
#endif

#ifdef __cplusplus
#define B2_API extern "C" BOX2D_EXPORT
#else
#define B2_API BOX2D_EXPORT
#endif

/// 版本编号方案。
typedef struct b2Version
{
	/// 重大变更
	int major;

	/// 增量变更
	int minor;

	/// 缺陷修复
	int revision;
} b2Version;

/// 重大 API 与行为变更。
#define B2_VERSION_MAJOR 3

/// API 变更与功能。
#define B2_VERSION_MINOR 2

/// 缺陷修复。无 API 变更。
#define B2_VERSION_REVISION 0

/// 获取 Box2D 的当前版本
B2_API b2Version b2GetVersion( void );

/// @return 若库以 BOX2D_DOUBLE_PRECISION（大世界模式）构建则为 true
B2_API bool b2IsDoublePrecision( void );

/// 用户断言回调原型。返回 0 可跳过调试器中断。
typedef int b2AssertCallbackFcn( const char* condition, const char* fileName, int lineNumber );

/// 覆盖默认断言函数
/// @param assertFcn 非空的断言回调
B2_API void b2SetAssertCallback( b2AssertCallbackFcn* assertFcn );

/// 用户日志回调原型。用于记录警告。
typedef void b2LogCallbackFcn( const char* message );

/// 覆盖默认日志函数
/// @param logFcn 非空的日志回调
B2_API void b2SetLogCallback( b2LogCallbackFcn* logFcn );

#if defined( _MSC_VER )
#define B2_BREAKPOINT __debugbreak()
#elif defined( __GNUC__ ) || defined( __clang__ )
#define B2_BREAKPOINT __builtin_trap()
#else
#include <assert.h>
#define B2_BREAKPOINT assert( 0 )
#endif

#if !defined( NDEBUG ) || defined( B2_ENABLE_ASSERT )
B2_API int b2InternalAssert( const char* condition, const char* fileName, int lineNumber );
#define B2_ASSERT( condition )                                                                     \
	( (void)( ( !!( condition ) ) ||                                                               \
			  ( b2InternalAssert( #condition, __FILE__, (int)( __LINE__ ) ), 0 ) ) )
#else
#define B2_ASSERT( ... ) ( (void)0 )
#endif

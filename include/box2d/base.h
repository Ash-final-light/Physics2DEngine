// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
#define B2_INLINE inline
#define B2_LITERAL( T ) T
#if defined( _MSC_VER )
#define B2_FORCE_INLINE __forceinline
#elif defined( __GNUC__ ) || defined( __clang__ )
#define B2_FORCE_INLINE inline __attribute__( ( always_inline ) )
#else
#define B2_FORCE_INLINE inline
#endif
#else
#define B2_API BOX2D_EXPORT
#define B2_INLINE static inline
#define B2_LITERAL( T ) ( T )
#if defined( _MSC_VER )
#define B2_FORCE_INLINE static __forceinline
#elif defined( __GNUC__ ) || defined( __clang__ )
#define B2_FORCE_INLINE static inline __attribute__( ( always_inline ) )
#else
#define B2_FORCE_INLINE static inline
#endif
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

/// 用户分配函数原型
/// @param size 分配大小，单位为字节
/// @param alignment 所需对齐，保证为 2 的幂
typedef void* b2AllocCallbackFcn( size_t size, int alignment );

/// 用户释放函数原型
/// @param mem 先前通过 `b2AllocCallbackFcn` 分配的内存
/// @param size 分配大小，单位为字节
typedef void b2FreeCallbackFcn( void* mem, size_t size );

/// 允许用户覆盖分配函数。应在应用程序启动期间设置。
B2_API void b2SetAllocator( b2AllocCallbackFcn* allocFcn, b2FreeCallbackFcn* freeFcn );

/// @return Box2D 已分配的总字节数
B2_API size_t b2GetByteCount( void );

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

#if defined( BOX2D_VALIDATE ) && !defined( NDEBUG )
#define B2_ENABLE_VALIDATION 1
#else
#define B2_ENABLE_VALIDATION 0
#endif

#if B2_ENABLE_VALIDATION
#define B2_VALIDATE( condition ) B2_ASSERT( condition )
#else
#define B2_VALIDATE( ... ) ( (void)0 )
#endif

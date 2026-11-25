// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

#include <cstring>
#include <type_traits>

// 描述符本身没有所有权钩子。创建和销毁显式配对。
template <typename T>
struct b2DynamicArray
{
	T* data;
	int count;
	int capacity;
};

#define b2DeclareArray( T ) struct T; using b2DynamicArray_##T = b2DynamicArray<T>
#define b2DeclareArrayNative( T ) using b2DynamicArray_##T = b2DynamicArray<T>
#define b2Array( T ) b2DynamicArray_##T

template <typename T>
inline void b2Array_Create( b2DynamicArray<T>& a )
{
	a = {};
}

template <typename T>
inline void b2Array_CreateN( b2DynamicArray<T>& a, int n )
{
	static_assert( std::is_trivial_v<T> && std::is_standard_layout_v<T> );
	static_assert( alignof( T ) <= B2_ALIGNMENT );
	B2_ASSERT( n >= 0 );
	a.data = n > 0 ? static_cast<T*>( b2GrowAlloc( nullptr, 0, n * sizeof( T ) ) ) : nullptr;
	a.count = 0;
	a.capacity = n;
}

template <typename T>
inline void b2Array_Destroy( b2DynamicArray<T>& a )
{
	static_assert( std::is_trivial_v<T> && std::is_standard_layout_v<T> );
	b2Free( a.data, a.capacity * sizeof( T ) );
	a = {};
}

template <typename T>
inline void b2Array_Reserve( b2DynamicArray<T>& a, int n )
{
	static_assert( std::is_trivial_v<T> && std::is_standard_layout_v<T> );
	static_assert( alignof( T ) <= B2_ALIGNMENT );
	B2_ASSERT( n >= 0 );
	if ( a.capacity < n )
	{
		a.data = static_cast<T*>( b2GrowAlloc( a.data, a.capacity * sizeof( T ), n * sizeof( T ) ) );
		a.capacity = n;
	}
}

template <typename T>
inline void b2Array_Grow( b2DynamicArray<T>& a, int n )
{
	if ( a.capacity < n )
	{
		b2Array_Reserve( a, 2 * a.capacity > n ? 2 * a.capacity : n );
	}
}

template <typename T>
inline void b2Array_Push( b2DynamicArray<T>& a, std::type_identity_t<T> value )
{
	if ( a.count >= a.capacity )
	{
		b2Array_Reserve( a, a.capacity == 0 ? 8 : 2 * a.capacity );
	}
	a.data[a.count++] = value;
}

template <typename T>
inline T* b2Array_Emplace( b2DynamicArray<T>& a )
{
	if ( a.count >= a.capacity )
	{
		b2Array_Reserve( a, a.capacity == 0 ? 8 : 2 * a.capacity );
	}
	return a.data + a.count++;
}

template <typename T>
inline T* b2Array_Get( b2DynamicArray<T>& a, int index )
{
	B2_ASSERT( 0 <= index && index < a.count );
	return a.data + index;
}

template <typename T>
inline const T* b2Array_Get( const b2DynamicArray<T>& a, int index )
{
	B2_ASSERT( 0 <= index && index < a.count );
	return a.data + index;
}

template <typename T>
inline T b2Array_Pop( b2DynamicArray<T>& a )
{
	B2_ASSERT( a.count > 0 );
	return a.data[--a.count];
}

template <typename T>
inline int b2Array_RemoveSwap( b2DynamicArray<T>& a, int index )
{
	B2_ASSERT( 0 <= index && index < a.count );
	--a.count;
	if ( index != a.count )
	{
		std::memcpy( a.data + index, a.data + a.count, sizeof( T ) );
		return a.count;
	}
	return B2_NULL_INDEX;
}

template <typename T>
inline void b2Array_Resize( b2DynamicArray<T>& a, int n )
{
	b2Array_Reserve( a, n );
	a.count = n;
}

template <typename T>
inline void b2Array_ResizeAndSetZero( b2DynamicArray<T>& a, int n )
{
	b2Array_Reserve( a, n );
	if ( n > 0 )
	{
		std::memset( a.data, 0, n * sizeof( T ) );
	}
	a.count = n;
}

template <typename T>
inline void b2Array_Clear( b2DynamicArray<T>& a )
{
	a.count = 0;
}

template <typename T>
inline int b2Array_ByteCount( const b2DynamicArray<T>& a )
{
	return a.capacity * static_cast<int>( sizeof( T ) );
}

b2DeclareArrayNative( int );

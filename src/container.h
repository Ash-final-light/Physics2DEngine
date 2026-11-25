// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"

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
	B2_ASSERT( n >= 0 );
	a.data = n > 0 ? static_cast<T*>( b2GrowAlloc( nullptr, 0, n * sizeof( T ) ) ) : nullptr;
	a.count = 0;
	a.capacity = n;
}

template <typename T>
inline void b2Array_Destroy( b2DynamicArray<T>& a )
{
	b2Free( a.data, a.capacity * sizeof( T ) );
	a = {};
}

template <typename T>
inline void b2Array_Reserve( b2DynamicArray<T>& a, int n )
{
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

b2DeclareArrayNative( int );

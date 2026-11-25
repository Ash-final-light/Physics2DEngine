// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "container.h"

#include <stdbool.h>

b2DeclareArray( b2StackEntry );

typedef struct b2StackEntry
{
	char* data;
	const char* name;
	int size;
	bool usedMalloc;
} b2StackEntry;

// 分配和释放必须成对嵌套。空间不足时改为单独堆分配。
typedef struct b2Stack
{
	char* data;
	int capacity;
	int index;

	int allocation;
	int maxAllocation;

	b2Array( b2StackEntry ) entries;
} b2Stack;

b2Stack b2CreateStack( int capacity );
void b2DestroyStack( b2Stack* allocator );

void* b2StackAlloc( b2Stack* alloc, int size, const char* name );
void b2StackFree( b2Stack* alloc, void* mem );
void b2GrowStack( b2Stack* alloc );

int b2GetStackCapacity( b2Stack* alloc );
int b2GetStackAllocation( b2Stack* alloc );
int b2GetMaxStackAllocation( b2Stack* alloc );

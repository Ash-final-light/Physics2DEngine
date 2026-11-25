// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/base.h"

#include <stdbool.h>
#include <stdint.h>

#define B2_SHAPE_PAIR_KEY( K1, K2 ) K1 < K2 ? (uint64_t)K1 << 32 | (uint64_t)K2 : (uint64_t)K2 << 32 | (uint64_t)K1

// 空槽的键是 0。哈希使用 Murmur 混合，避免递增整数对挤在同一槽。
B2_FORCE_INLINE uint64_t b2KeyHash( uint64_t key )
{
	uint64_t h = key;
	h ^= h >> 33;
	h *= 0xff51afd7ed558ccduLL;
	h ^= h >> 33;
	h *= 0xc4ceb9fe1a85ec53uLL;
	h ^= h >> 33;
	return h;
}

typedef struct b2SetItem
{
	uint64_t key;
} b2SetItem;

typedef struct b2HashSet
{
	b2SetItem* items;
	uint32_t capacity;
	uint32_t count;
} b2HashSet;

b2HashSet b2CreateSet( int capacity );
void b2DestroySet( b2HashSet* set );

// 键已在集合中时返回 true。
bool b2AddKey( b2HashSet* set, uint64_t key );

// 找到并删除该键时返回 true。
bool b2RemoveKey( b2HashSet* set, uint64_t key );

bool b2ContainsKey( const b2HashSet* set, uint64_t key );
bool b2ContainsHashedKey( const b2HashSet* set, uint64_t key, uint64_t hash );

int b2GetHashSetBytes( b2HashSet* set );

static inline int b2GetSetCount( b2HashSet* set )
{
	return set->count;
}

static inline int b2GetSetCapacity( b2HashSet* set )
{
	return set->capacity;
}

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

typedef struct b2WorldId
{
	uint16_t index1;
	uint16_t generation;
} b2WorldId;

typedef struct b2BodyId
{
	int32_t index1;
	uint16_t world0;
	uint16_t generation;
} b2BodyId;

static const b2BodyId b2_nullBodyId = {};


typedef struct b2ShapeId
{
    int32_t index1;
    uint16_t world0;
    uint16_t generation;
} b2ShapeId;

static const b2ShapeId b2_nullShapeId = {};


typedef struct b2ChainId
{
    int32_t index1;
    uint16_t world0;
    uint16_t generation;
} b2ChainId;

static const b2ChainId b2_nullChainId = {};


typedef struct b2ContactId
{
    int32_t index1;
    uint16_t world0;
    int16_t padding;
    uint32_t generation;
} b2ContactId;

static const b2ContactId b2_nullContactId = {};

typedef struct b2JointId
{
 int32_t index1;
 uint16_t world0;
 uint16_t generation;
} b2JointId;
static const b2JointId b2_nullJointId = {};

#ifdef __cplusplus
#define B2_ID_INLINE inline
#else
#define B2_ID_INLINE static inline
#endif

B2_ID_INLINE uint32_t b2StoreWorldId( b2WorldId id )
{
	return ( (uint32_t)id.index1 << 16 ) | (uint32_t)id.generation;
}

B2_ID_INLINE uint64_t b2StoreBodyId( b2BodyId id )
{
	return ( (uint64_t)id.index1 << 32 ) | ( (uint64_t)id.world0 ) << 16 | (uint64_t)id.generation;
}

B2_ID_INLINE uint64_t b2StoreShapeId( b2ShapeId id )
{
	return ( (uint64_t)id.index1 << 32 ) | ( (uint64_t)id.world0 ) << 16 | (uint64_t)id.generation;
}

B2_ID_INLINE uint64_t b2StoreChainId( b2ChainId id )
{
	return ( (uint64_t)id.index1 << 32 ) | ( (uint64_t)id.world0 ) << 16 | (uint64_t)id.generation;
}

B2_ID_INLINE uint64_t b2StoreJointId( b2JointId id )
{
	return ( (uint64_t)id.index1 << 32 ) | ( (uint64_t)id.world0 ) << 16 | (uint64_t)id.generation;
}

static const b2WorldId b2_nullWorldId = {};

B2_ID_INLINE b2WorldId b2LoadWorldId( uint32_t x )
{
	b2WorldId id = { (uint16_t)( x >> 16 ), (uint16_t)( x ) };
	return id;
}

B2_ID_INLINE b2BodyId b2LoadBodyId( uint64_t x )
{
	b2BodyId id = { (int32_t)( x >> 32 ), (uint16_t)( x >> 16 ), (uint16_t)( x ) };
	return id;
}

B2_ID_INLINE b2ShapeId b2LoadShapeId( uint64_t x )
{
	b2ShapeId id = { (int32_t)( x >> 32 ), (uint16_t)( x >> 16 ), (uint16_t)( x ) };
	return id;
}

B2_ID_INLINE b2ChainId b2LoadChainId( uint64_t x )
{
	b2ChainId id = { (int32_t)( x >> 32 ), (uint16_t)( x >> 16 ), (uint16_t)( x ) };
	return id;
}

B2_ID_INLINE b2JointId b2LoadJointId( uint64_t x )
{
	b2JointId id = { (int32_t)( x >> 32 ), (uint16_t)( x >> 16 ), (uint16_t)( x ) };
	return id;
}

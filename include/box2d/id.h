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

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

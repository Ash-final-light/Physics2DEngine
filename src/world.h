// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "arena_allocator.h"

#include "box2d/types.h"

struct b2World
{
	uint16_t worldId;
	uint16_t generation;
	bool inUse;
	bool locked;
	b2Vec2 gravity;
	float restitutionThreshold;
	int restitutionIterations;
	bool enableRestitutionPropagation;
	float hitEventThreshold;
	float contactHertz;
	float contactDampingRatio;
	float contactSpeed;
	float maxLinearSpeed;
	b2FrictionCallbackFcn* frictionCallback;
	b2RestitutionCallbackFcn* restitutionCallback;
	bool enableSleep;
	bool enableContinuous;
	int workerCount;
	void* userData;
	b2Stack stack;
};

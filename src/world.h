// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "arena_allocator.h"
#include "body.h"
#include "broad_phase.h"
#include "container.h"
#include "id_pool.h"
#include "shape.h"

#include "box2d/constants.h"
#include "box2d/types.h"

struct b2World
{
	uint16_t worldId;
	uint16_t generation;
	bool inUse;
	bool locked;
	b2Vec2 gravity;
	b2BroadPhase broadPhase;
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
	b2IdPool bodyIdPool;
	b2DynamicArray<b2Body> bodies;
	b2IdPool shapeIdPool;
	b2DynamicArray<b2Shape> shapes;
	b2DynamicArray<b2AABB> fatAABBs;
	b2IdPool chainIdPool;
	b2DynamicArray<b2ChainShape> chainShapes;
};

extern b2World b2_worlds[B2_MAX_WORLDS];

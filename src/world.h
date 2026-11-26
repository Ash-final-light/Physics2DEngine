// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "arena_allocator.h"
#include "body.h"
#include "broad_phase.h"
#include "contact.h"
#include "constraint_graph.h"
#include "solver_set.h"
#include "container.h"
#include "id_pool.h"
#include "shape.h"
#include "sensor.h"

#include "box2d/constants.h"
#include "box2d/types.h"

struct b2World
{
	uint16_t worldId;
	uint16_t generation;
	bool inUse;
	bool locked;
	b2Vec2 gravity;
	b2Capacity maxCapacity;
	b2Profile profile;
	b2DynamicArray<b2BodyMoveEvent> bodyMoveEvents;
	b2DynamicArray<b2ContactHitEvent> contactHitEvents;
	b2DynamicArray<b2ContactBeginTouchEvent> contactBeginEvents;
	b2DynamicArray<b2ContactEndTouchEvent> contactEndEvents[2];
	int endEventArrayIndex;
	b2DynamicArray<b2Sensor> sensors;
	b2DynamicArray<b2SensorTaskContext> sensorTaskContexts;
	b2DynamicArray<b2SensorBeginTouchEvent> sensorBeginEvents;
	b2DynamicArray<b2SensorEndTouchEvent> sensorEndEvents[2];
	int recycledContactCount;
	b2BroadPhase broadPhase;
	b2ConstraintGraph constraintGraph;
	b2IdPool islandIdPool;
	b2DynamicArray<b2Island> islands;
	b2IdPool contactIdPool;
	b2DynamicArray<b2Contact> contacts;
	b2IdPool solverSetIdPool;
	b2DynamicArray<b2SolverSet> solverSets;
	float restitutionThreshold;
	int restitutionIterations;
	bool enableRestitutionPropagation;
	float hitEventThreshold;
	float contactRecycleDistance;
	float contactHertz;
	float contactDampingRatio;
	float contactSpeed;
	float maxLinearSpeed;
	b2CustomFilterCallbackFcn* customFilterFcn;
	void* customFilterContext;
	b2FrictionCallbackFcn* frictionCallback;
	b2RestitutionCallbackFcn* restitutionCallback;
	bool enableWarmStarting;
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

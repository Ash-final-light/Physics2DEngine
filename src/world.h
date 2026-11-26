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
#include "joint.h"
#include "sensor.h"

#include "box2d/constants.h"
#include "box2d/types.h"

struct b2Scheduler;

struct b2TaskContext
{
    b2DynamicArray<uint64_t> pairKeys;
    b2BitSet contactStateBitSet;
    int recycledContactCount;
    b2BitSet jointStateBitSet;
    b2BitSet hitEventBitSet;
    bool hasHitEvents;
};

struct b2World
{
    b2DynamicArray<b2TaskContext> taskContexts;
	uint16_t worldId;
	uint16_t generation;
	bool inUse;
	bool locked;
	uint64_t stepIndex;
	int splitIslandId;
	b2Vec2 gravity;
	b2Capacity maxCapacity;
	b2Profile profile;
	float inv_dt;
	float inv_h;
	b2BitSet debugBodySet;
	b2BitSet debugContactSet;
	b2BitSet debugIslandSet;
	b2BitSet debugJointSet;

	// 本步已报告的关节力/力矩事件位，步进开始时清零。
	// 各任务上下文先收集，求解完成后合并到此位集。
	b2BitSet jointStateBitSet;
	b2DynamicArray<b2JointEvent> jointEvents;
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
	b2PreSolveCallbackFcn* preSolveFcn;
	b2PreContinuousCallbackFcn* preContinuousFcn;
	void* preSolveContext;
	b2CustomFilterCallbackFcn* customFilterFcn;
	void* customFilterContext;
	b2FrictionCallbackFcn* frictionCallback;
	b2RestitutionCallbackFcn* restitutionCallback;
	bool enableWarmStarting;
	bool enableSleep;
	bool enableContinuous;
	int workerCount;
	b2EnqueueTaskCallbackFcn* enqueueTaskFcn;
	b2FinishTaskCallbackFcn* finishTaskFcn;
	void* userTaskContext;
	struct b2Scheduler* scheduler;
	bool useScheduler;
	struct b2StepContext* stepContext;
	void* userData;
	b2Stack stack;
	b2IdPool bodyIdPool;
	b2DynamicArray<b2Body> bodies;
	b2IdPool shapeIdPool;
	b2DynamicArray<b2Shape> shapes;
	b2DynamicArray<b2AABB> fatAABBs;
	b2IdPool jointIdPool;
	b2DynamicArray<b2Joint> joints;
	b2IdPool chainIdPool;
	b2DynamicArray<b2ChainShape> chainShapes;
};

extern b2World b2_worlds[B2_MAX_WORLDS];

bool b2ComputeWorldBounds(b2World* world,b2AABB* bounds);

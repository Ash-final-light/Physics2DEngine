// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once
#include "box2d/constants.h"
#include "box2d/math_functions.h"
#include "box2d/types.h"

struct b2Softness { float biasRate;float massScale;float impulseScale; };

static inline b2Softness b2MakeSoft( float hertz, float zeta, float h )
{
	if ( hertz == 0.0f )
	{
		return b2Softness{
			.biasRate = 0.0f,
			.massScale = 0.0f,
			.impulseScale = 0.0f,
		};
	}

	float omega = 2.0f * B2_PI * hertz;
	float a1 = 2.0f * zeta + h * omega;
	float a2 = h * omega * a1;
	float a3 = 1.0f / ( 1.0f + a2 );

	// bias = w / (2 * z + hw)
	// massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
	// impulseScale = 1 / (1 + hw * (2 * z + hw))

	// 若 z == 0
	// bias = 1/h
	// massScale = hw^2 / (1 + hw^2)
	// impulseScale = 1 / (1 + hw^2)

	// w -> inf
	// bias = 1/h
	// massScale = 1
	// impulseScale = 0

	// 若 w = pi / 4  * inv_h
	// massScale = (pi/4)^2 / (1 + (pi/4)^2) = pi^2 / (16 + pi^2) ~= 0.38
	// impulseScale = 1 / (1 + (pi/4)^2) = 16 / (16 + pi^2) ~= 0.62

	// 所有情况下：
	// massScale + impulseScale == 1

	return b2Softness{
		.biasRate = omega / a1,
		.massScale = a2 * a3,
		.impulseScale = a3,
	};
}


struct b2World;
struct b2ConstraintGraph;
struct b2BodyState;
struct b2StepContext;

typedef struct b2StepTask
{
	b2TaskCallbackFcn* fcn;
	void* context;
	b2StepContext* stepContext;
	bool deferred;
} b2StepTask;

#define B2_STEP_TASK_CAPACITY ( B2_MAX_WORKERS + 2 )

struct b2StepContext
{
    b2World* world;
    b2ConstraintGraph* graph;
    b2BodyState* states;
    float h;
    float inv_h;
    bool anyRestitution;

	int batchTaskBase;
	int batchTaskCount;
	int taskCount;
	int joinTaskBase;
	int joinTaskCount;
	b2Task tasks[B2_STEP_TASK_CAPACITY];
	b2Task joinTasks[B2_STEP_TASK_CAPACITY];
	b2StepTask joinStepTasks[B2_STEP_TASK_CAPACITY];
	b2StepTask stepTasks[B2_STEP_TASK_CAPACITY];
};
void b2SolveFreeBodies( b2World* world, float timeStep, int subStepCount );

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

struct b2World;
struct b2ConstraintGraph;
struct b2BodyState;
struct b2StepContext
{
    b2World* world;
    b2ConstraintGraph* graph;
    b2BodyState* states;
    float h;
    float inv_h;
};
void b2SolveFreeBodies( b2World* world, float timeStep, int subStepCount );

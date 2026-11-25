// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

struct b2World;
void b2SolveFreeBodies( b2World* world, float timeStep, int subStepCount );

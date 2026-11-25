// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once
#include "solver.h"
#include "box2d/math_types.h"

struct b2ContactConstraintPoint
{
    b2Vec2 anchorA, anchorB;
    float baseSeparation;
    float normalImpulse;
    float totalNormalImpulse;
    float normalMass;
};

struct b2ContactConstraint
{
    // One-based active state indices; zero represents a static endpoint.
    int indexA, indexB;
    b2ContactConstraintPoint points[2];
    b2Vec2 normal;
    float invMassA, invMassB;
    float invIA, invIB;
    b2Softness softness;
    int pointCount;
};

void b2PrepareContactConstraints(b2StepContext* context);
void b2ReleaseContactConstraints(b2StepContext* context);
void b2SolveNormalContacts(b2StepContext* context);
void b2PushNormalContacts(b2StepContext* context);
void b2WarmStartContacts(b2StepContext* context);
void b2StoreContactImpulses(b2StepContext* context);

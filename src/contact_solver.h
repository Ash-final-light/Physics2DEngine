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
    float tangentImpulse;
    float tangentMass;
    float relativeVelocity;
    float restitutionImpulse;
};

struct b2ContactConstraint
{
    // One-based active state indices; zero represents a static endpoint.
    int indexA, indexB;
    b2ContactConstraintPoint points[2];
    b2Vec2 normal;
    float invMassA, invMassB;
    float invIA, invIB;
    float friction;
    float restitution;
    float tangentSpeed;
    float rollingResistance;
    float rollingMass;
    float rollingImpulse;
    b2Softness softness;
    int pointCount;
};

void b2PrepareContactConstraints(b2StepContext* context);
void b2ReleaseContactConstraints(b2StepContext* context);
void b2SolveNormalContacts(b2StepContext* context);
void b2PushNormalContacts(b2StepContext* context);
void b2WarmStartContacts(b2StepContext* context);
void b2ApplyContactRestitution(b2StepContext* context);
void b2StoreContactImpulses(b2StepContext* context);

void b2WarmStartContactColor(b2StepContext* context,int colorIndex);
void b2SolveNormalColor(b2StepContext* context,int colorIndex,bool useBias);

void b2SolveNormalRange(b2StepContext* context,int colorIndex,bool useBias,int startIndex,int endIndex);
void b2WarmStartContactRange(b2StepContext* context,int colorIndex,int startIndex,int endIndex);

int b2GetWideContactConstraintByteCountW4(void);
void b2PrepareContacts_WideW4(b2SolverBlock block,b2StepContext* context);
void b2WarmStartContacts_WideW4(b2SolverBlock block,b2StepContext* context);
void b2PushContacts_WideW4(b2SolverBlock block,b2StepContext* context);
void b2SolveContacts_WideW4(b2SolverBlock block,b2StepContext* context);
void b2ApplyRestitution_WideW4(b2SolverBlock block,b2StepContext* context);
void b2StoreImpulses_WideW4(b2SolverBlock block,b2StepContext* context,int workerIndex);

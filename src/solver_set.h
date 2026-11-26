// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "body.h"
#include "contact.h"
#include "joint.h"
#include "container.h"
#include "island.h"

enum b2SolverSetType
{
    b2_staticSet=0,
    b2_disabledSet=1,
    b2_awakeSet=2,
    b2_firstSleepingSet=3
};

struct b2SolverSet
{
    b2DynamicArray<b2BodySim> bodySims;
    b2DynamicArray<b2BodyState> bodyStates;
    b2DynamicArray<b2ContactSim> contactSims;
    b2DynamicArray<b2JointSim> jointSims;
    b2DynamicArray<b2IslandSim> islandSims;
    int setIndex;
};

void b2CreateSolverSets( b2World* world );
void b2DestroySolverSets( b2World* world );
int b2CreateSolverSet( b2World* world );
void b2DestroySolverSet( b2World* world, int setIndex );
void b2RemoveBodyFromSolverSet( b2World* world, b2Body* body );
void b2TransferBody( b2World* world, b2SolverSet* target, b2SolverSet* source, b2Body* body );
void b2RefreshContactSets( b2World* world );
void b2WakeSolverSet( b2World* world, int setIndex );
void b2TrySleepIsland(b2World* world,int islandId);

void b2MergeSolverSets(b2World* world,int setId1,int setId2);
void b2TransferJoint(b2World* world,b2SolverSet* target,b2SolverSet* source,b2Joint* joint);

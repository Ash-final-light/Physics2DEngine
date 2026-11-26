// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once
#include "box2d/types.h"
struct b2World;
struct b2JointEdge { int bodyId;int prevKey;int nextKey; };
struct b2Joint
{
 void* userData;
 b2JointEdge edges[2];
 int jointId;
 uint16_t generation;
};
b2Joint* b2GetJointFullId(b2World* world,b2JointId id);
b2JointId b2CreateJointRecord(b2World* world,b2BodyId bodyIdA,b2BodyId bodyIdB);
void b2DestroyJointInternal(b2World* world,b2Joint* joint);

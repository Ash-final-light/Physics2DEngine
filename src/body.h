// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/constants.h"
#include "box2d/id.h"
#include "box2d/math_types.h"
#include "box2d/types.h"

struct b2Body
{
	int id;
	uint16_t generation;
	b2BodyType type;
	b2WorldTransform transform;
	b2Vec2 linearVelocity;
	float angularVelocity;
	float linearDamping;
	float angularDamping;
	float gravityScale;
	float sleepThreshold;
	float safetyFactor;
	void* userData;
	char name[B2_NAME_LENGTH + 1];
	b2MotionLocks motionLocks;
	bool enableSleep;
	bool isAwake;
	bool isEnabled;
	bool isBullet;
	bool allowFastRotation;
	bool enableContactRecycling;
};

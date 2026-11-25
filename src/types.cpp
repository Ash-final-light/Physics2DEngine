// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math_functions.h"
#include "box2d/types.h"

#include "core.h"

b2WorldDef b2DefaultWorldDef( void )
{
	float lengthUnits = b2GetLengthUnitsPerMeter();
	b2WorldDef def = {};
	def.gravity.x = 0.0f;
	def.gravity.y = -10.0f;
	def.hitEventThreshold = 1.0f * lengthUnits;
	def.restitutionThreshold = 1.0f * lengthUnits;
	def.restitutionIterations = 2;
	def.enableRestitutionPropagation = false;
	def.contactSpeed = 3.0f * lengthUnits;
	def.contactHertz = 30.0f;
	def.contactDampingRatio = 10.0f;
	def.maximumLinearSpeed = 400.0f * lengthUnits;
	def.enableSleep = true;
	def.enableContinuous = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

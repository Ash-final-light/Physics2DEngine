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

b2BodyDef b2DefaultBodyDef( void )
{
	b2BodyDef def = {};
	def.type = b2_staticBody;
	def.rotation = b2Rot_identity;
	def.gravityScale = 1.0f;
	def.sleepThreshold = 0.05f * b2GetLengthUnitsPerMeter();
	def.safetyFactor = 0.5f;
	def.enableSleep = true;
	def.isAwake = true;
	def.isEnabled = true;
	def.enableContactRecycling = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}


b2Filter b2DefaultFilter( void )
{
    return b2Filter{ 1, UINT64_MAX, 0 };
}

b2SurfaceMaterial b2DefaultSurfaceMaterial( void )
{
    b2SurfaceMaterial material = {};
    material.friction = 0.6f;
    return material;
}

b2ShapeDef b2DefaultShapeDef( void )
{
    b2ShapeDef def = {};
    def.material = b2DefaultSurfaceMaterial();
    def.density = 1.0f;
    def.filter = b2DefaultFilter();
    def.invokeContactCreation = true;
    def.updateBodyMass = true;
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}


b2ChainDef b2DefaultChainDef( void )
{
    static b2SurfaceMaterial material = { .friction = 0.6f };
    b2ChainDef def = {};
    def.materials = &material;
    def.materialCount = 1;
    def.ghost1 = { INFINITY, INFINITY };
    def.ghost2 = { INFINITY, INFINITY };
    def.filter = b2DefaultFilter();
    def.internalValue = B2_SECRET_COOKIE;
    return def;
}

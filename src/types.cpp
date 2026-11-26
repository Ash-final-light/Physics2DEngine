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

b2QueryFilter b2DefaultQueryFilter(void)
{ return b2QueryFilter{1,UINT64_MAX}; }

b2ExplosionDef b2DefaultExplosionDef( void )
{
	b2ExplosionDef def = {};
	def.maskBits = UINT64_MAX;
	return def;
}

static void b2EmptyDrawPolygon( b2WorldTransform transform, const b2Vec2* vertices, int vertexCount, b2HexColor color, void* context )
{
	(void)transform; (void)vertices; (void)vertexCount; (void)color; (void)context;
}

static void b2EmptyDrawSolidPolygon( b2WorldTransform transform, const b2Vec2* vertices, int vertexCount, float radius,
									 b2HexColor color, void* context )
{
	(void)transform; (void)vertices; (void)vertexCount; (void)radius; (void)color; (void)context;
}

static void b2EmptyDrawCircle( b2Pos center, float radius, b2HexColor color, void* context )
{
	(void)center; (void)radius; (void)color; (void)context;
}

static void b2EmptyDrawSolidCircle( b2WorldTransform transform, b2Vec2 center, float radius, b2HexColor color, void* context )
{
	(void)transform; (void)center; (void)radius; (void)color; (void)context;
}

static void b2EmptyDrawSolidCapsule( b2Pos p1, b2Pos p2, float radius, b2HexColor color, void* context )
{
	(void)p1; (void)p2; (void)radius; (void)color; (void)context;
}

static void b2EmptyDrawSegment( b2Pos p1, b2Pos p2, b2HexColor color, void* context )
{
	(void)p1; (void)p2; (void)color; (void)context;
}

static void b2EmptyDrawTransform( b2WorldTransform transform, void* context )
{
	(void)transform; (void)context;
}

static void b2EmptyDrawPoint( b2Pos p, float size, b2HexColor color, void* context )
{
	(void)p; (void)size; (void)color; (void)context;
}

static void b2EmptyDrawString( b2Pos p, const char* s, b2HexColor color, void* context )
{
	(void)p; (void)s; (void)color; (void)context;
}

static void b2EmptyDrawBounds( b2AABB aabb, b2HexColor color, void* context )
{
	(void)aabb; (void)color; (void)context;
}

b2DebugDraw b2DefaultDebugDraw( void )
{
	b2DebugDraw draw = {};

	// 允许用户跳过部分实现而不触发空指针异常。
	draw.DrawPolygonFcn = b2EmptyDrawPolygon;
	draw.DrawSolidPolygonFcn = b2EmptyDrawSolidPolygon;
	draw.DrawCircleFcn = b2EmptyDrawCircle;
	draw.DrawSolidCircleFcn = b2EmptyDrawSolidCircle;
	draw.DrawSolidCapsuleFcn = b2EmptyDrawSolidCapsule;
	draw.DrawLineFcn = b2EmptyDrawSegment;
	draw.DrawTransformFcn = b2EmptyDrawTransform;
	draw.DrawPointFcn = b2EmptyDrawPoint;
	draw.DrawStringFcn = b2EmptyDrawString;
	draw.DrawBoundsFcn = b2EmptyDrawBounds;

	draw.drawingBounds.lowerBound = b2Vec2{ -FLT_MAX, -FLT_MAX };
	draw.drawingBounds.upperBound = b2Vec2{ FLT_MAX, FLT_MAX };
	draw.forceScale = 1.0f;
	draw.jointScale = 1.0f;
	draw.drawShapes = true;

	return draw;
}

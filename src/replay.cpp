// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "replay.h"

#include "body.h"
#include "world.h"
#include "state_hash.h"
#include "snapshot.h"

#include "box2d/types.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

// 读取原语

static void b2RecRdrCheck( b2RecReader* rdr, int size )
{
	// 用 64 位比较，防止损坏或过大的 size 使光标加法溢出
	if ( size < 0 || (int64_t)rdr->cursor + (int64_t)size > (int64_t)rdr->size )
	{
		rdr->ok = false;
	}
}

static bool b2RecReserveScratch( b2RecReader* rdr, void** data, int* cap, int need, int elemSize );

uint8_t b2RecR_U8( b2RecReader* rdr )
{
	b2RecRdrCheck( rdr, 1 );
	if ( !rdr->ok )
	{
		return 0;
	}
	return rdr->data[rdr->cursor++];
}

uint16_t b2RecR_U16( b2RecReader* rdr )
{
	b2RecRdrCheck( rdr, 2 );
	if ( !rdr->ok )
	{
		return 0;
	}
	uint16_t v = (uint16_t)rdr->data[rdr->cursor] | ( (uint16_t)rdr->data[rdr->cursor + 1] << 8 );
	rdr->cursor += 2;
	return v;
}

uint32_t b2RecR_U24( b2RecReader* rdr )
{
	b2RecRdrCheck( rdr, 3 );
	if ( !rdr->ok )
	{
		return 0;
	}
	uint32_t v = (uint32_t)rdr->data[rdr->cursor] | ( (uint32_t)rdr->data[rdr->cursor + 1] << 8 ) |
				 ( (uint32_t)rdr->data[rdr->cursor + 2] << 16 );
	rdr->cursor += 3;
	return v;
}

uint32_t b2RecR_U32( b2RecReader* rdr )
{
	b2RecRdrCheck( rdr, 4 );
	if ( !rdr->ok )
	{
		return 0;
	}
	uint32_t v = (uint32_t)rdr->data[rdr->cursor] | ( (uint32_t)rdr->data[rdr->cursor + 1] << 8 ) |
				 ( (uint32_t)rdr->data[rdr->cursor + 2] << 16 ) | ( (uint32_t)rdr->data[rdr->cursor + 3] << 24 );
	rdr->cursor += 4;
	return v;
}

uint64_t b2RecR_U64( b2RecReader* rdr )
{
	b2RecRdrCheck( rdr, 8 );
	if ( !rdr->ok )
	{
		return 0;
	}
	uint64_t v = (uint64_t)rdr->data[rdr->cursor] | ( (uint64_t)rdr->data[rdr->cursor + 1] << 8 ) |
				 ( (uint64_t)rdr->data[rdr->cursor + 2] << 16 ) | ( (uint64_t)rdr->data[rdr->cursor + 3] << 24 ) |
				 ( (uint64_t)rdr->data[rdr->cursor + 4] << 32 ) | ( (uint64_t)rdr->data[rdr->cursor + 5] << 40 ) |
				 ( (uint64_t)rdr->data[rdr->cursor + 6] << 48 ) | ( (uint64_t)rdr->data[rdr->cursor + 7] << 56 );
	rdr->cursor += 8;
	return v;
}

int32_t b2RecR_I32( b2RecReader* rdr )
{
	return (int32_t)b2RecR_U32( rdr );
}

float b2RecR_F32( b2RecReader* rdr )
{
	uint32_t bits = b2RecR_U32( rdr );
	float v;
	memcpy( &v, &bits, 4 );
	return v;
}

bool b2RecR_BOOL( b2RecReader* rdr )
{
	return b2RecR_U8( rdr ) != 0u;
}

b2Vec2 b2RecR_VEC2( b2RecReader* rdr )
{
	b2Vec2 v;
	v.x = b2RecR_F32( rdr );
	v.y = b2RecR_F32( rdr );
	return v;
}

b2Rot b2RecR_ROT( b2RecReader* rdr )
{
	b2Rot r;
	r.c = b2RecR_F32( rdr );
	r.s = b2RecR_F32( rdr );
	return r;
}

b2Transform b2RecR_XF( b2RecReader* rdr )
{
	b2Transform xf;
	xf.p = b2RecR_VEC2( rdr );
	xf.q = b2RecR_ROT( rdr );
	return xf;
}

double b2RecR_F64( b2RecReader* rdr )
{
	uint64_t bits = b2RecR_U64( rdr );
	double v;
	memcpy( &v, &bits, 8 );
	return v;
}

b2Pos b2RecR_POSITION( b2RecReader* rdr )
{
	b2Pos p;
#if defined( BOX2D_DOUBLE_PRECISION )
	p.x = b2RecR_F64( rdr );
	p.y = b2RecR_F64( rdr );
#else
	p.x = b2RecR_F32( rdr );
	p.y = b2RecR_F32( rdr );
#endif
	return p;
}

b2WorldTransform b2RecR_WORLDXF( b2RecReader* rdr )
{
	b2WorldTransform t;
	t.p = b2RecR_POSITION( rdr );
	t.q = b2RecR_ROT( rdr );
	return t;
}

b2WorldId b2RecR_WORLDID( b2RecReader* rdr )
{
	return b2LoadWorldId( b2RecR_U32( rdr ) );
}

b2BodyId b2RecR_BODYID( b2RecReader* rdr )
{
	return b2LoadBodyId( b2RecR_U64( rdr ) );
}

b2ShapeId b2RecR_SHAPEID( b2RecReader* rdr )
{
	return b2LoadShapeId( b2RecR_U64( rdr ) );
}

b2ChainId b2RecR_CHAINID( b2RecReader* rdr )
{
	return b2LoadChainId( b2RecR_U64( rdr ) );
}

b2JointId b2RecR_JOINTID( b2RecReader* rdr )
{
	return b2LoadJointId( b2RecR_U64( rdr ) );
}

// 将给定大小的无指针 POD 数据块读入 out，并推进光标。
// 越界时清零，使截断文件在读取检查时失败，而不是读到垃圾。
static void b2RecRdrBlob( b2RecReader* rdr, void* out, int size )
{
	b2RecRdrCheck( rdr, size );
	if ( !rdr->ok )
	{
		memset( out, 0, (size_t)size );
		return;
	}
	memcpy( out, rdr->data + rdr->cursor, (size_t)size );
	rdr->cursor += size;
}

b2Circle b2RecR_CIRCLE( b2RecReader* rdr )
{
	b2Circle c;
	b2RecRdrBlob( rdr, &c, (int)sizeof( c ) );
	return c;
}

b2Capsule b2RecR_CAPSULE( b2RecReader* rdr )
{
	b2Capsule c;
	b2RecRdrBlob( rdr, &c, (int)sizeof( c ) );
	return c;
}

b2Segment b2RecR_SEGMENT( b2RecReader* rdr )
{
	b2Segment s;
	b2RecRdrBlob( rdr, &s, (int)sizeof( s ) );
	return s;
}

b2Polygon b2RecR_POLYGON( b2RecReader* rdr )
{
	b2Polygon p;
	b2RecRdrBlob( rdr, &p, (int)sizeof( p ) );
	return p;
}

b2ChainSegment b2RecR_CHAINSEG( b2RecReader* rdr )
{
	b2ChainSegment cs;
	b2RecRdrBlob( rdr, &cs, (int)sizeof( cs ) );
	return cs;
}

b2Filter b2RecR_FILTER( b2RecReader* rdr )
{
	b2Filter f;
	f.categoryBits = b2RecR_U64( rdr );
	f.maskBits = b2RecR_U64( rdr );
	f.groupIndex = b2RecR_I32( rdr );
	return f;
}

b2SurfaceMaterial b2RecR_MATERIAL( b2RecReader* rdr )
{
	b2SurfaceMaterial m = b2DefaultSurfaceMaterial();
	m.friction = b2RecR_F32( rdr );
	m.restitution = b2RecR_F32( rdr );
	m.rollingResistance = b2RecR_F32( rdr );
	m.tangentSpeed = b2RecR_F32( rdr );
	m.userMaterialId = b2RecR_U64( rdr );
	m.customColor = b2RecR_U32( rdr );
	return m;
}

b2MassData b2RecR_MASSDATA( b2RecReader* rdr )
{
	b2MassData md;
	md.mass = b2RecR_F32( rdr );
	md.center = b2RecR_VEC2( rdr );
	md.rotationalInertia = b2RecR_F32( rdr );
	return md;
}

b2MotionLocks b2RecR_LOCKS( b2RecReader* rdr )
{
	b2MotionLocks locks;
	locks.linearX = b2RecR_BOOL( rdr );
	locks.linearY = b2RecR_BOOL( rdr );
	locks.angularZ = b2RecR_BOOL( rdr );
	return locks;
}

// 返回指向一组轮换静态缓冲区的指针，直到接下来几次 STR 读取前有效。
// 仅用于在分派期间把名字直接传入创建/设置调用。
const char* b2RecR_STR( b2RecReader* rdr )
{
	static char s_bufs[4][B2_NAME_LENGTH + 1];
	static int s_next = 0;
	char* buf = s_bufs[s_next];
	s_next = ( s_next + 1 ) & 3;

	uint16_t len = b2RecR_U16( rdr );
	if ( len == 0xFFFFu )
	{
		return NULL;
	}

	int n = (int)len;
	if ( n > B2_NAME_LENGTH )
	{
		n = B2_NAME_LENGTH;
	}
	b2RecRdrCheck( rdr, (int)len );
	if ( rdr->ok && n > 0 )
	{
		memcpy( buf, rdr->data + rdr->cursor, (size_t)n );
	}
	// 即使超过夹紧上限也跳过完整的已录制长度
	rdr->cursor += (int)len;
	buf[n] = '\0';
	return buf;
}

// 定义读取器：从 b2Default*Def() 取得 cookie/internalValue，再覆盖字段

b2BodyDef b2RecR_BODYDEF( b2RecReader* rdr )
{
	b2BodyDef def = b2DefaultBodyDef();
	def.type = (b2BodyType)b2RecR_I32( rdr );
	def.position = b2RecR_POSITION( rdr );
	def.rotation = b2RecR_ROT( rdr );
	def.linearVelocity = b2RecR_VEC2( rdr );
	def.angularVelocity = b2RecR_F32( rdr );
	def.linearDamping = b2RecR_F32( rdr );
	def.angularDamping = b2RecR_F32( rdr );
	def.gravityScale = b2RecR_F32( rdr );
	def.sleepThreshold = b2RecR_F32( rdr );
	def.safetyFactor = b2RecR_F32( rdr );

	// b2RecR_STR 处理超长夹紧并跳过完整已录制长度，因此即使名字长于 B2_NAME_LENGTH 光标仍对齐。
	// 直到创建调用前有效。
	def.name = b2RecR_STR( rdr );

	(void)b2RecR_U64( rdr ); // userData：不保留
	def.motionLocks = b2RecR_LOCKS( rdr );
	def.enableSleep = b2RecR_BOOL( rdr );
	def.isAwake = b2RecR_BOOL( rdr );
	def.isBullet = b2RecR_BOOL( rdr );
	def.isEnabled = b2RecR_BOOL( rdr );
	def.allowFastRotation = b2RecR_BOOL( rdr );
	def.enableContactRecycling = b2RecR_BOOL( rdr );
	def.userData = NULL;
	return def;
}

b2ShapeDef b2RecR_SHAPEDEF( b2RecReader* rdr )
{
	b2ShapeDef def = b2DefaultShapeDef();
	(void)b2RecR_U64( rdr ); // userData：不保留
	def.material = b2RecR_MATERIAL( rdr );
	def.density = b2RecR_F32( rdr );
	def.filter = b2RecR_FILTER( rdr );
	def.enableCustomFiltering = b2RecR_BOOL( rdr );
	def.isSensor = b2RecR_BOOL( rdr );
	def.enableSensorEvents = b2RecR_BOOL( rdr );
	def.enableContactEvents = b2RecR_BOOL( rdr );
	def.enableHitEvents = b2RecR_BOOL( rdr );
	def.enablePreSolveEvents = b2RecR_BOOL( rdr );
	def.invokeContactCreation = b2RecR_BOOL( rdr );
	def.updateBodyMass = b2RecR_BOOL( rdr );
	def.userData = NULL;
	return def;
}

b2ChainDef b2RecR_CHAINDEF( b2RecReader* rdr )
{
	b2ChainDef def = b2DefaultChainDef();
	(void)b2RecR_U64( rdr ); // userData：不保留

	int pointCount = b2RecR_I32( rdr );
	if ( pointCount < 0 )
	{
		pointCount = 0;
	}
	if ( b2RecReserveScratch( rdr, (void**)&rdr->chainPoints, &rdr->chainPointCap, pointCount, (int)sizeof( b2Vec2 ) ) == false )
	{
		pointCount = 0; // 计数损坏，读取已经失败
	}
	for ( int i = 0; i < pointCount; ++i )
	{
		rdr->chainPoints[i] = b2RecR_VEC2( rdr );
	}
	def.points = pointCount > 0 ? rdr->chainPoints : NULL;
	def.pointCount = pointCount;
	def.ghost1 = b2RecR_VEC2( rdr );
	def.ghost2 = b2RecR_VEC2( rdr );

	int materialCount = b2RecR_I32( rdr );
	if ( materialCount < 0 )
	{
		materialCount = 0;
	}
	if ( b2RecReserveScratch( rdr, (void**)&rdr->chainMaterials, &rdr->chainMaterialCap, materialCount,
							  (int)sizeof( b2SurfaceMaterial ) ) == false )
	{
		materialCount = 0;
	}
	for ( int i = 0; i < materialCount; ++i )
	{
		rdr->chainMaterials[i] = b2RecR_MATERIAL( rdr );
	}
	def.materials = materialCount > 0 ? rdr->chainMaterials : NULL;
	def.materialCount = materialCount;

	def.filter = b2RecR_FILTER( rdr );
	def.isLoop = b2RecR_BOOL( rdr );
	def.enableSensorEvents = b2RecR_BOOL( rdr );
	def.userData = NULL;
	return def;
}

b2ExplosionDef b2RecR_EXPLOSIONDEF( b2RecReader* rdr )
{
	b2ExplosionDef def = b2DefaultExplosionDef();
	def.maskBits = b2RecR_U64( rdr );
	def.position = b2RecR_POSITION( rdr );
	def.radius = b2RecR_F32( rdr );
	def.falloff = b2RecR_F32( rdr );
	def.impulsePerLength = b2RecR_F32( rdr );
	return def;
}

// 刚体 id 带着录制时的 world0 读取；创建分派器会重映射它们。
static void b2RecR_JointBase( b2RecReader* rdr, b2JointDef* base )
{
	(void)b2RecR_U64( rdr ); // userData：不保留
	base->bodyIdA = b2RecR_BODYID( rdr );
	base->bodyIdB = b2RecR_BODYID( rdr );
	base->localFrameA = b2RecR_XF( rdr );
	base->localFrameB = b2RecR_XF( rdr );
	base->forceThreshold = b2RecR_F32( rdr );
	base->torqueThreshold = b2RecR_F32( rdr );
	base->constraintHertz = b2RecR_F32( rdr );
	base->constraintDampingRatio = b2RecR_F32( rdr );
	base->drawScale = b2RecR_F32( rdr );
	base->collideConnected = b2RecR_BOOL( rdr );
	base->userData = NULL;
}

b2DistanceJointDef b2RecR_DISTANCEJOINTDEF( b2RecReader* rdr )
{
	b2DistanceJointDef def = b2DefaultDistanceJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.length = b2RecR_F32( rdr );
	def.enableSpring = b2RecR_BOOL( rdr );
	def.lowerSpringForce = b2RecR_F32( rdr );
	def.upperSpringForce = b2RecR_F32( rdr );
	def.hertz = b2RecR_F32( rdr );
	def.dampingRatio = b2RecR_F32( rdr );
	def.enableLimit = b2RecR_BOOL( rdr );
	def.minLength = b2RecR_F32( rdr );
	def.maxLength = b2RecR_F32( rdr );
	def.enableMotor = b2RecR_BOOL( rdr );
	def.maxMotorForce = b2RecR_F32( rdr );
	def.motorSpeed = b2RecR_F32( rdr );
	return def;
}

b2FilterJointDef b2RecR_FILTERJOINTDEF( b2RecReader* rdr )
{
	b2FilterJointDef def = b2DefaultFilterJointDef();
	b2RecR_JointBase( rdr, &def.base );
	return def;
}

b2MotorJointDef b2RecR_MOTORJOINTDEF( b2RecReader* rdr )
{
	b2MotorJointDef def = b2DefaultMotorJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.linearVelocity = b2RecR_VEC2( rdr );
	def.maxVelocityForce = b2RecR_F32( rdr );
	def.angularVelocity = b2RecR_F32( rdr );
	def.maxVelocityTorque = b2RecR_F32( rdr );
	def.linearHertz = b2RecR_F32( rdr );
	def.linearDampingRatio = b2RecR_F32( rdr );
	def.maxSpringForce = b2RecR_F32( rdr );
	def.angularHertz = b2RecR_F32( rdr );
	def.angularDampingRatio = b2RecR_F32( rdr );
	def.maxSpringTorque = b2RecR_F32( rdr );
	return def;
}

b2MoverJointDef b2RecR_MOVERJOINTDEF( b2RecReader* rdr )
{
	b2MoverJointDef def = b2DefaultMoverJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.linearVelocity = b2RecR_VEC2( rdr );
	def.maxVelocityForce = b2RecR_VEC2( rdr );
	return def;
}

b2PrismaticJointDef b2RecR_PRISMATICJOINTDEF( b2RecReader* rdr )
{
	b2PrismaticJointDef def = b2DefaultPrismaticJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.enableSpring = b2RecR_BOOL( rdr );
	def.hertz = b2RecR_F32( rdr );
	def.dampingRatio = b2RecR_F32( rdr );
	def.targetTranslation = b2RecR_F32( rdr );
	def.enableLimit = b2RecR_BOOL( rdr );
	def.lowerTranslation = b2RecR_F32( rdr );
	def.upperTranslation = b2RecR_F32( rdr );
	def.enableMotor = b2RecR_BOOL( rdr );
	def.maxMotorForce = b2RecR_F32( rdr );
	def.motorSpeed = b2RecR_F32( rdr );
	return def;
}

b2PogoJointDef b2RecR_POGOJOINTDEF( b2RecReader* rdr )
{
	b2PogoJointDef def = b2DefaultPogoJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.normal = b2RecR_VEC2( rdr );
	def.hertz = b2RecR_F32( rdr );
	def.dampingRatio = b2RecR_F32( rdr );
	def.restLength = b2RecR_F32( rdr );
	def.maxTensionForce = b2RecR_F32( rdr );
	def.maxCompressionForce = b2RecR_F32( rdr );
	def.impulse = b2RecR_F32( rdr );
	def.velocity = b2RecR_F32( rdr );
	return def;
}

b2RevoluteJointDef b2RecR_REVOLUTEJOINTDEF( b2RecReader* rdr )
{
	b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.targetAngle = b2RecR_F32( rdr );
	def.enableSpring = b2RecR_BOOL( rdr );
	def.hertz = b2RecR_F32( rdr );
	def.dampingRatio = b2RecR_F32( rdr );
	def.enableLimit = b2RecR_BOOL( rdr );
	def.lowerAngle = b2RecR_F32( rdr );
	def.upperAngle = b2RecR_F32( rdr );
	def.enableMotor = b2RecR_BOOL( rdr );
	def.maxMotorTorque = b2RecR_F32( rdr );
	def.motorSpeed = b2RecR_F32( rdr );
	return def;
}

b2WeldJointDef b2RecR_WELDJOINTDEF( b2RecReader* rdr )
{
	b2WeldJointDef def = b2DefaultWeldJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.linearHertz = b2RecR_F32( rdr );
	def.angularHertz = b2RecR_F32( rdr );
	def.linearDampingRatio = b2RecR_F32( rdr );
	def.angularDampingRatio = b2RecR_F32( rdr );
	return def;
}

b2WheelJointDef b2RecR_WHEELJOINTDEF( b2RecReader* rdr )
{
	b2WheelJointDef def = b2DefaultWheelJointDef();
	b2RecR_JointBase( rdr, &def.base );
	def.enableSpring = b2RecR_BOOL( rdr );
	def.hertz = b2RecR_F32( rdr );
	def.dampingRatio = b2RecR_F32( rdr );
	def.enableLimit = b2RecR_BOOL( rdr );
	def.lowerTranslation = b2RecR_F32( rdr );
	def.upperTranslation = b2RecR_F32( rdr );
	def.enableMotor = b2RecR_BOOL( rdr );
	def.maxMotorTorque = b2RecR_F32( rdr );
	def.motorSpeed = b2RecR_F32( rdr );
	return def;
}

b2AABB b2RecR_AABB( b2RecReader* rdr )
{
	b2AABB v;
	v.lowerBound = b2RecR_VEC2( rdr );
	v.upperBound = b2RecR_VEC2( rdr );
	return v;
}

b2QueryFilter b2RecR_QUERYFILTER( b2RecReader* rdr )
{
	b2QueryFilter f;
	f.categoryBits = b2RecR_U64( rdr );
	f.maskBits = b2RecR_U64( rdr );
	return f;
}

b2ShapeProxy b2RecR_SHAPEPROXY( b2RecReader* rdr )
{
	b2ShapeProxy p;
	memset( &p, 0, sizeof( p ) );
	int count = b2RecR_I32( rdr );
	if ( count < 0 )
		count = 0;
	if ( count > B2_MAX_POLYGON_VERTICES )
		count = B2_MAX_POLYGON_VERTICES;
	p.count = count;
	for ( int i = 0; i < count; ++i )
	{
		p.points[i] = b2RecR_VEC2( rdr );
	}
	p.radius = b2RecR_F32( rdr );
	return p;
}

b2RayResult b2RecR_RAYRESULT( b2RecReader* rdr )
{
	b2RayResult v;
	// shapeId 保留录制时的 world0；比较时再应用 b2RecMakeShapeId
	v.shapeId = b2RecR_SHAPEID( rdr );
	v.point = b2RecR_POSITION( rdr );
	v.normal = b2RecR_VEC2( rdr );
	v.fraction = b2RecR_F32( rdr );
	v.nodeVisits = b2RecR_I32( rdr );
	v.leafVisits = b2RecR_I32( rdr );
	v.hit = b2RecR_BOOL( rdr );
	return v;
}

b2PlaneResult b2RecR_PLANERESULT( b2RecReader* rdr )
{
	b2PlaneResult v;
	v.plane.normal = b2RecR_VEC2( rdr );
	v.plane.offset = b2RecR_F32( rdr );
	v.point = b2RecR_VEC2( rdr );
	v.hit = b2RecR_BOOL( rdr );
	return v;
}

b2TreeStats b2RecR_TREESTATS( b2RecReader* rdr )
{
	b2TreeStats v;
	v.nodeVisits = b2RecR_I32( rdr );
	v.leafVisits = b2RecR_I32( rdr );
	return v;
}

// 为来自不可信文件的计数预留读取器暂存。每个已录制元素至少占一字节，
// 因此有效计数绝不会超过文件剩余字节。拒绝更大（或负数，或会使字节大小溢出）的值，
// 使读取失败而不是疯狂分配。内容不保留；调用者在使用前覆盖。
static bool b2RecReserveScratch( b2RecReader* rdr, void** data, int* cap, int need, int elemSize )
{
	int remaining = rdr->size - rdr->cursor;
	if ( need < 0 || remaining < 0 || need > remaining || need > INT_MAX / elemSize )
	{
		rdr->ok = false;
		return false;
	}
	if ( need <= *cap )
	{
		return true;
	}
	int newCap = need <= INT_MAX / elemSize - 8 ? need + 8 : need;
	if ( *data != NULL )
	{
		b2Free( *data, (size_t)*cap * (size_t)elemSize );
	}
	*data = b2Alloc( (size_t)newCap * (size_t)elemSize );
	*cap = newCap;
	return true;
}

// 播放器累计绘制数组的溢出安全增长。计数来自回放本身而非文件，因此只防护字节大小乘法。
// 保留 keep 个元素。


void b2RecEnsureHits( b2RecReader* rdr, int n )
{
	b2RecReserveScratch( rdr, (void**)&rdr->hits, &rdr->hitCap, n, (int)sizeof( b2RecRecordedHit ) );
}

// 每个操作的分派，唯一出现真实公开 API 名的地方
// 刚体和形状 id 的 world0 被替换为回放世界的槽索引

static b2BodyId b2RecMakeBodyId( b2RecReader* rdr, b2BodyId recorded )
{
	b2BodyId id;
	id.index1 = recorded.index1;
	id.world0 = (uint16_t)( rdr->replayWorldId.index1 - 1u );
	id.generation = recorded.generation;
	return id;
}

static b2ShapeId b2RecMakeShapeId( b2RecReader* rdr, b2ShapeId recorded )
{
	b2ShapeId id;
	id.index1 = recorded.index1;
	id.world0 = (uint16_t)( rdr->replayWorldId.index1 - 1u );
	id.generation = recorded.generation;
	return id;
}

static b2ChainId b2RecMakeChainId( b2RecReader* rdr, b2ChainId recorded )
{
	b2ChainId id;
	id.index1 = recorded.index1;
	id.world0 = (uint16_t)( rdr->replayWorldId.index1 - 1u );
	id.generation = recorded.generation;
	return id;
}

static b2JointId b2RecMakeJointId( b2RecReader* rdr, b2JointId recorded )
{
	b2JointId id;
	id.index1 = recorded.index1;
	id.world0 = (uint16_t)( rdr->replayWorldId.index1 - 1u );
	id.generation = recorded.generation;
	return id;
}

// 创建操作在参数后追加返回的 id。回放只比较 index1 和 generation，
// 因为录制与回放的 world0 不同。不匹配表示结构漂移。

static void b2RecCheckId( b2RecReader* rdr, const char* kind, int gotIndex, unsigned gotGen, int recIndex, unsigned recGen )
{
	if ( gotIndex != recIndex || gotGen != recGen )
	{
		printf( "b2ReplayFile: %s id mismatch (rec index1=%d gen=%u, got index1=%d gen=%u)\n", kind, recIndex, recGen, gotIndex,
				gotGen );
		rdr->ok = false;
	}
}

static void b2RecCheckBodyId( b2RecReader* rdr, b2BodyId got, b2BodyId rec )
{
	b2RecCheckId( rdr, "body", got.index1, got.generation, rec.index1, rec.generation );
}

static void b2RecCheckShapeId( b2RecReader* rdr, b2ShapeId got, b2ShapeId rec )
{
	b2RecCheckId( rdr, "shape", got.index1, got.generation, rec.index1, rec.generation );
}

static void b2RecCheckChainId( b2RecReader* rdr, b2ChainId got, b2ChainId rec )
{
	b2RecCheckId( rdr, "chain", got.index1, got.generation, rec.index1, rec.generation );
}

static void b2RecCheckJointId( b2RecReader* rdr, b2JointId got, b2JointId rec )
{
	b2RecCheckId( rdr, "joint", got.index1, got.generation, rec.index1, rec.generation );
}

static void b2RecDispatch_DestroyWorld( const b2RecArgs_DestroyWorld* a, b2RecReader* rdr )
{
	(void)a;
	(void)rdr;
	// 已录制会话在此结束。播放器持有回放世界的生命周期，并在 b2DestroyReplay/Restart 中拆除，
	// 以便查看器可继续绘制最后一步。每个录制一个世界，且这总是最后一条记录，因此保持存活是安全的。
}

static void b2RecDispatch_Step( const b2RecArgs_Step* a, b2RecReader* rdr )
{
	(void)a;
	b2World_Step( rdr->replayWorldId, a->dt, a->subStepCount );
}

static void b2RecDispatch_WorldEnableSleeping( const b2RecArgs_WorldEnableSleeping* a, b2RecReader* rdr )
{
	b2World_EnableSleeping( rdr->replayWorldId, a->flag );
}

static void b2RecDispatch_WorldEnableContinuous( const b2RecArgs_WorldEnableContinuous* a, b2RecReader* rdr )
{
	b2World_EnableContinuous( rdr->replayWorldId, a->flag );
}

static void b2RecDispatch_WorldSetRestitutionThreshold( const b2RecArgs_WorldSetRestitutionThreshold* a, b2RecReader* rdr )
{
	b2World_SetRestitutionThreshold( rdr->replayWorldId, a->value );
}

static void b2RecDispatch_WorldSetHitEventThreshold( const b2RecArgs_WorldSetHitEventThreshold* a, b2RecReader* rdr )
{
	b2World_SetHitEventThreshold( rdr->replayWorldId, a->value );
}

static void b2RecDispatch_WorldSetGravity( const b2RecArgs_WorldSetGravity* a, b2RecReader* rdr )
{
	b2World_SetGravity( rdr->replayWorldId, a->gravity );
}

static void b2RecDispatch_WorldExplode( const b2RecArgs_WorldExplode* a, b2RecReader* rdr )
{
	b2World_Explode( rdr->replayWorldId, &a->def );
}

static void b2RecDispatch_WorldSetContactTuning( const b2RecArgs_WorldSetContactTuning* a, b2RecReader* rdr )
{
	b2World_SetContactTuning( rdr->replayWorldId, a->hertz, a->dampingRatio, a->pushSpeed );
}

static void b2RecDispatch_WorldSetContactRecycleDistance( const b2RecArgs_WorldSetContactRecycleDistance* a, b2RecReader* rdr )
{
	b2World_SetContactRecycleDistance( rdr->replayWorldId, a->recycleDistance );
}

static void b2RecDispatch_WorldSetMaxLinearSpeed( const b2RecArgs_WorldSetMaxLinearSpeed* a, b2RecReader* rdr )
{
	b2World_SetMaxLinearSpeed( rdr->replayWorldId, a->maximumLinearSpeed );
}

static void b2RecDispatch_WorldEnableWarmStarting( const b2RecArgs_WorldEnableWarmStarting* a, b2RecReader* rdr )
{
	b2World_EnableWarmStarting( rdr->replayWorldId, a->flag );
}

static void b2RecDispatch_WorldRebuildStaticTree( const b2RecArgs_WorldRebuildStaticTree* a, b2RecReader* rdr )
{
	(void)a;
	b2World_RebuildStaticTree( rdr->replayWorldId );
}

static void b2RecDispatch_WorldSetRestitutionIterations( const b2RecArgs_WorldSetRestitutionIterations* a, b2RecReader* rdr )
{
	b2World_SetRestitutionIterations( rdr->replayWorldId, a->iterations );
}

static void b2RecDispatch_WorldEnableRestitutionPropagation( const b2RecArgs_WorldEnableRestitutionPropagation* a,
															 b2RecReader* rdr )
{
	b2World_EnableRestitutionPropagation( rdr->replayWorldId, a->flag );
}


static void b2RecGrow( void** data, int* capacity, int need, int keep, int elemSize )
{
	if ( need <= *capacity )
	{
		return;
	}
	int newCap = *capacity == 0 ? 8 : 2 * *capacity;
	if ( newCap < need )
	{
		newCap = need;
	}
	void* grown = b2Alloc( (size_t)newCap * (size_t)elemSize );
	if ( *data != NULL )
	{
		if ( keep > 0 )
		{
			memcpy( grown, *data, (size_t)keep * (size_t)elemSize );
		}
		b2Free( *data, (size_t)*capacity * (size_t)elemSize );
	}
	*data = grown;
	*capacity = newCap;
}

static void b2RecTrackBodyCreate( b2Replay* player, b2BodyId id )
{
	b2RecGrow( (void**)&player->bodyIds, &player->bodyIdCap, player->bodyIdCount + 1, player->bodyIdCount,
			   (int)sizeof( b2BodyId ) );
	player->bodyIds[player->bodyIdCount] = id;
	player->bodyIdCount += 1;
}

static void b2RecTrackBodyDestroy( b2Replay* player, b2BodyId id )
{
	for ( int i = 0; i < player->bodyIdCount; ++i )
	{
		if ( B2_ID_EQUALS( player->bodyIds[i], id ) )
		{
			player->bodyIds[i] = b2_nullBodyId;
			return;
		}
	}
}

static void b2RecSeedBodyIds( b2Replay* player )
{
	b2World* world = ( b2_worlds + player->rdr.replayWorldId.index1 - 1 );
	player->bodyIdCount = 0;
	int count = world->bodies.count;
	for ( int i = 0; i < count; ++i )
	{
		if ( world->bodies.data[i].id != i )
		{
			continue; // 空闲槽
		}
		b2RecTrackBodyCreate( player, b2MakeBodyId( world, i ) );
	}
}

static void b2RecDispatch_CreateBody( const b2RecArgs_CreateBody* a, b2RecReader* rdr )
{
	// 已录制 id 追加在参数之后（在 b2RecEndRecord 之前写入）
	b2BodyId recId = b2RecR_BODYID( rdr );
	b2BodyId gotId = b2CreateBody( rdr->replayWorldId, &a->def );
	b2RecCheckBodyId( rdr, gotId, recId );
    if ( rdr->owner != NULL ) { b2RecTrackBodyCreate( rdr->owner, gotId ); }

}

static void b2RecDispatch_DestroyBody( const b2RecArgs_DestroyBody* a, b2RecReader* rdr )
{
	b2BodyId id = b2RecMakeBodyId( rdr, a->body );
    if ( rdr->owner != NULL ) { b2RecTrackBodyDestroy( rdr->owner, id ); }

	b2DestroyBody( id );
}

static void b2RecDispatch_BodySetTransform( const b2RecArgs_BodySetTransform* a, b2RecReader* rdr )
{
	b2BodyId id = b2RecMakeBodyId( rdr, a->body );
	b2Body_SetTransform( id, a->position, a->rotation );
}

static void b2RecDispatch_BodySetLinearVelocity( const b2RecArgs_BodySetLinearVelocity* a, b2RecReader* rdr )
{
	b2BodyId id = b2RecMakeBodyId( rdr, a->body );
	b2Body_SetLinearVelocity( id, a->v );
}

static void b2RecDispatch_BodySetType( const b2RecArgs_BodySetType* a, b2RecReader* rdr )
{
	b2Body_SetType( b2RecMakeBodyId( rdr, a->body ), (b2BodyType)a->type );
}

static void b2RecDispatch_BodySetName( const b2RecArgs_BodySetName* a, b2RecReader* rdr )
{
	b2Body_SetName( b2RecMakeBodyId( rdr, a->body ), a->name );
}

static void b2RecDispatch_BodySetAngularVelocity( const b2RecArgs_BodySetAngularVelocity* a, b2RecReader* rdr )
{
	b2Body_SetAngularVelocity( b2RecMakeBodyId( rdr, a->body ), a->w );
}

static void b2RecDispatch_BodySetTargetTransform( const b2RecArgs_BodySetTargetTransform* a, b2RecReader* rdr )
{
	b2Body_SetTargetTransform( b2RecMakeBodyId( rdr, a->body ), a->target, a->timeStep, a->wake );
}

static void b2RecDispatch_BodyApplyForce( const b2RecArgs_BodyApplyForce* a, b2RecReader* rdr )
{
	b2Body_ApplyForce( b2RecMakeBodyId( rdr, a->body ), a->force, a->point, a->wake );
}

static void b2RecDispatch_BodyApplyForceToCenter( const b2RecArgs_BodyApplyForceToCenter* a, b2RecReader* rdr )
{
	b2Body_ApplyForceToCenter( b2RecMakeBodyId( rdr, a->body ), a->force, a->wake );
}

static void b2RecDispatch_BodyApplyTorque( const b2RecArgs_BodyApplyTorque* a, b2RecReader* rdr )
{
	b2Body_ApplyTorque( b2RecMakeBodyId( rdr, a->body ), a->torque, a->wake );
}

static void b2RecDispatch_BodyClearForces( const b2RecArgs_BodyClearForces* a, b2RecReader* rdr )
{
	b2Body_ClearForces( b2RecMakeBodyId( rdr, a->body ) );
}

static void b2RecDispatch_BodyApplyLinearImpulse( const b2RecArgs_BodyApplyLinearImpulse* a, b2RecReader* rdr )
{
	b2Body_ApplyLinearImpulse( b2RecMakeBodyId( rdr, a->body ), a->impulse, a->point, a->wake );
}

static void b2RecDispatch_BodyApplyLinearImpulseToCenter( const b2RecArgs_BodyApplyLinearImpulseToCenter* a, b2RecReader* rdr )
{
	b2Body_ApplyLinearImpulseToCenter( b2RecMakeBodyId( rdr, a->body ), a->impulse, a->wake );
}

static void b2RecDispatch_BodyApplyAngularImpulse( const b2RecArgs_BodyApplyAngularImpulse* a, b2RecReader* rdr )
{
	b2Body_ApplyAngularImpulse( b2RecMakeBodyId( rdr, a->body ), a->impulse, a->wake );
}

static void b2RecDispatch_BodySetMassData( const b2RecArgs_BodySetMassData* a, b2RecReader* rdr )
{
	b2Body_SetMassData( b2RecMakeBodyId( rdr, a->body ), a->massData );
}

static void b2RecDispatch_BodyUpdateMassFromShapes( const b2RecArgs_BodyUpdateMassFromShapes* a, b2RecReader* rdr )
{
	b2Body_UpdateMassFromShapes( b2RecMakeBodyId( rdr, a->body ) );
}

static void b2RecDispatch_BodySetLinearDamping( const b2RecArgs_BodySetLinearDamping* a, b2RecReader* rdr )
{
	b2Body_SetLinearDamping( b2RecMakeBodyId( rdr, a->body ), a->damping );
}

static void b2RecDispatch_BodySetAngularDamping( const b2RecArgs_BodySetAngularDamping* a, b2RecReader* rdr )
{
	b2Body_SetAngularDamping( b2RecMakeBodyId( rdr, a->body ), a->damping );
}

static void b2RecDispatch_BodySetGravityScale( const b2RecArgs_BodySetGravityScale* a, b2RecReader* rdr )
{
	b2Body_SetGravityScale( b2RecMakeBodyId( rdr, a->body ), a->scale );
}

static void b2RecDispatch_BodySetAwake( const b2RecArgs_BodySetAwake* a, b2RecReader* rdr )
{
	b2Body_SetAwake( b2RecMakeBodyId( rdr, a->body ), a->awake );
}

static void b2RecDispatch_BodyWakeTouching( const b2RecArgs_BodyWakeTouching* a, b2RecReader* rdr )
{
	b2Body_WakeTouching( b2RecMakeBodyId( rdr, a->body ) );
}

static void b2RecDispatch_BodyEnableSleeping( const b2RecArgs_BodyEnableSleeping* a, b2RecReader* rdr )
{
	b2Body_EnableSleeping( b2RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b2RecDispatch_BodySetSleepThreshold( const b2RecArgs_BodySetSleepThreshold* a, b2RecReader* rdr )
{
	b2Body_SetSleepThreshold( b2RecMakeBodyId( rdr, a->body ), a->threshold );
}

static void b2RecDispatch_BodySetSafetyFactor( const b2RecArgs_BodySetSafetyFactor* a, b2RecReader* rdr )
{
	b2Body_SetSafetyFactor( b2RecMakeBodyId( rdr, a->body ), a->value );
}

static void b2RecDispatch_BodyDisable( const b2RecArgs_BodyDisable* a, b2RecReader* rdr )
{
	b2Body_Disable( b2RecMakeBodyId( rdr, a->body ) );
}

static void b2RecDispatch_BodyEnable( const b2RecArgs_BodyEnable* a, b2RecReader* rdr )
{
	b2Body_Enable( b2RecMakeBodyId( rdr, a->body ) );
}

static void b2RecDispatch_BodySetMotionLocks( const b2RecArgs_BodySetMotionLocks* a, b2RecReader* rdr )
{
	b2Body_SetMotionLocks( b2RecMakeBodyId( rdr, a->body ), a->locks );
}

static void b2RecDispatch_BodySetBullet( const b2RecArgs_BodySetBullet* a, b2RecReader* rdr )
{
	b2Body_SetBullet( b2RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b2RecDispatch_BodyEnableContactRecycling( const b2RecArgs_BodyEnableContactRecycling* a, b2RecReader* rdr )
{
	b2Body_EnableContactRecycling( b2RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b2RecDispatch_BodyEnableContactEvents( const b2RecArgs_BodyEnableContactEvents* a, b2RecReader* rdr )
{
	b2Body_EnableContactEvents( b2RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b2RecDispatch_BodyEnableHitEvents( const b2RecArgs_BodyEnableHitEvents* a, b2RecReader* rdr )
{
	b2Body_EnableHitEvents( b2RecMakeBodyId( rdr, a->body ), a->flag );
}

static void b2RecDispatch_CreateCircleShape( const b2RecArgs_CreateCircleShape* a, b2RecReader* rdr )
{
	b2ShapeId recId = b2RecR_SHAPEID( rdr );
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ShapeId gotId = b2CreateCircleShape( bodyId, &a->def, &a->circle );
	b2RecCheckShapeId( rdr, gotId, recId );
}

static void b2RecDispatch_CreateCapsuleShape( const b2RecArgs_CreateCapsuleShape* a, b2RecReader* rdr )
{
	b2ShapeId recId = b2RecR_SHAPEID( rdr );
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ShapeId gotId = b2CreateCapsuleShape( bodyId, &a->def, &a->capsule );
	b2RecCheckShapeId( rdr, gotId, recId );
}

static void b2RecDispatch_CreateSegmentShape( const b2RecArgs_CreateSegmentShape* a, b2RecReader* rdr )
{
	b2ShapeId recId = b2RecR_SHAPEID( rdr );
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ShapeId gotId = b2CreateSegmentShape( bodyId, &a->def, &a->segment );
	b2RecCheckShapeId( rdr, gotId, recId );
}

static void b2RecDispatch_CreatePolygonShape( const b2RecArgs_CreatePolygonShape* a, b2RecReader* rdr )
{
	b2ShapeId recId = b2RecR_SHAPEID( rdr );
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ShapeId gotId = b2CreatePolygonShape( bodyId, &a->def, &a->polygon );
	b2RecCheckShapeId( rdr, gotId, recId );
}

static void b2RecDispatch_CreateChainSegmentShape( const b2RecArgs_CreateChainSegmentShape* a, b2RecReader* rdr )
{
	b2ShapeId recId = b2RecR_SHAPEID( rdr );
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ShapeId gotId = b2CreateChainSegmentShape( bodyId, &a->def, &a->chainSegment );
	b2RecCheckShapeId( rdr, gotId, recId );
}

static void b2RecDispatch_DestroyShape( const b2RecArgs_DestroyShape* a, b2RecReader* rdr )
{
	b2DestroyShape( b2RecMakeShapeId( rdr, a->shape ), a->updateBodyMass );
}

static void b2RecDispatch_ShapeSetDensity( const b2RecArgs_ShapeSetDensity* a, b2RecReader* rdr )
{
	b2Shape_SetDensity( b2RecMakeShapeId( rdr, a->shape ), a->density, a->updateBodyMass );
}

static void b2RecDispatch_ShapeSetFriction( const b2RecArgs_ShapeSetFriction* a, b2RecReader* rdr )
{
	b2Shape_SetFriction( b2RecMakeShapeId( rdr, a->shape ), a->friction );
}

static void b2RecDispatch_ShapeSetRestitution( const b2RecArgs_ShapeSetRestitution* a, b2RecReader* rdr )
{
	b2Shape_SetRestitution( b2RecMakeShapeId( rdr, a->shape ), a->restitution );
}

static void b2RecDispatch_ShapeSetUserMaterial( const b2RecArgs_ShapeSetUserMaterial* a, b2RecReader* rdr )
{
	b2Shape_SetUserMaterial( b2RecMakeShapeId( rdr, a->shape ), a->material );
}

static void b2RecDispatch_ShapeSetSurfaceMaterial( const b2RecArgs_ShapeSetSurfaceMaterial* a, b2RecReader* rdr )
{
	b2Shape_SetSurfaceMaterial( b2RecMakeShapeId( rdr, a->shape ), &a->material );
}

static void b2RecDispatch_ShapeSetFilter( const b2RecArgs_ShapeSetFilter* a, b2RecReader* rdr )
{
	b2Shape_SetFilter( b2RecMakeShapeId( rdr, a->shape ), a->filter );
}

static void b2RecDispatch_ShapeEnableSensorEvents( const b2RecArgs_ShapeEnableSensorEvents* a, b2RecReader* rdr )
{
	b2Shape_EnableSensorEvents( b2RecMakeShapeId( rdr, a->shape ), a->flag );
}

static void b2RecDispatch_ShapeEnableContactEvents( const b2RecArgs_ShapeEnableContactEvents* a, b2RecReader* rdr )
{
	b2Shape_EnableContactEvents( b2RecMakeShapeId( rdr, a->shape ), a->flag );
}

static void b2RecDispatch_ShapeEnablePreSolveEvents( const b2RecArgs_ShapeEnablePreSolveEvents* a, b2RecReader* rdr )
{
	b2Shape_EnablePreSolveEvents( b2RecMakeShapeId( rdr, a->shape ), a->flag );
}

static void b2RecDispatch_ShapeEnableHitEvents( const b2RecArgs_ShapeEnableHitEvents* a, b2RecReader* rdr )
{
	b2Shape_EnableHitEvents( b2RecMakeShapeId( rdr, a->shape ), a->flag );
}

static void b2RecDispatch_ShapeSetCircle( const b2RecArgs_ShapeSetCircle* a, b2RecReader* rdr )
{
	b2Shape_SetCircle( b2RecMakeShapeId( rdr, a->shape ), &a->circle );
}

static void b2RecDispatch_ShapeSetCapsule( const b2RecArgs_ShapeSetCapsule* a, b2RecReader* rdr )
{
	b2Shape_SetCapsule( b2RecMakeShapeId( rdr, a->shape ), &a->capsule );
}

static void b2RecDispatch_ShapeSetSegment( const b2RecArgs_ShapeSetSegment* a, b2RecReader* rdr )
{
	b2Shape_SetSegment( b2RecMakeShapeId( rdr, a->shape ), &a->segment );
}

static void b2RecDispatch_ShapeSetPolygon( const b2RecArgs_ShapeSetPolygon* a, b2RecReader* rdr )
{
	b2Shape_SetPolygon( b2RecMakeShapeId( rdr, a->shape ), &a->polygon );
}

static void b2RecDispatch_ShapeSetChainSegment( const b2RecArgs_ShapeSetChainSegment* a, b2RecReader* rdr )
{
	b2Shape_SetChainSegment( b2RecMakeShapeId( rdr, a->shape ), &a->chainSegment );
}

static void b2RecDispatch_ShapeApplyWind( const b2RecArgs_ShapeApplyWind* a, b2RecReader* rdr )
{
	b2Shape_ApplyWind( b2RecMakeShapeId( rdr, a->shape ), a->wind, a->drag, a->lift, a->wake );
}

static void b2RecDispatch_CreateChain( const b2RecArgs_CreateChain* a, b2RecReader* rdr )
{
	b2ChainId recId = b2RecR_CHAINID( rdr );
	if ( !rdr->ok )
	{
		// 损坏的点/材质计数使定义退化，不要据此构建链条
		return;
	}
	b2BodyId bodyId = b2RecMakeBodyId( rdr, a->body );
	b2ChainId gotId = b2CreateChain( bodyId, &a->def );
	b2RecCheckChainId( rdr, gotId, recId );
}

static void b2RecDispatch_DestroyChain( const b2RecArgs_DestroyChain* a, b2RecReader* rdr )
{
	b2DestroyChain( b2RecMakeChainId( rdr, a->chain ) );
}

static void b2RecDispatch_ChainSetSurfaceMaterial( const b2RecArgs_ChainSetSurfaceMaterial* a, b2RecReader* rdr )
{
	b2Chain_SetSurfaceMaterial( b2RecMakeChainId( rdr, a->chain ), &a->material, a->segmentIndex );
}

static void b2RecDispatch_ChainSetAllSurfaceMaterials( const b2RecArgs_ChainSetAllSurfaceMaterials* a, b2RecReader* rdr )
{
	b2Chain_SetAllSurfaceMaterials( b2RecMakeChainId( rdr, a->chain ), &a->material );
}

// 关节创建：调用前将定义中的刚体 id 重映射到回放世界。

static void b2RecDispatch_CreateDistanceJoint( const b2RecArgs_CreateDistanceJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2DistanceJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateDistanceJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreateFilterJoint( const b2RecArgs_CreateFilterJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2FilterJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateFilterJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreateMotorJoint( const b2RecArgs_CreateMotorJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2MotorJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateMotorJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreateMoverJoint( const b2RecArgs_CreateMoverJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2MoverJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateMoverJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreatePrismaticJoint( const b2RecArgs_CreatePrismaticJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2PrismaticJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreatePrismaticJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreatePogoJoint( const b2RecArgs_CreatePogoJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2PogoJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreatePogoJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreateRevoluteJoint( const b2RecArgs_CreateRevoluteJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2RevoluteJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateRevoluteJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreateWeldJoint( const b2RecArgs_CreateWeldJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2WeldJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateWeldJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_CreateWheelJoint( const b2RecArgs_CreateWheelJoint* a, b2RecReader* rdr )
{
	b2JointId recId = b2RecR_JOINTID( rdr );
	b2WheelJointDef def = a->def;
	def.base.bodyIdA = b2RecMakeBodyId( rdr, def.base.bodyIdA );
	def.base.bodyIdB = b2RecMakeBodyId( rdr, def.base.bodyIdB );
	b2RecCheckJointId( rdr, b2CreateWheelJoint( rdr->replayWorldId, &def ), recId );
}

static void b2RecDispatch_DestroyJoint( const b2RecArgs_DestroyJoint* a, b2RecReader* rdr )
{
	b2DestroyJoint( b2RecMakeJointId( rdr, a->joint ) );
}

// 通用关节修改器

static void b2RecDispatch_JointSetLocalFrameA( const b2RecArgs_JointSetLocalFrameA* a, b2RecReader* rdr )
{
	b2Joint_SetLocalFrameA( b2RecMakeJointId( rdr, a->joint ), a->localFrame );
}

static void b2RecDispatch_JointSetLocalFrameB( const b2RecArgs_JointSetLocalFrameB* a, b2RecReader* rdr )
{
	b2Joint_SetLocalFrameB( b2RecMakeJointId( rdr, a->joint ), a->localFrame );
}

static void b2RecDispatch_JointSetCollideConnected( const b2RecArgs_JointSetCollideConnected* a, b2RecReader* rdr )
{
	b2Joint_SetCollideConnected( b2RecMakeJointId( rdr, a->joint ), a->shouldCollide );
}

static void b2RecDispatch_JointWakeBodies( const b2RecArgs_JointWakeBodies* a, b2RecReader* rdr )
{
	b2Joint_WakeBodies( b2RecMakeJointId( rdr, a->joint ) );
}

static void b2RecDispatch_JointSetConstraintTuning( const b2RecArgs_JointSetConstraintTuning* a, b2RecReader* rdr )
{
	b2Joint_SetConstraintTuning( b2RecMakeJointId( rdr, a->joint ), a->hertz, a->dampingRatio );
}

static void b2RecDispatch_JointSetForceThreshold( const b2RecArgs_JointSetForceThreshold* a, b2RecReader* rdr )
{
	b2Joint_SetForceThreshold( b2RecMakeJointId( rdr, a->joint ), a->threshold );
}

static void b2RecDispatch_JointSetTorqueThreshold( const b2RecArgs_JointSetTorqueThreshold* a, b2RecReader* rdr )
{
	b2Joint_SetTorqueThreshold( b2RecMakeJointId( rdr, a->joint ), a->threshold );
}

// 距离关节

static void b2RecDispatch_DistanceJointSetRestLength( const b2RecArgs_DistanceJointSetRestLength* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetRestLength( b2RecMakeJointId( rdr, a->joint ), a->length );
}

static void b2RecDispatch_DistanceJointEnableSpring( const b2RecArgs_DistanceJointEnableSpring* a, b2RecReader* rdr )
{
	b2DistanceJoint_EnableSpring( b2RecMakeJointId( rdr, a->joint ), a->enableSpring );
}

static void b2RecDispatch_DistanceJointSetSpringForceRange( const b2RecArgs_DistanceJointSetSpringForceRange* a,
															b2RecReader* rdr )
{
	b2DistanceJoint_SetSpringForceRange( b2RecMakeJointId( rdr, a->joint ), a->lowerForce, a->upperForce );
}

static void b2RecDispatch_DistanceJointSetSpringHertz( const b2RecArgs_DistanceJointSetSpringHertz* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetSpringHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_DistanceJointSetSpringDampingRatio( const b2RecArgs_DistanceJointSetSpringDampingRatio* a,
															  b2RecReader* rdr )
{
	b2DistanceJoint_SetSpringDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b2RecDispatch_DistanceJointEnableLimit( const b2RecArgs_DistanceJointEnableLimit* a, b2RecReader* rdr )
{
	b2DistanceJoint_EnableLimit( b2RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b2RecDispatch_DistanceJointSetLimits( const b2RecArgs_DistanceJointSetLimits* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetLimits( b2RecMakeJointId( rdr, a->joint ), a->minLength, a->maxLength );
}

static void b2RecDispatch_DistanceJointEnableMotor( const b2RecArgs_DistanceJointEnableMotor* a, b2RecReader* rdr )
{
	b2DistanceJoint_EnableMotor( b2RecMakeJointId( rdr, a->joint ), a->enableMotor );
}

static void b2RecDispatch_DistanceJointSetTargetMotorSpeed( const b2RecArgs_DistanceJointSetTargetMotorSpeed* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetTargetMotorSpeed( b2RecMakeJointId( rdr, a->joint ), a->motorSpeed );
}

static void b2RecDispatch_DistanceJointSetMaxMotorForce( const b2RecArgs_DistanceJointSetMaxMotorForce* a, b2RecReader* rdr )
{
	b2DistanceJoint_SetMaxMotorForce( b2RecMakeJointId( rdr, a->joint ), a->force );
}

// 马达关节

static void b2RecDispatch_MotorJointSetLinearVelocity( const b2RecArgs_MotorJointSetLinearVelocity* a, b2RecReader* rdr )
{
	b2MotorJoint_SetLinearVelocity( b2RecMakeJointId( rdr, a->joint ), a->velocity );
}

static void b2RecDispatch_MotorJointSetAngularVelocity( const b2RecArgs_MotorJointSetAngularVelocity* a, b2RecReader* rdr )
{
	b2MotorJoint_SetAngularVelocity( b2RecMakeJointId( rdr, a->joint ), a->velocity );
}

static void b2RecDispatch_MotorJointSetMaxVelocityForce( const b2RecArgs_MotorJointSetMaxVelocityForce* a, b2RecReader* rdr )
{
	b2MotorJoint_SetMaxVelocityForce( b2RecMakeJointId( rdr, a->joint ), a->maxForce );
}

static void b2RecDispatch_MotorJointSetMaxVelocityTorque( const b2RecArgs_MotorJointSetMaxVelocityTorque* a, b2RecReader* rdr )
{
	b2MotorJoint_SetMaxVelocityTorque( b2RecMakeJointId( rdr, a->joint ), a->maxTorque );
}

static void b2RecDispatch_MotorJointSetLinearHertz( const b2RecArgs_MotorJointSetLinearHertz* a, b2RecReader* rdr )
{
	b2MotorJoint_SetLinearHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_MotorJointSetLinearDampingRatio( const b2RecArgs_MotorJointSetLinearDampingRatio* a, b2RecReader* rdr )
{
	b2MotorJoint_SetLinearDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->damping );
}

static void b2RecDispatch_MotorJointSetAngularHertz( const b2RecArgs_MotorJointSetAngularHertz* a, b2RecReader* rdr )
{
	b2MotorJoint_SetAngularHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_MotorJointSetAngularDampingRatio( const b2RecArgs_MotorJointSetAngularDampingRatio* a,
															b2RecReader* rdr )
{
	b2MotorJoint_SetAngularDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->damping );
}

static void b2RecDispatch_MotorJointSetMaxSpringForce( const b2RecArgs_MotorJointSetMaxSpringForce* a, b2RecReader* rdr )
{
	b2MotorJoint_SetMaxSpringForce( b2RecMakeJointId( rdr, a->joint ), a->maxForce );
}

static void b2RecDispatch_MotorJointSetMaxSpringTorque( const b2RecArgs_MotorJointSetMaxSpringTorque* a, b2RecReader* rdr )
{
	b2MotorJoint_SetMaxSpringTorque( b2RecMakeJointId( rdr, a->joint ), a->maxTorque );
}

// 移动关节
static void b2RecDispatch_MoverJointSetLinearVelocity( const b2RecArgs_MoverJointSetLinearVelocity* a, b2RecReader* rdr )
{
	b2MoverJoint_SetLinearVelocity( b2RecMakeJointId( rdr, a->joint ), a->velocity );
}

static void b2RecDispatch_MoverJointSetMaxVelocityForce( const b2RecArgs_MoverJointSetMaxVelocityForce* a, b2RecReader* rdr )
{
	b2MoverJoint_SetMaxVelocityForce( b2RecMakeJointId( rdr, a->joint ), a->maxForce );
}

// 弹簧杆关节

static void b2RecDispatch_PogoJointSetRestLength( const b2RecArgs_PogoJointSetRestLength* a, b2RecReader* rdr )
{
	b2PogoJoint_SetRestLength( b2RecMakeJointId( rdr, a->joint ), a->length );
}

static void b2RecDispatch_PogoJointSetSpringHertz( const b2RecArgs_PogoJointSetSpringHertz* a, b2RecReader* rdr )
{
	b2PogoJoint_SetSpringHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_PogoJointSetSpringDampingRatio( const b2RecArgs_PogoJointSetSpringDampingRatio* a, b2RecReader* rdr )
{
	b2PogoJoint_SetSpringDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

// 移动关节

static void b2RecDispatch_PrismaticJointEnableSpring( const b2RecArgs_PrismaticJointEnableSpring* a, b2RecReader* rdr )
{
	b2PrismaticJoint_EnableSpring( b2RecMakeJointId( rdr, a->joint ), a->enableSpring );
}

static void b2RecDispatch_PrismaticJointSetSpringHertz( const b2RecArgs_PrismaticJointSetSpringHertz* a, b2RecReader* rdr )
{
	b2PrismaticJoint_SetSpringHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_PrismaticJointSetSpringDampingRatio( const b2RecArgs_PrismaticJointSetSpringDampingRatio* a,
															   b2RecReader* rdr )
{
	b2PrismaticJoint_SetSpringDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b2RecDispatch_PrismaticJointSetTargetTranslation( const b2RecArgs_PrismaticJointSetTargetTranslation* a,
															  b2RecReader* rdr )
{
	b2PrismaticJoint_SetTargetTranslation( b2RecMakeJointId( rdr, a->joint ), a->translation );
}

static void b2RecDispatch_PrismaticJointEnableLimit( const b2RecArgs_PrismaticJointEnableLimit* a, b2RecReader* rdr )
{
	b2PrismaticJoint_EnableLimit( b2RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b2RecDispatch_PrismaticJointSetLimits( const b2RecArgs_PrismaticJointSetLimits* a, b2RecReader* rdr )
{
	b2PrismaticJoint_SetLimits( b2RecMakeJointId( rdr, a->joint ), a->lower, a->upper );
}

static void b2RecDispatch_PrismaticJointEnableMotor( const b2RecArgs_PrismaticJointEnableMotor* a, b2RecReader* rdr )
{
	b2PrismaticJoint_EnableMotor( b2RecMakeJointId( rdr, a->joint ), a->enableMotor );
}

static void b2RecDispatch_PrismaticJointSetTargetMotorSpeed( const b2RecArgs_PrismaticJointSetTargetMotorSpeed* a, b2RecReader* rdr )
{
	b2PrismaticJoint_SetTargetMotorSpeed( b2RecMakeJointId( rdr, a->joint ), a->motorSpeed );
}

static void b2RecDispatch_PrismaticJointSetMaxMotorForce( const b2RecArgs_PrismaticJointSetMaxMotorForce* a, b2RecReader* rdr )
{
	b2PrismaticJoint_SetMaxMotorForce( b2RecMakeJointId( rdr, a->joint ), a->force );
}

// 旋转关节

static void b2RecDispatch_RevoluteJointEnableSpring( const b2RecArgs_RevoluteJointEnableSpring* a, b2RecReader* rdr )
{
	b2RevoluteJoint_EnableSpring( b2RecMakeJointId( rdr, a->joint ), a->enableSpring );
}

static void b2RecDispatch_RevoluteJointSetSpringHertz( const b2RecArgs_RevoluteJointSetSpringHertz* a, b2RecReader* rdr )
{
	b2RevoluteJoint_SetSpringHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_RevoluteJointSetSpringDampingRatio( const b2RecArgs_RevoluteJointSetSpringDampingRatio* a,
															  b2RecReader* rdr )
{
	b2RevoluteJoint_SetSpringDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b2RecDispatch_RevoluteJointSetTargetAngle( const b2RecArgs_RevoluteJointSetTargetAngle* a, b2RecReader* rdr )
{
	b2RevoluteJoint_SetTargetAngle( b2RecMakeJointId( rdr, a->joint ), a->angle );
}

static void b2RecDispatch_RevoluteJointEnableLimit( const b2RecArgs_RevoluteJointEnableLimit* a, b2RecReader* rdr )
{
	b2RevoluteJoint_EnableLimit( b2RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b2RecDispatch_RevoluteJointSetLimits( const b2RecArgs_RevoluteJointSetLimits* a, b2RecReader* rdr )
{
	b2RevoluteJoint_SetLimits( b2RecMakeJointId( rdr, a->joint ), a->lower, a->upper );
}

static void b2RecDispatch_RevoluteJointEnableMotor( const b2RecArgs_RevoluteJointEnableMotor* a, b2RecReader* rdr )
{
	b2RevoluteJoint_EnableMotor( b2RecMakeJointId( rdr, a->joint ), a->enableMotor );
}

static void b2RecDispatch_RevoluteJointSetTargetMotorSpeed( const b2RecArgs_RevoluteJointSetTargetMotorSpeed* a, b2RecReader* rdr )
{
	b2RevoluteJoint_SetTargetMotorSpeed( b2RecMakeJointId( rdr, a->joint ), a->motorSpeed );
}

static void b2RecDispatch_RevoluteJointSetMaxMotorTorque( const b2RecArgs_RevoluteJointSetMaxMotorTorque* a, b2RecReader* rdr )
{
	b2RevoluteJoint_SetMaxMotorTorque( b2RecMakeJointId( rdr, a->joint ), a->torque );
}

// 焊接关节

static void b2RecDispatch_WeldJointSetLinearHertz( const b2RecArgs_WeldJointSetLinearHertz* a, b2RecReader* rdr )
{
	b2WeldJoint_SetLinearHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_WeldJointSetLinearDampingRatio( const b2RecArgs_WeldJointSetLinearDampingRatio* a, b2RecReader* rdr )
{
	b2WeldJoint_SetLinearDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b2RecDispatch_WeldJointSetAngularHertz( const b2RecArgs_WeldJointSetAngularHertz* a, b2RecReader* rdr )
{
	b2WeldJoint_SetAngularHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_WeldJointSetAngularDampingRatio( const b2RecArgs_WeldJointSetAngularDampingRatio* a, b2RecReader* rdr )
{
	b2WeldJoint_SetAngularDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

// 轮关节

static void b2RecDispatch_WheelJointEnableSpring( const b2RecArgs_WheelJointEnableSpring* a, b2RecReader* rdr )
{
	b2WheelJoint_EnableSpring( b2RecMakeJointId( rdr, a->joint ), a->enableSpring );
}

static void b2RecDispatch_WheelJointSetSpringHertz( const b2RecArgs_WheelJointSetSpringHertz* a, b2RecReader* rdr )
{
	b2WheelJoint_SetSpringHertz( b2RecMakeJointId( rdr, a->joint ), a->hertz );
}

static void b2RecDispatch_WheelJointSetSpringDampingRatio( const b2RecArgs_WheelJointSetSpringDampingRatio* a, b2RecReader* rdr )
{
	b2WheelJoint_SetSpringDampingRatio( b2RecMakeJointId( rdr, a->joint ), a->dampingRatio );
}

static void b2RecDispatch_WheelJointEnableLimit( const b2RecArgs_WheelJointEnableLimit* a, b2RecReader* rdr )
{
	b2WheelJoint_EnableLimit( b2RecMakeJointId( rdr, a->joint ), a->enableLimit );
}

static void b2RecDispatch_WheelJointSetLimits( const b2RecArgs_WheelJointSetLimits* a, b2RecReader* rdr )
{
	b2WheelJoint_SetLimits( b2RecMakeJointId( rdr, a->joint ), a->lower, a->upper );
}

static void b2RecDispatch_WheelJointEnableMotor( const b2RecArgs_WheelJointEnableMotor* a, b2RecReader* rdr )
{
	b2WheelJoint_EnableMotor( b2RecMakeJointId( rdr, a->joint ), a->enableMotor );
}

static void b2RecDispatch_WheelJointSetTargetMotorSpeed( const b2RecArgs_WheelJointSetTargetMotorSpeed* a, b2RecReader* rdr )
{
	b2WheelJoint_SetTargetMotorSpeed( b2RecMakeJointId( rdr, a->joint ), a->motorSpeed );
}

static void b2RecDispatch_WheelJointSetMaxMotorTorque( const b2RecArgs_WheelJointSetMaxMotorTorque* a, b2RecReader* rdr )
{
	b2WheelJoint_SetMaxMotorTorque( b2RecMakeJointId( rdr, a->joint ), a->torque );
}

// 浮点位比较器：比较原始位，以便一致处理 NaN != NaN

static bool b2RecF32Differs( float a, float b )
{
	uint32_t ua, ub;
	memcpy( &ua, &a, 4 );
	memcpy( &ub, &b, 4 );
	return ua != ub;
}

static bool b2RecVec2Differs( b2Vec2 a, b2Vec2 b )
{
	return b2RecF32Differs( a.x, b.x ) || b2RecF32Differs( a.y, b.y );
}

// 重叠 AABB 分派器

// 查询回放跳板的共享上下文：按序遍历已录制命中，并标记与重新发出查询的任何分叉
typedef struct b2RecReplayQueryCtx
{
	b2RecReader* rdr;
	const b2RecRecordedHit* hits;
	int count;
	int cursor;
} b2RecReplayQueryCtx;

static bool b2RecReplayOverlapTrampoline( b2ShapeId id, void* ctx )
{
	b2RecReplayQueryCtx* rc = static_cast<b2RecReplayQueryCtx *>( ctx );
	if ( rc->cursor >= rc->count )
	{
		rc->rdr->diverged = true;
		return false;
	}
	const b2RecRecordedHit* h = &rc->hits[rc->cursor++];
	if ( id.index1 != h->id.index1 || id.generation != h->id.generation )
	{
		rc->rdr->diverged = true;
	}
	return h->userReturnB;
}

static void b2RecGrowFrameQueries( b2Replay* player )
{
	b2RecGrow( (void**)&player->frameQueries, &player->frameQueryCap, player->frameQueryCount + 1, player->frameQueryCount,
			   (int)sizeof( b2RecDrawQuery ) );
}

static void b2RecGrowFrameHits( b2Replay* player, int need )
{
	b2RecGrow( (void**)&player->frameHits, &player->frameHitCap, player->frameHitCount + need, player->frameHitCount,
			   (int)sizeof( b2RecRecordedHit ) );
}

static b2RecDrawQuery* b2RecStashQueryBegin( b2Replay* player, int kind, const b2RecRecordedHit* hits, int hitCount )
{
	b2RecGrowFrameQueries( player );
	b2RecDrawQuery* q = &player->frameQueries[player->frameQueryCount];
	memset( q, 0, sizeof( *q ) );
	q->kind = kind;
	q->hitStart = player->frameHitCount;
	q->hitCount = hitCount;
	b2RecGrowFrameHits( player, hitCount );
	for ( int i = 0; i < hitCount; ++i )
	{
		player->frameHits[player->frameHitCount + i] = hits[i];
	}
	player->frameHitCount += hitCount;
	player->frameQueryCount++;
	return q;
}

static void b2RecDispatch_QueryOverlapAABB( const b2RecArgs_QueryOverlapAABB* a, b2RecReader* rdr )
{
	uint32_t n = b2RecR_U32( rdr );
	b2RecEnsureHits( rdr, (int)n );
	if ( !rdr->ok )
	{
		return;
	}
	for ( uint32_t i = 0; i < n; ++i )
	{
		rdr->hits[i].id = b2RecMakeShapeId( rdr, b2RecR_SHAPEID( rdr ) );
		rdr->hits[i].userReturnB = b2RecR_BOOL( rdr );
	}
	(void)b2RecR_TREESTATS( rdr );
	if ( !rdr->ok )
		return;
	b2RecReplayQueryCtx rc = { rdr, rdr->hits, (int)n, 0 };
	b2World_OverlapAABB( rdr->replayWorldId, a->origin, a->aabb, a->filter, b2RecReplayOverlapTrampoline, &rc );
	if ( rc.cursor != (int)n )
		rdr->diverged = true;

	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_OVERLAP_AABB, rdr->hits, (int)n );
		q->filter = a->filter;
		q->origin = a->origin;
		q->aabb = a->aabb;
	}
}

static void b2RecDispatch_QueryOverlapShape( const b2RecArgs_QueryOverlapShape* a, b2RecReader* rdr )
{
	uint32_t n = b2RecR_U32( rdr );
	b2RecEnsureHits( rdr, (int)n );
	if ( !rdr->ok )
	{
		return;
	}
	for ( uint32_t i = 0; i < n; ++i )
	{
		rdr->hits[i].id = b2RecMakeShapeId( rdr, b2RecR_SHAPEID( rdr ) );
		rdr->hits[i].userReturnB = b2RecR_BOOL( rdr );
	}
	(void)b2RecR_TREESTATS( rdr );
	if ( !rdr->ok )
		return;
	b2RecReplayQueryCtx rc = { rdr, rdr->hits, (int)n, 0 };
	b2World_OverlapShape( rdr->replayWorldId, a->origin, &a->proxy, a->filter, b2RecReplayOverlapTrampoline, &rc );
	if ( rc.cursor != (int)n )
		rdr->diverged = true;

	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_OVERLAP_SHAPE, rdr->hits, (int)n );
		q->filter = a->filter;
		q->origin = a->origin;
		q->proxy = a->proxy;
	}
}

// 射线检测分派器

static float b2RecReplayCastTrampoline( b2ShapeId id, b2Pos point, b2Vec2 normal, float fraction, void* ctx )
{
	b2RecReplayQueryCtx* rc = static_cast<b2RecReplayQueryCtx *>( ctx );
	if ( rc->cursor >= rc->count )
	{
		rc->rdr->diverged = true;
		return 0.0f;
	}
	const b2RecRecordedHit* h = &rc->hits[rc->cursor++];
	// 通过全宽度差值比较位置，两侧都截断会在远离原点时空通过
	if ( id.index1 != h->id.index1 || id.generation != h->id.generation ||
		 b2RecVec2Differs( b2SubPos( point, h->point ), b2Vec2_zero ) || b2RecVec2Differs( normal, h->normal ) ||
		 b2RecF32Differs( fraction, h->fraction ) )
	{
		rc->rdr->diverged = true;
	}
	return h->userReturnF;
}

static void b2RecDispatch_QueryCastRay( const b2RecArgs_QueryCastRay* a, b2RecReader* rdr )
{
	uint32_t n = b2RecR_U32( rdr );
	b2RecEnsureHits( rdr, (int)n );
	if ( !rdr->ok )
	{
		return;
	}
	for ( uint32_t i = 0; i < n; ++i )
	{
		rdr->hits[i].id = b2RecMakeShapeId( rdr, b2RecR_SHAPEID( rdr ) );
		rdr->hits[i].point = b2RecR_POSITION( rdr );
		rdr->hits[i].normal = b2RecR_VEC2( rdr );
		rdr->hits[i].fraction = b2RecR_F32( rdr );
		rdr->hits[i].userReturnF = b2RecR_F32( rdr );
	}
	(void)b2RecR_TREESTATS( rdr );
	if ( !rdr->ok )
		return;
	b2RecReplayQueryCtx rc = { rdr, rdr->hits, (int)n, 0 };
	b2World_CastRay( rdr->replayWorldId, a->origin, a->translation, a->filter, b2RecReplayCastTrampoline, &rc );
	if ( rc.cursor != (int)n )
		rdr->diverged = true;

	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_CAST_RAY, rdr->hits, (int)n );
		q->filter = a->filter;
		q->origin = a->origin;
		q->translation = a->translation;
	}
}

static void b2RecDispatch_QueryCastShape( const b2RecArgs_QueryCastShape* a, b2RecReader* rdr )
{
	uint32_t n = b2RecR_U32( rdr );
	b2RecEnsureHits( rdr, (int)n );
	if ( !rdr->ok )
	{
		return;
	}
	for ( uint32_t i = 0; i < n; ++i )
	{
		rdr->hits[i].id = b2RecMakeShapeId( rdr, b2RecR_SHAPEID( rdr ) );
		rdr->hits[i].point = b2RecR_POSITION( rdr );
		rdr->hits[i].normal = b2RecR_VEC2( rdr );
		rdr->hits[i].fraction = b2RecR_F32( rdr );
		rdr->hits[i].userReturnF = b2RecR_F32( rdr );
	}
	(void)b2RecR_TREESTATS( rdr );
	if ( !rdr->ok )
		return;
	b2RecReplayQueryCtx rc = { rdr, rdr->hits, (int)n, 0 };
	b2World_CastShape( rdr->replayWorldId, a->origin, &a->proxy, a->translation, a->filter, b2RecReplayCastTrampoline, &rc );
	if ( rc.cursor != (int)n )
		rdr->diverged = true;

	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_CAST_SHAPE, rdr->hits, (int)n );
		q->filter = a->filter;
		q->origin = a->origin;
		q->proxy = a->proxy;
		q->translation = a->translation;
	}
}

// CollideMover 分派器

static bool b2RecReplayPlaneTrampoline( b2ShapeId id, const b2PlaneResult* plane, void* ctx )
{
	b2RecReplayQueryCtx* rc = static_cast<b2RecReplayQueryCtx *>( ctx );
	if ( rc->cursor >= rc->count )
	{
		rc->rdr->diverged = true;
		return true;
	}
	const b2RecRecordedHit* h = &rc->hits[rc->cursor++];
	if ( id.index1 != h->id.index1 || id.generation != h->id.generation ||
		 b2RecVec2Differs( plane->plane.normal, h->plane.plane.normal ) ||
		 b2RecF32Differs( plane->plane.offset, h->plane.plane.offset ) || b2RecVec2Differs( plane->point, h->plane.point ) ||
		 plane->hit != h->plane.hit )
	{
		rc->rdr->diverged = true;
	}
	return h->userReturnB;
}

static void b2RecDispatch_QueryCollideMover( const b2RecArgs_QueryCollideMover* a, b2RecReader* rdr )
{
	uint32_t n = b2RecR_U32( rdr );
	b2RecEnsureHits( rdr, (int)n );
	if ( !rdr->ok )
	{
		return;
	}
	for ( uint32_t i = 0; i < n; ++i )
	{
		rdr->hits[i].id = b2RecMakeShapeId( rdr, b2RecR_SHAPEID( rdr ) );
		rdr->hits[i].plane = b2RecR_PLANERESULT( rdr );
		rdr->hits[i].userReturnB = b2RecR_BOOL( rdr );
	}
	// CollideMover 没有 TREESTATS 尾部（返回 void）
	if ( !rdr->ok )
		return;
	b2RecReplayQueryCtx rc = { rdr, rdr->hits, (int)n, 0 };
	b2World_CollideMover( rdr->replayWorldId, a->origin, &a->mover, a->filter, b2RecReplayPlaneTrampoline, &rc );
	if ( rc.cursor != (int)n )
		rdr->diverged = true;

	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_COLLIDE_MOVER, rdr->hits, (int)n );
		q->filter = a->filter;
		q->origin = a->origin;
		q->mover = a->mover;
	}
}

// CastRayClosest 分派器

static void b2RecDispatch_QueryCastRayClosest( const b2RecArgs_QueryCastRayClosest* a, b2RecReader* rdr )
{
	b2RayResult rec = b2RecR_RAYRESULT( rdr );
	if ( !rdr->ok )
		return;
	b2RayResult got = b2World_CastRayClosest( rdr->replayWorldId, a->origin, a->translation, a->filter );
	if ( got.hit != rec.hit ||
		 ( got.hit && ( got.shapeId.index1 != rec.shapeId.index1 || got.shapeId.generation != rec.shapeId.generation ||
						b2RecVec2Differs( b2SubPos( got.point, rec.point ), b2Vec2_zero ) ||
						b2RecVec2Differs( got.normal, rec.normal ) || b2RecF32Differs( got.fraction, rec.fraction ) ) ) )
	{
		rdr->diverged = true;
	}

	if ( rdr->owner )
	{
		// 将最近结果暂存为单个池化命中，以便共享绘制循环渲染其点
		b2RecRecordedHit h = {};
		h.id = b2RecMakeShapeId( rdr, rec.shapeId );
		h.point = rec.point;
		h.normal = rec.normal;
		h.fraction = rec.fraction;
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_CAST_RAY_CLOSEST, &h, rec.hit ? 1 : 0 );
		q->filter = a->filter;
		q->origin = a->origin;
		q->translation = a->translation;
	}
}

// CastMover 分派器

static void b2RecDispatch_QueryCastMover( const b2RecArgs_QueryCastMover* a, b2RecReader* rdr )
{
	float rec = b2RecR_F32( rdr );
	if ( !rdr->ok )
		return;
	float got = b2World_CastMover( rdr->replayWorldId, a->origin, &a->mover, a->translation, a->filter );
	if ( b2RecF32Differs( got, rec ) )
		rdr->diverged = true;

	if ( rdr->owner )
	{
		b2RecDrawQuery* q = b2RecStashQueryBegin( rdr->owner, B2_RECQ_CAST_MOVER, NULL, 0 );
		q->filter = a->filter;
		q->origin = a->origin;
		q->mover = a->mover;
		q->translation = a->translation;
		q->castFraction = rec;
	}
}

static void b2RecDispatch_StateHash( const b2RecArgs_StateHash* a, b2RecReader* rdr )
{
	b2World* world = ( b2_worlds + rdr->replayWorldId.index1 - 1 );
	uint64_t computed = b2HashWorldState( world );
	if ( computed != a->hash )
	{
		printf( "b2ReplayFile: StateHash mismatch (recorded=0x%llX, computed=0x%llX)\n", (unsigned long long)a->hash,
				(unsigned long long)computed );
		// 非致命：继续读取，以便查看器能显示分叉开始位置
		rdr->diverged = true;
	}
}

static void b2RecDispatch_RecordingBounds( const b2RecArgs_RecordingBounds* a, b2RecReader* rdr )
{
	// 主要解析是打开时扫描，若该值提前出现也能保持正确
	rdr->owner->bounds = a->bounds;
}

// 代码生成第 2 遍构建读取并分派的 switch 分支。每个分支把 ARG 字段读入
// b2RecArgs_<Name> 再分派。创建操作在其分派器中读取返回的 id。
// 返回刚分派的操作码，文件结束或致命读取错误时返回 -1。

static int b2RecDispatchOne( b2Replay* player )
{
	b2RecReader* rdr = &player->rdr;
	if ( rdr->cursor >= rdr->size || !rdr->ok )
	{
		return -1;
	}

	uint8_t opcode = b2RecR_U8( rdr );
	if ( !rdr->ok )
	{
		return -1;
	}
	uint32_t payloadSize = b2RecR_U24( rdr );
	if ( !rdr->ok )
	{
		return -1;
	}
	int payloadStart = rdr->cursor;

	switch ( opcode )
	{
#define ARG( TAG, field ) a.field = b2RecR_##TAG( rdr );
#define B2_REC_OP( op, Name, RET, ... )                                                                                          \
	case op:                                                                                                                     \
	{                                                                                                                            \
		b2RecArgs_##Name a;                                                                                                      \
		memset( &a, 0, sizeof( a ) );                                                                                            \
		__VA_ARGS__                                                                                                              \
		if ( rdr->ok )                                                                                                           \
		{                                                                                                                        \
			b2RecDispatch_##Name( &a, rdr );                                                                                     \
		}                                                                                                                        \
		break;                                                                                                                   \
	}
#include "recording_ops.inl"
#undef B2_REC_OP
#undef ARG
		default:
			printf( "b2ReplayFile: unknown opcode 0x%02X, skipping %u bytes\n", opcode, payloadSize );
			// payloadStart 在范围内，因此 size - payloadStart 是剩余待跳过的字节
			if ( payloadSize > (uint32_t)( rdr->size - payloadStart ) )
			{
				rdr->ok = false;
			}
			else
			{
				rdr->cursor = payloadStart + (int)payloadSize;
			}
			break;
	}

	return (int)opcode;
}

// 不经分派地遍历记录一次，以计步并读取第一步的调参。
// 分帧为 opcode u8 + payload u24 + payload，因此可盲跳记录。
static void b2RecScanFile( b2Replay* player )
{
	const uint8_t* data = player->data;
	int size = player->size;
	int cursor = player->headerEnd;
	int frameCount = 0;
	bool gotStep = false;

	while ( cursor + 4 <= size )
	{
		uint8_t opcode = data[cursor];
		uint32_t payloadSize =
			(uint32_t)data[cursor + 1] | ( (uint32_t)data[cursor + 2] << 8 ) | ( (uint32_t)data[cursor + 3] << 16 );
		int payloadStart = cursor + 4;
		if ( payloadStart + (int)payloadSize > size )
		{
			break;
		}

		if ( opcode == 0x80 ) // Step: [u32 world][f32 dt][i32 subStepCount]
		{
			frameCount += 1;
			if ( gotStep == false && payloadSize >= 12 )
			{
				uint32_t dtBits = (uint32_t)data[payloadStart + 4] | ( (uint32_t)data[payloadStart + 5] << 8 ) |
								  ( (uint32_t)data[payloadStart + 6] << 16 ) | ( (uint32_t)data[payloadStart + 7] << 24 );
				memcpy( &player->recordedDt, &dtBits, 4 );
				player->recordedSubStepCount =
					(int)( (uint32_t)data[payloadStart + 8] | ( (uint32_t)data[payloadStart + 9] << 8 ) |
						   ( (uint32_t)data[payloadStart + 10] << 16 ) | ( (uint32_t)data[payloadStart + 11] << 24 ) );
				gotStep = true;
			}
		}
		else if ( opcode == 0xF2 && payloadSize >= 16 ) // RecordingBounds: [f32 lo.x][lo.y][hi.x][hi.y]
		{
			// 小端 float 与 b2AABB 布局匹配，头已拒绝大端文件
			memcpy( &player->bounds, data + payloadStart, 16 );
		}

		cursor = payloadStart + (int)payloadSize;
	}

	player->frameCount = frameCount;
}


b2Replay* b2CreateReplay( const void* data, int size, int workerCount )
{
	if ( data == NULL || size < 32 )
	{
		printf( "b2CreateReplay: recording too small\n" );
		return NULL;
	}

	// 拷贝任何内容前先校验头
	b2RecHeader hdr;
	memcpy( &hdr, data, sizeof( b2RecHeader ) );

	if ( hdr.magic != B2_REC_MAGIC )
	{
		printf( "b2CreateReplay: bad magic (got 0x%08X)\n", hdr.magic );
		return NULL;
	}

	if ( hdr.versionMajor != B2_REC_VERSION_MAJOR || hdr.versionMinor != B2_REC_VERSION_MINOR )
	{
		printf( "b2CreateReplay: version mismatch (file=%u.%u, runtime=%u.%u)\n", hdr.versionMajor, hdr.versionMinor,
				B2_REC_VERSION_MAJOR, B2_REC_VERSION_MINOR );
		return NULL;
	}

	if ( hdr.pointerWidth != (uint8_t)sizeof( void* ) )
	{
		printf( "b2CreateReplay: pointer width mismatch (file=%u, runtime=%u)\n", hdr.pointerWidth,
				(unsigned)sizeof( void* ) );
		return NULL;
	}

	if ( hdr.bigEndian != 0 )
	{
		printf( "b2CreateReplay: big-endian recording not supported\n" );
		return NULL;
	}

	// 每段录制都以快照播种：数据块位于头与操作流之间
	if ( hdr.snapshotSize == 0 || hdr.snapshotSize > (uint64_t)( size - 32 ) )
	{
		printf( "b2CreateReplay: missing or oversized snapshot\n" );
		return NULL;
	}
	int headerEnd = 32 + (int)hdr.snapshotSize;

	// 持有字节的私有副本，以便调用者可在此调用后立即释放其缓冲区
	uint8_t* copy = static_cast<uint8_t *>( b2Alloc( size ) );
	memcpy( copy, data, (size_t)size );

	b2Replay* player = static_cast<b2Replay *>( b2Alloc( (int)sizeof( b2Replay ) ) );
	player->data = copy;
	player->size = size;
	player->headerEnd = headerEnd;
	player->lengthScale = hdr.lengthScale;
	player->previousLengthScale = b2GetLengthUnitsPerMeter();
	player->frame = 0;
	player->frameCount = 0;
	player->recordedWorkerCount = 0;
	player->recordedDt = 0.0f;
	player->recordedSubStepCount = 0;
	player->bounds = b2AABB{};
	player->divergeFrame = -1;
	player->atEnd = false;
	player->rdr.data = copy;
	player->rdr.size = size;
	player->rdr.cursor = headerEnd; // 越过头部与快照 blob
	player->rdr.replayWorldId = b2_nullWorldId;
	player->rdr.workerCount = workerCount;
	player->rdr.ok = true;
	player->rdr.diverged = false;
	player->rdr.chainPoints = NULL;
	player->rdr.chainPointCap = 0;
	player->rdr.chainMaterials = NULL;
	player->rdr.chainMaterialCap = 0;
	player->rdr.hits = NULL;
	player->rdr.hitCap = 0;
	player->rdr.owner = player;
    player->frameQueries = NULL; player->frameQueryCount = 0; player->frameQueryCap = 0;
    player->frameHits = NULL; player->frameHitCount = 0; player->frameHitCap = 0;
    player->bodyIds = NULL; player->bodyIdCount = 0; player->bodyIdCap = 0;
    player->frame0BodyIds = NULL; player->frame0BodyIdCount = 0;
    player->keyframes = NULL; player->keyframeCount = 0; player->keyframeCapacity = 0;
    player->keyframeBudget = 512 * 1024 * 1024; player->keyframeBytes = 0;
    player->keyframeMinInterval = 16; player->keyframeInterval = 16; player->lastKeyframeFrame = 0;
	// 用录制的长度单位覆盖全局长度单位，使回放复现相同常量。
	// 这是全局引擎状态，会影响调用者的其他世界，因此先前值已在上面捕获，并在 b2DestroyReplay 中恢复。
	if ( hdr.lengthScale > 0.0f )
	{
		b2SetLengthUnitsPerMeter( hdr.lengthScale );
	}

	// 计步并读取第一步的调参，以便查看器能预先显示长度和 hz
	b2RecScanFile( player );

	// 反序列化种子快照以建立回放世界。随后的操作流是钩子日志。
	// 该数据块兼作第 0 帧恢复映像，由我们持有的副本拥有。
	player->rdr.replayWorldId = b2CreateWorldFromSnapshot( copy + 32, (int)hdr.snapshotSize, workerCount );
	if ( b2World_IsValid( player->rdr.replayWorldId ) == false )
	{
		printf( "b2CreateReplay: snapshot deserialize failed\n" );
		b2DestroyReplay( player );
		return NULL;
	}
	player->recordedWorkerCount = workerCount;
    player->frame0Image = copy + 32;
	player->frame0Size = (int)hdr.snapshotSize;
	b2RecSeedBodyIds( player );
    player->frame0BodyIdCount = player->bodyIdCount;
    if ( player->bodyIdCount > 0 ) {
        player->frame0BodyIds = (b2BodyId*)b2Alloc( player->bodyIdCount * (int)sizeof( b2BodyId ) );
        memcpy( player->frame0BodyIds, player->bodyIds, player->bodyIdCount * (int)sizeof( b2BodyId ) );
    }

    return player;
}

static void b2FreeKeyframe( b2RecKeyframe* kf )
{
	b2Free( kf->image, kf->imageCapacity );
	if ( kf->bodyIds != NULL )
	{
		b2Free( kf->bodyIds, kf->bodyIdCount * (int)sizeof( b2BodyId ) );
	}
}

static void b2RecCaptureKeyframe( b2Replay* player )
{
	// 序列化到关键帧将取得所有权的缓冲区，因此没有第二次全尺寸分配和拷贝。
	// 缓冲区超额分配，因此预算和释放跟踪其容量而非 size。
	b2World* world = ( b2_worlds + player->rdr.replayWorldId.index1 - 1 );
	b2RecBuffer buf = {};
	b2SerializeWorld( world, &buf );

	size_t bodyBytes = (size_t)player->bodyIdCount * sizeof( b2BodyId );
	size_t newBytes = (size_t)buf.capacity + bodyBytes;

	// 在预算下腾出空间：加倍间距会丢弃非网格关键帧，大约减半字节，直到新关键帧能放下或只剩它。
	// 当单个快照已超出预算时，预算是软限制。
	while ( player->keyframeCount > 0 && player->keyframeBytes + newBytes > player->keyframeBudget )
	{
		player->keyframeInterval *= 2;
		int kept = 0;
		size_t keptBytes = 0;
		for ( int i = 0; i < player->keyframeCount; ++i )
		{
			b2RecKeyframe* kf = &player->keyframes[i];
			if ( kf->frame % player->keyframeInterval == 0 )
			{
				player->keyframes[kept] = *kf;
				keptBytes += (size_t)kf->imageCapacity + (size_t)kf->bodyIdCount * sizeof( b2BodyId );
				kept += 1;
			}
			else
			{
				b2FreeKeyframe( kf );
			}
		}
		bool progress = kept < player->keyframeCount;
		player->keyframeCount = kept;
		player->keyframeBytes = keptBytes;
		if ( progress == false )
		{
			break;
		}
	}

	b2RecGrow( (void**)&player->keyframes, &player->keyframeCapacity, player->keyframeCount + 1, player->keyframeCount,
			   (int)sizeof( b2RecKeyframe ) );

	b2RecKeyframe* kf = &player->keyframes[player->keyframeCount];
	// 把序列化缓冲区交给关键帧，而不是拷贝到精确大小的块
	kf->image = buf.data;
	kf->imageSize = buf.size;
	kf->imageCapacity = buf.capacity;

	kf->frame = player->frame;
	kf->cursor = player->rdr.cursor;
	kf->divergeFrame = player->divergeFrame;
	kf->diverged = player->rdr.diverged;
	kf->bodyIdCount = player->bodyIdCount;
	kf->bodyIds = NULL;
	if ( bodyBytes > 0 )
	{
		kf->bodyIds = static_cast<b2BodyId *>( b2Alloc( bodyBytes ) );
		memcpy( kf->bodyIds, player->bodyIds, (size_t)bodyBytes );
	}

	player->keyframeBytes += newBytes;
	player->keyframeCount += 1;
	player->lastKeyframeFrame = player->frame;
}

bool b2Replay_StepFrame( b2Replay* player )
{
	if ( player->atEnd )
	{
		return false;
	}


	// 运行本帧的 Step，然后消耗其后的记录（StateHash、查询、任何帧间修改器）直到下一步。
	// 一帧的查询和哈希录在其 Step 之后，因此把它们与该 Step 分组，使其与计算时的世界状态配对。
	// 在下一步之前停止，才能精确前进一帧。
	player->frameQueryCount = 0; player->frameHitCount = 0;
    bool stepped = false;
	for ( ;; )
	{
		// 窥视下一操作码而不消耗。下一帧的 Step 结束本帧。
		if ( player->rdr.cursor >= player->rdr.size || player->rdr.ok == false )
		{
			player->atEnd = true;
			return stepped;
		}
		if ( stepped && player->rdr.data[player->rdr.cursor] == 0x80 )
		{
			if ( player->frame > player->lastKeyframeFrame && player->frame % player->keyframeInterval == 0 ) { b2RecCaptureKeyframe( player ); }
    return true;
		}

		int opcode = b2RecDispatchOne( player );
		if ( opcode < 0 )
		{
			player->atEnd = true;
			return stepped;
		}
		if ( opcode == 0x80 ) // Step
		{
			player->frame += 1;
			stepped = true;
		}
		// 锁存首次分叉的帧，供时间线标记使用
		if ( player->divergeFrame < 0 && player->rdr.diverged )
		{
			player->divergeFrame = player->frame;
		}
	}
}

void b2DestroyReplay( b2Replay* player )
{
    if ( player == NULL ) { return; }
    if ( b2World_IsValid( player->rdr.replayWorldId ) ) { b2DestroyWorld( player->rdr.replayWorldId ); }
    b2Free( player->data, player->size );
    b2Free( player->rdr.chainPoints, player->rdr.chainPointCap * (int)sizeof( b2Vec2 ) );
    b2Free( player->rdr.chainMaterials, player->rdr.chainMaterialCap * (int)sizeof( b2SurfaceMaterial ) );
    b2Free( player->rdr.hits, player->rdr.hitCap * (int)sizeof( b2RecRecordedHit ) );
    b2Free( player->bodyIds, player->bodyIdCap * (int)sizeof( b2BodyId ) );
    b2Free( player->frame0BodyIds, player->frame0BodyIdCount * (int)sizeof( b2BodyId ) );
    for ( int i = 0; i < player->keyframeCount; ++i ) { b2FreeKeyframe( player->keyframes + i ); }
    b2Free( player->keyframes, player->keyframeCapacity * (int)sizeof( b2RecKeyframe ) );
    b2Free( player->frameQueries, player->frameQueryCap * (int)sizeof( b2RecDrawQuery ) );
    b2Free( player->frameHits, player->frameHitCap * (int)sizeof( b2RecRecordedHit ) );
    b2SetLengthUnitsPerMeter( player->previousLengthScale );
    b2Free( player, (int)sizeof( b2Replay ) );
}

b2WorldId b2Replay_GetWorldId( const b2Replay* player )
{
	return player != NULL ? player->rdr.replayWorldId : b2_nullWorldId;
}

int b2Replay_GetFrame( const b2Replay* player )
{
	return player != NULL ? player->frame : 0;
}

b2ReplayInfo b2Replay_GetInfo( const b2Replay* player )
{
	b2ReplayInfo info = {};
	if ( player != NULL )
	{
		info.frameCount = player->frameCount;
		info.workerCount = player->recordedWorkerCount;
		info.timeStep = player->recordedDt;
		info.subStepCount = player->recordedSubStepCount;
		info.lengthScale = player->lengthScale;
		info.bounds = player->bounds;
	}
	return info;
}

bool b2Replay_IsAtEnd( const b2Replay* player )
{
	return player != NULL ? player->atEnd : true;
}

void b2Replay_Restart( b2Replay* player )
{
	// 原地恢复第 0 帧映像，使回放世界 id 在重启或向后拖动时保持稳定。
	// 步进从第一个 Step 恢复，该步会确定性重建刚体列表。
	if ( b2World_RestoreFromSnapshot( player->rdr.replayWorldId, player->frame0Image, player->frame0Size ) == false )
	{
		player->rdr.ok = false;
		return;
	}
	// 步进从第一个 Step 恢复，它紧接在头和快照数据块之后
	player->rdr.cursor = player->headerEnd;
	player->rdr.ok = true;
	player->rdr.diverged = false;
	player->frame = 0;
	player->divergeFrame = -1;
	player->atEnd = false;
    player->frameQueryCount = 0; player->frameHitCount = 0;
    player->bodyIdCount = player->frame0BodyIdCount;
    if ( player->frame0BodyIdCount > 0 ) { memcpy( player->bodyIds, player->frame0BodyIds, player->frame0BodyIdCount * (int)sizeof( b2BodyId ) ); }

}

bool b2Replay_HasDiverged( const b2Replay* player )
{
	return player != NULL ? player->rdr.diverged : false;
}

int b2Replay_GetDivergeFrame( const b2Replay* player )
{
	return player != NULL ? player->divergeFrame : -1;
}

void b2Replay_SetKeyframePolicy( b2Replay* player, size_t budgetBytes, int minIntervalFrames )
{
	if ( player == NULL )
	{
		return;
	}
	if ( budgetBytes > 0 )
	{
		player->keyframeBudget = budgetBytes;
	}
	if ( minIntervalFrames > 0 )
	{
		player->keyframeMinInterval = minIntervalFrames;
	}

	// 丢弃环，以便下次回放按新策略重新填充
	for ( int i = 0; i < player->keyframeCount; ++i )
	{
		b2FreeKeyframe( &player->keyframes[i] );
	}
	player->keyframeCount = 0;
	player->keyframeBytes = 0;
	player->keyframeInterval = player->keyframeMinInterval;
	player->lastKeyframeFrame = 0;
}

size_t b2Replay_GetKeyframeBudget( const b2Replay* player )
{
	return player != NULL ? player->keyframeBudget : 0;
}

int b2Replay_GetKeyframeMinInterval( const b2Replay* player )
{
	return player != NULL ? player->keyframeMinInterval : 0;
}

int b2Replay_GetKeyframeInterval( const b2Replay* player )
{
	return player != NULL ? player->keyframeInterval : 0;
}

size_t b2Replay_GetKeyframeBytes( const b2Replay* player )
{
	return player != NULL ? player->keyframeBytes : 0;
}

static void b2ReplayRestoreKeyframe( b2Replay* player, const b2RecKeyframe* kf )
{
	if ( b2World_RestoreFromSnapshot( player->rdr.replayWorldId, kf->image, kf->imageSize ) == false )
	{
		player->rdr.ok = false;
		return;
	}
	player->rdr.cursor = kf->cursor;
	player->rdr.ok = true;
	player->rdr.diverged = kf->diverged;
	player->frame = kf->frame;
	player->divergeFrame = kf->divergeFrame;
	player->atEnd = false;

	b2RecGrow( (void**)&player->bodyIds, &player->bodyIdCap, kf->bodyIdCount, 0, (int)sizeof( b2BodyId ) );
	player->bodyIdCount = kf->bodyIdCount;
	if ( kf->bodyIdCount > 0 )
	{
		memcpy( player->bodyIds, kf->bodyIds, kf->bodyIdCount * (int)sizeof( b2BodyId ) );
	}
}

void b2Replay_SeekFrame( b2Replay* player, int targetFrame )
{
	if ( player == NULL )
	{
		return;
	}

	if ( targetFrame < 0 )
	{
		targetFrame = 0;
	}

	// 当最近且严格低于目标的关键帧优于当前光标时，从该处恢复。
	// 向后寻址必须恢复，因为光标不能回退。向前寻址仅当关键帧位于光标前方时才恢复，
	// 把长距离前甩限制在一个关键帧间距的回放，而不是每一个中间帧。
	// 严格低于目标，使步进循环仍运行目标帧并再生其每帧查询存储、刚体列表和分叉锁存，
	// 与普通正向回放完全一致。
	const b2RecKeyframe* best = NULL;
	for ( int i = 0; i < player->keyframeCount; ++i )
	{
		const b2RecKeyframe* kf = &player->keyframes[i];
		if ( kf->frame < targetFrame && ( best == NULL || kf->frame > best->frame ) )
		{
			best = kf;
		}
	}

	if ( targetFrame < player->frame )
	{
		if ( best != NULL )
		{
			b2ReplayRestoreKeyframe( player, best );
		}
		else
		{
			b2Replay_Restart( player );
		}
	}
	else if ( best != NULL && best->frame > player->frame )
	{
		b2ReplayRestoreKeyframe( player, best );
	}

	while ( player->frame < targetFrame && b2Replay_StepFrame( player ) )
	{
	}
}

static void b2RecDrawHitAABBs( const b2Replay* player, const b2RecDrawQuery* q, b2DebugDraw* draw )
{
	if ( draw->DrawPolygonFcn == NULL )
	{
		return;
	}
	for ( int hi = q->hitStart; hi < q->hitStart + q->hitCount; ++hi )
	{
		b2ShapeId id = player->frameHits[hi].id;
		if ( b2Shape_IsValid( id ) == false )
		{
			continue;
		}
		b2AABB b = b2Shape_GetBounds( id );
		b2Vec2 lower = b.lowerBound;
		b2Vec2 upper = b.upperBound;
		b2Vec2 vs[4] = { lower, { upper.x, lower.y }, upper, { lower.x, upper.y } };
		draw->DrawPolygonFcn( b2WorldTransform_identity, vs, 4, b2_colorMagenta, draw->context );
	}
}

void b2Replay_DrawFrameQueries( b2Replay* player, b2DebugDraw* draw, int queryIndex )
{
	if ( player == NULL || draw == NULL )
		return;

	// queryIndex < 0 绘制全部查询，否则只绘制查看器中选中的那一个
	for ( int qi = 0; qi < player->frameQueryCount; ++qi )
	{
		if ( queryIndex >= 0 && qi != queryIndex )
		{
			continue;
		}

		const b2RecDrawQuery* q = &player->frameQueries[qi];

		switch ( q->kind )
		{
			case B2_RECQ_CAST_RAY:
			case B2_RECQ_CAST_RAY_CLOSEST:
			{
				// 射线原点到终点
				b2Pos origin = q->origin;
				b2Pos end = b2OffsetPos( origin, q->translation );
				if ( draw->DrawLineFcn )
				{
					draw->DrawLineFcn( origin, end, b2_colorYellow, draw->context );
				}
				// 每个命中的点 + 短法线
				for ( int hi = q->hitStart; hi < q->hitStart + q->hitCount; ++hi )
				{
					const b2RecRecordedHit* h = &player->frameHits[hi];
					b2Pos point = h->point;
					if ( draw->DrawPointFcn )
					{
						draw->DrawPointFcn( point, 4.0f, b2_colorYellow, draw->context );
					}
					if ( draw->DrawLineFcn )
					{
						b2Pos np = b2OffsetPos( point, b2MulSV( 0.2f, h->normal ) );
						draw->DrawLineFcn( point, np, b2_colorLightYellow, draw->context );
					}
				}
				break;
			}
			case B2_RECQ_CAST_SHAPE:
			{
				// 形状投射：沿扫掠路径绘制每个命中点
				for ( int hi = q->hitStart; hi < q->hitStart + q->hitCount; ++hi )
				{
					const b2RecRecordedHit* h = &player->frameHits[hi];
					b2Pos point = h->point;
					if ( draw->DrawPointFcn )
					{
						draw->DrawPointFcn( point, 4.0f, b2_colorSkyBlue, draw->context );
					}
					if ( draw->DrawLineFcn )
					{
						b2Pos np = b2OffsetPos( point, b2MulSV( 0.2f, h->normal ) );
						draw->DrawLineFcn( point, np, b2_colorLightSkyBlue, draw->context );
					}
				}
				break;
			}
			case B2_RECQ_CAST_MOVER:
			{
				// 移动胶囊相对查询原点
				if ( draw->DrawSolidCapsuleFcn )
				{
					b2Pos c1 = b2OffsetPos( q->origin, q->mover.center1 );
					b2Pos c2 = b2OffsetPos( q->origin, q->mover.center2 );
					draw->DrawSolidCapsuleFcn( c1, c2, q->mover.radius, b2_colorLightSkyBlue, draw->context );
				}
				break;
			}
			case B2_RECQ_OVERLAP_AABB:
			{
				// 查询盒相对查询原点
				b2Vec2 lower = q->aabb.lowerBound;
				b2Vec2 upper = q->aabb.upperBound;
				b2Vec2 vs[4] = { lower, { upper.x, lower.y }, upper, { lower.x, upper.y } };
				if ( draw->DrawPolygonFcn )
				{
					draw->DrawPolygonFcn( b2WorldTransform{ q->origin, b2Rot_identity }, vs, 4, b2_colorLimeGreen,
										  draw->context );
				}
				b2RecDrawHitAABBs( player, q, draw );
				break;
			}
			case B2_RECQ_OVERLAP_SHAPE:
			{
				// 代理点相对查询原点
				if ( q->proxy.count == 1 )
				{
					if ( draw->DrawCircleFcn )
					{
						draw->DrawCircleFcn( b2OffsetPos( q->origin, q->proxy.points[0] ), q->proxy.radius, b2_colorLimeGreen,
											 draw->context );
					}
				}
				else if ( q->proxy.count >= 2 && draw->DrawPolygonFcn )
				{
					draw->DrawPolygonFcn( b2WorldTransform{ q->origin, b2Rot_identity }, q->proxy.points, q->proxy.count,
										  b2_colorLimeGreen, draw->context );
				}
				b2RecDrawHitAABBs( player, q, draw );
				break;
			}
			case B2_RECQ_COLLIDE_MOVER:
			{
				// 移动胶囊和碰撞平面相对查询原点
				if ( draw->DrawSolidCapsuleFcn )
				{
					b2Pos c1 = b2OffsetPos( q->origin, q->mover.center1 );
					b2Pos c2 = b2OffsetPos( q->origin, q->mover.center2 );
					draw->DrawSolidCapsuleFcn( c1, c2, q->mover.radius, b2_colorTan, draw->context );
				}
				// 每个命中的平面点和法线
				for ( int hi = q->hitStart; hi < q->hitStart + q->hitCount; ++hi )
				{
					const b2RecRecordedHit* h = &player->frameHits[hi];
					if ( h->plane.hit && draw->DrawLineFcn )
					{
						b2Pos point = b2OffsetPos( q->origin, h->plane.point );
						b2Pos np = b2OffsetPos( point, b2MulSV( 0.2f, h->plane.plane.normal ) );
						draw->DrawLineFcn( point, np, b2_colorOrange, draw->context );
					}
				}
				break;
			}
			default:
				break;
		}
	}
}

int b2Replay_GetFrameQueryCount( const b2Replay* player )
{
	return player != NULL ? player->frameQueryCount : 0;
}

b2ReplayQueryInfo b2Replay_GetFrameQuery( const b2Replay* player, int index )
{
	b2ReplayQueryInfo info = {};
	if ( player == NULL || index < 0 || index >= player->frameQueryCount )
	{
		return info;
	}

	const b2RecDrawQuery* q = &player->frameQueries[index];
	info.type = (b2ReplayQueryType)q->kind;
	info.filter = q->filter;
	info.aabb = q->aabb;
	info.origin = q->origin;
	info.translation = q->translation;
	info.hitCount = q->hitCount;
	return info;
}

b2ReplayQueryHit b2Replay_GetFrameQueryHit( const b2Replay* player, int queryIndex, int hitIndex )
{
	b2ReplayQueryHit hit = {};
	if ( player == NULL || queryIndex < 0 || queryIndex >= player->frameQueryCount )
	{
		return hit;
	}

	const b2RecDrawQuery* q = &player->frameQueries[queryIndex];
	if ( hitIndex < 0 || hitIndex >= q->hitCount )
	{
		return hit;
	}

	const b2RecRecordedHit* h = &player->frameHits[q->hitStart + hitIndex];
	hit.shapeId = h->id;
	hit.point = h->point;
	hit.normal = h->normal;
	hit.fraction = h->fraction;
	return hit;
}

int b2Replay_GetBodyCount( const b2Replay* player )
{
	return player != NULL ? player->bodyIdCount : 0;
}

b2BodyId b2Replay_GetBodyId( const b2Replay* player, int index )
{
	if ( player == NULL || index < 0 || index >= player->bodyIdCount )
	{
		return b2_nullBodyId;
	}
	return player->bodyIds[index];
}

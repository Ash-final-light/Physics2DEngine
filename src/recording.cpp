// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "recording.h"

#include <string.h>

// 缓冲区辅助函数

void b2RecBufAppend( b2RecBuffer* buf, const void* data, int size )
{
	if ( size <= 0 )
	{
		return;
	}

	// 计尺寸遍：只累计字节，不分配也不拷贝
	if ( buf->countOnly )
	{
		buf->size += size;
		return;
	}

	if ( buf->size + size > buf->capacity )
	{
		int newCap = buf->capacity * 2;
		if ( newCap < buf->size + size + 64 )
		{
			newCap = buf->size + size + 64;
		}
		if ( buf->data == NULL )
		{
			buf->data = static_cast<uint8_t *>( b2Alloc( newCap ) );
		}
		else
		{
			buf->data = static_cast<uint8_t *>( b2GrowAlloc( buf->data, buf->capacity, newCap ) );
		}
		buf->capacity = newCap;
	}

	memcpy( buf->data + buf->size, data, (size_t)size );
	buf->size += size;
}

void b2RecBufFree( b2RecBuffer* buf )
{
	if ( buf->data != NULL )
	{
		b2Free( buf->data, buf->capacity );
		buf->data = NULL;
		buf->capacity = 0;
		buf->size = 0;
	}
}

// 写入原语

void b2RecW_U8( b2RecBuffer* buf, uint8_t v )
{
	b2RecBufAppend( buf, &v, 1 );
}

void b2RecW_U16( b2RecBuffer* buf, uint16_t v )
{
	uint8_t b[2] = { (uint8_t)v, (uint8_t)( v >> 8 ) };
	b2RecBufAppend( buf, b, 2 );
}

void b2RecW_U32( b2RecBuffer* buf, uint32_t v )
{
	uint8_t b[4] = { (uint8_t)v, (uint8_t)( v >> 8 ), (uint8_t)( v >> 16 ), (uint8_t)( v >> 24 ) };
	b2RecBufAppend( buf, b, 4 );
}

void b2RecW_U64( b2RecBuffer* buf, uint64_t v )
{
	uint8_t b[8] = { (uint8_t)v,		   (uint8_t)( v >> 8 ),	 (uint8_t)( v >> 16 ), (uint8_t)( v >> 24 ),
					 (uint8_t)( v >> 32 ), (uint8_t)( v >> 40 ), (uint8_t)( v >> 48 ), (uint8_t)( v >> 56 ) };
	b2RecBufAppend( buf, b, 8 );
}

void b2RecW_I32( b2RecBuffer* buf, int32_t v )
{
	b2RecW_U32( buf, (uint32_t)v );
}

void b2RecW_F32( b2RecBuffer* buf, float v )
{
	uint32_t bits;
	memcpy( &bits, &v, 4 );
	b2RecW_U32( buf, bits );
}

void b2RecW_BOOL( b2RecBuffer* buf, bool v )
{
	b2RecW_U8( buf, v ? 1u : 0u );
}

void b2RecW_VEC2( b2RecBuffer* buf, b2Vec2 v )
{
	b2RecW_F32( buf, v.x );
	b2RecW_F32( buf, v.y );
}

void b2RecW_ROT( b2RecBuffer* buf, b2Rot v )
{
	b2RecW_F32( buf, v.c );
	b2RecW_F32( buf, v.s );
}

void b2RecW_XF( b2RecBuffer* buf, b2Transform v )
{
	b2RecW_VEC2( buf, v.p );
	b2RecW_ROT( buf, v.q );
}

void b2RecW_F64( b2RecBuffer* buf, double v )
{
	uint64_t bits;
	memcpy( &bits, &v, 8 );
	b2RecW_U64( buf, bits );
}

// 世界位置在线上保持全精度，使录制能在远离原点处复现仿真。
// 在 float 构建中这是两个 float，与 VEC2 相同。
void b2RecW_POSITION( b2RecBuffer* buf, b2Pos v )
{
#if defined( BOX2D_DOUBLE_PRECISION )
	b2RecW_F64( buf, v.x );
	b2RecW_F64( buf, v.y );
#else
	b2RecW_F32( buf, v.x );
	b2RecW_F32( buf, v.y );
#endif
}

void b2RecW_WORLDXF( b2RecBuffer* buf, b2WorldTransform v )
{
	b2RecW_POSITION( buf, v.p );
	b2RecW_ROT( buf, v.q );
}

void b2RecW_WORLDID( b2RecBuffer* buf, b2WorldId v )
{
	b2RecW_U32( buf, b2StoreWorldId( v ) );
}

void b2RecW_BODYID( b2RecBuffer* buf, b2BodyId v )
{
	b2RecW_U64( buf, b2StoreBodyId( v ) );
}

void b2RecW_SHAPEID( b2RecBuffer* buf, b2ShapeId v )
{
	b2RecW_U64( buf, b2StoreShapeId( v ) );
}

void b2RecW_CHAINID( b2RecBuffer* buf, b2ChainId v )
{
	b2RecW_U64( buf, b2StoreChainId( v ) );
}

void b2RecW_JOINTID( b2RecBuffer* buf, b2JointId v )
{
	b2RecW_U64( buf, b2StoreJointId( v ) );
}

// 几何是无指针 POD，头中的 pointerWidth 门控布局

void b2RecW_CIRCLE( b2RecBuffer* buf, b2Circle v )
{
	b2RecBufAppend( buf, &v, (int)sizeof( b2Circle ) );
}

void b2RecW_CAPSULE( b2RecBuffer* buf, b2Capsule v )
{
	b2RecBufAppend( buf, &v, (int)sizeof( b2Capsule ) );
}

void b2RecW_SEGMENT( b2RecBuffer* buf, b2Segment v )
{
	b2RecBufAppend( buf, &v, (int)sizeof( b2Segment ) );
}

void b2RecW_POLYGON( b2RecBuffer* buf, b2Polygon v )
{
	b2RecBufAppend( buf, &v, (int)sizeof( b2Polygon ) );
}

void b2RecW_CHAINSEG( b2RecBuffer* buf, b2ChainSegment v )
{
	b2RecBufAppend( buf, &v, (int)sizeof( b2ChainSegment ) );
}

void b2RecW_FILTER( b2RecBuffer* buf, b2Filter v )
{
	b2RecW_U64( buf, v.categoryBits );
	b2RecW_U64( buf, v.maskBits );
	b2RecW_I32( buf, v.groupIndex );
}

void b2RecW_MATERIAL( b2RecBuffer* buf, b2SurfaceMaterial v )
{
	b2RecW_F32( buf, v.friction );
	b2RecW_F32( buf, v.restitution );
	b2RecW_F32( buf, v.rollingResistance );
	b2RecW_F32( buf, v.tangentSpeed );
	b2RecW_U64( buf, v.userMaterialId );
	b2RecW_U32( buf, v.customColor );
}

void b2RecW_MASSDATA( b2RecBuffer* buf, b2MassData v )
{
	b2RecW_F32( buf, v.mass );
	b2RecW_VEC2( buf, v.center );
	b2RecW_F32( buf, v.rotationalInertia );
}

void b2RecW_LOCKS( b2RecBuffer* buf, b2MotionLocks v )
{
	b2RecW_BOOL( buf, v.linearX );
	b2RecW_BOOL( buf, v.linearY );
	b2RecW_BOOL( buf, v.angularZ );
}

void b2RecW_STR( b2RecBuffer* buf, const char* s )
{
	if ( s == NULL )
	{
		b2RecW_U16( buf, 0xFFFFu );
		return;
	}
	int len = 0;
	while ( s[len] != '\0' && len < 65534 )
	{
		len++;
	}
	b2RecW_U16( buf, (uint16_t)len );
	if ( len > 0 )
	{
		b2RecBufAppend( buf, s, len );
	}
}

// 手写定义辅助。序列化前将所有指针和 cookie 字段清零
// 读取器先调用 b2Default*Def() 取得 cookie，再覆盖字段

void b2RecW_BODYDEF( b2RecBuffer* buf, b2BodyDef v )
{
	b2RecW_I32( buf, (int32_t)v.type );
	b2RecW_POSITION( buf, v.position );
	b2RecW_ROT( buf, v.rotation );
	b2RecW_VEC2( buf, v.linearVelocity );
	b2RecW_F32( buf, v.angularVelocity );
	b2RecW_F32( buf, v.linearDamping );
	b2RecW_F32( buf, v.angularDamping );
	b2RecW_F32( buf, v.gravityScale );
	b2RecW_F32( buf, v.sleepThreshold );
	b2RecW_F32( buf, v.safetyFactor );
	b2RecW_STR( buf, v.name );
	// userData：不保留
	b2RecW_U64( buf, 0u );
	b2RecW_LOCKS( buf, v.motionLocks );
	b2RecW_BOOL( buf, v.enableSleep );
	b2RecW_BOOL( buf, v.isAwake );
	b2RecW_BOOL( buf, v.isBullet );
	b2RecW_BOOL( buf, v.isEnabled );
	b2RecW_BOOL( buf, v.allowFastRotation );
	b2RecW_BOOL( buf, v.enableContactRecycling );
	// 省略 internalValue
}

void b2RecW_SHAPEDEF( b2RecBuffer* buf, b2ShapeDef v )
{
	// userData：不保留
	b2RecW_U64( buf, 0u );
	b2RecW_MATERIAL( buf, v.material );
	b2RecW_F32( buf, v.density );
	b2RecW_FILTER( buf, v.filter );
	b2RecW_BOOL( buf, v.enableCustomFiltering );
	b2RecW_BOOL( buf, v.isSensor );
	b2RecW_BOOL( buf, v.enableSensorEvents );
	b2RecW_BOOL( buf, v.enableContactEvents );
	b2RecW_BOOL( buf, v.enableHitEvents );
	b2RecW_BOOL( buf, v.enablePreSolveEvents );
	b2RecW_BOOL( buf, v.invokeContactCreation );
	b2RecW_BOOL( buf, v.updateBodyMass );
	// 省略 internalValue
}

// 变长定义：点与材质数组带长度前缀并内联。
// 数组由 b2CreateChain 克隆，因此只需活过这次分派调用。
void b2RecW_CHAINDEF( b2RecBuffer* buf, b2ChainDef v )
{
	// userData：不保留
	b2RecW_U64( buf, 0u );
	b2RecW_I32( buf, v.pointCount );
	for ( int i = 0; i < v.pointCount; ++i )
	{
		b2RecW_VEC2( buf, v.points[i] );
	}
	b2RecW_VEC2( buf, v.ghost1 );
	b2RecW_VEC2( buf, v.ghost2 );
	b2RecW_I32( buf, v.materialCount );
	for ( int i = 0; i < v.materialCount; ++i )
	{
		b2RecW_MATERIAL( buf, v.materials[i] );
	}
	b2RecW_FILTER( buf, v.filter );
	b2RecW_BOOL( buf, v.isLoop );
	b2RecW_BOOL( buf, v.enableSensorEvents );
	// 省略 internalValue
}

void b2RecW_EXPLOSIONDEF( b2RecBuffer* buf, b2ExplosionDef v )
{
	b2RecW_U64( buf, v.maskBits );
	b2RecW_POSITION( buf, v.position );
	b2RecW_F32( buf, v.radius );
	b2RecW_F32( buf, v.falloff );
	b2RecW_F32( buf, v.impulsePerLength );
}

// 关节定义共享基类。刚体 id 按 id 写出，读取时重映射到回放世界。
// userData 和 internalValue 不序列化。
static void b2RecW_JointBase( b2RecBuffer* buf, const b2JointDef* base )
{
	b2RecW_U64( buf, 0u ); // userData：不保留
	b2RecW_BODYID( buf, base->bodyIdA );
	b2RecW_BODYID( buf, base->bodyIdB );
	b2RecW_XF( buf, base->localFrameA );
	b2RecW_XF( buf, base->localFrameB );
	b2RecW_F32( buf, base->forceThreshold );
	b2RecW_F32( buf, base->torqueThreshold );
	b2RecW_F32( buf, base->constraintHertz );
	b2RecW_F32( buf, base->constraintDampingRatio );
	b2RecW_F32( buf, base->drawScale );
	b2RecW_BOOL( buf, base->collideConnected );
}

void b2RecW_DISTANCEJOINTDEF( b2RecBuffer* buf, b2DistanceJointDef v )
{
	b2RecW_JointBase( buf, &v.base );
	b2RecW_F32( buf, v.length );
	b2RecW_BOOL( buf, v.enableSpring );
	b2RecW_F32( buf, v.lowerSpringForce );
	b2RecW_F32( buf, v.upperSpringForce );
	b2RecW_F32( buf, v.hertz );
	b2RecW_F32( buf, v.dampingRatio );
	b2RecW_BOOL( buf, v.enableLimit );
	b2RecW_F32( buf, v.minLength );
	b2RecW_F32( buf, v.maxLength );
	b2RecW_BOOL( buf, v.enableMotor );
	b2RecW_F32( buf, v.maxMotorForce );
	b2RecW_F32( buf, v.motorSpeed );
}

void b2RecW_FILTERJOINTDEF( b2RecBuffer* buf, b2FilterJointDef v )
{
	b2RecW_JointBase( buf, &v.base );
}

void b2RecW_MOTORJOINTDEF( b2RecBuffer* buf, b2MotorJointDef v )
{
	b2RecW_JointBase( buf, &v.base );
	b2RecW_VEC2( buf, v.linearVelocity );
	b2RecW_F32( buf, v.maxVelocityForce );
	b2RecW_F32( buf, v.angularVelocity );
	b2RecW_F32( buf, v.maxVelocityTorque );
	b2RecW_F32( buf, v.linearHertz );
	b2RecW_F32( buf, v.linearDampingRatio );
	b2RecW_F32( buf, v.maxSpringForce );
	b2RecW_F32( buf, v.angularHertz );
	b2RecW_F32( buf, v.angularDampingRatio );
	b2RecW_F32( buf, v.maxSpringTorque );
}

void b2RecW_MOVERJOINTDEF( b2RecBuffer* buf, b2MoverJointDef v )
{
	b2RecW_JointBase( buf, &v.base );
	b2RecW_VEC2( buf, v.linearVelocity );
	b2RecW_VEC2( buf, v.maxVelocityForce );
}

void b2RecW_POGOJOINTDEF( b2RecBuffer* buf, b2PogoJointDef v )
{
	b2RecW_JointBase( buf, &v.base );
	b2RecW_VEC2( buf, v.normal );
	b2RecW_F32( buf, v.hertz );
	b2RecW_F32( buf, v.dampingRatio );
	b2RecW_F32( buf, v.restLength );
	b2RecW_F32( buf, v.maxTensionForce );
	b2RecW_F32( buf, v.maxCompressionForce );
	b2RecW_F32( buf, v.impulse );
	b2RecW_F32( buf, v.velocity );
}

void b2RecW_PRISMATICJOINTDEF( b2RecBuffer* buf, b2PrismaticJointDef v )
{
	b2RecW_JointBase( buf, &v.base );
	b2RecW_BOOL( buf, v.enableSpring );
	b2RecW_F32( buf, v.hertz );
	b2RecW_F32( buf, v.dampingRatio );
	b2RecW_F32( buf, v.targetTranslation );
	b2RecW_BOOL( buf, v.enableLimit );
	b2RecW_F32( buf, v.lowerTranslation );
	b2RecW_F32( buf, v.upperTranslation );
	b2RecW_BOOL( buf, v.enableMotor );
	b2RecW_F32( buf, v.maxMotorForce );
	b2RecW_F32( buf, v.motorSpeed );
}

void b2RecW_REVOLUTEJOINTDEF( b2RecBuffer* buf, b2RevoluteJointDef v )
{
	b2RecW_JointBase( buf, &v.base );
	b2RecW_F32( buf, v.targetAngle );
	b2RecW_BOOL( buf, v.enableSpring );
	b2RecW_F32( buf, v.hertz );
	b2RecW_F32( buf, v.dampingRatio );
	b2RecW_BOOL( buf, v.enableLimit );
	b2RecW_F32( buf, v.lowerAngle );
	b2RecW_F32( buf, v.upperAngle );
	b2RecW_BOOL( buf, v.enableMotor );
	b2RecW_F32( buf, v.maxMotorTorque );
	b2RecW_F32( buf, v.motorSpeed );
}

void b2RecW_WELDJOINTDEF( b2RecBuffer* buf, b2WeldJointDef v )
{
	b2RecW_JointBase( buf, &v.base );
	b2RecW_F32( buf, v.linearHertz );
	b2RecW_F32( buf, v.angularHertz );
	b2RecW_F32( buf, v.linearDampingRatio );
	b2RecW_F32( buf, v.angularDampingRatio );
}

void b2RecW_WHEELJOINTDEF( b2RecBuffer* buf, b2WheelJointDef v )
{
	b2RecW_JointBase( buf, &v.base );
	b2RecW_BOOL( buf, v.enableSpring );
	b2RecW_F32( buf, v.hertz );
	b2RecW_F32( buf, v.dampingRatio );
	b2RecW_BOOL( buf, v.enableLimit );
	b2RecW_F32( buf, v.lowerTranslation );
	b2RecW_F32( buf, v.upperTranslation );
	b2RecW_BOOL( buf, v.enableMotor );
	b2RecW_F32( buf, v.maxMotorTorque );
	b2RecW_F32( buf, v.motorSpeed );
}

void b2RecW_AABB( b2RecBuffer* buf, b2AABB v )
{
	b2RecW_VEC2( buf, v.lowerBound );
	b2RecW_VEC2( buf, v.upperBound );
}

void b2RecW_QUERYFILTER( b2RecBuffer* buf, b2QueryFilter v )
{
	b2RecW_U64( buf, v.categoryBits );
	b2RecW_U64( buf, v.maskBits );
}

void b2RecW_SHAPEPROXY( b2RecBuffer* buf, b2ShapeProxy v )
{
	int count = v.count;
	if ( count < 0 )
		count = 0;
	if ( count > B2_MAX_POLYGON_VERTICES )
		count = B2_MAX_POLYGON_VERTICES;
	b2RecW_I32( buf, count );
	for ( int i = 0; i < count; ++i )
	{
		b2RecW_VEC2( buf, v.points[i] );
	}
	b2RecW_F32( buf, v.radius );
}

void b2RecW_RAYRESULT( b2RecBuffer* buf, b2RayResult v )
{
	b2RecW_SHAPEID( buf, v.shapeId );
	b2RecW_POSITION( buf, v.point );
	b2RecW_VEC2( buf, v.normal );
	b2RecW_F32( buf, v.fraction );
	b2RecW_I32( buf, v.nodeVisits );
	b2RecW_I32( buf, v.leafVisits );
	b2RecW_BOOL( buf, v.hit );
}

void b2RecW_PLANERESULT( b2RecBuffer* buf, b2PlaneResult v )
{
	b2RecW_VEC2( buf, v.plane.normal );
	b2RecW_F32( buf, v.plane.offset );
	b2RecW_VEC2( buf, v.point );
	b2RecW_BOOL( buf, v.hit );
}

void b2RecW_TREESTATS( b2RecBuffer* buf, b2TreeStats v )
{
	b2RecW_I32( buf, v.nodeVisits );
	b2RecW_I32( buf, v.leafVisits );
}

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "recording.h"
#include "snapshot.h"
#include "state_hash.h"
#include "world.h"
#include "box2d/math_functions.h"

#include <string.h>
#include <stdio.h>
#include <limits.h>

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

b2Recording* b2CreateRecording( int byteCapacity )
{
	b2Recording* rec = static_cast<b2Recording *>( b2Alloc( (int)sizeof( b2Recording ) ) );
	*rec = b2Recording{};

	// 按需预分配；缓冲区仍会按需加倍。几百 KiB 对短会话足够，并可避免早期重分配。
	int initCap = byteCapacity > 0 ? byteCapacity : 65536;
	rec->buffer.data = static_cast<uint8_t *>( b2Alloc( initCap ) );
	rec->buffer.capacity = initCap;
	rec->buffer.size = 0;
	rec->lock = b2CreateMutex();
	return rec;
}

void b2DestroyRecording( b2Recording* recording )
{
	if ( recording == NULL )
	{
		return;
	}

	b2RecBufFree( &recording->buffer );
	b2DestroyMutex( recording->lock );
	b2Free( recording, (int)sizeof( b2Recording ) );
}

const uint8_t* b2Recording_GetData( const b2Recording* recording )
{
	return recording->buffer.data;
}

int b2Recording_GetSize( const b2Recording* recording )
{
	return recording->buffer.size;
}

// 记录分帧

void b2RecBeginRecord( b2Recording* rec, uint8_t opcode )
{
	b2RecW_U8( &rec->buffer, opcode );
	rec->recordStart = rec->buffer.size;
	// 预留保存 24 位载荷大小的空间，该大小直到调用 b2RecEndRecord 才知道。
	uint8_t zero[3] = { 0, 0, 0 };
	b2RecBufAppend( &rec->buffer, zero, 3 );
}

void b2RecEndRecord( b2Recording* rec )
{
	// 计算最终载荷大小，并写入紧接操作码之后预留的 24 位空间。
	int payloadSize = rec->buffer.size - rec->recordStart - 3;
	B2_ASSERT( payloadSize >= 0 && payloadSize < ( 1 << 24 ) );
	uint8_t* p = rec->buffer.data + rec->recordStart;
	p[0] = (uint8_t)payloadSize;
	p[1] = (uint8_t)( payloadSize >> 8 );
	p[2] = (uint8_t)( payloadSize >> 16 );
}

// 代码生成第 1b 遍：参数写入器。每个生成的函数将其结构字段写入缓冲区
// 例如：
// B2_REC_OP( 0x80, Step, RET_NONE, ARG( WORLDID, world ) ARG( F32, dt ) ARG( I32, subStepCount ) )
// 变为：
// void b2RecWriteArgs_Step( b2Recording* rec, const b2RecArgs_Step* a)
// {
//   b2RecW_WORLDID( &rec->buffer, a->world );
//   b2RecW_F32( &rec->buffer, a->dt );
//   b2RecW_I32( &rec->buffer, a->subStepCount );
// }
#define ARG( TAG, field ) b2RecW_##TAG( &rec->buffer, a->field );
#define B2_REC_OP( op, Name, RET, ... )                                                                                          \
	void b2RecWriteArgs_##Name( b2Recording* rec, const b2RecArgs_##Name* a )                                                    \
	{                                                                                                                            \
		__VA_ARGS__                                                                                                              \
	}
#include "recording_ops.inl"
#undef B2_REC_OP
#undef ARG

// 代码生成：包装 begin、参数写入器、end 的完整写入器
// 设置器与创建可能在各自持有不同对象的线程上运行，因此整条记录期间持锁。
// 否则并发写入器会把字节插入我们的 begin 与 end 之间，记录会使回放失步。
// 与查询提交路径使用同一把锁。
// 例如：
// B2_REC_OP( 0x80, Step, RET_NONE, ARG( WORLDID, world ) ARG( F32, dt ) ARG( I32, subStepCount ) )
// 变为：
// void b2RecWrite_Step( b2Recording* rec, const b2RecArgs_Step* a)
// {
//   b2LockMutex( rec->lock );
//   b2RecBeginRecord( rec, (uint8_t)( 0x80 ) );
//   b2RecWriteArgs_Step( rec, a );
//   b2RecEndRecord( rec );
//   b2UnlockMutex( rec->lock );
// }
#define B2_REC_OP( op, Name, RET, ... )                                                                                          \
	void b2RecWrite_##Name( b2Recording* rec, const b2RecArgs_##Name* a )                                                        \
	{                                                                                                                            \
		b2LockMutex( rec->lock );                                                                                                \
		b2RecBeginRecord( rec, (uint8_t)( op ) );                                                                                \
		b2RecWriteArgs_##Name( rec, a );                                                                                         \
		b2RecEndRecord( rec );                                                                                                   \
		b2UnlockMutex( rec->lock );                                                                                              \
	}
#include "recording_ops.inl"
#undef B2_REC_OP

// 代码生成：在记录内追加返回 id 的创建操作写入器。RET 标签
// 选择 id 类型及其写入原语。RET_NONE 操作不生成任何内容。
// 例如：
// B2_REC_OP( 0x40, CreateCircleShape, RET_SHAPEID, ARG( BODYID, body ) ARG( SHAPEDEF, def ) ARG( CIRCLE, circle ) )
// 变为：
// 	void b2RecWriteRet_CreateCircleShape( b2Recording* rec, const b2RecArgs_CreateCircleShape* a, idType id )
//	{
//		b2LockMutex( rec->lock );
//		b2RecBeginRecord( rec, (uint8_t)( 0x40 ) );
//		b2RecWriteArgs_CreateCircleShape( rec, a );
//		b2RecW_SHAPEID( &rec->buffer, id );
//		b2RecEndRecord( rec );
//		b2UnlockMutex( rec->lock );
//	}

#define B2_REC_RETWRITE( op, Name, idType, idW )                                                                                 \
	void b2RecWriteRet_##Name( b2Recording* rec, const b2RecArgs_##Name* a, idType id )                                          \
	{                                                                                                                            \
		b2LockMutex( rec->lock );                                                                                                \
		b2RecBeginRecord( rec, (uint8_t)( op ) );                                                                                \
		b2RecWriteArgs_##Name( rec, a );                                                                                         \
		idW( &rec->buffer, id );                                                                                                 \
		b2RecEndRecord( rec );                                                                                                   \
		b2UnlockMutex( rec->lock );                                                                                              \
	}
#define B2_REC_RETWRITE_RET_NONE( op, Name )
#define B2_REC_RETWRITE_RET_BODYID( op, Name ) B2_REC_RETWRITE( op, Name, b2BodyId, b2RecW_BODYID )
#define B2_REC_RETWRITE_RET_SHAPEID( op, Name ) B2_REC_RETWRITE( op, Name, b2ShapeId, b2RecW_SHAPEID )
#define B2_REC_RETWRITE_RET_CHAINID( op, Name ) B2_REC_RETWRITE( op, Name, b2ChainId, b2RecW_CHAINID )
#define B2_REC_RETWRITE_RET_JOINTID( op, Name ) B2_REC_RETWRITE( op, Name, b2JointId, b2RecW_JOINTID )
#define B2_REC_OP( op, Name, RET, ... ) B2_REC_RETWRITE_##RET( op, Name )
#include "recording_ops.inl"
#undef B2_REC_OP
#undef B2_REC_RETWRITE_RET_NONE
#undef B2_REC_RETWRITE_RET_BODYID
#undef B2_REC_RETWRITE_RET_SHAPEID
#undef B2_REC_RETWRITE_RET_CHAINID
#undef B2_REC_RETWRITE_RET_JOINTID
#undef B2_REC_RETWRITE

void b2RecAccumulateBounds( b2Recording* rec, b2AABB bounds )
{
	rec->accumulatedBounds = rec->haveBounds ? b2AABB_Union( rec->accumulatedBounds, bounds ) : bounds;
	rec->haveBounds = true;
}

void b2StartRecordingIntoBuffer( b2World* world, b2Recording* recording )
{
	// 重置以便录制句柄可复用于新会话
	recording->buffer.size = 0;
	recording->recordStart = 0;
	recording->haveBounds = false;

	// 将活动世界序列化为头之后的数据块，并为回放播种。
	b2RecBuffer blob = {};
	b2SerializeWorld( world, &blob );

	b2RecHeader hdr = {};
	hdr.magic = B2_REC_MAGIC;
	hdr.versionMajor = B2_REC_VERSION_MAJOR;
	hdr.versionMinor = B2_REC_VERSION_MINOR;
	hdr.lengthScale = b2GetLengthUnitsPerMeter();
	hdr.pointerWidth = (uint8_t)sizeof( void* );
	hdr.bigEndian = 0;
	hdr.validationEnabled = B2_ENABLE_VALIDATION ? 1u : 0u;
	hdr.snapshotSize = (uint64_t)blob.size;
	b2RecBufAppend( &recording->buffer, &hdr, (int)sizeof( hdr ) );
	b2RecBufAppend( &recording->buffer, blob.data, blob.size );
	b2RecBufFree( &blob );

	world->recording = recording;

	// 用快照状态为包围盒播种，即使没有运动也能框住第 0 帧
	b2AABB seed;
	if ( b2ComputeWorldBounds( world, &seed ) )
	{
		b2RecAccumulateBounds( recording, seed );
	}

	// 锚定已录制的状态哈希，以便回放验证数据块反序列化为同一世界。
	b2WorldId worldId = { (uint16_t)( world->worldId + 1 ), world->generation };
	b2RecArgs_StateHash stateHash = { worldId, b2HashWorldState( world ) };
	b2RecWrite_StateHash( recording, &stateHash );
}

void b2StopRecordingInternal( b2World* world )
{
	if ( world->recording == NULL )
	{
		return;
	}

	b2Recording* rec = world->recording;
	world->recording = NULL;

	// 暂存累计包围盒，使查看器在打开时能框住整个运动。位于操作流中结束标记之前；
	// 旧录制中不存在。
	b2RecArgs_RecordingBounds rb = {};
	if ( rec->haveBounds )
	{
		rb.bounds = rec->accumulatedBounds;
	}
	b2RecWrite_RecordingBounds( rec, &rb );

	// 写入 DestroyWorld，使缓冲区自包含，作为查看器读取的结束标记。
	// 缓冲区和句柄属于用户，由 b2DestroyRecording 释放。
	b2WorldId wid = { (uint16_t)( world->worldId + 1 ), world->generation };
	b2RecArgs_DestroyWorld a = { wid };
	b2RecWrite_DestroyWorld( rec, &a );
}

int b2RecReserveU32( b2RecBuffer* buf )
{
	int offset = buf->size;
	uint8_t zero[4] = { 0, 0, 0, 0 };
	b2RecBufAppend( buf, zero, 4 );
	return offset;
}

void b2RecPatchU32( b2RecBuffer* buf, int offset, uint32_t v )
{
	B2_ASSERT( offset >= 0 && offset + 4 <= buf->size );
	uint8_t* p = buf->data + offset;
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)( v >> 8 );
	p[2] = (uint8_t)( v >> 16 );
	p[3] = (uint8_t)( v >> 24 );
}

// 并发查询提交被串行化，使记录不会在共享缓冲区中交错
void b2RecCommitRecord( b2Recording* rec, uint8_t opcode, const uint8_t* payload, int payloadSize )
{
	B2_ASSERT( payloadSize >= 0 && payloadSize < ( 1 << 24 ) );
	b2LockMutex( rec->lock );
	b2RecW_U8( &rec->buffer, opcode );
	uint8_t sz[3] = { (uint8_t)payloadSize, (uint8_t)( payloadSize >> 8 ), (uint8_t)( payloadSize >> 16 ) };
	b2RecBufAppend( &rec->buffer, sz, 3 );
	b2RecBufAppend( &rec->buffer, payload, payloadSize );
	b2UnlockMutex( rec->lock );
}

void b2RecQueryBegin( b2RecQueryWriter* w, void* context )
{
	w->buf = b2RecBuffer{};
	w->userFcn.overlapFcn = NULL;
	w->userContext = context;
	w->hitCount = 0;
	w->countOffset = 0;
}

void b2RecQueryCommit( b2Recording* rec, uint8_t opcode, b2RecQueryWriter* w )
{
	b2RecCommitRecord( rec, opcode, w->buf.data, w->buf.size );
	b2RecBufFree( &w->buf );
}

bool b2RecOverlapTrampoline( b2ShapeId id, void* ctx )
{
	b2RecQueryWriter* w = (b2RecQueryWriter*)ctx;
	bool ret = w->userFcn.overlapFcn( id, w->userContext );
	b2RecW_SHAPEID( &w->buf, id );
	b2RecW_BOOL( &w->buf, ret );
	w->hitCount++;
	return ret;
}

float b2RecCastTrampoline( b2ShapeId id, b2Pos point, b2Vec2 normal, float fraction, void* ctx )
{
	b2RecQueryWriter* w = (b2RecQueryWriter*)ctx;
	float ret = w->userFcn.castFcn( id, point, normal, fraction, w->userContext );
	b2RecW_SHAPEID( &w->buf, id );
	b2RecW_POSITION( &w->buf, point );
	b2RecW_VEC2( &w->buf, normal );
	b2RecW_F32( &w->buf, fraction );
	b2RecW_F32( &w->buf, ret );
	w->hitCount++;
	return ret;
}

bool b2RecPlaneTrampoline( b2ShapeId id, const b2PlaneResult* plane, void* ctx )
{
	b2RecQueryWriter* w = (b2RecQueryWriter*)ctx;
	bool ret = w->userFcn.planeFcn( id, plane, w->userContext );
	b2RecW_SHAPEID( &w->buf, id );
	b2RecW_PLANERESULT( &w->buf, *plane );
	b2RecW_BOOL( &w->buf, ret );
	w->hitCount++;
	return ret;
}

bool b2Recording_SaveToFile( const b2Recording* recording, const char* path )
{
	if ( recording == NULL || path == NULL )
	{
		return false;
	}

	FILE* f = fopen( path, "wb" );
	if ( f == NULL )
	{
		return false;
	}

	size_t written = fwrite( recording->buffer.data, 1, (size_t)recording->buffer.size, f );
	int closeResult = fclose( f );
	return (int)written == recording->buffer.size && closeResult == 0;
}

// Structural file validation. Operation payload semantics are checked by replay dispatch.
static bool b2RecValidateFile( const uint8_t* data, int size )
{
    if ( size < (int)sizeof( b2RecHeader ) ) { return false; }
    b2RecHeader hdr;
    memcpy( &hdr, data, sizeof( hdr ) );
    if ( hdr.magic != B2_REC_MAGIC || hdr.versionMajor != B2_REC_VERSION_MAJOR ||
         hdr.versionMinor != B2_REC_VERSION_MINOR || hdr.pointerWidth != sizeof( void* ) || hdr.bigEndian != 0 ||
         !b2IsValidFloat( hdr.lengthScale ) || hdr.lengthScale <= 0 ||
         hdr.snapshotSize < 16 || hdr.snapshotSize > (uint64_t)( size - sizeof( hdr ) ) ) { return false; }
    uint32_t snapshotHeader[4];
    memcpy( snapshotHeader, data + sizeof( hdr ), sizeof( snapshotHeader ) );
    if ( snapshotHeader[0] != 0x32534E42u || snapshotHeader[1] != 13u ||
         ( ( snapshotHeader[3] & 2u ) != 0 ) != b2IsDoublePrecision() ) { return false; }
    int cursor = (int)sizeof( hdr ) + (int)hdr.snapshotSize;
    while ( cursor < size )
    {
        if ( size - cursor < 4 ) { return false; }
        switch ( data[cursor] )
        {
#define B2_REC_OP( op, Name, RET, ... ) case op: break;
#include "recording_ops.inl"
#undef B2_REC_OP
            default: return false;
        }
        int payload = data[cursor + 1] | (int)data[cursor + 2] << 8 | (int)data[cursor + 3] << 16;
        if ( payload > size - cursor - 4 ) { return false; }
        cursor += 4 + payload;
    }
    return true;
}

b2Recording* b2CreateRecordingFromFile( const char* path )
{
	if ( path == NULL )
	{
		return NULL;
	}

	FILE* f = fopen( path, "rb" );
	if ( f == NULL )
	{
		return NULL;
	}

	if ( fseek( f, 0, SEEK_END ) != 0 )
	{
		fclose( f );
		return NULL;
	}

	long fileSize = ftell( f );
	// 小于固定头的内容不可能是录制，因此在此拒绝，而不是交回稍后才失败的畸形缓冲区
	if ( fileSize < (long)sizeof( b2RecHeader ) || fileSize > INT_MAX )
	{
		fclose( f );
		return NULL;
	}
	fseek( f, 0, SEEK_SET );

	b2Recording* rec = b2CreateRecording( (int)fileSize );
	size_t readSize = fread( rec->buffer.data, 1, (size_t)fileSize, f );
	fclose( f );

	if ( (long)readSize != fileSize )
	{
		b2DestroyRecording( rec );
		return NULL;
	}

	// Reject incompatible headers and incomplete frames before returning ownership.
	if ( !b2RecValidateFile( rec->buffer.data, (int)fileSize ) )
	{
		b2DestroyRecording( rec );
		return NULL;
	}

	rec->buffer.size = (int)fileSize;
	return rec;
}

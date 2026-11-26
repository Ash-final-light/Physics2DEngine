// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "core.h"
#include "box2d/types.h"

// 可增长的只追加字节缓冲区。按需加倍。countOnly 模式下只累计大小而不分配，
// 因此可先廉价估算序列化大小，再二次填充真实缓冲区。
typedef struct b2RecBuffer
{
	uint8_t* data;
	int capacity;
	int size;
	bool countOnly;
} b2RecBuffer;

// 底层缓冲区辅助函数
void b2RecBufAppend( b2RecBuffer* buf, const void* data, int size );
void b2RecBufFree( b2RecBuffer* buf );

// 写入原语
void b2RecW_U8( b2RecBuffer* buf, uint8_t v );
void b2RecW_U16( b2RecBuffer* buf, uint16_t v );
void b2RecW_U32( b2RecBuffer* buf, uint32_t v );
void b2RecW_U64( b2RecBuffer* buf, uint64_t v );
void b2RecW_I32( b2RecBuffer* buf, int32_t v );
void b2RecW_F32( b2RecBuffer* buf, float v );
void b2RecW_BOOL( b2RecBuffer* buf, bool v );
void b2RecW_VEC2( b2RecBuffer* buf, b2Vec2 v );
void b2RecW_ROT( b2RecBuffer* buf, b2Rot v );
void b2RecW_XF( b2RecBuffer* buf, b2Transform v );
void b2RecW_F64( b2RecBuffer* buf, double v );
// 世界位置与世界变换。大世界模式下每个位置两个 double，否则两个 float，
// 使线上字节与 float 构建中的 VEC2 / XF 完全相同。
void b2RecW_POSITION( b2RecBuffer* buf, b2Pos v );
void b2RecW_WORLDXF( b2RecBuffer* buf, b2WorldTransform v );
void b2RecW_WORLDID( b2RecBuffer* buf, b2WorldId v );
void b2RecW_BODYID( b2RecBuffer* buf, b2BodyId v );
void b2RecW_SHAPEID( b2RecBuffer* buf, b2ShapeId v );
void b2RecW_CHAINID( b2RecBuffer* buf, b2ChainId v );
void b2RecW_JOINTID( b2RecBuffer* buf, b2JointId v );
void b2RecW_CIRCLE( b2RecBuffer* buf, b2Circle v );
void b2RecW_CAPSULE( b2RecBuffer* buf, b2Capsule v );
void b2RecW_SEGMENT( b2RecBuffer* buf, b2Segment v );
void b2RecW_POLYGON( b2RecBuffer* buf, b2Polygon v );
void b2RecW_CHAINSEG( b2RecBuffer* buf, b2ChainSegment v );
void b2RecW_FILTER( b2RecBuffer* buf, b2Filter v );
void b2RecW_MATERIAL( b2RecBuffer* buf, b2SurfaceMaterial v );
void b2RecW_MASSDATA( b2RecBuffer* buf, b2MassData v );
void b2RecW_LOCKS( b2RecBuffer* buf, b2MotionLocks v );
void b2RecW_STR( b2RecBuffer* buf, const char* s );
void b2RecW_EXPLOSIONDEF( b2RecBuffer* buf, b2ExplosionDef v );
void b2RecW_BODYDEF( b2RecBuffer* buf, b2BodyDef v );
void b2RecW_SHAPEDEF( b2RecBuffer* buf, b2ShapeDef v );
void b2RecW_CHAINDEF( b2RecBuffer* buf, b2ChainDef v );
void b2RecW_DISTANCEJOINTDEF( b2RecBuffer* buf, b2DistanceJointDef v );
void b2RecW_FILTERJOINTDEF( b2RecBuffer* buf, b2FilterJointDef v );
void b2RecW_MOTORJOINTDEF( b2RecBuffer* buf, b2MotorJointDef v );
void b2RecW_MOVERJOINTDEF( b2RecBuffer* buf, b2MoverJointDef v );
void b2RecW_POGOJOINTDEF( b2RecBuffer* buf, b2PogoJointDef v );
void b2RecW_PRISMATICJOINTDEF( b2RecBuffer* buf, b2PrismaticJointDef v );
void b2RecW_REVOLUTEJOINTDEF( b2RecBuffer* buf, b2RevoluteJointDef v );
void b2RecW_WELDJOINTDEF( b2RecBuffer* buf, b2WeldJointDef v );
void b2RecW_WHEELJOINTDEF( b2RecBuffer* buf, b2WheelJointDef v );
void b2RecW_AABB( b2RecBuffer* buf, b2AABB v );
void b2RecW_QUERYFILTER( b2RecBuffer* buf, b2QueryFilter v );
void b2RecW_SHAPEPROXY( b2RecBuffer* buf, b2ShapeProxy v );
void b2RecW_RAYRESULT( b2RecBuffer* buf, b2RayResult v );
void b2RecW_PLANERESULT( b2RecBuffer* buf, b2PlaneResult v );
void b2RecW_TREESTATS( b2RecBuffer* buf, b2TreeStats v );

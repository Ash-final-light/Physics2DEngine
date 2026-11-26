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

// 用户持有的录制缓冲区。录制期间世界向其中追加；由用户保存并销毁。在公开 API 中不透明。
typedef struct b2Recording
{
	b2RecBuffer buffer;
	int recordStart; // 供 u24 回填的 3 字节大小字段偏移
	b2Mutex* lock;	 // 跨并发查询线程串行化查询记录提交

	// 每个已录制步的世界包围盒并集，在停止时写出，以便回放框住整个运动。
	// haveBounds 以与 b2World_GetBounds 相同的方式门控第一次并集。
	b2AABB accumulatedBounds;
	bool haveBounds;
} b2Recording;

typedef struct b2World b2World;

// 魔数 'B2RC' 的小端表示：字节 B2、R、C 得到 0x43523242
#define B2_REC_MAGIC 0x43523242u

// 录制格式版本。任何不匹配都拒绝加载。次版本号跟踪在保持 32 字节头形状下的操作流布局变更。
#define B2_REC_VERSION_MAJOR 3
#define B2_REC_VERSION_MINOR 13 // 未录制形状查询

// 文件头，固定 32 字节，小端
typedef struct b2RecHeader
{
	uint32_t magic;		   // 'B2RC' = 0x43523242
	uint16_t versionMajor; // B2_REC_VERSION_MAJOR
	uint16_t versionMinor; // B2_REC_VERSION_MINOR
	uint32_t reserved2;
	float lengthScale; // 世界长度单位
	uint8_t reserved3;
	uint8_t pointerWidth;	   // sizeof(void*)，用于门控 POD 定义的 memcpy
	uint8_t bigEndian;		   // 所有支持的目标上为 0
	uint8_t validationEnabled; // 带校验构建时为 1，仅用于布局不匹配时的诊断
	uint32_t reserved1;
	uint64_t snapshotSize; // 头之后快照数据块的字节数
} b2RecHeader;

static_assert( sizeof( b2RecHeader ) == 32, "recording header must be 32 bytes" );


// 按 TAG 的 C 类型别名，用于代码生成的参数结构
typedef bool b2RecCType_BOOL;
typedef int32_t b2RecCType_I32;
typedef uint32_t b2RecCType_U32;
typedef uint64_t b2RecCType_U64;
typedef float b2RecCType_F32;
typedef b2Vec2 b2RecCType_VEC2;
typedef b2Rot b2RecCType_ROT;
typedef b2Transform b2RecCType_XF;
typedef b2Pos b2RecCType_POSITION;
typedef b2WorldTransform b2RecCType_WORLDXF;
typedef const char* b2RecCType_STR;
typedef b2WorldId b2RecCType_WORLDID;
typedef b2BodyId b2RecCType_BODYID;
typedef b2ShapeId b2RecCType_SHAPEID;
typedef b2ChainId b2RecCType_CHAINID;
typedef b2JointId b2RecCType_JOINTID;
typedef b2Circle b2RecCType_CIRCLE;
typedef b2Capsule b2RecCType_CAPSULE;
typedef b2Segment b2RecCType_SEGMENT;
typedef b2Polygon b2RecCType_POLYGON;
typedef b2ChainSegment b2RecCType_CHAINSEG;
typedef b2Filter b2RecCType_FILTER;
typedef b2SurfaceMaterial b2RecCType_MATERIAL;
typedef b2MassData b2RecCType_MASSDATA;
typedef b2MotionLocks b2RecCType_LOCKS;
typedef b2ExplosionDef b2RecCType_EXPLOSIONDEF;
typedef b2BodyDef b2RecCType_BODYDEF;
typedef b2ShapeDef b2RecCType_SHAPEDEF;
typedef b2ChainDef b2RecCType_CHAINDEF;
typedef b2DistanceJointDef b2RecCType_DISTANCEJOINTDEF;
typedef b2FilterJointDef b2RecCType_FILTERJOINTDEF;
typedef b2MotorJointDef b2RecCType_MOTORJOINTDEF;
typedef b2MoverJointDef b2RecCType_MOVERJOINTDEF;
typedef b2PogoJointDef b2RecCType_POGOJOINTDEF;
typedef b2PrismaticJointDef b2RecCType_PRISMATICJOINTDEF;
typedef b2RevoluteJointDef b2RecCType_REVOLUTEJOINTDEF;
typedef b2WeldJointDef b2RecCType_WELDJOINTDEF;
typedef b2WheelJointDef b2RecCType_WHEELJOINTDEF;
typedef b2AABB b2RecCType_AABB;
typedef b2QueryFilter b2RecCType_QUERYFILTER;
typedef b2ShapeProxy b2RecCType_SHAPEPROXY;

// 代码生成第 1a 遍：参数结构，在 recording.c 中生成，在此声明供调用点使用。
// 这些 typedef 在 recording.c 的写辅助函数之前，但必须对使用 B2_REC 的 body.c 和 shape.c 可见。
// 在此通过 X 宏生成。
// 重要：此块不得把 ARG 或 B2_REC_OP 展开为函数。
//
// 例如，此：
// B2_REC_OP( 0x80, Step, RET_NONE, ARG( WORLDID, world ) ARG( F32, dt ) ARG( I32, subStepCount ) )
// 变为：
// typedef struct
// {
//   b2RecCType_WORLDID world;
//   b2RecCType_F32 dt;
//   b2RecCType_I32 subStepCount;
// } b2RecArgs_Step;
// 即 b2World_Step 的参数
#define ARG( TAG, field ) b2RecCType_##TAG field;
#define B2_REC_OP( op, Name, RET, ... )                                                                                          \
	typedef struct                                                                                                               \
	{                                                                                                                            \
		__VA_ARGS__                                                                                                              \
	} b2RecArgs_##Name;
#include "recording_ops.inl"
#undef B2_REC_OP
#undef ARG

// 记录分帧
void b2RecBeginRecord( b2Recording* rec, uint8_t opcode );
void b2RecEndRecord( b2Recording* rec );

// 每个操作的参数写入器（无分帧）和完整写入器（分帧加参数），由清单生成。
// 创建操作直接调用参数写入器，以便调用点可在同一记录内追加返回的 id；
// void 操作通过 B2_REC 调用完整写入器。
#define B2_REC_OP( op, Name, RET, ... )                                                                                          \
	void b2RecWriteArgs_##Name( b2Recording* rec, const b2RecArgs_##Name* a );                                                   \
	void b2RecWrite_##Name( b2Recording* rec, const b2RecArgs_##Name* a );
#include "recording_ops.inl"
#undef B2_REC_OP

// 创建操作还需要一个在同一记录内追加返回 id 的写入器。
// 仅对带 RET 标签的操作生成。id 类型来自该标签。
// 录制返回的 id 以便校验确定性回放。
#define B2_REC_RETDECL_RET_NONE( Name )
#define B2_REC_RETDECL_RET_BODYID( Name ) void b2RecWriteRet_##Name( b2Recording* rec, const b2RecArgs_##Name* a, b2BodyId id );
#define B2_REC_RETDECL_RET_SHAPEID( Name ) void b2RecWriteRet_##Name( b2Recording* rec, const b2RecArgs_##Name* a, b2ShapeId id );
#define B2_REC_RETDECL_RET_CHAINID( Name ) void b2RecWriteRet_##Name( b2Recording* rec, const b2RecArgs_##Name* a, b2ChainId id );
#define B2_REC_RETDECL_RET_JOINTID( Name ) void b2RecWriteRet_##Name( b2Recording* rec, const b2RecArgs_##Name* a, b2JointId id );
#define B2_REC_OP( op, Name, RET, ... ) B2_REC_RETDECL_##RET( Name )
#include "recording_ops.inl"
#undef B2_REC_OP
#undef B2_REC_RETDECL_RET_NONE
#undef B2_REC_RETDECL_RET_BODYID
#undef B2_REC_RETDECL_RET_SHAPEID
#undef B2_REC_RETDECL_RET_CHAINID
#undef B2_REC_RETDECL_RET_JOINTID

// 录制一个 void 操作。录制关闭时只有一个分支，参数在该分支内构造
#define B2_REC( world, Name, ... )                                                                                               \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( world )->recording != NULL )                                                                                      \
		{                                                                                                                        \
			b2RecArgs_##Name _a = { __VA_ARGS__ };                                                                               \
			b2RecWrite_##Name( ( world )->recording, &_a );                                                                      \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

// 将创建操作及其返回 id 录进一条分帧记录。id 追加在参数之后，以便回放断言匹配。
// 放在真正的创建调用之后。
#define B2_REC_CREATE( world, Name, id, ... )                                                                                    \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( world )->recording != NULL )                                                                                      \
		{                                                                                                                        \
			b2RecArgs_##Name _ca = { __VA_ARGS__ };                                                                              \
			b2RecWriteRet_##Name( ( world )->recording, &_ca, id );                                                              \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )


void b2StartRecordingIntoBuffer( b2World* world, b2Recording* recording );
void b2StopRecordingInternal( b2World* world );
void b2RecAccumulateBounds( b2Recording* rec, b2AABB bounds );

// 查询命中计数回填的补丁辅助函数
int b2RecReserveU32( b2RecBuffer* buf );
void b2RecPatchU32( b2RecBuffer* buf, int offset, uint32_t v );

// 在锁下提交已完成的查询记录。本地缓冲区仍由调用者持有。
void b2RecCommitRecord( b2Recording* rec, uint8_t opcode, const uint8_t* payload, int payloadSize );

// 每次查询的写入上下文：持有用户 fcn+ctx、本地载荷缓冲区和命中计数器
typedef struct b2RecQueryWriter
{
	union
	{
		b2OverlapResultCallbackFcn* overlapFcn;
		b2CastResultCallbackFcn* castFcn;
		b2PlaneResultCallbackFcn* planeFcn;
	} userFcn;
	void* userContext;
	b2RecBuffer buf; // 每次调用的本地载荷，堆分配
	int countOffset; // 预留的 u32 命中计数槽偏移
	uint32_t hitCount;
} b2RecQueryWriter;

void b2RecQueryBegin( b2RecQueryWriter* w, void* context );
void b2RecQueryCommit( b2Recording* rec, uint8_t opcode, b2RecQueryWriter* w );

// 录制跳板：替换用户 fcn 指针，以便在分派前捕获命中
bool b2RecOverlapTrampoline( b2ShapeId id, void* ctx );
float b2RecCastTrampoline( b2ShapeId id, b2Pos point, b2Vec2 normal, float fraction, void* ctx );
bool b2RecPlaneTrampoline( b2ShapeId id, const b2PlaneResult* plane, void* ctx );

// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once
#include "recording.h"

typedef struct b2Replay b2Replay;

typedef struct b2RecRecordedHit
{
	b2ShapeId id;
	b2Pos point;
	b2Vec2 normal;
	float fraction;
	b2PlaneResult plane;
	float userReturnF;
	bool userReturnB;
} b2RecRecordedHit;

typedef enum b2RecQueryKind
{
	B2_RECQ_OVERLAP_AABB,
	B2_RECQ_OVERLAP_SHAPE,
	B2_RECQ_CAST_RAY,
	B2_RECQ_CAST_SHAPE,
	B2_RECQ_COLLIDE_MOVER,
	B2_RECQ_CAST_RAY_CLOSEST,
	B2_RECQ_CAST_MOVER
} b2RecQueryKind;

typedef struct b2RecDrawQuery
{
	int kind;
	b2QueryFilter filter;
	b2AABB aabb;
	b2ShapeProxy proxy;
	b2Capsule mover;
	b2Pos origin;
	b2Vec2 translation;
	float castFraction;
	int hitStart;
	int hitCount;
} b2RecDrawQuery;


typedef struct b2RecReader
{
	const uint8_t* data;
	int size;
	int cursor;
	b2WorldId replayWorldId; // 回放期间创建的世界；CreateWorld 记录之后有效
	int workerCount;		 // 0 = 使用录制的数量
	bool ok;				 // 读取越界或 id 不匹配时为 false，致命停止
	bool diverged;			 // StateHash 未能复现，非致命，以便查看器可继续播放

	// 变长定义（链条点/材质）的暂存，按需增长，随播放器释放。
	// 由创建调用克隆，因此仅在一次分派期间有效。
	b2Vec2* chainPoints;
	int chainPointCap;
	b2SurfaceMaterial* chainMaterials;
	int chainMaterialCap;

	// 已录制查询命中的暂存；按需增长，随播放器释放
	b2RecRecordedHit* hits;
	int hitCap;

	b2Replay* owner; // 持有此读取器的播放器
} b2RecReader;

typedef struct b2RecKeyframe
{
	uint8_t* image; // 本帧结束时序列化的世界
	int imageSize;
	int imageCapacity; // image 的分配大小，会超额分配，因此释放大小与之匹配
	int frame;		   // 此恢复点对应的帧，为步后边界
	int cursor;		   // 下一帧恢复处的操作流偏移
	b2BodyId* bodyIds; // 截至本帧的大纲列表
	int bodyIdCount;
	int divergeFrame; // 截至本帧锁存的分叉，使寻址能报告线性路径的状态
	bool diverged;
} b2RecKeyframe;


struct b2Replay
{
    uint8_t* data;
    int size, headerEnd;
    float lengthScale, previousLengthScale;
    int frame, frameCount, recordedWorkerCount;
    float recordedDt;
    int recordedSubStepCount;
    b2AABB bounds;
    int divergeFrame;
    bool atEnd;
    b2RecReader rdr;
    b2RecDrawQuery* frameQueries; int frameQueryCount, frameQueryCap;
    b2RecRecordedHit* frameHits; int frameHitCount, frameHitCap;
    const uint8_t* frame0Image;
    int frame0Size;
	// 按创建顺序的活动刚体，由创建/销毁操作跟踪以驱动查看器大纲。
	// 已销毁槽持有 b2_nullBodyId，使序号保持稳定。回放时确定性重建。
	b2BodyId* bodyIds;
	int bodyIdCount;
	int bodyIdCap;

	// 第 0 帧映像，用于原地重启，使回放世界 id 在重启或向后拖动时保持稳定。
	// 指向所持副本内的种子快照数据块。
	b2BodyId* frame0BodyIds;
	int frame0BodyIdCount;

	// 用于快速向后寻址的关键帧环。随回放正向播放按递增帧序捕获。
	// 一旦触及内存预算，间距加倍并驱逐非网格关键帧，使内存保持有界，
	// 寻址代价仅在录制超出预算后才增长。
	b2RecKeyframe* keyframes;
	int keyframeCount;
	int keyframeCapacity;
	size_t keyframeBudget;	 // 保留快照的内存上限（字节）
	size_t keyframeBytes;	 // 已保留快照 + 刚体列表字节的累计
	int keyframeMinInterval; // 最细帧间距
	int keyframeInterval;	 // 当前间距，最小间距的 2 的幂倍，驱逐时加倍
	int lastKeyframeFrame;	 // 已捕获的最高帧，防止回退步进时重复捕获

};

uint8_t b2RecR_U8( b2RecReader* rdr );
uint16_t b2RecR_U16( b2RecReader* rdr );
uint32_t b2RecR_U24( b2RecReader* rdr );
uint32_t b2RecR_U32( b2RecReader* rdr );
uint64_t b2RecR_U64( b2RecReader* rdr );
int32_t b2RecR_I32( b2RecReader* rdr );
float b2RecR_F32( b2RecReader* rdr );
bool b2RecR_BOOL( b2RecReader* rdr );
b2Vec2 b2RecR_VEC2( b2RecReader* rdr );
b2Rot b2RecR_ROT( b2RecReader* rdr );
b2Transform b2RecR_XF( b2RecReader* rdr );
double b2RecR_F64( b2RecReader* rdr );
b2Pos b2RecR_POSITION( b2RecReader* rdr );
b2WorldTransform b2RecR_WORLDXF( b2RecReader* rdr );
b2WorldId b2RecR_WORLDID( b2RecReader* rdr );
b2BodyId b2RecR_BODYID( b2RecReader* rdr );
b2ShapeId b2RecR_SHAPEID( b2RecReader* rdr );
b2ChainId b2RecR_CHAINID( b2RecReader* rdr );
b2JointId b2RecR_JOINTID( b2RecReader* rdr );
b2Circle b2RecR_CIRCLE( b2RecReader* rdr );
b2Capsule b2RecR_CAPSULE( b2RecReader* rdr );
b2Segment b2RecR_SEGMENT( b2RecReader* rdr );
b2Polygon b2RecR_POLYGON( b2RecReader* rdr );
b2ChainSegment b2RecR_CHAINSEG( b2RecReader* rdr );
b2Filter b2RecR_FILTER( b2RecReader* rdr );
b2SurfaceMaterial b2RecR_MATERIAL( b2RecReader* rdr );
b2MassData b2RecR_MASSDATA( b2RecReader* rdr );
b2MotionLocks b2RecR_LOCKS( b2RecReader* rdr );
const char* b2RecR_STR( b2RecReader* rdr );
b2ExplosionDef b2RecR_EXPLOSIONDEF( b2RecReader* rdr );
b2BodyDef b2RecR_BODYDEF( b2RecReader* rdr );
b2ShapeDef b2RecR_SHAPEDEF( b2RecReader* rdr );
b2ChainDef b2RecR_CHAINDEF( b2RecReader* rdr );
b2DistanceJointDef b2RecR_DISTANCEJOINTDEF( b2RecReader* rdr );
b2FilterJointDef b2RecR_FILTERJOINTDEF( b2RecReader* rdr );
b2MotorJointDef b2RecR_MOTORJOINTDEF( b2RecReader* rdr );
b2MoverJointDef b2RecR_MOVERJOINTDEF( b2RecReader* rdr );
b2PogoJointDef b2RecR_POGOJOINTDEF( b2RecReader* rdr );
b2PrismaticJointDef b2RecR_PRISMATICJOINTDEF( b2RecReader* rdr );
b2RevoluteJointDef b2RecR_REVOLUTEJOINTDEF( b2RecReader* rdr );
b2WeldJointDef b2RecR_WELDJOINTDEF( b2RecReader* rdr );
b2WheelJointDef b2RecR_WHEELJOINTDEF( b2RecReader* rdr );
b2AABB b2RecR_AABB( b2RecReader* rdr );
b2QueryFilter b2RecR_QUERYFILTER( b2RecReader* rdr );
b2ShapeProxy b2RecR_SHAPEPROXY( b2RecReader* rdr );
b2RayResult b2RecR_RAYRESULT( b2RecReader* rdr );
b2PlaneResult b2RecR_PLANERESULT( b2RecReader* rdr );
b2TreeStats b2RecR_TREESTATS( b2RecReader* rdr );

// 将读取器的命中暂存扩到至少 n 项
void b2RecEnsureHits( b2RecReader* rdr, int n );

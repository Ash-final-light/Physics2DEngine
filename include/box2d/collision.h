// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "math_types.h"

/// 凸多边形最多使用的顶点数。
#define B2_MAX_POLYGON_VERTICES 8

/// 射线检测输入。translation 是从 origin 出发的整段位移，maxFraction 限制实际考虑的比例。
typedef struct b2RayCastInput
{
	b2Vec2 origin;
	b2Vec2 translation;
	float maxFraction;
} b2RayCastInput;

/// 射线检测输出。未命中时 hit 为 false，其余字段为零。
typedef struct b2CastOutput
{
	b2Vec2 normal;
	b2Vec2 point;
	float fraction;
	int iterations;
	bool hit;
} b2CastOutput;

/// 形状的质量数据。
typedef struct b2MassData
{
	float mass;
	b2Vec2 center;
	float rotationalInertia;
} b2MassData;

/// 实心圆。
typedef struct b2Circle
{
	b2Vec2 center;
	float radius;
} b2Circle;

/// 具有双侧碰撞的线段。
typedef struct b2Segment
{
	b2Vec2 point1;
	b2Vec2 point2;
} b2Segment;

/// 带相邻幽灵顶点的链段。碰撞只发生在线段右侧。
typedef struct b2ChainSegment
{
	b2Vec2 ghost1;
	b2Segment segment;
	b2Vec2 ghost2;
	int chainId;
} b2ChainSegment;

/// 实心胶囊，由两个半圆和中间矩形组成。
typedef struct b2Capsule
{
	b2Vec2 center1;
	b2Vec2 center2;
	float radius;
} b2Capsule;

/// 凸包。点按逆时针排列。
typedef struct b2Hull
{
	b2Vec2 points[B2_MAX_POLYGON_VERTICES];
	int count;
} b2Hull;

/// 计算点集的凸包。点数不足、超过 8 个、全部过近或共线时返回空凸包。
B2_API b2Hull b2ComputeHull( const b2Vec2* points, int count );

/// 检查凸包是否严格凸、逆时针，且没有过近的共线点。
B2_API bool b2ValidateHull( const b2Hull* hull );

/// 实心凸多边形。顶点按逆时针排列，法线指向外侧。
typedef struct b2Polygon
{
	b2Vec2 vertices[B2_MAX_POLYGON_VERTICES];
	b2Vec2 normals[B2_MAX_POLYGON_VERTICES];
	b2Vec2 centroid;
	float radius;
	int count;
} b2Polygon;

B2_API b2Polygon b2MakePolygon( const b2Hull* hull, float radius );
B2_API b2Polygon b2MakeOffsetPolygon( const b2Hull* hull, b2Vec2 position, b2Rot rotation );
B2_API b2Polygon b2MakeOffsetRoundedPolygon( const b2Hull* hull, b2Vec2 position, b2Rot rotation, float radius );
B2_API b2Polygon b2MakeBox( float halfWidth, float halfHeight );
B2_API b2Polygon b2MakeSquare( float halfWidth );
B2_API b2Polygon b2MakeRoundedBox( float halfWidth, float halfHeight, float radius );
B2_API b2Polygon b2MakeOffsetBox( float halfWidth, float halfHeight, b2Vec2 center, b2Rot rotation );
B2_API b2Polygon b2MakeOffsetRoundedBox( float halfWidth, float halfHeight, b2Vec2 center, b2Rot rotation, float radius );
B2_API b2MassData b2ComputeCircleMass( const b2Circle* shape, float density );
B2_API b2MassData b2ComputeCapsuleMass( const b2Capsule* shape, float density );
B2_API b2MassData b2ComputePolygonMass( const b2Polygon* shape, float density );
B2_API b2AABB b2ComputeCircleBounds( const b2Circle* shape, b2WorldTransform transform );
B2_API b2AABB b2ComputeCapsuleBounds( const b2Capsule* shape, b2WorldTransform transform );
B2_API b2AABB b2ComputePolygonBounds( const b2Polygon* shape, b2WorldTransform transform );
B2_API b2AABB b2ComputeSegmentBounds( const b2Segment* shape, b2WorldTransform transform );
B2_API bool b2PointInCircle( const b2Circle* shape, b2Vec2 point );
B2_API bool b2PointInCapsule( const b2Capsule* shape, b2Vec2 point );
B2_API bool b2PointInPolygon( const b2Polygon* shape, b2Vec2 point );
B2_API bool b2IsValidRay( const b2RayCastInput* input );
B2_API b2CastOutput b2CastRayCircle( const b2Circle* shape, const b2RayCastInput* input );
B2_API b2CastOutput b2CastRayCapsule( const b2Capsule* shape, const b2RayCastInput* input );
B2_API b2CastOutput b2CastRaySegment( const b2Segment* shape, const b2RayCastInput* input, bool oneSided );
B2_API b2CastOutput b2CastRayPolygon( const b2Polygon* shape, const b2RayCastInput* input );

typedef struct b2SegmentDistanceResult
{
	b2Vec2 closest1;
	b2Vec2 closest2;
	float fraction1;
	float fraction2;
	float distanceSquared;
} b2SegmentDistanceResult;

B2_API b2SegmentDistanceResult b2ComputeSegmentDistance( b2Vec2 p1, b2Vec2 q1, b2Vec2 p2, b2Vec2 q2 );

/// 距离查询使用的点云。点数为 1 到 B2_MAX_POLYGON_VERTICES，半径可以为 0。
typedef struct b2ShapeProxy
{
	b2Vec2 points[B2_MAX_POLYGON_VERTICES];
	int count;
	float radius;
} b2ShapeProxy;

B2_API b2ShapeProxy b2MakeProxy( const b2Vec2* points, int count, float radius );
B2_API b2ShapeProxy b2MakeOffsetProxy( const b2Vec2* points, int count, float radius, b2Vec2 position, b2Rot rotation );

typedef struct b2SimplexCache
{
	uint16_t count;
	uint8_t indexA[3];
	uint8_t indexB[3];
} b2SimplexCache;

static const b2SimplexCache b2_emptySimplexCache = B2_ZERO_INIT;

typedef struct b2DistanceInput
{
	b2ShapeProxy proxyA;
	b2ShapeProxy proxyB;
	b2Transform transform;
	bool useRadii;
} b2DistanceInput;

typedef struct b2DistanceOutput
{
	b2Vec2 pointA;
	b2Vec2 pointB;
	b2Vec2 normal;
	float distance;
	int iterations;
	int simplexCount;
} b2DistanceOutput;

typedef struct b2SimplexVertex
{
	b2Vec2 wA;
	b2Vec2 wB;
	b2Vec2 w;
	float a;
	int indexA;
	int indexB;
} b2SimplexVertex;

typedef struct b2Simplex
{
	b2SimplexVertex v1, v2, v3;
	int count;
} b2Simplex;

B2_API b2DistanceOutput b2ComputeShapeDistance( const b2DistanceInput* input, b2SimplexCache* cache, b2Simplex* simplexes, int simplexCapacity );

typedef struct b2ShapeCastPairInput
{
	b2ShapeProxy proxyA;
	b2ShapeProxy proxyB;
	b2Transform transform;
	b2Vec2 translationB;
	float maxFraction;
	bool canEncroach;
} b2ShapeCastPairInput;

B2_API b2CastOutput b2CastShape( const b2ShapeCastPairInput* input );

typedef struct b2ShapeCastInput
{
	b2ShapeProxy proxy;
	b2Vec2 translation;
	float maxFraction;
	bool canEncroach;
} b2ShapeCastInput;

B2_API b2CastOutput b2CastShapeCircle( const b2Circle* shape, const b2ShapeCastInput* input );
B2_API b2CastOutput b2CastShapeCapsule( const b2Capsule* shape, const b2ShapeCastInput* input );
B2_API b2CastOutput b2CastShapeSegment( const b2Segment* shape, const b2ShapeCastInput* input );
B2_API b2CastOutput b2CastShapePolygon( const b2Polygon* shape, const b2ShapeCastInput* input );

typedef struct b2Sweep
{
	b2Vec2 localCenter;
	b2Vec2 c1;
	b2Vec2 c2;
	b2Rot q1;
	b2Rot q2;
} b2Sweep;

B2_API b2Transform b2GetSweepTransform( const b2Sweep* sweep, float time );

typedef struct b2TOIInput
{
	b2ShapeProxy proxyA;
	b2ShapeProxy proxyB;
	b2Sweep sweepA;
	b2Sweep sweepB;
	float maxFraction;
} b2TOIInput;

typedef enum b2TOIState
{
	b2_toiStateUnknown,
	b2_toiStateFailed,
	b2_toiStateOverlapped,
	b2_toiStateHit,
	b2_toiStateSeparated
} b2TOIState;

typedef struct b2TOIOutput
{
	b2TOIState state;
	b2Vec2 point;
	b2Vec2 normal;
	float fraction;
} b2TOIOutput;

B2_API b2TOIOutput b2ComputeTimeOfImpact( const b2TOIInput* input );

typedef struct b2ManifoldPoint
{
	/// 接触点相对刚体 A 质心在世界空间中的位置。可使用以下方式转换为世界点：
	/// b2Pos worldPointA = b2OffsetPos(b2Body_GetWorldCenter(myBodyIdA), anchorA);
	b2Vec2 anchorA;

	/// 刚体 B 类似。
	b2Vec2 anchorB;

	/// 接触点的分离量，穿透时为负
	float separation;

	/// 沿流形法向量的冲量。
	float normalImpulse;

	/// 摩擦冲量
	float tangentImpulse;

	/// 跨子步与恢复系数施加的法向冲量总和。这对于识别在该时间步发生相互作用的
	/// 预测性接触点很重要。包括热启动冲量、子步增量冲量以及恢复冲量。
	float totalNormalImpulse;

	/// 求解前的相对法向速度。接近时为负。仅在启用命中事件时计算。
	/// 作为 SIMD 打包的副作用也可能被计算，但不保证。
	float normalVelocity;

	/// 用于接触回收的缓存分离量
	float baseSeparation;

	/// 唯一标识两个形状之间的接触点
	uint16_t id;

	/// 该接触点在上一时间步是否存在？
	bool persisted;
} b2ManifoldPoint;

/// 接触流形描述碰撞形状之间的接触点。
/// @note Box2D 使用预测性碰撞，因此部分接触点可能是分离的。
typedef struct b2Manifold
{
	/// 世界空间中的单位法向量，从形状 A 指向形状 B。
	b2Vec2 normal;

	/// 为滚动阻力施加的角冲量。N * m * s = kg * m^2 / s
	float rollingImpulse;

	/// 流形点，二维中最多两个。
	b2ManifoldPoint points[2];

	/// 接触点数，为 0、1 或 2。
	int pointCount;

} b2Manifold;


typedef struct b2LocalManifoldPoint
{
	b2Vec2 point;
	float separation;
	uint16_t id;
} b2LocalManifoldPoint;

typedef struct b2LocalManifold
{
	b2Vec2 normal;
	b2LocalManifoldPoint points[2];
	int pointCount;
} b2LocalManifold;

B2_API b2LocalManifold b2CollideCircles( const b2Circle* circleA, const b2Circle* circleB, b2Transform xf );
B2_API b2LocalManifold b2CollideCapsuleAndCircle( const b2Capsule* capsuleA, const b2Circle* circleB, b2Transform xf );
B2_API b2LocalManifold b2CollidePolygonAndCircle( const b2Polygon* polygonA, const b2Circle* circleB, b2Transform xf );
B2_API b2LocalManifold b2CollideCapsules( const b2Capsule* capsuleA, const b2Capsule* capsuleB, b2Transform xf );
B2_API b2LocalManifold b2CollideSegmentAndCircle( const b2Segment* segmentA, const b2Circle* circleB, b2Transform xf );
B2_API b2LocalManifold b2CollideSegmentAndCapsule( const b2Segment* segmentA, const b2Capsule* capsuleB, b2Transform xf );
B2_API b2LocalManifold b2CollidePolygonAndCapsule( const b2Polygon* polygonA, const b2Capsule* capsuleB, b2Transform xf );
B2_API b2LocalManifold b2CollidePolygons( const b2Polygon* polygonA, const b2Polygon* polygonB, b2Transform xf );
B2_API b2LocalManifold b2CollideSegmentAndPolygon( const b2Segment* segmentA, const b2Polygon* polygonB, b2Transform xf );
B2_API b2LocalManifold b2CollideChainSegmentAndCircle( const b2ChainSegment* segmentA, const b2Circle* circleB, b2Transform xf );
B2_API b2LocalManifold b2CollideChainSegmentAndCapsule( const b2ChainSegment* segmentA, const b2Capsule* capsuleB, b2Transform xf,
													   b2SimplexCache* cache );
B2_API b2LocalManifold b2CollideChainSegmentAndPolygon( const b2ChainSegment* segmentA, const b2Polygon* polygonB, b2Transform xf,
														b2SimplexCache* cache );

typedef enum b2ChainStatus
{
	b2_chainStatusSuccess,
	b2_chainStatusCornerTouch,
	b2_chainStatusOverlap,
} b2ChainStatus;

typedef struct b2ChainData
{
	b2Vec2* points;
	int count;
} b2ChainData;

typedef struct b2ChainResult
{
	b2ChainData* chains;
	int chainCount;
	b2ChainStatus status;
} b2ChainResult;

B2_API b2ChainResult b2ComputeChains( const b2Polygon* polygons, int polygonCount, float weldTolerance );
B2_API void b2FreeChains( b2ChainResult* result );

typedef struct b2DynamicTree b2DynamicTree;

B2_API b2DynamicTree* b2CreateDynamicTree( int proxyCapacity );
B2_API void b2DestroyDynamicTree( b2DynamicTree* tree );
B2_API int b2CreateTreeProxy( b2DynamicTree* tree, b2AABB aabb, uint64_t categoryBits, uint64_t userData );
B2_API void b2DestroyTreeProxy( b2DynamicTree* tree, int proxyId );
B2_API void b2DynamicTree_MoveProxy( b2DynamicTree* tree, int proxyId, b2AABB aabb );
B2_API void b2DynamicTree_EnlargeProxy( b2DynamicTree* tree, int proxyId, b2AABB aabb );
B2_API void b2DynamicTree_SetCategoryBits( b2DynamicTree* tree, int proxyId, uint64_t categoryBits );
B2_API uint64_t b2DynamicTree_GetCategoryBits( b2DynamicTree* tree, int proxyId );
B2_API int b2DynamicTree_Rebuild( b2DynamicTree* tree, bool fullBuild );
B2_API void b2DynamicTree_Validate( const b2DynamicTree* tree );
B2_API void b2DynamicTree_MarkProxyMoved( b2DynamicTree* tree, int proxyId, b2AABB aabb );
B2_API int b2DynamicTree_GatherMovedProxies( const b2DynamicTree* tree, int* proxyIds );
B2_API void b2DynamicTree_Refit( b2DynamicTree* tree );

typedef struct b2TreeStats
{
	int nodeVisits;
	int leafVisits;
} b2TreeStats;

typedef bool b2TreeQueryCallbackFcn( int proxyId, uint64_t userData, void* context );

B2_API b2TreeStats b2DynamicTree_OverlapAABB( const b2DynamicTree* tree, b2AABB aabb, uint64_t maskBits,
											 b2TreeQueryCallbackFcn* callback, void* context );
B2_API b2TreeStats b2DynamicTree_OverlapAABBAll( const b2DynamicTree* tree, b2AABB aabb, b2TreeQueryCallbackFcn* callback,
												 void* context );

typedef float b2TreeRayCastCallbackFcn( const b2RayCastInput* input, int proxyId, uint64_t userData, void* context );

B2_API b2TreeStats b2DynamicTree_CastRay( const b2DynamicTree* tree, const b2RayCastInput* input, uint64_t maskBits,
										  b2TreeRayCastCallbackFcn* callback, void* context );

typedef struct b2BoxCastInput
{
	b2AABB box;
	b2Vec2 translation;
	float maxFraction;
} b2BoxCastInput;

typedef float b2TreeBoxCastCallbackFcn( const b2BoxCastInput* input, int proxyId, uint64_t userData, void* context );

B2_API b2TreeStats b2DynamicTree_CastBox( const b2DynamicTree* tree, const b2BoxCastInput* input, uint64_t maskBits,
										  b2TreeBoxCastCallbackFcn* callback, void* context );
B2_API int b2DynamicTree_GetHeight( const b2DynamicTree* tree );
B2_API float b2DynamicTree_GetAreaRatio( const b2DynamicTree* tree );
B2_API b2AABB b2DynamicTree_GetRootBounds( const b2DynamicTree* tree );
B2_API size_t b2DynamicTree_GetByteCount( const b2DynamicTree* tree );
B2_API int b2DynamicTree_GetProxyCount( const b2DynamicTree* tree );
B2_API uint64_t b2DynamicTree_GetUserData( const b2DynamicTree* tree, int proxyId );
B2_API b2AABB b2DynamicTree_GetProxyBounds( const b2DynamicTree* tree, int proxyId );

/// 当前单精度世界中的投射结果。
typedef b2CastOutput b2WorldCastOutput;

typedef struct b2CollisionPlane
{
	/// 移动器与某形状之间的碰撞平面。
	b2Plane plane;

	/// 设为 FLT_MAX 可使平面尽可能刚硬。较低值可使平面碰撞变软。通常以米为单位。
	float pushLimit;

	/// 由 b2SolvePlanes 确定的对移动器的推移。通常以米为单位。
	float push;

	/// 指示 b2ClipVector 是否应对此平面裁剪。软碰撞应为 false。
	bool clipVelocity;
} b2CollisionPlane;

/// b2SolvePlanes 返回的结果。
typedef struct b2PlaneSolverResult
{
	/// 最终相对平移。
	b2Vec2 delta;

	/// 平面求解器使用的迭代次数。用于诊断。
	int iterationCount;
} b2PlaneSolverResult;

/// 求解满足给定碰撞平面的移动器位置。
/// @param targetDelta 相对用于生成碰撞平面的位置的期望平移
/// @param planes 碰撞平面
/// @param count 碰撞平面数量
B2_API b2PlaneSolverResult b2SolvePlanes( b2Vec2 targetDelta, b2CollisionPlane* planes, int count );

B2_API b2Vec2 b2ClipVector(b2Vec2 vector,const b2CollisionPlane* planes,int count);

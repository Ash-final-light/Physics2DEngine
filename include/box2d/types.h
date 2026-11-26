// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "collision.h"
#include "id.h"
#include "math_types.h"

/// Box2D 任务函数原型：每次入队恰好调用一次，并收到同一个 taskContext 指针。
typedef void b2TaskCallbackFcn( void* taskContext );

typedef enum b2TaskType
{
	b2_taskPairs,
	b2_taskCollide,
	b2_taskTreeRebuild,
	b2_taskIslandSplit,
	b2_taskSolver,
	b2_taskFinalize,
	b2_taskContinuous,
	b2_taskSensors,
	b2_taskTypeCount
} b2TaskType;

typedef struct b2Task
{
	b2TaskCallbackFcn* fcn;
	void* context;
	b2TaskType type;
	bool deferred;
} b2Task;

typedef struct b2TaskBatch
{
	const b2Task* tasks;
	int count;
	const b2Task* joinTasks;
	int joinCount;
} b2TaskBatch;

typedef void* b2EnqueueTaskCallbackFcn( b2TaskCallbackFcn* task, void* taskContext, void* userContext );
typedef void b2FinishTaskCallbackFcn( void* userTask, void* userContext );

typedef void b2PreSolveCallbackFcn(b2ShapeId shapeIdA,b2ShapeId shapeIdB,b2Manifold* manifold,void* context);
typedef bool b2PreContinuousCallbackFcn(b2ShapeId shapeIdA,b2ShapeId shapeIdB,b2Pos point,b2Vec2 normal,void* context);
B2_API void b2World_SetPreSolveCallback(b2WorldId worldId,b2PreSolveCallbackFcn* preSolveFcn,b2PreContinuousCallbackFcn* preContinuousFcn,void* context);
B2_API void b2Shape_EnablePreSolveEvents(b2ShapeId shapeId,bool flag);
B2_API bool b2Shape_ArePreSolveEventsEnabled(b2ShapeId shapeId);

typedef bool b2CustomFilterCallbackFcn(b2ShapeId shapeIdA,b2ShapeId shapeIdB,void* context);
B2_API void b2World_SetCustomFilterCallback(b2WorldId worldId,b2CustomFilterCallbackFcn* fcn,void* context);

typedef float b2FrictionCallbackFcn( float frictionA, uint64_t userMaterialIdA, float frictionB, uint64_t userMaterialIdB );
typedef float b2RestitutionCallbackFcn( float restitutionA, uint64_t userMaterialIdA, float restitutionB, uint64_t userMaterialIdB );

typedef struct b2Capacity
{
	int staticShapeCount;
	int dynamicShapeCount;
	int staticBodyCount;
	int dynamicBodyCount;
	int contactCount;
	int jointCount;
	int arenaByteCount;
} b2Capacity;

typedef struct b2WorldDef
{
	b2Vec2 gravity;
	float restitutionThreshold;
	int restitutionIterations;
	bool enableRestitutionPropagation;
	float hitEventThreshold;
	float contactHertz;
	float contactDampingRatio;
	float contactSpeed;
	float maximumLinearSpeed;
	b2FrictionCallbackFcn* frictionCallback;
	b2RestitutionCallbackFcn* restitutionCallback;
	bool enableSleep;
	bool enableContinuous;
	int workerCount;
	b2EnqueueTaskCallbackFcn* enqueueTask;
	b2FinishTaskCallbackFcn* finishTask;
	void* userTaskContext;
	void* userData;
	b2Capacity capacity;
	int internalValue;
} b2WorldDef;

/// 设置工作线程数，范围 [1, B2_MAX_WORKERS]。
B2_API void b2World_SetWorkerCount( b2WorldId worldId, int count );
B2_API int b2World_GetWorkerCount( b2WorldId worldId );

/// 开始一次分阶段步进，返回当前阶段需要执行的任务批次。
B2_API b2TaskBatch b2World_BeginStep( b2WorldId worldId, float timeStep, int subStepCount );
/// 继续执行分阶段步进的收尾阶段。
B2_API b2TaskBatch b2World_ContinueStep( b2WorldId worldId );

B2_API b2WorldDef b2DefaultWorldDef( void );
B2_API b2WorldId b2CreateWorld( const b2WorldDef* def );
B2_API void b2DestroyWorld( b2WorldId worldId );
B2_API bool b2World_IsValid( b2WorldId id );

/// 计算仿真状态的深层哈希。用于检测不同步，例如在回滚重仿真中。
B2_API uint64_t b2World_GetStateHash( b2WorldId worldId );

/// 将世界仿真状态写入调用方缓冲区。image 为 NULL 时只返回所需大小。
/// 必须在步进边界调用；返回所需字节数，若处于步进中则返回 0。
B2_API int b2World_GetSnapshot( b2WorldId worldId, uint8_t* image, int capacity );

/// 从快照映像创建新世界。回调与用户数据重置为默认值。
/// 失败时返回无效世界 id。
B2_API b2WorldId b2CreateWorldFromSnapshot( const uint8_t* image, int size, int workerCount );

/// 将快照映像恢复到已有世界。世界 id、线程配置与回调保留；对象用户数据置空。
/// 头校验失败时世界不变；载荷损坏后世界不可用，调用方必须销毁它。
B2_API bool b2World_RestoreFromSnapshot( b2WorldId worldId, const uint8_t* image, int size );

B2_API void b2World_SetUserData( b2WorldId worldId, void* userData );
B2_API void* b2World_GetUserData( b2WorldId worldId );

typedef enum b2BodyType
{
	b2_staticBody = 0,
	b2_kinematicBody = 1,
	b2_dynamicBody = 2,
	b2_bodyTypeCount
} b2BodyType;

typedef struct b2MotionLocks
{
	bool linearX;
	bool linearY;
	bool angularZ;
} b2MotionLocks;

typedef struct b2BodyDef
{
	b2BodyType type;
	b2Pos position;
	b2Rot rotation;
	b2Vec2 linearVelocity;
	float angularVelocity;
	float linearDamping;
	float angularDamping;
	float gravityScale;
	float sleepThreshold;
	float safetyFactor;
	const char* name;
	void* userData;
	b2MotionLocks motionLocks;
	bool enableSleep;
	bool isAwake;
	bool isBullet;
	bool isEnabled;
	bool allowFastRotation;
	bool enableContactRecycling;
	int internalValue;
} b2BodyDef;

B2_API b2BodyDef b2DefaultBodyDef( void );
B2_API b2BodyId b2CreateBody( b2WorldId worldId, const b2BodyDef* def );
B2_API void b2DestroyBody( b2BodyId bodyId );
B2_API bool b2Body_IsValid( b2BodyId id );
B2_API void b2Body_SetName( b2BodyId bodyId, const char* name );
B2_API const char* b2Body_GetName( b2BodyId bodyId );
B2_API void b2Body_SetUserData( b2BodyId bodyId, void* userData );
B2_API void* b2Body_GetUserData( b2BodyId bodyId );

B2_API b2Pos b2Body_GetPosition( b2BodyId bodyId );
B2_API b2Rot b2Body_GetRotation( b2BodyId bodyId );
B2_API b2WorldTransform b2Body_GetTransform( b2BodyId bodyId );
B2_API b2Vec2 b2Body_GetLocalPoint( b2BodyId bodyId, b2Pos worldPoint );
B2_API b2Pos b2Body_GetWorldPoint( b2BodyId bodyId, b2Vec2 localPoint );
B2_API b2Vec2 b2Body_GetLocalVector( b2BodyId bodyId, b2Vec2 worldVector );
B2_API b2Vec2 b2Body_GetWorldVector( b2BodyId bodyId, b2Vec2 localVector );
B2_API void b2Body_SetTransform( b2BodyId bodyId, b2Pos position, b2Rot rotation );
B2_API b2AABB b2Body_ComputeBounds( b2BodyId bodyId );


typedef struct b2QueryFilter
{
	/// 此查询的碰撞类别位。通常只需设置一位。
	uint64_t categoryBits;

	/// 碰撞掩码位。这声明此查询会接受碰撞的形状类别。
	uint64_t maskBits;
} b2QueryFilter;
B2_API b2QueryFilter b2DefaultQueryFilter(void);
typedef float b2CastResultCallbackFcn(b2ShapeId shapeId,b2Pos point,b2Vec2 normal,float fraction,void* context);
B2_API b2TreeStats b2World_CastRay(b2WorldId worldId,b2Pos origin,b2Vec2 translation,b2QueryFilter filter,b2CastResultCallbackFcn* fcn,void* context);
typedef bool b2OverlapResultCallbackFcn(b2ShapeId shapeId,void* context);
B2_API b2TreeStats b2World_OverlapShape(b2WorldId worldId,b2Pos origin,const b2ShapeProxy* proxy,b2QueryFilter filter,b2OverlapResultCallbackFcn* fcn,void* context);
B2_API b2TreeStats b2World_OverlapAABB(b2WorldId worldId,b2Pos origin,b2AABB aabb,b2QueryFilter filter,b2OverlapResultCallbackFcn* fcn,void* context);
typedef struct b2Filter
{
    uint64_t categoryBits;
    uint64_t maskBits;
    int groupIndex;
} b2Filter;

typedef enum b2ShapeType
{
    b2_circleShape,
    b2_capsuleShape,
    b2_segmentShape,
    b2_polygonShape,
    b2_chainSegmentShape,
    b2_shapeTypeCount
} b2ShapeType;

typedef struct b2SurfaceMaterial
{
    float friction;
    float restitution;
    float rollingResistance;
    float tangentSpeed;
    uint64_t userMaterialId;
    uint32_t customColor;
} b2SurfaceMaterial;

typedef struct b2ShapeDef
{
    void* userData;
    b2SurfaceMaterial material;
    float density;
    b2Filter filter;
    bool enableCustomFiltering;
    bool isSensor;
    bool enableSensorEvents;
    bool enableContactEvents;
    bool enableHitEvents;
    bool enablePreSolveEvents;
    bool invokeContactCreation;
    bool updateBodyMass;
    int internalValue;
} b2ShapeDef;

B2_API b2Filter b2DefaultFilter( void );
B2_API b2SurfaceMaterial b2DefaultSurfaceMaterial( void );
B2_API b2ShapeDef b2DefaultShapeDef( void );
B2_API b2ShapeId b2CreateCircleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Circle* circle );
B2_API void b2DestroyShape( b2ShapeId shapeId, bool updateBodyMass );
B2_API bool b2Shape_IsValid( b2ShapeId shapeId );

B2_API b2ShapeId b2CreateCapsuleShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Capsule* capsule );

B2_API b2ShapeId b2CreatePolygonShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Polygon* polygon );

B2_API b2ShapeId b2CreateSegmentShape( b2BodyId bodyId, const b2ShapeDef* def, const b2Segment* segment );

B2_API void b2Shape_SetUserData( b2ShapeId shapeId, void* userData );
B2_API void* b2Shape_GetUserData( b2ShapeId shapeId );

B2_API b2Circle b2Shape_GetCircle( b2ShapeId shapeId );
B2_API void b2Shape_SetCircle( b2ShapeId shapeId, const b2Circle* circle );

B2_API b2Capsule b2Shape_GetCapsule( b2ShapeId shapeId );
B2_API void b2Shape_SetCapsule( b2ShapeId shapeId, const b2Capsule* capsule );

B2_API b2Polygon b2Shape_GetPolygon( b2ShapeId shapeId );
B2_API void b2Shape_SetPolygon( b2ShapeId shapeId, const b2Polygon* polygon );

B2_API b2Segment b2Shape_GetSegment( b2ShapeId shapeId );
B2_API void b2Shape_SetSegment( b2ShapeId shapeId, const b2Segment* segment );

B2_API b2ShapeType b2Shape_GetType( b2ShapeId shapeId );


B2_API b2MassData b2Body_GetMassData( b2BodyId bodyId );
B2_API float b2Body_GetMass( b2BodyId bodyId );
B2_API float b2Body_GetRotationalInertia( b2BodyId bodyId );
B2_API b2Vec2 b2Body_GetLocalCenter( b2BodyId bodyId );
B2_API b2Pos b2Body_GetWorldCenter( b2BodyId bodyId );
B2_API void b2Body_UpdateMassFromShapes( b2BodyId bodyId );
B2_API void b2Shape_SetDensity( b2ShapeId shapeId, float density, bool updateBodyMass );
B2_API float b2Shape_GetDensity( b2ShapeId shapeId );
B2_API b2MassData b2Shape_ComputeMassData( b2ShapeId shapeId );

B2_API void b2Body_SetMassData( b2BodyId bodyId, b2MassData massData );


typedef struct b2ChainDef
{
    void* userData;
    const b2Vec2* points;
    int pointCount;
    b2Vec2 ghost1;
    b2Vec2 ghost2;
    const b2SurfaceMaterial* materials;
    int materialCount;
    b2Filter filter;
    bool isLoop;
    bool enableSensorEvents;
    int internalValue;
} b2ChainDef;

B2_API b2ChainDef b2DefaultChainDef( void );
B2_API b2ChainId b2CreateChain( b2BodyId bodyId, const b2ChainDef* def );
B2_API void b2DestroyChain( b2ChainId chainId );
B2_API bool b2Chain_IsValid( b2ChainId chainId );


B2_API b2WorldId b2Chain_GetWorld( b2ChainId chainId );
B2_API int b2Chain_GetSegmentCount( b2ChainId chainId );
B2_API int b2Chain_GetSegments( b2ChainId chainId, b2ShapeId* segmentArray, int capacity );
B2_API b2ChainSegment b2Shape_GetChainSegment( b2ShapeId shapeId );
B2_API b2ChainId b2Shape_GetParentChain( b2ShapeId shapeId );


B2_API void b2Chain_SetSurfaceMaterial( b2ChainId chainId, const b2SurfaceMaterial* material, int segmentIndex );
B2_API void b2Chain_SetAllSurfaceMaterials( b2ChainId chainId, const b2SurfaceMaterial* material );
B2_API b2SurfaceMaterial b2Chain_GetSurfaceMaterial( b2ChainId chainId, int segmentIndex );


B2_API b2WorldId b2Body_GetWorld( b2BodyId bodyId );
B2_API int b2Body_GetShapeCount( b2BodyId bodyId );
B2_API int b2Body_GetShapes( b2BodyId bodyId, b2ShapeId* shapeArray, int capacity );
B2_API b2BodyId b2Shape_GetBody( b2ShapeId shapeId );
B2_API b2WorldId b2Shape_GetWorld( b2ShapeId shapeId );

B2_API b2Filter b2Shape_GetFilter( b2ShapeId shapeId );
B2_API void b2Shape_SetFilter( b2ShapeId shapeId, b2Filter filter );

typedef struct b2SensorBeginTouchEvent
{
	/// 传感器形状的 id
	b2ShapeId sensorShapeId;

	/// 开始触摸传感器形状的形状 id
	b2ShapeId visitorShapeId;
} b2SensorBeginTouchEvent;

/// 当形状停止与传感器形状重叠时生成结束触摸事件。通常此事件由仿真运动生成。
/// 但也可由 API 交互生成：设置刚体变换、销毁刚体或形状，或更改过滤器。
/// 若传感器被销毁，你也会得到结束事件。因此应始终使用 b2Shape_IsValid 确认形状 id 有效。
typedef struct b2SensorEndTouchEvent
{
	/// 传感器形状的 id
	/// @warning 此形状可能已被销毁
	/// @see b2Shape_IsValid
	b2ShapeId sensorShapeId;

	/// 停止触摸传感器形状的形状 id
	/// @warning 此形状可能已被销毁
	/// @see b2Shape_IsValid
	b2ShapeId visitorShapeId;

} b2SensorEndTouchEvent;

/// 传感器事件在世界中缓冲，并在时间步完成后作为开始/结束重叠事件数组可用。
/// 注意：若刚体和/或形状被销毁，这些可能变得无效
typedef struct b2SensorEvents
{
	/// 传感器开始触摸事件数组
	b2SensorBeginTouchEvent* beginEvents;

	/// 传感器结束触摸事件数组
	b2SensorEndTouchEvent* endEvents;

	/// 开始触摸事件数量
	int beginCount;

	/// 结束触摸事件数量
	int endCount;
} b2SensorEvents;
B2_API b2SensorEvents b2World_GetSensorEvents(b2WorldId worldId);
B2_API bool b2Shape_IsSensor(b2ShapeId shapeId);
B2_API void b2Shape_EnableSensorEvents(b2ShapeId shapeId,bool flag);
B2_API bool b2Shape_AreSensorEventsEnabled(b2ShapeId shapeId);
B2_API int b2Shape_GetSensorCapacity(b2ShapeId shapeId);
B2_API int b2Shape_GetSensorData(b2ShapeId shapeId,b2ShapeId* visitorIds,int capacity);
typedef struct b2ContactBeginTouchEvent
{
	/// 第一个形状的 id
	b2ShapeId shapeIdA;

	/// 第二个形状的 id
	b2ShapeId shapeIdB;

	/// 瞬时接触 id。当世界被修改或仿真时，此接触可能被自动销毁。
	/// 使用此前请用 b2Contact_IsValid 校验。
	b2ContactId contactId;
} b2ContactBeginTouchEvent;

/// 当两个形状停止接触时生成结束触摸事件。若你在上一世界步进之前做了任何销毁接触的事，
/// 也会得到结束事件。这些包括设置变换、销毁刚体或形状，或更改过滤器或刚体类型。
typedef struct b2ContactEndTouchEvent
{
	/// 第一个形状的 id
	/// @warning 此形状可能已被销毁
	/// @see b2Shape_IsValid
	b2ShapeId shapeIdA;

	/// 第二个形状的 id
	/// @warning 此形状可能已被销毁
	/// @see b2Shape_IsValid
	b2ShapeId shapeIdB;

	/// 接触的 id。
	/// @warning 此接触可能已被销毁
	/// @see b2Contact_IsValid
	b2ContactId contactId;
} b2ContactEndTouchEvent;
typedef struct b2ContactHitEvent
{
	/// 第一个形状的 id
	b2ShapeId shapeIdA;

	/// 第二个形状的 id
	b2ShapeId shapeIdB;

	/// 接触的 id。
	/// @warning 此接触可能已被销毁
	/// @see b2Contact_IsValid
	b2ContactId contactId;

	/// 形状命中的点。这是两个表面之间的中点。它可能是预测点，形状在时间步开始时并未接触。
	/// 该点在步进后重建，因此当两个刚体都在移动时，它可能落后撞击最多一个步进的运动。
	b2Pos point;

	/// 从形状 A 指向形状 B 的法向量
	b2Vec2 normal;

	/// 形状接近的速度。始终为正。通常以米每秒为单位。
	float approachSpeed;
} b2ContactHitEvent;
typedef struct b2ContactEvents
{
    b2ContactBeginTouchEvent* beginEvents;
    b2ContactEndTouchEvent* endEvents;
    b2ContactHitEvent* hitEvents;
    int beginCount;
    int endCount;
    int hitCount;
} b2ContactEvents;
B2_API b2ContactEvents b2World_GetContactEvents(b2WorldId worldId);
B2_API void b2Shape_EnableContactEvents(b2ShapeId shapeId,bool flag);
B2_API bool b2Shape_AreContactEventsEnabled(b2ShapeId shapeId);
B2_API void b2Body_EnableContactEvents(b2BodyId bodyId,bool flag);
B2_API void b2Shape_EnableHitEvents(b2ShapeId shapeId,bool flag);
B2_API bool b2Shape_AreHitEventsEnabled(b2ShapeId shapeId);
B2_API void b2Body_EnableHitEvents(b2BodyId bodyId,bool flag);
B2_API void b2World_SetHitEventThreshold(b2WorldId worldId,float value);
B2_API float b2World_GetHitEventThreshold(b2WorldId worldId);
typedef struct b2BodyMoveEvent
{
	/// 刚体用户数据。
	void* userData;

	/// 刚体的世界变换。
	b2WorldTransform transform;

	/// 移动的刚体的 id。
	b2BodyId bodyId;

	/// 此刚体是否在本时间步进入休眠？
	bool fellAsleep;
} b2BodyMoveEvent;

/// 刚体事件在 Box2D 世界中缓冲，并在时间步完成后作为事件数组可用。
/// 注意：若刚体被销毁，此数据变得无效
typedef struct b2BodyEvents
{
	/// 移动事件数组
	b2BodyMoveEvent* moveEvents;

	/// 移动事件数量
	int moveCount;
} b2BodyEvents;
B2_API b2BodyEvents b2World_GetBodyEvents(b2WorldId worldId);

/// 关节事件报告醒着且力与/或力矩超过阈值的关节。出于效率原因不返回观测到的力与力矩。
typedef struct b2JointEvent
{
	/// 关节 id
	b2JointId jointId;

	/// 来自关节的用户数据，为方便起见
	void* userData;
} b2JointEvent;

/// 关节事件在世界中缓冲，并在时间步完成后作为事件数组可用。
/// 注意：若关节被销毁，此数据变得无效
typedef struct b2JointEvents
{
	/// 事件数组
	b2JointEvent* jointEvents;

	/// 事件数量
	int count;
} b2JointEvents;
B2_API b2JointEvents b2World_GetJointEvents(b2WorldId worldId);

typedef struct b2Profile
{
	float step;				   ///<
	float pairs;			   ///<
	float collide;			   ///<
	float solve;			   ///<
	float solverSetup;		   ///<
	float constraints;		   ///<
	float prepareConstraints;  ///<
	float integrateVelocities; ///<
	float warmStart;		   ///<
	float solveImpulses;	   ///<
	float integratePositions;  ///<
	float relaxImpulses;	   ///<
	float restitution;		   ///<
	float storeImpulses;	   ///<
	float splitIslands;		   ///<
	float transforms;		   ///<
	float sensorHits;		   ///<
	float jointEvents;		   ///<
	float hitEvents;		   ///<
	float refit;			   ///<
	float bullets;			   ///<
	float sleepIslands;		   ///<
	float sensors;			   ///<
} b2Profile;
B2_API b2Profile b2World_GetProfile(b2WorldId worldId);
B2_API void b2World_DumpMemoryStats(b2WorldId worldId);
typedef struct b2Counters
{
	size_t byteCount;	  ///<
	int bodyCount;		  ///<
	int shapeCount;		  ///<
	int contactCount;	  ///<
	int jointCount;		  ///<
	int islandCount;	  ///<
	int stackUsed;		  ///<
	int staticTreeHeight; ///<
	int treeHeight;		  ///<
	int taskCount;		  ///<
	int enqueueCount;	  ///<
	int colorCounts[24];  ///<

	/// 碰撞阶段触及的接触数量（图接触 + 醒着集合中的非触摸接触）。
	int awakeContactCount;

	/// 最近一步中回收的接触数量。
	int recycledContactCount;

} b2Counters;
B2_API b2Counters b2World_GetCounters(b2WorldId worldId);
B2_API b2Capacity b2World_GetMaxCapacity(b2WorldId worldId);
B2_API void b2World_Step( b2WorldId worldId, float timeStep, int subStepCount );
B2_API void b2World_EnableWarmStarting(b2WorldId worldId,bool flag);
B2_API bool b2World_IsWarmStartingEnabled(b2WorldId worldId);
B2_API void b2World_SetContactRecycleDistance(b2WorldId worldId,float recycleDistance);
B2_API float b2World_GetContactRecycleDistance(b2WorldId worldId);
B2_API void b2World_SetRestitutionThreshold(b2WorldId worldId,float value);
B2_API float b2World_GetRestitutionThreshold(b2WorldId worldId);
B2_API void b2World_SetRestitutionIterations(b2WorldId worldId,int iterations);
B2_API int b2World_GetRestitutionIterations(b2WorldId worldId);
B2_API void b2World_EnableRestitutionPropagation(b2WorldId worldId,bool flag);
B2_API bool b2World_IsRestitutionPropagationEnabled(b2WorldId worldId);
B2_API void b2World_SetFrictionCallback(b2WorldId worldId,b2FrictionCallbackFcn* callback);
B2_API void b2World_SetRestitutionCallback(b2WorldId worldId,b2RestitutionCallbackFcn* callback);
B2_API void b2Shape_SetUserMaterial(b2ShapeId shapeId,uint64_t material);
B2_API uint64_t b2Shape_GetUserMaterial(b2ShapeId shapeId);
B2_API void b2Body_EnableContactRecycling(b2BodyId bodyId,bool flag);
B2_API bool b2Body_IsContactRecyclingEnabled(b2BodyId bodyId);
B2_API void b2Body_SetMotionLocks(b2BodyId bodyId,b2MotionLocks locks);
B2_API b2MotionLocks b2Body_GetMotionLocks(b2BodyId bodyId);
B2_API void b2Body_Disable(b2BodyId bodyId);
B2_API void b2Body_Enable(b2BodyId bodyId);
B2_API bool b2Body_IsEnabled(b2BodyId bodyId);
B2_API bool b2Body_IsAwake(b2BodyId bodyId);
B2_API void b2Body_SetAwake(b2BodyId bodyId,bool awake);
B2_API void b2Body_WakeTouching(b2BodyId bodyId);
B2_API void b2Body_SetBullet(b2BodyId bodyId,bool flag);
B2_API bool b2Body_IsBullet(b2BodyId bodyId);
B2_API void b2Body_SetSafetyFactor(b2BodyId bodyId,float safetyFactor);
B2_API float b2Body_GetSafetyFactor(b2BodyId bodyId);
B2_API void b2World_SetMaxLinearSpeed(b2WorldId worldId,float maximumLinearSpeed);
B2_API float b2World_GetMaxLinearSpeed(b2WorldId worldId);
B2_API void b2World_SetGravity(b2WorldId worldId,b2Vec2 gravity);
B2_API b2Vec2 b2World_GetGravity(b2WorldId worldId);
B2_API void b2World_EnableContinuous(b2WorldId worldId,bool flag);
B2_API bool b2World_IsContinuousEnabled(b2WorldId worldId);
B2_API void b2Body_SetSleepThreshold(b2BodyId bodyId,float sleepThreshold);
B2_API float b2Body_GetSleepThreshold(b2BodyId bodyId);
B2_API void b2Body_EnableSleeping(b2BodyId bodyId,bool enableSleep);
B2_API bool b2Body_IsSleepingEnabled(b2BodyId bodyId);
B2_API void b2World_EnableSleeping(b2WorldId worldId,bool flag);
B2_API bool b2World_IsSleepingEnabled(b2WorldId worldId);
B2_API void b2Shape_SetFriction(b2ShapeId shapeId,float friction);
B2_API float b2Shape_GetFriction(b2ShapeId shapeId);
B2_API void b2Shape_SetRestitution(b2ShapeId shapeId,float restitution);
B2_API float b2Shape_GetRestitution(b2ShapeId shapeId);
B2_API void b2Shape_SetSurfaceMaterial(b2ShapeId shapeId,const b2SurfaceMaterial* surfaceMaterial);
B2_API b2SurfaceMaterial b2Shape_GetSurfaceMaterial(b2ShapeId shapeId);
B2_API void b2World_SetContactTuning(b2WorldId worldId,float hertz,float dampingRatio,float pushSpeed);
B2_API void b2World_GetContactTuning(b2WorldId worldId,float* hertz,float* dampingRatio,float* pushSpeed);


B2_API b2Vec2 b2Body_GetLinearVelocity( b2BodyId bodyId );
B2_API float b2Body_GetAngularVelocity( b2BodyId bodyId );
B2_API void b2Body_SetLinearVelocity( b2BodyId bodyId, b2Vec2 linearVelocity );
B2_API void b2Body_SetAngularVelocity( b2BodyId bodyId, float angularVelocity );
B2_API b2Vec2 b2Body_GetLocalPointVelocity( b2BodyId bodyId, b2Vec2 localPoint );
B2_API b2Vec2 b2Body_GetWorldPointVelocity( b2BodyId bodyId, b2Pos worldPoint );

B2_API void b2Body_SetGravityScale( b2BodyId bodyId, float gravityScale );
B2_API float b2Body_GetGravityScale( b2BodyId bodyId );


B2_API void b2Body_SetLinearDamping( b2BodyId bodyId, float linearDamping );
B2_API float b2Body_GetLinearDamping( b2BodyId bodyId );
B2_API void b2Body_SetAngularDamping( b2BodyId bodyId, float angularDamping );
B2_API float b2Body_GetAngularDamping( b2BodyId bodyId );


B2_API void b2Body_ApplyForce( b2BodyId bodyId, b2Vec2 force, b2Pos point, bool wake );
B2_API void b2Body_ApplyForceToCenter( b2BodyId bodyId, b2Vec2 force, bool wake );
B2_API void b2Body_ClearForces( b2BodyId bodyId );

B2_API void b2Body_ApplyTorque( b2BodyId bodyId, float torque, bool wake );


B2_API void b2Body_ApplyLinearImpulse( b2BodyId bodyId, b2Vec2 impulse, b2Pos point, bool wake );
B2_API void b2Body_ApplyLinearImpulseToCenter( b2BodyId bodyId, b2Vec2 impulse, bool wake );

B2_API void b2Body_ApplyAngularImpulse( b2BodyId bodyId, float impulse, bool wake );

B2_API void b2Body_SetTargetTransform( b2BodyId bodyId, b2WorldTransform target, float timeStep, bool wake );

B2_API b2BodyType b2Body_GetType( b2BodyId bodyId );

B2_API void b2Body_SetType( b2BodyId bodyId, b2BodyType type );

B2_API bool b2Contact_IsValid( b2ContactId contactId );


typedef struct b2ContactData
{
    b2ContactId contactId;
    b2ShapeId shapeIdA;
    b2ShapeId shapeIdB;
    b2Manifold manifold;
} b2ContactData;

B2_API b2ContactData b2Contact_GetData( b2ContactId contactId );
B2_API int b2Body_GetContactCapacity( b2BodyId bodyId );
B2_API int b2Body_GetContactData( b2BodyId bodyId, b2ContactData* contactData, int capacity );
B2_API int b2Shape_GetContactCapacity( b2ShapeId shapeId );
B2_API int b2Shape_GetContactData( b2ShapeId shapeId, b2ContactData* contactData, int capacity );

typedef struct b2RayResult
{
	/// 命中的形状。
	b2ShapeId shapeId;

	/// 接触点，位于命中形状的表面上。
	b2Pos point;

	/// 接触点处的单位法向量。
	b2Vec2 normal;

	/// 输入射线的分数。
	float fraction;

	/// 访问的 BVH 节点数。用于诊断。
	int nodeVisits;

	/// 访问的 BVH 叶节点数。用于诊断。
	int leafVisits;

	/// 射线是否命中？若为 false，所有其他数据无效。
	bool hit;
} b2RayResult;


B2_API b2RayResult b2World_CastRayClosest(b2WorldId worldId,b2Pos origin,b2Vec2 translation,b2QueryFilter filter);

B2_API b2TreeStats b2World_CastShape(b2WorldId worldId,b2Pos origin,const b2ShapeProxy* proxy,b2Vec2 translation,b2QueryFilter filter,b2CastResultCallbackFcn* fcn,void* context);

B2_API bool b2Shape_TestPoint(b2ShapeId shapeId,b2Pos point);

B2_API b2WorldCastOutput b2Shape_CastRay(b2ShapeId shapeId,b2Pos origin,b2Vec2 translation);

B2_API b2Pos b2Shape_GetClosestPoint(b2ShapeId shapeId,b2Pos target);

typedef struct b2ExplosionDef
{
	/// 用于过滤形状的掩码位
	uint64_t maskBits;

	/// 爆炸中心，位于世界空间
	b2Pos position;

	/// 爆炸半径
	float radius;

	/// 超出半径的衰减距离。冲量在此距离处减为零。
	float falloff;

	/// 单位长度冲量。这根据面向爆炸的形状周长施加冲量。爆炸仅适用于圆、胶囊与多边形。
	/// 可为负值表示内爆。
	float impulsePerLength;
} b2ExplosionDef;

/// 用此初始化你的爆炸定义
/// @ingroup world
B2_API b2ExplosionDef b2DefaultExplosionDef( void );


B2_API void b2World_Explode(b2WorldId worldId,const b2ExplosionDef* explosionDef);

B2_API void b2Shape_ApplyWind(b2ShapeId shapeId,b2Vec2 wind,float drag,float lift,bool wake);

B2_API float b2World_CastMover(b2WorldId worldId,b2Pos origin,const b2Capsule* mover,b2Vec2 translation,b2QueryFilter filter);

typedef bool b2PlaneResultCallbackFcn(b2ShapeId shapeId,const b2PlaneResult* plane,void* context);
B2_API void b2World_CollideMover(b2WorldId worldId,b2Pos origin,const b2Capsule* mover,b2QueryFilter filter,b2PlaneResultCallbackFcn* fcn,void* context);

B2_API b2AABB b2World_GetBounds(b2WorldId worldId);

typedef enum b2HexColor
{
	b2_colorAliceBlue = 0xF0F8FF,
	b2_colorAntiqueWhite = 0xFAEBD7,
	b2_colorAqua = 0x00FFFF,
	b2_colorAquamarine = 0x7FFFD4,
	b2_colorAzure = 0xF0FFFF,
	b2_colorBeige = 0xF5F5DC,
	b2_colorBisque = 0xFFE4C4,
	b2_colorBlack = 0x000000,
	b2_colorBlanchedAlmond = 0xFFEBCD,
	b2_colorBlue = 0x0000FF,
	b2_colorBlueViolet = 0x8A2BE2,
	b2_colorBrown = 0xA52A2A,
	b2_colorBurlywood = 0xDEB887,
	b2_colorCadetBlue = 0x5F9EA0,
	b2_colorChartreuse = 0x7FFF00,
	b2_colorChocolate = 0xD2691E,
	b2_colorCoral = 0xFF7F50,
	b2_colorCornflowerBlue = 0x6495ED,
	b2_colorCornsilk = 0xFFF8DC,
	b2_colorCrimson = 0xDC143C,
	b2_colorCyan = 0x00FFFF,
	b2_colorDarkBlue = 0x00008B,
	b2_colorDarkCyan = 0x008B8B,
	b2_colorDarkGoldenRod = 0xB8860B,
	b2_colorDarkGray = 0xA9A9A9,
	b2_colorDarkGreen = 0x006400,
	b2_colorDarkKhaki = 0xBDB76B,
	b2_colorDarkMagenta = 0x8B008B,
	b2_colorDarkOliveGreen = 0x556B2F,
	b2_colorDarkOrange = 0xFF8C00,
	b2_colorDarkOrchid = 0x9932CC,
	b2_colorDarkRed = 0x8B0000,
	b2_colorDarkSalmon = 0xE9967A,
	b2_colorDarkSeaGreen = 0x8FBC8F,
	b2_colorDarkSlateBlue = 0x483D8B,
	b2_colorDarkSlateGray = 0x2F4F4F,
	b2_colorDarkTurquoise = 0x00CED1,
	b2_colorDarkViolet = 0x9400D3,
	b2_colorDeepPink = 0xFF1493,
	b2_colorDeepSkyBlue = 0x00BFFF,
	b2_colorDimGray = 0x696969,
	b2_colorDodgerBlue = 0x1E90FF,
	b2_colorFireBrick = 0xB22222,
	b2_colorFloralWhite = 0xFFFAF0,
	b2_colorForestGreen = 0x228B22,
	b2_colorFuchsia = 0xFF00FF,
	b2_colorGainsboro = 0xDCDCDC,
	b2_colorGhostWhite = 0xF8F8FF,
	b2_colorGold = 0xFFD700,
	b2_colorGoldenRod = 0xDAA520,
	b2_colorGray = 0x808080,
	b2_colorGreen = 0x008000,
	b2_colorGreenYellow = 0xADFF2F,
	b2_colorHoneyDew = 0xF0FFF0,
	b2_colorHotPink = 0xFF69B4,
	b2_colorIndianRed = 0xCD5C5C,
	b2_colorIndigo = 0x4B0082,
	b2_colorIvory = 0xFFFFF0,
	b2_colorKhaki = 0xF0E68C,
	b2_colorLavender = 0xE6E6FA,
	b2_colorLavenderBlush = 0xFFF0F5,
	b2_colorLawnGreen = 0x7CFC00,
	b2_colorLemonChiffon = 0xFFFACD,
	b2_colorLightBlue = 0xADD8E6,
	b2_colorLightCoral = 0xF08080,
	b2_colorLightCyan = 0xE0FFFF,
	b2_colorLightGoldenRodYellow = 0xFAFAD2,
	b2_colorLightGray = 0xD3D3D3,
	b2_colorLightGreen = 0x90EE90,
	b2_colorLightPink = 0xFFB6C1,
	b2_colorLightSalmon = 0xFFA07A,
	b2_colorLightSeaGreen = 0x20B2AA,
	b2_colorLightSkyBlue = 0x87CEFA,
	b2_colorLightSlateGray = 0x778899,
	b2_colorLightSteelBlue = 0xB0C4DE,
	b2_colorLightYellow = 0xFFFFE0,
	b2_colorLime = 0x00FF00,
	b2_colorLimeGreen = 0x32CD32,
	b2_colorLinen = 0xFAF0E6,
	b2_colorMagenta = 0xFF00FF,
	b2_colorMaroon = 0x800000,
	b2_colorMediumAquaMarine = 0x66CDAA,
	b2_colorMediumBlue = 0x0000CD,
	b2_colorMediumOrchid = 0xBA55D3,
	b2_colorMediumPurple = 0x9370DB,
	b2_colorMediumSeaGreen = 0x3CB371,
	b2_colorMediumSlateBlue = 0x7B68EE,
	b2_colorMediumSpringGreen = 0x00FA9A,
	b2_colorMediumTurquoise = 0x48D1CC,
	b2_colorMediumVioletRed = 0xC71585,
	b2_colorMidnightBlue = 0x191970,
	b2_colorMintCream = 0xF5FFFA,
	b2_colorMistyRose = 0xFFE4E1,
	b2_colorMoccasin = 0xFFE4B5,
	b2_colorNavajoWhite = 0xFFDEAD,
	b2_colorNavy = 0x000080,
	b2_colorOldLace = 0xFDF5E6,
	b2_colorOlive = 0x808000,
	b2_colorOliveDrab = 0x6B8E23,
	b2_colorOrange = 0xFFA500,
	b2_colorOrangeRed = 0xFF4500,
	b2_colorOrchid = 0xDA70D6,
	b2_colorPaleGoldenRod = 0xEEE8AA,
	b2_colorPaleGreen = 0x98FB98,
	b2_colorPaleTurquoise = 0xAFEEEE,
	b2_colorPaleVioletRed = 0xDB7093,
	b2_colorPapayaWhip = 0xFFEFD5,
	b2_colorPeachPuff = 0xFFDAB9,
	b2_colorPeru = 0xCD853F,
	b2_colorPink = 0xFFC0CB,
	b2_colorPlum = 0xDDA0DD,
	b2_colorPowderBlue = 0xB0E0E6,
	b2_colorPurple = 0x800080,
	b2_colorRebeccaPurple = 0x663399,
	b2_colorRed = 0xFF0000,
	b2_colorRosyBrown = 0xBC8F8F,
	b2_colorRoyalBlue = 0x4169E1,
	b2_colorSaddleBrown = 0x8B4513,
	b2_colorSalmon = 0xFA8072,
	b2_colorSandyBrown = 0xF4A460,
	b2_colorSeaGreen = 0x2E8B57,
	b2_colorSeaShell = 0xFFF5EE,
	b2_colorSienna = 0xA0522D,
	b2_colorSilver = 0xC0C0C0,
	b2_colorSkyBlue = 0x87CEEB,
	b2_colorSlateBlue = 0x6A5ACD,
	b2_colorSlateGray = 0x708090,
	b2_colorSnow = 0xFFFAFA,
	b2_colorSpringGreen = 0x00FF7F,
	b2_colorSteelBlue = 0x4682B4,
	b2_colorTan = 0xD2B48C,
	b2_colorTeal = 0x008080,
	b2_colorThistle = 0xD8BFD8,
	b2_colorTomato = 0xFF6347,
	b2_colorTurquoise = 0x40E0D0,
	b2_colorViolet = 0xEE82EE,
	b2_colorWheat = 0xF5DEB3,
	b2_colorWhite = 0xFFFFFF,
	b2_colorWhiteSmoke = 0xF5F5F5,
	b2_colorYellow = 0xFFFF00,
	b2_colorYellowGreen = 0x9ACD32,

	b2_colorBox2DRed = 0xDC3132,
	b2_colorBox2DBlue = 0x30AEBF,
	b2_colorBox2DGreen = 0x8CC924,
	b2_colorBox2DYellow = 0xFFEE8C
} b2HexColor;

/// 获取分配给约束图颜色槽的可视化颜色。最后一个索引（B2_GRAPH_COLOR_COUNT - 1）是溢出颜色。
/// @ingroup debug
B2_API b2HexColor b2GetGraphColor( int index );

/// 此结构保存你可实现以绘制 Box2D 世界的回调。回调接收世界坐标。
/// 在大世界模式下平移为双精度，以便远离原点时仍保持准确。在回调内转换到你自己的相机坐标系。
/// 用 b2DefaultDebugDraw 初始化。
/// @ingroup debug
typedef struct b2DebugDraw
{
	/// 绘制以逆时针顺序提供的闭合多边形。
	void ( *DrawPolygonFcn )( b2WorldTransform transform, const b2Vec2* vertices, int vertexCount, b2HexColor color,
							  void* context );

	/// 绘制以逆时针顺序提供的实心闭合多边形。
	void ( *DrawSolidPolygonFcn )( b2WorldTransform transform, const b2Vec2* vertices, int vertexCount, float radius,
								   b2HexColor color, void* context );

	/// 绘制圆。
	void ( *DrawCircleFcn )( b2Pos center, float radius, b2HexColor color, void* context );

	/// 绘制实心圆。
	void ( *DrawSolidCircleFcn )( b2WorldTransform transform, b2Vec2 center, float radius, b2HexColor color,
								  void* context );

	/// 绘制实心胶囊。
	void ( *DrawSolidCapsuleFcn )( b2Pos p1, b2Pos p2, float radius, b2HexColor color, void* context );

	/// 绘制线段。
	void ( *DrawLineFcn )( b2Pos p1, b2Pos p2, b2HexColor color, void* context );

	/// 绘制变换。自行选择长度比例。
	void ( *DrawTransformFcn )( b2WorldTransform transform, void* context );

	/// 绘制点。
	void ( *DrawPointFcn )( b2Pos p, float size, b2HexColor color, void* context );

	/// 在世界空间中绘制字符串
	void ( *DrawStringFcn )( b2Pos p, const char* s, b2HexColor color, void* context );

	/// 绘制包围盒。启用双精度时，单精度包围盒在远离原点时会增加填充。
	void ( *DrawBoundsFcn )( b2AABB aabb, b2HexColor color, void* context );

	/// 用于调试绘制的世界边界。空盒子不绘制任何内容。
	b2AABB drawingBounds;

	/// 绘制力时使用的缩放
	float forceScale;

	/// 关节绘制的全局缩放
	float jointScale;

	/// 是否绘制接触点
	bool drawContacts;

	/// 绘制接触点的锚点 A（而非锚点 B）
	bool drawAnchorA;

	/// 是否绘制形状
	bool drawShapes;

	/// 是否绘制链条形状法线
	bool drawChainNormals;

	/// 是否绘制关节
	bool drawJoints;

	/// 是否绘制关节的附加信息
	bool drawJointExtras;

	/// 是否绘制形状的包围盒
	bool drawBounds;

	/// 是否绘制动态刚体的质量与质心
	bool drawMass;

	/// 是否绘制刚体名称
	bool drawBodyNames;

	/// 是否可视化用于接触与关节的图着色
	bool drawGraphColors;

	/// 是否绘制接触特征 id
	bool drawContactFeatures;

	/// 是否绘制接触法线
	bool drawContactNormals;

	/// 是否绘制接触法向力
	bool drawContactForces;

	/// 是否绘制接触摩擦力
	bool drawFrictionForces;

	/// 是否将岛绘制为包围盒
	bool drawIslands;

	/// 作为参数传递给绘制回调函数的用户上下文
	void* context;
} b2DebugDraw;

/// 用此初始化你的绘制接口。这允许你只实现绘制函数的子集。
/// @ingroup debug
B2_API b2DebugDraw b2DefaultDebugDraw( void );


B2_API void b2World_Draw(b2WorldId worldId,b2DebugDraw* draw);

B2_API void b2World_RebuildStaticTree(b2WorldId worldId);

B2_API bool b2Joint_IsValid(b2JointId jointId);
B2_API void b2DestroyJoint(b2JointId jointId);
B2_API b2WorldId b2Joint_GetWorld(b2JointId jointId);
B2_API b2BodyId b2Joint_GetBodyA(b2JointId jointId);
B2_API b2BodyId b2Joint_GetBodyB(b2JointId jointId);
B2_API void b2Joint_SetUserData(b2JointId jointId,void* userData);
B2_API void* b2Joint_GetUserData(b2JointId jointId);
B2_API void b2Joint_SetCollideConnected(b2JointId jointId,bool shouldCollide);
B2_API bool b2Joint_GetCollideConnected(b2JointId jointId);

/// 设置关节事件的力阈值（牛顿）
B2_API void b2Joint_SetForceThreshold(b2JointId jointId,float threshold);

/// 获取关节事件的力阈值（牛顿）
B2_API float b2Joint_GetForceThreshold(b2JointId jointId);

/// 设置关节事件的力矩阈值（N-m）
B2_API void b2Joint_SetTorqueThreshold(b2JointId jointId,float threshold);

/// 获取关节事件的力矩阈值（N-m）
B2_API float b2Joint_GetTorqueThreshold(b2JointId jointId);

B2_API int b2Body_GetJointCount(b2BodyId bodyId);
B2_API int b2Body_GetJoints(b2BodyId bodyId,b2JointId* jointArray,int capacity);

typedef struct b2JointDef
{
	/// 用户数据指针
	void* userData;

	/// 第一个附着刚体
	b2BodyId bodyIdA;

	/// 第二个附着刚体
	b2BodyId bodyIdB;

	/// 第一个局部关节坐标系
	b2Transform localFrameA;

	/// 第二个局部关节坐标系
	b2Transform localFrameB;

	/// 关节事件的力阈值
	float forceThreshold;

	/// 关节事件的力矩阈值
	float torqueThreshold;

	/// 约束赫兹（高级功能）
	float constraintHertz;

	/// 约束阻尼比（高级功能）
	float constraintDampingRatio;

	/// 调试绘制缩放
	float drawScale;

	/// 若附着刚体应碰撞，将此标志设为 true
	bool collideConnected;

} b2JointDef;

typedef struct b2DistanceJointDef
{
	/// 基础关节定义
	b2JointDef base;

	/// 此关节的静止长度。夹紧到稳定的最小值。
	float length;

	/// 使距离约束表现为弹簧。若为 false，则距离关节为刚性，覆盖限位与马达。
	bool enableSpring;

	/// 下限弹簧力控制它能承受的张力。
	float lowerSpringForce;

	/// 上限弹簧力控制它能承受的压缩。
	float upperSpringForce;

	/// 弹簧线性刚度赫兹，每秒周期数。
	float hertz;

	/// 弹簧线性阻尼比，无量纲
	float dampingRatio;

	/// 启用/禁用关节限位
	bool enableLimit;

	/// 限位的最小长度。夹紧到稳定的最小值。
	float minLength;

	/// 限位的最大长度。必须大于或等于最小长度。
	float maxLength;

	/// 启用/禁用关节马达
	bool enableMotor;

	/// 最大马达力，通常以牛顿为单位
	float maxMotorForce;

	/// 期望的马达速度，通常以米每秒为单位
	float motorSpeed;

	/// 内部用于检测有效定义。请勿设置。
	int internalValue;
} b2DistanceJointDef;
B2_API b2DistanceJointDef b2DefaultDistanceJointDef(void);

typedef enum b2JointType { b2_distanceJoint, b2_filterJoint=1, b2_revoluteJoint=6, b2_prismaticJoint=5, b2_wheelJoint=8, b2_weldJoint=7, b2_motorJoint=2, b2_moverJoint=3, b2_pogoJoint=4 } b2JointType;

B2_API b2JointId b2CreateDistanceJoint(b2WorldId worldId,const b2DistanceJointDef* def);
B2_API b2JointType b2Joint_GetType(b2JointId jointId);
B2_API b2Transform b2Joint_GetLocalFrameA(b2JointId jointId);
B2_API b2Transform b2Joint_GetLocalFrameB(b2JointId jointId);
B2_API void b2Joint_SetLocalFrameA(b2JointId jointId,b2Transform localFrame);
B2_API void b2Joint_SetLocalFrameB(b2JointId jointId,b2Transform localFrame);
B2_API void b2Joint_SetConstraintTuning(b2JointId jointId,float hertz,float dampingRatio);
B2_API void b2Joint_GetConstraintTuning(b2JointId jointId,float* hertz,float* dampingRatio);
B2_API b2Vec2 b2Joint_GetConstraintForce(b2JointId jointId);
B2_API float b2Joint_GetConstraintTorque(b2JointId jointId);
B2_API float b2Joint_GetLinearSeparation(b2JointId jointId);
B2_API float b2Joint_GetAngularSeparation(b2JointId jointId);
B2_API void b2DistanceJoint_SetRestLength(b2JointId jointId,float length);
B2_API float b2DistanceJoint_GetRestLength(b2JointId jointId);
B2_API float b2DistanceJoint_GetCurrentLength(b2JointId jointId);

B2_API void b2DistanceJoint_EnableSpring( b2JointId jointId, bool enableSpring );
B2_API bool b2DistanceJoint_IsSpringEnabled( b2JointId jointId );
B2_API void b2DistanceJoint_SetSpringForceRange( b2JointId jointId, float lowerForce, float upperForce );
B2_API void b2DistanceJoint_GetSpringForceRange( b2JointId jointId, float* lowerForce, float* upperForce );
B2_API void b2DistanceJoint_SetSpringHertz( b2JointId jointId, float hertz );
B2_API void b2DistanceJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio );
B2_API float b2DistanceJoint_GetSpringHertz( b2JointId jointId );
B2_API float b2DistanceJoint_GetSpringDampingRatio( b2JointId jointId );
B2_API float b2DistanceJoint_GetSpringForce( b2JointId jointId );

B2_API void b2DistanceJoint_EnableLimit( b2JointId jointId, bool enableLimit );
B2_API bool b2DistanceJoint_IsLimitEnabled( b2JointId jointId );
B2_API void b2DistanceJoint_SetLimits( b2JointId jointId, float minLength, float maxLength );
B2_API float b2DistanceJoint_GetLowerLimit( b2JointId jointId );
B2_API float b2DistanceJoint_GetUpperLimit( b2JointId jointId );

B2_API void b2DistanceJoint_EnableMotor( b2JointId jointId, bool enableMotor );
B2_API bool b2DistanceJoint_IsMotorEnabled( b2JointId jointId );
B2_API void b2DistanceJoint_SetTargetMotorSpeed( b2JointId jointId, float motorSpeed );
B2_API float b2DistanceJoint_GetTargetMotorSpeed( b2JointId jointId );
B2_API float b2DistanceJoint_GetCurrentMotorForce( b2JointId jointId );
B2_API void b2DistanceJoint_SetMaxMotorForce( b2JointId jointId, float force );
B2_API float b2DistanceJoint_GetMaxMotorForce( b2JointId jointId );

typedef struct b2RevoluteJointDef
{
	/// 基础关节定义
	b2JointDef base;

	/// 关节的目标角，单位为弧度。弹簧-阻尼器将驱动到此角。
	float targetAngle;

	/// 在旋转铰链轴上启用旋转弹簧
	bool enableSpring;

	/// 弹簧刚度赫兹，每秒周期数
	float hertz;

	/// 弹簧阻尼比，无量纲
	float dampingRatio;

	/// 启用关节限位的标志
	bool enableLimit;

	/// 关节限位的下限角，单位为弧度。最小为 -0.99*pi 弧度。
	float lowerAngle;

	/// 关节限位的上限角，单位为弧度。最大为 0.99*pi 弧度。
	float upperAngle;

	/// 启用关节马达的标志
	bool enableMotor;

	/// 最大马达力矩，通常以牛顿·米为单位
	float maxMotorTorque;

	/// 期望的马达速度，单位为弧度每秒
	float motorSpeed;

	/// 内部用于检测有效定义。请勿设置。
	int internalValue;
} b2RevoluteJointDef;
B2_API b2RevoluteJointDef b2DefaultRevoluteJointDef(void);
B2_API b2JointId b2CreateRevoluteJoint(b2WorldId worldId,const b2RevoluteJointDef* def);

B2_API float b2RevoluteJoint_GetCurrentAngle( b2JointId jointId );
B2_API float b2RevoluteJoint_GetCurrentSpeed( b2JointId jointId );

B2_API void b2RevoluteJoint_EnableSpring( b2JointId jointId, bool enableSpring );
B2_API bool b2RevoluteJoint_IsSpringEnabled( b2JointId jointId );
B2_API void b2RevoluteJoint_SetSpringHertz( b2JointId jointId, float hertz );
B2_API float b2RevoluteJoint_GetSpringHertz( b2JointId jointId );
B2_API void b2RevoluteJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio );
B2_API float b2RevoluteJoint_GetSpringDampingRatio( b2JointId jointId );
B2_API void b2RevoluteJoint_SetTargetAngle( b2JointId jointId, float angle );
B2_API float b2RevoluteJoint_GetTargetAngle( b2JointId jointId );

B2_API void b2RevoluteJoint_EnableLimit( b2JointId jointId, bool enableLimit );
B2_API bool b2RevoluteJoint_IsLimitEnabled( b2JointId jointId );
B2_API float b2RevoluteJoint_GetLowerLimit( b2JointId jointId );
B2_API float b2RevoluteJoint_GetUpperLimit( b2JointId jointId );
B2_API void b2RevoluteJoint_SetLimits( b2JointId jointId, float lower, float upper );

B2_API void b2RevoluteJoint_EnableMotor( b2JointId jointId, bool enableMotor );
B2_API bool b2RevoluteJoint_IsMotorEnabled( b2JointId jointId );
B2_API void b2RevoluteJoint_SetTargetMotorSpeed( b2JointId jointId, float motorSpeed );
B2_API float b2RevoluteJoint_GetTargetMotorSpeed( b2JointId jointId );
B2_API void b2RevoluteJoint_SetMaxMotorTorque( b2JointId jointId, float torque );
B2_API float b2RevoluteJoint_GetMaxMotorTorque( b2JointId jointId );
B2_API float b2RevoluteJoint_GetCurrentMotorTorque( b2JointId jointId );

typedef struct b2PrismaticJointDef
{
	/// 基础关节定义
	b2JointDef base;

	/// 沿棱柱关节轴启用线性弹簧
	bool enableSpring;

	/// 弹簧刚度赫兹，每秒周期数
	float hertz;

	/// 弹簧阻尼比，无量纲
	float dampingRatio;

	/// 关节的目标平移，单位为米。弹簧-阻尼器将驱动到此平移。
	float targetTranslation;

	/// 启用/禁用关节限位
	bool enableLimit;

	/// 平移下限
	float lowerTranslation;

	/// 平移上限
	float upperTranslation;

	/// 启用/禁用关节马达
	bool enableMotor;

	/// 最大马达力，通常以牛顿为单位
	float maxMotorForce;

	/// 期望的马达速度，通常以米每秒为单位
	float motorSpeed;

	/// 内部用于检测有效定义。请勿设置。
	int internalValue;
} b2PrismaticJointDef;
B2_API b2PrismaticJointDef b2DefaultPrismaticJointDef(void);
B2_API b2JointId b2CreatePrismaticJoint(b2WorldId worldId,const b2PrismaticJointDef* def);

B2_API float b2PrismaticJoint_GetCurrentTranslation( b2JointId jointId );
B2_API float b2PrismaticJoint_GetCurrentSpeed( b2JointId jointId );

B2_API void b2PrismaticJoint_EnableSpring( b2JointId jointId, bool enableSpring );
B2_API bool b2PrismaticJoint_IsSpringEnabled( b2JointId jointId );
B2_API void b2PrismaticJoint_SetSpringHertz( b2JointId jointId, float hertz );
B2_API float b2PrismaticJoint_GetSpringHertz( b2JointId jointId );
B2_API void b2PrismaticJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio );
B2_API float b2PrismaticJoint_GetSpringDampingRatio( b2JointId jointId );
B2_API void b2PrismaticJoint_SetTargetTranslation( b2JointId jointId, float translation );
B2_API float b2PrismaticJoint_GetTargetTranslation( b2JointId jointId );

B2_API void b2PrismaticJoint_EnableLimit( b2JointId jointId, bool enableLimit );
B2_API bool b2PrismaticJoint_IsLimitEnabled( b2JointId jointId );
B2_API void b2PrismaticJoint_SetLimits( b2JointId jointId, float lower, float upper );
B2_API float b2PrismaticJoint_GetLowerLimit( b2JointId jointId );
B2_API float b2PrismaticJoint_GetUpperLimit( b2JointId jointId );

B2_API void b2PrismaticJoint_EnableMotor( b2JointId jointId, bool enableMotor );
B2_API bool b2PrismaticJoint_IsMotorEnabled( b2JointId jointId );
B2_API void b2PrismaticJoint_SetTargetMotorSpeed( b2JointId jointId, float motorSpeed );
B2_API float b2PrismaticJoint_GetTargetMotorSpeed( b2JointId jointId );
B2_API float b2PrismaticJoint_GetCurrentMotorForce( b2JointId jointId );
B2_API void b2PrismaticJoint_SetMaxMotorForce( b2JointId jointId, float force );
B2_API float b2PrismaticJoint_GetMaxMotorForce( b2JointId jointId );

typedef struct b2WheelJointDef
{
	/// 基础关节定义
	b2JointDef base;

	/// 沿局部轴启用线性弹簧
	bool enableSpring;

	/// 弹簧刚度，单位为赫兹
	float hertz;

	/// 弹簧阻尼比，无量纲
	float dampingRatio;

	/// 启用/禁用关节线性限位
	bool enableLimit;

	/// 平移下限
	float lowerTranslation;

	/// 平移上限
	float upperTranslation;

	/// 启用/禁用关节旋转马达
	bool enableMotor;

	/// 最大马达力矩，通常以牛顿·米为单位
	float maxMotorTorque;

	/// 期望的马达速度，单位为弧度每秒
	float motorSpeed;

	/// 内部用于检测有效定义。请勿设置。
	int internalValue;
} b2WheelJointDef;
B2_API b2WheelJointDef b2DefaultWheelJointDef(void);
B2_API b2JointId b2CreateWheelJoint(b2WorldId worldId,const b2WheelJointDef* def);

B2_API void b2WheelJoint_EnableSpring( b2JointId jointId, bool enableSpring );
B2_API bool b2WheelJoint_IsSpringEnabled( b2JointId jointId );
B2_API void b2WheelJoint_SetSpringHertz( b2JointId jointId, float hertz );
B2_API float b2WheelJoint_GetSpringHertz( b2JointId jointId );
B2_API void b2WheelJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio );
B2_API float b2WheelJoint_GetSpringDampingRatio( b2JointId jointId );

B2_API void b2WheelJoint_EnableLimit( b2JointId jointId, bool enableLimit );
B2_API bool b2WheelJoint_IsLimitEnabled( b2JointId jointId );
B2_API void b2WheelJoint_SetLimits( b2JointId jointId, float lower, float upper );
B2_API float b2WheelJoint_GetLowerLimit( b2JointId jointId );
B2_API float b2WheelJoint_GetUpperLimit( b2JointId jointId );

B2_API void b2WheelJoint_EnableMotor( b2JointId jointId, bool enableMotor );
B2_API bool b2WheelJoint_IsMotorEnabled( b2JointId jointId );
B2_API void b2WheelJoint_SetTargetMotorSpeed( b2JointId jointId, float motorSpeed );
B2_API float b2WheelJoint_GetTargetMotorSpeed( b2JointId jointId );
B2_API float b2WheelJoint_GetCurrentMotorTorque( b2JointId jointId );
B2_API void b2WheelJoint_SetMaxMotorTorque( b2JointId jointId, float torque );
B2_API float b2WheelJoint_GetMaxMotorTorque( b2JointId jointId );

typedef struct b2WeldJointDef
{
	/// 基础关节定义
	b2JointDef base;

	/// 以赫兹（每秒周期数）表示的线性刚度。使用零表示最大刚度。
	float linearHertz;

	/// 以赫兹（每秒周期数）表示的角刚度。使用零表示最大刚度。
	float angularHertz;

	/// 线性阻尼比，无量纲。使用 1 表示临界阻尼。
	float linearDampingRatio;

	/// 角阻尼比，无量纲。使用 1 表示临界阻尼。
	float angularDampingRatio;

	/// 内部用于检测有效定义。请勿设置。
	int internalValue;
} b2WeldJointDef;
B2_API b2WeldJointDef b2DefaultWeldJointDef(void);
B2_API b2JointId b2CreateWeldJoint(b2WorldId worldId,const b2WeldJointDef* def);

B2_API void b2WeldJoint_SetLinearHertz( b2JointId jointId, float hertz );
B2_API float b2WeldJoint_GetLinearHertz( b2JointId jointId );
B2_API void b2WeldJoint_SetAngularHertz( b2JointId jointId, float hertz );
B2_API float b2WeldJoint_GetAngularHertz( b2JointId jointId );
B2_API void b2WeldJoint_SetLinearDampingRatio( b2JointId jointId, float dampingRatio );
B2_API float b2WeldJoint_GetLinearDampingRatio( b2JointId jointId );
B2_API void b2WeldJoint_SetAngularDampingRatio( b2JointId jointId, float dampingRatio );
B2_API float b2WeldJoint_GetAngularDampingRatio( b2JointId jointId );

typedef struct b2MotorJointDef
{
	/// 基础关节定义
	b2JointDef base;

	/// 期望的线速度
	b2Vec2 linearVelocity;

	/// 最大马达力，单位为牛顿
	float maxVelocityForce;

	/// 期望的角速度
	float angularVelocity;

	/// 最大马达力矩，单位为牛顿·米
	float maxVelocityTorque;

	/// 用于位置控制的线性弹簧赫兹
	float linearHertz;

	/// 线性弹簧阻尼比
	float linearDampingRatio;

	/// 最大弹簧力，单位为牛顿
	float maxSpringForce;

	/// 用于位置控制的角弹簧赫兹
	float angularHertz;

	/// 角弹簧阻尼比
	float angularDampingRatio;

	/// 最大弹簧力矩，单位为牛顿·米
	float maxSpringTorque;

	/// 内部用于检测有效定义。请勿设置。
	int internalValue;
} b2MotorJointDef;
B2_API b2MotorJointDef b2DefaultMotorJointDef(void);
B2_API b2JointId b2CreateMotorJoint(b2WorldId worldId,const b2MotorJointDef* def);

B2_API void b2MotorJoint_SetLinearVelocity( b2JointId jointId, b2Vec2 velocity );
B2_API b2Vec2 b2MotorJoint_GetLinearVelocity( b2JointId jointId );
B2_API void b2MotorJoint_SetAngularVelocity( b2JointId jointId, float velocity );
B2_API float b2MotorJoint_GetAngularVelocity( b2JointId jointId );
B2_API void b2MotorJoint_SetMaxVelocityTorque( b2JointId jointId, float maxTorque );
B2_API float b2MotorJoint_GetMaxVelocityTorque( b2JointId jointId );
B2_API void b2MotorJoint_SetMaxVelocityForce( b2JointId jointId, float maxForce );
B2_API float b2MotorJoint_GetMaxVelocityForce( b2JointId jointId );

typedef struct b2MoverJointDef
{
	b2JointDef base;
	b2Vec2 linearVelocity;
	b2Vec2 maxVelocityForce;
	int internalValue;
} b2MoverJointDef;

B2_API b2MoverJointDef b2DefaultMoverJointDef( void );
B2_API b2JointId b2CreateMoverJoint( b2WorldId worldId, const b2MoverJointDef* def );
B2_API void b2MoverJoint_SetLinearVelocity( b2JointId jointId, b2Vec2 velocity );
B2_API b2Vec2 b2MoverJoint_GetLinearVelocity( b2JointId jointId );
B2_API void b2MoverJoint_SetMaxVelocityForce( b2JointId jointId, b2Vec2 maxForce );
B2_API b2Vec2 b2MoverJoint_GetMaxVelocityForce( b2JointId jointId );

typedef struct b2PogoJointDef
{
	b2JointDef base;
	b2Vec2 normal;
	float hertz;
	float dampingRatio;
	float restLength;
	float maxTensionForce;
	float maxCompressionForce;
	float impulse;
	float velocity;
	int internalValue;
} b2PogoJointDef;

typedef struct b2FilterJointDef
{
	b2JointDef base;
	int internalValue;
} b2FilterJointDef;

B2_API b2FilterJointDef b2DefaultFilterJointDef( void );
B2_API b2JointId b2CreateFilterJoint( b2WorldId worldId, const b2FilterJointDef* def );

B2_API b2PogoJointDef b2DefaultPogoJointDef( void );
B2_API b2JointId b2CreatePogoJoint( b2WorldId worldId, const b2PogoJointDef* def );
B2_API void b2PogoJoint_SetRestLength( b2JointId jointId, float length );
B2_API float b2PogoJoint_GetRestLength( b2JointId jointId );
B2_API void b2PogoJoint_SetSpringHertz( b2JointId jointId, float hertz );
B2_API float b2PogoJoint_GetSpringHertz( b2JointId jointId );
B2_API void b2PogoJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio );
B2_API float b2PogoJoint_GetSpringDampingRatio( b2JointId jointId );
B2_API float b2PogoJoint_GetCurrentLength( b2JointId jointId );
B2_API float b2PogoJoint_GetImpulse( b2JointId jointId );
B2_API float b2PogoJoint_GetCurrentSpeed( b2JointId jointId );

B2_API void b2MotorJoint_SetLinearHertz( b2JointId jointId, float hertz );
B2_API float b2MotorJoint_GetLinearHertz( b2JointId jointId );
B2_API void b2MotorJoint_SetLinearDampingRatio( b2JointId jointId, float damping );
B2_API float b2MotorJoint_GetLinearDampingRatio( b2JointId jointId );
B2_API void b2MotorJoint_SetAngularHertz( b2JointId jointId, float hertz );
B2_API float b2MotorJoint_GetAngularHertz( b2JointId jointId );
B2_API void b2MotorJoint_SetAngularDampingRatio( b2JointId jointId, float damping );
B2_API float b2MotorJoint_GetAngularDampingRatio( b2JointId jointId );
B2_API void b2MotorJoint_SetMaxSpringForce( b2JointId jointId, float maxForce );
B2_API float b2MotorJoint_GetMaxSpringForce( b2JointId jointId );
B2_API void b2MotorJoint_SetMaxSpringTorque( b2JointId jointId, float maxTorque );
B2_API float b2MotorJoint_GetMaxSpringTorque( b2JointId jointId );

/// 由用户持有的录制字节缓冲区。
typedef struct b2Recording b2Recording;
/// byteCapacity为0时使用默认预分配，后续按需增长。
B2_API b2Recording* b2CreateRecording( int byteCapacity );
B2_API void b2DestroyRecording( b2Recording* recording );
/// 指针在下一次录制调用或销毁之前有效。
B2_API const uint8_t* b2Recording_GetData( const b2Recording* recording );
B2_API int b2Recording_GetSize( const b2Recording* recording );

/// 在步边界开始快照播种的录制会话；已录制、锁定或带自定义过滤回调时无效果。
B2_API void b2World_StartRecording( b2WorldId worldId, b2Recording* recording );
/// 停止会话，字节仍由用户持有。
B2_API void b2World_StopRecording( b2WorldId worldId );

/// 创建没有父链的独立单面链段，并修正退化幽灵顶点。
B2_API b2ShapeId b2CreateChainSegmentShape( b2BodyId bodyId, const b2ShapeDef* def, const b2ChainSegment* chainSegment );
/// 设置独立链段几何；父链拥有的链段不能单独修改。
B2_API void b2Shape_SetChainSegment( b2ShapeId shapeId, const b2ChainSegment* chainSegment );

/// 唤醒关节连接的两个刚体；锁定的世界不受影响。
B2_API void b2Joint_WakeBodies( b2JointId jointId );

/// 将当前录制字节保存到二进制文件，无法完整写入时返回false。
B2_API bool b2Recording_SaveToFile( const b2Recording* recording, const char* path );

/// 从文件加载独立拥有的录制缓冲；无效或不兼容文件返回NULL。
B2_API b2Recording* b2CreateRecordingFromFile( const char* path );

typedef struct b2Replay b2Replay;

/// 描述录制的静态元数据，在播放器打开文件时一次性解析。
/// @ingroup replay
typedef struct b2ReplayInfo
{
	int frameCount;	   ///< 已录制步进总数
	int workerCount;   ///< 回放世界使用的工作线程数
	float timeStep;	   ///< 已录制步进的 dt
	int subStepCount;  ///< 已录制子步
	float lengthScale; ///< 录制时生效的每米长度单位
	b2AABB bounds;	   ///< 录制期间累积的世界边界，不可用时为零外延
} b2ReplayInfo;

B2_API b2Replay* b2CreateReplay( const void* data, int size, int workerCount );
B2_API void b2DestroyReplay( b2Replay* player );
B2_API bool b2Replay_StepFrame( b2Replay* player );
B2_API b2WorldId b2Replay_GetWorldId( const b2Replay* player );
B2_API int b2Replay_GetFrame( const b2Replay* player );
B2_API b2ReplayInfo b2Replay_GetInfo( const b2Replay* player );
B2_API bool b2Replay_IsAtEnd( const b2Replay* player );

B2_API void b2Replay_Restart( b2Replay* player );

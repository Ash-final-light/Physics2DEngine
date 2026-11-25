// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"
#include "box2d/constants.h"
#include "core.h"

static inline int b2DecodeAwakeIndex(int encodedBodySimIndex)
{ return encodedBodySimIndex>=0?encodedBodySimIndex:B2_NULL_INDEX; }

enum b2BodyFlags
{
	// 此刚体沿 x 轴平移锁定
	b2_lockLinearX = 0x00000001,

	// 此刚体沿 y 轴平移锁定
	b2_lockLinearY = 0x00000002,

	// 此刚体旋转锁定
	b2_lockAngularZ = 0x00000004,

	// 用于调试绘制和接触回收。仅在 b2BodySim 上。
	b2_isFast = 0x00000008,

	// 此动态刚体对所有刚体类型做最终 CCD 扫描，但不与其他子弹互扫
	b2_isBullet = 0x00000010,

	// 此刚体在当前时间步被限速
	b2_isSpeedCapped = 0x00000020,

	// 此刚体在当前时间步发生过撞击时刻事件（仅刚体仿真）。
	b2_hadTimeOfImpact = 0x00000040,

	// 此刚体角速度无上限
	b2_allowFastRotation = 0x00000080,

	// 此子弹刚体需要扩大 AABB。因为子弹不走标准粗检测更新。
	b2_enlargeBulletBounds = 0x00000100,

	// 此刚体为动态，求解器应写入它。
	// 避免写入运动学刚体，即使值未变也会在多线程共享时造成缓存一致性问题。
	// 用于 b2BodyState 标志。
	b2_dynamicFlag = 0x00000200,

	// 用户用 updateBodyMass 选项推迟了质量计算，
	// 但世界步进前未调用 b2Body_UpdateMassFromShapes。
	b2_dirtyMass = 0x00000400,

	b2_enableSleep = 0x00000800,

	b2_bodyEnableContactRecycling = 0x00001000,

	// 全部锁定标志
	b2_allLocks = b2_lockAngularZ | b2_lockLinearX | b2_lockLinearY,

	// 设置此标志表示刚体旋转锁定
	b2_fixedRotation = b2_lockAngularZ,

	// 这些标志每个时间步是瞬时的。在 b2Body、b2BodySim 和 b2BodyState 上可能不同。
	b2_bodyTransientFlags = b2_isSpeedCapped | b2_hadTimeOfImpact,
};

// 不用于求解器的刚体组织结构。
typedef struct b2Body
{
	void* userData;

	// 存储在 b2World 中的求解器集合下标
	// 可为 B2_NULL_INDEX
	int setIndex;

	// 集合内的刚体仿真和状态下标
	// 可为 B2_NULL_INDEX
	int localIndex;

	// [31 : contactId | 1 : edgeIndex]
	int headContactKey;
	int contactCount;

	// todo 或许移到刚体仿真
	int headShapeId;
	int shapeCount;

	int headChainId;

	// [31 : jointId | 1 : edgeIndex]
	int headJointKey;
	int jointCount;

	// 所有启用的动态和运动学刚体都在某个岛中。
	int islandId;

	// 用于加速并查集的岛下标
	int islandIndex;

	float mass;

	// 绕质心的转动惯量。
	float inertia;

	float sleepThreshold;
	float sleepTime;
	float safetyFactor;

	// 用于调整刚体移动数组中的 fellAsleep 标志
	int bodyMoveIndex;

	int id;

	// b2BodyFlags
	uint32_t flags;

	b2BodyType type;

	// 此槽位分配刚体时单调递增
	// 用于检查无效的 b2BodyId
	uint16_t generation;

	char name[B2_NAME_LENGTH + 1];
} b2Body;

// 刚体状态
// 刚体状态设计为通过 scatter-gather 快速与 SIMD 互转。
// 仅唤醒的动态和运动学刚体有刚体状态。
// 用于性能关键的约束求解器
//
// 求解器操作刚体状态。刚体状态数组不含静态刚体。静态刚体在工作线程间共享。
// 读取其状态没问题，但写入即使值不变也会在工作线程间造成缓存颠簸。
// 计算锚点时会带来麻烦。每个子步我用刚体旋转来旋转关节锚点。静态刚体的锚点不旋转。
// A 或 B 可能是静态的，这会导致大量分支。应尽量减少这些分支。
//
// 方案 1：
// 使用增量旋转。锚点需在世界空间准备。静态刚体用哑状态，增量旋转为单位旋转。
// 需计算基准分离和角度。流形会滞后一帧，但刚体快速运动时这可能最好。
//
// 方案 2：
// 使用完整旋转。静态刚体锚点在世界空间，动态刚体锚点在局部空间。
// 可能令人困惑且易出 bug。
//
// 注意：
// 每个子步旋转关节锚点但不旋转接触锚点。按子步进度旋转关节锚点可大幅提升关节稳定性。
// 子步中旋转接触锚点会降低稳定性，尤其是圆形形状。

// 32 字节
typedef struct b2BodyState
{
	b2Vec2 linearVelocity; // 8
	float angularVelocity; // 4

	// b2BodyFlags
	// 重要标志：锁定、动态
	uint32_t flags; // 4

	// 使用增量位置可减少远离原点时的舍入误差
	b2Vec2 deltaPosition; // 8

	// 使用增量旋转，因为求解器无法访问静态刚体的完整旋转，
	// 静态刚体必须用零增量旋转 (c,s) = (1,0)
	b2Rot deltaRotation; // 8
} b2BodyState;

// 单位刚体状态，注意 deltaRotation 为 {1, 0}
static const b2BodyState b2_identityBodyState = { { 0.0f, 0.0f }, 0.0f, 0, { 0.0f, 0.0f }, { 1.0f, 0.0f } };

// 用于位置和速度积分的刚体仿真数据
// 变换数据用于碰撞和求解器准备。
typedef struct b2BodySim
{
	// 刚体原点变换，大世界模式下为双精度平移
	b2WorldTransform transform;

	// 世界空间中的质心位置
	b2Pos center;

	// 上一帧旋转和质心，用于 TOI
	b2Rot rotation0;
	b2Pos center0;

	// 质心相对刚体原点的位置
	b2Vec2 localCenter;

	b2Vec2 force;
	float torque;

	// 逆转动惯量
	float invMass;
	float invInertia;

	float minExtent;
	float maxExtent;
	float linearDamping;
	float angularDamping;
	float gravityScale;

	// b2Body 的下标
	int bodyId;

	// b2BodyFlags
	uint32_t flags;
} b2BodySim;


struct b2World;
b2Body* b2GetBodyFullId( b2World* world, b2BodyId id );
b2BodySim* b2GetBodySim( b2World* world, b2Body* body );
b2BodyState* b2GetBodyState( b2World* world, b2Body* body );
int b2EncodeBodySimIndex( const b2Body* body );
bool b2WakeBody( b2World* world, b2Body* body );
void b2UpdateBodyMassData( b2World* world, b2Body* body );
void b2SyncBodyFlags( b2World* world, b2Body* body );

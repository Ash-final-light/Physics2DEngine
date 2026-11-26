// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#pragma once
#include "solver.h"
#include "box2d/types.h"
struct b2World;
struct b2Body;
typedef struct b2JointEdge
{
	int bodyId;
	int prevKey;
	int nextKey;
} b2JointEdge;

// 从 b2JointId 映射到求解器集合中的 b2Joint
typedef struct b2Joint
{
	void* userData;

	// 存储在 b2World 中的仿真集合下标
	// 槽位空闲时为 B2_NULL_INDEX
	int setIndex;

	// 约束图颜色数组下标，休眠/禁用关节可为 B2_NULL_INDEX
	// 槽位空闲时为 B2_NULL_INDEX
	int colorIndex;

	// 集合或图颜色内的关节下标
	// 槽位空闲时为 B2_NULL_INDEX
	int localIndex;

	b2JointEdge edges[2];

	int jointId;
	int islandId;

	// 岛关节数组下标，用于 O(1) 交换删除。
	// 不在岛中时为 B2_NULL_INDEX。
	int islandIndex;

	float drawScale;

	b2JointType type;

	// 此槽位分配刚体时单调递增
	// 用于检查无效的 b2JointId
	uint16_t generation;

	bool collideConnected;

} b2Joint;

typedef struct b2DistanceJoint
{
	float length;
	float hertz;
	float dampingRatio;
	float lowerSpringForce;
	float upperSpringForce;
	float minLength;
	float maxLength;

	float maxMotorForce;
	float motorSpeed;

	float impulse;
	float lowerImpulse;
	float upperImpulse;
	float motorImpulse;

	int indexA;
	int indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	b2Softness distanceSoftness;
	float axialMass;

	bool enableSpring;
	bool enableLimit;
	bool enableMotor;
} b2DistanceJoint;

typedef struct b2RevoluteJoint
{
	b2Vec2 linearImpulse;
	float springImpulse;
	float motorImpulse;
	float lowerImpulse;
	float upperImpulse;
	float hertz;
	float dampingRatio;
	float targetAngle;
	float maxMotorTorque;
	float motorSpeed;
	float lowerAngle;
	float upperAngle;

	int indexA;
	int indexB;
	b2Transform frameA;
	b2Transform frameB;
	b2Vec2 deltaCenter;
	float axialMass;
	b2Softness springSoftness;

	bool enableSpring;
	bool enableMotor;
	bool enableLimit;
} b2RevoluteJoint;

typedef struct b2PrismaticJoint
{
	b2Vec2 impulse;
	float springImpulse;
	float motorImpulse;
	float lowerImpulse;
	float upperImpulse;
	float hertz;
	float dampingRatio;
	float targetTranslation;
	float maxMotorForce;
	float motorSpeed;
	float lowerTranslation;
	float upperTranslation;

	int indexA;
	int indexB;
	b2Transform frameA;
	b2Transform frameB;
	b2Vec2 deltaCenter;
	b2Softness springSoftness;

	bool enableSpring;
	bool enableLimit;
	bool enableMotor;
} b2PrismaticJoint;

typedef struct b2WheelJoint
{
	float perpImpulse;
	float motorImpulse;
	float springImpulse;
	float lowerImpulse;
	float upperImpulse;
	float maxMotorTorque;
	float motorSpeed;
	float lowerTranslation;
	float upperTranslation;
	float hertz;
	float dampingRatio;

	int indexA;
	int indexB;
	b2Transform frameA;
	b2Transform frameB;
	b2Vec2 deltaCenter;
	float perpMass;
	float motorMass;
	float axialMass;
	b2Softness springSoftness;

	bool enableSpring;
	bool enableMotor;
	bool enableLimit;
} b2WheelJoint;

typedef struct b2WeldJoint
{
	float linearHertz;
	float linearDampingRatio;
	float angularHertz;
	float angularDampingRatio;

	b2Softness linearSpring;
	b2Softness angularSpring;
	b2Vec2 linearImpulse;
	float angularImpulse;

	int indexA;
	int indexB;
	b2Transform frameA;
	b2Transform frameB;
	b2Vec2 deltaCenter;
	float axialMass;
} b2WeldJoint;

typedef struct b2MotorJoint
{
	b2Vec2 linearVelocity;
	float maxVelocityForce;
	float angularVelocity;
	float maxVelocityTorque;
	float linearHertz;
	float linearDampingRatio;
	float maxSpringForce;
	float angularHertz;
	float angularDampingRatio;
	float maxSpringTorque;

	b2Vec2 linearVelocityImpulse;
	float angularVelocityImpulse;
	b2Vec2 linearSpringImpulse;
	float angularSpringImpulse;

	b2Softness linearSpring;
	b2Softness angularSpring;

	int indexA;
	int indexB;
	b2Transform frameA;
	b2Transform frameB;
	b2Vec2 deltaCenter;
	b2Mat22 linearMass;
	float angularMass;
} b2MotorJoint;

typedef struct b2MoverJoint
{
	b2Vec2 linearVelocity;
	b2Vec2 maxVelocityForce;
	b2Vec2 linearVelocityImpulse;
	int indexA;
	int indexB;
	float linearMass;
} b2MoverJoint;

typedef struct b2PogoJoint
{
	b2Vec2 normal;
	float restLength;
	float hertz;
	float dampingRatio;
	float maxTensionForce;
	float maxCompressionForce;
	float impulse;
	int indexA;
	int indexB;
	b2Transform frameA;
	b2Transform frameB;
	b2Vec2 deltaCenter;
	float linearMass;
	float velocity;
} b2PogoJoint;

typedef struct b2JointSim
{
	int jointId;

	int bodyIdA;
	int bodyIdB;

	b2JointType type;

	b2Transform localFrameA;
	b2Transform localFrameB;

	float invMassA, invMassB;
	float invIA, invIB;

	float constraintHertz;
	float constraintDampingRatio;

	b2Softness constraintSoftness;

	float forceThreshold;
	float torqueThreshold;

	union { b2DistanceJoint distanceJoint;b2RevoluteJoint revoluteJoint;b2PrismaticJoint prismaticJoint;b2WheelJoint wheelJoint;b2WeldJoint weldJoint;b2MotorJoint motorJoint;b2MoverJoint moverJoint;b2PogoJoint pogoJoint; };

} b2JointSim;
b2Joint* b2GetJointFullId(b2World* world,b2JointId id);
b2JointSim* b2GetJointSim(b2World* world,b2Joint* joint);
b2JointId b2CreateJointRecord(b2World* world,b2BodyId bodyIdA,b2BodyId bodyIdB);
void b2DestroyJointInternal(b2World* world,b2Joint* joint);
void b2UnlinkBodyJoints(b2World* world,b2Body* body);
void b2RelinkBodyJoints(b2World* world,b2Body* body);

b2JointSim* b2GetJointSimCheckType(b2JointId id,b2JointType type);
b2Vec2 b2GetJointConstraintForce(b2World* world,b2Joint* joint);
float b2GetJointConstraintTorque(b2World* world,b2Joint* joint);
void b2PrepareJoint(b2JointSim* joint,b2StepContext* context);
void b2WarmStartJoint(b2JointSim* joint,b2StepContext* context);
void b2SolveJoint(b2JointSim* joint,b2StepContext* context,bool useBias);
void b2PrepareDistanceJoint(b2JointSim* joint,b2StepContext* context);
void b2WarmStartDistanceJoint(b2JointSim* joint,b2StepContext* context);
void b2SolveDistanceJoint(b2JointSim* joint,b2StepContext* context,bool useBias);

void b2PrepareRevoluteJoint(b2JointSim* base,b2StepContext* context);
void b2WarmStartRevoluteJoint(b2JointSim* base,b2StepContext* context);
void b2SolveRevoluteJoint(b2JointSim* base,b2StepContext* context,bool useBias);

void b2PreparePrismaticJoint(b2JointSim* base,b2StepContext* context);
void b2WarmStartPrismaticJoint(b2JointSim* base,b2StepContext* context);
void b2SolvePrismaticJoint(b2JointSim* base,b2StepContext* context,bool useBias);

void b2PrepareWheelJoint(b2JointSim* base,b2StepContext* context);
void b2WarmStartWheelJoint(b2JointSim* base,b2StepContext* context);
void b2SolveWheelJoint(b2JointSim* base,b2StepContext* context,bool useBias);

void b2PrepareWeldJoint(b2JointSim* base,b2StepContext* context);
void b2WarmStartWeldJoint(b2JointSim* base,b2StepContext* context);
void b2SolveWeldJoint(b2JointSim* base,b2StepContext* context,bool useBias);

void b2PrepareMotorJoint(b2JointSim* base,b2StepContext* context);
void b2WarmStartMotorJoint(b2JointSim* base,b2StepContext* context);
void b2SolveMotorJoint(b2JointSim* base,b2StepContext* context);

void b2PrepareMoverJoint(b2JointSim* base,b2StepContext* context);
void b2WarmStartMoverJoint(b2JointSim* base,b2StepContext* context);
void b2SolveMoverJoint(b2JointSim* base,b2StepContext* context);

void b2PreparePogoJoint(b2JointSim* base,b2StepContext* context);
void b2WarmStartPogoJoint(b2JointSim* base,b2StepContext* context);
void b2SolvePogoJoint(b2JointSim* base,b2StepContext* context,bool useBias);

void b2DrawJoint(b2DebugDraw* draw,b2World* world,b2Joint* joint);
void b2DrawDistanceJoint(b2DebugDraw* draw,b2JointSim* base,b2WorldTransform transformA,b2WorldTransform transformB);
void b2DrawPrismaticJoint(b2DebugDraw* draw,b2JointSim* base,b2WorldTransform transformA,b2WorldTransform transformB,float drawScale);
void b2DrawRevoluteJoint(b2DebugDraw* draw,b2JointSim* base,b2WorldTransform transformA,b2WorldTransform transformB,float drawScale);
void b2DrawWeldJoint(b2DebugDraw* draw,b2JointSim* base,b2WorldTransform transformA,b2WorldTransform transformB,float drawScale);
void b2DrawWheelJoint(b2DebugDraw* draw,b2JointSim* base,b2WorldTransform transformA,b2WorldTransform transformB,float drawScale);

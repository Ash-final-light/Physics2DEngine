// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "joint.h"
#include "world.h"

void b2PrepareWheelJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_wheelJoint );

	// 按刚体 id 找到其所在的求解器集合
	int idA = base->bodyIdA;
	int idB = base->bodyIdB;

	b2World* world = context->world;

	b2Body* bodyA = b2Array_Get( world->bodies, idA );
	b2Body* bodyB = b2Array_Get( world->bodies, idB );

	B2_ASSERT( bodyA->setIndex == b2_awakeSet || bodyB->setIndex == b2_awakeSet );
	b2SolverSet* setA = b2Array_Get( world->solverSets, bodyA->setIndex );
	b2SolverSet* setB = b2Array_Get( world->solverSets, bodyB->setIndex );

	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;

	b2BodySim* bodySimA = b2Array_Get( setA->bodySims, localIndexA );
	b2BodySim* bodySimB = b2Array_Get( setB->bodySims, localIndexB );

	float mA = bodySimA->invMass;
	float iA = bodySimA->invInertia;
	float mB = bodySimB->invMass;
	float iB = bodySimB->invInertia;

	base->invMassA = mA;
	base->invMassB = mB;
	base->invIA = iA;
	base->invIB = iB;

	b2WheelJoint* joint = &base->wheelJoint;

	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// 用世界空间旋转计算关节锚点坐标系，相对于质心
	joint->frameA.q = b2MulRot( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b2RotateVector( bodySimA->transform.q, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b2MulRot( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b2RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// 计算初始质心差。增量位置更新都相对此值。
	joint->deltaCenter = b2SubPos( bodySimB->center, bodySimA->center );

	b2Vec2 rA = joint->frameA.p;
	b2Vec2 rB = joint->frameB.p;

	b2Vec2 d = b2Add( joint->deltaCenter, b2Sub( rB, rA ) );
	b2Vec2 axisA = b2RotateVector( joint->frameA.q, b2Vec2{ 1.0f, 0.0f } );
	b2Vec2 perpA = b2LeftPerp( axisA );

	// 垂直约束（使轮保持在直线上）
	float s1 = b2Cross( b2Add( d, rA ), perpA );
	float s2 = b2Cross( rB, perpA );

	float kp = mA + mB + iA * s1 * s1 + iB * s2 * s2;
	joint->perpMass = kp > 0.0f ? 1.0f / kp : 0.0f;

	// 弹簧约束
	float a1 = b2Cross( b2Add( d, rA ), axisA );
	float a2 = b2Cross( rB, axisA );

	float ka = mA + mB + iA * a1 * a1 + iB * a2 * a2;
	joint->axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	joint->springSoftness = b2MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	float km = iA + iB;
	joint->motorMass = km > 0.0f ? 1.0f / km : 0.0f;

	if ( context->world->enableWarmStarting == false )
	{
		joint->perpImpulse = 0.0f;
		joint->springImpulse = 0.0f;
		joint->motorImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}
}

void b2WarmStartWheelJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_wheelJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// 静态刚体的占位状态
	b2BodyState dummyState = b2_identityBodyState;

	b2WheelJoint* joint = &base->wheelJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	b2Vec2 d = b2Add( b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b2Sub( rB, rA ) );
	b2Vec2 axisA = b2RotateVector( joint->frameA.q, b2Vec2{ 1.0f, 0.0f } );
	axisA = b2RotateVector( stateA->deltaRotation, axisA );
	b2Vec2 perpA = b2LeftPerp( axisA );

	float a1 = b2Cross( b2Add( d, rA ), axisA );
	float a2 = b2Cross( rB, axisA );
	float s1 = b2Cross( b2Add( d, rA ), perpA );
	float s2 = b2Cross( rB, perpA );

	float axialImpulse = joint->springImpulse + joint->lowerImpulse - joint->upperImpulse;

	b2Vec2 P = b2Add( b2MulSV( axialImpulse, axisA ), b2MulSV( joint->perpImpulse, perpA ) );
	float LA = axialImpulse * a1 + joint->perpImpulse * s1 + joint->motorImpulse;
	float LB = axialImpulse * a2 + joint->perpImpulse * s2 + joint->motorImpulse;

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, P );
		stateA->angularVelocity -= iA * LA;
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, P );
		stateB->angularVelocity += iB * LB;
	}
}

void b2SolveWheelJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_ASSERT( base->type == b2_wheelJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// 静态刚体的占位状态
	b2BodyState dummyState = b2_identityBodyState;

	b2WheelJoint* joint = &base->wheelJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	bool fixedRotation = ( iA + iB == 0.0f );

	// 当前锚点
	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	b2Vec2 d = b2Add( b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b2Sub( rB, rA ) );
	b2Vec2 axisA = b2RotateVector( joint->frameA.q, b2Vec2{ 1.0f, 0.0f } );
	axisA = b2RotateVector( stateA->deltaRotation, axisA );
	float translation = b2Dot( axisA, d );

	float a1 = b2Cross( b2Add( d, rA ), axisA );
	float a2 = b2Cross( rB, axisA );

	// 马达约束
	if ( joint->enableMotor && fixedRotation == false )
	{
		float Cdot = wB - wA - joint->motorSpeed;
		float impulse = -joint->motorMass * Cdot;
		float oldImpulse = joint->motorImpulse;
		float maxImpulse = context->h * joint->maxMotorTorque;
		joint->motorImpulse = b2ClampFloat( joint->motorImpulse + impulse, -maxImpulse, maxImpulse );
		impulse = joint->motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// 弹簧约束
	if ( joint->enableSpring )
	{
		// 这是真实弹簧，即使在松弛阶段也应施加
		float C = translation;
		float bias = joint->springSoftness.biasRate * C;
		float massScale = joint->springSoftness.massScale;
		float impulseScale = joint->springSoftness.impulseScale;

		float Cdot = b2Dot( axisA, b2Sub( vB, vA ) ) + a2 * wB - a1 * wA;
		float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->springImpulse;
		joint->springImpulse += impulse;

		b2Vec2 P = b2MulSV( impulse, axisA );
		float LA = impulse * a1;
		float LB = impulse * a2;

		vA = b2MulSub( vA, mA, P );
		wA -= iA * LA;
		vB = b2MulAdd( vB, mB, P );
		wB += iB * LB;
	}

	if ( joint->enableLimit )
	{
		// 下限
		{
			float C = translation - joint->lowerTranslation;
			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;

			if ( C > 0.0f )
			{
				// 预测性
				bias = C * context->inv_h;
			}
			else if ( useBias )
			{
				bias = base->constraintSoftness.biasRate * C;
				massScale = base->constraintSoftness.massScale;
				impulseScale = base->constraintSoftness.impulseScale;
			}

			float Cdot = b2Dot( axisA, b2Sub( vB, vA ) ) + a2 * wB - a1 * wA;
			float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->lowerImpulse;
			float oldImpulse = joint->lowerImpulse;
			joint->lowerImpulse = b2MaxFloat( oldImpulse + impulse, 0.0f );
			impulse = joint->lowerImpulse - oldImpulse;

			b2Vec2 P = b2MulSV( impulse, axisA );
			float LA = impulse * a1;
			float LB = impulse * a2;

			vA = b2MulSub( vA, mA, P );
			wA -= iA * LA;
			vB = b2MulAdd( vB, mB, P );
			wB += iB * LB;
		}

		// 上限
		// 注意：翻转符号，使约束满足时 C 为正。
		// 这也使限位激活时冲量为正。
		{
			// 符号翻转
			float C = joint->upperTranslation - translation;
			float bias = 0.0f;
			float massScale = 1.0f;
			float impulseScale = 0.0f;

			if ( C > 0.0f )
			{
				// 预测性
				bias = C * context->inv_h;
			}
			else if ( useBias )
			{
				bias = base->constraintSoftness.biasRate * C;
				massScale = base->constraintSoftness.massScale;
				impulseScale = base->constraintSoftness.impulseScale;
			}

			// Cdot 符号翻转
			float Cdot = b2Dot( axisA, b2Sub( vA, vB ) ) + a1 * wA - a2 * wB;
			float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->upperImpulse;
			float oldImpulse = joint->upperImpulse;
			joint->upperImpulse = b2MaxFloat( oldImpulse + impulse, 0.0f );
			impulse = joint->upperImpulse - oldImpulse;

			b2Vec2 P = b2MulSV( impulse, axisA );
			float LA = impulse * a1;
			float LB = impulse * a2;

			// 施加冲量时符号翻转
			vA = b2MulAdd( vA, mA, P );
			wA += iA * LA;
			vB = b2MulSub( vB, mB, P );
			wB -= iB * LB;
		}
	}

	// 点到线约束
	{
		b2Vec2 perpA = b2LeftPerp( axisA );

		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias )
		{
			float C = b2Dot( perpA, d );
			bias = base->constraintSoftness.biasRate * C;
			massScale = base->constraintSoftness.massScale;
			impulseScale = base->constraintSoftness.impulseScale;
		}

		float s1 = b2Cross( b2Add( d, rA ), perpA );
		float s2 = b2Cross( rB, perpA );
		float Cdot = b2Dot( perpA, b2Sub( vB, vA ) ) + s2 * wB - s1 * wA;

		float impulse = -massScale * joint->perpMass * ( Cdot + bias ) - impulseScale * joint->perpImpulse;
		joint->perpImpulse += impulse;

		b2Vec2 P = b2MulSV( impulse, perpA );
		float LA = impulse * s1;
		float LB = impulse * s2;

		vA = b2MulSub( vA, mA, P );
		wA -= iA * LA;
		vB = b2MulAdd( vB, mB, P );
		wB += iB * LB;
	}

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = vA;
		stateA->angularVelocity = wA;
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = vB;
		stateB->angularVelocity = wB;
	}
}

void b2WheelJoint_EnableSpring( b2JointId jointId, bool enableSpring )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );

	if ( enableSpring != joint->wheelJoint.enableSpring )
	{
		joint->wheelJoint.enableSpring = enableSpring;
		joint->wheelJoint.springImpulse = 0.0f;
	}
}

bool b2WheelJoint_IsSpringEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.enableSpring;
}

void b2WheelJoint_SetSpringHertz( b2JointId jointId, float hertz )
{
	if (!(b2IsValidFloat( hertz ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	joint->wheelJoint.hertz = hertz;
}

float b2WheelJoint_GetSpringHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.hertz;
}

void b2WheelJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio )
{
	if (!(b2IsValidFloat( dampingRatio ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	joint->wheelJoint.dampingRatio = dampingRatio;
}

float b2WheelJoint_GetSpringDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.dampingRatio;
}

void b2WheelJoint_EnableLimit( b2JointId jointId, bool enableLimit )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	if ( joint->wheelJoint.enableLimit != enableLimit )
	{
		joint->wheelJoint.lowerImpulse = 0.0f;
		joint->wheelJoint.upperImpulse = 0.0f;
		joint->wheelJoint.enableLimit = enableLimit;
	}
}

bool b2WheelJoint_IsLimitEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.enableLimit;
}

void b2WheelJoint_SetLimits( b2JointId jointId, float lower, float upper )
{
	if (!(b2IsValidFloat( lower ))) { return; }
	if (!(b2IsValidFloat( upper ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	B2_ASSERT( lower <= upper );

	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	joint->wheelJoint.lowerTranslation = b2MinFloat( lower, upper );
	joint->wheelJoint.upperTranslation = b2MaxFloat( lower, upper );
}

float b2WheelJoint_GetLowerLimit( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.lowerTranslation;
}

float b2WheelJoint_GetUpperLimit( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.upperTranslation;
}

void b2WheelJoint_EnableMotor( b2JointId jointId, bool enableMotor )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	if ( joint->wheelJoint.enableMotor != enableMotor )
	{
		joint->wheelJoint.motorImpulse = 0.0f;
		joint->wheelJoint.enableMotor = enableMotor;
	}
}

bool b2WheelJoint_IsMotorEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.enableMotor;
}

void b2WheelJoint_SetTargetMotorSpeed( b2JointId jointId, float motorSpeed )
{
	if (!(b2IsValidFloat( motorSpeed ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	joint->wheelJoint.motorSpeed = motorSpeed;
}

float b2WheelJoint_GetTargetMotorSpeed( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.motorSpeed;
}

float b2WheelJoint_GetCurrentMotorTorque( b2JointId jointId )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return world->inv_h * joint->wheelJoint.motorImpulse;
}

void b2WheelJoint_SetMaxMotorTorque( b2JointId jointId, float torque )
{
	if (!(b2IsValidFloat( torque ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	joint->wheelJoint.maxMotorTorque = torque;
}

float b2WheelJoint_GetMaxMotorTorque( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_wheelJoint );
	return joint->wheelJoint.maxMotorTorque;
}

void b2DrawWheelJoint( b2DebugDraw* draw, b2JointSim* base, b2WorldTransform transformA, b2WorldTransform transformB,
					   float drawScale )
{
	B2_ASSERT( base->type == b2_wheelJoint );

	b2WheelJoint* joint = &base->wheelJoint;

	b2WorldTransform frameA = b2MulTransforms( transformA, base->localFrameA );
	b2WorldTransform frameB = b2MulTransforms( transformB, base->localFrameB );
	b2Vec2 axisA = b2RotateVector( frameA.q, b2Vec2{ 1.0f, 0.0f } );

	b2HexColor c1 = b2_colorGray;
	b2HexColor c2 = b2_colorGreen;
	b2HexColor c3 = b2_colorRed;
	b2HexColor c4 = b2_colorDimGray;
	b2HexColor c5 = b2_colorBlue;

	draw->DrawLineFcn( frameA.p, frameB.p, c5, draw->context );

	if ( joint->enableLimit )
	{
		b2Pos lower = b2OffsetPos( frameA.p, b2MulSV( joint->lowerTranslation, axisA ) );
		b2Pos upper = b2OffsetPos( frameA.p, b2MulSV( joint->upperTranslation, axisA ) );
		b2Vec2 perp = b2LeftPerp( axisA );
		draw->DrawLineFcn( lower, upper, c1, draw->context );
		draw->DrawLineFcn( b2OffsetPos( lower, b2MulSV( -0.1f * drawScale, perp ) ),
						   b2OffsetPos( lower, b2MulSV( 0.1f * drawScale, perp ) ), c2, draw->context );
		draw->DrawLineFcn( b2OffsetPos( upper, b2MulSV( -0.1f * drawScale, perp ) ),
						   b2OffsetPos( upper, b2MulSV( 0.1f * drawScale, perp ) ), c3, draw->context );
	}
	else
	{
		draw->DrawLineFcn( b2OffsetPos( frameA.p, b2Neg( axisA ) ), b2OffsetPos( frameA.p, axisA ), c1, draw->context );
	}

	draw->DrawPointFcn( frameA.p, 5.0f, c1, draw->context );
	draw->DrawPointFcn( frameB.p, 5.0f, c4, draw->context );
}

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "joint.h"
#include "world.h"

void b2PrepareRevoluteJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_revoluteJoint );

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

	b2RevoluteJoint* joint = &base->revoluteJoint;

	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// 用世界空间旋转计算关节锚点坐标系，相对于质心。
	// 尽量避免此处的舍入误差。
	// b2Vec2 pf = (xf.p - c) + rot(xf.q, f.p)
	// pf = xf.p - (xf.p + rot(xf.q, lc)) + rot(xf.q, f.p)
	// pf = rot(xf.q, f.p - lc)
	joint->frameA.q = b2MulRot( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b2RotateVector( bodySimA->transform.q, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b2MulRot( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b2RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// 计算初始质心差。增量位置更新都相对此值。
	joint->deltaCenter = b2SubPos( bodySimB->center, bodySimA->center );

	float k = iA + iB;
	joint->axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	joint->springSoftness = b2MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	if ( context->world->enableWarmStarting == false )
	{
		joint->linearImpulse = b2Vec2_zero;
		joint->springImpulse = 0.0f;
		joint->motorImpulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
	}
}

void b2WarmStartRevoluteJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_revoluteJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// 静态刚体的占位状态
	b2BodyState dummyState = b2_identityBodyState;

	b2RevoluteJoint* joint = &base->revoluteJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	float axialImpulse = joint->springImpulse + joint->motorImpulse + joint->lowerImpulse - joint->upperImpulse;

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, joint->linearImpulse );
		stateA->angularVelocity -= iA * ( b2Cross( rA, joint->linearImpulse ) + axialImpulse );
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, joint->linearImpulse );
		stateB->angularVelocity += iB * ( b2Cross( rB, joint->linearImpulse ) + axialImpulse );
	}
}

void b2SolveRevoluteJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_ASSERT( base->type == b2_revoluteJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// 静态刚体的占位状态
	b2BodyState dummyState = b2_identityBodyState;

	b2RevoluteJoint* joint = &base->revoluteJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	b2Rot qA = b2MulRot( stateA->deltaRotation, joint->frameA.q );
	b2Rot qB = b2MulRot( stateB->deltaRotation, joint->frameB.q );
	b2Rot relQ = b2InvMulRot( qA, qB );

	// 当前锚点
	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	b2Vec2 linearCdot = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );
	float angularCdot = wB - wA;
	float axialImpulse = 0.0f;

	float k = iA + iB;
	bool fixedRotation = ( k == 0.0f );

	// 求解弹簧。
	if ( joint->enableSpring && fixedRotation == false )
	{
		float jointAngle = b2Rot_GetAngle( relQ );
		float jointAngleDelta = b2UnwindAngle( jointAngle - joint->targetAngle );

		float C = jointAngleDelta;
		float bias = joint->springSoftness.biasRate * C;
		float massScale = joint->springSoftness.massScale;
		float impulseScale = joint->springSoftness.impulseScale;

		float Cdot = angularCdot;
		float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->springImpulse;
		joint->springImpulse += impulse;

		angularCdot += k * impulse;
		axialImpulse += impulse;
	}

	// 求解马达约束。
	if ( joint->enableMotor && fixedRotation == false )
	{
		float Cdot = angularCdot - joint->motorSpeed;
		float oldImpulse = joint->motorImpulse;
		float maxImpulse = context->h * joint->maxMotorTorque;
		float impulse = b2ClampFloat( -joint->axialMass * Cdot, -maxImpulse - oldImpulse, maxImpulse - oldImpulse );
		joint->motorImpulse = oldImpulse + impulse;

		angularCdot += k * impulse;
		axialImpulse += impulse;
	}

	if ( joint->enableLimit && fixedRotation == false )
	{
		float jointAngle = b2Rot_GetAngle( relQ );

		// 下限
		{
			float C = jointAngle - joint->lowerAngle;
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

			float Cdot = angularCdot;
			float oldImpulse = joint->lowerImpulse;
			float impulse = b2MaxFloat( -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * oldImpulse, -oldImpulse );
			joint->lowerImpulse = oldImpulse + impulse;

			angularCdot += k * impulse;
			axialImpulse += impulse;
		}

		// 上限
		// 注意：翻转符号，使约束满足时 C 为正。
		// 这也使限位激活时冲量为正。
		{
			float C = joint->upperAngle - jointAngle;
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
			float Cdot = -angularCdot;
			float oldImpulse = joint->upperImpulse;
			float impulse = b2MaxFloat( -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * oldImpulse, -oldImpulse );
			joint->upperImpulse = oldImpulse + impulse;

			// 施加冲量时符号翻转
			axialImpulse -= impulse;
		}
	}

	// 求解点对点约束
	{
		// J = [-I -r1_skew I r2_skew]
		// r_skew = [-ry; rx]
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

		b2Vec2 Cdot = b2MulAdd( linearCdot, axialImpulse, b2Add( b2CrossSV( iA, rA ), b2CrossSV( iB, rB ) ) );

		b2Vec2 bias = b2Vec2_zero;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias )
		{
			b2Vec2 dcA = stateA->deltaPosition;
			b2Vec2 dcB = stateB->deltaPosition;

			b2Vec2 separation = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );
			bias = b2MulSV( base->constraintSoftness.biasRate, separation );
			massScale = base->constraintSoftness.massScale;
			impulseScale = base->constraintSoftness.impulseScale;
		}

		b2Mat22 K;
		K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.cx.y = K.cy.x;
		K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		b2Vec2 b = b2Solve22( K, b2Add( Cdot, bias ) );

		b2Vec2 impulse;
		impulse.x = -massScale * b.x - impulseScale * joint->linearImpulse.x;
		impulse.y = -massScale * b.y - impulseScale * joint->linearImpulse.y;
		joint->linearImpulse.x += impulse.x;
		joint->linearImpulse.y += impulse.y;

		vA = b2MulSub( vA, mA, impulse );
		wA -= iA * ( b2Cross( rA, impulse ) + axialImpulse );
		vB = b2MulAdd( vB, mB, impulse );
		wB += iB * ( b2Cross( rB, impulse ) + axialImpulse );
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


float b2RevoluteJoint_GetCurrentAngle( b2JointId jointId )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* jointSim = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	b2Rot qA = b2MulRot( b2GetBodySim(world,world->bodies.data+jointSim->bodyIdA)->transform.q, jointSim->localFrameA.q );
	b2Rot qB = b2MulRot( b2GetBodySim(world,world->bodies.data+jointSim->bodyIdB)->transform.q, jointSim->localFrameB.q );

	float angle = b2RelativeAngle( qA, qB );
	return angle;
}

float b2RevoluteJoint_GetCurrentSpeed( b2JointId jointId )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_revoluteJoint );

	b2Body* bodyA = b2Array_Get( world->bodies, base->bodyIdA );
	b2Body* bodyB = b2Array_Get( world->bodies, base->bodyIdB );
	b2BodyState* bodyStateA = b2GetBodyState( world, bodyA );
	b2BodyState* bodyStateB = b2GetBodyState( world, bodyB );

	float wA = bodyStateA ? bodyStateA->angularVelocity : 0.0f;
	float wB = bodyStateB ? bodyStateB->angularVelocity : 0.0f;

	return wB - wA;
}

void b2RevoluteJoint_EnableSpring( b2JointId jointId, bool enableSpring )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( enableSpring != joint->revoluteJoint.enableSpring )
	{
		joint->revoluteJoint.enableSpring = enableSpring;
		joint->revoluteJoint.springImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsSpringEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.enableSpring;
}

void b2RevoluteJoint_SetSpringHertz( b2JointId jointId, float hertz )
{
	if (!(b2IsValidFloat( hertz ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.hertz = hertz;
}

float b2RevoluteJoint_GetSpringHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.hertz;
}

void b2RevoluteJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio )
{
	if (!(b2IsValidFloat( dampingRatio ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.dampingRatio = dampingRatio;
}

float b2RevoluteJoint_GetSpringDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.dampingRatio;
}

void b2RevoluteJoint_SetTargetAngle( b2JointId jointId, float angle )
{
	if (!(b2IsValidFloat( angle ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.targetAngle = angle;
}

float b2RevoluteJoint_GetTargetAngle( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.targetAngle;
}

void b2RevoluteJoint_EnableLimit( b2JointId jointId, bool enableLimit )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( enableLimit != joint->revoluteJoint.enableLimit )
	{
		joint->revoluteJoint.enableLimit = enableLimit;
		joint->revoluteJoint.lowerImpulse = 0.0f;
		joint->revoluteJoint.upperImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsLimitEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.enableLimit;
}

float b2RevoluteJoint_GetLowerLimit( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.lowerAngle;
}

float b2RevoluteJoint_GetUpperLimit( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.upperAngle;
}

void b2RevoluteJoint_SetLimits( b2JointId jointId, float lower, float upper )
{
	if (!(b2IsValidFloat( lower ))) { return; }
	if (!(b2IsValidFloat( upper ))) { return; }

	B2_ASSERT( lower <= upper );

	b2World* world = b2_worlds + jointId.world0;

	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	float lowerAngle = b2MinFloat( lower, upper );
	float upperAngle = b2MaxFloat( lower, upper );
	joint->revoluteJoint.lowerAngle = b2ClampFloat( lowerAngle, -0.99f * B2_PI, 0.99f * B2_PI );
	joint->revoluteJoint.upperAngle = b2ClampFloat( upperAngle, -0.99f * B2_PI, 0.99f * B2_PI );
}

void b2RevoluteJoint_EnableMotor( b2JointId jointId, bool enableMotor )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	if ( enableMotor != joint->revoluteJoint.enableMotor )
	{
		joint->revoluteJoint.enableMotor = enableMotor;
		joint->revoluteJoint.motorImpulse = 0.0f;
	}
}

bool b2RevoluteJoint_IsMotorEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.enableMotor;
}

void b2RevoluteJoint_SetTargetMotorSpeed( b2JointId jointId, float motorSpeed )
{
	if (!(b2IsValidFloat( motorSpeed ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.motorSpeed = motorSpeed;
}

float b2RevoluteJoint_GetTargetMotorSpeed( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.motorSpeed;
}

void b2RevoluteJoint_SetMaxMotorTorque( b2JointId jointId, float torque )
{
	if (!(b2IsValidFloat( torque ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	joint->revoluteJoint.maxMotorTorque = torque;
}

float b2RevoluteJoint_GetMaxMotorTorque( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return joint->revoluteJoint.maxMotorTorque;
}

float b2RevoluteJoint_GetCurrentMotorTorque( b2JointId jointId )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_revoluteJoint );
	return world->inv_h * joint->revoluteJoint.motorImpulse;
}

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "joint.h"
#include "recording.h"
#include "body.h"
#include "world.h"

void b2DistanceJoint_SetRestLength( b2JointId jointId, float length )
{
	if(!b2IsValidFloat(length)) { return; }

	b2World* world = b2_worlds + jointId.world0;

	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), DistanceJointSetRestLength, jointId, length );
	}
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;

	joint->length = b2ClampFloat( length, B2_LINEAR_SLOP, B2_HUGE );
	joint->impulse = 0.0f;
	joint->lowerImpulse=0;
	joint->upperImpulse=0;
}

float b2DistanceJoint_GetRestLength( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->length;
}

float b2DistanceJoint_GetCurrentLength( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );

	b2World* world = b2_worlds + jointId.world0;
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return 0.0f;
	}

	// 相对刚体 A，使差值在远离原点时仍保持浮点精度
	b2WorldTransform wxfA = b2GetBodySim(world,world->bodies.data+base->bodyIdA)->transform;
	b2Transform transformA = b2ToRelativeTransform( wxfA, wxfA.p );
	b2Transform transformB = b2ToRelativeTransform( b2GetBodySim(world,world->bodies.data+base->bodyIdB)->transform, wxfA.p );

	b2Vec2 pA = b2TransformPoint( transformA, base->localFrameA.p );
	b2Vec2 pB = b2TransformPoint( transformB, base->localFrameB.p );
	b2Vec2 d = b2Sub( pB, pA );
	float length = b2Length( d );
	return length;
}

void b2PrepareDistanceJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_distanceJoint );

	// 根据刚体 id 找到其所在求解器集合
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

	b2DistanceJoint* joint = &base->distanceJoint;

	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// 世界空间中的初始锚点
	joint->anchorA = b2RotateVector( bodySimA->transform.q, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->anchorB = b2RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );
	joint->deltaCenter = b2SubPos( bodySimB->center, bodySimA->center );

	b2Vec2 rA = joint->anchorA;
	b2Vec2 rB = joint->anchorB;
	b2Vec2 separation = b2Add( b2Sub( rB, rA ), joint->deltaCenter );
	b2Vec2 axis = b2Normalize( separation );

	// 计算有效质量
	float crA = b2Cross( rA, axis );
	float crB = b2Cross( rB, axis );
	float k = mA + mB + iA * crA * crA + iB * crB * crB;
	joint->axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	joint->distanceSoftness = b2MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	if ( context->world->enableWarmStarting == false )
	{
		joint->impulse = 0.0f;
		joint->lowerImpulse = 0.0f;
		joint->upperImpulse = 0.0f;
		joint->motorImpulse = 0.0f;
	}
}

void b2WarmStartDistanceJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_distanceJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// 静态刚体的哑状态
	b2BodyState dummyState = b2_identityBodyState;

	b2DistanceJoint* joint = &base->distanceJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->anchorA );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->anchorB );

	b2Vec2 ds = b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), b2Sub( rB, rA ) );
	b2Vec2 separation = b2Add( joint->deltaCenter, ds );
	b2Vec2 axis = b2Normalize( separation );

	float axialImpulse = joint->impulse + joint->lowerImpulse - joint->upperImpulse + joint->motorImpulse;
	b2Vec2 P = b2MulSV( axialImpulse, axis );

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, P );
		stateA->angularVelocity -= iA * b2Cross( rA, P );
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, P );
		stateB->angularVelocity += iB * b2Cross( rB, P );
	}
}

void b2SolveDistanceJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_ASSERT( base->type == b2_distanceJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// 静态刚体的哑状态
	b2BodyState dummyState = b2_identityBodyState;

	b2DistanceJoint* joint = &base->distanceJoint;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	// 当前锚点
	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->anchorA );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->anchorB );

	// 当前分离
	b2Vec2 ds = b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), b2Sub( rB, rA ) );
	b2Vec2 separation = b2Add( joint->deltaCenter, ds );

	float length = b2Length( separation );
	b2Vec2 axis = b2Normalize( separation );

	// 关节为软约束，若
	// - 弹簧已启用
	// - 且（关节限位禁用或限位上下限不相等）
	if ( joint->enableSpring && ( joint->minLength < joint->maxLength || joint->enableLimit == false ) )
	{
		// 弹簧
		if ( joint->hertz > 0.0f )
		{
			// Cdot = dot(u, v + cross(w, r))
			b2Vec2 vr = b2Add( b2Sub( vB, vA ), b2Sub( b2CrossSV( wB, rB ), b2CrossSV( wA, rA ) ) );
			float cdot = b2Dot( axis, vr );
			float c = length - joint->length;
			float bias = joint->distanceSoftness.biasRate * c;

			float m = joint->distanceSoftness.massScale * joint->axialMass;
			float oldImpulse = joint->impulse;
			float impulse = -m * ( cdot + bias ) - joint->distanceSoftness.impulseScale * oldImpulse;

			float h = context->h;
			joint->impulse = b2ClampFloat( joint->impulse + impulse, joint->lowerSpringForce * h, joint->upperSpringForce * h );
			impulse = joint->impulse - oldImpulse;

			b2Vec2 P = b2MulSV( impulse, axis );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );
			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}

		if ( joint->enableMotor )
		{
			b2Vec2 vr = b2Add( b2Sub( vB, vA ), b2Sub( b2CrossSV( wB, rB ), b2CrossSV( wA, rA ) ) );
			float Cdot = b2Dot( axis, vr );
			float impulse = joint->axialMass * ( joint->motorSpeed - Cdot );
			float oldImpulse = joint->motorImpulse;
			float maxImpulse = context->h * joint->maxMotorForce;
			joint->motorImpulse = b2ClampFloat( joint->motorImpulse + impulse, -maxImpulse, maxImpulse );
			impulse = joint->motorImpulse - oldImpulse;

			b2Vec2 P = b2MulSV( impulse, axis );
			vA = b2MulSub( vA, mA, P );
			wA -= iA * b2Cross( rA, P );
			vB = b2MulAdd( vB, mB, P );
			wB += iB * b2Cross( rB, P );
		}

		if ( joint->enableLimit )
		{
			// 下限
			{
				b2Vec2 vr = b2Add( b2Sub( vB, vA ), b2Sub( b2CrossSV( wB, rB ), b2CrossSV( wA, rA ) ) );
				float cdot = b2Dot( axis, vr );

				float c = length - joint->minLength;

				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;
				if ( c > 0.0f )
				{
					// 预测性
					bias = c * context->inv_h;
				}
				else if ( useBias )
				{
					bias = base->constraintSoftness.biasRate * c;
					massScale = base->constraintSoftness.massScale;
					impulseScale = base->constraintSoftness.impulseScale;
				}

				float impulse = -massScale * joint->axialMass * ( cdot + bias ) - impulseScale * joint->lowerImpulse;
				float newImpulse = b2MaxFloat( 0.0f, joint->lowerImpulse + impulse );
				impulse = newImpulse - joint->lowerImpulse;
				joint->lowerImpulse = newImpulse;

				b2Vec2 P = b2MulSV( impulse, axis );
				vA = b2MulSub( vA, mA, P );
				wA -= iA * b2Cross( rA, P );
				vB = b2MulAdd( vB, mB, P );
				wB += iB * b2Cross( rB, P );
			}

			// 上限
			{
				b2Vec2 vr = b2Add( b2Sub( vA, vB ), b2Sub( b2CrossSV( wA, rA ), b2CrossSV( wB, rB ) ) );
				float Cdot = b2Dot( axis, vr );

				float C = joint->maxLength - length;

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

				float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->upperImpulse;
				float newImpulse = b2MaxFloat( 0.0f, joint->upperImpulse + impulse );
				impulse = newImpulse - joint->upperImpulse;
				joint->upperImpulse = newImpulse;

				b2Vec2 P = b2MulSV( -impulse, axis );
				vA = b2MulSub( vA, mA, P );
				wA -= iA * b2Cross( rA, P );
				vB = b2MulAdd( vB, mB, P );
				wB += iB * b2Cross( rB, P );
			}
		}
	}
	else
	{
		// 刚约束
		b2Vec2 vr = b2Add( b2Sub( vB, vA ), b2Sub( b2CrossSV( wB, rB ), b2CrossSV( wA, rA ) ) );
		float Cdot = b2Dot( axis, vr );

		float C = length - joint->length;

		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias )
		{
			bias = base->constraintSoftness.biasRate * C;
			massScale = base->constraintSoftness.massScale;
			impulseScale = base->constraintSoftness.impulseScale;
		}

		float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->impulse;
		joint->impulse += impulse;

		b2Vec2 P = b2MulSV( impulse, axis );
		vA = b2MulSub( vA, mA, P );
		wA -= iA * b2Cross( rA, P );
		vB = b2MulAdd( vB, mB, P );
		wB += iB * b2Cross( rB, P );
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


void b2DistanceJoint_EnableSpring( b2JointId jointId, bool enableSpring )
{
	b2World* world = b2_worlds + jointId.world0;
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), DistanceJointEnableSpring, jointId, enableSpring );
	}
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	base->distanceJoint.enableSpring = enableSpring;
}

bool b2DistanceJoint_IsSpringEnabled( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return base->distanceJoint.enableSpring;
}

void b2DistanceJoint_SetSpringForceRange( b2JointId jointId, float lowerForce, float upperForce )
{
	if (!(b2IsValidFloat( lowerForce ))) { return; }
	if (!(b2IsValidFloat( upperForce ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	B2_ASSERT( lowerForce <= upperForce );
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), DistanceJointSetSpringForceRange, jointId, lowerForce, upperForce );
	}
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	base->distanceJoint.lowerSpringForce = b2MinFloat( lowerForce, upperForce );
	base->distanceJoint.upperSpringForce = b2MaxFloat( lowerForce, upperForce );
}

void b2DistanceJoint_GetSpringForceRange( b2JointId jointId, float* lowerForce, float* upperForce )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	*lowerForce = base->distanceJoint.lowerSpringForce;
	*upperForce = base->distanceJoint.upperSpringForce;
}

void b2DistanceJoint_SetSpringHertz( b2JointId jointId, float hertz )
{
	if (!(b2IsValidFloat( hertz ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), DistanceJointSetSpringHertz, jointId, hertz );
	}
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	base->distanceJoint.hertz = hertz;
}

void b2DistanceJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio )
{
	if (!(b2IsValidFloat( dampingRatio ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), DistanceJointSetSpringDampingRatio, jointId, dampingRatio );
	}
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	base->distanceJoint.dampingRatio = dampingRatio;
}

float b2DistanceJoint_GetSpringHertz( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->hertz;
}

float b2DistanceJoint_GetSpringDampingRatio( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->dampingRatio;
}

float b2DistanceJoint_GetSpringForce( b2JointId jointId )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	if ( joint->enableSpring )
	{
		return joint->impulse * world->inv_h;
	}

	return 0.0f;
}

void b2DistanceJoint_EnableLimit( b2JointId jointId, bool enableLimit )
{
	b2World* world = b2_worlds + jointId.world0;
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), DistanceJointEnableLimit, jointId, enableLimit );
	}
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	joint->enableLimit = enableLimit;
}

bool b2DistanceJoint_IsLimitEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return joint->distanceJoint.enableLimit;
}

void b2DistanceJoint_SetLimits( b2JointId jointId, float minLength, float maxLength )
{
	if (!(b2IsValidFloat( minLength ))) { return; }
	if (!(b2IsValidFloat( maxLength ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), DistanceJointSetLimits, jointId, minLength, maxLength );
	}
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;

	minLength = b2ClampFloat( minLength, B2_LINEAR_SLOP, B2_HUGE );
	maxLength = b2ClampFloat( maxLength, B2_LINEAR_SLOP, B2_HUGE );
	joint->minLength = b2MinFloat( minLength, maxLength );
	joint->maxLength = b2MaxFloat( minLength, maxLength );
}

float b2DistanceJoint_GetLowerLimit( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->minLength;
}

float b2DistanceJoint_GetUpperLimit( b2JointId jointId )
{
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;
	return joint->maxLength;
}

void b2DistanceJoint_EnableMotor( b2JointId jointId, bool enableMotor )
{
	b2World* world = b2_worlds + jointId.world0;
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), DistanceJointEnableMotor, jointId, enableMotor );
	}
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	if ( enableMotor != joint->distanceJoint.enableMotor )
	{
		joint->distanceJoint.enableMotor = enableMotor;
		joint->distanceJoint.motorImpulse = 0.0f;
	}
}

bool b2DistanceJoint_IsMotorEnabled( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return joint->distanceJoint.enableMotor;
}

void b2DistanceJoint_SetTargetMotorSpeed( b2JointId jointId, float motorSpeed )
{
	if (!(b2IsValidFloat( motorSpeed ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), DistanceJointSetTargetMotorSpeed, jointId, motorSpeed );
	}
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	joint->distanceJoint.motorSpeed = motorSpeed;
}

float b2DistanceJoint_GetTargetMotorSpeed( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return joint->distanceJoint.motorSpeed;
}

float b2DistanceJoint_GetCurrentMotorForce( b2JointId jointId )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return world->inv_h * base->distanceJoint.motorImpulse;
}

void b2DistanceJoint_SetMaxMotorForce( b2JointId jointId, float force )
{
	if (!(b2IsValidFloat( force ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), DistanceJointSetMaxMotorForce, jointId, force );
	}
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	joint->distanceJoint.maxMotorForce = force;
}

float b2DistanceJoint_GetMaxMotorForce( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	return joint->distanceJoint.maxMotorForce;
}

void b2DrawDistanceJoint( b2DebugDraw* draw, b2JointSim* base, b2WorldTransform transformA, b2WorldTransform transformB )
{
	B2_ASSERT( base->type == b2_distanceJoint );

	b2DistanceJoint* joint = &base->distanceJoint;

	b2Pos pA = b2TransformWorldPoint( transformA, base->localFrameA.p );
	b2Pos pB = b2TransformWorldPoint( transformB, base->localFrameB.p );

	b2Vec2 axis = b2Normalize( b2SubPos( pB, pA ) );

	if ( joint->minLength < joint->maxLength && joint->enableLimit )
	{
		b2Pos pMin = b2OffsetPos( pA, b2MulSV( joint->minLength, axis ) );
		b2Pos pMax = b2OffsetPos( pA, b2MulSV( joint->maxLength, axis ) );
		b2Vec2 offset = b2MulSV( 0.05f * b2GetLengthUnitsPerMeter(), b2RightPerp( axis ) );

		if ( joint->minLength > B2_LINEAR_SLOP )
		{
			draw->DrawLineFcn( b2OffsetPos( pMin, b2Neg( offset ) ), b2OffsetPos( pMin, offset ), b2_colorLightGreen,
							   draw->context );
		}

		if ( joint->maxLength < B2_HUGE )
		{
			draw->DrawLineFcn( b2OffsetPos( pMax, b2Neg( offset ) ), b2OffsetPos( pMax, offset ), b2_colorRed, draw->context );
		}

		if ( joint->minLength > B2_LINEAR_SLOP && joint->maxLength < B2_HUGE )
		{
			draw->DrawLineFcn( pMin, pMax, b2_colorGray, draw->context );
		}
	}

	draw->DrawLineFcn( pA, pB, b2_colorWhite, draw->context );
	draw->DrawPointFcn( pA, 4.0f, b2_colorWhite, draw->context );
	draw->DrawPointFcn( pB, 4.0f, b2_colorWhite, draw->context );

	if ( joint->hertz > 0.0f && joint->enableSpring )
	{
		b2Pos pRest = b2OffsetPos( pA, b2MulSV( joint->length, axis ) );
		draw->DrawPointFcn( pRest, 4.0f, b2_colorBlue, draw->context );
	}
}

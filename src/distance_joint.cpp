// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "joint.h"
#include "body.h"
#include "world.h"

void b2DistanceJoint_SetRestLength( b2JointId jointId, float length )
{
	if(!b2IsValidFloat(length)) { return; }

	b2World* world = b2_worlds + jointId.world0;

	b2JointSim* base = b2GetJointSimCheckType( jointId, b2_distanceJoint );
	b2DistanceJoint* joint = &base->distanceJoint;

	joint->length = b2ClampFloat( length, B2_LINEAR_SLOP, B2_HUGE );
	joint->impulse = 0.0f;
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


	if ( context->world->enableWarmStarting == false )
	{
		joint->impulse = 0.0f;
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

	float axialImpulse = joint->impulse;
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

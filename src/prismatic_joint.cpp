// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "joint.h"
#include "world.h"

void b2PreparePrismaticJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_prismaticJoint );

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

	b2PrismaticJoint* joint = &base->prismaticJoint;
	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// 用世界空间旋转计算关节锚点坐标系，相对于质心
	joint->frameA.q = b2MulRot( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b2RotateVector( bodySimA->transform.q, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b2MulRot( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b2RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// 计算初始质心差。增量位置更新都相对此值。
	joint->deltaCenter = b2SubPos( bodySimB->center, bodySimA->center );


	if ( context->world->enableWarmStarting == false )
	{
		joint->impulse = b2Vec2_zero;
	}
}

void b2WarmStartPrismaticJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_prismaticJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// 静态刚体的占位状态
	b2BodyState dummyState = b2_identityBodyState;

	b2PrismaticJoint* joint = &base->prismaticJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	b2Vec2 d = b2Add( b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b2Sub( rB, rA ) );

	b2Vec2 axisA = b2RotateVector( joint->frameA.q, b2Vec2{ 1.0f, 0.0f } );
	axisA = b2RotateVector( stateA->deltaRotation, axisA );

	// 冲量施加在刚体 B 的锚点上
	float a1 = b2Cross( b2Add( rA, d ), axisA );
	float a2 = b2Cross( rB, axisA );
	float axialImpulse = 0.0f;

	// 垂直约束
	b2Vec2 perpA = b2LeftPerp( axisA );
	float s1 = b2Cross( b2Add( rA, d ), perpA );
	float s2 = b2Cross( rB, perpA );
	float perpImpulse = joint->impulse.x;
	float angleImpulse = joint->impulse.y;

	b2Vec2 P = b2Add( b2MulSV( axialImpulse, axisA ), b2MulSV( perpImpulse, perpA ) );
	float LA = axialImpulse * a1 + perpImpulse * s1 + angleImpulse;
	float LB = axialImpulse * a2 + perpImpulse * s2 + angleImpulse;

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

void b2SolvePrismaticJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_ASSERT( base->type == b2_prismaticJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// 静态刚体的占位状态
	b2BodyState dummyState = b2_identityBodyState;

	b2PrismaticJoint* joint = &base->prismaticJoint;

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

	b2Vec2 d = b2Add( b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b2Sub( rB, rA ) );

	b2Vec2 axisA = b2RotateVector( joint->frameA.q, b2Vec2{ 1.0f, 0.0f } );
	axisA = b2RotateVector( stateA->deltaRotation, axisA );
	float translation = b2Dot( axisA, d );

	// 这些标量对应轴向力产生的力矩
	float a1 = b2Cross( b2Add( rA, d ), axisA );
	float a2 = b2Cross( rB, axisA );

	float k = mA + mB + iA * a1 * a1 + iB * a2 * a2;
	float axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	b2Softness softness = base->constraintSoftness;









	// 以块形式求解移动关节约束
	{
		b2Vec2 perpA = b2LeftPerp( axisA );

		// 这些标量对应垂直约束力产生的力矩
		float s1 = b2Cross( b2Add( d, rA ), perpA );
		float s2 = b2Cross( rB, perpA );

		b2Vec2 Cdot;
		Cdot.x = b2Dot( perpA, b2Sub( vB, vA ) ) + s2 * wB - s1 * wA;
		Cdot.y = wB - wA;

		b2Vec2 bias = b2Vec2_zero;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias )
		{
			b2Vec2 C;
			C.x = b2Dot( perpA, d );
			C.y = b2Rot_GetAngle( relQ );

			bias = b2MulSV( softness.biasRate, C );
			massScale = softness.massScale;
			impulseScale = softness.impulseScale;
		}

		float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
		float k12 = iA * s1 + iB * s2;
		float k22 = iA + iB;
		if ( k22 == 0.0f )
		{
			// 用于旋转被锁定的刚体。
			k22 = 1.0f;
		}

		b2Mat22 K = { { k11, k12 }, { k12, k22 } };

		b2Vec2 b = b2Solve22( K, b2Add( Cdot, bias ) );
		b2Vec2 deltaImpulse;
		deltaImpulse.x = -massScale * b.x - impulseScale * joint->impulse.x;
		deltaImpulse.y = -massScale * b.y - impulseScale * joint->impulse.y;

		joint->impulse.x += deltaImpulse.x;
		joint->impulse.y += deltaImpulse.y;

		b2Vec2 P = b2MulSV( deltaImpulse.x, perpA );
		float LA = deltaImpulse.x * s1 + deltaImpulse.y;
		float LB = deltaImpulse.x * s2 + deltaImpulse.y;

		vA = b2MulSub( vA, mA, P );
		wA -= iA * LA;
		vB = b2MulAdd( vB, mB, P );
		wB += iB * LB;
	}

	B2_ASSERT( b2IsValidVec2( vA ) );
	B2_ASSERT( b2IsValidFloat( wA ) );
	B2_ASSERT( b2IsValidVec2( vB ) );
	B2_ASSERT( b2IsValidFloat( wB ) );

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


float b2PrismaticJoint_GetCurrentTranslation( b2JointId jointId )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* jointSim = b2GetJointSimCheckType( jointId, b2_prismaticJoint );
	// 相对于刚体 A，使远离原点时差值仍保持 float 精度
	b2WorldTransform wxfA = b2GetBodySim(world,world->bodies.data+jointSim->bodyIdA)->transform;
	b2Transform transformA = b2ToRelativeTransform( wxfA, wxfA.p );
	b2Transform transformB = b2ToRelativeTransform( b2GetBodySim(world,world->bodies.data+jointSim->bodyIdB)->transform, wxfA.p );

	b2Vec2 localAxisA = b2RotateVector( jointSim->localFrameA.q, b2Vec2{ 1.0f, 0.0f } );
	b2Vec2 axisA = b2RotateVector( transformA.q, localAxisA );
	b2Vec2 pA = b2TransformPoint( transformA, jointSim->localFrameA.p );
	b2Vec2 pB = b2TransformPoint( transformB, jointSim->localFrameB.p );
	b2Vec2 d = b2Sub( pB, pA );
	float translation = b2Dot( d, axisA );
	return translation;
}

float b2PrismaticJoint_GetCurrentSpeed( b2JointId jointId )
{
	b2World* world = b2_worlds + jointId.world0;
	b2Joint* joint = b2GetJointFullId( world, jointId );
	B2_ASSERT( joint->type == b2_prismaticJoint );
	b2JointSim* base = b2GetJointSim( world, joint );
	B2_ASSERT( base->type == b2_prismaticJoint );

	b2Body* bodyA = b2Array_Get( world->bodies, base->bodyIdA );
	b2Body* bodyB = b2Array_Get( world->bodies, base->bodyIdB );
	b2BodySim* bodySimA = b2GetBodySim( world, bodyA );
	b2BodySim* bodySimB = b2GetBodySim( world, bodyB );
	b2BodyState* bodyStateA = b2GetBodyState( world, bodyA );
	b2BodyState* bodyStateB = b2GetBodyState( world, bodyB );

	b2Rot qA = bodySimA->transform.q;
	b2Rot qB = bodySimB->transform.q;

	b2Vec2 localAxisA = b2RotateVector( base->localFrameA.q, b2Vec2{ 1.0f, 0.0f } );
	b2Vec2 axisA = b2RotateVector( qA, localAxisA );
	b2Vec2 rA = b2RotateVector( qA, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	b2Vec2 rB = b2RotateVector( qB, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// 用 double 对质心做差，使远离原点时速度仍精确
	b2Vec2 dc = b2SubPos( bodySimB->center, bodySimA->center );
	b2Vec2 d = b2Add( dc, b2Sub( rB, rA ) );

	b2Vec2 vA = bodyStateA ? bodyStateA->linearVelocity : b2Vec2_zero;
	b2Vec2 vB = bodyStateB ? bodyStateB->linearVelocity : b2Vec2_zero;
	float wA = bodyStateA ? bodyStateA->angularVelocity : 0.0f;
	float wB = bodyStateB ? bodyStateB->angularVelocity : 0.0f;

	b2Vec2 vRel = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );
	float speed = b2Dot( d, b2CrossSV( wA, axisA ) ) + b2Dot( axisA, vRel );
	return speed;
}

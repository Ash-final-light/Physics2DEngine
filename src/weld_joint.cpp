// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "joint.h"
#include "recording.h"
#include "world.h"

void b2PrepareWeldJoint( b2JointSim* base, b2StepContext* context )
{
	B2_ASSERT( base->type == b2_weldJoint );

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

	b2WeldJoint* joint = &base->weldJoint;
	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// 用世界空间旋转计算关节锚点坐标系，相对于质心
	joint->frameA.q = b2MulRot( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b2RotateVector( bodySimA->transform.q, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b2MulRot( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b2RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );

	// 计算初始质心差。增量位置更新都相对此值。
	joint->deltaCenter = b2SubPos( bodySimB->center, bodySimA->center );

	float ka = iA + iB;
	joint->axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	if ( joint->linearHertz == 0.0f )
	{
		joint->linearSpring = base->constraintSoftness;
	}
	else
	{
		joint->linearSpring = b2MakeSoft( joint->linearHertz, joint->linearDampingRatio, context->h );
	}

	if ( joint->angularHertz == 0.0f )
	{
		joint->angularSpring = base->constraintSoftness;
	}
	else
	{
		joint->angularSpring = b2MakeSoft( joint->angularHertz, joint->angularDampingRatio, context->h );
	}

	if ( context->world->enableWarmStarting == false )
	{
		joint->linearImpulse = b2Vec2_zero;
		joint->angularImpulse = 0.0f;
	}
}

void b2WarmStartWeldJoint( b2JointSim* base, b2StepContext* context )
{
	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// 静态刚体的占位状态
	b2BodyState dummyState = b2_identityBodyState;

	b2WeldJoint* joint = &base->weldJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = b2MulSub( stateA->linearVelocity, mA, joint->linearImpulse );
		stateA->angularVelocity -= iA * ( b2Cross( rA, joint->linearImpulse ) + joint->angularImpulse );
	}

	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, mB, joint->linearImpulse );
		stateB->angularVelocity += iB * ( b2Cross( rB, joint->linearImpulse ) + joint->angularImpulse );
	}
}

void b2SolveWeldJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	B2_ASSERT( base->type == b2_weldJoint );

	float mA = base->invMassA;
	float mB = base->invMassB;
	float iA = base->invIA;
	float iB = base->invIB;

	// 静态刚体的占位状态
	b2BodyState dummyState = b2_identityBodyState;

	b2WeldJoint* joint = &base->weldJoint;

	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;

	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;

	// 混合刚度时块求解不正确


	// 角度约束
	{
		b2Rot qA = b2MulRot( stateA->deltaRotation, joint->frameA.q );
		b2Rot qB = b2MulRot( stateB->deltaRotation, joint->frameB.q );
		b2Rot relQ = b2InvMulRot( qA, qB );
		float jointAngle = b2Rot_GetAngle( relQ );

		float bias = 0.0f;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias || joint->angularHertz > 0.0f )
		{
			float C = jointAngle;
			bias = joint->angularSpring.biasRate * C;
			massScale = joint->angularSpring.massScale;
			impulseScale = joint->angularSpring.impulseScale;
		}

		float Cdot = wB - wA;
		float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->angularImpulse;
		joint->angularImpulse += impulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// 线性约束
	{
		b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
		b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );

		b2Vec2 bias = b2Vec2_zero;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias || joint->linearHertz > 0.0f )
		{
			b2Vec2 dcA = stateA->deltaPosition;
			b2Vec2 dcB = stateB->deltaPosition;
			b2Vec2 C = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );

			bias = b2MulSV( joint->linearSpring.biasRate, C );
			massScale = joint->linearSpring.massScale;
			impulseScale = joint->linearSpring.impulseScale;
		}

		b2Vec2 Cdot = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );

		b2Mat22 K;
		K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.cx.y = K.cy.x;
		K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		b2Vec2 b = b2Solve22( K, b2Add( Cdot, bias ) );

		b2Vec2 impulse = {
			-massScale * b.x - impulseScale * joint->linearImpulse.x,
			-massScale * b.y - impulseScale * joint->linearImpulse.y,
		};

		joint->linearImpulse = b2Add( joint->linearImpulse, impulse );

		vA = b2MulSub( vA, mA, impulse );
		wA -= iA * b2Cross( rA, impulse );
		vB = b2MulAdd( vB, mB, impulse );
		wB += iB * b2Cross( rB, impulse );
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

void b2WeldJoint_SetLinearHertz( b2JointId jointId, float hertz )
{
	if (!(b2IsValidFloat( hertz ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	B2_ASSERT( b2IsValidFloat( hertz ) && hertz >= 0.0f );
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), WeldJointSetLinearHertz, jointId, hertz );
	}
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.linearHertz = hertz;
}

float b2WeldJoint_GetLinearHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.linearHertz;
}

void b2WeldJoint_SetAngularHertz( b2JointId jointId, float hertz )
{
	if (!(b2IsValidFloat( hertz ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	B2_ASSERT( b2IsValidFloat( hertz ) && hertz >= 0.0f );
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), WeldJointSetAngularHertz, jointId, hertz );
	}
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.angularHertz = hertz;
}

float b2WeldJoint_GetAngularHertz( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.angularHertz;
}

void b2WeldJoint_SetLinearDampingRatio( b2JointId jointId, float dampingRatio )
{
	if (!(b2IsValidFloat( dampingRatio ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	B2_ASSERT( b2IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), WeldJointSetLinearDampingRatio, jointId, dampingRatio );
	}
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.linearDampingRatio = dampingRatio;
}

float b2WeldJoint_GetLinearDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.linearDampingRatio;
}

void b2WeldJoint_SetAngularDampingRatio( b2JointId jointId, float dampingRatio )
{
	if (!(b2IsValidFloat( dampingRatio ))) { return; }

	b2World* world = b2_worlds + jointId.world0;
	B2_ASSERT( b2IsValidFloat( dampingRatio ) && dampingRatio >= 0.0f );
	if ( b2Joint_IsValid( jointId ) )
	{
		B2_REC( ( b2_worlds + jointId.world0 ), WeldJointSetAngularDampingRatio, jointId, dampingRatio );
	}
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	joint->weldJoint.angularDampingRatio = dampingRatio;
}

float b2WeldJoint_GetAngularDampingRatio( b2JointId jointId )
{
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_weldJoint );
	return joint->weldJoint.angularDampingRatio;
}

void b2DrawWeldJoint( b2DebugDraw* draw, b2JointSim* base, b2WorldTransform transformA, b2WorldTransform transformB,
					  float drawScale )
{
	B2_ASSERT( base->type == b2_weldJoint );

	b2WorldTransform frameA = b2MulTransforms( transformA, base->localFrameA );
	b2WorldTransform frameB = b2MulTransforms( transformB, base->localFrameB );

	b2Polygon box = b2MakeBox( 0.25f * drawScale, 0.125f * drawScale );
	draw->DrawPolygonFcn( frameA, box.vertices, 4, b2_colorDarkOrange, draw->context );
	draw->DrawPolygonFcn( frameB, box.vertices, 4, b2_colorDarkCyan, draw->context );
}

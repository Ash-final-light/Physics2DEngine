// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "joint.h"
#include "world.h"

#include "box2d/math_functions.h"

static const b2Vec2 b2_pogoAxis = { 0.0f, 1.0f };

void b2PogoJoint_SetRestLength( b2JointId jointId, float length )
{
	if ( b2IsValidFloat( length ) == false )
	{
		return;
	}
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	if ( joint == nullptr )
	{
		return;
	}
	joint->pogoJoint.restLength = length;
}

float b2PogoJoint_GetRestLength( b2JointId jointId )
{
	return b2GetJointSimCheckType( jointId, b2_pogoJoint )->pogoJoint.restLength;
}

void b2PogoJoint_SetSpringHertz( b2JointId jointId, float hertz )
{
	if ( b2IsValidFloat( hertz ) == false )
	{
		return;
	}
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	if ( joint == nullptr )
	{
		return;
	}
	joint->pogoJoint.hertz = hertz;
}

float b2PogoJoint_GetSpringHertz( b2JointId jointId )
{
	return b2GetJointSimCheckType( jointId, b2_pogoJoint )->pogoJoint.hertz;
}

void b2PogoJoint_SetSpringDampingRatio( b2JointId jointId, float dampingRatio )
{
	if ( b2IsValidFloat( dampingRatio ) == false )
	{
		return;
	}
	b2JointSim* joint = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	if ( joint == nullptr )
	{
		return;
	}
	joint->pogoJoint.dampingRatio = dampingRatio;
}

float b2PogoJoint_GetSpringDampingRatio( b2JointId jointId )
{
	return b2GetJointSimCheckType( jointId, b2_pogoJoint )->pogoJoint.dampingRatio;
}

float b2PogoJoint_GetCurrentLength( b2JointId jointId )
{
	b2World* world = b2_worlds + jointId.world0;
	b2JointSim* jointSim = b2GetJointSimCheckType( jointId, b2_pogoJoint );
	b2Transform transformA = b2GetBodySim( world, world->bodies.data + jointSim->bodyIdA )->transform;
	b2Transform transformB = b2GetBodySim( world, world->bodies.data + jointSim->bodyIdB )->transform;
	b2Vec2 axis = b2RotateVector( transformB.q, b2RotateVector( jointSim->localFrameB.q, b2_pogoAxis ) );
	b2Vec2 pA = b2TransformPoint( transformA, jointSim->localFrameA.p );
	b2Vec2 pB = b2TransformPoint( transformB, jointSim->localFrameB.p );
	return b2Dot( b2Sub( pB, pA ), axis );
}

float b2PogoJoint_GetImpulse( b2JointId jointId )
{
	return b2GetJointSimCheckType( jointId, b2_pogoJoint )->pogoJoint.impulse;
}

float b2PogoJoint_GetCurrentSpeed( b2JointId jointId )
{
	return b2GetJointSimCheckType( jointId, b2_pogoJoint )->pogoJoint.velocity;
}

void b2PreparePogoJoint( b2JointSim* base, b2StepContext* context )
{
	b2World* world = context->world;
	b2Body* bodyA = b2Array_Get( world->bodies, base->bodyIdA );
	b2Body* bodyB = b2Array_Get( world->bodies, base->bodyIdB );
	b2SolverSet* setA = b2Array_Get( world->solverSets, bodyA->setIndex );
	b2SolverSet* setB = b2Array_Get( world->solverSets, bodyB->setIndex );
	int localIndexA = bodyA->localIndex;
	int localIndexB = bodyB->localIndex;
	b2BodySim* bodySimA = b2Array_Get( setA->bodySims, localIndexA );
	b2BodySim* bodySimB = b2Array_Get( setB->bodySims, localIndexB );
	base->invMassA = bodySimA->invMass;
	base->invMassB = bodySimB->invMass;
	base->invIA = bodySimA->invInertia;
	base->invIB = bodySimB->invInertia;

	b2PogoJoint* joint = &base->pogoJoint;
	joint->indexA = bodyA->setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint->indexB = bodyB->setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;
	joint->frameA.q = b2MulRot( bodySimA->transform.q, base->localFrameA.q );
	joint->frameA.p = b2RotateVector( bodySimA->transform.q, b2Sub( base->localFrameA.p, bodySimA->localCenter ) );
	joint->frameB.q = b2MulRot( bodySimB->transform.q, base->localFrameB.q );
	joint->frameB.p = b2RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );
	joint->deltaCenter = b2SubPos( bodySimB->center, bodySimA->center );
	float crA = b2Cross( joint->frameA.p, joint->normal );
	float crB = b2Cross( joint->frameB.p, joint->normal );
	float k = base->invMassA + base->invMassB + base->invIA * crA * crA + base->invIB * crB * crB;
	joint->linearMass = k > 0.0f ? 1.0f / k : 0.0f;
	if ( world->enableWarmStarting == false )
	{
		joint->impulse = 0.0f;
	}
}

void b2WarmStartPogoJoint( b2JointSim* base, b2StepContext* context )
{
	b2PogoJoint* joint = &base->pogoJoint;
	if ( joint->hertz == 0.0f )
	{
		joint->impulse = 0.0f;
		return;
	}
	b2BodyState dummyState = b2_identityBodyState;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;
	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );
	b2Vec2 linearImpulse = b2MulSV( joint->impulse, joint->normal );
	if ( stateA->flags & b2_dynamicFlag )
	{
		stateA->linearVelocity = b2MulSub( stateA->linearVelocity, base->invMassA, linearImpulse );
		stateA->angularVelocity -= base->invIA * b2Cross( rA, linearImpulse );
	}
	if ( stateB->flags & b2_dynamicFlag )
	{
		stateB->linearVelocity = b2MulAdd( stateB->linearVelocity, base->invMassB, linearImpulse );
		stateB->angularVelocity += base->invIB * b2Cross( rB, linearImpulse );
	}
}

void b2SolvePogoJoint( b2JointSim* base, b2StepContext* context, bool useBias )
{
	b2PogoJoint* joint = &base->pogoJoint;
	if ( joint->hertz == 0.0f )
	{
		joint->impulse = 0.0f;
		return;
	}
	b2BodyState dummyState = b2_identityBodyState;
	b2BodyState* stateA = joint->indexA == B2_NULL_INDEX ? &dummyState : context->states + joint->indexA;
	b2BodyState* stateB = joint->indexB == B2_NULL_INDEX ? &dummyState : context->states + joint->indexB;
	b2Vec2 vA = stateA->linearVelocity;
	float wA = stateA->angularVelocity;
	b2Vec2 vB = stateB->linearVelocity;
	float wB = stateB->angularVelocity;
	b2Vec2 rA = b2RotateVector( stateA->deltaRotation, joint->frameA.p );
	b2Vec2 rB = b2RotateVector( stateB->deltaRotation, joint->frameB.p );
	float bias = 0.0f;
	if ( useBias )
	{
		b2Vec2 d = b2Add( b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), b2Sub( rB, rA ) ), joint->deltaCenter );
		b2Vec2 pogoAxis = b2RotateVector( joint->frameB.q, b2_pogoAxis );
		float c = b2Dot( pogoAxis, d ) - joint->restLength;
		joint->velocity = b2SpringDamper( joint->hertz, joint->dampingRatio, c, joint->velocity, context->h );
		bias = -joint->velocity;
	}
	b2Vec2 vr = b2Sub( b2Add( vB, b2CrossSV( wB, rB ) ), b2Add( vA, b2CrossSV( wA, rA ) ) );
	float impulse = -joint->linearMass * ( b2Dot( joint->normal, vr ) + bias );
	float oldImpulse = joint->impulse;
	joint->impulse = b2ClampFloat( joint->impulse + impulse, -context->h * joint->maxTensionForce, context->h * joint->maxCompressionForce );
	b2Vec2 P = b2MulSV( joint->impulse - oldImpulse, joint->normal );
	vA = b2MulSub( vA, base->invMassA, P );
	wA -= base->invIA * b2Cross( rA, P );
	vB = b2MulAdd( vB, base->invMassB, P );
	wB += base->invIB * b2Cross( rB, P );
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

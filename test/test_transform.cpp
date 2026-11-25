// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math_functions.h"

static int Near( float a, float b )
{
	float d = a - b;
	if ( d < 0.0f )
	{
		d = -d;
	}
	return d <= 0.00001f;
}

static int NearVec( b2Vec2 a, b2Vec2 b )
{
	return Near( a.x, b.x ) && Near( a.y, b.y );
}

int TransformTest( void )
{
	b2Transform shift = b2Transform_identity;
	shift.p = { 2.0f, 3.0f };
	b2Vec2 local = { 1.0f, 0.0f };
	b2Vec2 world = b2TransformPoint( shift, local );
	if ( NearVec( world, { 3.0f, 3.0f } ) == 0 )
	{
		return 1;
	}
	if ( NearVec( b2InvTransformPoint( shift, world ), local ) == 0 )
	{
		return 1;
	}

	b2Vec2 direction = b2RotateVector( shift.q, { 0.0f, 4.0f } );
	if ( NearVec( direction, { 0.0f, 4.0f } ) == 0 )
	{
		return 1;
	}

	b2Transform turn = { { 0.0f, 0.0f }, { 0.0f, 1.0f } };
	b2Vec2 turned = b2TransformPoint( turn, local );
	if ( NearVec( turned, { 0.0f, 1.0f } ) == 0 )
	{
		return 1;
	}
	if ( NearVec( b2InvTransformPoint( turn, turned ), local ) == 0 )
	{
		return 1;
	}

	b2Transform combined = b2MulTransforms( shift, turn );
	b2Vec2 once = b2TransformPoint( combined, local );
	b2Vec2 twice = b2TransformPoint( shift, b2TransformPoint( turn, local ) );
	if ( NearVec( once, twice ) == 0 )
	{
		return 1;
	}

	b2Transform relative = b2InvMulTransforms( shift, combined );
	b2Vec2 throughRelative = b2TransformPoint( relative, local );
	if ( NearVec( throughRelative, turned ) == 0 )
	{
		return 1;
	}

	if ( b2IsValidTransform( shift ) == false || b2IsValidTransform( turn ) == false )
	{
		return 1;
	}
	b2Transform broken = shift;
	broken.q = { 2.0f, 0.0f };
	if ( b2IsValidRot( broken.q ) || b2IsValidTransform( broken ) )
	{
		return 1;
	}

	return 0;
}

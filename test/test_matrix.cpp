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

int MatrixTest( void )
{
	b2Mat22 identity = { { 1.0f, 0.0f }, { 0.0f, 1.0f } };
	b2Vec2 input = { 3.0f, -2.0f };
	b2Vec2 multiplied = b2MulMV( identity, input );
	if ( Near( multiplied.x, 3.0f ) == 0 || Near( multiplied.y, -2.0f ) == 0 )
	{
		return 1;
	}

	b2Mat22 scale = { { 2.0f, 0.0f }, { 0.0f, 4.0f } };
	b2Vec2 solved = b2Solve22( scale, { 6.0f, 8.0f } );
	if ( Near( solved.x, 3.0f ) == 0 || Near( solved.y, 2.0f ) == 0 )
	{
		return 1;
	}

	b2Mat22 inverse = b2GetInverse22( scale );
	b2Vec2 back = b2MulMV( inverse, { 6.0f, 8.0f } );
	if ( Near( back.x, solved.x ) == 0 || Near( back.y, solved.y ) == 0 )
	{
		return 1;
	}

	b2Mat22 singular = { { 1.0f, 2.0f }, { 2.0f, 4.0f } };
	b2Vec2 degenerate = b2Solve22( singular, { 1.0f, 1.0f } );
	b2Mat22 degenerateInverse = b2GetInverse22( singular );
	if ( degenerate.x != 0.0f || degenerate.y != 0.0f )
	{
		return 1;
	}
	if ( degenerateInverse.cx.x != 0.0f || degenerateInverse.cx.y != 0.0f || degenerateInverse.cy.x != 0.0f ||
		 degenerateInverse.cy.y != 0.0f )
	{
		return 1;
	}

	return 0;
}

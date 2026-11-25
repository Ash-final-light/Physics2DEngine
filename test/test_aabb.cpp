// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math_functions.h"
#include "aabb.h"
#include "test_macros.h"

#include <float.h>
#include <math.h>

int AABBTest( void )
{
	b2AABB inverted;
	inverted.lowerBound = { -1.0f, -1.0f };
	inverted.upperBound = { -2.0f, -2.0f };
	ENSURE( b2IsValidAABB( inverted ) == false );

	b2AABB flat;
	flat.lowerBound = { 0.0f, 0.0f };
	flat.upperBound = { 0.0f, 2.0f };
	ENSURE( b2IsValidAABB( flat ) == true );

	b2AABB bad = flat;
	bad.upperBound.x = NAN;
	ENSURE( b2IsValidAABB( bad ) == false );
	bad = flat;
	bad.lowerBound.y = INFINITY;
	ENSURE( b2IsValidAABB( bad ) == false );

	b2AABB a;
	a.lowerBound = { -1.0f, -1.0f };
	a.upperBound = { 1.0f, 1.0f };
	ENSURE( b2IsValidAABB( a ) == true );

	b2Vec2 center = b2AABB_Center( a );
	b2Vec2 extents = b2AABB_Extents( a );
	ENSURE( center.x == 0.0f && center.y == 0.0f );
	ENSURE( extents.x == 1.0f && extents.y == 1.0f );

	b2AABB b = { { 2.0f, 2.0f }, { 4.0f, 4.0f } };
	ENSURE( b2AABB_Overlaps( a, b ) == false );
	ENSURE( b2AABB_Contains( a, b ) == false );

	b2AABB touch = { { 1.0f, -1.0f }, { 2.0f, 1.0f } };
	ENSURE( b2AABB_Overlaps( a, touch ) == true );

	b2AABB inner = { { -0.5f, -0.5f }, { 0.5f, 0.5f } };
	ENSURE( b2AABB_Contains( a, inner ) == true );
	ENSURE( b2AABB_Contains( inner, a ) == false );

	b2AABB u = b2AABB_Union( a, b );
	ENSURE( u.lowerBound.x == -1.0f && u.lowerBound.y == -1.0f );
	ENSURE( u.upperBound.x == 4.0f && u.upperBound.y == 4.0f );
	ENSURE( b2AABB_Contains( u, a ) && b2AABB_Contains( u, b ) );
	ENSURE( a.upperBound.x == 1.0f && b.lowerBound.x == 2.0f );
	return 0;
}

int AABBRayCastTest( void )
{
	b2AABB aabb = { { -1.0f, -1.0f }, { 1.0f, 1.0f } };

	{
		b2Vec2 p1 = { -3.0f, 0.0f };
		b2Vec2 p2 = { 3.0f, 0.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );
		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.y, FLT_EPSILON );
		ENSURE_SMALL( output.point.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.point.y, FLT_EPSILON );
	}

	{
		b2Vec2 p1 = { 3.0f, 0.0f };
		b2Vec2 p2 = { -3.0f, 0.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );
		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.x - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.point.x - 1.0f, FLT_EPSILON );
	}

	{
		b2Vec2 p1 = { 0.0f, -3.0f };
		b2Vec2 p2 = { 0.0f, 3.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );
		ENSURE( output.hit == true );
		ENSURE_SMALL( output.normal.y + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.point.y + 1.0f, FLT_EPSILON );
	}

	{
		b2Vec2 p1 = { 0.0f, 3.0f };
		b2Vec2 p2 = { 0.0f, -3.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );
		ENSURE( output.hit == true );
		ENSURE_SMALL( output.normal.y - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.point.y - 1.0f, FLT_EPSILON );
	}

	{
		b2Vec2 p1 = { -3.0f, 2.0f };
		b2Vec2 p2 = { 3.0f, 2.0f };
		ENSURE( b2AABB_RayCast( aabb, p1, p2 ).hit == false );
		p1 = { 2.0f, -3.0f };
		p2 = { 2.0f, 3.0f };
		ENSURE( b2AABB_RayCast( aabb, p1, p2 ).hit == false );
		p1 = { -2.0f, 1.5f };
		p2 = { 2.0f, 1.5f };
		ENSURE( b2AABB_RayCast( aabb, p1, p2 ).hit == false );
	}

	{
		b2Vec2 p1 = { 0.0f, 0.0f };
		b2Vec2 p2 = { 2.0f, 0.0f };
		ENSURE( b2AABB_RayCast( aabb, p1, p2 ).hit == false );
		p2 = p1;
		ENSURE( b2AABB_RayCast( aabb, p1, p2 ).hit == false );
	}

	{
		b2Vec2 p1 = { -2.0f, -2.0f };
		b2Vec2 p2 = { 2.0f, 2.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );
		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 0.25f, FLT_EPSILON );
		ENSURE( ( output.normal.x == -1.0f && output.normal.y == 0.0f ) || ( output.normal.x == 0.0f && output.normal.y == -1.0f ) );
	}

	{
		b2Vec2 p1 = { -2.0f, 1.0f };
		b2Vec2 p2 = { 2.0f, 1.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );
		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 0.25f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
	}

	{
		b2Vec2 p1 = { -3.0f, 0.0f };
		b2Vec2 p2 = { -2.5f, 0.0f };
		ENSURE( b2AABB_RayCast( aabb, p1, p2 ).hit == false );
		p1 = { -2.0f, 0.0f };
		p2 = { -1.0f, 0.0f };
		b2CastOutput output = b2AABB_RayCast( aabb, p1, p2 );
		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.point.x + 1.0f, FLT_EPSILON );
	}

	{
		b2AABB offset = { { 2.0f, 3.0f }, { 4.0f, 5.0f } };
		b2Vec2 p1 = { 0.0f, 4.0f };
		b2Vec2 p2 = { 6.0f, 4.0f };
		b2CastOutput output = b2AABB_RayCast( offset, p1, p2 );
		ENSURE( output.hit == true );
		ENSURE_SMALL( output.fraction - 1.0f / 3.0f, FLT_EPSILON );
		ENSURE_SMALL( output.normal.x + 1.0f, FLT_EPSILON );
		ENSURE_SMALL( output.point.x - 2.0f, FLT_EPSILON );
		ENSURE_SMALL( output.point.y - 4.0f, FLT_EPSILON );
	}

	return 0;
}

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/collision.h"
#include "box2d/constants.h"
#include "box2d/math_functions.h"

#include <float.h>

int CircleManifoldTest( void )
{
	b2Circle circleA = { { 0.0f, 0.0f }, 1.0f };
	b2Circle circleB = { { 0.0f, 0.0f }, 1.0f };
	b2Transform xf = { { 1.5f, 0.0f }, b2Rot_identity };

	b2LocalManifold hit = b2CollideCircles( &circleA, &circleB, xf );
	ENSURE( hit.pointCount == 1 );
	ENSURE_SMALL( hit.normal.x - 1.0f, 0.0001f );
	ENSURE_SMALL( hit.normal.y, 0.0001f );
	ENSURE_SMALL( hit.points[0].separation - ( 1.5f - 2.0f ), 0.0001f );
	ENSURE_SMALL( hit.points[0].point.x - 0.75f, 0.0001f );
	ENSURE_SMALL( hit.points[0].point.y, 0.0001f );

	xf.p = { 2.0f, 0.0f };
	b2LocalManifold touch = b2CollideCircles( &circleA, &circleB, xf );
	ENSURE( touch.pointCount == 1 );
	ENSURE_SMALL( touch.points[0].separation, 0.0001f );

	xf.p = { 2.0f + B2_SPECULATIVE_DISTANCE * 0.5f, 0.0f };
	b2LocalManifold near = b2CollideCircles( &circleA, &circleB, xf );
	ENSURE( near.pointCount == 1 );
	ENSURE( near.points[0].separation > 0.0f );
	ENSURE( near.points[0].separation <= B2_SPECULATIVE_DISTANCE );

	xf.p = { 3.0f, 0.0f };
	b2LocalManifold miss = b2CollideCircles( &circleA, &circleB, xf );
	ENSURE( miss.pointCount == 0 );
	return 0;
}

int CapsuleCircleManifoldTest( void )
{
	b2Capsule capsule = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 1.0f };
	b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
	b2Transform side = { { 0.0f, 1.5f }, b2Rot_identity };
	b2LocalManifold flank = b2CollideCapsuleAndCircle( &capsule, &circle, side );
	ENSURE( flank.pointCount == 1 );
	ENSURE_SMALL( flank.normal.x, 0.0001f );
	ENSURE_SMALL( flank.normal.y - 1.0f, 0.0001f );
	ENSURE_SMALL( flank.points[0].separation - ( 1.5f - 2.0f ), 0.0001f );
	ENSURE_SMALL( flank.points[0].point.x, 0.0001f );
	ENSURE_SMALL( flank.points[0].point.y - 0.75f, 0.0001f );

	b2Transform cap = { { 3.0f, 0.0f }, b2Rot_identity };
	b2LocalManifold end = b2CollideCapsuleAndCircle( &capsule, &circle, cap );
	ENSURE( end.pointCount == 1 );
	ENSURE_SMALL( end.normal.x - 1.0f, 0.0001f );
	ENSURE_SMALL( end.points[0].separation, 0.0001f );
	ENSURE_SMALL( end.points[0].point.x - 2.0f, 0.0001f );

	b2Transform far = { { 0.0f, 4.0f }, b2Rot_identity };
	ENSURE( b2CollideCapsuleAndCircle( &capsule, &circle, far ).pointCount == 0 );
	return 0;
}

int PolygonCircleManifoldTest( void )
{
	b2Polygon box = b2MakeBox( 1.0f, 1.0f );
	b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
	b2Transform face = { { 2.0f, 0.0f }, b2Rot_identity };
	b2LocalManifold side = b2CollidePolygonAndCircle( &box, &circle, face );
	ENSURE( side.pointCount == 1 );
	ENSURE_SMALL( side.normal.x - 1.0f, 0.0001f );
	ENSURE_SMALL( side.normal.y, 0.0001f );
	ENSURE_SMALL( side.points[0].separation, 0.0001f );
	ENSURE_SMALL( side.points[0].point.x - 1.0f, 0.0001f );
	ENSURE_SMALL( side.points[0].point.y, 0.0001f );

	b2Transform inside = b2Transform_identity;
	circle.radius = 0.5f;
	b2LocalManifold contained = b2CollidePolygonAndCircle( &box, &circle, inside );
	ENSURE( contained.pointCount == 1 );
	ENSURE( contained.points[0].separation < 0.0f );

	circle.radius = 0.5f;
	b2Transform corner = { { 1.2f, 1.2f }, b2Rot_identity };
	b2LocalManifold vertex = b2CollidePolygonAndCircle( &box, &circle, corner );
	ENSURE( vertex.pointCount == 1 );
	ENSURE( vertex.points[0].separation < 0.0f );
	ENSURE( vertex.normal.x > 0.5f && vertex.normal.y > 0.5f );

	b2Polygon rounded = b2MakeRoundedBox( 1.0f, 1.0f, 0.2f );
	circle.radius = 0.2f;
	b2Transform roundFace = { { 1.3f, 0.0f }, b2Rot_identity };
	b2LocalManifold round = b2CollidePolygonAndCircle( &rounded, &circle, roundFace );
	ENSURE( round.pointCount == 1 );
	ENSURE_SMALL( round.normal.x - 1.0f, 0.0001f );
	ENSURE( round.points[0].separation < 0.0f );

	circle.radius = 0.5f;
	b2Transform away = { { 4.0f, 0.0f }, b2Rot_identity };
	ENSURE( b2CollidePolygonAndCircle( &box, &circle, away ).pointCount == 0 );
	return 0;
}

int CapsuleManifoldTest( void )
{
	b2Capsule lower = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 0.5f };
	b2Capsule upper = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 0.5f };
	b2Transform side = { { 0.0f, 1.0f }, b2Rot_identity };
	b2LocalManifold parallel = b2CollideCapsules( &lower, &upper, side );
	ENSURE( parallel.pointCount == 2 );
	ENSURE_SMALL( parallel.normal.x, 0.0001f );
	ENSURE( parallel.normal.y > 0.9f || parallel.normal.y < -0.9f );
	ENSURE_SMALL( parallel.points[0].separation, 0.001f );
	ENSURE_SMALL( parallel.points[1].separation, 0.001f );

	b2Capsule cross = { { 0.0f, -1.0f }, { 0.0f, 1.0f }, 0.5f };
	b2LocalManifold crossed = b2CollideCapsules( &lower, &cross, b2Transform_identity );
	ENSURE( crossed.pointCount >= 1 );
	float crossedSeparation = crossed.points[0].separation;
	if ( crossed.pointCount == 2 && crossed.points[1].separation < crossedSeparation )
	{
		crossedSeparation = crossed.points[1].separation;
	}
	ENSURE( crossedSeparation <= 0.001f );

	b2Capsule tip = { { 1.5f, 0.0f }, { 3.0f, 0.0f }, 0.5f };
	b2LocalManifold ends = b2CollideCapsules( &lower, &tip, b2Transform_identity );
	ENSURE( ends.pointCount == 1 );
	ENSURE( ends.normal.x > 0.9f );
	ENSURE( ends.points[0].separation < 0.0f );

	b2Transform far = { { 0.0f, 5.0f }, b2Rot_identity };
	ENSURE( b2CollideCapsules( &lower, &upper, far ).pointCount == 0 );
	return 0;
}

int SegmentCircleManifoldTest( void )
{
	b2Segment segment = { { 0.0f, -1.0f }, { 0.0f, 1.0f } };
	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	b2Transform right = { { 0.4f, 0.0f }, b2Rot_identity };
	b2LocalManifold front = b2CollideSegmentAndCircle( &segment, &circle, right );
	ENSURE( front.pointCount == 1 );
	ENSURE( front.normal.x > 0.9f );
	ENSURE( front.points[0].separation < 0.0f );

	b2Transform left = { { -0.4f, 0.0f }, b2Rot_identity };
	b2LocalManifold back = b2CollideSegmentAndCircle( &segment, &circle, left );
	ENSURE( back.pointCount == 1 );
	ENSURE( back.normal.x < -0.9f );
	ENSURE( back.points[0].separation < 0.0f );

	b2Transform end = { { 0.0f, 1.4f }, b2Rot_identity };
	b2LocalManifold cap = b2CollideSegmentAndCircle( &segment, &circle, end );
	ENSURE( cap.pointCount == 1 );
	ENSURE( cap.normal.y > 0.9f );

	b2Transform away = { { 2.0f, 0.0f }, b2Rot_identity };
	ENSURE( b2CollideSegmentAndCircle( &segment, &circle, away ).pointCount == 0 );
	return 0;
}

int SegmentCapsuleManifoldTest( void )
{
	b2Segment segment = { { -1.0f, 0.0f }, { 1.0f, 0.0f } };
	b2Capsule capsule = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 0.5f };
	b2Transform side = { { 0.0f, 0.4f }, b2Rot_identity };
	b2LocalManifold parallel = b2CollideSegmentAndCapsule( &segment, &capsule, side );
	ENSURE( parallel.pointCount == 2 );
	ENSURE( parallel.normal.y > 0.9f );
	ENSURE( parallel.points[0].separation < 0.0f );
	ENSURE( parallel.points[1].separation < 0.0f );

	b2Transform below = { { 0.0f, -0.4f }, b2Rot_identity };
	b2LocalManifold otherSide = b2CollideSegmentAndCapsule( &segment, &capsule, below );
	ENSURE( otherSide.pointCount == 2 );
	ENSURE( otherSide.normal.y < -0.9f );

	b2Capsule tip = { { 1.2f, 0.0f }, { 3.0f, 0.0f }, 0.5f };
	b2LocalManifold end = b2CollideSegmentAndCapsule( &segment, &tip, b2Transform_identity );
	ENSURE( end.pointCount == 1 );
	ENSURE( end.normal.x > 0.9f );
	ENSURE( end.points[0].separation < 0.0f );

	b2Transform far = { { 0.0f, 3.0f }, b2Rot_identity };
	ENSURE( b2CollideSegmentAndCapsule( &segment, &capsule, far ).pointCount == 0 );
	return 0;
}

int PolygonCapsuleManifoldTest( void )
{
	b2Polygon box = b2MakeBox( 1.0f, 1.0f );
	b2Capsule capsule = { { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.3f };
	b2Transform side = { { 0.0f, 1.2f }, b2Rot_identity };
	b2LocalManifold face = b2CollidePolygonAndCapsule( &box, &capsule, side );
	ENSURE( face.pointCount == 2 );
	ENSURE( face.normal.y > 0.9f );
	ENSURE( face.points[0].separation < 0.0f );
	ENSURE( face.points[1].separation < 0.0f );
	ENSURE( face.points[0].id != face.points[1].id );
	b2LocalManifold again = b2CollidePolygonAndCapsule( &box, &capsule, side );
	ENSURE( again.points[0].id == face.points[0].id );
	ENSURE( again.points[1].id == face.points[1].id );

	b2Capsule tip = { { 1.4f, 0.0f }, { 3.0f, 0.0f }, 0.5f };
	b2LocalManifold end = b2CollidePolygonAndCapsule( &box, &tip, b2Transform_identity );
	ENSURE( end.pointCount == 2 );
	ENSURE( end.normal.x > 0.9f );
	float endSeparation = end.points[0].separation < end.points[1].separation ? end.points[0].separation : end.points[1].separation;
	ENSURE( endSeparation < 0.0f );

	b2Polygon rounded = b2MakeRoundedBox( 1.0f, 1.0f, 0.2f );
	capsule.radius = 0.2f;
	b2Transform roundSide = { { 0.0f, 1.3f }, b2Rot_identity };
	b2LocalManifold round = b2CollidePolygonAndCapsule( &rounded, &capsule, roundSide );
	ENSURE( round.pointCount >= 1 );
	ENSURE( round.normal.y > 0.9f );
	ENSURE( round.points[0].separation < 0.0f );

	b2Transform away = { { 0.0f, 4.0f }, b2Rot_identity };
	capsule.radius = 0.3f;
	ENSURE( b2CollidePolygonAndCapsule( &box, &capsule, away ).pointCount == 0 );
	return 0;
}

// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/collision.h"
#include "box2d/constants.h"

b2PlaneSolverResult b2SolvePlanes( b2Vec2 targetDelta, b2CollisionPlane* planes, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		planes[i].push = 0.0f;
	}

	b2Vec2 delta = targetDelta;
	float tolerance = B2_LINEAR_SLOP;

	int iteration;
	for ( iteration = 0; iteration < 20; ++iteration )
	{
		float totalPush = 0.0f;
		for ( int planeIndex = 0; planeIndex < count; ++planeIndex )
		{
			b2CollisionPlane* plane = planes + planeIndex;

			// 加入间隙以防抖动
			float separation = b2PlaneSeparation( plane->plane, delta ) + B2_LINEAR_SLOP;
			// if (separation > 0.0f)
			//{
			//	continue;
			// }

			float push = -separation;

			// 钳制累积推挤
			float accumulatedPush = plane->push;
			plane->push = b2ClampFloat( plane->push + push, 0.0f, plane->pushLimit );
			push = plane->push - accumulatedPush;
			delta = b2MulAdd( delta, push, plane->plane.normal );

			// 跟踪最大推挤以判断收敛
			totalPush += b2AbsFloat( push );
		}

		if ( totalPush < tolerance )
		{
			break;
		}
	}

	return b2PlaneSolverResult{
		.delta = delta,
		.iterationCount = iteration,
	};
}

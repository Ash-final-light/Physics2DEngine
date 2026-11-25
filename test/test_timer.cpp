// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/base.h"
#include "test_macros.h"

int TimerTest( void )
{
	uint64_t start = b2GetTicks();
	volatile int sum = 0;
	for ( int i = 0; i < 200000; ++i )
	{
		sum += i;
	}

	float elapsed = b2GetMilliseconds( start );
	ENSURE( elapsed >= 0.0f );

	uint64_t mark = start;
	float reset = b2GetMillisecondsAndReset( &mark );
	ENSURE( reset >= 0.0f );
	ENSURE( mark >= start );

	uint64_t after = b2GetTicks();
	ENSURE( after >= mark );
	ENSURE( b2GetMilliseconds( mark ) >= 0.0f );
	return 0;
}

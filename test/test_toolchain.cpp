// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "toolchain_c.h"

#include <span>

namespace
{
int SumSpan( std::span<const int> values )
{
	int sum = 0;
	for ( int value : values )
	{
		sum += value;
	}
	return sum;
}
} // namespace

int ToolchainTest( void )
{
	constexpr int values[] = { 1, 2, 3, 4 };
	ENSURE( SumSpan( std::span<const int>( values ) ) == 10 );
	ENSURE( toolchain_c17_allocate_and_free( 4 ) == 10 );
	ENSURE( toolchain_c17_allocate_and_free( 0 ) == 0 );
	ENSURE( toolchain_c17_allocate_and_free( -3 ) == 0 );
	return 0;
}

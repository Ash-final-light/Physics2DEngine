// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdio.h>

#define RUN_TEST( T )                                                                                                            \
	do                                                                                                                           \
	{                                                                                                                            \
		int result = T();                                                                                                        \
		if ( result == 1 )                                                                                                       \
		{                                                                                                                        \
			printf( "test failed: " #T "\n" );                                                                                   \
			return 1;                                                                                                            \
		}                                                                                                                        \
		else                                                                                                                     \
		{                                                                                                                        \
			printf( "test passed: " #T "\n" );                                                                                   \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

#define ENSURE( C )                                                                                                              \
	do                                                                                                                           \
	{                                                                                                                            \
		if ( ( C ) == false )                                                                                                    \
		{                                                                                                                        \
			printf( "condition false: " #C "\n" );                                                                               \
			return 1;                                                                                                            \
		}                                                                                                                        \
	}                                                                                                                            \
	while ( 0 )

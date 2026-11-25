// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include <cstdio>
#include <cstring>

#if defined( _WIN32 )
#include <io.h>
#define b2dup _dup
#define b2dup2 _dup2
#define b2fileno _fileno
#define b2close _close
#else
#include <unistd.h>
#define b2dup dup
#define b2dup2 dup2
#define b2fileno fileno
#define b2close close
#endif

static int s_logHits;
static char s_logMessage[128];

static void CaptureLog( const char* message )
{
	s_logHits += 1;
	std::strncpy( s_logMessage, message, sizeof( s_logMessage ) - 1 );
	s_logMessage[sizeof( s_logMessage ) - 1] = '\0';
}

static int SilentAssert( const char* condition, const char* fileName, int lineNumber )
{
	(void)condition;
	(void)fileName;
	(void)lineNumber;
	return 0;
}

int LogCallbackTest( void )
{
	const char* path = "m005-default-log.txt";
	FILE* file = std::fopen( path, "w+" );
	if ( file == nullptr )
	{
		return 1;
	}

	std::fflush( stdout );
	int saved = b2dup( b2fileno( stdout ) );
	if ( saved < 0 || b2dup2( b2fileno( file ), b2fileno( stdout ) ) != 0 )
	{
		std::fclose( file );
		std::remove( path );
		return 1;
	}

	b2Log( "plain" );
	std::fflush( stdout );
	b2dup2( saved, b2fileno( stdout ) );
	b2close( saved );

	std::rewind( file );
	char captured[128];
	if ( std::fgets( captured, (int)sizeof( captured ), file ) == nullptr )
	{
		std::fclose( file );
		std::remove( path );
		return 1;
	}
	std::fclose( file );
	std::remove( path );
	if ( std::strcmp( captured, "Box2D: plain\n" ) != 0 )
	{
		std::printf( "default log was [%s]\n", captured );
		return 1;
	}

	b2SetAssertCallback( SilentAssert );
	s_logHits = 0;
	s_logMessage[0] = '\0';
	b2SetLogCallback( CaptureLog );
	b2Log( "n=%d", 12 );
	if ( s_logHits != 1 || std::strcmp( s_logMessage, "n=12" ) != 0 )
	{
		return 1;
	}

	return 0;
}

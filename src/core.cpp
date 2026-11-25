// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include "box2d/math_functions.h"

#include <cmath>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( B2_PLATFORM_WINDOWS )
#include <malloc.h>
#endif

b2Version b2GetVersion( void )
{
	return b2Version{
		.major = B2_VERSION_MAJOR,
		.minor = B2_VERSION_MINOR,
		.revision = B2_VERSION_REVISION,
	};
}

bool b2IsDoublePrecision( void )
{
#if defined( BOX2D_DOUBLE_PRECISION )
	return true;
#else
	return false;
#endif
}

static int b2DefaultAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	fprintf( stderr, "BOX2D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );
	fflush( stderr );
	return 1;
}

static b2AssertCallbackFcn* b2AssertHandler = b2DefaultAssertFcn;

void b2SetAssertCallback( b2AssertCallbackFcn* assertFcn )
{
	B2_ASSERT( assertFcn != NULL );
	b2AssertHandler = assertFcn;
}

#if !defined( NDEBUG ) || defined( B2_ENABLE_ASSERT )
int b2InternalAssert( const char* condition, const char* fileName, int lineNumber )
{
	int result = b2AssertHandler( condition, fileName, lineNumber );
	if ( result )
	{
		B2_BREAKPOINT;
	}
	return result;
}
#endif

static void b2DefaultLogFcn( const char* message )
{
	printf( "Box2D: %s\n", message );
}

static b2LogCallbackFcn* b2LogHandler = b2DefaultLogFcn;

void b2SetLogCallback( b2LogCallbackFcn* logFcn )
{
	B2_ASSERT( logFcn != NULL );
	b2LogHandler = logFcn;
}

void b2Log( const char* format, ... )
{
	va_list args;
	va_start( args, format );
	char buffer[512];
	vsnprintf( buffer, sizeof( buffer ), format, args );
	b2LogHandler( buffer );
	va_end( args );
}

float b2_lengthUnitsPerMeter = 1.0f;

void b2SetLengthUnitsPerMeter( float lengthUnits )
{
	if ( std::isfinite( lengthUnits ) == false || lengthUnits <= 0.0f )
	{
		b2Log( "invalid input: lengthUnits in %s\n", __func__ );
		B2_ASSERT( std::isfinite( lengthUnits ) && lengthUnits > 0.0f );
		return;
	}

	b2_lengthUnitsPerMeter = lengthUnits;
}

float b2GetLengthUnitsPerMeter( void )
{
	return b2_lengthUnitsPerMeter;
}

static b2AllocCallbackFcn* b2_allocFcn = NULL;
static b2FreeCallbackFcn* b2_freeFcn = NULL;
static b2AtomicI64 b2_byteCount;

void b2SetAllocator( b2AllocCallbackFcn* allocFcn, b2FreeCallbackFcn* freeFcn )
{
	b2_allocFcn = allocFcn;
	b2_freeFcn = freeFcn;
}

size_t b2GetByteCount( void )
{
	return (size_t)b2AtomicLoadI64( &b2_byteCount );
}

void* b2Alloc( size_t size )
{
	if ( size == 0 )
	{
		return NULL;
	}

	b2AtomicFetchAddI64( &b2_byteCount, (int64_t)size );
	size_t alignedSize = ( ( size - 1 ) | ( B2_ALIGNMENT - 1 ) ) + 1;

	void* ptr;
	if ( b2_allocFcn != NULL )
	{
		ptr = b2_allocFcn( alignedSize, B2_ALIGNMENT );
	}
	else
	{
#ifdef B2_PLATFORM_WINDOWS
		ptr = _aligned_malloc( alignedSize, B2_ALIGNMENT );
#else
		ptr = aligned_alloc( B2_ALIGNMENT, alignedSize );
#endif
	}

	B2_ASSERT( ptr != NULL );
	B2_ASSERT( ( (uintptr_t)ptr & ( B2_ALIGNMENT - 1 ) ) == 0 );
	return ptr;
}

void b2Free( void* mem, size_t size )
{
	if ( mem == NULL )
	{
		return;
	}

	if ( b2_freeFcn != NULL )
	{
		size_t alignedSize = size > 0 ? ( ( size - 1 ) | ( B2_ALIGNMENT - 1 ) ) + 1 : 0;
		b2_freeFcn( mem, alignedSize );
	}
	else
	{
#ifdef B2_PLATFORM_WINDOWS
		_aligned_free( mem );
#else
		free( mem );
#endif
	}

	b2AtomicFetchAddI64( &b2_byteCount, -(int64_t)size );
}

void* b2GrowAlloc( void* oldMem, size_t oldSize, size_t newSize )
{
	B2_ASSERT( newSize > oldSize );
	void* newMem = b2Alloc( newSize );
	if ( oldSize > 0 )
	{
		memcpy( newMem, oldMem, oldSize );
		b2Free( oldMem, oldSize );
	}
	return newMem;
}

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/base.h"

int VersionCTest( void )
{
	b2Version version = b2GetVersion();
	if ( version.major != B2_VERSION_MAJOR || version.minor != B2_VERSION_MINOR || version.revision != B2_VERSION_REVISION )
	{
		return 1;
	}

	if ( b2IsDoublePrecision() != false )
	{
		return 1;
	}

	return 0;
}

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/base.h"

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

// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/collision.h"

bool b2IsValidCircle( const b2Circle* circle );
bool b2IsValidSegment( const b2Segment* segment );
bool b2IsValidCapsule( const b2Capsule* capsule );

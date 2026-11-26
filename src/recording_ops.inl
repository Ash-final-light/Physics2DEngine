// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

B2_REC_OP( 0x01, DestroyWorld, RET_NONE, ARG( WORLDID, world ) )
B2_REC_OP( 0xF1, StateHash, RET_NONE, ARG( WORLDID, world ) ARG( U64, hash ) )
B2_REC_OP( 0xF2, RecordingBounds, RET_NONE, ARG( AABB, bounds ) )

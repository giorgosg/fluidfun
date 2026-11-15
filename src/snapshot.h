// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "recording.h"

#include <stdint.h>

typedef struct b2World b2World;

// Serialize the complete simulation state of world into buf. Backs the public
// b2World_GetSnapshot. Must be called at a step boundary (between b2World_Step calls).
// Reuses b2RecBuffer/b2RecBufAppend for output.
void b2SerializeWorld( b2World* world, b2RecBuffer* buf );
// fluidfun: the same with the fluid's section whether or not the world has had a fluid, for a recording's seed and a
// replay's keyframes, so that the serials the fluid's ids come from restore exactly
void b2SerializeWorldWithFluid( b2World* world, b2RecBuffer* buf );

// Extensive hash for testing
uint64_t b2HashWorldStateDeep( b2World* world );

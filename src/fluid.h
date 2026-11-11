// SPDX-License-Identifier: MIT
// What the world and the solver call on the world's fluid. The public API is in box2d/fluidfun.h.
#pragma once

#include "box2d/types.h"

struct b2fFluid;

// Before a step, while the world is unlocked: gathers the shapes near the liquid and changes particle sizes
void b2PrepareFluid( struct b2fFluid* fluid );

// Once per sub-step, after the bodies have moved
void b2FluidSubStep( b2SubStepContext* context, int subStepIndex, int subStepCount, float h, struct b2fFluid* fluid );

// Frees a fluid the world is done with
void b2FreeFluid( struct b2fFluid* fluid );

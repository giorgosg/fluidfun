# fluidfun

Particle liquids for [Box2D](https://github.com/erincatto/box2d) v3, in a fork of Box2D.

Liquids run inside Box2D's solver sub-steps, on its worker threads. Bodies float, sink, push the liquid and get
pushed and dragged by it.

**Status: early.** There is no release, the API will still change, and only the CMake build includes the liquids.
fluidfun is not part of Box2D and is not made by its author.

## What it does

- **Double density relaxation** (Clavet et al. 2005), with viscosity, cohesion and wet friction at shapes.
- **Two-way coupling** with Box2D bodies, through a sub-step callback the fork adds.
- **Adaptive particle sizes** (after Winchenbach et al. 2017): particles are fine at the surface and around bodies,
  and coarser deep down. A deep pool needs far fewer particles.
- **Several liquids** that keep apart, each with its own density, stiffness, viscosity and cohesion. Light liquids
  float on heavy ones.
- **Deterministic**: the same scene gives the same result bit for bit at any worker count.
- **Large worlds**: particle positions are `b2Pos`, so they are doubles when Box2D is built with
  `BOX2D_DOUBLE_PRECISION`.

## Changes to Box2D

Besides the liquid code (`src/fluid*.c`, `include/box2d/fluidfun.h`), the fork adds one thing to Box2D: a callback
that runs at the end of every solver sub-step, with access to the step's workers and to body motion.

```c
b2World_SetSubStepCallback( worldId, fcn, context );
b2SubStep_ParallelFor( context, task, itemCount, minRange, taskContext );
b2SubStep_GetBodyMotion( context, bodyId, &body );
b2SubStep_ApplyImpulse( context, bodyId, impulse, angularImpulse );
```

The rest is upstream Box2D. The library builds and installs as `fluidfun`, so it can sit next to a stock Box2D:
`#include <fluidfun/fluidfun.h>` and link `fluidfun::fluidfun`.

## Building

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Run the samples from the build folder, and pick a scene under **Fluid**:

```bash
cd build && bin/samples
```

`benchmark/fluid/run.sh "Paddle Boat"` opens one scene directly.

The scenes also run without a window, and the tests check that every scene gives the same result at 1, 3, 6 and 12
workers and loses no liquid:

```bash
ctest --test-dir build/benchmark/fluid
```

## Using it

```c
#include "box2d/fluidfun.h"

b2fFluidDef def = b2fDefaultFluidDef();
def.particleSpacing = 0.07f;
b2fFluid* fluid = b2fCreateFluid( worldId, &def );
b2fFluid_CreateParticlesInBox( fluid, 0, (b2Pos){ -4.0f, 0.1f }, (b2Pos){ 4.0f, 3.0f } );

// A second liquid, lighter than the first
b2fLiquidDef oil = b2fDefaultLiquidDef();
oil.massDensity = 0.6f;
int oilIndex = b2fFluid_CreateLiquid( fluid, &oil );
b2fFluid_CreateParticlesInBox( fluid, oilIndex, (b2Pos){ -4.0f, 3.1f }, (b2Pos){ 4.0f, 4.0f } );

// Each frame
b2fFluid_Prepare( fluid );
b2World_Step( worldId, 1.0f / 60.0f, 4 );

int count = b2fFluid_GetParticleCount( fluid );
const b2Pos* positions = b2fFluid_GetPositions( fluid );
const float* sizes = b2fFluid_GetSizes( fluid );

b2fDestroyFluid( fluid );
```

One fluid holds all of a world's liquids, up to 16 of them. Every setting is described in `include/box2d/fluidfun.h`.

## License

MIT, as Box2D. Box2D is by Erin Catto; see [LICENSE](LICENSE).

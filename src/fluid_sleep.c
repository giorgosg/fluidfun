// SPDX-License-Identifier: MIT
// Sleep for still liquid. Goswami and Pajarola 2011 (section 5) hold slow particles still and skip them in the
// substep, and give the passive particles beside active ones, which they call semi-active, a density but no move. Here
// a particle falls asleep once it has been slower than B2F_SLEEP_SPEED for B2_TIME_TO_SLEEP, as a Box2D body does, by
// its velocity smoothed over B2F_CALM_TIME, so that liquid which only jitters or rings within a step does not keep
// waking. In each substep:
//
// - Prediction, the move pass, the push out of shapes and the velocity pass skip asleep particles, so they hold still
//   at zero velocity. Every particle is still sorted, and still finds the shapes in its reach. Once all are asleep
//   and none was added or removed, the substep skips the sort too, and runs only in the first substep of a step and
//   while a body near the liquid is awake (b2FluidSubStep); the size pass does not run.
// - An asleep particle that touches an awake body, or lies in the cells near one (MarkHeld in fluid_boundary.c),
//   wakes at once, since the body may move into it, and none there falls asleep (B2F_WAKE_HELD). Its sleep time keeps
//   running, so a body at rest in still liquid can sleep, and then the liquid round it.
// - Liquid with springs never sleeps, since both ends of a spring must work it out alike. The size pass sizes asleep
//   particles as the rest, and one whose size changes wakes.
// - The shell is the asleep particles with an awake particle within their search window, found from the cells
//   (b2fFindShell), and those against a shape. Shell particles work out their density and pressure, which their
//   awake neighbours read; deep asleep particles skip the density pass as well, apart from those that fell asleep in
//   the last substep (B2F_ASLEEP_NEW). A shell particle wakes at the end of the substep if its density has moved
//   B2F_WAKE_DENSITY of rest from what it was in its first substep asleep, or if an awake neighbour is faster than
//   B2F_WAKE_SPEED in this substep, as Goswami wakes the neighbours of fast particles; smoothed, the speed lagged a
//   splash so far that a third of a pool slept on through the waves of a crate dropped into it. The density test catches what speed
//   does not: liquid draining slowly away from a sleeping region, a shape taken away from under it, or a body
//   pressing into it.
// - Calls that move, add or remove liquid wake what they touch (fluid.c, fluid_push.c, fluid_spawn.c), and a change
//   of settings, of gravity or of either sleep switch wakes everything.
//
// Awake particles see asleep ones as neighbours that do not move, so the push between them is one-sided, as against a
// wall, and the shell never takes its share of a move. Goswami notes the same loss of momentum. It lasts only while
// the push is too small to wake the shell.
#include "fluid_internal.h"

#include "physics_world.h"

void b2fWakeParticle( b2fFluid* f, int i )
{
	// Asleep, calm held the density it wakes by; awake, it is the smoothed velocity, which goes on
	if ( f->asleep[i] != 0 )
	{
		f->calm[i] = b2Vec2_zero;
	}
	f->asleep[i] = 0;
	f->sleepTime[i] = 0.0f;
}

// Times reset too, since a setting that changes may make still liquid move
void b2fWakeAll( b2fFluid* f )
{
	memset( f->asleep, 0, (size_t)f->count );
	memset( f->sleepTime, 0, (size_t)f->count * sizeof( float ) );
	memset( f->calm, 0, (size_t)f->count * sizeof( b2Vec2 ) );
	f->asleepCount = 0;
}

void b2fWakeBox( b2fFluid* f, b2fCoord minX, b2fCoord minY, b2fCoord maxX, b2fCoord maxY )
{
	for ( int i = 0; i < f->count; ++i )
	{
		b2Pos p = f->pos[i];
		if ( minX <= p.x && p.x <= maxX && minY <= p.y && p.y <= maxY )
		{
			b2fWakeParticle( f, i );
		}
	}
}

void b2fPrepareSleep( b2fFluid* f, b2Vec2 gravity )
{
	f->sleepOn = f->def.enableSleep && f->world->enableSleep;
	if ( f->sleepOn == false || gravity.x != f->worldGravity.x || gravity.y != f->worldGravity.y )
	{
		b2fWakeAll( f );
	}
	b2fCountAsleep( f );
}

void b2fCountAsleep( b2fFluid* f )
{
	int count = 0;
	for ( int i = 0; i < f->count; ++i )
	{
		count += f->asleep[i] != 0 ? 1 : 0;
	}
	f->asleepCount = count;
}

// For each row of cells, a running count of the cells holding an awake particle
static void CellRowTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	int width = f->gridWidth, stride = width + 1;
	for ( int row = start; row < end; ++row )
	{
		int* counts = f->cellAwake + row * stride;
		int sum = 0;
		counts[0] = 0;
		for ( int x = 0; x < width; ++x )
		{
			int c = row * width + x;
			int awake = 0;
			for ( int i = f->cellStart[c], last = f->cellStart[c + 1]; i < last && awake == 0; ++i )
			{
				awake = f->asleep[i] == 0;
			}
			sum += awake;
			counts[x + 1] = sum;
		}
	}
}

// An asleep particle is in the shell if any cell of its search window holds an awake particle, which covers every
// awake particle within its reach, since the window finds pairs from both sides, or if it is against a shape
static void ShellTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	int width = f->gridWidth, stride = width + 1;
	for ( int i = start; i < end; ++i )
	{
		if ( f->asleep[i] == 0 )
		{
			continue;
		}
		bool shell = ( f->asleep[i] & ( B2F_ASLEEP_SHAPE | B2F_ASLEEP_NEW ) ) != 0;
		int reach = f->uniformWindow > 0 ? f->uniformWindow : f->window[i];
		int cx = f->cell[i] % width, cy = f->cell[i] / width;
		for ( int row = cy - reach; row <= cy + reach && shell == false; ++row )
		{
			const int* counts = f->cellAwake + row * stride;
			shell = counts[cx + reach + 1] > counts[cx - reach];
		}
		f->wake[i] |= shell ? B2F_WAKE_SHELL : 0u;
	}
}

void b2fFindShell( b2fFluid* f, b2SubStepContext* context )
{
	if ( f->asleepCount == 0 )
	{
		return;
	}

	int needed = f->gridHeight * ( f->gridWidth + 1 );
	if ( needed > f->cellAwakeCapacity )
	{
		f->cellAwake = b2fGrowArray( f->cellAwake, f->cellAwakeCapacity, 2 * needed, sizeof( int ) );
		f->cellAwakeCapacity = 2 * needed;
	}
	b2SubStep_ParallelFor( context, CellRowTask, f->gridHeight, 16, f );
	b2SubStep_ParallelFor( context, ShellTask, f->count, 1024, f );
}

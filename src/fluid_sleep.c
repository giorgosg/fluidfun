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
// - Both ends of a spring work it out alike, so particles joined by springs, directly or through others, sleep and wake
//   as one group (b2fSpringGroups): a group falls asleep in the substep its last particle has been slow for
//   B2_TIME_TO_SLEEP, and when any of it wakes, for whatever reason, all of it wakes before the next spring pass.
//   While both ends are asleep a spring is skipped, and springs neither form nor go at asleep particles. The size
//   pass sizes asleep particles as the rest, and one whose size changes wakes.
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
	f->springAsleep = 0;
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

// What a spring group holds, at its root
#define B2F_GROUP_ASLEEP 1u
#define B2F_GROUP_AWAKE 2u
#define B2F_GROUP_RESTLESS 4u

static int GroupRoot( int* parent, int i )
{
	while ( parent[i] != i )
	{
		parent[i] = parent[parent[i]];
		i = parent[i];
	}
	return i;
}

// The groups are the particles of liquids with springs joined through the springs either end holds, found by union
// and find over the springs once, on one thread, which takes no ordering between workers. That runs only when a group
// may change state: when some particle is ready to fall asleep, or the count of asleep ones has changed since the last
// call, since then something has woken (falling asleep happens only here). A spring forms only between awake
// particles, and the spring pass ends one that is held by an awake particle alone and whose other end is asleep, so
// the springs held at the start of the substep are the groups' springs.
void b2fSpringGroups( b2fFluid* f, bool settle )
{
	uint32_t springLiquids = f->springLiquids;
	int n = f->count, asleep = 0, ready = 0;
	for ( int i = 0; i < n; ++i )
	{
		if ( ( springLiquids >> f->liquid[i] & 1u ) != 0 )
		{
			asleep += f->asleep[i] != 0 ? 1 : 0;
			ready += settle && f->asleep[i] == 0 && ( f->wake[i] & B2F_WAKE_CALM ) != 0 ? 1 : 0;
		}
	}
	if ( ready == 0 && ( asleep == f->springAsleep || asleep == 0 ) )
	{
		f->springAsleep = asleep;
		return;
	}

	int* parent = f->springParent;
	unsigned char* group = f->springGroup;
	for ( int i = 0; i < n; ++i )
	{
		parent[i] = i;
		group[i] = 0;
	}
	for ( int i = 0; i < n; ++i )
	{
		if ( ( springLiquids >> f->liquid[i] & 1u ) == 0 )
		{
			continue;
		}
		const b2fSprings* sp = f->springs + i;
		for ( int s = 0; s < sp->count; ++s )
		{
			int j = sp->rest[s] >= 0.0f ? b2fFindParticle( f, sp->id[s] ) : -1;
			if ( j < 0 )
			{
				continue;
			}
			int a = GroupRoot( parent, i ), b = GroupRoot( parent, j );
			if ( a != b )
			{
				// The lower index is the root, though which is does not change the groups
				parent[a > b ? a : b] = a < b ? a : b;
			}
		}
	}

	for ( int i = 0; i < n; ++i )
	{
		if ( ( springLiquids >> f->liquid[i] & 1u ) == 0 )
		{
			continue;
		}
		unsigned char state = B2F_GROUP_ASLEEP;
		if ( f->asleep[i] == 0 )
		{
			bool calm = settle && ( f->wake[i] & B2F_WAKE_CALM ) != 0;
			state = calm ? B2F_GROUP_AWAKE : B2F_GROUP_AWAKE | B2F_GROUP_RESTLESS;
		}
		group[GroupRoot( parent, i )] |= state;
	}

	asleep = 0;
	for ( int i = 0; i < n; ++i )
	{
		if ( ( springLiquids >> f->liquid[i] & 1u ) == 0 )
		{
			continue;
		}
		unsigned char state = group[GroupRoot( parent, i )];
		if ( f->asleep[i] != 0 && ( state & B2F_GROUP_AWAKE ) != 0 )
		{
			b2fWakeParticle( f, i );
		}
		else if ( f->asleep[i] == 0 && state == B2F_GROUP_AWAKE )
		{
			// All of the group ready, it falls asleep as VelocityTask puts other particles to sleep
			unsigned int shape = f->contactCount[i] > 0 ? B2F_ASLEEP_SHAPE : 0u;
			f->asleep[i] = (unsigned char)( B2F_ASLEEP | B2F_ASLEEP_NEW | shape );
			f->vx[i] = 0.0f;
			f->vy[i] = 0.0f;
			f->calm[i] = b2Vec2_zero;
		}
		asleep += f->asleep[i] != 0 ? 1 : 0;
	}
	f->springAsleep = asleep;
}

// SPDX-License-Identifier: MIT
// Sleep for still liquid. Goswami and Pajarola 2011 (section 5) hold slow particles still and skip them in the
// substep, and give the passive particles beside active ones, which they call semi-active, a density but no move. Here
// a particle falls asleep once it has been slower than B2F_SLEEP_SPEED for B2_TIME_TO_SLEEP, as a Box2D body does, by
// its velocity smoothed over B2F_CALM_TIME, so that liquid which only jitters or rings within a step does not keep
// waking. In each substep:
//
// - Asleep particles are kept apart from the awake ones, as Box2D keeps sleeping islands out of its awake set: the awake
//   come first in the particle arrays, activeCount of them, and the asleep after them, filed by cell in a grid of their
//   own, the sleep grid, which is made when they are filed and kept while none of them wakes (b2fFileAsleep in
//   fluid_grid.c). Particles are filed again, in the substep after, once some fall asleep or wake, or are added or
//   removed. The substep predicts, sorts, moves, pushes out of shapes and works out velocities for the awake particles
//   alone, so they hold still at zero velocity, and the asleep ones cost nothing there. Awake particles find their
//   asleep neighbours in the sleep grid, from their cell there, which the sort gives them. Once all are asleep, the
//   substep runs only in the first substep of a step and while a body near the liquid is awake (b2FluidSubStep); the
//   size pass does not run.
// - An asleep particle that touches an awake body, or lies near one (MarkHeld in fluid_boundary.c, b2fWakeHeld here),
//   wakes at once, since the body may move into it, and none there falls asleep (B2F_WAKE_HELD). Its sleep time keeps
//   running, so a body at rest in still liquid can sleep, and then the liquid round it. Asleep particles near an
//   awake body wake before the substep moves anything, and are filed with the awake ones.
// - A spring is worked out once, for both its ends, and particles joined by springs, directly or through others, sleep
//   and wake as one group (b2fSpringGroups): a group falls asleep in the substep its last particle has been slow for
//   B2_TIME_TO_SLEEP, and when any of it wakes, for whatever reason, all of it wakes before the next spring pass.
//   While both ends are asleep a spring is skipped, and springs neither form nor go at asleep particles. The size
//   pass sizes the asleep particles of the shell as the awake ones, and one whose size changes wakes; the others keep
//   their sizes, and the depth boxes cover only the particles sized and their reach (fluid_adapt.c).
// - The shell is the asleep particles in the cells of the sleep grid that awake particles search (b2fFindShell), those
//   that fell asleep in the last substep (B2F_ASLEEP_NEW), and in the first substep of a step those against a shape.
//   Shell particles find their contacts and work out their density and pressure, which their awake neighbours read,
//   and the size pass sizes them; the other asleep particles skip all of it, and find their contacts only in the
//   first substep of a step, which is when shapes can have changed under them. A shell particle wakes at the end of the substep if its density has moved
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
	f->asleepActive = 0;
	f->awakeSettled = f->settledCount;
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

// Over the asleep flags alone, a byte a particle, so it costs little beside anything else a substep does
void b2fCountAsleep( b2fFluid* f )
{
	int n = f->count, active = f->activeCount < n ? f->activeCount : n;
	int asleepActive = 0, asleepSettled = 0;
	for ( int i = 0; i < active; ++i )
	{
		asleepActive += f->asleep[i] != 0 ? 1 : 0;
	}
	for ( int i = active; i < n; ++i )
	{
		asleepSettled += f->asleep[i] != 0 ? 1 : 0;
	}
	f->asleepCount = asleepActive + asleepSettled;
	f->asleepActive = asleepActive;
	f->awakeSettled = n - active - asleepSettled;
}

// An asleep particle's cell in the grid of awake particles, clamped as the sort clamps theirs, and how far it searches
// there
static void FindAwakeCell( b2fFluid* f, int i )
{
	const b2fGrid* grid = &f->grid;
	float inv = 1.0f / grid->cell;
	int cx, cy;
	f->cell[i] = b2fGridCell( grid, f->pos[i], &cx, &cy );
	if ( grid->uniformWindow == 0 )
	{
		int bw = ( grid->width + B2F_BLOCK_CELLS - 1 ) >> B2F_BLOCK_SHIFT;
		float near = grid->blockReach.data[( cy >> B2F_BLOCK_SHIFT ) * bw + ( cx >> B2F_BLOCK_SHIFT )];
		float size = b2fLatticeSize( f->size[i] ), reach = size + ( near > size ? near : size );
		f->window[i] = (int)ceilf( reach * inv );
	}
}

void b2fAddShell( b2fFluid* f, int i )
{
	f->inShell[i] = 1;
	f->shell[f->shellCount++] = i;
	if ( f->activeCount > 0 )
	{
		FindAwakeCell( f, i );
	}
}

// For each row of the awake particles' grid, a running count of the cells holding a particle
static void CellRowTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	const b2fGrid* grid = &f->grid;
	int width = grid->width, stride = width + 1;
	for ( int row = start; row < end; ++row )
	{
		int* counts = f->cellAwake.data + row * stride;
		const int* cells = grid->start.data + row * width;
		int sum = 0;
		counts[0] = 0;
		for ( int x = 0; x < width; ++x )
		{
			sum += cells[x + 1] > cells[x] ? 1 : 0;
			counts[x + 1] = sum;
		}
	}
}

// An asleep particle is in the shell if some cell of its search window in the awake particles' grid holds a particle,
// which takes in every awake particle within its reach, since the window finds pairs from both sides. One outside the
// bounds of the awake particles by more than any reach is not.
static void ShellTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	const b2fGrid* grid = &f->grid;
	int width = grid->width, stride = width + 1, a = f->activeCount;
	bool filed = f->filedNow;
	b2fBox box = grid->all;
	for ( int k = start; k < end; ++k )
	{
		int i = a + k;
		bool shell = filed && ( f->asleep[i] & B2F_ASLEEP_NEW ) != 0;
		b2Pos p = f->pos[i];
		float s = b2fLatticeSize( f->size[i] ), reach = s + ( grid->largest > s ? grid->largest : s ) + grid->cell;
		if ( a > 0 && p.x >= box.minX - reach && p.x <= box.maxX + reach && p.y >= box.minY - reach && p.y <= box.maxY + reach )
		{
			FindAwakeCell( f, i );
			int r = grid->uniformWindow > 0 ? grid->uniformWindow : f->window[i];
			int cx = f->cell[i] % width, cy = f->cell[i] / width;
			for ( int row = cy - r; row <= cy + r && shell == false; ++row )
			{
				const int* counts = f->cellAwake.data + row * stride;
				shell = counts[cx + r + 1] > counts[cx - r];
			}
		}
		f->inShell[i] = shell ? 1 : 0;
	}
}

// On every worker over the asleep particles, and then listed on one thread in particle order, so the list is the same
// at any worker count
void b2fFindShell( b2fFluid* f, b2SubStepContext* context )
{
	int a = f->activeCount, n = f->count;
	f->shellCount = 0;
	if ( f->settledCount == 0 )
	{
		f->filedNow = false;
		return;
	}

	if ( a > 0 )
	{
		int needed = f->grid.height * ( f->grid.width + 1 );
		b2fArray_ResizeGrow( f->cellAwake, needed );
		b2SubStep_ParallelFor( context, CellRowTask, f->grid.height, 16, f );
	}
	b2SubStep_ParallelFor( context, ShellTask, n - a, 512, f );
	f->filedNow = false;
	for ( int i = a; i < n; ++i )
	{
		if ( f->inShell[i] )
		{
			f->shell[f->shellCount++] = i;
		}
	}
}

// Asleep particles round an awake body that forces or a motion move wake before the substep moves anything, since the
// body may move into them: those within its shapes' bounds grown by half their larger side. MarkHeld holds awake the
// awake particles in the cells those bounds touch, which takes in all of these, so none woken here falls asleep again
// while the body is near. The grid's cells only gather candidates, so that particles far off along its edge, clamped
// into its cells, stay asleep.
bool b2fWakeHeld( b2fFluid* f )
{
	if ( f->settledCount == 0 || f->sleepOn == false )
	{
		return false;
	}

	const b2fGrid* grid = &f->sleepGrid;
	int width = grid->width, height = grid->height, woke = 0;
	for ( int si = 0; si < f->shapes.count; ++si )
	{
		const b2fShape* s = f->shapes.data + si;
		if ( f->bodies.data[s->bodyIndex].awake == false )
		{
			continue;
		}

		float grow = 0.5f * (float)( s->maxX - s->minX > s->maxY - s->minY ? s->maxX - s->minX : s->maxY - s->minY );
		b2fCoord minX = s->minX - grow, maxX = s->maxX + grow, minY = s->minY - grow, maxY = s->maxY + grow;
		if ( maxX < grid->all.minX || maxY < grid->all.minY || minX > grid->all.maxX || minY > grid->all.maxY )
		{
			continue;
		}
		// Clamped as the particles' cells are, so every particle inside the bounds is in a cell of the range
		int cx0, cx1, cy0, cy1;
		b2fGridRange( grid, &grid->ax, width, minX, maxX, &cx0, &cx1 );
		b2fGridRange( grid, &grid->ay, height, minY, maxY, &cy0, &cy1 );
		for ( int y = cy0; y <= cy1; ++y )
		{
			for ( int c = y * width + cx0, last = y * width + cx1; c <= last; ++c )
			{
				for ( int k = grid->start.data[c]; k < grid->start.data[c + 1]; ++k )
				{
					// Just filed, the grid holds where particles are to go
					int i = f->filePending ? f->order[k] : k;
					b2Pos p = f->pos[i];
					if ( f->asleep[i] != 0 && minX <= p.x && p.x <= maxX && minY <= p.y && p.y <= maxY )
					{
						// Its sleep time starts again. With these bounds grown by a cell more than MarkHeld's, a particle
						// woken outside the cells it holds that kept its sleep time fell asleep again at the end of the
						// substep, and woke in the next: mercury round a stone ball flickered so, and held the ball
						// rocking in its shaft.
						b2fWakeParticle( f, i );
						woke += 1;
					}
				}
			}
		}
	}
	f->awakeSettled += woke;
	f->asleepCount -= woke;
	return woke > 0;
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

// The groups are the particles of liquids with springs joined through the spring table, found by union and find over
// the springs once, on one thread, which takes no ordering between workers. That runs only when a group may change
// state: when some particle is ready to fall asleep, or the count of asleep ones has changed since the last call, since
// then something has woken (falling asleep happens only here). A spring forms only between awake particles, so a group
// is never joined to another while one of them is asleep.
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
	// Every spring once, for its pair. A spring whose end has gone, or is of another liquid, joins nothing: the next
	// spring pass removes it without working it out.
	const b2fSpring* springs = f->springs.data;
	for ( int s = 0; s < f->springs.count; ++s )
	{
		int i = springs[s].rest >= 0.0f ? b2fFindParticle( f, springs[s].a ) : -1;
		int j = i >= 0 ? b2fFindParticle( f, springs[s].b ) : -1;
		if ( j < 0 || f->liquid[i] != f->liquid[j] )
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

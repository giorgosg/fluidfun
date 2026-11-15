// SPDX-License-Identifier: MIT
// Sorts particles into grid cells, so neighbours are found in the cells around a particle. Cells fit the finest
// particles, and each particle searches as many cells around its own as its largest possible neighbour needs.
#include "fluid_internal.h"

// Particles i and j interact closer than s_i + s_j, with sizes on the solver's lattice, as everywhere in this file. If
// j is the larger, they are closer than 2 s_j, so each block takes the size of the largest particle within twice its
// size of the block. A particle's reach is then its own size plus the larger of its own and its block's, which finds
// every neighbour, so pairs are always found from both sides.
// Points k blocks apart are at least k - 1 blocks apart, so a size reaches ceil( 2 size / block ) blocks.
static void SpreadBlockSizes( b2fFluid* f, int bw, int bh )
{
	memset( f->blockReach, 0, (size_t)( bw * bh ) * sizeof( float ) );
	float blockLength = B2F_BLOCK_CELLS * f->gridCell;
	for ( int by = 0; by < bh; ++by )
	{
		for ( int bx = 0; bx < bw; ++bx )
		{
			float size = f->blockSize[by * bw + bx];
			if ( size <= 0.0f )
			{
				continue;
			}

			int k = (int)ceilf( 2.0f * size / blockLength );
			int x0 = bx > k ? bx - k : 0, x1 = bx + k < bw - 1 ? bx + k : bw - 1;
			int y0 = by > k ? by - k : 0, y1 = by + k < bh - 1 ? by + k : bh - 1;
			for ( int y = y0; y <= y1; ++y )
			{
				for ( int x = x0; x <= x1; ++x )
				{
					float* reach = f->blockReach + y * bw + x;
					*reach = *reach > size ? *reach : size;
				}
			}
		}
	}
}

// The grid has at most this many cells a side, and this many in all for n particles
#define B2F_GRID_SIDE 8192
static int64_t CellLimit( int n )
{
	return 16 * (int64_t)n > ( 1 << 16 ) ? 16 * (int64_t)n : ( 1 << 16 );
}

// Cells along one side of a grid over a box, border included, or more than B2F_GRID_SIDE for any side too long, without
// overflowing the conversion for a particle however far away. The same arithmetic as the grid's own.
static int CellsAcross( b2fCoord lo, b2fCoord hi, float cell, int border )
{
	float t = (float)( hi - lo ) / cell;
	return t < (float)B2F_GRID_SIDE ? (int)t + 1 + 2 * border : B2F_GRID_SIDE + 1;
}

static bool GridFits( const b2fBox* box, float cell, int border, int64_t limit )
{
	int width = CellsAcross( box->minX, box->maxX, cell, border ), height = CellsAcross( box->minY, box->maxY, cell, border );
	return width <= B2F_GRID_SIDE && height <= B2F_GRID_SIDE && (int64_t)width * height <= limit;
}

static inline b2fCoord Along( const b2fFluid* f, int i, int axis )
{
	return axis == 0 ? f->pos[i].x : f->pos[i].y;
}

// Bins of the histogram that finds the bulk along an axis
#define B2F_BULK_BINS 4096
// A window this many bins long or longer is placed by the histogram; a shorter one is first zoomed into
#define B2F_BULK_ZOOM 64

// Along one axis, the stretch no longer than length that holds the most particles, from a histogram over [lo, hi],
// which every particle is in at first: bounds of the particles in the run of bins that fits in length and holds the
// most, the first such run. Where the bins are too coarse to place it well, the histogram is taken again over the run
// that holds the most, two bins longer than what fits, which narrows the range by at least B2F_BULK_BINS / 66 a time.
static void DensestRun( const b2fFluid* f, int axis, double length, b2fCoord* lo, b2fCoord* hi )
{
	int counts[B2F_BULK_BINS];
	b2fCoord a = *lo, b = *hi;
	for ( int zoom = 0; zoom < 8 && (double)( b - a ) > length; ++zoom )
	{
		double bin = (double)( b - a ) / B2F_BULK_BINS;
		memset( counts, 0, sizeof( counts ) );
		for ( int i = 0; i < f->count; ++i )
		{
			b2fCoord v = Along( f, i, axis );
			if ( a <= v && v <= b )
			{
				int k = (int)( (double)( v - a ) / bin );
				counts[k < B2F_BULK_BINS ? k : B2F_BULK_BINS - 1] += 1;
			}
		}

		int fit = (int)( length / bin );
		bool placed = fit >= B2F_BULK_ZOOM;
		int run = placed ? fit : ( fit > 1 ? fit : 1 ) + 2;
		run = run < B2F_BULK_BINS ? run : B2F_BULK_BINS;
		int sum = 0, best = -1, start = 0;
		for ( int k = 0; k < B2F_BULK_BINS; ++k )
		{
			sum += counts[k] - ( k >= run ? counts[k - run] : 0 );
			if ( k >= run - 1 && sum > best )
			{
				best = sum;
				start = k - run + 1;
			}
		}
		b2fCoord na = a + (b2fCoord)( start * bin ), nb = start + run < B2F_BULK_BINS ? a + (b2fCoord)( ( start + run ) * bin ) : b;
		a = na;
		b = nb;
		if ( placed )
		{
			break;
		}
	}

	// The particles in the stretch, from their own bounds
	b2fCoord low = b, high = a;
	for ( int i = 0; i < f->count; ++i )
	{
		b2fCoord v = Along( f, i, axis );
		if ( a <= v && v <= b )
		{
			low = v < low ? v : low;
			high = v > high ? v : high;
		}
	}
	*lo = low;
	*hi = high;
}

// Sets the bulk along one axis to the stretch of all that holds the most particles within cells cells of the grid's
// side, border included. Returns false if that leaves no room.
static bool Widen( const b2fFluid* f, int axis, int64_t cells, float cell, int border, const b2fBox* all, b2fBox* bulk )
{
	int64_t room = ( cells < B2F_GRID_SIDE ? cells : B2F_GRID_SIDE ) - 2 - 2 * border;
	if ( room < 1 )
	{
		return false;
	}
	b2fCoord* lo = axis == 0 ? &bulk->minX : &bulk->minY;
	b2fCoord* hi = axis == 0 ? &bulk->maxX : &bulk->maxY;
	*lo = axis == 0 ? all->minX : all->minY;
	*hi = axis == 0 ? all->maxX : all->maxY;
	DensestRun( f, axis, (double)cell * (double)room, lo, hi );
	return true;
}

bool b2fFindBulk( const b2fFluid* f, float cell, int border, b2fBox* all, b2fBox* bulk )
{
	b2fBox box = { f->pos[0].x, f->pos[0].y, f->pos[0].x, f->pos[0].y };
	for ( int i = 1; i < f->count; ++i )
	{
		box.minX = box.minX < f->pos[i].x ? box.minX : f->pos[i].x;
		box.maxX = box.maxX > f->pos[i].x ? box.maxX : f->pos[i].x;
		box.minY = box.minY < f->pos[i].y ? box.minY : f->pos[i].y;
		box.maxY = box.maxY > f->pos[i].y ? box.maxY : f->pos[i].y;
	}
	*all = box;
	*bulk = box;
	int64_t limit = CellLimit( f->count );
	if ( GridFits( &box, cell, border, limit ) )
	{
		return false;
	}

	// First the most particles each axis holds within the side of a square grid, which finds the densest part of the
	// liquid along each. Then along x the most within the width that part's height leaves, and along y the most
	// within the height that width leaves, so a long pool is not cut short. Each run's bounds are its particles', which
	// leaves the other axis all the room it can have. Two cells short of each limit, for rounding.
	int64_t square = (int64_t)sqrt( (double)limit );
	if ( Widen( f, 0, square, cell, border, &box, bulk ) == false || Widen( f, 1, square, cell, border, &box, bulk ) == false ||
		 Widen( f, 0, limit / CellsAcross( bulk->minY, bulk->maxY, cell, border ), cell, border, &box, bulk ) == false ||
		 Widen( f, 1, limit / CellsAcross( bulk->minX, bulk->maxX, cell, border ), cell, border, &box, bulk ) == false )
	{
		*bulk = box;
		return false;
	}
	if ( GridFits( bulk, cell, border, limit ) == false )
	{
		*bulk = box;
		return false;
	}

	// Liquid split into parts of similar size has no bulk: filed into edge cells, the rest would cost more than larger
	// cells do
	int outside = 0;
	for ( int i = 0; i < f->count; ++i )
	{
		b2Pos p = f->pos[i];
		outside += p.x < bulk->minX || p.x > bulk->maxX || p.y < bulk->minY || p.y > bulk->maxY ? 1 : 0;
	}
	if ( outside > f->count / B2F_BULK_OUTSIDE )
	{
		*bulk = box;
		return false;
	}
	return true;
}

static void BuildGrid( b2fFluid* f )
{
	bool adaptive = f->adaptive;
	float largest = 0.5f * f->radius;
	if ( adaptive )
	{
		for ( int i = 0; i < f->count; ++i )
		{
			float size = b2fLatticeSize( f->size[i] );
			largest = largest > size ? largest : size;
		}
		f->maxRadius = 2.0f * largest;
	}

	// The border holds the widest search: a reach of at most twice the largest size, and one more cell for the merge
	// search, which runs after particles have moved. Uniform sizes keep one border cell.
	// The grid covers every particle when that fits at the smallest cell. Otherwise it covers the bulk of the
	// liquid when that fits (b2fFindBulk), and the particles outside go in the cells along its edge, the nearest to
	// them. Cells only gather candidates, which every pass accepts by their distance, so a particle in the wrong cell
	// is still right: it finds no neighbours. Clamping keeps the order of coordinates, and moves two points no further
	// apart in cells, so particles out there still find each other. Liquid spread thinly over a wide area, with no
	// bulk that fits, gets larger cells, so the grid stays in proportion to the particle count.
	float cs = f->cellSize;
	int border = adaptive ? (int)ceilf( 2.0f * largest / cs ) + 1 : 1;
	b2fBox all, box;
	f->gridBulkOnly = b2fFindBulk( f, cs, border, &all, &box );
	int64_t cellLimit = CellLimit( f->count );
	int width, height;
	while ( true )
	{
		border = adaptive ? (int)ceilf( 2.0f * largest / cs ) + 1 : 1;
		width = CellsAcross( box.minX, box.maxX, cs, border );
		height = CellsAcross( box.minY, box.maxY, cs, border );
		if ( width <= B2F_GRID_SIDE && height <= B2F_GRID_SIDE && (int64_t)width * height <= cellLimit )
		{
			break;
		}
		cs *= 2.0f;
	}
	f->gridBulk = box;
	f->gridCell = cs;
	f->gridBorder = border;
	f->uniformWindow = adaptive ? 0 : (int)ceilf( f->radius / cs );
	f->gridX = box.minX - border * cs;
	f->gridY = box.minY - border * cs;
	f->gridWidth = width;
	f->gridHeight = height;

	int cellCount = width * height;
	if ( cellCount + 2 > f->gridCapacity )
	{
		f->cellStart = b2fGrowArray( f->cellStart, f->gridCapacity, 2 * cellCount + 2, sizeof( int ) );
		f->gridCapacity = 2 * cellCount + 2;
	}
	memset( f->cellStart, 0, ( (size_t)cellCount + 2 ) * sizeof( int ) );

	int bw = ( width + B2F_BLOCK_CELLS - 1 ) >> B2F_BLOCK_SHIFT, bh = ( height + B2F_BLOCK_CELLS - 1 ) >> B2F_BLOCK_SHIFT;
	if ( adaptive )
	{
		if ( bw * bh > f->blockCapacity )
		{
			f->blockSize = b2fGrowArray( f->blockSize, f->blockCapacity, 2 * bw * bh, sizeof( float ) );
			f->blockReach = b2fGrowArray( f->blockReach, f->blockCapacity, 2 * bw * bh, sizeof( float ) );
			f->blockCapacity = 2 * bw * bh;
		}
		memset( f->blockSize, 0, (size_t)( bw * bh ) * sizeof( float ) );
	}

	// Clamped in float before the conversion, so a particle however far out is in range
	float inv = 1.0f / cs;
	float firstCell = (float)border, lastX = (float)( width - 1 - border ), lastY = (float)( height - 1 - border );
	for ( int i = 0; i < f->count; ++i )
	{
		float tx = (float)( f->pos[i].x - f->gridX ) * inv, ty = (float)( f->pos[i].y - f->gridY ) * inv;
		int cx = (int)( tx > firstCell ? ( tx < lastX ? tx : lastX ) : firstCell );
		int cy = (int)( ty > firstCell ? ( ty < lastY ? ty : lastY ) : firstCell );
		int c = cy * width + cx;
		f->sortCell[i] = c;
		f->cellStart[c + 1] += 1;
		if ( adaptive )
		{
			int b = ( cy >> B2F_BLOCK_SHIFT ) * bw + ( cx >> B2F_BLOCK_SHIFT );
			f->sortBlock[i] = b;
			float size = b2fLatticeSize( f->size[i] );
			f->blockSize[b] = f->blockSize[b] > size ? f->blockSize[b] : size;
		}
	}

	for ( int c = 0; c < cellCount; ++c )
	{
		f->cellStart[c + 1] += f->cellStart[c];
	}

	// Counting sort, keeping the previous order within a cell
	for ( int i = 0; i < f->count; ++i )
	{
		f->permutation[f->cellStart[f->sortCell[i]]++] = i;
	}
	for ( int c = cellCount; c > 0; --c )
	{
		f->cellStart[c] = f->cellStart[c - 1];
	}
	f->cellStart[0] = 0;

	if ( adaptive )
	{
		SpreadBlockSizes( f, bw, bh );
	}
}

// Moves each particle to its sorted place, and gives it its cell and, with sizes that vary, how many cells around it
// to search
static void ReorderTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float inverse = 1.0f / f->gridCell;
	for ( int i = start; i < end; ++i )
	{
		int p = f->permutation[i];
#define B2F_REORDER_FIELD( type, name ) f->sorted.name[i] = f->name[p];
		B2F_PARTICLE_FIELDS( B2F_REORDER_FIELD )
#undef B2F_REORDER_FIELD
		if ( f->springs != NULL )
		{
			f->sortedSprings[i] = f->springs[p];
		}

		f->cell[i] = f->sortCell[p];
		if ( f->uniformWindow == 0 )
		{
			float near = f->blockReach[f->sortBlock[p]];
			float s = b2fLatticeSize( f->size[p] ), reach = s + ( near > s ? near : s );
			f->window[i] = (int)ceilf( reach * inverse );
		}
	}
}

static void SwapArrays( b2fFluid* f )
{
#define B2F_SWAP_FIELD( type, name )                                                                                   \
	{                                                                                                                  \
		type* t = f->name;                                                                                             \
		f->name = f->sorted.name;                                                                                      \
		f->sorted.name = t;                                                                                            \
	}
	B2F_PARTICLE_FIELDS( B2F_SWAP_FIELD )
#undef B2F_SWAP_FIELD
	if ( f->springs != NULL )
	{
		b2fSprings* t = f->springs;
		f->springs = f->sortedSprings;
		f->sortedSprings = t;
	}
}

void b2fSortParticles( b2fFluid* f, b2SubStepContext* context )
{
	BuildGrid( f );
	b2SubStep_ParallelFor( context, ReorderTask, f->count, 1024, f );
	SwapArrays( f );
}

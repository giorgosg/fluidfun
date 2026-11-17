// SPDX-License-Identifier: MIT
// Sorts particles into grid cells, so neighbours are found in the cells around a particle. Cells fit the finest
// particles, and each particle searches as many cells around its own as its largest possible neighbour needs.
#include "fluid_internal.h"

// Particles i and j interact closer than s_i + s_j, with sizes on the solver's lattice, as everywhere in this file. If
// j is the larger, they are closer than 2 s_j, so each block takes the size of the largest particle within twice its
// size of the block. A particle's reach is then its own size plus the larger of its own and its block's, which finds
// every neighbour, so pairs are always found from both sides.
// Points k blocks apart are at least k - 1 blocks apart, so a size reaches ceil( 2 size / block ) blocks.
static void SpreadBlockSizes( b2fGrid* grid, int bw, int bh )
{
	memset( grid->blockReach, 0, (size_t)( bw * bh ) * sizeof( float ) );
	float blockLength = B2F_BLOCK_CELLS * grid->cell;
	for ( int by = 0; by < bh; ++by )
	{
		for ( int bx = 0; bx < bw; ++bx )
		{
			float size = grid->blockSize[by * bw + bx];
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
					float* reach = grid->blockReach + y * bw + x;
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

// The particles of a run: those listed from first to last, or with no list the particles first to last themselves
#define FOR_RUN( list, first, last, i ) for ( int k_ = ( first ), i = 0; k_ < ( last ) && ( i = ( list ) != NULL ? ( list )[k_] : k_, true ); ++k_ )

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
static void DensestRun( const b2fFluid* f, const int* list, int first, int last, int axis, double length, b2fCoord* lo, b2fCoord* hi )
{
	int counts[B2F_BULK_BINS];
	b2fCoord a = *lo, b = *hi;
	for ( int zoom = 0; zoom < 8 && (double)( b - a ) > length; ++zoom )
	{
		double bin = (double)( b - a ) / B2F_BULK_BINS;
		memset( counts, 0, sizeof( counts ) );
		FOR_RUN( list, first, last, i )
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
	FOR_RUN( list, first, last, i )
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
static bool Widen( const b2fFluid* f, const int* list, int first, int last, int axis, int64_t cells, float cell, int border, const b2fBox* all,
				   b2fBox* bulk )
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
	DensestRun( f, list, first, last, axis, (double)cell * (double)room, lo, hi );
	return true;
}

bool b2fFindBulk( const b2fFluid* f, const int* list, int first, int last, float cell, int border, b2fBox* all, b2fBox* bulk )
{
	int i0 = list != NULL ? list[first] : first;
	b2fBox box = { f->pos[i0].x, f->pos[i0].y, f->pos[i0].x, f->pos[i0].y };
	FOR_RUN( list, first + 1, last, i )
	{
		box.minX = box.minX < f->pos[i].x ? box.minX : f->pos[i].x;
		box.maxX = box.maxX > f->pos[i].x ? box.maxX : f->pos[i].x;
		box.minY = box.minY < f->pos[i].y ? box.minY : f->pos[i].y;
		box.maxY = box.maxY > f->pos[i].y ? box.maxY : f->pos[i].y;
	}
	*all = box;
	*bulk = box;
	int64_t limit = CellLimit( last - first );
	if ( GridFits( &box, cell, border, limit ) )
	{
		return false;
	}

	// First the most particles each axis holds within the side of a square grid, which finds the densest part of the
	// liquid along each. Then along x the most within the width that part's height leaves, and along y the most
	// within the height that width leaves, so a long pool is not cut short. Each run's bounds are its particles', which
	// leaves the other axis all the room it can have. Two cells short of each limit, for rounding.
	int64_t square = (int64_t)sqrt( (double)limit );
	if ( Widen( f, list, first, last, 0, square, cell, border, &box, bulk ) == false ||
		 Widen( f, list, first, last, 1, square, cell, border, &box, bulk ) == false ||
		 Widen( f, list, first, last, 0, limit / CellsAcross( bulk->minY, bulk->maxY, cell, border ), cell, border, &box, bulk ) == false ||
		 Widen( f, list, first, last, 1, limit / CellsAcross( bulk->minX, bulk->maxX, cell, border ), cell, border, &box, bulk ) == false )
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
	FOR_RUN( list, first, last, i )
	{
		b2Pos p = f->pos[i];
		outside += p.x < bulk->minX || p.x > bulk->maxX || p.y < bulk->minY || p.y > bulk->maxY ? 1 : 0;
	}
	if ( outside > ( last - first ) / B2F_BULK_OUTSIDE )
	{
		*bulk = box;
		return false;
	}
	return true;
}


// The largest particle any liquid can have, on the solver's lattice: adaptive sizes merge up to adaptiveRatio times the
// finest. The sleep grid's border makes room for it, so an awake particle grown since the grid was made still finds
// its neighbours there.
static float LargestPossible( const b2fFluid* f )
{
	float largest = 0.5f * f->radius;
	if ( f->adaptive )
	{
		for ( int l = 0; l < f->liquidCount; ++l )
		{
			float ratio = f->liquids[l].def.tuning.adaptiveRatio;
			float size = b2fLatticeSize( ratio * f->def.particleSpacing );
			largest = f->liquids[l].alive && size > largest ? size : largest;
		}
	}
	return largest;
}

// Lays a grid over the particles first to last and counts them into its cells: their cells in sortCell, and in
// permutation, from first on, the particle each sorted place takes. The border makes room for particles as large as
// room, as well as the largest in the run.
static void BuildGrid( b2fFluid* f, b2fGrid* grid, const int* list, int first, int last, float room )
{
	bool adaptive = f->adaptive;
	float largest = 0.5f * f->radius;
	if ( adaptive )
	{
		FOR_RUN( list, first, last, i )
		{
			float size = b2fLatticeSize( f->size[i] );
			largest = largest > size ? largest : size;
		}
	}
	grid->sizeLargest = largest;
	largest = largest > room ? largest : room;
	grid->largest = largest;

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
	b2fBox box;
	grid->bulkOnly = b2fFindBulk( f, list, first, last, cs, border, &grid->all, &box );
	int64_t cellLimit = CellLimit( last - first );
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
	grid->bulk = box;
	grid->cell = cs;
	grid->border = border;
	grid->uniformWindow = adaptive ? 0 : (int)ceilf( f->radius / cs );
	grid->x = box.minX - border * cs;
	grid->y = box.minY - border * cs;
	grid->width = width;
	grid->height = height;

	int cellCount = width * height;
	if ( cellCount + 2 > grid->capacity )
	{
		grid->start = b2fGrowArray( grid->start, grid->capacity, 2 * cellCount + 2, sizeof( int ) );
		grid->capacity = 2 * cellCount + 2;
	}
	int* start = grid->start;
	memset( start, 0, ( (size_t)cellCount + 2 ) * sizeof( int ) );

	int bw = ( width + B2F_BLOCK_CELLS - 1 ) >> B2F_BLOCK_SHIFT, bh = ( height + B2F_BLOCK_CELLS - 1 ) >> B2F_BLOCK_SHIFT;
	if ( adaptive )
	{
		if ( bw * bh > grid->blockCapacity )
		{
			grid->blockSize = b2fGrowArray( grid->blockSize, grid->blockCapacity, 2 * bw * bh, sizeof( float ) );
			grid->blockReach = b2fGrowArray( grid->blockReach, grid->blockCapacity, 2 * bw * bh, sizeof( float ) );
			grid->blockCapacity = 2 * bw * bh;
		}
		memset( grid->blockSize, 0, (size_t)( bw * bh ) * sizeof( float ) );
	}

	// Clamped in float before the conversion, so a particle however far out is in range
	float inv = 1.0f / cs;
	float firstCell = (float)border, lastX = (float)( width - 1 - border ), lastY = (float)( height - 1 - border );
	FOR_RUN( list, first, last, i )
	{
		float tx = (float)( f->pos[i].x - grid->x ) * inv, ty = (float)( f->pos[i].y - grid->y ) * inv;
		int cx = (int)( tx > firstCell ? ( tx < lastX ? tx : lastX ) : firstCell );
		int cy = (int)( ty > firstCell ? ( ty < lastY ? ty : lastY ) : firstCell );
		int c = cy * width + cx;
		f->sortCell[i] = c;
		start[c + 1] += 1;
		if ( adaptive )
		{
			int b = ( cy >> B2F_BLOCK_SHIFT ) * bw + ( cx >> B2F_BLOCK_SHIFT );
			f->sortBlock[i] = b;
			float size = b2fLatticeSize( f->size[i] );
			grid->blockSize[b] = grid->blockSize[b] > size ? grid->blockSize[b] : size;
		}
	}

	start[0] = first;
	for ( int c = 0; c < cellCount; ++c )
	{
		start[c + 1] += start[c];
	}

	// Counting sort, keeping the previous order within a cell
	FOR_RUN( list, first, last, i )
	{
		f->permutation[start[f->sortCell[i]]++] = i;
	}
	for ( int c = cellCount; c > 0; --c )
	{
		start[c] = start[c - 1];
	}
	start[0] = first;

	if ( adaptive )
	{
		SpreadBlockSizes( grid, bw, bh );
	}
}

// How many cells each side of its own a particle of this size in this block searches in a grid: its own size plus the
// larger of its own and the largest whose reach touches the block
static inline int GridWindow( const b2fGrid* grid, int block, float size )
{
	float near = grid->blockReach[block], s = b2fLatticeSize( size ), reach = s + ( near > s ? near : s );
	return (int)ceilf( reach / grid->cell );
}

// A particle's cell and window in the sleep grid, from where it is. One far enough outside the asleep particles that
// it can reach none of them searches no cells there. The window never passes the border, which made room for the
// largest particle any liquid can have; one larger than that, which the next sort files again for, finds only those
// within the border's reach for a substep.
static void FindSleepCell( b2fFluid* f, int i, b2Pos p, float size )
{
	const b2fGrid* grid = &f->sleepGrid;
	float s = b2fLatticeSize( size ), reach = s + ( grid->largest > s ? grid->largest : s ) + grid->cell;
	if ( p.x < grid->all.minX - reach || p.x > grid->all.maxX + reach || p.y < grid->all.minY - reach || p.y > grid->all.maxY + reach )
	{
		f->sleepCell[i] = grid->border * grid->width + grid->border;
		f->sleepWindow[i] = -1;
		return;
	}

	float inv = 1.0f / grid->cell;
	float firstCell = (float)grid->border, lastX = (float)( grid->width - 1 - grid->border ), lastY = (float)( grid->height - 1 - grid->border );
	float tx = (float)( p.x - grid->x ) * inv, ty = (float)( p.y - grid->y ) * inv;
	int cx = (int)( tx > firstCell ? ( tx < lastX ? tx : lastX ) : firstCell );
	int cy = (int)( ty > firstCell ? ( ty < lastY ? ty : lastY ) : firstCell );
	f->sleepCell[i] = cy * grid->width + cx;
	int window = grid->uniformWindow;
	if ( window == 0 )
	{
		int bw = ( grid->width + B2F_BLOCK_CELLS - 1 ) >> B2F_BLOCK_SHIFT;
		window = GridWindow( grid, ( cy >> B2F_BLOCK_SHIFT ) * bw + ( cx >> B2F_BLOCK_SHIFT ), size );
		window = window < grid->border - 1 ? window : grid->border - 1;
	}
	f->sleepWindow[i] = window;
}

typedef struct b2fReorder
{
	b2fFluid* f;
	int first;
	bool sleepFields;
} b2fReorder;

// Moves each particle of a run to its sorted place. Awake particles take their cells in both grids and, with sizes that
// vary, how many cells around them to search in the grid of awake ones; asleep ones their cells in the sleep grid.
static void ReorderTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fReorder* reorder = context;
	b2fFluid* f = reorder->f;
	bool sleepFields = reorder->sleepFields;
	int active = f->activeCount;
	for ( int k = start; k < end; ++k )
	{
		int i = reorder->first + k;
		int p = f->permutation[i];
#define B2F_REORDER_FIELD( type, name ) f->sorted.name[i] = f->name[p];
		B2F_CORE_FIELDS( B2F_REORDER_FIELD )
		if ( sleepFields )
		{
			B2F_SLEEP_FIELDS( B2F_REORDER_FIELD )
		}
		if ( f->speedMemory != NULL )
		{
			B2F_SPEED_FIELDS( B2F_REORDER_FIELD )
		}
#undef B2F_REORDER_FIELD
		if ( f->springs != NULL )
		{
			f->sortedSprings[i] = f->springs[p];
		}

		if ( i < active )
		{
			f->cell[i] = f->sortCell[p];
			if ( f->grid.uniformWindow == 0 )
			{
				f->window[i] = GridWindow( &f->grid, f->sortBlock[p], f->size[p] );
			}
			if ( f->settledCount > 0 )
			{
				FindSleepCell( f, i, f->pos[p], f->size[p] );
			}
		}
		else
		{
			// It holds still where it is, as prediction holds an asleep particle among the awake
			const b2fGrid* grid = &f->sleepGrid;
			f->sorted.prev[i] = f->pos[p];
			f->sleepCell[i] = f->sortCell[p];
			f->sleepWindow[i] = grid->uniformWindow > 0 ? grid->uniformWindow : GridWindow( grid, f->sortBlock[p], f->size[p] );
		}
	}
}

// After a reorder of the run first to last: the sorted arrays hold it, and take the place of the others when the run is
// every particle. Otherwise the run is copied back, so the particles outside it, which the sorted arrays do not hold,
// stay as they are.
static void FinishReorder( b2fFluid* f, int first, int last, bool sleepFields )
{
	if ( first == 0 && last == f->count )
	{
#define B2F_SWAP_FIELD( type, name )                                                                                   \
	{                                                                                                                  \
		type* t = f->name;                                                                                             \
		f->name = f->sorted.name;                                                                                      \
		f->sorted.name = t;                                                                                            \
	}
		B2F_CORE_FIELDS( B2F_SWAP_FIELD )
		if ( sleepFields )
		{
			B2F_SLEEP_FIELDS( B2F_SWAP_FIELD )
		}
		if ( f->speedMemory != NULL )
		{
			B2F_SPEED_FIELDS( B2F_SWAP_FIELD )
		}
#undef B2F_SWAP_FIELD
		if ( f->springs != NULL )
		{
			b2fSprings* t = f->springs;
			f->springs = f->sortedSprings;
			f->sortedSprings = t;
		}
		return;
	}

	size_t n = (size_t)( last - first );
#define B2F_COPY_BACK( type, name ) memcpy( f->name + first, f->sorted.name + first, n * sizeof( type ) );
	B2F_CORE_FIELDS( B2F_COPY_BACK )
	if ( sleepFields )
	{
		B2F_SLEEP_FIELDS( B2F_COPY_BACK )
	}
	if ( f->speedMemory != NULL )
	{
		B2F_SPEED_FIELDS( B2F_COPY_BACK )
	}
#undef B2F_COPY_BACK
	if ( f->springs != NULL )
	{
		memcpy( f->springs + first, f->sortedSprings + first, n * sizeof( b2fSprings ) );
	}
}

static void Reorder( b2fFluid* f, b2SubStepContext* context, int first, int last, bool sleepFields )
{
	b2fReorder reorder = { f, first, sleepFields };
	b2SubStep_ParallelFor( context, ReorderTask, last - first, 1024, &reorder );
	FinishReorder( f, first, last, sleepFields );
}

bool b2fNeedsFiling( const b2fFluid* f )
{
	return f->gridStale || f->asleepActive > 0 || f->awakeSettled > 0 || f->activeCount + f->settledCount != f->count;
}

// The order filing puts the particles in, awake ones first, in order, and asleep ones after, sorted into the sleep grid,
// takes effect with the next sort of the awake particles, which moves every particle once (b2fSortAwake). Until then
// order lists the particles, as they still are, in that order, and the sleep grid holds them by where they will be:
// particle order[k] is to go to k.
void b2fFileAsleep( b2fFluid* f, b2SubStepContext* context )
{
	(void)context;
	int n = f->count;
	f->gridStale = false;
	f->asleepActive = 0;
	f->awakeSettled = 0;
	f->shellCount = 0;

	int awake = 0;
	for ( int i = 0; i < n; ++i )
	{
		awake += f->asleep[i] == 0 ? 1 : 0;
	}
	f->asleepCount = n - awake;
	f->activeCount = awake;
	f->settledCount = n - awake;
	f->filedNow = awake < n;
	if ( awake == n )
	{
		// None asleep: they are in order already
		f->filePending = false;
		return;
	}

	for ( int i = 0, a = 0, k = awake; i < n; ++i )
	{
		f->order[f->asleep[i] == 0 ? a++ : k++] = i;
	}
	BuildGrid( f, &f->sleepGrid, f->order, awake, n, LargestPossible( f ) );
	f->filePending = true;
}

void b2fSortAwake( b2fFluid* f, b2SubStepContext* context )
{
	int a = f->activeCount, n = f->count;
	bool pending = f->filePending;
	f->filePending = false;
	if ( a == 0 )
	{
		if ( pending )
		{
			Reorder( f, context, 0, n, true );
		}
		return;
	}

	// The border makes room for the asleep particles that search this grid too, the shell
	bool asleep = f->settledCount > 0;
	BuildGrid( f, &f->grid, pending ? f->order : NULL, 0, a, asleep ? f->sleepGrid.sizeLargest : 0.0f );
	if ( f->adaptive )
	{
		f->maxRadius = 2.0f * f->grid.largest;
	}
	if ( pending )
	{
		Reorder( f, context, 0, n, true );
	}
	else
	{
		Reorder( f, context, 0, a, f->sleepOn );
	}

	// One grown larger than the sleep grid made room for has the asleep particles filed again with room for it
	if ( asleep )
	{
		f->gridStale = f->gridStale || f->grid.sizeLargest > f->sleepGrid.largest;
	}
}

void b2fSortParticles( b2fFluid* f, b2SubStepContext* context )
{
	b2fCountAsleep( f );
	if ( b2fNeedsFiling( f ) )
	{
		b2fFileAsleep( f, context );
	}
	b2fSortAwake( f, context );
}

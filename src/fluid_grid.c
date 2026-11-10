// SPDX-License-Identifier: MIT
// Sorts particles into grid cells, so neighbours are found in the cells around a particle. Cells fit the finest
// particles, and each particle searches as many cells around its own as its largest possible neighbour needs.
#include "fluid_internal.h"

// Blocks of this many cells a side, as a power of two, carry the largest particle size near them
#define B2F_BLOCK_SHIFT 2
#define B2F_BLOCK_CELLS ( 1 << B2F_BLOCK_SHIFT )

// Particles i and j interact closer than s_i + s_j. If j is the larger, they are closer than 2 s_j, so each block
// takes the size of the largest particle within twice its size of the block. A particle's reach is then its own size
// plus the larger of its own and its block's, which finds every neighbour, so pairs are always found from both sides.
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

static void BuildGrid( b2fFluid* f )
{
	bool adaptive = f->adaptive;
	b2fCoord minX = f->pos[0].x, maxX = f->pos[0].x, minY = f->pos[0].y, maxY = f->pos[0].y;
	for ( int i = 1; i < f->count; ++i )
	{
		minX = minX < f->pos[i].x ? minX : f->pos[i].x;
		maxX = maxX > f->pos[i].x ? maxX : f->pos[i].x;
		minY = minY < f->pos[i].y ? minY : f->pos[i].y;
		maxY = maxY > f->pos[i].y ? maxY : f->pos[i].y;
	}
	float largest = 0.5f * f->radius;
	if ( adaptive )
	{
		for ( int i = 0; i < f->count; ++i )
		{
			largest = largest > f->size[i] ? largest : f->size[i];
		}
		f->maxRadius = 2.0f * largest;
	}

	// Liquid spread thinly over a wide area gets larger cells, so the grid stays in proportion to the particle count.
	// The border holds the widest search: a reach of at most twice the largest size, and one more cell for the
	// merge search, which runs after particles have moved. Uniform sizes keep one border cell.
	float cs = f->cellSize;
	int width, height, border;
	int64_t cellLimit = 16 * (int64_t)f->count > ( 1 << 16 ) ? 16 * (int64_t)f->count : ( 1 << 16 );
	while ( true )
	{
		border = adaptive ? (int)ceilf( 2.0f * largest / cs ) + 1 : 1;
		width = (int)( (float)( maxX - minX ) / cs ) + 1 + 2 * border;
		height = (int)( (float)( maxY - minY ) / cs ) + 1 + 2 * border;
		if ( width <= 8192 && height <= 8192 && (int64_t)width * height <= cellLimit )
		{
			break;
		}
		cs *= 2.0f;
	}
	f->gridCell = cs;
	f->gridBorder = border;
	f->uniformWindow = adaptive ? 0 : (int)ceilf( f->radius / cs );
	f->gridX = minX - border * cs;
	f->gridY = minY - border * cs;
	f->gridWidth = width;
	f->gridHeight = height;

	int cellCount = width * height;
	if ( cellCount + 2 > f->gridCapacity )
	{
		f->gridCapacity = 2 * cellCount + 2;
		f->cellStart = realloc( f->cellStart, (size_t)f->gridCapacity * sizeof( int ) );
	}
	memset( f->cellStart, 0, ( (size_t)cellCount + 2 ) * sizeof( int ) );

	int bw = ( width + B2F_BLOCK_CELLS - 1 ) >> B2F_BLOCK_SHIFT, bh = ( height + B2F_BLOCK_CELLS - 1 ) >> B2F_BLOCK_SHIFT;
	if ( adaptive )
	{
		if ( bw * bh > f->blockCapacity )
		{
			f->blockCapacity = 2 * bw * bh;
			f->blockSize = realloc( f->blockSize, (size_t)f->blockCapacity * sizeof( float ) );
			f->blockReach = realloc( f->blockReach, (size_t)f->blockCapacity * sizeof( float ) );
		}
		memset( f->blockSize, 0, (size_t)( bw * bh ) * sizeof( float ) );
	}

	float inv = 1.0f / cs;
	for ( int i = 0; i < f->count; ++i )
	{
		int cx = (int)( (float)( f->pos[i].x - f->gridX ) * inv );
		int cy = (int)( (float)( f->pos[i].y - f->gridY ) * inv );
		cx = cx < border ? border : ( cx > width - 1 - border ? width - 1 - border : cx );
		cy = cy < border ? border : ( cy > height - 1 - border ? height - 1 - border : cy );
		int c = cy * width + cx;
		f->sortCell[i] = c;
		f->cellStart[c + 1] += 1;
		if ( adaptive )
		{
			int b = ( cy >> B2F_BLOCK_SHIFT ) * bw + ( cx >> B2F_BLOCK_SHIFT );
			f->sortBlock[i] = b;
			f->blockSize[b] = f->blockSize[b] > f->size[i] ? f->blockSize[b] : f->size[i];
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
			f->newIndex[p] = i;
		}

		f->cell[i] = f->sortCell[p];
		if ( f->uniformWindow == 0 )
		{
			float near = f->blockReach[f->sortBlock[p]];
			float s = f->size[p], reach = s + ( near > s ? near : s );
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
	f->gridValid = true;
	b2SubStep_ParallelFor( context, ReorderTask, f->count, 1024, f );
	SwapArrays( f );
}

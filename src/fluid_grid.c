// SPDX-License-Identifier: MIT
// Sorts particles into grid cells, so neighbours are found in the 3x3 cells around a particle.
#include "fluid_internal.h"

static void BuildGrid( b2fFluid* f )
{
	b2fCoord minX = f->pos[0].x, maxX = f->pos[0].x, minY = f->pos[0].y, maxY = f->pos[0].y;
	for ( int i = 1; i < f->count; ++i )
	{
		minX = minX < f->pos[i].x ? minX : f->pos[i].x;
		maxX = maxX > f->pos[i].x ? maxX : f->pos[i].x;
		minY = minY < f->pos[i].y ? minY : f->pos[i].y;
		maxY = maxY > f->pos[i].y ? maxY : f->pos[i].y;
	}

	// Cells must fit the largest particle, however few of them there are
	if ( f->def.adaptiveRatio > 1.0f )
	{
		float largest = f->size[0];
		for ( int i = 1; i < f->count; ++i )
		{
			largest = largest > f->size[i] ? largest : f->size[i];
		}
		f->maxRadius = 2.0f * largest;
		f->cellSize = 1.1f * f->maxRadius;
	}

	// One empty border cell on each side means the 3x3 window never needs bounds checks.
	float cs = f->cellSize;
	f->gridX = minX - cs;
	f->gridY = minY - cs;
	int width = (int)( (float)( maxX - minX ) / cs ) + 3;
	int height = (int)( (float)( maxY - minY ) / cs ) + 3;

	// Stray particles cannot be allowed to blow up the grid. They share the outer cells instead.
	width = width < 8192 ? width : 8192;
	height = height < 8192 ? height : 8192;
	f->gridWidth = width;
	f->gridHeight = height;

	int cellCount = width * height;
	if ( cellCount + 2 > f->gridCapacity )
	{
		f->gridCapacity = 2 * cellCount + 2;
		f->cellStart = realloc( f->cellStart, (size_t)f->gridCapacity * sizeof( int ) );
	}

	memset( f->cellStart, 0, ( (size_t)cellCount + 2 ) * sizeof( int ) );
	float inv = 1.0f / cs;
	for ( int i = 0; i < f->count; ++i )
	{
		int cx = (int)( (float)( f->pos[i].x - f->gridX ) * inv );
		int cy = (int)( (float)( f->pos[i].y - f->gridY ) * inv );
		cx = cx < 1 ? 1 : ( cx > width - 2 ? width - 2 : cx );
		cy = cy < 1 ? 1 : ( cy > height - 2 ? height - 2 : cy );
		int c = cy * width + cx;
		f->cell[i] = c;
		f->cellStart[c + 1] += 1;
	}

	for ( int c = 0; c < cellCount; ++c )
	{
		f->cellStart[c + 1] += f->cellStart[c];
	}

	// Counting sort, keeping the previous order within a cell
	for ( int i = 0; i < f->count; ++i )
	{
		f->permutation[f->cellStart[f->cell[i]]++] = i;
	}
	for ( int c = cellCount; c > 0; --c )
	{
		f->cellStart[c] = f->cellStart[c - 1];
	}
	f->cellStart[0] = 0;
}

static void ReorderTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	for ( int i = start; i < end; ++i )
	{
		int p = f->permutation[i];
#define B2F_REORDER_FIELD( type, name ) f->sorted.name[i] = f->name[p];
		B2F_PARTICLE_FIELDS( B2F_REORDER_FIELD )
#undef B2F_REORDER_FIELD
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
}

void b2fSortParticles( b2fFluid* f, b2SubStepContext* context )
{
	BuildGrid( f );
	f->gridValid = true;
	b2SubStep_ParallelFor( context, ReorderTask, f->count, 1024, f );
	SwapArrays( f );

	// The sort moved particles, so cell[i] has to be rebuilt in sorted order
	for ( int c = 0; c < f->gridWidth * f->gridHeight; ++c )
	{
		for ( int i = f->cellStart[c]; i < f->cellStart[c + 1]; ++i )
		{
			f->cell[i] = c;
		}
	}
}

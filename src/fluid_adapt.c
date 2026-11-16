// SPDX-License-Identifier: MIT
// Adaptive particle sizes: density boxes measure depth, and particles split, merge and trade mass toward the size
// their depth asks for, after Winchenbach et al. 2017.
#include "fluid_internal.h"

// Depth at which particles of a liquid reach its coarsest size
static float AdaptiveReach( const b2fFluid* f, int liquid, float g, float h )
{
	const b2LiquidDef* def = &f->liquids[liquid].def;
	float coarse = def->tuning.adaptiveRatio * f->spacing;
	float reach = g > 0.0f && h > 0.0f ? 0.5f * B2F_DEPTH_LIMIT_SCALE * def->tuning.stiffness * coarse * coarse / ( g * h * h ) : FLT_MAX;
	return f->def.adaptiveDepth > 0.0f && f->def.adaptiveDepth < reach ? f->def.adaptiveDepth : reach;
}

// Particles are classed by mass relative to the target for their depth, as in Winchenbach et al. 2017.
enum
{
	b2f_tooSmall,	 // gives all its mass away and is removed
	b2f_small,		 // can receive mass
	b2f_good,
	b2f_large,		 // gives its excess to small neighbours
	b2f_tooLarge	 // splits
};

#define B2F_MAX_PARTNERS 6

// Seconds for a particle's remembered speed to halve
#define B2F_SPEED_MEMORY 0.5f
#define B2F_MAX_CHILDREN 7

// A merging particle is removed once it is down to this fraction of the finest mass, or one step's exchange
#define B2F_REMOVE_FLOOR 0.25f

// Boxes asked per cell when measuring depth by fill, each this much smaller than the last
#define B2F_BOX_LEVELS 8
#define B2F_BOX_STEP 1.5f

// Box grid cells, in units of the finest spacing
#define B2F_BOX_CELL 2.0f

// Half width of the box flow and squeeze are averaged over, as a fraction of the largest particle
#define B2F_FLOW_BOX 0.87f

// A box this full sits on a flat surface, so depth below the smallest box runs to zero at this fill
#define B2F_SURFACE_FILL 0.5

// Fill a cell on an interface between liquids takes from a box, over the cells along the box's width. At 2 a band
// of such cells across a box leaves it well short of full, so an interface grades sizes as air does. At 1, the
// layers of the Three Liquids scene were still settling after 40 seconds at ratio 10; at 2 they were flat by 20.
#define B2F_INTERFACE_FILL 2.0

// How full liquid at rest packs a lattice at particleSpacing, so that a box counts as full against what the liquid
// holds when still rather than against the spawn lattice. Liquid rests where near pressure's push balances two pulls,
// cohesion's and pressure's below rest density, and its packing follows their sum over the push. In weightless drops
// of cohesion 0.025 to 0.3, tensilePressure 0 to 1, and nearStiffness and stiffness halved and doubled, the 43 that
// held together lie on this line to within 0.05, and 0.085 at worst: 0.94 for the default liquid, 1.04 for oil and
// mercury, 1.21 for honey and 1.29 for slime. It stops rising near 1.3. Drops with a sum below about 0.9 fly apart,
// and pools of such liquid rest at about 0.9, which the floor follows. Weight packs liquid tighter still, so still
// liquid at any depth reads at least this. A fit to the solver's pair rule: a change to the kernel or the terms means
// measuring it again, as the fill within 0.6 of the radius of a weightless drop 3 m across, of uniform particles at
// the default spacing, after 15 s.
#define B2F_PACKING_BASE 0.782f
#define B2F_PACKING_SLOPE 0.17f
#define B2F_PACKING_MIN 0.87f
#define B2F_PACKING_MAX 1.3f
// Pressure's pull, for stiffness times tensilePressure, as cohesion
#define B2F_PACKING_TENSILE 2.5f
// How full the spawn lattice packs, at B2F_SPAWN_SPACING of the solver's spacings: ( B2F_REST_PITCH / 1.09 )^2
#define B2F_SPAWN_PACKING 1.0652f

static float RestPacking( const b2LiquidTuning* t, float cohesion )
{
	if ( t->nearStiffness <= 0.0f )
	{
		return B2F_PACKING_MAX;
	}
	float pull = ( cohesion + B2F_PACKING_TENSILE * t->stiffness * t->tensilePressure ) / t->nearStiffness;
	float packing = B2F_PACKING_BASE + B2F_PACKING_SLOPE * pull;
	return packing < B2F_PACKING_MIN ? B2F_PACKING_MIN : ( packing > B2F_PACKING_MAX ? B2F_PACKING_MAX : packing );
}

// Mass found in a box, compared with a full one. A box that is all liquid says the particle is at least that deep.
// Going from large to small, the first full box gives the depth. Anything that is not liquid leaves a box short,
// whether air, a wall or a body, so liquid stays fine around all of them. Summed tables make every box four
// lookups, and asking per cell rather than per particle keeps the cost independent of the particle count.
static double BoxSum( const double* table, int stride, int x0, int y0, int x1, int y1 )
{
	return table[y1 * stride + x1] - table[y0 * stride + x1] - table[y1 * stride + x0] + table[y0 * stride + x0];
}

// Which particles a measurement takes: all of them, those near the bulk the grid covers, or the rest
enum
{
	b2f_allParticles,
	b2f_nearBulk,
	b2f_awayFromBulk
};

static inline bool Takes( const b2fFluid* f, int i, int part, const b2fBox* near )
{
	if ( part == b2f_allParticles )
	{
		return true;
	}
	b2Pos p = f->pos[i];
	bool inside = near->minX <= p.x && p.x <= near->maxX && near->minY <= p.y && p.y <= near->maxY;
	return inside == ( part == b2f_nearBulk );
}

// The cell a particle reads depth from, with the three after it along x and y, and how far it is across them. Clamped
// to the outer cells' centres before the weights are taken, so a particle in the outer half cell reads the edge cell
// alone.
static inline void ReadCell( b2Pos p, b2fCoord originX, b2fCoord originY, float cell, int width, int height, int* cx, int* cy, float* fx,
							 float* fy )
{
	float u = (float)( p.x - originX ) / cell - 0.5f, v = (float)( p.y - originY ) / cell - 0.5f;
	u = u < 0.0f ? 0.0f : ( u > (float)( width - 1 ) ? (float)( width - 1 ) : u );
	v = v < 0.0f ? 0.0f : ( v > (float)( height - 1 ) ? (float)( height - 1 ) : v );
	int x = (int)u, y = (int)v;
	x = x > width - 2 ? width - 2 : x;
	y = y > height - 2 ? height - 2 : y;
	*cx = x;
	*cy = y;
	if ( fx != NULL )
	{
		*fx = u - (float)x;
		*fy = v - (float)y;
	}
}

// Whether the size pass works out a particle's size: an awake one, or an asleep one in the shell of the last substep,
// within reach of awake liquid, or every particle while spawns settle. The other asleep particles keep their sizes:
// the liquid round them has been still since they fell asleep, apart from awake liquid out of their reach.
static inline bool Evaluated( const b2fFluid* f, int i, bool settle )
{
	return settle || i < f->activeCount || f->inShell[i] != 0;
}

// Boxes count the particles of the liquids in counted, a bit per liquid, and give depths to those in measured, of the
// particles in part. Returns false, measuring nothing, when part has no particles. Only the particles the pass sizes
// (Evaluated) need depths: the boxes cover them and the reach of the largest box round them, and the depth of a cell
// is worked out only where one of them reads it. While a spawn's sizes settle, before
// it moves, its particles stand on the spawn lattice and count as liquid at rest, so it takes the sizes it keeps once
// it has loosened or tightened to rest: counted by area, water would read over-full there and honey short.
static bool BoxMeasure( b2fFluid* f, float* depth, float* flow, float* pressureDepth, float reach, const float* perMetre, bool wantFlow,
						uint32_t counted, uint32_t measured, int part, const b2fBox* near, bool settle )
{
	b2fCoord minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f;
	b2fBox sized = { 0.0f, 0.0f, 0.0f, 0.0f };
	float largest = 0.0f;
	bool any = false, anySized = false, all = true;
	for ( int i = 0; i < f->count; ++i )
	{
		if ( Takes( f, i, part, near ) == false )
		{
			continue;
		}
		b2Pos p = f->pos[i];
		if ( any == false )
		{
			minX = maxX = p.x;
			minY = maxY = p.y;
			any = true;
		}
		minX = minX < p.x ? minX : p.x;
		maxX = maxX > p.x ? maxX : p.x;
		minY = minY < p.y ? minY : p.y;
		maxY = maxY > p.y ? maxY : p.y;
		largest = largest > f->size[i] ? largest : f->size[i];
		if ( Evaluated( f, i, settle ) == false )
		{
			all = false;
			continue;
		}
		if ( anySized == false )
		{
			sized = ( b2fBox ){ p.x, p.y, p.x, p.y };
			anySized = true;
		}
		sized.minX = sized.minX < p.x ? sized.minX : p.x;
		sized.maxX = sized.maxX > p.x ? sized.maxX : p.x;
		sized.minY = sized.minY < p.y ? sized.minY : p.y;
		sized.maxY = sized.maxY > p.y ? sized.maxY : p.y;
	}
	if ( anySized == false )
	{
		return false;
	}

	float spacing = f->spacing, cell = B2F_BOX_CELL * spacing;
	float extentX = (float)( maxX - minX ), extentY = (float)( maxY - minY );
	float extent = extentX > extentY ? extentX : extentY;
	while ( extent / cell > 2048.0f )
	{
		cell *= 2.0f;
	}

	// With some particles not sized, the boxes cover those that are and all a box or the four cells they read can reach
	// from them, within the bounds of all
	if ( all == false )
	{
		float grow = reach + B2F_FLOW_BOX * b2fLatticeSize( largest );
		cell = B2F_BOX_CELL * spacing;
		while ( true )
		{
			float margin = grow + 3.0f * cell;
			b2fCoord x0 = sized.minX - margin, y0 = sized.minY - margin, x1 = sized.maxX + margin, y1 = sized.maxY + margin;
			x0 = x0 > minX ? x0 : minX;
			y0 = y0 > minY ? y0 : minY;
			x1 = x1 < maxX ? x1 : maxX;
			y1 = y1 < maxY ? y1 : maxY;
			extentX = (float)( x1 - x0 );
			extentY = (float)( y1 - y0 );
			extent = extentX > extentY ? extentX : extentY;
			if ( extent / cell <= 2048.0f )
			{
				minX = x0;
				minY = y0;
				maxX = x1;
				maxY = y1;
				break;
			}
			cell *= 2.0f;
		}
	}

	// At least two cells each way, since each particle reads the four cells around it: liquid in one row, such as an
	// emitter's first row in a fluid with nothing else, would otherwise read before the tables
	int width = (int)( extentX / cell ) + 1, height = (int)( extentY / cell ) + 1;
	width = width > 2 ? width : 2;
	height = height > 2 ? height : 2;
	b2fCoord originX = minX, originY = minY;
	int stride = width + 1, nodes = stride * ( height + 1 );
	if ( nodes > f->boxCapacity )
	{
		int old = f->boxCapacity, capacity = 2 * nodes;
		f->boxMass = b2fGrowArray( f->boxMass, old, capacity, sizeof( double ) );
		f->boxPx = b2fGrowArray( f->boxPx, old, capacity, sizeof( double ) );
		f->boxPy = b2fGrowArray( f->boxPy, old, capacity, sizeof( double ) );
		f->boxWeight = b2fGrowArray( f->boxWeight, old, capacity, sizeof( double ) );
		f->boxMixed = b2fGrowArray( f->boxMixed, old, capacity, sizeof( double ) );
		f->boxLiquid = b2fGrowArray( f->boxLiquid, old, capacity, 1 );
		f->boxPressure = b2fGrowArray( f->boxPressure, old, capacity, sizeof( float ) );
		f->boxDepth = b2fGrowArray( f->boxDepth, old, capacity, sizeof( float ) );
		f->boxFlow = b2fGrowArray( f->boxFlow, old, capacity, sizeof( float ) );
		f->boxNeeded = b2fGrowArray( f->boxNeeded, old, capacity, 1 );
		f->boxCapacity = capacity;
	}

	memset( f->boxMass, 0, (size_t)nodes * sizeof( double ) );
	memset( f->boxPx, 0, (size_t)nodes * sizeof( double ) );
	memset( f->boxPy, 0, (size_t)nodes * sizeof( double ) );
	memset( f->boxWeight, 0, (size_t)nodes * sizeof( double ) );

	// Mass here is the area a particle fills at rest, whatever its liquid's density, over how full that liquid packs
	// when still, so each liquid fills a box to 1 at rest
	float fineSize = f->def.particleSpacing, inverse[B2F_MAX_LIQUIDS];
	float spawnInverse = 1.0f / ( fineSize * fineSize * B2F_SPAWN_PACKING );
	for ( int l = 0; l < f->liquidCount; ++l )
	{
		const b2LiquidDef* def = &f->liquids[l].def;
		inverse[l] = 1.0f / ( fineSize * fineSize * RestPacking( &def->tuning, def->cohesion ) );
	}
	for ( int i = 0; i < f->count; ++i )
	{
		if ( ( counted >> f->liquid[i] & 1u ) == 0 || Takes( f, i, part, near ) == false )
		{
			continue;
		}
		b2Pos p = f->pos[i];
		if ( all == false && ( p.x < minX || p.x > maxX || p.y < minY || p.y > maxY ) )
		{
			continue;
		}
		int cx = (int)( (float)( p.x - originX ) / cell ), cy = (int)( (float)( p.y - originY ) / cell );
		int c = ( cy + 1 ) * stride + cx + 1;
		int l = f->liquid[i];
		bool spawned = settle && ( f->coarsenLiquids >> l & 1u ) != 0 && f->id[i] >= f->coarsenFrom[l];
		double m = f->fineArea * f->size[i] * f->size[i] * ( spawned ? spawnInverse : inverse[l] );
		f->boxMass[c] += m;
		f->boxPx[c] += m * f->vx[i];
		f->boxPy[c] += m * f->vy[i];
		// Weightless liquid is squeezed by nothing, so squeeze says nothing about its depth
		float slope = perMetre[l];
		float squeezeDepth = slope > 0.0f ? ( f->rho[i] / f->restDensity - 1.0f - B2F_SQUEEZE_SURFACE ) * ( f->size[i] / fineSize ) / slope : 0.0f;
		f->boxWeight[c] += m * ( squeezeDepth > 0.0f ? squeezeDepth : 0.0f );
	}

	// With several liquids, a cell holding more than one is on an interface. Interfaces count as surface, so particles
	// stay fine along them as they do at air.
	bool multi = f->liquidCount > 1;
	if ( multi )
	{
		memset( f->boxLiquid, 0xFF, (size_t)nodes );
		memset( f->boxMixed, 0, (size_t)nodes * sizeof( double ) );
		for ( int i = 0; i < f->count; ++i )
		{
			if ( ( counted >> f->liquid[i] & 1u ) == 0 || Takes( f, i, part, near ) == false )
			{
				continue;
			}
			b2Pos p = f->pos[i];
			if ( all == false && ( p.x < minX || p.x > maxX || p.y < minY || p.y > maxY ) )
			{
				continue;
			}
			int cx = (int)( (float)( p.x - originX ) / cell ), cy = (int)( (float)( p.y - originY ) / cell );
			int c = ( cy + 1 ) * stride + cx + 1;
			unsigned char seen = f->boxLiquid[c];
			f->boxLiquid[c] = seen == 0xFF || seen == f->liquid[i] ? f->liquid[i] : 0xFE;
		}
		for ( int c = 0; c < nodes; ++c )
		{
			f->boxMixed[c] = f->boxLiquid[c] == 0xFE ? 1.0 : 0.0;
		}
	}

	double* tables[5] = { f->boxMass, f->boxPx, f->boxPy, f->boxWeight, f->boxMixed };
	for ( int t = 0; t < ( multi ? 5 : 4 ); ++t )
	{
		double* table = tables[t];
		for ( int y = 1; y <= height; ++y )
		{
			double row = 0.0;
			for ( int x = 1; x <= width; ++x )
			{
				row += table[y * stride + x];
				table[y * stride + x] = row + table[( y - 1 ) * stride + x];
			}
		}
	}

	// Half widths in cells, large to small
	int sizes[B2F_BOX_LEVELS], levels = 0;
	for ( float k = reach / cell; k >= 1.0f && levels < B2F_BOX_LEVELS; k /= B2F_BOX_STEP )
	{
		sizes[levels++] = (int)k;
	}
	int flowSize = (int)( B2F_FLOW_BOX * b2fLatticeSize( largest ) / cell + 0.5f );
	flowSize = flowSize < 1 ? 1 : flowSize;

	// The cells the sized particles read, below
	memset( f->boxNeeded, 0, (size_t)( width * height ) );
	for ( int i = 0; i < f->count; ++i )
	{
		if ( ( measured >> f->liquid[i] & 1u ) == 0 || Takes( f, i, part, near ) == false || Evaluated( f, i, settle ) == false )
		{
			continue;
		}
		int cx, cy;
		ReadCell( f->pos[i], originX, originY, cell, width, height, &cx, &cy, NULL, NULL );
		int c = cy * width + cx;
		f->boxNeeded[c] = f->boxNeeded[c + 1] = f->boxNeeded[c + width] = f->boxNeeded[c + width + 1] = 1;
	}

	// A cell's mass when full: a lattice at particleSpacing, which mass over packing makes liquid at rest
	double fullMass = f->fineArea / ( 0.8660254 * fineSize * fineSize ) * cell * cell;
	double full = f->def.adaptiveFullBox;
	for ( int y = 0; y < height; ++y )
	{
		for ( int x = 0; x < width; ++x )
		{
			int c = y * width + x;
			f->boxDepth[c] = 0.0f;
			f->boxFlow[c] = 0.0f;
			f->boxPressure[c] = 0.0f;
			if ( f->boxNeeded[c] == 0 )
			{
				continue;
			}

			// Particles read the four nearest cells, so nothing further from liquid than that is ever asked
			int ax = x > 0 ? x - 1 : 0, ay = y > 0 ? y - 1 : 0, bx = x + 2 < width ? x + 2 : width, by = y + 2 < height ? y + 2 : height;
			if ( BoxSum( f->boxMass, stride, ax, ay, bx, by ) <= 0.0 )
			{
				continue;
			}

			// Depth is where the fill crosses full, found between the first full box and the one before it, so it
			// moves smoothly instead of stepping from one box size to the next. Below the smallest box it runs to
			// zero at half full, which is a box sitting on the surface.
			float best = 0.0f, lastHalf = 0.0f;
			double lastFill = 0.0;
			bool found = false;
			for ( int l = 0; l < levels; ++l )
			{
				int k = sizes[l];
				float half = ( (float)k + 0.5f ) * cell;
				int x0 = x > k ? x - k : 0, y0 = y > k ? y - k : 0;
				int x1 = x + k + 1 < width ? x + k + 1 : width, y1 = y + k + 1 < height ? y + k + 1 : height;
				double area = (double)( ( 2 * k + 1 ) * ( 2 * k + 1 ) );
				double fill = BoxSum( f->boxMass, stride, x0, y0, x1, y1 ) / ( area * fullMass );

				// Boxes do not reach across an interface, since coarse particles along one hold its shape: cohesion
				// moves particles in proportion to their size, gravity does not
				if ( multi )
				{
					fill -= BoxSum( f->boxMixed, stride, x0, y0, x1, y1 ) * B2F_INTERFACE_FILL / (double)( 2 * k + 1 );
				}
				if ( fill >= full )
				{
					best = l == 0 ? half : half + ( lastHalf - half ) * (float)( ( fill - full ) / ( fill - lastFill ) );
					found = true;
					break;
				}
				lastHalf = half;
				lastFill = fill;
			}
			if ( found == false && levels > 0 )
			{
				double room = full - B2F_SURFACE_FILL;
				double t = ( lastFill - B2F_SURFACE_FILL ) / ( room > 0.01 ? room : 0.01 );
				t = t < 0.0 ? 0.0 : ( t > 1.0 ? 1.0 : t );
				best = lastHalf * (float)t;
			}
			f->boxDepth[c] = best;

			// One particle's squeeze is noisy, so depth from it is a mass weighted mean over the same box as flow
			{
				int k = flowSize;
				int x0 = x > k ? x - k : 0, y0 = y > k ? y - k : 0;
				int x1 = x + k + 1 < width ? x + k + 1 : width, y1 = y + k + 1 < height ? y + k + 1 : height;
				double mass = BoxSum( f->boxMass, stride, x0, y0, x1, y1 );
				if ( mass > 0.0 )
				{
					f->boxPressure[c] = (float)( BoxSum( f->boxWeight, stride, x0, y0, x1, y1 ) / mass );
				}
			}

			if ( wantFlow )
			{
				int k = flowSize;
				int x0 = x > k ? x - k : 0, y0 = y > k ? y - k : 0;
				int x1 = x + k + 1 < width ? x + k + 1 : width, y1 = y + k + 1 < height ? y + k + 1 : height;
				double mass = BoxSum( f->boxMass, stride, x0, y0, x1, y1 );
				if ( mass > 0.0 )
				{
					double px = BoxSum( f->boxPx, stride, x0, y0, x1, y1 ), py = BoxSum( f->boxPy, stride, x0, y0, x1, y1 );
					f->boxFlow[c] = (float)( sqrt( px * px + py * py ) / mass );
				}
			}
		}
	}

	// Blended between cells, so a target does not step as a particle crosses one
	for ( int i = 0; i < f->count; ++i )
	{
		if ( ( measured >> f->liquid[i] & 1u ) == 0 || Takes( f, i, part, near ) == false || Evaluated( f, i, settle ) == false )
		{
			continue;
		}
		int cx, cy;
		float fx, fy;
		ReadCell( f->pos[i], originX, originY, cell, width, height, &cx, &cy, &fx, &fy );
		int c = cy * width + cx;
		float a = ( 1.0f - fx ) * ( 1.0f - fy ), b = fx * ( 1.0f - fy ), e = ( 1.0f - fx ) * fy, d = fx * fy;
		depth[i] = a * f->boxDepth[c] + b * f->boxDepth[c + 1] + e * f->boxDepth[c + width] + d * f->boxDepth[c + width + 1];
		flow[i] = a * f->boxFlow[c] + b * f->boxFlow[c + 1] + e * f->boxFlow[c + width] + d * f->boxFlow[c + width + 1];
		pressureDepth[i] = a * f->boxPressure[c] + b * f->boxPressure[c + 1] + e * f->boxPressure[c + width] + d * f->boxPressure[c + width + 1];
	}
	return true;
}

// Depth for the particles of the liquids in measured, counting those in counted. Where the grid covers only the bulk of
// the liquid, liquid far from it would make the cells coarse for every particle, so the bulk, and particles within the
// grid's border of it, are measured apart from the rest, each in cells over their own bounds. Particles away from the
// bulk count only each other, so a stray drop reads as a drop, and a pool far away as a pool, with cells as coarse as
// the spread of everything outside the bulk takes.
static void MeasureDepth( b2fFluid* f, float reach, const float* perMetre, bool wantFlow, uint32_t counted, uint32_t measured, bool settle )
{
	if ( f->activeCount == 0 || f->grid.bulkOnly == false )
	{
		BoxMeasure( f, f->adaptDepth, f->adaptFlow, f->adaptPressure, reach, perMetre, wantFlow, counted, measured, b2f_allParticles, NULL, settle );
		return;
	}

	float margin = (float)f->grid.border * f->grid.cell;
	b2fBox near = { f->grid.bulk.minX - margin, f->grid.bulk.minY - margin, f->grid.bulk.maxX + margin, f->grid.bulk.maxY + margin };
	BoxMeasure( f, f->adaptDepth, f->adaptFlow, f->adaptPressure, reach, perMetre, wantFlow, counted, measured, b2f_nearBulk, &near, settle );
	BoxMeasure( f, f->adaptDepth, f->adaptFlow, f->adaptPressure, reach, perMetre, wantFlow, counted, measured, b2f_awayFromBulk, &near, settle );
}

// What the size pass reads per liquid
typedef struct b2fLiquidSizes
{
	float baseMass, ratio, reach, fastSpeed, tolerance, limit, rate, safeScale;
	bool depthLimit;
} b2fLiquidSizes;

// Runs at the end of a step, or before one when settling, on one thread, in particle order, so that the result does not
// depend on worker count. It reads the grid of the sort just before it. Sizes change by as much as elapsed seconds
// allow. Settling, they change as far as one pass can, and only for the particles waiting since their spawn; the rest
// stand still and take no part.
static void ChangeSizes( b2fFluid* f, float elapsed, bool settle )
{
	f->sizeChanges = 0;
	int n = f->count;

	// Mass linear in depth, as in Winchenbach et al. 2017, reaching the coarsest size at half the depth that size can
	// hold up by pressure alone. That grows sizes fastest right at a surface, twice the finest within 0.4 m at the
	// default settings, which keeps small, soft particles to a thin layer along a deep floor. Next to a moving body it
	// cost lift, a quarter of it for a crate near the floor, so there size grows in a line with distance instead.
	// Masses are the areas particles fill at rest, whatever the liquid.
	float spacing = f->spacing, fineSize = f->def.particleSpacing, fineArea = f->fineArea;
	b2fLiquidSizes sizes[B2F_MAX_LIQUIDS];
	float perMetre[B2F_MAX_LIQUIDS];
	float deepest = 0.0f;
	bool wantFlow = false;
	for ( int l = 0; l < f->liquidCount; ++l )
	{
		const b2LiquidDef* def = &f->liquids[l].def;
		float ratio = def->tuning.adaptiveRatio > 1.0f ? def->tuning.adaptiveRatio : 1.0f;
		// Depth is measured the same way in every direction, by boxes and by squeeze, so only how strong gravity is
		// matters, not which way it points. Without gravity nothing has weight, so no size is unsafe: the reach is
		// adaptiveDepth alone, and the depth limit does nothing.
		float g = b2AbsFloat( def->gravityScale ) * b2Length( f->worldGravity );
		float reach = AdaptiveReach( f, l, g, f->h );
		sizes[l] = ( b2fLiquidSizes ){
			.baseMass = fineArea * ratio * ratio,
			.ratio = ratio,
			.reach = reach,
			.fastSpeed = def->tuning.adaptiveSpeed,
			.tolerance = def->tuning.adaptiveTolerance,
			.limit = def->tuning.adaptiveLimit,
			.rate = def->tuning.adaptiveRate,
			.safeScale = fineArea * 2.0f * g * f->h * f->h / ( B2F_DEPTH_LIMIT_SCALE * def->tuning.stiffness * spacing * spacing ),
			.depthLimit = def->tuning.adaptiveDepthLimit,
		};
		perMetre[l] = B2F_SQUEEZE_SLOPE * g * f->h * f->h / ( def->tuning.stiffness * spacing * spacing );
		// A liquid with no reach, weightless and with no adaptiveDepth, stays fine and needs no boxes
		if ( ratio > 1.0f && reach < FLT_MAX )
		{
			deepest = deepest > reach ? deepest : reach;
			wantFlow = wantFlow || def->tuning.adaptiveSpeed > 0.0f;
		}
	}

	// Depth goes by the liquid a particle's liquid pushes on: liquid it passes through neither holds it up nor makes an
	// interface with it, so with liquids that pass through each other the boxes are measured once for each set of
	// liquids that push on the same liquids, counting those alone
	uint64_t ticks = b2GetTicks();
	if ( f->liquidsFiltered == false )
	{
		MeasureDepth( f, deepest, perMetre, wantFlow, UINT32_MAX, UINT32_MAX, settle );
	}
	else
	{
		uint32_t done = 0;
		for ( int l = 0; l < f->liquidCount; ++l )
		{
			if ( f->liquids[l].alive == false || ( done >> l & 1u ) != 0 )
			{
				continue;
			}
			uint32_t measured = 0;
			for ( int k = l; k < f->liquidCount; ++k )
			{
				measured |= f->liquids[k].alive && f->liquidSees[k] == f->liquidSees[l] ? 1u << k : 0u;
			}
			MeasureDepth( f, deepest, perMetre, wantFlow, f->liquidSees[l], measured, settle );
			done |= measured;
		}
	}
	f->measureTime = b2GetMilliseconds( ticks );

	float memoryDecay = powf( 0.5f, elapsed / B2F_SPEED_MEMORY );

	for ( int i = 0; i < n; ++i )
	{
		const b2fLiquidSizes* liquid = sizes + f->liquid[i];
		bool stays = settle ? ( f->coarsenLiquids >> f->liquid[i] & 1u ) == 0 || f->id[i] < f->coarsenFrom[f->liquid[i]] : Evaluated( f, i, false ) == false;
		if ( stays )
		{
			float s = f->size[i] / fineSize;
			f->adaptMass[i] = f->adaptTarget[i] = fineArea * s * s;
			f->adaptUsed[i] = 0;
			f->adaptClass[i] = b2f_good;
			// Asleep out of reach of awake liquid, the flow round it is none, so its memory of speed fades
			if ( settle == false && liquid->fastSpeed > 0.0f )
			{
				f->speedMemory[i] *= memoryDecay;
			}
			continue;
		}

		float baseMass = liquid->baseMass, fastSpeed = liquid->fastSpeed, limit = liquid->limit, tolerance = liquid->tolerance;
		float t = f->adaptDepth[i] / liquid->reach;
		t = t < 1.0f ? t : 1.0f;
		float target = fineArea + ( baseMass - fineArea ) * t;

		// Around a moving body size grows in a line with distance from it instead, at the rate that reaches the
		// coarsest size at the reach. A body within this distance of the particle makes it finer than its depth asks.
		float slope = ( liquid->ratio - 1.0f ) / liquid->reach;
		float near = slope > 0.0f ? ( sqrtf( target / fineArea ) - 1.0f ) / slope : 0.0f;
		b2Pos p = f->pos[i];
		bool filtered = f->shapesFiltered;
		for ( int si = 0; si < f->shapeCount; ++si )
		{
			const b2fShape* shape = f->shapes + si;
			const b2fBody* body = f->bodies + shape->bodyIndex;
			// A body the liquid passes through is not there for it
			if ( ( filtered && ( shape->liquids >> f->liquid[i] & 1u ) == 0 ) || body->adheres || p.x < shape->minX - near || p.x > shape->maxX + near || p.y < shape->minY - near || p.y > shape->maxY + near )
			{
				continue;
			}

			float distance;
			b2Vec2 normal;
			if ( b2fShapeDistance( shape, b2InvRotateVector( body->motion.rotation, b2SubPos( p, body->origin ) ), &distance, &normal ) && distance < near )
			{
				near = distance > 0.0f ? distance : 0.0f;
				float size = 1.0f + slope * near;
				target = fineArea * size * size;
			}
		}

		// Remembering recent speed means a particle near the threshold does not split and merge over and over
		if ( fastSpeed > 0.0f )
		{
			float speed = f->adaptFlow[i];
			float memory = f->speedMemory[i] * memoryDecay;
			memory = speed > memory ? speed : memory;
			f->speedMemory[i] = memory;
			float k = 2.0f * memory / fastSpeed - 1.0f;
			k = k < 0.0f ? 0.0f : ( k > 1.0f ? 1.0f : k );
			target += ( fineArea - target ) * k;
		}

		if ( liquid->depthLimit )
		{
			// Boxes read walls and floors as air, so the limit goes by squeeze, which feels the weight above
			float safe = liquid->safeScale * f->adaptPressure[i];
			safe = safe < baseMass ? safe : baseMass;
			target = target > safe ? target : safe;
		}

		float s = f->size[i] / fineSize;
		float mass = fineArea * s * s;
		float relative = mass / target;
		f->adaptMass[i] = mass;
		f->adaptTarget[i] = target;
		f->adaptUsed[i] = 0;
		// Settling, nothing moves, so a small particle would never be given mass and the liquid would stay finer than
		// its depth asks, with half as many particles again as when it is graded as it spawns. So it merges away too,
		// into neighbours that are small as well.
		bool small = relative <= 1.0f - tolerance;
		f->adaptClass[i] = relative < 1.0f / limit || ( settle && small ) ? b2f_tooSmall : small ? b2f_small : relative < 1.0f + tolerance ? b2f_good : relative <= limit ? b2f_large : b2f_tooLarge;
	}

	// Removing one particle at a time gives n+1 to n merges, which can always find partners
	for ( int i = 0; i < n; ++i )
	{
		int cls = f->adaptClass[i];

		b2fCoord x = f->pos[i].x, y = f->pos[i].y;
		float reachI = 2.0f * b2fLatticeSize( f->size[i] ), reach2 = reachI * reachI;

		// Splits happen after this pass, but count as changes here
		if ( cls == b2f_tooLarge && f->adaptUsed[i] == 0 )
		{
			f->sizeChanges += 1;
			continue;
		}

		if ( ( cls != b2f_tooSmall && cls != b2f_large ) || f->adaptUsed[i] )
		{
			continue;
		}

		// A particle moves at most this much mass a step, its rate per second over the step's time, so its size and
		// its neighbours' change a little at a time. One that is merging away is only removed once what is left is
		// that small, or a quarter of the finest, so it never shrinks to a speck with too short a reach to find
		// neighbours.
		const b2fLiquidSizes* liquid = sizes + f->liquid[i];
		float cap = ( settle ? 1.0f : liquid->rate * elapsed ) * f->adaptTarget[i], baseMass = liquid->baseMass;
		float last = cap > B2F_REMOVE_FLOOR * fineArea ? cap : B2F_REMOVE_FLOOR * fineArea;
		bool removing = cls == b2f_tooSmall && f->adaptMass[i] <= last;
		float excess = cls == b2f_tooSmall ? f->adaptMass[i] : f->adaptMass[i] - f->adaptTarget[i];
		float give = removing ? excess : ( excess < cap ? excess : cap );
		if ( cls == b2f_tooSmall && removing == false && f->adaptMass[i] - give < last )
		{
			give = f->adaptMass[i] - last;
		}
		// Particles have moved since the grid was sorted, by under one cell in the last substep, so search one more
		int partners[B2F_MAX_PARTNERS], count = 0;
		FOR_NEIGHBOURS_WITHIN( f, i, j, f->window[i] + 1, f->sleepWindow[i] < 0 ? -1 : f->sleepWindow[i] + 1 )
		{
			if ( j == i || f->adaptUsed[j] || count == B2F_MAX_PARTNERS || f->adaptClass[j] > b2f_small || f->liquid[j] != f->liquid[i] )
			{
				continue;
			}

			// A particle merging away goes into neighbours at least as heavy, not into a lighter one merging away
			// too: two such with no other neighbour to take mass, among liquid at its size, otherwise passed the same
			// mass back and forth every step, each jumping a few centimetres and back, for as long as it lasted
			if ( cls == b2f_tooSmall && f->adaptClass[j] == b2f_tooSmall &&
				 ( f->adaptMass[j] < f->adaptMass[i] || ( f->adaptMass[j] == f->adaptMass[i] && f->id[j] < f->id[i] ) ) )
			{
				continue;
			}

			float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y );
			if ( dx * dx + dy * dy < reach2 && f->adaptMass[j] + give / (float)( count + 1 ) < baseMass )
			{
				partners[count++] = j;
			}
		}

		if ( count == 0 )
		{
			continue;
		}

		float share = give / (float)count;
		for ( int k = 0; k < count; ++k )
		{
			int j = partners[k];
			float m = f->adaptMass[j], total = m + share;
			f->pos[j].x = ( f->pos[j].x * m + x * share ) / total;
			f->pos[j].y = ( f->pos[j].y * m + y * share ) / total;
			f->vx[j] = ( f->vx[j] * m + f->vx[i] * share ) / total;
			f->vy[j] = ( f->vy[j] * m + f->vy[i] * share ) / total;
			f->adaptMass[j] = total;
			f->adaptUsed[j] = 1;
		}

		f->sizeChanges += 1;

		f->adaptUsed[i] = 1;
		f->adaptMass[i] -= give;
		if ( removing )
		{
			f->adaptMass[i] = 0.0f;
		}
	}

	// Particles take the sizes of their new masses, and those with none left are removed; the classes and targets go
	// with the rest for the splits below. Asleep particles of the shell, within reach of awake ones, are sized as the
	// rest are, so liquid that has settled has none to change, and one whose size changes wakes, with what it splits
	// into. Keeping them out of the pass instead left an awake particle beside sleeping ones with only one neighbour to
	// trade mass with, and the two passed it back and forth every step, each moving 4 cm and back at 1.5 m/s.
	for ( int i = 0; i < n; ++i )
	{
		f->marked[i] = f->adaptMass[i] <= 0.0f ? 1 : 0;
		if ( f->marked[i] == 0 )
		{
			f->size[i] = fineSize * sqrtf( f->adaptMass[i] / fineArea );
		}
		if ( f->asleep[i] != 0 && f->adaptUsed[i] )
		{
			b2fWakeParticle( f, i );
		}
	}
	int kept = n - b2fRemoveMarked( f );

	// Children sit on a ring, with one in the middle past four, and copy the parent's velocity
	for ( int i = 0; i < kept; ++i )
	{
		if ( f->adaptClass[i] != b2f_tooLarge )
		{
			continue;
		}

		float s = f->size[i] / fineSize;
		float mass = fineArea * s * s;
		int children = (int)ceilf( mass / f->adaptTarget[i] );
		children = children < B2F_MAX_CHILDREN ? children : B2F_MAX_CHILDREN;
		// No substep is running, so the fluid can grow here; it stays whole only at maxCapacity
		if ( children < 2 || b2fGrow( f, f->count + children - 1 ) == false )
		{
			continue;
		}

		// The children's size, and the same on the solver's lattice, which the distances here go by
		float child = f->size[i] / sqrtf( (float)children ), latticeChild = b2fLatticeSize( child );
		int ring = children > 4 ? children - 1 : children;
		float radius = latticeChild * B2F_SPAWN_SPACING * 0.5f / sinf( 3.14159265f / (float)ring );
		float phase = 2.3999632f * (float)f->id[i];
		b2fCoord x = f->pos[i].x, y = f->pos[i].y;

		// A child inside a shape reads as a deep collision and would kick the body, so stay whole until clear
		bool clear = true;
		for ( int si = 0; si < f->shapeCount && clear; ++si )
		{
			const b2fShape* shape = f->shapes + si;
			float reachOut = radius + latticeChild;
			if ( ( f->shapesFiltered && ( shape->liquids >> f->liquid[i] & 1u ) == 0 ) || x + reachOut < shape->minX || x - reachOut > shape->maxX || y + reachOut < shape->minY || y - reachOut > shape->maxY )
			{
				continue;
			}

			const b2fBody* body = f->bodies + shape->bodyIndex;
			for ( int c = 0; c < ring && clear; ++c )
			{
				float angle = phase + 6.2831853f * (float)c / (float)ring;
				b2Pos point = { x + radius * cosf( angle ), y + radius * sinf( angle ) };
				b2Vec2 local = b2InvRotateVector( body->motion.rotation, b2SubPos( point, body->origin ) );
				float distance;
				b2Vec2 normal;
				if ( b2fShapeDistance( shape, local, &distance, &normal ) && distance < 0.5f * latticeChild )
				{
					clear = false;
				}
			}
		}

		if ( clear == false )
		{
			continue;
		}
		b2fWakeParticle( f, i );

		for ( int c = 0; c < children; ++c )
		{
			int index = c == 0 ? i : f->count++;
			f->gridStale = true;
			if ( index != i )
			{
				b2fCopyParticle( f, index, i );
				f->id[index] = f->nextParticleId++;
				// A new particle holds no springs. Its parent's partners hold theirs to the parent's id. It forms its own
				// in its first substep, in a liquid without formSpringsAfterSpawn too.
				if ( f->springs != NULL )
				{
					f->springs[index].count = 0;
					f->springs[index].formed = false;
				}
			}
			bool centre = children > 4 && c == children - 1;
			float angle = phase + 6.2831853f * (float)c / (float)ring;
			f->pos[index].x = centre ? x : x + radius * cosf( angle );
			f->pos[index].y = centre ? y : y + radius * sinf( angle );
			f->size[index] = child;
		}
	}
}

bool b2fAdaptSizes( b2fFluid* f )
{
	// A step of no time ran no substeps, so there is no grid of this step's particles
	// With every particle asleep no size can change
	if ( f->adaptive == false || f->stepSubSteps == 0 || f->stepTime <= 0.0f || f->h <= 0.0f || f->count == 0 || f->asleepCount == f->count )
	{
		return false;
	}

	ChangeSizes( f, f->stepTime, false );
	return true;
}

// A spawned particle that took mass from its neighbours while its size settled. The spawn's particles that did not,
// and all other liquid, stay where they are while the merged ones settle.
static bool IsMerged( const b2fFluid* f, int i )
{
	int l = f->liquid[i];
	return ( f->coarsenLiquids >> l & 1u ) != 0 && f->id[i] >= f->coarsenFrom[l] && f->size[i] > 1.0001f * f->def.particleSpacing;
}

// Settling stops once no merged particle moves more than this share of its lattice size in an iteration, or after this
// many iterations. Each costs about one substep of the solver, on one thread.
#define B2F_SETTLE_MOVE 0.02f
#define B2F_SETTLE_ITERATIONS 64

// A merged particle sits at the mean of where its mass came from, so merged liquid is unevenly spaced, and in its first
// step the pressure that evens it out threw the pool about, at 2 to 4 m/s rms. So before it moves, the solver's own
// passes run on the merged particles alone, with gravity, each from rest: every iteration predicts, sorts, finds
// contacts, relaxes density and pushes out of shapes, and then velocities go back to zero. The spawn's particles that
// did not merge, which line its outline, and all other liquid, are put back after each iteration, and bodies stand
// where they are and feel nothing. Velocities are then what they were before, so only positions change.
static void SettleMerged( b2fFluid* f )
{
	int n = f->count;
	bool any = false;
	for ( int i = 0; i < n && any == false; ++i )
	{
		any = IsMerged( f, i );
	}
	if ( any == false )
	{
		return;
	}

	// Filed first, so the sorts below move only the awake particles
	b2fSortParticles( f, NULL );

	// Sorts move particles, so each keeps where it started, to take back its velocity from there
	size_t floats = (size_t)n * sizeof( float ), ints = (size_t)n * sizeof( int );
	float* startVx = b2Alloc( floats );
	float* startVy = b2Alloc( floats );
	int* from = b2Alloc( ints );
	int* scratch = b2Alloc( ints );
	memcpy( startVx, f->vx, floats );
	memcpy( startVy, f->vy, floats );
	for ( int i = 0; i < n; ++i )
	{
		from[i] = i;
		f->vx[i] = 0.0f;
		f->vy[i] = 0.0f;
	}

	// Springs would form between particles that are still moving into place
	bool springy = f->springy;
	f->springy = false;
	b2fGatherShapes( f );

	for ( int iteration = 0; iteration < B2F_SETTLE_ITERATIONS; ++iteration )
	{
		b2fPredict( f, NULL );
		b2fSortAwake( f, NULL );
		for ( int i = 0; i < n; ++i )
		{
			scratch[i] = i < f->activeCount ? from[f->permutation[i]] : from[i];
		}
		int* swap = from;
		from = scratch;
		scratch = swap;
		b2fHoldBodies( f );
		b2fFindContacts( f, NULL, false );
		b2fRelaxDensity( f, NULL );
		b2fCollideHeld( f );

		float largest = 0.0f;
		for ( int i = 0; i < n; ++i )
		{
			f->vx[i] = 0.0f;
			f->vy[i] = 0.0f;
			if ( IsMerged( f, i ) == false )
			{
				f->pos[i] = f->prev[i];
				continue;
			}
			float dx = (float)( f->pos[i].x - f->prev[i].x ), dy = (float)( f->pos[i].y - f->prev[i].y );
			float move = sqrtf( dx * dx + dy * dy ) / b2fLatticeSize( f->size[i] );
			largest = move > largest ? move : largest;
		}
		if ( largest < B2F_SETTLE_MOVE )
		{
			break;
		}
	}

	for ( int i = 0; i < n; ++i )
	{
		f->vx[i] = startVx[from[i]];
		f->vy[i] = startVy[from[i]];
	}
	b2Free( startVx, floats );
	b2Free( startVy, floats );
	b2Free( from, ints );
	b2Free( scratch, ints );
	f->springy = springy;
}

// A pass can only merge a particle into neighbours that are not merging themselves, so coarsening fine liquid by a
// mass ratio r takes about log r / log( 7 / 6 ) passes; 100 takes 30
#define B2F_COARSEN_PASSES 64

// Particles do not move between passes, so this is the sizes the liquid would settle to if it held still, by the rules
// of the passes during steps: those within the split and merge limits of their targets stay as they are. Each pass sorts
// again, since merges and splits change the particles and their sizes.
void b2fCoarsenSpawned( b2fFluid* f, float h )
{
	if ( f->coarsenLiquids == 0 || f->adaptive == false || f->count == 0 || h <= 0.0f )
	{
		f->coarsenLiquids = 0;
		return;
	}

	f->h = h;
	for ( int pass = 0; pass < B2F_COARSEN_PASSES; ++pass )
	{
		b2fSortParticles( f, NULL );
		ChangeSizes( f, 0.0f, true );
		if ( f->sizeChanges == 0 )
		{
			break;
		}
	}

	SettleMerged( f );
	f->coarsenLiquids = 0;
}

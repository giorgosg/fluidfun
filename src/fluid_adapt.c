// SPDX-License-Identifier: MIT
// Adaptive particle sizes: density boxes measure depth, and particles split, merge and trade mass toward the size
// their depth asks for, after Winchenbach et al. 2017.
#include "fluid_internal.h"

// Depth at which particles of a liquid reach its coarsest size
float b2fAdaptiveReach( const b2fFluid* f, int liquid, float g, float h )
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

// A merging particle is removed once it is down to this fraction of the finest mass, or one frame's exchange
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

// Mass found in a box, compared with a full one. A box that is all liquid says the particle is at least that deep.
// Going from large to small, the first full box gives the depth. Anything that is not liquid leaves a box short,
// whether air, a wall or a body, so liquid stays fine around all of them. Summed tables make every box four
// lookups, and asking per cell rather than per particle keeps the cost independent of the particle count.
static double BoxSum( const double* table, int stride, int x0, int y0, int x1, int y1 )
{
	return table[y1 * stride + x1] - table[y0 * stride + x1] - table[y1 * stride + x0] + table[y0 * stride + x0];
}

// Boxes count the particles of the liquids in counted, a bit per liquid, and give depths to those in measured.
static void BoxMeasure( b2fFluid* f, float* depth, float* flow, float* pressureDepth, float reach, const float* perMetre, bool wantFlow,
						uint32_t counted, uint32_t measured )
{
	b2fCoord minX = f->pos[0].x, maxX = f->pos[0].x, minY = f->pos[0].y, maxY = f->pos[0].y;
	float largest = 0.0f;
	for ( int i = 0; i < f->count; ++i )
	{
		minX = minX < f->pos[i].x ? minX : f->pos[i].x;
		maxX = maxX > f->pos[i].x ? maxX : f->pos[i].x;
		minY = minY < f->pos[i].y ? minY : f->pos[i].y;
		maxY = maxY > f->pos[i].y ? maxY : f->pos[i].y;
		largest = largest > f->size[i] ? largest : f->size[i];
	}

	float spacing = f->spacing, cell = B2F_BOX_CELL * spacing;
	float extentX = (float)( maxX - minX ), extentY = (float)( maxY - minY );
	float extent = extentX > extentY ? extentX : extentY;
	while ( extent / cell > 2048.0f )
	{
		cell *= 2.0f;
	}

	int width = (int)( extentX / cell ) + 1, height = (int)( extentY / cell ) + 1;
	b2fCoord originX = minX, originY = minY;
	int stride = width + 1, nodes = stride * ( height + 1 );
	if ( nodes > f->boxCapacity )
	{
		f->boxCapacity = 2 * nodes;
		f->boxMass = realloc( f->boxMass, (size_t)f->boxCapacity * sizeof( double ) );
		f->boxPx = realloc( f->boxPx, (size_t)f->boxCapacity * sizeof( double ) );
		f->boxPy = realloc( f->boxPy, (size_t)f->boxCapacity * sizeof( double ) );
		f->boxWeight = realloc( f->boxWeight, (size_t)f->boxCapacity * sizeof( double ) );
		f->boxMixed = realloc( f->boxMixed, (size_t)f->boxCapacity * sizeof( double ) );
		f->boxLiquid = realloc( f->boxLiquid, (size_t)f->boxCapacity );
		f->boxPressure = realloc( f->boxPressure, (size_t)f->boxCapacity * sizeof( float ) );
		f->boxDepth = realloc( f->boxDepth, (size_t)f->boxCapacity * sizeof( float ) );
		f->boxFlow = realloc( f->boxFlow, (size_t)f->boxCapacity * sizeof( float ) );
	}

	memset( f->boxMass, 0, (size_t)nodes * sizeof( double ) );
	memset( f->boxPx, 0, (size_t)nodes * sizeof( double ) );
	memset( f->boxPy, 0, (size_t)nodes * sizeof( double ) );
	memset( f->boxWeight, 0, (size_t)nodes * sizeof( double ) );

	// Mass here is liquid 0's for every particle, which makes it a measure of area, so liquids of other densities
	// fill a box the same
	float inverse = 1.0f / ( spacing * spacing );
	for ( int i = 0; i < f->count; ++i )
	{
		if ( ( counted >> f->liquid[i] & 1u ) == 0 )
		{
			continue;
		}
		int cx = (int)( (float)( f->pos[i].x - originX ) / cell ), cy = (int)( (float)( f->pos[i].y - originY ) / cell );
		int c = ( cy + 1 ) * stride + cx + 1;
		double m = f->mass * f->size[i] * f->size[i] * inverse;
		f->boxMass[c] += m;
		f->boxPx[c] += m * f->vx[i];
		f->boxPy[c] += m * f->vy[i];
		float squeezeDepth = ( f->rho[i] / f->restDensity - 1.0f - B2F_SQUEEZE_SURFACE ) * ( f->size[i] / spacing ) / perMetre[f->liquid[i]];
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
			if ( ( counted >> f->liquid[i] & 1u ) == 0 )
			{
				continue;
			}
			int cx = (int)( (float)( f->pos[i].x - originX ) / cell ), cy = (int)( (float)( f->pos[i].y - originY ) / cell );
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
	int flowSize = (int)( B2F_FLOW_BOX * largest / cell + 0.5f );
	flowSize = flowSize < 1 ? 1 : flowSize;

	// Liquid at rest stands at the spacing it spawns at. Boxes count mass in particles of liquid 0, as area.
	double fullMass = f->mass / ( 0.8660254 * spacing * spacing ) * cell * cell / ( B2F_SPAWN_SPACING * B2F_SPAWN_SPACING );
	double full = f->def.adaptiveFullBox;
	for ( int y = 0; y < height; ++y )
	{
		for ( int x = 0; x < width; ++x )
		{
			int c = y * width + x;
			f->boxDepth[c] = 0.0f;
			f->boxFlow[c] = 0.0f;
			f->boxPressure[c] = 0.0f;

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
		if ( ( measured >> f->liquid[i] & 1u ) == 0 )
		{
			continue;
		}
		float u = (float)( f->pos[i].x - originX ) / cell - 0.5f, v = (float)( f->pos[i].y - originY ) / cell - 0.5f;
		int cx = (int)floorf( u ), cy = (int)floorf( v );
		float fx = u - (float)cx, fy = v - (float)cy;
		cx = cx < 0 ? 0 : ( cx > width - 2 ? width - 2 : cx );
		cy = cy < 0 ? 0 : ( cy > height - 2 ? height - 2 : cy );
		int c = cy * width + cx;
		float a = ( 1.0f - fx ) * ( 1.0f - fy ), b = fx * ( 1.0f - fy ), e = ( 1.0f - fx ) * fy, d = fx * fy;
		depth[i] = a * f->boxDepth[c] + b * f->boxDepth[c + 1] + e * f->boxDepth[c + width] + d * f->boxDepth[c + width + 1];
		flow[i] = a * f->boxFlow[c] + b * f->boxFlow[c + 1] + e * f->boxFlow[c + width] + d * f->boxFlow[c + width + 1];
		pressureDepth[i] = a * f->boxPressure[c] + b * f->boxPressure[c + 1] + e * f->boxPressure[c + width] + d * f->boxPressure[c + width + 1];
	}
}

// What the size pass reads per liquid
typedef struct b2fLiquidSizes
{
	float baseMass, ratio, reach, fastSpeed, tolerance, limit, rate, safeScale;
	bool depthLimit;
} b2fLiquidSizes;

// Runs between steps on one thread, in particle order, so that the result does not depend on worker count.
void b2fAdaptSizes( b2fFluid* f )
{
	if ( f->adaptive == false || f->gridValid == false || f->h <= 0.0f || f->count == 0 )
	{
		return;
	}

	f->sizeChanges = 0;
	int n = f->count;

	// Mass linear in depth, as in Winchenbach et al. 2017, reaching the coarsest size at half the depth that size can
	// hold up by pressure alone. That grows sizes fastest right at a surface, twice the finest within 0.4 m at the
	// default settings, which keeps small, soft particles to a thin layer along a deep floor. Next to a moving body it
	// cost lift, a quarter of it for a crate near the floor, so there size grows in a line with distance instead.
	// Masses count in finest particles of liquid 0, whatever the liquid, so they measure area.
	float spacing = f->spacing, fineMass = f->mass;
	b2fLiquidSizes sizes[B2F_MAX_LIQUIDS];
	float perMetre[B2F_MAX_LIQUIDS];
	float deepest = 0.0f;
	bool wantFlow = false;
	for ( int l = 0; l < f->liquidCount; ++l )
	{
		const b2LiquidDef* def = &f->liquids[l].def;
		float ratio = def->tuning.adaptiveRatio > 1.0f ? def->tuning.adaptiveRatio : 1.0f;
		float g = -def->gravityScale * f->worldGravity.y;
		float reach = b2fAdaptiveReach( f, l, g, f->h );
		sizes[l] = ( b2fLiquidSizes ){
			.baseMass = fineMass * ratio * ratio,
			.ratio = ratio,
			.reach = reach,
			.fastSpeed = def->tuning.adaptiveSpeed,
			.tolerance = def->tuning.adaptiveTolerance,
			.limit = def->tuning.adaptiveLimit,
			.rate = def->tuning.adaptiveRate,
			.safeScale = fineMass * 2.0f * g * f->h * f->h / ( B2F_DEPTH_LIMIT_SCALE * def->tuning.stiffness * spacing * spacing ),
			.depthLimit = def->tuning.adaptiveDepthLimit,
		};
		perMetre[l] = B2F_SQUEEZE_SLOPE * g * f->h * f->h / ( def->tuning.stiffness * spacing * spacing );
		if ( ratio > 1.0f )
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
		BoxMeasure( f, f->adaptDepth, f->adaptFlow, f->adaptPressure, deepest, perMetre, wantFlow, UINT32_MAX, UINT32_MAX );
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
			BoxMeasure( f, f->adaptDepth, f->adaptFlow, f->adaptPressure, deepest, perMetre, wantFlow, f->liquidSees[l], measured );
			done |= measured;
		}
	}
	f->measureTime = b2GetMilliseconds( ticks );

	float memoryDecay = powf( 0.5f, 1.0f / ( 60.0f * B2F_SPEED_MEMORY ) );

	for ( int i = 0; i < n; ++i )
	{
		const b2fLiquidSizes* liquid = sizes + f->liquid[i];
		float baseMass = liquid->baseMass, fastSpeed = liquid->fastSpeed, limit = liquid->limit, tolerance = liquid->tolerance;
		float t = f->adaptDepth[i] / liquid->reach;
		t = t < 1.0f ? t : 1.0f;
		float target = fineMass + ( baseMass - fineMass ) * t;

		// Around a moving body size grows in a line with distance from it instead, at the rate that reaches the
		// coarsest size at the reach. A body within this distance of the particle makes it finer than its depth asks.
		float slope = ( liquid->ratio - 1.0f ) / liquid->reach;
		float near = slope > 0.0f ? ( sqrtf( target / fineMass ) - 1.0f ) / slope : 0.0f;
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
				target = fineMass * size * size;
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
			target += ( fineMass - target ) * k;
		}

		if ( liquid->depthLimit )
		{
			// Boxes read walls and floors as air, so the limit goes by squeeze, which feels the weight above
			float safe = liquid->safeScale * f->adaptPressure[i];
			safe = safe < baseMass ? safe : baseMass;
			target = target > safe ? target : safe;
		}

		float s = f->size[i] / spacing;
		float mass = fineMass * s * s;
		float relative = mass / target;
		f->adaptMass[i] = mass;
		f->adaptTarget[i] = target;
		f->adaptUsed[i] = 0;
		f->adaptClass[i] = relative < 1.0f / limit ? b2f_tooSmall : relative <= 1.0f - tolerance ? b2f_small : relative < 1.0f + tolerance ? b2f_good : relative <= limit ? b2f_large : b2f_tooLarge;
	}

	// Removing one particle at a time gives n+1 to n merges, which can always find partners
	int removed = 0;
	for ( int i = 0; i < n; ++i )
	{
		int cls = f->adaptClass[i];

		b2fCoord x = f->pos[i].x, y = f->pos[i].y;
		float reachI = 2.0f * f->size[i], reach2 = reachI * reachI;

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

		// A particle moves at most this much mass a frame, so its size and its neighbours' change a little at a
		// time. One that is merging away is only removed once what is left is that small, or a quarter of the
		// finest, so it never shrinks to a speck with too short a reach to find neighbours.
		const b2fLiquidSizes* liquid = sizes + f->liquid[i];
		float cap = liquid->rate * f->adaptTarget[i], baseMass = liquid->baseMass;
		float last = cap > B2F_REMOVE_FLOOR * fineMass ? cap : B2F_REMOVE_FLOOR * fineMass;
		bool removing = cls == b2f_tooSmall && f->adaptMass[i] <= last;
		float excess = cls == b2f_tooSmall ? f->adaptMass[i] : f->adaptMass[i] - f->adaptTarget[i];
		float give = removing ? excess : ( excess < cap ? excess : cap );
		if ( cls == b2f_tooSmall && removing == false && f->adaptMass[i] - give < last )
		{
			give = f->adaptMass[i] - last;
		}
		// Particles have moved since the grid was sorted, by under one cell in the last substep, so search one more
		int partners[B2F_MAX_PARTNERS], count = 0;
		FOR_NEIGHBOURS_WITHIN( f, i, j, f->window[i] + 1 )
		{
			if ( j == i || j >= n || f->adaptUsed[j] || count == B2F_MAX_PARTNERS || f->adaptClass[j] > b2f_small || f->liquid[j] != f->liquid[i] )
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
			removed += 1;
		}
	}

	// Close the gaps left by removed particles, and carry the classes along for the splits below
	int kept = 0;
	for ( int i = 0; i < n; ++i )
	{
		if ( f->adaptMass[i] <= 0.0f )
		{
			continue;
		}

		b2fCopyParticle( f, kept, i );
		f->size[kept] = spacing * sqrtf( f->adaptMass[i] / fineMass );
		f->adaptClass[kept] = f->adaptClass[i];
		f->adaptTarget[kept] = f->adaptTarget[i];
		kept += 1;
	}
	f->count = kept;

	// Children sit on a ring, with one in the middle past four, and copy the parent's velocity
	for ( int i = 0; i < kept; ++i )
	{
		if ( f->adaptClass[i] != b2f_tooLarge )
		{
			continue;
		}

		float s = f->size[i] / spacing;
		float mass = fineMass * s * s;
		int children = (int)ceilf( mass / f->adaptTarget[i] );
		children = children < B2F_MAX_CHILDREN ? children : B2F_MAX_CHILDREN;
		// This runs between steps, so the fluid can grow here; it stays whole only at maxCapacity
		if ( children < 2 || b2fGrow( f, f->count + children - 1 ) == false )
		{
			continue;
		}

		float child = f->size[i] / sqrtf( (float)children );
		int ring = children > 4 ? children - 1 : children;
		float radius = child * B2F_SPAWN_SPACING * 0.5f / sinf( 3.14159265f / (float)ring );
		float phase = 2.3999632f * (float)f->id[i];
		b2fCoord x = f->pos[i].x, y = f->pos[i].y;

		// A child inside a shape reads as a deep collision and would kick the body, so stay whole until clear
		bool clear = true;
		for ( int si = 0; si < f->shapeCount && clear; ++si )
		{
			const b2fShape* shape = f->shapes + si;
			float reachOut = radius + child;
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
				if ( b2fShapeDistance( shape, local, &distance, &normal ) && distance < 0.5f * child )
				{
					clear = false;
				}
			}
		}

		if ( clear == false )
		{
			continue;
		}

		for ( int c = 0; c < children; ++c )
		{
			int index = c == 0 ? i : f->count++;
			if ( index != i )
			{
				b2fCopyParticle( f, index, i );
				f->id[index] = f->nextParticleId++;
				// A new particle holds no springs. Its parent's partners hold theirs to the parent's id.
				if ( f->springs != NULL )
				{
					f->springs[index].count = 0;
				}
			}
			bool centre = children > 4 && c == children - 1;
			float angle = phase + 6.2831853f * (float)c / (float)ring;
			f->pos[index].x = centre ? x : x + radius * cosf( angle );
			f->pos[index].y = centre ? y : y + radius * sinf( angle );
			f->size[index] = child;
		}
	}

	f->gridValid = false;
	(void)removed;
}

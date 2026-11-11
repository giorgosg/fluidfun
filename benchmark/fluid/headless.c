// SPDX-License-Identifier: MIT
// Runs the shared scene with no window. Prints timing and a hash of the final state, so that
// runs can be compared across worker counts. Usage: fluid_headless [frames] [liquid scale]
//
// The environment picks the run: LAB_SCENE (number from scene.h), LAB_CONTROL, LAB_CRATES, LAB_CRATE_HALF, LAB_SUBSTEPS,
// LAB_ONE (6 workers only), LAB_SECOND_LIQUID, LAB_CHECK (fail on differing hashes or leaks), and fluid settings, see
// ApplySettings. Probes print extra lines: LAB_LEVEL, LAB_CHANGES, LAB_BODIES, LAB_EVENTS, LAB_TRACE,
// LAB_LIQUIDS, LAB_CALM, LAB_SINK, LAB_STATE, LAB_HOLES (every so many frames), LAB_SQUEEZE, LAB_SQUEEZE_DUMP and LAB_FRAMES with
// LAB_FRAME_EVERY.
#include "scene.h"

#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Writes a binary PPM of the particles and every shape of the scene, for looking at without a window.
enum { imageWidth = 960, imageHeight = 480 };
static unsigned char image[imageWidth * imageHeight * 3];
static float viewScale;
static b2Vec2 viewCenter;

static void PlotPixel( int ix, int iy, int red, int green, int blue )
{
	if ( ix >= 0 && ix < imageWidth && iy >= 0 && iy < imageHeight )
	{
		unsigned char* p = image + 3 * ( iy * imageWidth + ix );
		p[0] = (unsigned char)red, p[1] = (unsigned char)green, p[2] = (unsigned char)blue;
	}
}

static void PlotLine( b2Vec2 a, b2Vec2 b )
{
	float length = b2Distance( a, b ) * viewScale;
	int steps = (int)length + 1;
	for ( int i = 0; i <= steps; ++i )
	{
		b2Vec2 p = b2Lerp( a, b, (float)i / (float)steps );
		int ix = (int)( 0.5f * imageWidth + ( p.x - viewCenter.x ) * viewScale );
		int iy = (int)( 0.5f * imageHeight - ( p.y - viewCenter.y ) * viewScale );
		PlotPixel( ix, iy, 255, 200, 40 );
	}
}

static void PlotBody( b2BodyId body )
{
	b2ShapeId shapes[128];
	int count = b2Body_GetShapes( body, shapes, 128 );
	b2WorldTransform t = b2Body_GetTransform( body );
	for ( int i = 0; i < count; ++i )
	{
		b2ShapeType type = b2Shape_GetType( shapes[i] );
		if ( type == b2_polygonShape )
		{
			b2Polygon polygon = b2Shape_GetPolygon( shapes[i] );
			for ( int j = 0; j < polygon.count; ++j )
			{
				b2Vec2 a = b2ToVec2( b2TransformWorldPoint( t, polygon.vertices[j] ) );
				b2Vec2 b = b2ToVec2( b2TransformWorldPoint( t, polygon.vertices[( j + 1 ) % polygon.count] ) );
				PlotLine( a, b );
			}
		}
		else if ( type == b2_capsuleShape )
		{
			b2Capsule capsule = b2Shape_GetCapsule( shapes[i] );
			b2Vec2 c1 = b2ToVec2( b2TransformWorldPoint( t, capsule.center1 ) );
			b2Vec2 c2 = b2ToVec2( b2TransformWorldPoint( t, capsule.center2 ) );
			b2Vec2 n = b2MulSV( capsule.radius, b2LeftPerp( b2Normalize( b2Sub( c2, c1 ) ) ) );
			PlotLine( b2Add( c1, n ), b2Add( c2, n ) );
			PlotLine( b2Sub( c1, n ), b2Sub( c2, n ) );
			PlotLine( b2Add( c1, n ), b2Sub( c1, n ) );
			PlotLine( b2Add( c2, n ), b2Sub( c2, n ) );
		}
		else if ( type == b2_segmentShape || type == b2_chainSegmentShape )
		{
			b2Segment segment = type == b2_segmentShape ? b2Shape_GetSegment( shapes[i] ) : b2Shape_GetChainSegment( shapes[i] ).segment;
			PlotLine( b2ToVec2( b2TransformWorldPoint( t, segment.point1 ) ), b2ToVec2( b2TransformWorldPoint( t, segment.point2 ) ) );
		}
		else if ( type == b2_circleShape )
		{
			b2Circle circle = b2Shape_GetCircle( shapes[i] );
			b2Vec2 c = b2ToVec2( b2TransformWorldPoint( t, circle.center ) );
			for ( int j = 0; j < 24; ++j )
			{
				float a0 = 2.0f * B2_PI * (float)j / 24.0f, a1 = 2.0f * B2_PI * (float)( j + 1 ) / 24.0f;
				PlotLine( b2Add( c, (b2Vec2){ circle.radius * cosf( a0 ), circle.radius * sinf( a0 ) } ),
						  b2Add( c, (b2Vec2){ circle.radius * cosf( a1 ), circle.radius * sinf( a1 ) } ) );
			}
		}
	}
}

static void WriteFrame( const char* path, b2FluidId fluid, const LabScene* scene )
{
	memset( image, 24, sizeof( image ) );
	// Tall scenes are fitted by height, wide ones by width
	float zoom = getenv( "LAB_ZOOM" ) ? (float)atof( getenv( "LAB_ZOOM" ) ) : 1.0f;
	viewScale = (float)imageWidth / ( 2.0f * zoom * scene->viewHalfWidth );
	viewCenter = scene->viewCenter;
	viewCenter.y = getenv( "LAB_VIEW_Y" ) ? (float)atof( getenv( "LAB_VIEW_Y" ) ) : viewCenter.y;

	b2FluidParticles particles = b2Fluid_GetParticles( fluid );
	int n = particles.count;
	const b2Pos* pos = particles.positions;
	// With LAB_SIZED each particle is a square 0.7 of its size across, as the sample draws them
	const float* sizes = particles.sizes;
	const unsigned char* liquids = particles.liquids;
	bool sized = getenv( "LAB_SIZED" ) != NULL;
	static const unsigned char tints[3][3] = { { 60, 140, 255 }, { 230, 170, 40 }, { 200, 40, 60 } };
	for ( int i = 0; i < n; ++i )
	{
		const unsigned char* tint = tints[liquids[i] % 3];
		int px = (int)( 0.5f * imageWidth + ( pos[i].x - viewCenter.x ) * viewScale );
		int py = (int)( 0.5f * imageHeight - ( pos[i].y - viewCenter.y ) * viewScale );
		int side = sized ? (int)( 0.7f * sizes[i] * viewScale + 0.5f ) : 2;
		side = side > 1 ? side : 1;
		for ( int y = 0; y < side; ++y )
		{
			for ( int x = 0; x < side; ++x )
			{
				PlotPixel( px - side / 2 + x, py - side / 2 + y, tint[0], tint[1], tint[2] );
			}
		}
	}

	for ( int i = 0; i < scene->bodyCount; ++i )
	{
		PlotBody( scene->bodies[i] );
	}

	FILE* file = fopen( path, "wb" );
	if ( file != NULL )
	{
		fprintf( file, "P6 %d %d 255\n", imageWidth, imageHeight );
		fwrite( image, 1, sizeof( image ), file );
		fclose( file );
	}
}

static uint64_t HashBytes( uint64_t hash, const void* data, size_t size )
{
	const unsigned char* bytes = data;
	for ( size_t i = 0; i < size; ++i )
	{
		hash = ( hash ^ bytes[i] ) * 1099511628211ull;
	}
	return hash;
}

// Fluid settings that can be set from the environment, for example LAB_ADAPTIVE=1 for uniform sizes
static void ApplySettings( LabDef* def )
{
	struct
	{
		const char* name;
		float* value;
	} settings[] = {
		{ "LAB_SPACING", &def->fluid.particleSpacing },
		{ "LAB_CONTACT", &def->liquid.tuning.buoyancyAssist },
		{ "LAB_ADDED_MASS", &def->liquid.tuning.addedMass },
		{ "LAB_SPRING", &def->liquid.springStiffness },
		{ "LAB_PLASTICITY", &def->liquid.plasticity },
		{ "LAB_YIELD", &def->liquid.yieldRatio },
		{ "LAB_FRICTION", &def->liquid.friction },
		{ "LAB_TENSILE", &def->liquid.tuning.tensilePressure },
		{ "LAB_COHESION", &def->liquid.cohesion },
		{ "LAB_ADHESION", &def->liquid.adhesion },
		{ "LAB_GRAVITY", &def->liquid.gravityScale },
		{ "LAB_VISCOSITY", &def->liquid.viscosity },
		{ "LAB_QUADRATIC", &def->liquid.tuning.quadraticViscosity },
		{ "LAB_ADAPTIVE", &def->liquid.tuning.adaptiveRatio },
		{ "LAB_ADAPTIVE_DEPTH", &def->fluid.adaptiveDepth },
		{ "LAB_FAST", &def->liquid.tuning.adaptiveSpeed },
		{ "LAB_FULL", &def->fluid.adaptiveFullBox },
		{ "LAB_TOLERANCE", &def->liquid.tuning.adaptiveTolerance },
		{ "LAB_LIMIT", &def->liquid.tuning.adaptiveLimit },
		{ "LAB_RATE", &def->liquid.tuning.adaptiveRate },
	};
	for ( size_t i = 0; i < sizeof( settings ) / sizeof( settings[0] ); ++i )
	{
		const char* text = getenv( settings[i].name );
		if ( text != NULL )
		{
			*settings[i].value = (float)atof( text );
		}
	}

	// Near stiffness follows stiffness, at the usual 2.5 times
	if ( getenv( "LAB_STIFFNESS" ) )
	{
		def->liquid.tuning.stiffness = (float)atof( getenv( "LAB_STIFFNESS" ) );
		def->liquid.tuning.nearStiffness = 2.5f * def->liquid.tuning.stiffness;
	}
	if ( getenv( "LAB_DEPTH_LIMIT" ) )
	{
		def->liquid.tuning.adaptiveDepthLimit = atoi( getenv( "LAB_DEPTH_LIMIT" ) ) != 0;
	}
}

// LAB_HOLES: the shape of the liquid. Holes are empty cells of two spacings that no path of empty cells joins to
// the outside, so they are gaps the liquid closes around. Packing is density over the lattice density for particles
// with at least 5 neighbours, and gap is each such particle's nearest neighbour over the spacing, as percentiles.
// Sizes are taken as uniform, so run it with LAB_ADAPTIVE=1.
static void PrintShape( b2FluidId fluid, float spacing, int frame )
{
	b2FluidParticles particles = b2Fluid_GetParticles( fluid );
	int n = particles.count;
	const b2Pos* pos = particles.positions;
	const float* rho = particles.densities;
	const float* vx = particles.velocitiesX;
	const float* vy = particles.velocitiesY;
	float rest = particles.restDensity;
	if ( n == 0 )
	{
		return;
	}

	float cell = 2.0f * spacing;
	double minX = 1e9, minY = 1e9, maxX = -1e9, maxY = -1e9, speed2 = 0.0;
	for ( int i = 0; i < n; ++i )
	{
		speed2 += (double)vx[i] * vx[i] + (double)vy[i] * vy[i];
		minX = pos[i].x < minX ? pos[i].x : minX;
		minY = pos[i].y < minY ? pos[i].y : minY;
		maxX = pos[i].x > maxX ? pos[i].x : maxX;
		maxY = pos[i].y > maxY ? pos[i].y : maxY;
	}
	int w = (int)( ( maxX - minX ) / cell ) + 3, h = (int)( ( maxY - minY ) / cell ) + 3;
	if ( (long)w * h > 50000000L )
	{
		return;
	}
	int* grid = calloc( (size_t)w * h, sizeof( int ) );
	int* head = malloc( (size_t)w * h * sizeof( int ) );
	int* next = malloc( (size_t)n * sizeof( int ) );
	for ( int c = 0; c < w * h; ++c )
	{
		head[c] = -1;
	}
	for ( int i = 0; i < n; ++i )
	{
		int c = (int)( ( pos[i].y - minY ) / cell + 1 ) * w + (int)( ( pos[i].x - minX ) / cell + 1 );
		grid[c] = 1;
		next[i] = head[c];
		head[c] = i;
	}

	// Flood the empty cells from the border, which is always empty
	int* stack = malloc( (size_t)w * h * sizeof( int ) );
	int top = 0;
	stack[top++] = 0;
	grid[0] = 2;
	while ( top > 0 )
	{
		int c = stack[--top], cx = c % w, cy = c / w;
		int around[4] = { cx > 0 ? c - 1 : -1, cx < w - 1 ? c + 1 : -1, cy > 0 ? c - w : -1, cy < h - 1 ? c + w : -1 };
		for ( int k = 0; k < 4; ++k )
		{
			if ( around[k] >= 0 && grid[around[k]] == 0 )
			{
				grid[around[k]] = 2;
				stack[top++] = around[k];
			}
		}
	}
	// Mass-weighted vertical speed in the strip within 0.4 m of the sides, and in the middle, below the top tenth
	const float* sizes = particles.sizes;
	double wallV = 0.0, wallM = 0.0, midV = 0.0, midM = 0.0, wallRho = 0.0, wallLow = 0.0, midRho = 0.0, midLow = 0.0;
	for ( int i = 0; i < n; ++i )
	{
		if ( pos[i].y > minY + 0.9 * ( maxY - minY ) )
		{
			continue;
		}
		double m = (double)sizes[i] * sizes[i];
		bool wall = pos[i].x < minX + 0.4 || pos[i].x > maxX - 0.4;
		bool middle = fabs( pos[i].x - 0.5 * ( minX + maxX ) ) < 0.25 * ( maxX - minX );
		wallV += wall ? m * vy[i] : 0.0;
		wallM += wall ? m : 0.0;
		midV += middle ? m * vy[i] : 0.0;
		midM += middle ? m : 0.0;

		// Density over rest in the bottom half, where section 19 found the wall strip denser
		bool low = pos[i].y < minY + 0.5 * ( maxY - minY );
		wallRho += wall && low ? m * rho[i] / rest : 0.0;
		wallLow += wall && low ? m : 0.0;
		midRho += middle && low ? m * rho[i] / rest : 0.0;
		midLow += middle && low ? m : 0.0;
	}

	int holes = 0, full = 0;
	for ( int c = 0; c < w * h; ++c )
	{
		holes += grid[c] == 0;
		full += grid[c] == 1;
	}

	// Nearest neighbours and packing of particles inside the liquid
	enum { binCount = 200 };
	static int gapBins[binCount], packBins[binCount];
	memset( gapBins, 0, sizeof( gapBins ) );
	memset( packBins, 0, sizeof( packBins ) );
	int inside = 0;
	for ( int i = 0; i < n; ++i )
	{
		int cx = (int)( ( pos[i].x - minX ) / cell + 1 ), cy = (int)( ( pos[i].y - minY ) / cell + 1 );
		float nearest = 1e9f;
		int neighbours = 0;
		for ( int y = cy - 1; y <= cy + 1; ++y )
		{
			for ( int x = cx - 1; x <= cx + 1; ++x )
			{
				for ( int j = head[y * w + x]; j >= 0; j = next[j] )
				{
					float dx = (float)( pos[j].x - pos[i].x ), dy = (float)( pos[j].y - pos[i].y ), d = sqrtf( dx * dx + dy * dy );
					if ( j != i && d < cell )
					{
						neighbours += 1;
						nearest = d < nearest ? d : nearest;
					}
				}
			}
		}
		if ( neighbours >= 5 )
		{
			int g = (int)( 100.0f * nearest / spacing ), r = (int)( 100.0f * rho[i] / rest );
			gapBins[g < binCount ? g : binCount - 1] += 1;
			packBins[r < binCount ? r : binCount - 1] += 1;
			inside += 1;
		}
	}
	float gap[3], pack[3], shares[3] = { 0.05f, 0.5f, 0.95f };
	for ( int k = 0; k < 3; ++k )
	{
		int want = (int)( shares[k] * inside ), sumGap = 0, sumPack = 0;
		gap[k] = pack[k] = 0.0f;
		for ( int b = 0, gotGap = 0, gotPack = 0; b < binCount; ++b )
		{
			sumGap += gapBins[b];
			sumPack += packBins[b];
			if ( !gotGap && sumGap > want )
			{
				gap[k] = 0.01f * (float)b;
				gotGap = 1;
			}
			if ( !gotPack && sumPack > want )
			{
				pack[k] = 0.01f * (float)b;
				gotPack = 1;
			}
		}
	}
	printf( "HOLES frame %d holes %d of %d cells, packing %.2f %.2f %.2f, gap %.2f %.2f %.2f, top %.2f, rms %.3f, width %.2f height %.2f, wall vy %.3f middle vy %.3f, low density wall %.3f middle %.3f\n", frame, holes, full,
			(double)pack[0], (double)pack[1], (double)pack[2], (double)gap[0], (double)gap[1], (double)gap[2], maxY, sqrt( speed2 / n ), maxX - minX, maxY - minY,
			wallM > 0.0 ? wallV / wallM : 0.0, midM > 0.0 ? midV / midM : 0.0,
			wallLow > 0.0 ? wallRho / wallLow : 0.0, midLow > 0.0 ? midRho / midLow : 0.0 );
	free( grid );
	free( head );
	free( next );
	free( stack );
}

int main( int argc, char** argv )
{
	int frames = argc > 1 ? atoi( argv[1] ) : 300;
	float scale = argc > 2 ? (float)atof( argv[2] ) : 1.0f;
	bool oneRun = getenv( "LAB_ONE" ) != NULL;

	// With LAB_CHECK the exit code says whether every worker count gave the same state and nothing leaked
	bool check = getenv( "LAB_CHECK" ) != NULL;
	uint64_t firstHash = 0;
	int failures = 0;

	static const int workerCounts[] = { 1, 3, 6, 12 };
	for ( size_t w = oneRun ? 2 : 0; w < sizeof( workerCounts ) / sizeof( workerCounts[0] ); ++w )
	{
		b2WorldDef worldDef = b2DefaultWorldDef();
		worldDef.workerCount = workerCounts[w];
		b2WorldId worldId = b2CreateWorld( &worldDef );

		LabScene scene = LabDefaultScene( getenv( "LAB_SCENE" ) ? atoi( getenv( "LAB_SCENE" ) ) : lab_tank );
		if ( getenv( "LAB_CONTROL" ) )
		{
			scene.control = (float)atof( getenv( "LAB_CONTROL" ) );
		}
		if ( getenv( "LAB_CRATES" ) )
		{
			scene.crateCount = atoi( getenv( "LAB_CRATES" ) );
		}
		if ( getenv( "LAB_CRATE_HALF" ) )
		{
			scene.crateHalfSize = (float)atof( getenv( "LAB_CRATE_HALF" ) );
		}
		scene.liquidWidth *= scale;
		scene.tankWidth *= scale;
		scene.liquidHeight *= scale;
		scene.tankHeight *= scale;
		LabDef def = LabDefaultDef();
		def.fluid.capacity = 2000000;
		LabSceneSettings( scene.type, &def );
		ApplySettings( &def );
		b2FluidId fluid = LabCreateScene( worldId, &scene, &def );
		// LAB_SECOND_LIQUID: add a liquid like the first with no particles, which runs the passes for several
		// liquids on the same physics, to time them
		if ( getenv( "LAB_SECOND_LIQUID" ) )
		{
			b2CreateLiquid( fluid, &def.liquid );
		}

		// LAB_BODY_DENSITY: give every moving body this density, to find the lightest body the coupling holds
		if ( getenv( "LAB_BODY_DENSITY" ) )
		{
			float density = (float)atof( getenv( "LAB_BODY_DENSITY" ) );
			for ( int k = 0; k < scene.bodyCount; ++k )
			{
				if ( b2Body_GetType( scene.bodies[k] ) != b2_dynamicBody )
				{
					continue;
				}
				b2ShapeId shapes[128];
				int shapeCount = b2Body_GetShapes( scene.bodies[k], shapes, 128 );
				for ( int s = 0; s < shapeCount; ++s )
				{
					b2Shape_SetDensity( shapes[s], density, s == shapeCount - 1 );
				}
			}
		}

		// LAB_BODY_STATS=seconds: per moving body, the peak speed over the run, and the mean and peak speed and spin
		// after that many seconds, when it should have settled, and the highest it got. A coupling that blows up shows as
		// large values, and a light body launched out of the liquid as a height well above the surface.
		float settle = getenv( "LAB_BODY_STATS" ) ? (float)atof( getenv( "LAB_BODY_STATS" ) ) : -1.0f;
		float peakSpeed[LAB_MAX_BODIES] = { 0 }, lateMean[LAB_MAX_BODIES] = { 0 }, latePeak[LAB_MAX_BODIES] = { 0 };
		float latePeakSpin[LAB_MAX_BODIES] = { 0 }, peakY[LAB_MAX_BODIES];
		for ( int k = 0; k < LAB_MAX_BODIES; ++k )
		{
			peakY[k] = -FLT_MAX;
		}
		float lowBottom[LAB_MAX_BODIES], highBottom[LAB_MAX_BODIES], under[LAB_MAX_BODIES] = { 0 };
		for ( int k = 0; k < LAB_MAX_BODIES; ++k )
		{
			lowBottom[k] = FLT_MAX;
			highBottom[k] = -FLT_MAX;
		}
		int lateFrames = 0;

		uint64_t start = b2GetTicks();
		b2FluidProfile sum = { 0 };
		double measureSum = 0.0;
		for ( int i = 0; i < frames; ++i )
		{
			LabStepScene( &scene, fluid, 1.0f / 60.0f );
			// LAB_SINK=start,end: between those seconds drive every moving body down at 4 m/s, as dragging it to the
			// floor with the mouse does, then let go. SINK lines give each body's height every half second.
			if ( getenv( "LAB_SINK" ) )
			{
				float sinkStart = 0.0f, sinkEnd = 0.0f, t = (float)i / 60.0f;
				sscanf( getenv( "LAB_SINK" ), "%f,%f", &sinkStart, &sinkEnd );
				if ( i % 30 == 0 )
				{
					printf( "SINK %.1f", (double)t );
				}
				for ( int k = 0; k < scene.bodyCount; ++k )
				{
					if ( b2Body_GetType( scene.bodies[k] ) != b2_dynamicBody )
					{
						continue;
					}
					if ( t >= sinkStart && t < sinkEnd )
					{
						b2Body_SetLinearVelocity( scene.bodies[k], (b2Vec2){ 0.0f, -4.0f } );
						b2Body_SetAngularVelocity( scene.bodies[k], 0.0f );
					}
					if ( i % 30 == 0 )
					{
						printf( " %.2f", (double)b2Body_GetPosition( scene.bodies[k] ).y );
					}
				}
				if ( i % 30 == 0 )
				{
					printf( "\n" );
				}
			}

			b2World_Step( worldId, 1.0f / 60.0f, getenv( "LAB_SUBSTEPS" ) ? atoi( getenv( "LAB_SUBSTEPS" ) ) : ( scene.subSteps > 0 ? scene.subSteps : 4 ) );
			b2FluidProfile p = b2Fluid_GetProfile( fluid );
			measureSum += p.measure;
			if ( settle >= 0.0f )
			{
				bool late = (float)( i + 1 ) / 60.0f > settle;
				lateFrames += late ? 1 : 0;
				for ( int k = 0; k < scene.bodyCount; ++k )
				{
					if ( b2Body_GetType( scene.bodies[k] ) != b2_dynamicBody )
					{
						continue;
					}
					float speed = b2Length( b2Body_GetLinearVelocity( scene.bodies[k] ) );
					float spin = fabsf( b2Body_GetAngularVelocity( scene.bodies[k] ) );
					speed = isfinite( speed ) ? speed : 1e9f;
					peakSpeed[k] = speed > peakSpeed[k] ? speed : peakSpeed[k];
					float y = (float)b2Body_GetPosition( scene.bodies[k] ).y;
					peakY[k] = y > peakY[k] ? y : peakY[k];
					if ( late )
					{
						lateMean[k] += speed;
						latePeak[k] = speed > latePeak[k] ? speed : latePeak[k];
						latePeakSpin[k] = spin > latePeakSpin[k] ? spin : latePeakSpin[k];

						// How high the body's bottom rode, and how many particles were under it, inside its footprint
						// and below its bottom, where liquid seeps in under a body resting on the floor
						b2AABB box = b2Body_ComputeBounds( scene.bodies[k] );
						lowBottom[k] = box.lowerBound.y < lowBottom[k] ? box.lowerBound.y : lowBottom[k];
						highBottom[k] = box.lowerBound.y > highBottom[k] ? box.lowerBound.y : highBottom[k];
						float inset = 2.0f * def.fluid.particleSpacing;
						b2FluidParticles particles = b2Fluid_GetParticles( fluid );
						const b2Pos* ps = particles.positions;
						int n = particles.count, count = 0;
						for ( int j = 0; j < n; ++j )
						{
							float px = (float)ps[j].x, py = (float)ps[j].y;
							count += px > box.lowerBound.x + inset && px < box.upperBound.x - inset && py < box.lowerBound.y + 0.3f * def.fluid.particleSpacing &&
											 py > box.lowerBound.y - 4.0f * def.fluid.particleSpacing
										 ? 1
										 : 0;
						}
						under[k] += (float)count;
					}
				}
			}
			if ( getenv( "LAB_HOLES" ) && ( i + 1 ) % atoi( getenv( "LAB_HOLES" ) ) == 0 )
			{
				PrintShape( fluid, def.fluid.particleSpacing, i + 1 );
			}
			if ( getenv( "LAB_LEVEL" ) && ( i < 60 ? i % 10 == 0 : i % 120 == 0 ) )
			{
				// Mass weighted height of the liquid: lower means it packed tighter or left a gap
				b2FluidParticles particles = b2Fluid_GetParticles( fluid );
				int n = particles.count;
				const b2Pos* pos = particles.positions;
				const float* ps = particles.sizes;
				double my = 0.0, m = 0.0, fineY = 0.0, coarseY = 0.0;
				int fine = 0, coarse = 0;
				for ( int k = 0; k < n; ++k )
				{
					double weight = (double)ps[k] * ps[k];
					my += weight * pos[k].y;
					m += weight;
					float r = ps[k] / def.fluid.particleSpacing;
					if ( r < 1.5f )
					{
						fineY += pos[k].y;
						fine += 1;
					}
					else if ( r > 3.0f )
					{
						coarseY += pos[k].y;
						coarse += 1;
					}
				}
				printf( "LV %d %.4f n=%d fine %d at y %.2f, coarse %d at y %.2f\n", i, my / m, n, fine, fine > 0 ? fineY / fine : 0.0,
						coarse, coarse > 0 ? coarseY / coarse : 0.0 );
			}
			sum.sort += p.sort;
			sum.density += p.density;
			sum.collide += p.collide;
			sum.total += p.total;
			const char* frameDir = getenv( "LAB_FRAMES" );
			if ( frameDir != NULL && i % ( getenv( "LAB_FRAME_EVERY" ) ? atoi( getenv( "LAB_FRAME_EVERY" ) ) : 60 ) == 0 )
			{
				char path[512];
				snprintf( path, sizeof( path ), "%s/%04d.ppm", frameDir, i );
				WriteFrame( path, fluid, &scene );
			}
			// LAB_CALM: every half second, the liquid's mean and peak speed and the mean height of its particles, to see
			// how quickly liquid spawned at rest settles, and how far it sinks or swells while it does
			if ( getenv( "LAB_CALM" ) && i % 30 == 0 )
			{
				b2FluidParticles particles = b2Fluid_GetParticles( fluid );
				int n = particles.count;
				const b2Pos* pos = particles.positions;
				const float* cvx = particles.velocitiesX;
				const float* cvy = particles.velocitiesY;
				double speedSum = 0.0, height = 0.0;
				float peak = 0.0f;
				for ( int k = 0; k < n; ++k )
				{
					float v = sqrtf( cvx[k] * cvx[k] + cvy[k] * cvy[k] );
					speedSum += v;
					height += pos[k].y;
					peak = peak > v ? peak : v;
				}
				printf( "CALM %.1f mean %.3f peak %.2f y %.3f\n", (double)i / 60.0, n > 0 ? speedSum / n : 0.0, (double)peak, n > 0 ? height / n : 0.0 );
			}
			// After the first and last steps, density over rest density and particle size in bands a metre high, to see
			// whether liquid spawns packed tighter or looser than it rests
			if ( getenv( "LAB_CALM" ) && ( i == 0 || i == frames - 1 ) )
			{
				b2FluidParticles particles = b2Fluid_GetParticles( fluid );
				int n = particles.count;
				const b2Pos* pos = particles.positions;
				{
					const float* rho = particles.densities;
					const float* size = particles.sizes;
					float rest = particles.restDensity;
					double bandRho[64] = { 0 }, bandSize[64] = { 0 };
					int bandCount[64] = { 0 };
					for ( int k = 0; k < n; ++k )
					{
						int band = (int)floor( pos[k].y );
						band = band < 0 ? 0 : ( band > 63 ? 63 : band );
						bandRho[band] += rho[k] / rest;
						bandSize[band] += size[k];
						bandCount[band] += 1;
					}
					for ( int band = 0; band < 64; ++band )
					{
						if ( bandCount[band] > 0 )
						{
							printf( "%s y %d-%d: %d particles, density/rest %.3f, size %.3f\n", i == 0 ? "SPAWN" : "END", band, band + 1,
									bandCount[band], bandRho[band] / bandCount[band], bandSize[band] / bandCount[band] );
						}
					}
				}
			}

			if ( getenv( "LAB_CHANGES" ) )
			{
				printf( "CH %d\n", p.sizeChanges );
			}

			if ( getenv( "LAB_LIQUIDS" ) && i % 60 == 0 )
			{
				// Per liquid: area, area weighted mean height and its spread. Separated layers stack by density,
				// each with a spread near its thickness over the square root of 12.
				b2FluidParticles particles = b2Fluid_GetParticles( fluid );
				int n = particles.count;
				const b2Pos* pos = particles.positions;
				const float* ps = particles.sizes;
				const unsigned char* liquid = particles.liquids;
				double area[B2_MAX_LIQUIDS] = { 0 }, my[B2_MAX_LIQUIDS] = { 0 }, myy[B2_MAX_LIQUIDS] = { 0 };
				for ( int k = 0; k < n; ++k )
				{
					double weight = (double)ps[k] * ps[k];
					area[liquid[k]] += weight;
					my[liquid[k]] += weight * pos[k].y;
					myy[liquid[k]] += weight * pos[k].y * pos[k].y;
				}
				printf( "LQ %d", i );
				for ( int l = 0; l < b2Fluid_GetLiquidCount( fluid ); ++l )
				{
					double mean = area[l] > 0.0 ? my[l] / area[l] : 0.0;
					double spread = area[l] > 0.0 ? sqrt( fmax( myy[l] / area[l] - mean * mean, 0.0 ) ) : 0.0;
					printf( " | %d: area %.3f y %.3f spread %.3f", l, area[l] * 0.8660254, mean, spread );
				}
				printf( "\n" );
			}
			// LAB_STATE: every half second, or every so many frames if it is a number above 1, each moving body's share under liquid and the liquid's push along up over its
			// weight, which is near 1 for a body afloat, and every frame the fluid's events. Level is the share of the
			// body's height under the surface found two spacings to the right of its shapes, to compare submerged with.
			if ( getenv( "LAB_STATE" ) )
			{
				b2Vec2 gravity = b2World_GetGravity( worldId );
				float g = b2Length( gravity );
				b2Vec2 up = g > 0.0f ? b2MulSV( -1.0f / g, gravity ) : ( b2Vec2 ){ 0.0f, 1.0f };
				int every = atoi( getenv( "LAB_STATE" ) ) > 1 ? atoi( getenv( "LAB_STATE" ) ) : 30;
				if ( ( i + 1 ) % every == 0 )
				{
					printf( "STATE %.2f", (double)( i + 1 ) / 60.0 );
					for ( int k = 0; k < scene.bodyCount; ++k )
					{
						if ( b2Body_GetType( scene.bodies[k] ) != b2_dynamicBody )
						{
							continue;
						}
						b2FluidBodyState state = b2Fluid_GetBodyState( fluid, scene.bodies[k] );
						float weight = b2Body_GetMass( scene.bodies[k] ) * g;
						// The body's extent from its polygons' corners, since the bounds Box2D keeps are padded
						b2ShapeId shapes[16];
						int shapeCount = b2Body_GetShapes( scene.bodies[k], shapes, 16 );
						b2WorldTransform transform = b2Body_GetTransform( scene.bodies[k] );
						double low = DBL_MAX, high = -DBL_MAX, right = -DBL_MAX;
						for ( int s = 0; s < shapeCount; ++s )
						{
							if ( b2Shape_GetType( shapes[s] ) != b2_polygonShape )
							{
								continue;
							}
							b2Polygon polygon = b2Shape_GetPolygon( shapes[s] );
							for ( int v = 0; v < polygon.count; ++v )
							{
								b2Pos corner = b2TransformWorldPoint( transform, polygon.vertices[v] );
								low = corner.y < low ? corner.y : low;
								high = corner.y > high ? corner.y : high;
								right = corner.x > right ? corner.x : right;
							}
						}
						float level = 0.0f;
						b2Pos beside = { right + 2.0f * def.fluid.particleSpacing, low }, surface = beside;
						if ( high > low && b2Fluid_GetSurfaceHeight( fluid, beside, &surface ) )
						{
							level = (float)( ( surface.y - low ) / ( high - low ) );
							level = level < 0.0f ? 0.0f : ( level > 1.0f ? 1.0f : level );
						}
						printf( " | %d: %s submerged %.2f level %.2f force/weight %.2f", k, state.inLiquid ? "in" : "out", (double)state.submerged,
								(double)level, weight > 0.0f ? (double)( b2Dot( state.force, up ) / weight ) : 0.0 );
					}
					printf( "\n" );
				}

				b2FluidEvents events = b2Fluid_GetEvents( fluid );
				for ( int e = 0; e < events.bodyBeginCount; ++e )
				{
					printf( "EVENT %.2f body %d begins in liquid %d\n", (double)( i + 1 ) / 60.0, events.bodyBeginEvents[e].bodyId.index1,
							B2_IS_NULL( events.bodyBeginEvents[e].liquidId ) ? -1 : b2Liquid_GetIndex( events.bodyBeginEvents[e].liquidId ) );
				}
				for ( int e = 0; e < events.bodyEndCount; ++e )
				{
					printf( "EVENT %.2f body %d ends\n", (double)( i + 1 ) / 60.0, events.bodyEndEvents[e].bodyId.index1 );
				}
				for ( int e = 0; e < events.sensorBeginCount; ++e )
				{
					printf( "EVENT %.2f sensor %d begins\n", (double)( i + 1 ) / 60.0, events.sensorBeginEvents[e].sensorShapeId.index1 );
				}
				for ( int e = 0; e < events.sensorEndCount; ++e )
				{
					printf( "EVENT %.2f sensor %d ends\n", (double)( i + 1 ) / 60.0, events.sensorEndEvents[e].sensorShapeId.index1 );
				}
			}

			if ( getenv( "LAB_BODIES" ) && i % 120 == 0 )
			{
				printf( "T %.0f", (double)i / 60.0 );
				for ( int k = 0; k < scene.bodyCount; ++k )
				{
					if ( b2Body_GetType( scene.bodies[k] ) != b2_staticBody )
					{
						printf( " | x=%.1f y=%.2f a=%.3f vx=%.2f w=%.2f", (double)b2Body_GetPosition( scene.bodies[k] ).x, (double)b2Body_GetPosition( scene.bodies[k] ).y,
								(double)b2Rot_GetAngle( b2Body_GetRotation( scene.bodies[k] ) ), (double)b2Body_GetLinearVelocity( scene.bodies[k] ).x, (double)b2Body_GetAngularVelocity( scene.bodies[k] ) );
					}
				}
				printf( "\n" );
			}

			if ( getenv( "LAB_EVENTS" ) && i % 6 == 0 )
			{
				b2FluidParticles particles = b2Fluid_GetParticles( fluid );
				const b2Pos* pos = particles.positions;
				const float* evx = particles.velocitiesX;
				const float* evy = particles.velocitiesY;
				int count = particles.count;
				double e = 0.0, best = 0.0;
				int bi = 0;
				for ( int k = 0; k < count; ++k )
				{
					double v = evx[k] * evx[k] + evy[k] * evy[k];
					e += v;
					if ( v > best )
					{
						best = v;
						bi = k;
					}
				}
				printf( "EV %.2f rms=%.3f top=%.2f x=%.2f y=%.2f N=%d\n", (double)i / 60.0, sqrt( e / count ), sqrt( best ), (double)pos[bi].x, (double)pos[bi].y, count );
			}

			if ( getenv( "LAB_TRACE" ) && i % 30 == 0 )
			{
				b2FluidParticles particles = b2Fluid_GetParticles( fluid );
				const b2Pos* pos = particles.positions;
				const float* tvx = particles.velocitiesX;
				const float* tvy = particles.velocitiesY;
				int count = particles.count;
				float maxSpeed = 0.0f, minY = 1e9f, maxY = -1e9f, minX = 1e9f, maxX = -1e9f;
				for ( int k = 0; k < count; ++k )
				{
					float v = tvx[k] * tvx[k] + tvy[k] * tvy[k];
					maxSpeed = maxSpeed > v ? maxSpeed : v;
					minY = minY < pos[k].y ? minY : (float)pos[k].y;
					maxY = maxY > pos[k].y ? maxY : (float)pos[k].y;
					minX = minX < pos[k].x ? minX : (float)pos[k].x;
					maxX = maxX > pos[k].x ? maxX : (float)pos[k].x;
				}
				printf( "  frame %d: %.1f ms x[%.1f %.1f] y[%.2f %.2f] vmax %.1f\n", i, p.total, minX, maxX, minY, maxY, sqrtf( maxSpeed ) );
				fflush( stdout );
			}
		}
		float ms = b2GetMilliseconds( start ) / (float)frames;

		b2FluidParticles particles = b2Fluid_GetParticles( fluid );
		int n = particles.count;
		uint64_t hash = 14695981039346656037ull;
		const b2Pos* hashed = particles.positions;
		for ( int i = 0; i < n; ++i )
		{
			hash = HashBytes( hash, &hashed[i].x, sizeof( hashed[i].x ) );
		}
		for ( int i = 0; i < n; ++i )
		{
			hash = HashBytes( hash, &hashed[i].y, sizeof( hashed[i].y ) );
		}
		hash = HashBytes( hash, particles.velocitiesX, (size_t)n * sizeof( float ) );
		hash = HashBytes( hash, particles.velocitiesY, (size_t)n * sizeof( float ) );

		float top = -1e9f;
		const b2Pos* pos = particles.positions;
		for ( int i = 0; i < n; ++i )
		{
			top = top > pos[i].y ? top : (float)pos[i].y;
		}

		float f = (float)frames;
		printf( "workers=%2d n=%d %6.2f ms/frame | fluid %6.2f (sort %.2f density %.2f collide %.2f) top=%.2f hash=%016llx\n",
				workerCounts[w], n, ms, sum.total / f, sum.sort / f, sum.density / f, sum.collide / f, top,
				(unsigned long long)hash );
		printf( "  leaked %d, measure %.3f ms/frame\n", scene.leaked, measureSum / f );
		if ( w == ( oneRun ? 2u : 0u ) )
		{
			firstHash = hash;
		}
		if ( hash != firstHash || scene.leaked > 0 )
		{
			failures += 1;
		}
		if ( scene.type == lab_drain )
		{
			printf( "  pumped %d mass %.0f stall %.1fs\n", scene.pumped, (double)b2Fluid_GetMovedMass( fluid ), (double)scene.longestStall / 60.0 );
		}
		{
			const float* size = particles.sizes;
			int bins[6] = { 0 };
			for ( int i = 0; i < n; ++i )
			{
				float ratio = size[i] / def.fluid.particleSpacing;
				int bin = ratio < 1.1f ? 0 : ratio < 1.3f ? 1 : ratio < 1.6f ? 2 : ratio < 2.1f ? 3 : ratio < 3.0f ? 4 : 5;
				bins[bin] += 1;
			}
			printf( "  sizes over fine: <1.1 %d%%, 1.1-1.3 %d%%, 1.3-1.6 %d%%, 1.6-2.1 %d%%, 2.1-3 %d%%, >3 %d%%\n", 100 * bins[0] / n, 100 * bins[1] / n,
					100 * bins[2] / n, 100 * bins[3] / n, 100 * bins[4] / n, 100 * bins[5] / n );
		}
		if ( getenv( "LAB_DUMP" ) )
		{
			// Every particle as x, y, size over particleSpacing, density over rest
			FILE* dump = fopen( getenv( "LAB_DUMP" ), "w" );
			const float* ps = particles.sizes;
			const float* rho = particles.densities;
			float rest = particles.restDensity;
			for ( int i = 0; dump != NULL && i < n; ++i )
			{
				fprintf( dump, "%.5f %.5f %.4f %.4f\n", (double)pos[i].x, (double)pos[i].y, (double)( ps[i] / def.fluid.particleSpacing ), (double)( rho[i] / rest ) );
			}
			if ( dump != NULL )
			{
				fclose( dump );
			}
		}
		if ( getenv( "LAB_FILL" ) )
		{
			// Mass over area against depth below the top particle, in the middle half of the width, as a share of a
			// lattice at particleSpacing, which liquid at rest fills. Spawned liquid stands at 1 / 0.97^2 throughout.
			const float* ps = particles.sizes;
			float surface = -1e9f, left = 1e9f, right = -1e9f;
			for ( int i = 0; i < n; ++i )
			{
				surface = surface > pos[i].y ? surface : (float)pos[i].y;
				left = left < pos[i].x ? left : (float)pos[i].x;
				right = right > pos[i].x ? right : (float)pos[i].x;
			}
			enum { bins = 200 };
			double fill[bins] = { 0 }, edge[bins] = { 0 };
			float binDepth = (float)atof( getenv( "LAB_FILL" ) ), middle = 0.5f * ( left + right ), half = 0.25f * ( right - left ), strip = 0.5f;
			for ( int i = 0; i < n; ++i )
			{
				int b = (int)( ( surface - pos[i].y ) / binDepth );
				double m = 0.8660254 * ps[i] * ps[i];
				if ( b >= 0 && b < bins && fabsf( (float)pos[i].x - middle ) < half )
				{
					fill[b] += m / ( 2.0 * half * binDepth );
				}
				if ( b >= 0 && b < bins && pos[i].x < left + strip )
				{
					edge[b] += m / ( strip * binDepth );
				}
			}
			printf( "  fill by depth below %.2f: depth middle wall\n", (double)surface );
			for ( int b = 0; b < bins; ++b )
			{
				if ( fill[b] > 0.0 )
				{
					printf( "  FILL %6.3f %.4f %.4f\n", (double)( ( b + 0.5f ) * binDepth ), fill[b], edge[b] );
				}
			}
		}
		if ( getenv( "LAB_SQUEEZE" ) )
		{
			// How far past rest each particle is squeezed, against its depth below the surface, by size
			const float* ps = particles.sizes;
			const float* rho = particles.densities;
			float rest = particles.restDensity;
			float surface = -1e9f;
			for ( int i = 0; i < n; ++i )
			{
				surface = surface > pos[i].y ? surface : (float)pos[i].y;
			}
			enum { bins = 48 };
			double binSum[3][bins] = { { 0 } }, sum2[3][bins] = { { 0 } };
			int count[3][bins] = { { 0 } };
			float binDepth = 0.5f;
			for ( int i = 0; i < n; ++i )
			{
				int b = (int)( ( surface - pos[i].y ) / binDepth );
				float r = ps[i] / def.fluid.particleSpacing;
				int c = r < 1.5f ? 0 : r < 3.0f ? 1 : 2;
				if ( b >= 0 && b < bins )
				{
					double q = rho[i] / rest - 1.0;
					binSum[c][b] += q;
					sum2[c][b] += q * q;
					count[c][b] += 1;
				}
			}
			if ( getenv( "LAB_SQUEEZE_DUMP" ) )
			{
				FILE* dump = fopen( getenv( "LAB_SQUEEZE_DUMP" ), "w" );
				for ( int i = 0; dump != NULL && i < n; ++i )
				{
					fprintf( dump, "%g %g %g\n", (double)( surface - pos[i].y ), (double)( ps[i] / def.fluid.particleSpacing ), (double)( rho[i] / rest - 1.0f ) );
				}
				if ( dump != NULL )
				{
					fclose( dump );
				}
			}
			printf( "  squeeze by depth (top particle at %.2f): depth | fine <1.5 mean sd n | mid 1.5-3 | coarse >3\n", (double)surface );
			for ( int b = 0; b < bins; ++b )
			{
				if ( count[0][b] + count[1][b] + count[2][b] == 0 )
				{
					continue;
				}
				printf( "  SQ %5.2f", (double)( ( b + 0.5f ) * binDepth ) );
				for ( int c = 0; c < 3; ++c )
				{
					if ( count[c][b] > 0 )
					{
						double m = binSum[c][b] / count[c][b], v = sum2[c][b] / count[c][b] - m * m;
						printf( " | %7.4f %6.4f %5d", m, sqrt( v > 0.0 ? v : 0.0 ), count[c][b] );
					}
					else
					{
						printf( " |       -      -     0" );
					}
				}
				printf( "\n" );
			}
		}
		fflush( stdout );

		if ( settle >= 0.0f )
		{
			for ( int k = 0; k < scene.bodyCount; ++k )
			{
				if ( b2Body_GetType( scene.bodies[k] ) != b2_dynamicBody )
				{
					continue;
				}
				b2Pos p = b2Body_GetPosition( scene.bodies[k] );
				float settledFrames = lateFrames > 0 ? (float)lateFrames : 1.0f;
				printf( "body %d mass %.3f: peak %.2f | settled mean %.3f peak %.3f spin %.3f | y %.2f highest %.2f | bottom %.3f-%.3f under %.1f\n", k,
						(double)b2Body_GetMass( scene.bodies[k] ), (double)peakSpeed[k], (double)( lateMean[k] / settledFrames ), (double)latePeak[k],
						(double)latePeakSpin[k], (double)p.y, (double)peakY[k], (double)lowBottom[k], (double)highBottom[k], (double)( under[k] / settledFrames ) );
			}
		}

		if ( getenv( "LAB_TRACE" ) )
		{
			for ( int i = 0; i < scene.bodyCount; ++i )
			{
				if ( b2Body_GetType( scene.bodies[i] ) == b2_staticBody )
				{
					continue;
				}
				b2Pos p = b2Body_GetPosition( scene.bodies[i] );
				b2Vec2 v = b2Body_GetLinearVelocity( scene.bodies[i] );
				printf( "  body %d: x=%.2f y=%.2f vx=%.2f vy=%.2f w=%.2f angle=%.1f awake=%d\n", i, (double)p.x, (double)p.y, (double)v.x, (double)v.y,
						(double)b2Body_GetAngularVelocity( scene.bodies[i] ), (double)( b2Rot_GetAngle( b2Body_GetRotation( scene.bodies[i] ) ) * 57.3f ),
						b2Body_IsAwake( scene.bodies[i] ) );
			}
		}

		b2DestroyFluid( fluid );
		b2DestroyWorld( worldId );
		if ( oneRun )
		{
			break;
		}
	}
	if ( check && failures > 0 )
	{
		printf( "FAILED: %d runs differed from the first or leaked\n", failures );
		return 1;
	}
	return 0;
}

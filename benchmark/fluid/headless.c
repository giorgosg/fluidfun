// SPDX-License-Identifier: MIT
// Runs the shared scene with no window. Prints timing and a hash of the final state, so that
// runs can be compared across worker counts. Usage: fluid_headless [frames] [liquid scale]
//
// The environment picks the run: LAB_SCENE (number from scene.h), LAB_CONTROL, LAB_CRATES, LAB_SUBSTEPS,
// LAB_ONE (6 workers only), LAB_CHECK (fail on differing hashes or leaks), and fluid settings, see
// ApplySettings. Probes print extra lines: LAB_LEVEL, LAB_CHANGES, LAB_BODIES, LAB_EVENTS, LAB_TRACE,
// LAB_SQUEEZE, LAB_SQUEEZE_DUMP and LAB_FRAMES with LAB_FRAME_EVERY.
#include "scene.h"

#include "box2d/math_functions.h"

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
	b2ShapeId shapes[32];
	int count = b2Body_GetShapes( body, shapes, 32 );
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

static void WriteFrame( const char* path, b2fFluid* fluid, const LabScene* scene )
{
	memset( image, 24, sizeof( image ) );
	// Tall scenes are fitted by height, wide ones by width
	float zoom = getenv( "LAB_ZOOM" ) ? (float)atof( getenv( "LAB_ZOOM" ) ) : 1.0f;
	viewScale = (float)imageWidth / ( 2.0f * zoom * scene->viewHalfWidth );
	viewCenter = scene->viewCenter;

	int n = b2fFluid_GetParticleCount( fluid );
	const b2fCoord* x = b2fFluid_GetPositionsX( fluid );
	const b2fCoord* y = b2fFluid_GetPositionsY( fluid );
	for ( int i = 0; i < n; ++i )
	{
		int px = (int)( 0.5f * imageWidth + ( x[i] - viewCenter.x ) * viewScale );
		int py = (int)( 0.5f * imageHeight - ( y[i] - viewCenter.y ) * viewScale );
		PlotPixel( px, py, 60, 140, 255 );
		PlotPixel( px + 1, py, 60, 140, 255 );
		PlotPixel( px, py + 1, 60, 140, 255 );
		PlotPixel( px + 1, py + 1, 60, 140, 255 );
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
static void ApplySettings( b2fFluidDef* def )
{
	struct
	{
		const char* name;
		float* value;
	} settings[] = {
		{ "LAB_SPACING", &def->particleSpacing },
		{ "LAB_CONTACT", &def->contactPressure },
		{ "LAB_VISCOSITY", &def->linearViscosity },
		{ "LAB_QUADRATIC", &def->quadraticViscosity },
		{ "LAB_ADAPTIVE", &def->adaptiveRatio },
		{ "LAB_ADAPTIVE_DEPTH", &def->adaptiveDepth },
		{ "LAB_FAST", &def->adaptiveSpeed },
		{ "LAB_FULL", &def->adaptiveFullBox },
		{ "LAB_TOLERANCE", &def->adaptiveTolerance },
		{ "LAB_LIMIT", &def->adaptiveLimit },
		{ "LAB_RATE", &def->adaptiveRate },
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
		def->stiffness = (float)atof( getenv( "LAB_STIFFNESS" ) );
		def->nearStiffness = 2.5f * def->stiffness;
	}
	if ( getenv( "LAB_DEPTH_LIMIT" ) )
	{
		def->adaptiveDepthLimit = atoi( getenv( "LAB_DEPTH_LIMIT" ) ) != 0;
	}
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
		scene.liquidWidth *= scale;
		scene.tankWidth *= scale;
		scene.liquidHeight *= scale;
		scene.tankHeight *= scale;
		b2fFluidDef def = b2fDefaultFluidDef();
		def.capacity = 2000000;
		ApplySettings( &def );
		b2fFluid* fluid = LabCreateScene( worldId, &scene, &def );

		uint64_t start = b2GetTicks();
		b2fProfile sum = { 0 };
		double measureSum = 0.0;
		for ( int i = 0; i < frames; ++i )
		{
			LabStepScene( &scene, fluid, 1.0f / 60.0f );
			b2fFluid_Prepare( fluid );
			b2World_Step( worldId, 1.0f / 60.0f, getenv( "LAB_SUBSTEPS" ) ? atoi( getenv( "LAB_SUBSTEPS" ) ) : 4 );
			b2fProfile p = b2fFluid_GetProfile( fluid );
			measureSum += p.measure;
			if ( getenv( "LAB_LEVEL" ) && ( i < 60 ? i % 10 == 0 : i % 120 == 0 ) )
			{
				// Mass weighted height of the liquid: lower means it packed tighter or left a gap
				int n = b2fFluid_GetParticleCount( fluid );
				const b2fCoord* py = b2fFluid_GetPositionsY( fluid );
				const float* ps = b2fFluid_GetSizes( fluid );
				double my = 0.0, m = 0.0, fineY = 0.0, coarseY = 0.0;
				int fine = 0, coarse = 0;
				for ( int k = 0; k < n; ++k )
				{
					double w = (double)ps[k] * ps[k];
					my += w * py[k];
					m += w;
					float r = ps[k] / def.particleSpacing;
					if ( r < 1.5f )
					{
						fineY += py[k];
						fine += 1;
					}
					else if ( r > 3.0f )
					{
						coarseY += py[k];
						coarse += 1;
					}
				}
				printf( "LV %d %.4f n=%d fine %d at y %.2f, coarse %d at y %.2f\n", i, my / m, n, fine, fine > 0 ? fineY / fine : 0.0,
						coarse, coarse > 0 ? coarseY / coarse : 0.0 );
			}
			sum.sort += p.sort;
			sum.viscosity += p.viscosity;
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
			if ( getenv( "LAB_CHANGES" ) )
			{
				printf( "CH %d\n", p.sizeChanges );
			}

			if ( getenv( "LAB_BODIES" ) && i % 120 == 0 )
			{
				printf( "T %.0f", (double)i / 60.0 );
				for ( int k = 0; k < scene.bodyCount; ++k )
				{
					if ( b2Body_GetType( scene.bodies[k] ) != b2_staticBody )
					{
						printf( " | x=%.1f vx=%.2f w=%.2f", (double)b2Body_GetPosition( scene.bodies[k] ).x, (double)b2Body_GetLinearVelocity( scene.bodies[k] ).x, (double)b2Body_GetAngularVelocity( scene.bodies[k] ) );
					}
				}
				printf( "\n" );
			}

			if ( getenv( "LAB_EVENTS" ) && i % 6 == 0 )
			{
				const b2fCoord* ex = b2fFluid_GetPositionsX( fluid );
				const b2fCoord* ey = b2fFluid_GetPositionsY( fluid );
				const float* evx = b2fFluid_GetVelocitiesX( fluid );
				const float* evy = b2fFluid_GetVelocitiesY( fluid );
				int count = b2fFluid_GetParticleCount( fluid );
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
				printf( "EV %.2f rms=%.3f top=%.2f x=%.2f y=%.2f N=%d\n", (double)i / 60.0, sqrt( e / count ), sqrt( best ), (double)ex[bi], (double)ey[bi], count );
			}

			if ( getenv( "LAB_TRACE" ) && i % 30 == 0 )
			{
				const b2fCoord* tx = b2fFluid_GetPositionsX( fluid );
				const b2fCoord* ty = b2fFluid_GetPositionsY( fluid );
				const float* tvx = b2fFluid_GetVelocitiesX( fluid );
				const float* tvy = b2fFluid_GetVelocitiesY( fluid );
				int count = b2fFluid_GetParticleCount( fluid );
				float maxSpeed = 0.0f, minY = 1e9f, maxY = -1e9f, minX = 1e9f, maxX = -1e9f;
				for ( int k = 0; k < count; ++k )
				{
					float v = tvx[k] * tvx[k] + tvy[k] * tvy[k];
					maxSpeed = maxSpeed > v ? maxSpeed : v;
					minY = minY < ty[k] ? minY : (float)ty[k];
					maxY = maxY > ty[k] ? maxY : (float)ty[k];
					minX = minX < tx[k] ? minX : (float)tx[k];
					maxX = maxX > tx[k] ? maxX : (float)tx[k];
				}
				printf( "  frame %d: %.1f ms x[%.1f %.1f] y[%.2f %.2f] vmax %.1f\n", i, p.total, minX, maxX, minY, maxY, sqrtf( maxSpeed ) );
				fflush( stdout );
			}
		}
		float ms = b2GetMilliseconds( start ) / (float)frames;

		int n = b2fFluid_GetParticleCount( fluid );
		uint64_t hash = 14695981039346656037ull;
		hash = HashBytes( hash, b2fFluid_GetPositionsX( fluid ), (size_t)n * sizeof( b2fCoord ) );
		hash = HashBytes( hash, b2fFluid_GetPositionsY( fluid ), (size_t)n * sizeof( b2fCoord ) );
		hash = HashBytes( hash, b2fFluid_GetVelocitiesX( fluid ), (size_t)n * sizeof( float ) );
		hash = HashBytes( hash, b2fFluid_GetVelocitiesY( fluid ), (size_t)n * sizeof( float ) );

		float top = -1e9f;
		const b2fCoord* y = b2fFluid_GetPositionsY( fluid );
		for ( int i = 0; i < n; ++i )
		{
			top = top > y[i] ? top : (float)y[i];
		}

		float f = (float)frames;
		printf( "workers=%2d n=%d %6.2f ms/frame | fluid %6.2f (sort %.2f visc %.2f density %.2f collide %.2f) top=%.2f hash=%016llx\n",
				workerCounts[w], n, ms, sum.total / f, sum.sort / f, sum.viscosity / f, sum.density / f, sum.collide / f, top,
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
			printf( "  pumped %d mass %.0f stall %.1fs\n", scene.pumped, (double)b2fFluid_GetMovedMass( fluid ), (double)scene.longestStall / 60.0 );
		}
		{
			const float* size = b2fFluid_GetSizes( fluid );
			int bins[6] = { 0 };
			for ( int i = 0; i < n; ++i )
			{
				float ratio = size[i] / def.particleSpacing;
				int bin = ratio < 1.1f ? 0 : ratio < 1.3f ? 1 : ratio < 1.6f ? 2 : ratio < 2.1f ? 3 : ratio < 3.0f ? 4 : 5;
				bins[bin] += 1;
			}
			printf( "  sizes over fine: <1.1 %d%%, 1.1-1.3 %d%%, 1.3-1.6 %d%%, 1.6-2.1 %d%%, 2.1-3 %d%%, >3 %d%%\n", 100 * bins[0] / n, 100 * bins[1] / n,
					100 * bins[2] / n, 100 * bins[3] / n, 100 * bins[4] / n, 100 * bins[5] / n );
		}
		if ( getenv( "LAB_SQUEEZE" ) )
		{
			// How far past rest each particle is squeezed, against its depth below the surface, by size
			const b2fCoord* py = b2fFluid_GetPositionsY( fluid );
			const float* ps = b2fFluid_GetSizes( fluid );
			const float* rho = b2fFluid_GetDensities( fluid );
			float rest = b2fFluid_GetRestDensity( fluid );
			float surface = -1e9f;
			for ( int i = 0; i < n; ++i )
			{
				surface = surface > py[i] ? surface : (float)py[i];
			}
			enum { bins = 48 };
			double sum[3][bins] = { { 0 } }, sum2[3][bins] = { { 0 } };
			int count[3][bins] = { { 0 } };
			float binDepth = 0.5f;
			for ( int i = 0; i < n; ++i )
			{
				int b = (int)( ( surface - py[i] ) / binDepth );
				float r = ps[i] / def.particleSpacing;
				int c = r < 1.5f ? 0 : r < 3.0f ? 1 : 2;
				if ( b >= 0 && b < bins )
				{
					double q = rho[i] / rest - 1.0;
					sum[c][b] += q;
					sum2[c][b] += q * q;
					count[c][b] += 1;
				}
			}
			if ( getenv( "LAB_SQUEEZE_DUMP" ) )
			{
				FILE* dump = fopen( getenv( "LAB_SQUEEZE_DUMP" ), "w" );
				for ( int i = 0; dump != NULL && i < n; ++i )
				{
					fprintf( dump, "%g %g %g\n", (double)( surface - py[i] ), (double)( ps[i] / def.particleSpacing ), (double)( rho[i] / rest - 1.0f ) );
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
						double m = sum[c][b] / count[c][b], v = sum2[c][b] / count[c][b] - m * m;
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

		b2fDestroyFluid( fluid );
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

// SPDX-License-Identifier: MIT
// Settles a column of liquid and reports how it holds up, one line per run.
// Usage: fluid_compare stiffness depth [substeps] [seconds] [workers]
// Near stiffness is 2.5 times stiffness.
#include "box2d/fluidfun.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int CompareFloats( const void* a, const void* b )
{
	float x = *(const float*)a, y = *(const float*)b;
	return x < y ? -1 : ( x > y ? 1 : 0 );
}

static void AddWall( b2BodyId body, float x, float y, float halfWidth, float halfHeight )
{
	b2Polygon box = b2MakeOffsetBox( halfWidth, halfHeight, (b2Vec2){ x, y }, b2Rot_identity );
	b2ShapeDef def = b2DefaultShapeDef();
	b2CreatePolygonShape( body, &def, &box );
}

int main( int argc, char** argv )
{
	if ( argc < 3 )
	{
		puts( "fluid_compare stiffness depth [substeps] [seconds] [workers]" );
		return 1;
	}

	float param = (float)atof( argv[1] ), depth = (float)atof( argv[2] );
	int substeps = argc > 3 ? atoi( argv[3] ) : 4;
	float seconds = argc > 4 ? (float)atof( argv[4] ) : 20.0f;
	int workers = argc > 5 ? atoi( argv[5] ) : 4;

	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = workers;
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId tank = b2CreateBody( worldId, &bodyDef );
	float halfWidth = 6.0f, wallHeight = 3.0f * depth + 2.0f;
	AddWall( tank, 0.0f, -1.0f, halfWidth + 2.0f, 1.0f );
	AddWall( tank, -halfWidth - 1.0f, wallHeight, 1.0f, wallHeight + 2.0f );
	AddWall( tank, halfWidth + 1.0f, wallHeight, 1.0f, wallHeight + 2.0f );

	b2FluidDef def = b2DefaultFluidDef();
	b2LiquidDef liquid = b2DefaultLiquidDef();
	liquid.tuning.stiffness = param;
	liquid.tuning.nearStiffness = 2.5f * param;
	if ( getenv( "LAB_VISCOSITY" ) )
	{
		liquid.viscosity = (float)atof( getenv( "LAB_VISCOSITY" ) );
	}
	if ( getenv( "LAB_QUADRATIC" ) )
	{
		liquid.tuning.quadraticViscosity = (float)atof( getenv( "LAB_QUADRATIC" ) );
	}
	if ( getenv( "LAB_NEAR" ) )
	{
		liquid.tuning.nearStiffness = (float)atof( getenv( "LAB_NEAR" ) );
	}

	if ( getenv( "LAB_SPACING" ) )
	{
		def.particleSpacing = (float)atof( getenv( "LAB_SPACING" ) );
		def.capacity = 1000000;
	}
	if ( getenv( "LAB_ADAPTIVE_DEPTH" ) )
	{
		def.adaptiveDepth = (float)atof( getenv( "LAB_ADAPTIVE_DEPTH" ) );
	}
	if ( getenv( "LAB_ADAPTIVE" ) )
	{
		liquid.tuning.adaptiveRatio = (float)atof( getenv( "LAB_ADAPTIVE" ) );
	}

	b2FluidId fluid = b2CreateFluid( worldId, &def );
	b2LiquidId liquidId = b2CreateLiquid( fluid, &liquid );
	float spacing = def.particleSpacing;
	b2Liquid_CreateParticlesInBox( liquidId, (b2Pos){ -halfWidth + 0.5f * spacing, 0.5f * spacing },
								   (b2Pos){ halfWidth - 0.5f * spacing, depth } );
	int n = b2Fluid_GetParticleCount( fluid );
	double startArea = 0.0;
	{
		const float* size = b2Fluid_GetParticles( fluid ).sizes;
		for ( int i = 0; i < n; ++i )
		{
			startArea += (double)size[i] * size[i];
		}
	}
	float idealHeight = (float)( startArea * 0.8660254f / ( 2.0f * halfWidth ) );
	int startCount = n;

	int frames = (int)( seconds * 60.0f );
	double rms = 0.0, peak = 0.0;
	int samples = 0;
	uint64_t start = b2GetTicks();
	bool blown = false;
	for ( int frame = 0; frame < frames && blown == false; ++frame )
	{
		b2World_Step( worldId, 1.0f / 60.0f, substeps );
		n = b2Fluid_GetParticleCount( fluid );
		if ( getenv( "LAB_KEEP_GOING" ) && frame % 120 == 0 )
		{
			printf( "  t=%.0fs fluid %.1f ms\n", (double)frame / 60.0, (double)b2Fluid_GetProfile( fluid ).total );
			fflush( stdout );
		}

		b2FluidParticles particles = b2Fluid_GetParticles( fluid );
		const b2Pos* pos = particles.positions;
		const float* vx = particles.velocitiesX;
		const float* vy = particles.velocitiesY;
		double sum = 0.0, top = 0.0;
		for ( int i = 0; i < n; ++i )
		{
			double v2 = vx[i] * vx[i] + vy[i] * vy[i];
			sum += v2;
			top = v2 > top ? v2 : top;
			if ( ( isfinite( pos[i].y ) == false || pos[i].y > 3.0f * depth + 10.0f ) && getenv( "LAB_KEEP_GOING" ) == NULL )
			{
				blown = true;
			}
		}

		if ( getenv( "LAB_ONSET" ) && frame % 30 == 0 )
		{
			double fastest = 0.0;
			float fastX = 0.0f, fastY = 0.0f;
			for ( int i = 0; i < n; ++i )
			{
				double v2 = vx[i] * vx[i] + vy[i] * vy[i];
				if ( v2 > fastest )
				{
					fastest = v2;
					fastX = (float)pos[i].x;
					fastY = (float)pos[i].y;
				}
			}
			printf( "  t=%.1f rms=%.2f fastest %.1f at x=%.1f y=%.1f\n", (double)frame / 60.0, sqrt( sum / n ), sqrt( fastest ), (double)fastX, (double)fastY );
		}

		if ( getenv( "LAB_SERIES" ) && frame % 300 == 0 )
		{
			printf( "  t=%.0f N=%d rms=%.2f\n", (double)frame / 60.0, n, sqrt( sum / n ) );
			fflush( stdout );
		}

		if ( frame >= frames * 3 / 4 )
		{
			rms += sqrt( sum / n );
			peak += sqrt( top );
			samples += 1;
		}
	}
	float ms = b2GetMilliseconds( start ) / (float)frames;

	if ( blown )
	{
		printf( "stiffness=%.3f depth=%.0f sub=%d BLEW UP\n", param, depth, substeps );
	}
	else
	{
		// Height as the median of the surface across columns of the tank
		enum { binCount = 48 };
		float tops[binCount] = { 0 };
		b2FluidParticles particles = b2Fluid_GetParticles( fluid );
		const b2Pos* pos = particles.positions;
		for ( int i = 0; i < n; ++i )
		{
			int bin = (int)( ( pos[i].x + halfWidth ) / ( 2.0f * halfWidth ) * binCount );
			bin = bin < 0 ? 0 : ( bin >= binCount ? binCount - 1 : bin );
			tops[bin] = tops[bin] > pos[i].y ? tops[bin] : (float)pos[i].y;
		}
		qsort( tops, binCount, sizeof( float ), CompareFloats );
		float height = tops[binCount / 2] + 0.43f * spacing;
		// Density of a slice near the bottom and near the top, from the particle count per unit height
		if ( getenv( "LAB_PROFILE" ) )
		{
			float slice[6] = { 0 };
			float bandHeight = height / 6.0f;
			for ( int i = 0; i < n; ++i )
			{
				int band = (int)( pos[i].y / bandHeight );
				band = band < 0 ? 0 : ( band > 5 ? 5 : band );
				slice[band] += 1.0f;
			}
			printf( "  layers per ideal, bottom to top:" );
			for ( int i = 0; i < 6; ++i )
			{
				printf( " %.2f", slice[i] * 0.8660254f * spacing * spacing / ( 2.0f * halfWidth * bandHeight ) );
			}
			printf( "\n" );
		}

		// Nearest neighbour distance in spacings, from a sample of particles, shows pairing up.
		enum { sampleCount = 400 };
		float nearest[sampleCount];
		int taken = 0;
		for ( int i = 0; i < n && taken < sampleCount; i += n / sampleCount + 1 )
		{
			float best = 1e9f;
			for ( int j = 0; j < n; ++j )
			{
				float dx = (float)( pos[j].x - pos[i].x ), dy = (float)( pos[j].y - pos[i].y ), d2 = dx * dx + dy * dy;
				if ( j != i && d2 < best )
				{
					best = d2;
				}
			}
			nearest[taken++] = sqrtf( best ) / spacing;
		}
		qsort( nearest, taken, sizeof( float ), CompareFloats );
		printf( "nearest p5=%.2f median=%.2f p95=%.2f | ", nearest[taken / 20], nearest[taken / 2], nearest[taken * 19 / 20] );

		{
			const float* size = particles.sizes;
			double area = 0.0;
			float smallest = 1e9f, largest = 0.0f;
			for ( int i = 0; i < n; ++i )
			{
				area += (double)size[i] * size[i];
				smallest = smallest < size[i] ? smallest : size[i];
				largest = largest > size[i] ? largest : size[i];
			}
			printf( "start N=%d mass drift %.2e sizes %.3f-%.3f | ", startCount, area / startArea - 1.0, smallest, largest );
		}
		printf( "stiffness=%.3f depth=%.0f sub=%d N=%d height/ideal=%.3f rms=%.3f peak=%.2f ms=%.1f\n", param, depth,
				substeps, n, height / idealHeight, rms / samples, peak / samples, ms );
	}

	b2DestroyFluid( fluid );
	b2DestroyWorld( worldId );
	return 0;
}

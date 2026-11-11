// SPDX-License-Identifier: MIT
// Adding and moving particles.
#include "fluid_internal.h"

// Liquid spawns 1.09 of the solver's spacings apart, which is 0.97 of particleSpacing. A settled pool is that much
// tighter than liquid at rest on average, from its own weight: spawned at particleSpacing, 10 and 25 m pools sank by
// 3 and 5%, and spawned here they rise by 3 and 1%. Much tighter than this holds pressure, which a free blob
// releases all at once, tearing open a void in its middle.
static float SpawnSpacing( const b2fFluid* f )
{
	return B2F_SPAWN_SPACING * f->spacing;
}

static bool AddParticle( b2fFluid* f, int liquid, b2fCoord x, b2fCoord y, float size )
{
	if ( f->count == f->capacity )
	{
		return false;
	}
	int i = f->count++;
	b2fClearParticle( f, i );
	f->pos[i].x = x;
	f->pos[i].y = y;
	f->id[i] = f->nextParticleId++;
	f->size[i] = size;
	f->liquid[i] = (unsigned char)liquid;
	return true;
}

// With adaptive sizes particles start at the size they would settle to, so a pool does not begin by merging all
// over. Density boxes read depth as distance to whatever is not liquid, walls and floor included, so this grades
// by distance to the nearest edge of the box: rectangular rings from the edge inwards, each as coarse as its
// distance asks, with its particles spread evenly around it.
static int CreateGradedParticles( b2fFluid* f, int liquid, b2Pos lower, b2Pos upper )
{
	const b2LiquidDef* def = &f->liquids[liquid].def;
	float spacing = f->spacing, ratio = def->tuning.adaptiveRatio;
	float g = -def->gravityScale * b2World_GetGravity( f->worldId ).y;
	float reach = b2fAdaptiveReach( f, liquid, g, f->h > 0.0f ? f->h : 1.0f / 240.0f );
	b2fCoord cx = 0.5f * ( lower.x + upper.x ), cy = 0.5f * ( lower.y + upper.y );
	int added = 0, ring = 0;
	float inset = 0.0f;
	while ( true )
	{
		float a = (float)( 0.5f * ( upper.x - lower.x ) ) - inset, b = (float)( 0.5f * ( upper.y - lower.y ) ) - inset;
		if ( a <= 0.0f || b <= 0.0f )
		{
			break;
		}

		float t = inset / reach;
		t = t < 1.0f ? t : 1.0f;
		float pitch = B2F_SPAWN_SPACING * spacing * sqrtf( 1.0f + ( ratio * ratio - 1.0f ) * t );
		float dy = 0.8660254f * pitch;

		// What is left is too thin for a ring, so it gets one line down its middle
		if ( a < dy || b < dy )
		{
			bool across = a > b;
			float length = 2.0f * ( across ? a : b );
			int count = (int)( length / pitch + 0.5f );
			count = count > 1 ? count : 1;
			float step = length / (float)count, size = sqrtf( step * 2.0f * ( across ? b : a ) / 0.8660254f ) / B2F_SPAWN_SPACING;
			for ( int i = 0; i < count; ++i )
			{
				float u = -0.5f * length + ( (float)i + 0.5f ) * step;
				if ( AddParticle( f, liquid, across ? cx + u : cx, across ? cy : cy + u, size ) == false )
				{
					return added;
				}
				added += 1;
			}
			break;
		}

		// Walk the ring's middle line, from the lower left corner anticlockwise
		float ha = a - 0.5f * dy, hb = b - 0.5f * dy;
		float perimeter = 4.0f * ( ha + hb );
		int count = (int)( perimeter / pitch + 0.5f );
		count = count > 4 ? count : 4;
		float step = perimeter / (float)count;
		float size = sqrtf( step * dy / 0.8660254f ) / B2F_SPAWN_SPACING;
		for ( int i = 0; i < count; ++i )
		{
			float u = ( (float)i + ( ring & 1 ? 0.5f : 0.0f ) ) * step;
			b2fCoord x, y;
			if ( u < 2.0f * ha )
			{
				x = cx - ha + u, y = cy - hb;
			}
			else if ( u < 2.0f * ( ha + hb ) )
			{
				x = cx + ha, y = cy - hb + ( u - 2.0f * ha );
			}
			else if ( u < 2.0f * ( 2.0f * ha + hb ) )
			{
				x = cx + ha - ( u - 2.0f * ( ha + hb ) ), y = cy + hb;
			}
			else
			{
				x = cx - ha, y = cy + hb - ( u - 2.0f * ( 2.0f * ha + hb ) );
			}

			if ( AddParticle( f, liquid, x, y, size ) == false )
			{
				return added;
			}
			added += 1;
		}
		inset += dy;
		ring += 1;
	}
	return added;
}

int b2Liquid_CreateParticlesInBox( b2LiquidId liquidId, b2Pos lower, b2Pos upper )
{
	b2fFluid* f;
	int liquid = b2fGetLiquid( liquidId, &f );

	if ( f->liquids[liquid].def.tuning.adaptiveRatio > 1.0f )
	{
		return CreateGradedParticles( f, liquid, lower, upper );
	}

	float spacing = SpawnSpacing( f );
	float rowSpacing = 0.8660254f * spacing;
	int added = 0;
	int row = 0;
	for ( b2fCoord y = lower.y; y <= upper.y; y += rowSpacing, ++row )
	{
		for ( b2fCoord x = lower.x + ( row & 1 ? 0.5f * spacing : 0.0f ); x <= upper.x; x += spacing )
		{
			if ( AddParticle( f, liquid, x, y, f->spacing ) == false )
			{
				return added;
			}
			added += 1;
		}
	}
	return added;
}

int b2Fluid_MoveParticles( b2FluidId fluidId, b2AABB from, b2AABB to, b2Vec2 velocity, int maxCount )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	float spacing = SpawnSpacing( f );
	int perRow = (int)( ( to.upperBound.x - to.lowerBound.x ) / spacing ) + 1;
	int moved = 0;
	for ( int i = 0; i < f->count && moved < maxCount; ++i )
	{
		if ( f->pos[i].x < from.lowerBound.x || f->pos[i].x > from.upperBound.x || f->pos[i].y < from.lowerBound.y || f->pos[i].y > from.upperBound.y )
		{
			continue;
		}

		f->pos[i].x = to.lowerBound.x + spacing * (float)( moved % perRow );
		f->pos[i].y = to.lowerBound.y + spacing * (float)( moved / perRow );
		f->vx[i] = velocity.x;
		f->vy[i] = velocity.y;
		float ratio = f->size[i] / f->spacing;
		f->movedMass += f->liquids[f->liquid[i]].mass * ratio * ratio;
		moved += 1;
	}
	return moved;
}

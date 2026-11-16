// SPDX-License-Identifier: MIT
// Pushing the liquid between steps: a change of velocity or a blend toward a velocity in a region, and explosions.
//
// Each of these only writes particle velocities, f->vx and f->vy. Between steps those are the particles' velocities,
// and the next step starts from them: each substep's prediction adds gravity to them and moves the particle by them,
// and the end of the substep rewrites them from how far the particle went, after density, springs and shapes have
// moved it. Springs and density act on positions, so a liquid with springs carries the push the same way, with its
// springs pulling it back afterwards. The size pass before the step passes velocity along with mass when particles
// merge and split, so a push is kept through that too. A particle pushed wakes, and its neighbours with it once it
// moves (fluid_sleep.c).
#include "fluid_internal.h"

#include "core.h"
#include "fluid.h"
#include "physics_world.h"
#include "recording.h"

typedef struct Push
{
	// Added to the velocity, then the velocity moved toward target by share
	b2Vec2 change, target;
	float share;
} Push;

// Pushes the particles a call picks, in one pass
static int PushInside( b2fFluid* f, int liquid, const b2fOutline* outline, Push push )
{
	int pushed = 0;
	float keep = 1.0f - push.share;
	float targetX = push.share * push.target.x, targetY = push.share * push.target.y;
	for ( int i = 0; i < f->count; ++i )
	{
		if ( b2fPicks( f, liquid, outline, i ) == false )
		{
			continue;
		}
		// keep v + share t rather than v + share ( t - v ), so that a share of 1 sets the velocity exactly
		f->vx[i] = keep * ( f->vx[i] + push.change.x ) + targetX;
		f->vy[i] = keep * ( f->vy[i] + push.change.y ) + targetY;
		b2fWakeParticle( f, i );
		pushed += 1;
	}
	return pushed;
}

static int PushInRegion( b2FluidId fluidId, b2LiquidId liquidId, b2FluidRegion region, Push push, b2fFluid** fluid )
{
	*fluid = NULL;
	b2fFluid* f = b2fGetFluid( fluidId );
	if ( b2fWorldLocked( fluidId ) )
	{
		return 0;
	}

	int liquid = b2fPickLiquid( f, liquidId );
	b2fOutline outline;
	if ( b2fMakeRegionOutline( &region, &outline ) == false )
	{
		return 0;
	}
	*fluid = f;
	return PushInside( f, liquid, &outline, push );
}

int b2Fluid_Accelerate( b2FluidId fluidId, b2LiquidId liquidId, b2FluidRegion region, b2Vec2 velocityChange )
{
	B2_CHECK_INPUT_RETURN( b2IsValidVec2( velocityChange ), 0 );
	b2fFluid* f;
	int pushed = PushInRegion( fluidId, liquidId, region, ( Push ){ velocityChange, b2Vec2_zero, 0.0f }, &f );
	if ( f != NULL )
	{
		B2_REC_CREATE( f->world, FluidAccelerate, pushed, fluidId, liquidId, region, velocityChange );
	}
	return pushed;
}

int b2Fluid_BlendVelocity( b2FluidId fluidId, b2LiquidId liquidId, b2FluidRegion region, b2Vec2 velocity, float share )
{
	B2_CHECK_INPUT_RETURN( b2IsValidVec2( velocity ) && b2IsValidFloat( share ), 0 );
	float clamped = b2ClampFloat( share, 0.0f, 1.0f );
	b2fFluid* f;
	int pushed = PushInRegion( fluidId, liquidId, region, ( Push ){ b2Vec2_zero, velocity, clamped }, &f );
	if ( f != NULL )
	{
		B2_REC_CREATE( f->world, FluidBlendVelocity, pushed, fluidId, liquidId, region, velocity, share );
	}
	return pushed;
}

// An explosion as Box2D gives one to a shape (ExplosionCallback in physics_world.c), for the liquid. Box2D finds the
// distance d from the centre to the shape and the direction from the centre to its closest point, and gives the body
// an impulse along that direction of
//
//   J = impulsePerLength * W * scale,  scale = 1 for d <= radius, ( radius + falloff - d ) / falloff beyond, 0 past it
//
// where W is the shape's width across that direction (b2GetShapeProjectedPerimeter). A body of width W and mass
// density rho, about W^2 rho in mass, so changes speed by about impulsePerLength * scale / ( rho W ).
//
// Taking W as each particle's own size would make the push grow as particles shrink: halving particleSpacing would
// throw the same pool twice as hard, and liquid would leave a blast far faster than a body of its size. So every
// particle instead changes speed as a body as wide as the blast would, with the length
//
//   L = radius + falloff / 2,  the distance at which the push has fallen to half,
//
//   dv = impulsePerLength * scale / ( rho L ),
//
// along the direction from the centre to the particle's centre (+x for a particle at the centre, as Box2D does for a
// shape the centre is inside). d is to the particle's centre, so nothing past radius + falloff moves. This depends on
// no particle size, so it is the same at any particleSpacing and with adaptive sizes, and a blob of liquid as wide as
// L is thrown as hard as a body as wide as L. A smaller blob, or a smaller body, is the case where the two differ: the
// body is thrown harder, by L over its width.
//
// The liquid filling the blast takes, summed over its area A with mass rho per unit area,
//
//   integral of rho dv dA = ( impulsePerLength / L ) * pi * ( radius^2 + radius falloff + falloff^2 / 3 )
//
// (the disc within radius at full push, plus the ring of the falloff, integral from 0 to falloff of
// ( 1 - u / falloff ) 2 pi ( radius + u ) du). A body filling the blast, 2 ( radius + falloff ) wide, takes
// impulsePerLength * 2 ( radius + falloff ). The liquid's over the body's is
//
//   pi ( radius^2 + radius falloff + falloff^2 / 3 ) / ( 2 ( radius + falloff ) ( radius + falloff / 2 ) ),
//
// 1.22 for falloff equal to radius, pi / 2 = 1.57 with no falloff, and pi / 3 = 1.05 with no radius. The liquid's
// impulse points out all round and the body's one way, so this compares how hard each is pushed, not net momentum.
//
// Box2D casts no shadows, and neither does this: every particle within reach gets its push, however much liquid is
// between it and the centre. A liquid with no density has no mass to push and is left alone.
int b2fExplode( b2fFluid* f, const b2ExplosionDef* def )
{
	float radius = def->radius, falloff = def->falloff, reach = radius + falloff;
	b2fOutline circle = b2fMakeCircleOutline( def->position, reach );

	// As Box2D throws the shapes whose category the mask includes, this throws those liquids
	uint32_t thrown = 0;
	for ( int l = 0; l < f->liquidCount; ++l )
	{
		thrown |= f->liquids[l].alive && ( f->liquids[l].def.categoryBits & def->maskBits ) != 0 ? 1u << l : 0u;
	}
	if ( reach <= 0.0f || thrown == 0 )
	{
		return 0;
	}

	float length = radius + 0.5f * falloff;
	int pushed = 0;
	for ( int i = 0; i < f->count; ++i )
	{
		if ( ( thrown >> f->liquid[i] & 1u ) == 0 || b2fInsideOutline( &circle, f->pos[i] ) == false )
		{
			continue;
		}

		float dx = (float)( f->pos[i].x - def->position.x ), dy = (float)( f->pos[i].y - def->position.y );
		float d2 = dx * dx + dy * dy, d = sqrtf( d2 );
		float scale = 1.0f;
		if ( d > radius && falloff > 0.0f )
		{
			scale = b2ClampFloat( ( reach - d ) / falloff, 0.0f, 1.0f );
		}
		float density = f->liquids[f->liquid[i]].def.density;
		if ( scale == 0.0f || density <= 0.0f )
		{
			continue;
		}

		b2Vec2 direction = { 1.0f, 0.0f };
		if ( d2 > 100.0f * FLT_EPSILON * FLT_EPSILON )
		{
			direction = ( b2Vec2 ){ dx / d, dy / d };
		}
		float change = def->impulsePerLength * scale / ( density * length );
		f->vx[i] += change * direction.x;
		f->vy[i] += change * direction.y;
		b2fWakeParticle( f, i );
		pushed += 1;
	}
	return pushed;
}

int b2Fluid_Explode( b2FluidId fluidId, const b2ExplosionDef* def )
{
	B2_CHECK_INPUT_RETURN( b2IsValidPos( def->position ), 0 );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->radius ) && def->radius >= 0.0f, 0 );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->falloff ) && def->falloff >= 0.0f, 0 );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->impulsePerLength ), 0 );

	b2fFluid* f = b2fGetFluid( fluidId );
	if ( b2fWorldLocked( fluidId ) )
	{
		return 0;
	}
	int pushed = b2fExplode( f, def );
	B2_REC_CREATE( f->world, FluidExplode, pushed, fluidId, *def );
	return pushed;
}

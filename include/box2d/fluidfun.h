// SPDX-License-Identifier: MIT
#pragma once

#include "box2d.h"

#ifdef __cplusplus
extern "C" {
#endif

/// The most liquids one fluid can hold
#define B2_MAX_LIQUIDS 16

/// Settings of one liquid. A fluid can hold several liquids, which share its particle spacing and grid but move by
/// their own settings. Liquids do not mix: each pulls only on itself unless b2Fluid_SetCohesionBetween says otherwise.
typedef struct b2LiquidDef
{
	/// Mass per unit area of the liquid at rest, to compare with b2ShapeDef::material density: a body lighter than
	/// this floats. Between liquids the lighter one takes more of each push, which makes it float.
	float massDensity;
	/// Displacement per substep per unit of density error, as a fraction of the interaction radius
	float stiffness;
	float nearStiffness;
	/// Share of the pull that pressure gives below rest density, from 0, where pressure only pushes, to 1, where it
	/// pulls as hard as it pushes. At 1 the pull is the liquid's only tension, as in Clavet's double density, and it
	/// can tear the bulk into holes. Lower values need cohesion to hold the liquid together. At 0, liquid with little
	/// cohesion and adaptive sizes never settles: gaps left by merging particles do not close, so particles split and
	/// merge over and over. The default of a half closes them without tearing holes.
	float tensilePressure;
	/// Pull between neighbours more than one spacing apart, and a push closer in, as its own term so that tension is
	/// set apart from stiffness. In the same units as stiffness. This is the pull within the liquid; the pull between
	/// two liquids is set with b2Fluid_SetCohesionBetween.
	float cohesion;
	/// Pull of static and kinematic shapes on the liquid, after Akinci et al. 2013: particles between half and all of
	/// the interaction radius from such a shape are drawn towards it. Bodies moved by forces do not pull. It helps a
	/// light body pressed onto the floor come back up. Zero turns it off. In the units of
	/// cohesion; above about 0.06 bodies slightly heavier than the liquid stop settling, and by 0.15 thin films creep
	/// up walls.
	float adhesion;
	/// Between two liquids viscosity is the mean of theirs
	float linearViscosity;
	float quadraticViscosity;
	/// Share of a particle's sliding speed along a shape lost in each substep it touches it, from 0, free slip, to
	/// 1, no slip. It does not grow with how hard the particle is pressed.
	float friction;
	float gravityScale;
	/// Share of its buoyancy a body is made up to where the particles' push falls short of it. Still liquid pushes on
	/// a body with the weight of the liquid it displaces. Particles cannot fill a gap narrower than a few of them, so a
	/// body touching a surface, or within three particle spacings of one, gets the missing push along the direction
	/// away from that surface. A nearly still body with liquid all the way round gets it up or down, since settled
	/// particles can otherwise hold it in place. At 1 a body lighter than the liquid lifts off the floor and rises, and
	/// a heavier one sinks and stays down. Zero turns both off.
	float contactPressure;
	/// Share of the liquid within reach of a body that moves with it when the body speeds up or slows down, from 0
	/// to 1. Without it a light body takes the liquid's push alone, so one much lighter than the liquid shoots out of
	/// it. The body and that liquid share each change of the body's motion as one lump, which keeps momentum, and a
	/// body floating or resting at a steady speed is not affected.
	float addedMass;
	/// Springs between neighbouring particles, which make the liquid elastic, after Clavet et al. 2005. Stiffness per
	/// unit mass in 1/s^2, so the square of the angular frequency of a spring holding one particle; zero turns springs
	/// off. A spring forms between two particles of this liquid closer than the interaction radius, at their distance
	/// then, and is written so that it acts the same whatever the substep count. It weakens as its rest length grows
	/// towards the interaction radius, and goes once it gets there.
	float springStiffness;
	/// How fast a spring's rest length follows its length once stretched or squeezed past the yield ratio, per second.
	/// Zero keeps every spring's rest length, which makes an elastic solid; higher values let the liquid flow and
	/// keep its new shape, as dough or clay does.
	float plasticity;
	/// Stretch or squeeze a spring takes, as a share of its rest length, before plasticity changes its rest length
	float yieldRatio;
	/// Spacing of the coarsest particles over the fluid's particleSpacing, which is the finest. One keeps this liquid
	/// fine everywhere. Particles are fine at the surface and coarser with depth, growing more gently away from a
	/// moving body than from anything else, and they split and merge as they move.
	/// Merges only join particles of the same liquid.
	float adaptiveRatio;
	/// Particles moving at this speed stay fine. They turn fine quickly and relax slowly. Zero turns this off.
	float adaptiveSpeed;
	/// Keep particles no smaller than the safe size for the weight of liquid above them, read from how squeezed
	/// they are, since walls and floors make liquid near them fine whatever its depth.
	bool adaptiveDepthLimit;
	/// A particle within this fraction of its target mass is left alone. Wider means fewer changes and sizes
	/// further from their targets.
	float adaptiveTolerance;
	/// A particle splits above this many times its target mass, and merges into its neighbours below one over it.
	float adaptiveLimit;
	/// Most mass a particle passes to its neighbours in one frame, as a fraction of its target. Merges and mass
	/// sharing happen over several frames at small values. One moves everything at once.
	float adaptiveRate;
} b2LiquidDef;

/// Settings the whole fluid shares, and its first liquid
typedef struct b2FluidDef
{
	/// How far apart particles of water-like liquid rest, away from the weight of deep liquid. The interaction radius
	/// is 1.78 times this. A liquid with much more cohesion than the default rests closer.
	float particleSpacing;
	/// How far from anything that is not liquid particles reach the coarsest size, in length units. Air, walls
	/// and bodies all count, so liquid is fine around each of them. Other liquids do not count. Zero or anything
	/// past the deepest safe value uses that value, which barely coarsens anything shallower than that.
	float adaptiveDepth;
	/// How full a box has to be, compared with liquid at rest, to count as full
	float adaptiveFullBox;
	int capacity;
	/// Liquid 0, which every fluid has
	b2LiquidDef liquid;
} b2FluidDef;

/// Timing of the last step in milliseconds, summed over substeps.
typedef struct b2FluidProfile
{
	float prepare;
	float sort;
	/// Density relaxation and viscosity, which share a pass
	float density;
	float collide;
	float total;
	/// Splits, merges and mass shares in the last step
	int sizeChanges;
	/// Part of prepare spent measuring depth, flow and distance to bodies
	float measure;
} b2FluidProfile;

B2_API b2FluidDef b2DefaultFluidDef( void );
B2_API b2LiquidDef b2DefaultLiquidDef( void );

/// Give a world its fluid, which holds all of its liquids. A world has at most one, and b2World_Step steps it with
/// the bodies. Returns b2_nullFluidId if the world already has one. Call between steps.
B2_API b2FluidId b2CreateFluid( b2WorldId worldId, const b2FluidDef* def );

/// Remove a world's fluid and every particle in it. Destroying the world does this too. Call between steps.
B2_API void b2DestroyFluid( b2FluidId fluidId );

/// Whether the id names a fluid that still exists
B2_API bool b2Fluid_IsValid( b2FluidId fluidId );

/// Change settings between steps, including liquid 0's. The spacing and capacity cannot change.
B2_API void b2Fluid_SetDef( b2FluidId fluidId, const b2FluidDef* def );

/// Add a liquid. Returns its index, or -1 if the fluid already holds B2_MAX_LIQUIDS.
B2_API int b2Fluid_CreateLiquid( b2FluidId fluidId, const b2LiquidDef* def );
/// Change a liquid's settings between steps
B2_API void b2Fluid_SetLiquid( b2FluidId fluidId, int liquid, const b2LiquidDef* def );
B2_API int b2Fluid_GetLiquidCount( b2FluidId fluidId );

/// Pull between particles of two different liquids, in the units of b2LiquidDef::cohesion. It starts at zero, so
/// liquids keep apart; a value near their own cohesion lets them cling to each other.
B2_API void b2Fluid_SetCohesionBetween( b2FluidId fluidId, int liquidA, int liquidB, float cohesion );

/// Fill a box with particles of one liquid on a triangular lattice. Returns how many were added. With adaptive
/// sizes particles start at the size their distance from the edge of the box asks for.
B2_API int b2Fluid_CreateParticlesInBox( b2FluidId fluidId, int liquid, b2Pos lowerBound, b2Pos upperBound );

/// Move up to maxCount particles found in the box from to a lattice in the box to, with this velocity.
/// Call between steps. This is how a scene recycles liquid into a stream. Returns how many moved.
B2_API int b2Fluid_MoveParticles( b2FluidId fluidId, b2AABB from, b2AABB to, b2Vec2 velocity, int maxCount );

/// The total mass moved by b2Fluid_MoveParticles so far.
B2_API float b2Fluid_GetMovedMass( b2FluidId fluidId );

B2_API int b2Fluid_GetParticleCount( b2FluidId fluidId );
/// World positions, in double when Box2D is built for large worlds
B2_API const b2Pos* b2Fluid_GetPositions( b2FluidId fluidId );
B2_API const float* b2Fluid_GetVelocitiesX( b2FluidId fluidId );
B2_API const float* b2Fluid_GetVelocitiesY( b2FluidId fluidId );

/// The spacing of each particle, in the units of particleSpacing. Mass is proportional to its square. The array is
/// filled by this call.
B2_API const float* b2Fluid_GetSizes( b2FluidId fluidId );
/// The liquid each particle belongs to
B2_API const unsigned char* b2Fluid_GetLiquids( b2FluidId fluidId );
/// Kernel density of each particle from the last substep, in the same units as b2Fluid_GetRestDensity
B2_API const float* b2Fluid_GetDensities( b2FluidId fluidId );
B2_API float b2Fluid_GetRestDensity( b2FluidId fluidId );
B2_API b2FluidProfile b2Fluid_GetProfile( b2FluidId fluidId );

#ifdef __cplusplus
}
#endif

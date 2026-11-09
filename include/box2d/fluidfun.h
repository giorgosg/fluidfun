// SPDX-License-Identifier: MIT
#pragma once

#include "box2d.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct b2fFluid b2fFluid;

/// The most liquids one fluid can hold
#define B2F_MAX_LIQUIDS 16

/// Settings of one liquid. A fluid can hold several liquids, which share its particle spacing and grid but move by
/// their own settings. Liquids do not mix: each pulls only on itself unless b2fFluid_SetCohesionBetween says otherwise.
typedef struct b2fLiquidDef
{
	/// Mass per unit area, to compare with b2ShapeDef::material density. Between liquids the lighter one takes more
	/// of each push, which makes it float.
	float massDensity;
	/// Displacement per substep per unit of density error, as a fraction of the interaction radius
	float stiffness;
	float nearStiffness;
	/// Share of the pull that pressure gives below rest density, from 0, where pressure only pushes, to 1, where it
	/// pulls as hard as it pushes. At 1 the pull is the liquid's only tension, as in Clavet's double density, and it
	/// can tear the bulk into holes. Lower values need cohesion to hold the liquid together.
	float tensilePressure;
	/// Pull between neighbours more than one spacing apart, and a push closer in, as its own term so that tension is
	/// set apart from stiffness. In the same units as stiffness. This is the pull within the liquid; the pull between
	/// two liquids is set with b2fFluid_SetCohesionBetween.
	float cohesion;
	/// Between two liquids viscosity is the mean of theirs
	float linearViscosity;
	float quadraticViscosity;
	float friction;
	float gravityScale;
	/// Pressure on faces shut off from the liquid by a contact, as a fraction of the pressure beside the contact.
	/// Particles cannot enter a closed gap, so this stands in for liquid seeping under a resting body.
	float contactPressure;
	/// Spacing of the coarsest particles over the fluid's particleSpacing, which is the finest. One keeps this liquid
	/// fine everywhere. Particles are fine at the surface and coarser with depth, and split and merge as they move.
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
} b2fLiquidDef;

/// Settings the whole fluid shares, and its first liquid
typedef struct b2fFluidDef
{
	/// Rest distance between particles. The interaction radius is twice this.
	float particleSpacing;
	/// How far from anything that is not liquid particles reach the coarsest size, in length units. Air, walls
	/// and bodies all count, so liquid is fine around each of them. Other liquids do not count. Zero or anything
	/// past the deepest safe value uses that value, which barely coarsens anything shallower than that.
	float adaptiveDepth;
	/// How full a box has to be, compared with liquid at rest, to count as full
	float adaptiveFullBox;
	int capacity;
	/// Liquid 0, which every fluid has
	b2fLiquidDef liquid;
} b2fFluidDef;

/// Timing of the last step in milliseconds, summed over substeps.
typedef struct b2fProfile
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
} b2fProfile;

B2_API b2fFluidDef b2fDefaultFluidDef( void );
B2_API b2fLiquidDef b2fDefaultLiquidDef( void );

/// Installs the world's sub-step callback, so a world can have one fluid.
B2_API b2fFluid* b2fCreateFluid( b2WorldId worldId, const b2fFluidDef* def );
B2_API void b2fDestroyFluid( b2fFluid* fluid );

/// Change settings between steps, including liquid 0's. The spacing and capacity cannot change.
B2_API void b2fFluid_SetDef( b2fFluid* fluid, const b2fFluidDef* def );

/// Add a liquid. Returns its index, or -1 if the fluid already holds B2F_MAX_LIQUIDS.
B2_API int b2fFluid_CreateLiquid( b2fFluid* fluid, const b2fLiquidDef* def );
/// Change a liquid's settings between steps
B2_API void b2fFluid_SetLiquid( b2fFluid* fluid, int liquid, const b2fLiquidDef* def );
B2_API int b2fFluid_GetLiquidCount( const b2fFluid* fluid );

/// Pull between particles of two different liquids, in the units of b2fLiquidDef::cohesion. It starts at zero, so
/// liquids keep apart; a value near their own cohesion lets them cling to each other.
B2_API void b2fFluid_SetCohesionBetween( b2fFluid* fluid, int liquidA, int liquidB, float cohesion );

/// Fill a box with particles of one liquid on a triangular lattice. Returns how many were added. With adaptive
/// sizes particles start at the size their distance from the edge of the box asks for.
B2_API int b2fFluid_CreateParticlesInBox( b2fFluid* fluid, int liquid, b2Pos lowerBound, b2Pos upperBound );

/// Move up to maxCount particles found in the box from to a lattice in the box to, with this velocity.
/// Call between steps. This is how a scene recycles liquid into a stream. Returns how many moved.
B2_API int b2fFluid_MoveParticles( b2fFluid* fluid, b2AABB from, b2AABB to, b2Vec2 velocity, int maxCount );

/// The total mass moved by b2fFluid_MoveParticles so far.
B2_API float b2fFluid_GetMovedMass( const b2fFluid* fluid );

/// Call before b2World_Step, which then runs the fluid in each sub-step.
B2_API void b2fFluid_Prepare( b2fFluid* fluid );

B2_API int b2fFluid_GetParticleCount( const b2fFluid* fluid );
/// World positions, in double when Box2D is built for large worlds
B2_API const b2Pos* b2fFluid_GetPositions( const b2fFluid* fluid );
B2_API const float* b2fFluid_GetVelocitiesX( const b2fFluid* fluid );
B2_API const float* b2fFluid_GetVelocitiesY( const b2fFluid* fluid );

/// The spacing of each particle. Mass is proportional to its square.
B2_API const float* b2fFluid_GetSizes( const b2fFluid* fluid );
/// The liquid each particle belongs to
B2_API const unsigned char* b2fFluid_GetLiquids( const b2fFluid* fluid );
/// Kernel density of each particle from the last substep, in the same units as b2fFluid_GetRestDensity
B2_API const float* b2fFluid_GetDensities( const b2fFluid* fluid );
B2_API float b2fFluid_GetRestDensity( const b2fFluid* fluid );
B2_API b2fProfile b2fFluid_GetProfile( const b2fFluid* fluid );

#ifdef __cplusplus
}
#endif

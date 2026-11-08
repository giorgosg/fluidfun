// SPDX-License-Identifier: MIT
#pragma once

#include "box2d.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct b2fFluid b2fFluid;

/// A particle coordinate. Double when Box2D is built for large worlds, matching b2Pos.
#if defined( BOX2D_DOUBLE_PRECISION )
typedef double b2fCoord;
#else
typedef float b2fCoord;
#endif

typedef struct b2fFluidDef
{
	/// Rest distance between particles. The interaction radius is twice this.
	float particleSpacing;
	/// Mass per unit area, to compare with b2ShapeDef::material density
	float massDensity;
	/// Displacement per substep per unit of density error, as a fraction of the interaction radius
	float stiffness;
	float nearStiffness;
	float linearViscosity;
	float quadraticViscosity;
	float friction;
	float gravityScale;
	/// Spacing of the coarsest particles over particleSpacing, which is the finest. One turns adaptivity off.
	/// Particles are fine at the surface and coarser with depth, and split and merge as they move.
	float adaptiveRatio;
	/// How far from anything that is not liquid particles reach the coarsest size, in length units. Air, walls
	/// and bodies all count, so liquid is fine around each of them. Zero or anything past the deepest safe value
	/// uses that value, which barely coarsens anything shallower than that.
	float adaptiveDepth;
	/// Particles moving at this speed stay fine. They turn fine quickly and relax slowly. Zero turns this off.
	float adaptiveSpeed;
	/// Keep particles no smaller than the safe size for the weight of liquid above them, read from how squeezed
	/// they are, since walls and floors make liquid near them fine whatever its depth.
	bool adaptiveDepthLimit;
	/// How full a box has to be, compared with liquid at rest, to count as full
	float adaptiveFullBox;
	/// A particle within this fraction of its target mass is left alone. Wider means fewer changes and sizes
	/// further from their targets.
	float adaptiveTolerance;
	/// A particle splits above this many times its target mass, and merges into its neighbours below one over it.
	float adaptiveLimit;
	/// Most mass a particle passes to its neighbours in one frame, as a fraction of its target. Merges and mass
	/// sharing happen over several frames at small values. One moves everything at once.
	float adaptiveRate;
	/// Pressure on faces shut off from the liquid by a contact, as a fraction of the pressure beside the contact.
	/// Particles cannot enter a closed gap, so this stands in for liquid seeping under a resting body.
	float contactPressure;
	int capacity;
} b2fFluidDef;

/// Timing of the last step in milliseconds, summed over substeps.
typedef struct b2fProfile
{
	float prepare;
	float sort;
	float viscosity;
	float density;
	float collide;
	float total;
	/// Splits, merges and mass shares in the last step
	int sizeChanges;
	/// Part of prepare spent measuring depth, flow and distance to bodies
	float measure;
} b2fProfile;

B2_API b2fFluidDef b2fDefaultFluidDef( void );

/// Installs the world's sub-step callback, so a world can have one fluid.
B2_API b2fFluid* b2fCreateFluid( b2WorldId worldId, const b2fFluidDef* def );
B2_API void b2fDestroyFluid( b2fFluid* fluid );

/// Change settings between steps. The spacing and capacity cannot change.
B2_API void b2fFluid_SetDef( b2fFluid* fluid, const b2fFluidDef* def );

/// Fill a box with particles on a triangular lattice. Returns how many were added. With adaptive sizes
/// particles start at the size their distance from the edge of the box asks for.
B2_API int b2fFluid_CreateParticlesInBox( b2fFluid* fluid, b2Pos lowerBound, b2Pos upperBound );

/// Move up to maxCount particles found in the box from to a lattice in the box to, with this velocity.
/// Call between steps. This is how a scene recycles liquid into a stream. Returns how many moved.
B2_API int b2fFluid_MoveParticles( b2fFluid* fluid, b2AABB from, b2AABB to, b2Vec2 velocity, int maxCount );

/// The total mass moved by b2fFluid_MoveParticles so far.
B2_API float b2fFluid_GetMovedMass( const b2fFluid* fluid );

/// Call before b2World_Step, which then runs the fluid in each sub-step.
B2_API void b2fFluid_Prepare( b2fFluid* fluid );

B2_API int b2fFluid_GetParticleCount( const b2fFluid* fluid );
B2_API const b2fCoord* b2fFluid_GetPositionsX( const b2fFluid* fluid );
B2_API const b2fCoord* b2fFluid_GetPositionsY( const b2fFluid* fluid );
B2_API const float* b2fFluid_GetVelocitiesX( const b2fFluid* fluid );
B2_API const float* b2fFluid_GetVelocitiesY( const b2fFluid* fluid );

/// The spacing of each particle. Mass is proportional to its square.
B2_API const float* b2fFluid_GetSizes( const b2fFluid* fluid );
/// Kernel density of each particle from the last substep, in the same units as b2fFluid_GetRestDensity
B2_API const float* b2fFluid_GetDensities( const b2fFluid* fluid );
B2_API float b2fFluid_GetRestDensity( const b2fFluid* fluid );
B2_API b2fProfile b2fFluid_GetProfile( const b2fFluid* fluid );

#ifdef __cplusplus
}
#endif

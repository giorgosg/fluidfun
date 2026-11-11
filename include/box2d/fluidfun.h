// SPDX-License-Identifier: MIT
#pragma once

#include "box2d.h"

#ifdef __cplusplus
extern "C" {
#endif

/// The most liquids one fluid can hold
#define B2_MAX_LIQUIDS 16

/// The solver's settings for one liquid. The defaults suit most liquids; change them knowing how the solver works.
/// What each does, and what has been measured about it, is written up outside this header.
typedef struct b2LiquidTuning
{
	/// Displacement per substep per unit of density error, as a fraction of the interaction radius
	float stiffness;
	/// The same for near density, which keeps neighbours from clumping. Usually 2.5 times stiffness.
	float nearStiffness;
	/// Share of the pull that pressure gives below rest density, from 0, where pressure only pushes, to 1, where it
	/// pulls as hard as it pushes. At 1 the pull is the liquid's only tension, as in Clavet's double density, and it
	/// can tear the bulk into holes. Lower values need cohesion to hold the liquid together. At 0, liquid with little
	/// cohesion and adaptive sizes never settles: gaps left by merging particles do not close, so particles split and
	/// merge over and over. The default of a half closes them without tearing holes.
	float tensilePressure;
	/// Viscosity that grows with the square of the closing speed, which damps only fast impacts. Between two liquids
	/// it is the mean of theirs.
	float quadraticViscosity;
	/// Share of its buoyancy a body is made up to where the particles' push falls short of it. Still liquid pushes on
	/// a body with the weight of the liquid it displaces. Particles cannot fill a gap narrower than a few of them, so a
	/// body touching a surface, or within three particle spacings of one, gets the missing push along the direction
	/// away from that surface. A nearly still body with liquid all the way round gets it up or down, since settled
	/// particles can otherwise hold it in place. At 1 a body lighter than the liquid lifts off the floor and rises, and
	/// a heavier one sinks and stays down. Zero turns both off.
	float buoyancyAssist;
	/// Share of the liquid within reach of a body that moves with it when the body speeds up or slows down, from 0
	/// to 1. Without it a light body takes the liquid's push alone, so one much lighter than the liquid shoots out of
	/// it. The body and that liquid share each change of the body's motion as one lump, which keeps momentum, and a
	/// body floating or resting at a steady speed is not affected.
	float addedMass;
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
} b2LiquidTuning;

/// One liquid: what it is like, then the solver's settings for it. A fluid can hold several liquids, which share its
/// particle spacing and grid but move by their own settings. Liquids do not mix: each pulls only on itself unless
/// b2Liquid_SetCohesionBetween says otherwise.
typedef struct b2LiquidDef
{
	/// Mass per unit area of the liquid at rest, to compare with b2ShapeDef::material density: a body lighter than
	/// this floats. Between liquids the lighter one takes more of each push, which makes it float.
	float density;
	/// How strongly neighbours closing on each other are slowed, in proportion to their closing speed. Between two
	/// liquids it is the mean of theirs.
	float viscosity;
	/// Pull between neighbours more than one spacing apart, and a push closer in, as its own term so that tension is
	/// set apart from stiffness. In the same units as stiffness. This is the pull within the liquid; the pull between
	/// two liquids is set with b2Liquid_SetCohesionBetween.
	float cohesion;
	/// Pull of static and kinematic shapes on the liquid, after Akinci et al. 2013: particles between half and all of
	/// the interaction radius from such a shape are drawn towards it. Bodies moved by forces do not pull. It helps a
	/// light body pressed onto the floor come back up. Zero turns it off. In the units of
	/// cohesion; above about 0.06 bodies slightly heavier than the liquid stop settling, and by 0.15 thin films creep
	/// up walls.
	float adhesion;
	/// Share of a particle's sliding speed along a shape lost in each substep it touches it, from 0, free slip, to
	/// 1, no slip. It does not grow with how hard the particle is pressed.
	float friction;
	/// Scale on the world's gravity for this liquid. Zero makes it weightless.
	float gravityScale;
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

	/// The solver's settings, for those who need them
	b2LiquidTuning tuning;
} b2LiquidDef;

/// Settings every liquid in a fluid shares
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
	/// The most particles the fluid can hold, over all its liquids
	int capacity;
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

/// Water, which the other liquids below start from
B2_API b2LiquidDef b2DefaultLiquidDef( void );
/// Lighter than water, so it floats on it
B2_API b2LiquidDef b2MakeOilDef( void );
/// Heavier than water, thick and slow, and it clings to itself
B2_API b2LiquidDef b2MakeHoneyDef( void );
/// 13.6 times as dense as water, and it beads up
B2_API b2LiquidDef b2MakeMercuryDef( void );
/// Thick, sticky and slow
B2_API b2LiquidDef b2MakeSlimeDef( void );
/// Springs back to the shape it had
B2_API b2LiquidDef b2MakeJellyDef( void );
/// Gives way when pushed and keeps its new shape
B2_API b2LiquidDef b2MakeDoughDef( void );

/// Give a world its fluid, which holds all of its liquids. A world has at most one, and b2World_Step steps it with
/// the bodies. Returns b2_nullFluidId if the world already has one. Call between steps.
B2_API b2FluidId b2CreateFluid( b2WorldId worldId, const b2FluidDef* def );

/// Remove a world's fluid and every particle in it. Destroying the world does this too. Call between steps.
B2_API void b2DestroyFluid( b2FluidId fluidId );

/// Whether the id names a fluid that still exists
B2_API bool b2Fluid_IsValid( b2FluidId fluidId );

/// Change the fluid's settings between steps. The spacing and capacity cannot change.
B2_API void b2Fluid_SetDef( b2FluidId fluidId, const b2FluidDef* def );

/// Add a liquid to a fluid, which starts with none. Returns b2_nullLiquidId if the fluid already holds
/// B2_MAX_LIQUIDS. Call between steps.
B2_API b2LiquidId b2CreateLiquid( b2FluidId fluidId, const b2LiquidDef* def );

/// Remove a liquid and every particle of it. Call between steps.
B2_API void b2DestroyLiquid( b2LiquidId liquidId );

/// Whether the id names a liquid that still exists
B2_API bool b2Liquid_IsValid( b2LiquidId liquidId );

/// Change a liquid's settings between steps
B2_API void b2Liquid_SetDef( b2LiquidId liquidId, const b2LiquidDef* def );
B2_API b2LiquidDef b2Liquid_GetDef( b2LiquidId liquidId );

/// The number b2FluidParticles::liquids gives for particles of this liquid
B2_API int b2Liquid_GetIndex( b2LiquidId liquidId );

/// How many liquids the fluid holds
B2_API int b2Fluid_GetLiquidCount( b2FluidId fluidId );

/// Pull between particles of two different liquids of one fluid, in the units of b2LiquidDef::cohesion. It starts
/// at zero, so liquids keep apart; a value near their own cohesion lets them cling to each other.
B2_API void b2Liquid_SetCohesionBetween( b2LiquidId liquidA, b2LiquidId liquidB, float cohesion );

/// Fill a box with particles of a liquid on a triangular lattice. Returns how many were added. With adaptive
/// sizes particles start at the size their distance from the edge of the box asks for.
B2_API int b2Liquid_CreateParticlesInBox( b2LiquidId liquidId, b2Pos lowerBound, b2Pos upperBound );

/// Move up to maxCount particles found in the box from to a lattice in the box to, with this velocity.
/// Call between steps. This is how a scene recycles liquid into a stream. Returns how many moved.
B2_API int b2Fluid_MoveParticles( b2FluidId fluidId, b2AABB from, b2AABB to, b2Vec2 velocity, int maxCount );

/// The total mass moved by b2Fluid_MoveParticles so far.
B2_API float b2Fluid_GetMovedMass( b2FluidId fluidId );

/// How many particles the fluid holds, over all its liquids
B2_API int b2Fluid_GetParticleCount( b2FluidId fluidId );

/// Every particle of a fluid, as arrays of count values each, one entry per particle at the same index in every array.
/// The arrays belong to the fluid. They are good until the next b2World_Step, or the next call that adds, moves or
/// removes particles or destroys a liquid or the fluid; read them again after any of those. Particles change order
/// every step, so an index names a particle only until then, and ids follow a particle across steps.
typedef struct b2FluidParticles
{
	int count;
	/// World positions, in double when Box2D is built for large worlds
	const b2Pos* positions;
	const float* velocitiesX;
	const float* velocitiesY;
	/// The spacing of each particle, in the units of b2FluidDef::particleSpacing, so particleSpacing for a particle
	/// at the finest size. Mass is proportional to its square.
	const float* sizes;
	/// The liquid each particle belongs to, as the number b2Liquid_GetIndex gives
	const unsigned char* liquids;
	/// Kernel density of each particle from the last substep, in the units of restDensity
	const float* densities;
	/// A number for each particle, unique within the fluid and kept while the particle lasts, through every step.
	/// Particles get them in the order they are made, from zero, and a number is not given again. With adaptive sizes
	/// a particle that splits keeps its id on one of its parts, and the others get new ones; a particle that merges
	/// away gives its mass to neighbours, which keep their ids, and its own id is gone. b2Fluid_MoveParticles keeps
	/// ids, so a moved particle has its id at its new place. Ids are ints, so they stay unique for the first 2^31
	/// particles a fluid makes, splits included. Scenes of about 5000 particles make fewer than 7000 in 10 seconds.
	const int* ids;
	/// Kernel density of liquid at rest, which densities are compared with
	float restDensity;
} b2FluidParticles;

B2_API b2FluidParticles b2Fluid_GetParticles( b2FluidId fluidId );
B2_API b2FluidProfile b2Fluid_GetProfile( b2FluidId fluidId );

/// What the liquid did to a body over the last step. It is worked out at the end of each b2World_Step and is good
/// until the next one.
///
/// Only bodies the solver moves this step feel the liquid: dynamic bodies, which the fluid keeps awake near liquid, and
/// kinematic bodies that are awake. Static bodies, and kinematic bodies Box2D has put to sleep, are never in liquid
/// here and report a zeroed state. A kinematic body reports the push the liquid gives it, though nothing moves it.
typedef struct b2FluidBodyState
{
	/// Whether liquid has been within reach of the body: within the interaction radius of its outline, 1.78 particle
	/// spacings for particles at the finest size. It turns true once there has been liquid in reach for 3 steps in a
	/// row, and false once there has been none for 3 steps in a row, so a drop that splashes past does not begin and
	/// end it. The other fields are from the last step whatever this says.
	bool inLiquid;
	/// Share of the body under liquid, 0 to 1, averaged over the substeps of the last step. It is the solver's own
	/// estimate, which buoyancyAssist uses: twice the mean height of the liquid within reach above the body's lowest
	/// point, over the body's height, against gravity; and 1 with liquid all the way round the body. That would be
	/// exact for a box wet up its sides alone, but the liquid under the body's bottom counts too, so it reads low for
	/// a body afloat, and more so for a small one: with particles 0.08 apart, an upright crate 1.2 across that is half
	/// under reads about 0.37, and one 0.6 across that is 0.42 under reads about 0.06. Other shapes are rougher
	/// still. Use it to tell more from less; inLiquid and force are the better guides. Zero without gravity unless
	/// liquid is all round.
	float submerged;
	/// The liquid's push on the body, its impulse over the last step divided by the step's time, in Box2D's units of
	/// force. It counts the particles pushed out of the body and their pressure across its outline, and the buoyancy
	/// that b2LiquidTuning::buoyancyAssist makes up in gaps and around a still body. It leaves out what
	/// b2LiquidTuning::addedMass does, which shares each change of the body's motion with the liquid near it, so a
	/// light body speeding up accelerates by less than this force over its mass.
	b2Vec2 force;
	/// The turning effect of that push about the body's centre of mass, its angular impulse over the step's time
	float torque;
	/// The liquid most of the mass within reach of the body is, counted in the last substep of the last step that
	/// had liquid in reach at its end, or null if it has not been in liquid
	b2LiquidId liquidId;
} b2FluidBodyState;

/// The state of a body after the last step. A body that has not been near liquid, or that no longer exists, gives a
/// zeroed state with inLiquid false and a null liquid.
B2_API b2FluidBodyState b2Fluid_GetBodyState( b2FluidId fluidId, b2BodyId bodyId );

/// A body's inLiquid turned true in the last step
typedef struct b2FluidBodyBeginEvent
{
	b2BodyId bodyId;
	/// The liquid most of what is within reach of it is
	b2LiquidId liquidId;
} b2FluidBodyBeginEvent;

/// A body's inLiquid turned false in the last step, or a body that was in liquid was destroyed since the step before.
/// @warning the body may have been destroyed, so confirm it with b2Body_IsValid
typedef struct b2FluidBodyEndEvent
{
	b2BodyId bodyId;
} b2FluidBodyEndEvent;

/// A sensor shape went from holding no liquid to holding some in the last step
typedef struct b2FluidSensorBeginEvent
{
	b2ShapeId sensorShapeId;
} b2FluidSensorBeginEvent;

/// A sensor shape that held liquid holds none after the last step, or it was destroyed since the step before.
/// @warning the shape may have been destroyed, so confirm it with b2Shape_IsValid
typedef struct b2FluidSensorEndEvent
{
	b2ShapeId sensorShapeId;
} b2FluidSensorEndEvent;

/// What changed in the last step, as Box2D gives sensor and contact events: arrays with counts, which belong to the
/// fluid and are good until the next b2World_Step.
///
/// Sensors count liquid in every sensor shape near the liquid, by whether each particle's centre is inside the
/// shape's outline, once a step after the substeps. Sensor shapes made from segments hold nothing. Sensors have no
/// delay, unlike bodies: a sensor that one drop passes through begins and ends with it.
typedef struct b2FluidEvents
{
	b2FluidBodyBeginEvent* bodyBeginEvents;
	b2FluidBodyEndEvent* bodyEndEvents;
	b2FluidSensorBeginEvent* sensorBeginEvents;
	b2FluidSensorEndEvent* sensorEndEvents;
	int bodyBeginCount;
	int bodyEndCount;
	int sensorBeginCount;
	int sensorEndCount;
} b2FluidEvents;

B2_API b2FluidEvents b2Fluid_GetEvents( b2FluidId fluidId );

/// How much liquid a sensor shape held after the last step, as the area it fills at rest, in length units squared:
/// a particle of size s, as b2FluidParticles::sizes gives it, fills 0.866 s^2. Zero for a shape that is not a sensor
/// or is far from the liquid. Liquid squeezed by depth packs a little tighter than at rest, so a sensor under liquid
/// reads a little over its own area: 3% over, averaged over a second, for a box 0.8 by 0.3 between 0.25 and 0.55
/// under the surface with particles 0.08 apart. Particles crossing its edges make it vary from step to step.
B2_API float b2Fluid_GetSensorArea( b2FluidId fluidId, b2ShapeId sensorShapeId );

/// An amount of liquid
typedef struct b2FluidAmount
{
	/// Particles
	int count;
	/// The area they fill at rest, 0.866 s^2 for a particle of size s, in length units squared
	float area;
	/// That area times each particle's liquid's density
	float mass;
} b2FluidAmount;

/// The liquid whose particle centres are inside a box, counted now with a loop over every particle. Call it between
/// steps.
B2_API b2FluidAmount b2Fluid_MeasureBox( b2FluidId fluidId, b2Pos lowerBound, b2Pos upperBound );

/// Where the liquid's surface is over a point, counted now with a loop over every particle. Among particles within two
/// particle spacings of the line through the point along gravity, it takes the highest, leaving out spray, whose
/// density is under 0.4 of rest, and gives the point on that line half that particle's size above it. So it is the
/// top of the liquid in that column, wherever the point is: over liquid resting on a shelf above a pool it finds the
/// top of that on the shelf. Returns false, and leaves surface alone, if there is no such particle. With no gravity,
/// up is +y. Call it between steps.
B2_API bool b2Fluid_GetSurfaceHeight( b2FluidId fluidId, b2Pos point, b2Pos* surface );

#ifdef __cplusplus
}
#endif

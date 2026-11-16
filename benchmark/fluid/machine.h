// SPDX-License-Identifier: MIT
#pragma once

#include "box2d/fluidfun.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// A marble run through a large world. It is a row of sections, each of which takes a marble rolling in at its entry
// and sends one rolling out of its exit, by whatever it shows on the way: some with liquids, some with plain bodies.
// Sections butt together, turn back under each other, and each can be built alone with a ramp and a marble in front
// of it, to try it by itself. The machine also says where to look: a view that follows whatever is acting.

typedef enum MachineLiquid
{
	machine_water,
	machine_oil,
	machine_honey,
	machine_mercury,
	machine_jelly,
	machine_dough,
	machine_liquidCount
} MachineLiquid;

typedef enum MachineSectionType
{
	// A ramp with a marble held at its top
	section_start,
	// The marble opens a sluice, the water turns a wheel and fills a sump, and a marble floats out of the sump
	section_sluice,
	// Dominoes that grow, the last of which knocks a marble on
	section_dominoes,
	// The marble drops into a paddle boat, which crosses a lake and knocks a flap over onto a marble
	section_ferry,
	// Ramps back and forth down to the row below, which runs the other way
	section_turn,
	// A Newton's cradle
	section_cradle,
	// Trapdoors drop a block of dough and a block of jelly; the jelly lands on a seesaw that throws a marble
	section_jelly,
	// Honey fills a bucket on a balance, which lifts a tooth out of a marble's way
	section_honey,
	// Mercury fills a shaft and a stone ball floats up it
	section_mercury,
	// The stone ball sinks through oil and water onto a detonator
	section_finale,
	section_typeCount
} MachineSectionType;

#define MACHINE_MAX_SECTIONS 16
#define MACHINE_MAX_BODIES 256
#define MACHINE_SECTION_PARTS 8

typedef struct MachineDef
{
	b2FluidDef fluid;
	b2LiquidDef liquids[machine_liquidCount];
	// Marbles are lighter than water, so they float. The stone ball of the mercury lift and the finale is not one of these.
	float marbleRadius;
	float marbleDensity;
	// How many metres of height the view shows above and below its centre while it follows the action
	float viewHalfHeight;
	// How stiff the spring is that pulls the view toward the action, as the frequency it would swing at undamped
	float viewHertz;
	// The one section to build, behind a start ramp, or section_typeCount for the whole machine
	MachineSectionType only;
} MachineDef;

typedef struct MachineSection
{
	MachineSectionType type;
	// Where the marble rolls in, and which way it is going: 1 to the right, -1 to the left. A section is written
	// going right and built mirrored for the other way.
	b2Vec2 origin;
	float direction;
	// Where the next section starts
	b2Vec2 exit;
	// How far it has got, and for how long it has been there
	int phase;
	float clock;
	// What it moves or watches
	b2BodyId parts[MACHINE_SECTION_PARTS];
	b2JointId joint;
	// The marble it sends on
	b2BodyId marble;
	int count;
} MachineSection;

typedef struct Machine
{
	MachineDef def;
	b2WorldId worldId;
	b2FluidId fluid;
	b2LiquidId liquids[machine_liquidCount];
	b2BodyId ground;

	MachineSection sections[MACHINE_MAX_SECTIONS];
	int sectionCount;

	// The marble to follow, and the section it is in
	b2BodyId baton;
	int active;

	// Where to look: the target the active section asks for, and the view easing toward it
	b2Vec2 focus;
	float focusHalfHeight;
	b2Vec2 viewCenter;
	float viewHalfHeight;
	b2Vec2 viewVelocity;
	float viewZoomSpeed;

	// Everything built, for the overview at the end
	b2AABB bounds;

	// Every body but the ground, for drawing without a window
	b2BodyId bodies[MACHINE_MAX_BODIES];
	int bodyCount;

	float time;
	// Set once the last section is done and the view has pulled back, with the time since
	bool finished;
	float finishedFor;
} Machine;

MachineDef DefaultMachineDef( void );
const char* MachineSectionName( MachineSectionType type );

/// Builds the machine into a world with no fluid yet, and gives the world its fluid
void CreateMachine( Machine* machine, b2WorldId worldId, const MachineDef* def );

/// Call once per frame before b2World_Step
void StepMachine( Machine* machine, float timeStep );

#ifdef __cplusplus
}
#endif

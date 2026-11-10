// SPDX-License-Identifier: MIT
#pragma once

#include "box2d/fluidfun.h"

#ifdef __cplusplus
extern "C" {
#endif

// Shared by the sample and the headless runner, so that both run the same scenes.
typedef enum LabSceneType
{
	lab_tank,
	lab_hole,
	lab_boat,
	lab_drum,
	lab_wheel,
	lab_waves,
	lab_splash,
	lab_drain,
	lab_pool,
	lab_drop,
	lab_layers,
	lab_chain,
	lab_tap,
	lab_sceneCount
} LabSceneType;

#define LAB_MAX_BODIES 64

typedef struct LabScene
{
	LabSceneType type;
	const char* name;

	// The part of the world to look at
	b2Vec2 viewCenter;
	float viewHalfWidth;

	// Tank settings
	float tankWidth;
	float tankHeight;
	float liquidWidth;
	float liquidHeight;
	int crateCount;

	// One setting a scene can be driven by, or a null label if it has none
	const char* controlLabel;
	float control, controlMin, controlMax;

	// Every moving or drawn body, for tests and frame dumps. Crates come first in tank scenes.
	b2BodyId bodies[LAB_MAX_BODIES];
	int bodyCount;
	b2BodyId driven;
	b2JointId joint;
	float time;
	int leaked;
	int pumped;
	int stall, longestStall;

	// Liquid a scene keeps where it put it, such as a basin's water, as area in particle sizes squared, so that it
	// can move the rest back
	float keep;
} LabScene;

LabScene LabDefaultScene( LabSceneType type );

/// Changes the fluid settings a scene starts from, for scenes that show a liquid other than water. Call it on
/// b2fDefaultFluidDef before creating the scene.
void LabSceneSettings( LabSceneType type, b2fFluidDef* def );
b2fFluid* LabCreateScene( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def );

/// Call once per frame before b2fFluid_Prepare. Drives whatever the scene moves.
void LabStepScene( LabScene* scene, b2fFluid* fluid, float timeStep );

#ifdef __cplusplus
}
#endif

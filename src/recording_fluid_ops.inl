// SPDX-License-Identifier: MIT
// fluidfun: X-macro manifest of the fluid's recorded calls, the companion of recording_ops.inl.
// Include with B2_REC_FLUID_OP and ARG defined. No commas between ARG tokens.
//
// B2_REC_FLUID_OP( subop, Name, RET, ARGS )
//   RET in { RET_NONE, RET_FLUIDID, RET_LIQUIDID, RET_EMITTERID, RET_I32 }
//
// Every fluid record goes out under the one opcode B2_REC_OP_FLUID, with this subop as the first byte of its payload,
// so the fork takes a single opcode of Box2D's space and the fluid's calls can grow without meeting Box2D's own. A
// RET_I32 op appends what the call returned, how many particles it added or changed, which replay compares.
//
// Subop ranges:
//   0x01-0x0F  fluid lifecycle and settings
//   0x10-0x1F  liquids
//   0x20-0x2F  spawns
//   0x30-0x3F  calls on the particles in a region, and explosions
//   0x40-0x4F  emitters

B2_REC_FLUID_OP( 0x01, CreateFluid, RET_FLUIDID, ARG( WORLDID, world ) ARG( FLUIDDEF, def ) )
B2_REC_FLUID_OP( 0x02, DestroyFluid, RET_NONE, ARG( FLUIDID, fluid ) )
B2_REC_FLUID_OP( 0x03, FluidSetAdaptiveDepth, RET_NONE, ARG( FLUIDID, fluid ) ARG( F32, adaptiveDepth ) )
B2_REC_FLUID_OP( 0x04, FluidSetAdaptiveFullBox, RET_NONE, ARG( FLUIDID, fluid ) ARG( F32, adaptiveFullBox ) )
B2_REC_FLUID_OP( 0x05, FluidSetMaxCapacity, RET_NONE, ARG( FLUIDID, fluid ) ARG( I32, maxCapacity ) )

B2_REC_FLUID_OP( 0x10, CreateLiquid, RET_LIQUIDID, ARG( FLUIDID, fluid ) ARG( LIQUIDDEF, def ) )
B2_REC_FLUID_OP( 0x11, DestroyLiquid, RET_NONE, ARG( LIQUIDID, liquid ) )
B2_REC_FLUID_OP( 0x12, LiquidSetDef, RET_NONE, ARG( LIQUIDID, liquid ) ARG( LIQUIDDEF, def ) )
B2_REC_FLUID_OP( 0x13, LiquidSetCohesionBetween, RET_NONE, ARG( LIQUIDID, liquidA ) ARG( LIQUIDID, liquidB ) ARG( F32, cohesion ) )

B2_REC_FLUID_OP( 0x20, LiquidSpawnBox, RET_I32,
				 ARG( LIQUIDID, liquid ) ARG( POSITION, lowerBound ) ARG( POSITION, upperBound ) ARG( VEC2, velocity ) )
B2_REC_FLUID_OP( 0x21, LiquidSpawnCircle, RET_I32,
				 ARG( LIQUIDID, liquid ) ARG( POSITION, center ) ARG( F32, radius ) ARG( VEC2, velocity ) )
B2_REC_FLUID_OP( 0x22, LiquidSpawnCapsule, RET_I32,
				 ARG( LIQUIDID, liquid ) ARG( POSITION, center1 ) ARG( POSITION, center2 ) ARG( F32, radius ) ARG( VEC2, velocity ) )
B2_REC_FLUID_OP( 0x23, LiquidSpawnPolygon, RET_I32,
				 ARG( LIQUIDID, liquid ) ARG( POSITION, origin ) ARG( ROT, rotation ) ARG( POLYGON, polygon ) ARG( VEC2, velocity ) )

B2_REC_FLUID_OP( 0x30, FluidRemove, RET_I32, ARG( FLUIDID, fluid ) ARG( LIQUIDID, liquid ) ARG( REGION, region ) ARG( I32, maxCount ) )
B2_REC_FLUID_OP( 0x31, FluidAccelerate, RET_I32,
				 ARG( FLUIDID, fluid ) ARG( LIQUIDID, liquid ) ARG( REGION, region ) ARG( VEC2, velocityChange ) )
B2_REC_FLUID_OP( 0x32, FluidBlendVelocity, RET_I32,
				 ARG( FLUIDID, fluid ) ARG( LIQUIDID, liquid ) ARG( REGION, region ) ARG( VEC2, velocity ) ARG( F32, share ) )
B2_REC_FLUID_OP( 0x33, FluidExplode, RET_I32, ARG( FLUIDID, fluid ) ARG( EXPLOSIONDEF, def ) )

B2_REC_FLUID_OP( 0x40, CreateEmitter, RET_EMITTERID, ARG( LIQUIDID, liquid ) ARG( EMITTERDEF, def ) )
B2_REC_FLUID_OP( 0x41, DestroyEmitter, RET_NONE, ARG( EMITTERID, emitter ) )
B2_REC_FLUID_OP( 0x42, EmitterSetDef, RET_NONE, ARG( EMITTERID, emitter ) ARG( EMITTERDEF, def ) )

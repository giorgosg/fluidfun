// SPDX-License-Identifier: MIT
// Creating a liquid, its settings, what it reports, and the order of passes in each substep.
#include "fluid_internal.h"

#include "bitset.h"

#include "core.h"
#include "fluid.h"
#include "physics_world.h"
#include "platform.h"
#include "recording.h"

b2LiquidDef b2DefaultLiquidDef( void )
{
	b2LiquidDef def = { 0 };
	def.density = 1.0f;
	def.tuning.stiffness = 0.04f;
	def.tuning.nearStiffness = 0.1f;
	def.tuning.tensilePressure = 0.5f;
	def.cohesion = 0.04f;
	def.adhesion = 0.025f;
	def.viscosity = 0.5f;
	def.friction = 0.1f;
	def.gravityScale = 1.0f;
	def.tuning.buoyancyAssist = 1.0f;
	def.tuning.addedMass = 1.0f;
	def.springStiffness = 0.0f;
	def.plasticity = 0.5f;
	def.yieldRatio = 0.1f;
	def.formSpringsAfterSpawn = true;
	def.tuning.adaptiveRatio = 10.0f;
	def.tuning.adaptiveSpeed = 0.0f;
	def.tuning.adaptiveDepthLimit = false;
	def.tuning.adaptiveTolerance = 0.3f;
	def.tuning.adaptiveLimit = 3.0f;
	def.tuning.adaptiveRate = 3.0f;
	def.categoryBits = B2_DEFAULT_CATEGORY_BITS;
	def.maskBits = B2_DEFAULT_MASK_BITS;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

// The values the sample scenes use for these liquids
b2LiquidDef b2MakeOilDef( void )
{
	b2LiquidDef def = b2DefaultLiquidDef();
	def.density = 0.6f;
	def.cohesion = 0.1f;
	return def;
}

b2LiquidDef b2MakeHoneyDef( void )
{
	b2LiquidDef def = b2DefaultLiquidDef();
	def.density = 1.4f;
	def.cohesion = 0.2f;
	def.friction = 0.3f;
	def.viscosity = 20.0f;
	return def;
}

b2LiquidDef b2MakeMercuryDef( void )
{
	b2LiquidDef def = b2DefaultLiquidDef();
	def.density = 13.6f;
	def.cohesion = 0.1f;
	return def;
}

b2LiquidDef b2MakeSlimeDef( void )
{
	b2LiquidDef def = b2DefaultLiquidDef();
	def.cohesion = 0.25f;
	def.viscosity = 10.0f;
	def.friction = 0.5f;
	return def;
}

b2LiquidDef b2MakeJellyDef( void )
{
	b2LiquidDef def = b2DefaultLiquidDef();
	def.springStiffness = 10000.0f;
	def.plasticity = 0.0f;
	// Springs only from the spawn, so it pulls back to the shape it spawned with: forming new springs while a crate
	// squeezed it left a dent 0.17 m deep (findings section 58)
	def.formSpringsAfterSpawn = false;
	// Uniform sizes: a particle that splits starts with no springs and forms them where it is, so splits while it is
	// squeezed would record the squeeze again
	def.tuning.adaptiveRatio = 1.0f;
	return def;
}

b2LiquidDef b2MakeDoughDef( void )
{
	// Forms springs after spawn, as the default does: without, springs that stretch to the interaction radius go for good, and in 30 s
	// half the dough's were gone and it spread like a liquid (findings section 58)
	b2LiquidDef def = b2DefaultLiquidDef();
	def.springStiffness = 10000.0f;
	def.plasticity = 1.0f;
	def.yieldRatio = 0.1f;
	return def;
}

b2FluidDef b2DefaultFluidDef( void )
{
	b2FluidDef def = { 0 };
	// Lengths are in the caller's units, as b2SetLengthUnitsPerMeter sets them, as Box2D's own defaults are
	def.particleSpacing = 0.08f * b2_lengthUnitsPerMeter;
	def.adaptiveDepth = 12.0f * b2_lengthUnitsPerMeter;
	def.adaptiveFullBox = 0.9f;
	def.capacity = 4096;
	def.maxCapacity = 1000000;
	def.enableSleep = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

// The defs are checked as Box2D checks its own: an assert, and a log and a refusal in builds without asserts. Each
// range is what the field's documentation allows; SETTINGS.md has what the values do.
static bool IsValidFluidDef( const b2FluidDef* def )
{
	B2_CHECK_DEF( def );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->particleSpacing ) && def->particleSpacing > 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->adaptiveDepth ) && def->adaptiveDepth >= 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->adaptiveFullBox ) && def->adaptiveFullBox > 0.0f, false );
	B2_CHECK_INPUT_RETURN( def->capacity >= 0 && def->maxCapacity >= 0, false );
	return true;
}

static bool IsValidLiquidDef( const b2LiquidDef* def )
{
	B2_CHECK_DEF( def );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->density ) && def->density > 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->viscosity ) && def->viscosity >= 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->cohesion ) && def->cohesion >= 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->adhesion ) && def->adhesion >= 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->friction ) && 0.0f <= def->friction && def->friction <= 1.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->gravityScale ), false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->springStiffness ) && def->springStiffness >= 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->plasticity ) && def->plasticity >= 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( def->yieldRatio ) && def->yieldRatio >= 0.0f, false );

	const b2LiquidTuning* t = &def->tuning;
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( t->stiffness ) && t->stiffness >= 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( t->nearStiffness ) && t->nearStiffness >= 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( t->tensilePressure ) && 0.0f <= t->tensilePressure && t->tensilePressure <= 1.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( t->quadraticViscosity ) && t->quadraticViscosity >= 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( t->buoyancyAssist ) && t->buoyancyAssist >= 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( t->addedMass ) && 0.0f <= t->addedMass && t->addedMass <= 1.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( t->adaptiveRatio ) && t->adaptiveRatio >= 1.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( t->adaptiveSpeed ) && t->adaptiveSpeed >= 0.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( t->adaptiveTolerance ) && 0.0f <= t->adaptiveTolerance && t->adaptiveTolerance < 1.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( t->adaptiveLimit ) && t->adaptiveLimit > 1.0f, false );
	B2_CHECK_INPUT_RETURN( b2IsValidFloat( t->adaptiveRate ) && t->adaptiveRate >= 0.0f, false );
	return true;
}

// The id table b2fRelaxSprings fills: a power of two at least twice the room for particles
static size_t IdMapSlots( int capacity )
{
	size_t n = (size_t)capacity + 1, slots = 1;
	while ( slots < 2 * n )
	{
		slots *= 2;
	}
	return slots;
}

// Fields only adaptive sizes, adaptiveSpeed and springs use are made the first time some liquid needs them, zeroed, as
// they would have been at the start, and kept from then on
void b2fAllocateOptional( b2fFluid* f, bool adapt, bool speed, bool springs )
{
	size_t n = (size_t)f->capacity + 1;
	if ( adapt && f->adaptMass == NULL )
	{
#define B2F_ALLOC_FIELD( type, name ) f->name = b2AllocZero( n * sizeof( type ) );
		B2F_CARRIED_FIELDS( B2F_ALLOC_FIELD )
		B2F_ADAPT_FIELDS( B2F_ALLOC_FIELD )
#undef B2F_ALLOC_FIELD
	}
	if ( speed && f->speedMemory == NULL )
	{
#define B2F_ALLOC_FIELD( type, name ) f->name = b2AllocZero( n * sizeof( type ) );
#define B2F_ALLOC_SORTED( type, name ) f->sorted.name = b2AllocZero( n * sizeof( type ) );
		B2F_SPEED_FIELDS( B2F_ALLOC_FIELD )
		B2F_SPEED_FIELDS( B2F_ALLOC_SORTED )
#undef B2F_ALLOC_FIELD
#undef B2F_ALLOC_SORTED
	}

	if ( springs && f->springs == NULL )
	{
		f->springs = b2AllocZero( n * sizeof( b2fSprings ) );
		f->sortedSprings = b2AllocZero( n * sizeof( b2fSprings ) );
		f->idMap = b2Alloc( IdMapSlots( f->capacity ) * sizeof( b2fIdSlot ) );
		f->springParent = b2Alloc( n * sizeof( int ) );
		f->springGroup = b2Alloc( n * sizeof( unsigned char ) );
	}
}

// Rebuilds what the passes read from the liquids' settings. A destroyed liquid's slot keeps its last settings, which
// nothing reads since no particle is of it.
void b2fUpdateLiquids( b2fFluid* f )
{
	float spacing = f->spacing;
	f->adaptive = false;
	f->springy = false;
	f->springLiquids = 0;
	bool fastSpeed = false;
	for ( int a = 0; a < f->liquidCount; ++a )
	{
		b2fLiquid* liquid = f->liquids + a;
		liquid->mass = liquid->def.density * 0.8660254f * spacing * spacing / B2F_REST_FILL;
		f->cohesionBetween[a][a] = liquid->def.cohesion;
		if ( liquid->alive )
		{
			bool adaptive = liquid->def.tuning.adaptiveRatio > 1.0f;
			f->adaptive = f->adaptive || adaptive;
			fastSpeed = fastSpeed || ( adaptive && liquid->def.tuning.adaptiveSpeed > 0.0f );
			f->springy = f->springy || liquid->def.springStiffness > 0.0f;
			f->springLiquids |= liquid->def.springStiffness > 0.0f ? 1u << a : 0u;
		}
	}

	b2fAllocateOptional( f, f->adaptive, fastSpeed, f->springy );

	for ( int a = 0; a < f->liquidCount; ++a )
	{
		const b2LiquidDef* da = &f->liquids[a].def;
		for ( int b = 0; b < f->liquidCount; ++b )
		{
			const b2LiquidDef* db = &f->liquids[b].def;
			float sum = da->density + db->density;
			f->liquidPairs[a][b] = ( b2fLiquidPair ){
				.cohesion = f->cohesionBetween[a][b],
				.weight = sum > 0.0f ? 2.0f * db->density / sum : 1.0f,
				.viscosity = 0.5f * ( da->viscosity + db->viscosity ),
				.quadraticViscosity = 0.5f * ( da->tuning.quadraticViscosity + db->tuning.quadraticViscosity ),
			};
		}
	}

	// Which liquids push on which. A destroyed slot sees everything, which nothing reads since no particle is of it.
	f->liveLiquids = 0;
	f->liquidsFiltered = false;
	for ( int a = 0; a < B2F_MAX_LIQUIDS; ++a )
	{
		f->liquidSees[a] = UINT32_MAX;
		if ( a < f->liquidCount && f->liquids[a].alive )
		{
			f->liveLiquids |= 1u << a;
		}
	}
	for ( int a = 0; a < f->liquidCount; ++a )
	{
		const b2LiquidDef* da = &f->liquids[a].def;
		for ( int b = 0; b < f->liquidCount; ++b )
		{
			const b2LiquidDef* db = &f->liquids[b].def;
			bool live = f->liquids[a].alive && f->liquids[b].alive;
			if ( live && a != b && ( ( da->maskBits & db->categoryBits ) == 0 || ( da->categoryBits & db->maskBits ) == 0 ) )
			{
				f->liquidSees[a] &= ~( 1u << b );
				f->liquidsFiltered = true;
			}
		}
	}
}

uint32_t b2fLiquidsCollidingWith( const b2fFluid* f, b2Filter filter )
{
	uint32_t liquids = 0;
	for ( int l = 0; l < f->liquidCount; ++l )
	{
		const b2LiquidDef* def = &f->liquids[l].def;
		if ( f->liquids[l].alive && ( def->maskBits & filter.categoryBits ) != 0 && ( def->categoryBits & filter.maskBits ) != 0 )
		{
			liquids |= 1u << l;
		}
	}
	return liquids;
}

static float LatticeDensity( float spacing, float radius )
{
	float sum = 0.0f;
	float rowSpacing = 0.8660254f * spacing;
	for ( int row = -32; row <= 32; ++row )
	{
		for ( int column = -32; column <= 32; ++column )
		{
			float dx = ( (float)column + ( row & 1 ? 0.5f : 0.0f ) ) * spacing;
			float dy = (float)row * rowSpacing;
			float d = sqrtf( dx * dx + dy * dy );
			if ( d > 0.0f && d < radius )
			{
				float q = 1.0f - d / radius;
				sum += q * q;
			}
		}
	}
	return sum;
}

// The world holds its one fluid. An id is good while the world is and until the fluid is destroyed.
b2fFluid* b2fGetFluid( b2FluidId fluidId )
{
	b2World* world = b2GetWorld( fluidId.world0 );
	B2_ASSERT( world->fluid != NULL && fluidId.index1 == world->fluidSerial && fluidId.generation == world->generation );
	return world->fluid;
}

b2FluidId b2CreateFluid( b2WorldId worldId, const b2FluidDef* def )
{
	if ( IsValidFluidDef( def ) == false )
	{
		return b2_nullFluidId;
	}

	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked || world->fluid != NULL )
	{
		return b2_nullFluidId;
	}

	b2FluidDef clamped = *def;
	clamped.capacity = def->capacity > 0 ? def->capacity : 0;
	// Never below the room the fluid starts with, so the def holds the limit growing goes by
	clamped.maxCapacity = def->maxCapacity > clamped.capacity ? def->maxCapacity : clamped.capacity;
	world->fluid = b2fMakeFluid( world, &clamped, clamped.capacity );
	world->fluidSerial += 1;
	b2FluidId id = { world->fluidSerial, world->worldId, world->generation };
	B2_REC_CREATE( world, CreateFluid, id, worldId, *def );
	return id;
}

b2FluidId b2World_GetFluid( b2WorldId worldId )
{
	b2World* world = b2GetWorldFromId( worldId );
	if ( world == NULL || world->fluid == NULL )
	{
		return b2_nullFluidId;
	}
	return ( b2FluidId ){ world->fluidSerial, world->worldId, world->generation };
}

// A fluid with room for capacity particles and no liquids, as b2CreateFluid makes one, and as a snapshot restores one
b2fFluid* b2fMakeFluid( b2World* world, const b2FluidDef* def, int capacity )
{
	b2fFluid* f = b2AllocZero( sizeof( b2fFluid ) );
	f->world = world;
	f->def = *def;
	// Each grid's axes have a stretch from the start, as its corner and cell do, until the first sort lays them out
	b2fPlainAxis( &f->grid.ax, 0.0f, 1.0f );
	b2fPlainAxis( &f->grid.ay, 0.0f, 1.0f );
	b2fPlainAxis( &f->sleepGrid.ax, 0.0f, 1.0f );
	b2fPlainAxis( &f->sleepGrid.ay, 0.0f, 1.0f );
	f->capacity = capacity;
	f->spacing = b2fLatticeSize( def->particleSpacing );
	f->radius = 2.0f * f->spacing;
	f->cellSize = 1.1f * f->radius;
	f->fineArea = 0.8660254f * f->spacing * f->spacing / B2F_REST_FILL;
	b2fUpdateLiquids( f );
	f->restDensity = LatticeDensity( f->spacing, f->radius );
	f->maxRadius = f->radius;
	f->gridStale = true;
	f->springAsleep = -1;

	size_t n = (size_t)f->capacity + 1;
#define B2F_ALLOC_FIELD( type, name ) f->name = b2AllocZero( n * sizeof( type ) );
#define B2F_ALLOC_SORTED( type, name ) f->sorted.name = b2AllocZero( n * sizeof( type ) );
	B2F_PARTICLE_FIELDS( B2F_ALLOC_FIELD )
	B2F_PARTICLE_FIELDS( B2F_ALLOC_SORTED )
	B2F_WORK_FIELDS( B2F_ALLOC_FIELD )
#undef B2F_ALLOC_FIELD
#undef B2F_ALLOC_SORTED
	f->contacts = b2AllocZero( n * B2F_CONTACTS_PER_PARTICLE * sizeof( b2fContact ) );
	f->marked = b2CreateBitSet( (uint32_t)n );
	f->axisBits[0] = b2CreateBitSet( 1024 );
	f->axisBits[1] = b2CreateBitSet( 1024 );
	b2fBuildWallTable( f );
	return f;
}

void b2DestroyFluid( b2FluidId fluidId )
{
	b2World* world = b2GetWorld( fluidId.world0 );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	b2FreeFluid( b2fGetFluid( fluidId ) );
	world->fluid = NULL;
	B2_REC( world, DestroyFluid, fluidId );
}

void b2FreeFluid( b2fFluid* f )
{
	size_t n = (size_t)f->capacity + 1;
#define B2F_FREE_FIELD( type, name ) b2Free( f->name, n * sizeof( type ) );
#define B2F_FREE_SORTED( type, name ) b2Free( f->sorted.name, n * sizeof( type ) );
	B2F_PARTICLE_FIELDS( B2F_FREE_FIELD )
	B2F_PARTICLE_FIELDS( B2F_FREE_SORTED )
	B2F_WORK_FIELDS( B2F_FREE_FIELD )
	if ( f->adaptMass != NULL )
	{
		B2F_CARRIED_FIELDS( B2F_FREE_FIELD )
		B2F_ADAPT_FIELDS( B2F_FREE_FIELD )
	}
	if ( f->speedMemory != NULL )
	{
		B2F_SPEED_FIELDS( B2F_FREE_FIELD )
		B2F_SPEED_FIELDS( B2F_FREE_SORTED )
	}
#undef B2F_FREE_FIELD
#undef B2F_FREE_SORTED
	b2Free( f->contacts, n * B2F_CONTACTS_PER_PARTICLE * sizeof( b2fContact ) );
	b2DestroyBitSet( &f->marked );
	if ( f->springs != NULL )
	{
		b2Free( f->springs, n * sizeof( b2fSprings ) );
		b2Free( f->sortedSprings, n * sizeof( b2fSprings ) );
		b2Free( f->idMap, IdMapSlots( f->capacity ) * sizeof( b2fIdSlot ) );
		b2Free( f->springParent, n * sizeof( int ) );
		b2Free( f->springGroup, n * sizeof( unsigned char ) );
	}
	b2fGrid* grids[2] = { &f->grid, &f->sleepGrid };
	for ( int i = 0; i < 2; ++i )
	{
		b2Array_Destroy( grids[i]->start );
		b2Array_Destroy( grids[i]->blockSize );
		b2Array_Destroy( grids[i]->blockReach );
		b2fFreeAxis( &grids[i]->ax );
		b2fFreeAxis( &grids[i]->ay );
	}
	b2Array_Destroy( f->cellAwake );
	b2Array_Destroy( f->pairs );
	b2Array_Destroy( f->shapes );
	b2Array_Destroy( f->gatherMark );
	b2Array_Destroy( f->bodies );
	b2Array_Destroy( f->bodyOfIndex );
	b2Array_Destroy( f->reactions );
	b2Array_Destroy( f->patches );
	b2Array_Destroy( f->cellHold );
	b2fShapeBins* bins[3] = { &f->shapeBins, &f->fixedBins, &f->sleepBins };
	for ( int i = 0; i < 3; ++i )
	{
		b2Array_Destroy( bins[i]->start );
		b2Array_Destroy( bins[i]->shapes );
	}
	b2Array_Destroy( f->bodyShapes );
	b2Array_Destroy( f->shapeMark );
	b2Array_Destroy( f->candidates );
	size_t boxes = (size_t)f->boxCapacity;
	b2Free( f->boxMass, boxes * sizeof( double ) );
	b2Free( f->boxPx, boxes * sizeof( double ) );
	b2Free( f->boxPy, boxes * sizeof( double ) );
	b2Free( f->boxWeight, boxes * sizeof( double ) );
	b2Free( f->boxMixed, boxes * sizeof( double ) );
	b2Free( f->boxLiquid, boxes );
	b2Free( f->boxNeeded, boxes );
	b2Free( f->boxPressure, boxes * sizeof( float ) );
	b2Free( f->boxDepth, boxes * sizeof( float ) );
	b2Free( f->boxFlow, boxes * sizeof( float ) );
	b2Array_Destroy( f->partList );
	b2Array_Destroy( f->partOf );
	b2Array_Destroy( f->partStart );
	for ( int i = 0; i < 2; ++i )
	{
		b2fFreeAxis( f->partAxes + i );
		b2DestroyBitSet( f->axisBits + i );
	}
	b2fFreeState( f );
	b2Free( f, sizeof( b2fFluid ) );
}

void b2Fluid_SetAdaptiveDepth( b2FluidId fluidId, float adaptiveDepth )
{
	B2_CHECK_INPUT( b2IsValidFloat( adaptiveDepth ) && adaptiveDepth >= 0.0f );
	b2fFluid* f = b2fGetFluid( fluidId );
	f->def.adaptiveDepth = adaptiveDepth;
	b2fWakeAll( f );
	B2_REC( f->world, FluidSetAdaptiveDepth, fluidId, adaptiveDepth );
}

float b2Fluid_GetAdaptiveDepth( b2FluidId fluidId )
{
	return b2fGetFluid( fluidId )->def.adaptiveDepth;
}

void b2Fluid_SetAdaptiveFullBox( b2FluidId fluidId, float adaptiveFullBox )
{
	B2_CHECK_INPUT( b2IsValidFloat( adaptiveFullBox ) && adaptiveFullBox > 0.0f );
	b2fFluid* f = b2fGetFluid( fluidId );
	f->def.adaptiveFullBox = adaptiveFullBox;
	b2fWakeAll( f );
	B2_REC( f->world, FluidSetAdaptiveFullBox, fluidId, adaptiveFullBox );
}

float b2Fluid_GetAdaptiveFullBox( b2FluidId fluidId )
{
	return b2fGetFluid( fluidId )->def.adaptiveFullBox;
}

void b2Fluid_EnableSleep( b2FluidId fluidId, bool flag )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	f->def.enableSleep = flag;
	if ( flag == false )
	{
		b2fWakeAll( f );
	}
	B2_REC( f->world, FluidEnableSleep, fluidId, flag );
}

bool b2Fluid_IsSleepEnabled( b2FluidId fluidId )
{
	return b2fGetFluid( fluidId )->def.enableSleep;
}

void b2Fluid_SetMaxCapacity( b2FluidId fluidId, int maxCapacity )
{
	B2_CHECK_INPUT( maxCapacity >= 0 );
	b2fFluid* f = b2fGetFluid( fluidId );
	f->def.maxCapacity = maxCapacity > f->capacity ? maxCapacity : f->capacity;
	B2_REC( f->world, FluidSetMaxCapacity, fluidId, maxCapacity );
}

int b2Fluid_GetMaxCapacity( b2FluidId fluidId )
{
	return b2fGetFluid( fluidId )->def.maxCapacity;
}

float b2Fluid_GetParticleSpacing( b2FluidId fluidId )
{
	return b2fGetFluid( fluidId )->def.particleSpacing;
}

int b2Fluid_GetCapacity( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->capacity;
}

// Resizes an array of count + 1 values to newCount + 1, zeroing the new part when the array started zeroed
static void* Resize( void* array, int count, int newCount, size_t size, bool zero )
{
	char* p = b2GrowAlloc( array, ( (size_t)count + 1 ) * size, ( (size_t)newCount + 1 ) * size );
	if ( zero )
	{
		memset( p + ( (size_t)count + 1 ) * size, 0, (size_t)( newCount - count ) * size );
	}
	return p;
}

// Every per-particle array is carried at the new size with what it holds, and the new room is as b2CreateFluid
// leaves it, so a fluid that grew is the same as one made that large. Nothing reads capacity in the passes, so the
// liquid moves the same.
bool b2fGrow( b2fFluid* f, int needed )
{
	if ( needed <= f->capacity )
	{
		return true;
	}
	if ( needed > f->def.maxCapacity )
	{
		return false;
	}

	int64_t grown = f->capacity > 0 ? f->capacity : 64;
	while ( grown < needed )
	{
		grown *= 2;
	}
	int old = f->capacity, capacity = grown < f->def.maxCapacity ? (int)grown : f->def.maxCapacity;

#define B2F_GROW_FIELD( type, name ) f->name = Resize( f->name, old, capacity, sizeof( type ), true );
#define B2F_GROW_SORTED( type, name ) f->sorted.name = Resize( f->sorted.name, old, capacity, sizeof( type ), true );
	B2F_PARTICLE_FIELDS( B2F_GROW_FIELD )
	B2F_PARTICLE_FIELDS( B2F_GROW_SORTED )
	B2F_WORK_FIELDS( B2F_GROW_FIELD )
	if ( f->adaptMass != NULL )
	{
		B2F_CARRIED_FIELDS( B2F_GROW_FIELD )
		B2F_ADAPT_FIELDS( B2F_GROW_FIELD )
	}
	if ( f->speedMemory != NULL )
	{
		B2F_SPEED_FIELDS( B2F_GROW_FIELD )
		B2F_SPEED_FIELDS( B2F_GROW_SORTED )
	}
#undef B2F_GROW_FIELD
#undef B2F_GROW_SORTED
	f->contacts = Resize( f->contacts, old, capacity, B2F_CONTACTS_PER_PARTICLE * sizeof( b2fContact ), true );

	// The spring arrays, made once some liquid has springs, and the id table, which is cleared before each use
	if ( f->springs != NULL )
	{
		f->springs = Resize( f->springs, old, capacity, sizeof( b2fSprings ), true );
		f->sortedSprings = Resize( f->sortedSprings, old, capacity, sizeof( b2fSprings ), true );
		b2Free( f->idMap, IdMapSlots( old ) * sizeof( b2fIdSlot ) );
		f->idMap = b2Alloc( IdMapSlots( capacity ) * sizeof( b2fIdSlot ) );
		f->springParent = Resize( f->springParent, old, capacity, sizeof( int ), false );
		f->springGroup = Resize( f->springGroup, old, capacity, sizeof( unsigned char ), false );
	}

	f->capacity = capacity;
	return true;
}

void b2fClearMarks( b2fFluid* f )
{
	b2SetBitCountAndClear( &f->marked, (uint32_t)f->count );
}

// Compaction keeps the order. The size pass, which removes particles itself, runs right after a substep's sort, so
// no grid has to outlast a removal; what B2F_CARRIED_FIELDS lists is read across one, so it goes with each particle.
int b2fRemoveMarked( b2fFluid* f )
{
	const b2BitSet* marked = &f->marked;
	int n = f->count;
	B2_ASSERT( (uint32_t)n <= 64 * marked->blockCount );

	// The particles before the first mark stay where they are
	int first = n;
	for ( uint32_t block = 0; block < marked->blockCount; ++block )
	{
		if ( marked->bits[block] != 0 )
		{
			int bit = (int)( 64 * block + b2CTZ64( marked->bits[block] ) );
			first = bit < n ? bit : n;
			break;
		}
	}

	// The awake particles still come first, fewer by those removed among them, and they and the asleep ones are filed
	// again before the next substep
	int active = f->activeCount < n ? f->activeCount : n, removed = 0, kept = first;
	for ( int i = first; i < n; ++i )
	{
		if ( b2GetBit( marked, (uint32_t)i ) )
		{
			removed += i < active ? 1 : 0;
			continue;
		}
		b2fCopyParticle( f, kept, i );
		if ( f->adaptMass != NULL )
		{
#define B2F_CARRY_FIELD( type, name ) f->name[kept] = f->name[i];
			B2F_CARRIED_FIELDS( B2F_CARRY_FIELD )
#undef B2F_CARRY_FIELD
		}
		kept += 1;
	}

	for ( int i = kept; i < n; ++i )
	{
		b2fClearParticle( f, i );
	}
	f->count = kept;
	f->gridStale = f->gridStale || kept < n;
	f->activeCount = active - removed;
	f->settledCount = kept - f->activeCount < f->settledCount ? kept - f->activeCount : f->settledCount;
	return n - kept;
}

bool b2fWorldLocked( b2FluidId fluidId )
{
	b2World* world = b2GetWorld( fluidId.world0 );
	B2_ASSERT( world->locked == false );
	return world->locked;
}

int b2fPickLiquid( b2fFluid* f, b2LiquidId liquidId )
{
	if ( B2_IS_NULL( liquidId ) )
	{
		return -1;
	}
	b2fFluid* other;
	int liquid = b2fGetLiquid( liquidId, &other );
	B2_ASSERT( other == f );
	(void)f;
	return liquid;
}

// Marks the particles a call picks, at most maxCount, the first in the fluid's order, and clears the mark on the rest
static void MarkInside( b2fFluid* f, int liquid, const b2fOutline* outline, int maxCount )
{
	b2fClearMarks( f );
	int count = 0;
	for ( int i = 0; i < f->count && count < maxCount; ++i )
	{
		if ( b2fPicks( f, liquid, outline, i ) )
		{
			b2SetBit( &f->marked, (uint32_t)i );
			count += 1;
		}
	}
}

int b2Fluid_Remove( b2FluidId fluidId, b2LiquidId liquidId, b2FluidRegion region, int maxCount )
{
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

	MarkInside( f, liquid, &outline, maxCount );

	// What is left around the gap wakes to fill it: every particle within the coarsest reach of the region
	float grow = 2.0f * f->maxRadius;
	b2fWakeBox( f, outline.minX - grow, outline.minY - grow, outline.maxX + grow, outline.maxY + grow );
	int removed = b2fRemoveMarked( f );
	B2_REC_CREATE( f->world, FluidRemove, removed, fluidId, liquidId, region, maxCount );
	return removed;
}

b2LiquidId b2fMakeLiquidId( const b2fFluid* f, int index )
{
	return ( b2LiquidId ){ index + 1, f->world->worldId, f->liquids[index].generation };
}

int b2fGetLiquid( b2LiquidId liquidId, b2fFluid** fluid )
{
	b2World* world = b2GetWorld( liquidId.world0 );
	b2fFluid* f = world->fluid;
	int index = liquidId.index1 - 1;
	B2_ASSERT( f != NULL && 0 <= index && index < f->liquidCount );
	B2_ASSERT( f->liquids[index].alive && liquidId.generation == f->liquids[index].generation );
	*fluid = f;
	return index;
}

bool b2Liquid_IsValid( b2LiquidId liquidId )
{
	const b2World* world = b2FindWorld( liquidId.world0 );
	if ( world == NULL || world->fluid == NULL )
	{
		return false;
	}

	const b2fFluid* f = world->fluid;
	int index = liquidId.index1 - 1;
	return 0 <= index && index < f->liquidCount && f->liquids[index].alive &&
		   liquidId.generation == f->liquids[index].generation;
}

b2LiquidId b2CreateLiquid( b2FluidId fluidId, const b2LiquidDef* def )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	if ( IsValidLiquidDef( def ) == false )
	{
		return b2_nullLiquidId;
	}

	b2World* world = b2GetWorld( fluidId.world0 );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return b2_nullLiquidId;
	}

	// The lowest free slot, so that a fluid can go on making and destroying liquids within B2_MAX_LIQUIDS. The lowest
	// also keeps the first liquid of a fluid the one the uniform density pass reads.
	int index = 0;
	while ( index < f->liquidCount && f->liquids[index].alive )
	{
		index += 1;
	}
	if ( index == B2F_MAX_LIQUIDS )
	{
		return b2_nullLiquidId;
	}

	f->liquidCount = index < f->liquidCount ? f->liquidCount : index + 1;
	f->liquids[index].def = *def;
	f->liquids[index].alive = true;
	world->liquidSerial += 1;
	f->liquids[index].generation = world->liquidSerial;
	for ( int other = 0; other < B2F_MAX_LIQUIDS; ++other )
	{
		f->cohesionBetween[index][other] = 0.0f;
		f->cohesionBetween[other][index] = 0.0f;
	}
	b2fUpdateLiquids( f );
	b2fWakeAll( f );
	b2LiquidId id = { index + 1, fluidId.world0, f->liquids[index].generation };
	B2_REC_CREATE( world, CreateLiquid, id, fluidId, *def );
	return id;
}

void b2DestroyLiquid( b2LiquidId liquidId )
{
	b2fFluid* f;
	int index = b2fGetLiquid( liquidId, &f );
	b2World* world = b2GetWorld( liquidId.world0 );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	b2fClearMarks( f );
	for ( int i = 0; i < f->count; ++i )
	{
		if ( f->liquid[i] == index )
		{
			b2SetBit( &f->marked, (uint32_t)i );
		}
	}
	b2fRemoveMarked( f );
	b2fDestroyEmitters( f, index );
	// A spawn waiting to coarsen goes with it, or the next liquid in the slot would coarsen all its own particles
	f->coarsenLiquids &= ~( 1u << index );

	f->liquids[index].alive = false;
	while ( f->liquidCount > 0 && f->liquids[f->liquidCount - 1].alive == false )
	{
		f->liquidCount -= 1;
	}
	b2fUpdateLiquids( f );
	b2fWakeAll( f );
	B2_REC( world, DestroyLiquid, liquidId );
}

void b2Liquid_SetDef( b2LiquidId liquidId, const b2LiquidDef* def )
{
	b2fFluid* f;
	int index = b2fGetLiquid( liquidId, &f );
	if ( IsValidLiquidDef( def ) == false )
	{
		return;
	}
	b2World* world = b2GetWorld( liquidId.world0 );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}
	f->liquids[index].def = *def;
	// Without adaptive sizes a spawn waiting to coarsen stays as it is
	if ( def->tuning.adaptiveRatio <= 1.0f )
	{
		f->coarsenLiquids &= ~( 1u << index );
	}
	b2fUpdateLiquids( f );
	b2fWakeAll( f );
	B2_REC( world, LiquidSetDef, liquidId, *def );
}

b2LiquidDef b2Liquid_GetDef( b2LiquidId liquidId )
{
	b2fFluid* f;
	int index = b2fGetLiquid( liquidId, &f );
	return f->liquids[index].def;
}

int b2Liquid_GetIndex( b2LiquidId liquidId )
{
	b2fFluid* f;
	return b2fGetLiquid( liquidId, &f );
}

int b2Fluid_GetLiquidCount( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	int count = 0;
	for ( int i = 0; i < f->liquidCount; ++i )
	{
		count += f->liquids[i].alive ? 1 : 0;
	}
	return count;
}

void b2Liquid_SetCohesionBetween( b2LiquidId liquidA, b2LiquidId liquidB, float cohesion )
{
	b2fFluid *f, *other;
	int a = b2fGetLiquid( liquidA, &f ), b = b2fGetLiquid( liquidB, &other );
	B2_ASSERT( f == other );
	B2_CHECK_INPUT( b2IsValidFloat( cohesion ) && cohesion >= 0.0f );
	if ( f == other && a != b )
	{
		f->cohesionBetween[a][b] = cohesion;
		f->cohesionBetween[b][a] = cohesion;
		b2fUpdateLiquids( f );
		b2fWakeAll( f );
		B2_REC( f->world, LiquidSetCohesionBetween, liquidA, liquidB, cohesion );
	}
}

int b2Fluid_GetParticleCount( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->count;
}

b2FluidParticles b2Fluid_GetParticles( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );

	return ( b2FluidParticles ){
		.count = f->count,
		.positions = f->pos,
		.velocitiesX = f->vx,
		.velocitiesY = f->vy,
		.sizes = f->size,
		.liquids = f->liquid,
		.densities = f->rho,
		.ids = f->id,
		.asleep = f->asleep,
		.restDensity = f->restDensity,
	};
}

b2FluidProfile b2Fluid_GetProfile( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->profile;
}

void b2PrepareFluid( b2fFluid* f, float timeStep, int subStepCount )
{
	uint64_t ticks = b2GetTicks();
	memset( &f->profile, 0, sizeof( f->profile ) );

	// Spawned particles waiting for their sizes get them first, at this step's substep, and settle into place, which
	// gathers the shapes itself, where they are now. Sleep comes first: a change of gravity wakes everything.
	uint64_t sleepTicks = b2GetTicks();
	b2fPrepareSleep( f, f->world->gravity );
	f->profile.sleep += b2GetMilliseconds( sleepTicks );
	f->worldGravity = f->world->gravity;
	if ( f->coarsenLiquids != 0 && timeStep > 0.0f )
	{
		b2fCoarsenSpawned( f, timeStep / (float)( subStepCount > 1 ? subStepCount : 1 ) );
	}
	b2fStepEmitters( f, timeStep );
	// What woke between steps wakes the rest of its spring group before the first substep predicts anything
	if ( f->springy && ( f->sleepOn || f->springAsleep != 0 ) )
	{
		sleepTicks = b2GetTicks();
		b2fBuildIdMap( f );
		b2fSpringGroups( f, false );
		b2fCountAsleep( f );
		f->profile.sleep += b2GetMilliseconds( sleepTicks );
	}
	b2fGatherShapes( f );

	f->stepTime = 0.0f;
	f->stepSubSteps = 0;
	f->subStepTime = 0.0f;
	f->profile.total = b2GetMilliseconds( ticks );
	f->profile.prepare = f->profile.total - f->profile.sleep;
}

// Sizes change once the step is done, reading the grid, densities, shapes and bodies of its last substep, so nothing
// the caller does between steps comes between the pass and what it reads, and the caller sees the sizes the next step
// moves. The step's reports come first, from the particles as the last substep left them.
void b2FinishFluid( b2fFluid* f )
{
	uint64_t start = b2GetTicks();
	b2fWakeReached( f );
	f->profile.sleep += b2GetMilliseconds( start );
	b2fFinishStep( f );

	uint64_t ticks = b2GetTicks();
	if ( b2fAdaptSizes( f ) )
	{
		f->profile.sizes = b2GetMilliseconds( ticks );
		f->profile.sizeChanges = f->sizeChanges;
		f->profile.measure = f->measureTime;
	}

	// The world's profile: the step timer stopped before this, and the substeps ran inside solve and its constraints
	// (or in solve alone while no body is awake), so the end of the step goes into step and the substeps come out of
	// those two, which leaves them to the bodies
	float finish = b2GetMilliseconds( start );
	float subSteps = f->subStepTime;
	f->profile.total += finish;
	b2Profile* p = &f->world->profile;
	p->step += finish;
	p->solve = p->solve > subSteps ? p->solve - subSteps : 0.0f;
	p->constraints = p->constraints > subSteps ? p->constraints - subSteps : 0.0f;
	p->fluid = f->profile.total;
	p->fluidPrepare = f->profile.prepare;
	p->fluidSort = f->profile.sort;
	p->fluidDensity = f->profile.density;
	p->fluidCollide = f->profile.collide;
	p->fluidSleep = f->profile.sleep;
	p->fluidSizes = f->profile.sizes;
}

void b2FluidSubStep( b2SubStepContext* context, int subStepIndex, float h, b2fFluid* f )
{
	f->checkSprings = subStepIndex == 0;
	f->stepTime += h;
	f->stepSubSteps += 1;
	if ( f->count == 0 )
	{
		return;
	}

	uint64_t start = b2GetTicks();
	uint64_t ticks = start;
	float sleepStart = f->profile.sleep;

	// Gravity cannot change during a step, so it is still the one b2PrepareFluid read
	f->h = h;

	b2fUpdateBodies( f );
	bool awakeBody = false;
	for ( int i = 0; i < f->bodies.count && awakeBody == false; ++i )
	{
		awakeBody = f->bodies.data[i].awake;
	}

	// Asleep particles are filed apart from the awake ones once some fall asleep or wake, and those near an awake body
	// wake before anything moves, and are filed with the awake ones (fluid_sleep.c)
	if ( b2fNeedsFiling( f ) )
	{
		b2fFileAsleep( f, context );
	}
	if ( awakeBody && f->settledCount > 0 )
	{
		uint64_t sleepTicks = b2GetTicks();
		bool woke = b2fWakeHeld( f );
		f->profile.sleep += b2GetMilliseconds( sleepTicks );
		if ( woke )
		{
			b2fFileAsleep( f, context );
		}
	}

	if ( f->activeCount == 0 )
	{
		b2fSortAwake( f, context );
		// All asleep, and nothing has moved since they were filed. Only shapes and bodies can wake the liquid, and they
		// change only between steps, apart from awake bodies, which move in every substep. So the first substep of a
		// step looks for them, and the rest only while some body near the liquid is awake.
		if ( subStepIndex > 0 && awakeBody == false )
		{
			float time = b2GetMilliseconds( start );
			f->profile.total += time;
			f->profile.sort += time - ( f->profile.sleep - sleepStart );
			f->subStepTime += time;
			return;
		}
	}
	else
	{
		b2fPredict( f, context );
		b2fSortAwake( f, context );
	}
	f->profile.sort += b2GetMillisecondsAndReset( &ticks ) - ( f->profile.sleep - sleepStart );

	// The sleep passes inside density and collide (b2fFindShell, b2fSpringGroups) time themselves into sleep
	float sleepBefore = f->profile.sleep;
	b2fFindContacts( f, context, subStepIndex == 0 );
	if ( f->springy )
	{
		b2fRelaxSprings( f, context );
	}
	b2fRelaxDensity( f, context );
	f->profile.density += b2GetMillisecondsAndReset( &ticks ) - ( f->profile.sleep - sleepBefore );

	sleepBefore = f->profile.sleep;
	b2fCollide( f, context );
	b2fUpdateVelocities( f, context );
	f->profile.collide += b2GetMillisecondsAndReset( &ticks ) - ( f->profile.sleep - sleepBefore );
	float time = b2GetMilliseconds( start );
	f->profile.total += time;
	f->subStepTime += time;
}

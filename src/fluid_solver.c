// SPDX-License-Identifier: MIT
// Double density relaxation (Clavet et al. 2005) with viscosity, run as parallel passes over sorted particles. Viscosity
// shares the move pass and the pairs the density pass found. Pressure can be kept from pulling, with cohesion as a
// separate pull. Liquids with springs get Clavet's springs and plasticity in a pass of their own before it.
#include "fluid_internal.h"

// The density excess that pressure acts on. Below rest density only the tensile share of it pulls.
static inline float TensileClamp( float excess, float tensile )
{
	return excess > 0.0f ? excess : tensile * excess;
}

// Densities for one particle, from current positions, keeping each neighbour within reach for the move pass. A pair
// uses the mean of the two sizes and weights the other particle by its area, which keeps moves equal and opposite in
// momentum when sizes differ. A neighbour of a liquid this particle's liquid does not push on is not counted and not
// kept, so no pair move, viscosity or cohesion acts between them either.
static inline void Density( b2fFluid* f, int i, float* density, float* nearDensity )
{
	b2fCoord x = f->pos[i].x, y = f->pos[i].y;
	float si = f->size[i], a = 0.0f, b = 0.0f;
	uint32_t sees = f->liquidSees[f->liquid[i]];
	bool all = sees == UINT32_MAX;
	int* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
	int n = 0;
	FOR_NEIGHBOURS( f, i, j )
	{
		float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, sj = f->size[j], h = b2fLatticeSize( si + sj );
		if ( d2 < h * h && j != i && ( all || ( sees >> f->liquid[j] & 1u ) != 0 ) )
		{
			float mean = 0.5f * ( si + sj ), w = sj * sj / ( mean * mean );
			float l = sqrtf( d2 ), q = 1.0f - l / h;
			a += w * q * q;
			b += w * q * q * q;
			if ( n < B2F_PAIRS_PER_PARTICLE )
			{
				pairs[n] = j;
			}
			n += 1;
		}
	}
	f->pairCount[i] = n <= B2F_PAIRS_PER_PARTICLE ? n : -1;
	*density = a + f->wallDensity[i];
	*nearDensity = b + f->wallNearDensity[i];
}

// Shapes cannot move out of the way, so the particle takes the whole of the move. Below rest
// density the pressure part is negative, but a wall does not pull on a particle. Adhesion does: Akinci et al. 2013's
// spline (Eq. 7), which pulls a particle between h/2 and h from the shape and peaks at 3h/4, scaled so that its peak
// is 1. Only shapes that forces do not move, static and kinematic, pull: at bodies the pull also lifted crates
// heavier than the liquid off the floor.
static inline void ApplyWallMoves( b2fFluid* f, int i, float pressure, float nearPressure, float adhesion, float h )
{
	float limit = 0.5f * b2fLatticeSize( f->size[i] );
	b2fContact* contacts = f->contacts + B2F_CONTACTS_PER_PARTICLE * i;
	for ( int c = 0; c < f->contactCount[i]; ++c )
	{
		float move = pressure * contacts[c].pressureWeight + nearPressure * contacts[c].nearWeight;
		move = move < 0.0f ? 0.0f : ( move > limit ? limit : move );
		if ( adhesion > 0.0f && f->bodies.data[contacts[c].bodyIndex].adheres )
		{
			float d = ( 1.0f - contacts[c].share ) * h;
			float spline = -4.0f * d * d / h + 6.0f * d - 2.0f * h;
			if ( d > 0.5f * h && spline > 0.0f )
			{
				move -= B2F_ADHESION_SCALE * adhesion * h * sqrtf( sqrtf( spline / ( 0.25f * h ) ) );
			}
		}
		contacts[c].move = move;
		f->pos[i].x += move * contacts[c].normalX;
		f->pos[i].y += move * contacts[c].normalY;
	}
}

// After the density pass found a particle's neighbours, drops the springs the spring pass marked as gone, and adds one
// at the current distance to each neighbour of the same liquid it has none to. The neighbour does the same from its
// side. Clavet's Algorithm 4 starts a spring at the interaction radius, where it has no force until plasticity
// shortens it; starting at the distance instead lets an elastic liquid, with no plasticity, hold the shape it had.
static void AddSprings( b2fFluid* f, int i )
{
	b2fSprings* sp = f->springs + i;
	if ( f->liquids[f->liquid[i]].def.springStiffness <= 0.0f )
	{
		sp->count = 0;
		return;
	}
	// Asleep in the shell, its springs stay as they fell asleep; none forms between an asleep and an awake particle,
	// which would join an awake group to an asleep one
	if ( f->asleep[i] != 0 )
	{
		return;
	}

	int kept = 0;
	for ( int s = 0; s < sp->count; ++s )
	{
		if ( sp->rest[s] >= 0.0f )
		{
			sp->id[kept] = sp->id[s];
			sp->rest[kept] = sp->rest[s];
			kept += 1;
		}
	}
	sp->count = kept;

	// A particle with more neighbours than the list holds (pairCount -1) takes springs from the ones the list has,
	// which are more than it can hold springs to
	int n = f->pairCount[i] >= 0 ? f->pairCount[i] : B2F_PAIRS_PER_PARTICLE;
	const int* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
	b2fCoord x = f->pos[i].x, y = f->pos[i].y;
	// A liquid without formSpringsAfterSpawn forms springs only where one end is new. Both ends read the flags the
	// last substep left, so they agree.
	bool forming = f->liquids[f->liquid[i]].def.formSpringsAfterSpawn || sp->formed == false;
	for ( int p = 0; p < n && sp->count < B2F_SPRINGS_PER_PARTICLE; ++p )
	{
		int j = pairs[p], id = f->id[j];
		if ( f->liquid[j] != f->liquid[i] || f->asleep[j] != 0 || ( forming == false && f->springs[j].formed ) )
		{
			continue;
		}

		bool found = false;
		for ( int s = 0; s < sp->count && found == false; ++s )
		{
			found = sp->id[s] == id;
		}
		if ( found == false )
		{
			// The distance the density pass found, from the same positions
			float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y );
			sp->id[sp->count] = id;
			sp->rest[sp->count] = sqrtf( dx * dx + dy * dy );
			sp->count += 1;
		}
	}
}

// The same for one liquid with uniform sizes, where every pair's reach is the interaction radius and weighs 1. It
// gives the same densities up to rounding, and is kept because it is faster: without it and its move pass below, the two passes
// took 12% longer in the Tank and 17% longer in the Pool with uniform sizes.
static inline void UniformDensity( b2fFluid* f, int i, float* density, float* nearDensity )
{
	float h = f->radius, h2 = h * h, inv = 1.0f / h;
	b2fCoord x = f->pos[i].x, y = f->pos[i].y;
	float a = 0.0f, b = 0.0f;
	int* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
	int n = 0;
	FOR_NEIGHBOURS( f, i, j )
	{
		float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy;
		if ( d2 < h2 && j != i )
		{
			float l = sqrtf( d2 ), q = 1.0f - l * inv;
			float q2 = q * q;
			a += q2;
			b += q2 * q;
			if ( n < B2F_PAIRS_PER_PARTICLE )
			{
				pairs[n] = j;
			}
			n += 1;
		}
	}
	f->pairCount[i] = n <= B2F_PAIRS_PER_PARTICLE ? n : -1;
	*density = a + f->wallDensity[i];
	*nearDensity = b + f->wallNearDensity[i];
}

// One liquid with uniform sizes runs its own density and move passes
static inline bool UsesUniformPasses( const b2fFluid* f )
{
	return f->liquidCount <= 1 && f->adaptive == false;
}

// The move pass for one liquid with uniform sizes. It gives the same moves as the general pass up to rounding, faster.
static void UniformAccumulate( b2fFluid* f, int start, int end )
{
	const b2LiquidDef* def = &f->liquids[0].def;
	float h = f->radius, h2 = h * h, inv = 1.0f / h;
	float scale = def->tuning.stiffness * h, nearScale = def->tuning.nearStiffness * h, rest = 2.0f * f->restDensity;
	float tensile = def->tuning.tensilePressure, single = f->restDensity, pullScale = def->cohesion * h;
	bool clamped = tensile < 1.0f;
	float sigma = def->viscosity, beta = def->tuning.quadraticViscosity, dt = f->h;
	bool viscous = sigma > 0.0f || beta > 0.0f;
	for ( int i = start; i < end; ++i )
	{
		if ( f->asleep[i] != 0 )
		{
			continue;
		}
		b2fCoord x = f->pos[i].x, y = f->pos[i].y;
		float moveX = 0.0f, moveY = 0.0f;
		float rhoI = f->rho[i], nearI = f->rhoNear[i], pressureI = TensileClamp( rhoI - single, tensile );
		float vx = f->vx[i], vy = f->vy[i], dvx = 0.0f, dvy = 0.0f;
#define B2F_UNIFORM_PAIR( j )                                                                                        \
	{                                                                                                                  \
		float l = sqrtf( d2 ), q = 1.0f - l * inv;                                                                     \
		float excess = clamped ? pressureI + TensileClamp( f->rho[j] - single, tensile ) : rhoI + f->rho[j] - rest;   \
		float pull = pullScale * q * ( 1.0f - 2.0f * q );                                                              \
		float d = 0.5f * ( scale * excess * q + nearScale * ( nearI + f->rhoNear[j] ) * q * q - pull ) / l;            \
		moveX -= d * dx;                                                                                               \
		moveY -= d * dy;                                                                                               \
		float u = ( ( vx - f->vx[j] ) * dx + ( vy - f->vy[j] ) * dy ) / l;                                            \
		if ( viscous && u > 0.0f )                                                                                     \
		{                                                                                                              \
			float impulse = 0.5f * dt * q * ( sigma * u + beta * u * u ) / l;                                          \
			dvx -= impulse * dx;                                                                                       \
			dvy -= impulse * dy;                                                                                       \
		}                                                                                                              \
	}
		int n = f->pairCount[i];
		if ( n >= 0 )
		{
			const int* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
			for ( int p = 0; p < n; ++p )
			{
				int j = pairs[p];
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy;
				if ( d2 > 1e-12f )
				{
					B2F_UNIFORM_PAIR( j )
				}
			}
		}
		else
		{
			FOR_NEIGHBOURS( f, i, j )
			{
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy;
				if ( d2 < h2 && d2 > 1e-12f && j != i )
				{
					B2F_UNIFORM_PAIR( j )
				}
			}
		}
#undef B2F_UNIFORM_PAIR
		f->dx[i] = moveX;
		f->dy[i] = moveY;
		f->viscX[i] = dvx;
		f->viscY[i] = dvy;
	}
}

// Whether a shell particle, asleep beside awake liquid or against a shape, should wake: its density has moved from what
// it was in its first substep asleep, or an awake neighbour within reach is fast (fluid_sleep.c)
static bool ShellWakes( b2fFluid* f, int i )
{
	// Just asleep, this density is the one it wakes by. The bit goes at the end of the substep, since neighbours read
	// asleep in this pass.
	if ( f->asleep[i] & B2F_ASLEEP_NEW )
	{
		f->calm[i].x = f->rho[i];
	}
	else if ( b2AbsFloat( f->rho[i] - f->calm[i].x ) > B2F_WAKE_DENSITY * f->restDensity )
	{
		return true;
	}

	float fast2 = B2F_WAKE_SPEED * B2F_WAKE_SPEED;
	int n = f->pairCount[i];
	if ( n >= 0 )
	{
		const int* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
		for ( int p = 0; p < n; ++p )
		{
			int j = pairs[p];
			if ( f->asleep[j] == 0 && f->vx[j] * f->vx[j] + f->vy[j] * f->vy[j] > fast2 )
			{
				return true;
			}
		}
		return false;
	}

	// Too many to keep: any in the window counts
	FOR_NEIGHBOURS( f, i, j )
	{
		if ( f->asleep[j] == 0 && f->vx[j] * f->vx[j] + f->vy[j] * f->vy[j] > fast2 )
		{
			return true;
		}
	}
	return false;
}

// The density pass runs over the awake particles and then the shell; the other asleep particles skip it, and keep the
// density they last had
static inline int DensityParticle( const b2fFluid* f, int k )
{
	return k < f->activeCount ? k : f->shell[k - f->activeCount];
}

// Each particle's pressure and near pressure, from its own liquid's settings, scale with its size squared
static void DensityTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	bool springy = f->springy;
	if ( UsesUniformPasses( f ) )
	{
		for ( int k = start; k < end; ++k )
		{
			int i = DensityParticle( f, k );
			UniformDensity( f, i, f->rho + i, f->rhoNear + i );
			if ( springy )
			{
				AddSprings( f, i );
			}
			if ( f->asleep[i] != 0 && ShellWakes( f, i ) )
			{
				f->wake[i] |= B2F_WAKE_NOW;
			}
		}
		return;
	}

	float rest = f->restDensity;
	for ( int k = start; k < end; ++k )
	{
		int i = DensityParticle( f, k );
		Density( f, i, f->rho + i, f->rhoNear + i );
		const b2LiquidDef* def = &f->liquids[f->liquid[i]].def;
		float hi = 2.0f * b2fLatticeSize( f->size[i] ), hi2 = hi * hi;
		f->pressure[i] = def->tuning.stiffness * hi2 * TensileClamp( f->rho[i] - rest, def->tuning.tensilePressure );
		f->nearPressure[i] = def->tuning.nearStiffness * hi2 * f->rhoNear[i];
		if ( springy )
		{
			AddSprings( f, i );
		}
		if ( f->asleep[i] != 0 && ShellWakes( f, i ) )
		{
			f->wake[i] |= B2F_WAKE_NOW;
		}
	}
}

// The move from pressure, near pressure and cohesion, and the change of velocity from viscosity. A pair takes its
// cohesion, viscosity and mass weight from the table for the two liquids, which keeps the pair's moves equal and
// opposite in momentum when the liquids differ in density.
static void AccumulateTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	if ( UsesUniformPasses( f ) )
	{
		UniformAccumulate( f, start, end );
		return;
	}

	float dt = f->h;
	for ( int i = start; i < end; ++i )
	{
		if ( f->asleep[i] != 0 )
		{
			continue;
		}
		b2fCoord x = f->pos[i].x, y = f->pos[i].y;
		float si = f->size[i], moveX = 0.0f, moveY = 0.0f;
		float pressure = f->pressure[i], nearPressure = f->nearPressure[i];
		float vx = f->vx[i], vy = f->vy[i], dvx = 0.0f, dvy = 0.0f;
		const b2fLiquidPair* row = f->liquidPairs[f->liquid[i]];
		// The density pass kept only neighbours this particle's liquid pushes on; a particle with too many to keep
		// scans the grid, and skips the others here
		uint32_t sees = f->liquidSees[f->liquid[i]];
		bool all = sees == UINT32_MAX;
#define B2F_PAIR( j )                                                                                                \
	{                                                                                                                  \
		const b2fLiquidPair* pair = row + f->liquid[j];                                                                \
		float sj = f->size[j], h = b2fLatticeSize( si + sj ), l = sqrtf( d2 );                                         \
		float q = 1.0f - l / h, mean = 0.5f * ( si + sj ), w = pair->weight * sj * sj / ( mean * mean );             \
		float pull = pair->cohesion * h * h * q * ( 1.0f - 2.0f * q );                                                 \
		float d = 0.5f * w * ( ( pressure + f->pressure[j] ) * q + ( nearPressure + f->nearPressure[j] ) * q * q - pull ) / ( h * l ); \
		moveX -= d * dx;                                                                                               \
		moveY -= d * dy;                                                                                               \
		float u = ( ( vx - f->vx[j] ) * dx + ( vy - f->vy[j] ) * dy ) / l;                                            \
		if ( u > 0.0f )                                                                                                \
		{                                                                                                              \
			float impulse = 0.5f * w * dt * q * ( pair->viscosity * u + pair->quadraticViscosity * u * u ) / l;          \
			dvx -= impulse * dx;                                                                                       \
			dvy -= impulse * dy;                                                                                       \
		}                                                                                                              \
	}
		int n = f->pairCount[i];
		if ( n >= 0 )
		{
			const int* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
			for ( int p = 0; p < n; ++p )
			{
				int j = pairs[p];
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy;
				if ( d2 > 1e-12f )
				{
					B2F_PAIR( j )
				}
			}
		}
		else
		{
			FOR_NEIGHBOURS( f, i, j )
			{
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, reach = b2fLatticeSize( si + f->size[j] );
				if ( d2 < reach * reach && d2 > 1e-12f && j != i && ( all || ( sees >> f->liquid[j] & 1u ) != 0 ) )
				{
					B2F_PAIR( j )
				}
			}
		}
#undef B2F_PAIR
		f->dx[i] = moveX;
		f->dy[i] = moveY;
		f->viscX[i] = dvx;
		f->viscY[i] = dvy;
	}
}

static void ApplyTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float dt = f->h;
	float restRatio = 1.0f + B2F_SQUEEZE_SURFACE, invRest = 1.0f / f->restDensity;
	for ( int i = start; i < end; ++i )
	{
		if ( f->asleep[i] != 0 )
		{
			continue;
		}
		float radius = 2.0f * b2fLatticeSize( f->size[i] ), limit = B2F_MOVE_LIMIT * radius;
		float d2 = f->dx[i] * f->dx[i] + f->dy[i] * f->dy[i];
		if ( d2 > limit * limit )
		{
			float scale = limit / sqrtf( d2 );
			f->dx[i] *= scale;
			f->dy[i] *= scale;
		}
		// Viscosity changes velocity, and the position follows it for the rest of the substep. Velocity itself is
		// derived from the change in position at the end of the substep, so only the position needs the change.
		f->pos[i].x += f->dx[i] + dt * f->viscX[i];
		f->pos[i].y += f->dy[i] + dt * f->viscY[i];

		// A particle weighs what the area it fills would weigh at rest. Fine particles deep down pack up to a third
		// tighter than coarse ones or ones near the surface, so by count the liquid would be heavier there, and a
		// body would float or sink by where it is. Prediction moved the particle by its full weight; this takes back
		// the share its packing adds. Looser than rest, as in spray, it keeps its full weight.
		float squeeze = f->rho[i] * invRest - restRatio;
		if ( squeeze > 0.0f )
		{
			float extra = B2F_FILL_SLOPE * squeeze;
			float lighter = extra / ( B2F_REST_FILL + extra ) * dt * dt * f->liquids[f->liquid[i]].def.gravityScale;
			f->pos[i].x -= lighter * f->worldGravity.x;
			f->pos[i].y -= lighter * f->worldGravity.y;
		}
		if ( f->contactCount[i] > 0 )
		{
			const b2LiquidDef* def = &f->liquids[f->liquid[i]].def;
			float excess = TensileClamp( f->rho[i] - f->restDensity, def->tuning.tensilePressure );
			ApplyWallMoves( f, i, def->tuning.stiffness * radius * excess, def->tuning.nearStiffness * radius * f->rhoNear[i], def->adhesion, radius );
		}
	}
}

static void PredictTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->h;
	for ( int i = start; i < end; ++i )
	{
		// Asleep, it holds still, at zero velocity
		if ( f->asleep[i] != 0 )
		{
			f->prev[i] = f->pos[i];
			continue;
		}
		float scale = h * f->liquids[f->liquid[i]].def.gravityScale;
		f->vx[i] += scale * f->worldGravity.x;
		f->vy[i] += scale * f->worldGravity.y;
		f->prev[i].x = f->pos[i].x;
		f->prev[i].y = f->pos[i].y;
		f->pos[i].x += h * f->vx[i];
		f->pos[i].y += h * f->vy[i];
	}
}

// At the end of the substep the shell's particles that fell asleep in the last one are no longer new, and those to wake
// wake. One woken by touching an awake body (ContactTask) is awake already, and takes part from the next substep.
static void ShellEndTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	for ( int k = start; k < end; ++k )
	{
		int i = f->shell[k];
		if ( f->asleep[i] != 0 )
		{
			f->asleep[i] = (unsigned char)( f->asleep[i] & ~B2F_ASLEEP_NEW );
			if ( f->wake[i] & B2F_WAKE_NOW )
			{
				b2fWakeParticle( f, i );
			}
		}
	}
}

static void VelocityTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->h, inv = 1.0f / h, limit = 0.5f * f->radius, limit2 = limit * limit;
	float sleep2 = B2F_SLEEP_SPEED * B2F_SLEEP_SPEED, smooth = h < B2F_CALM_TIME ? h / B2F_CALM_TIME : 1.0f;
	uint32_t springLiquids = f->springLiquids;
	bool sleepOn = f->sleepOn;
	for ( int i = start; i < end; ++i )
	{
		if ( f->asleep[i] != 0 )
		{
			f->asleep[i] = (unsigned char)( f->asleep[i] & ~B2F_ASLEEP_NEW );
			if ( f->wake[i] & B2F_WAKE_NOW )
			{
				b2fWakeParticle( f, i );
			}
			continue;
		}

		float dx = (float)( f->pos[i].x - f->prev[i].x ), dy = (float)( f->pos[i].y - f->prev[i].y ), d2 = dx * dx + dy * dy;
		if ( d2 > limit2 )
		{
			float scale = limit / sqrtf( d2 );
			dx *= scale;
			dy *= scale;
			f->pos[i].x = f->prev[i].x + dx;
			f->pos[i].y = f->prev[i].y + dy;
		}
		f->vx[i] = dx * inv;
		f->vy[i] = dy * inv;
		if ( f->springs != NULL )
		{
			f->springs[i].formed = true;
		}

		// Slow for B2_TIME_TO_SLEEP, by its velocity smoothed over B2F_CALM_TIME, and touching no awake body, it falls
		// asleep where it is, or with springs, once all its group is ready too (b2fSpringGroups)
		if ( sleepOn == false )
		{
			f->sleepTime[i] = 0.0f;
			continue;
		}
		b2Vec2 calm = f->calm[i];
		calm.x += smooth * ( f->vx[i] - calm.x );
		calm.y += smooth * ( f->vy[i] - calm.y );
		f->calm[i] = calm;
		f->sleepTime[i] = b2LengthSquared( calm ) < sleep2 ? f->sleepTime[i] + h : 0.0f;
		bool ready = f->sleepTime[i] >= B2_TIME_TO_SLEEP && ( f->wake[i] & B2F_WAKE_HELD ) == 0;
		if ( ready && ( springLiquids >> f->liquid[i] & 1u ) != 0 )
		{
			f->wake[i] |= B2F_WAKE_CALM;
		}
		else if ( ready )
		{
			f->asleep[i] = (unsigned char)( B2F_ASLEEP | B2F_ASLEEP_NEW | ( f->contactCount[i] > 0 ? B2F_ASLEEP_SHAPE : 0u ) );
			f->vx[i] = 0.0f;
			f->vy[i] = 0.0f;
			f->calm[i] = b2Vec2_zero;
		}
	}
}

// Springs, Clavet et al. 2005 sections 5.1 and 5.2. Each particle moves by its share of each spring's stretch,
// w_i / (w_i + w_j + 1 / (k dt^2)) as in one iteration of XPBD (Macklin et al. 2016), so a spring is as stiff at any
// substep count. w is the inverse of a particle's mass in finest particles, and k the liquid's spring stiffness
// times Clavet's 1 - L / h, which weakens springs as their rest length L nears the interaction radius h. Then the
// rest length follows the length by Clavet's equation 8, past the yield ratio, and a spring whose rest length
// reaches h is marked gone, as is one whose partner has gone, holds no spring back, or is now another liquid.
// Both ends read the same positions and rest length, so they work a spring out the same way.
//
// A spring with both ends asleep is skipped: neither moves, so neither its pull nor its rest length would change.
// Particles joined by springs fall asleep and wake together (b2fSpringGroups), so the ends of a spring are never one
// asleep and one awake, which would work the spring from one side only and let the two copies part.
static void SpringTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float dt = f->h, fine = f->def.particleSpacing;
	bool check = f->checkSprings;
	for ( int i = start; i < end; ++i )
	{
		float moveX = 0.0f, moveY = 0.0f;
		b2fSprings* sp = f->springs + i;
		const b2LiquidDef* def = &f->liquids[f->liquid[i]].def;
		if ( def->springStiffness > 0.0f && f->asleep[i] == 0 )
		{
			b2fCoord x = f->pos[i].x, y = f->pos[i].y;
			float si = f->size[i], wi = fine * fine / ( si * si );
			int self = f->id[i];
			for ( int s = 0; s < sp->count; ++s )
			{
				float L = sp->rest[s];
				int j = L >= 0.0f ? b2fFindParticle( f, sp->id[s] ) : -1;
				// Only the first substep of a step checks the partner holds the spring too, which costs as much as
				// the rest. A spring held at one end comes from a partner with no free slot, and lasts one step.
				// The partner asleep with this end awake can only be a spring it does not hold, which goes, since
				// those it holds would have woken it
				bool mutual = j >= 0 && f->liquid[j] == f->liquid[i] && f->asleep[j] == 0 && check == false;
				if ( j >= 0 && f->liquid[j] == f->liquid[i] && f->asleep[j] == 0 && check )
				{
					const b2fSprings* other = f->springs + j;
					for ( int t = 0; t < other->count && mutual == false; ++t )
					{
						mutual = other->id[t] == self;
					}
				}
				if ( mutual == false )
				{
					sp->rest[s] = -1.0f;
					continue;
				}

				float sj = f->size[j], h = b2fLatticeSize( si + sj );
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), r = sqrtf( dx * dx + dy * dy );
				float k = def->springStiffness * ( 1.0f - L / h );
				if ( k > 0.0f && r > 1e-6f )
				{
					float wj = fine * fine / ( sj * sj );
					float share = wi / ( wi + wj + 1.0f / ( k * dt * dt ) );
					float move = share * ( r - L ) / r;
					moveX += move * dx;
					moveY += move * dy;
				}

				float yield = def->yieldRatio * L;
				if ( r > L + yield )
				{
					L += dt * def->plasticity * ( r - L - yield );
				}
				else if ( r < L - yield )
				{
					L -= dt * def->plasticity * ( L - yield - r );
				}
				sp->rest[s] = L < h ? L : -1.0f;
			}
		}
		f->dx[i] = moveX;
		f->dy[i] = moveY;
	}
}

static void SpringApplyTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	for ( int i = start; i < end; ++i )
	{
		float cap = B2F_MOVE_LIMIT * 2.0f * b2fLatticeSize( f->size[i] );
		float d2 = f->dx[i] * f->dx[i] + f->dy[i] * f->dy[i], scale = d2 > cap * cap ? cap / sqrtf( d2 ) : 1.0f;
		f->pos[i].x += scale * f->dx[i];
		f->pos[i].y += scale * f->dy[i];
	}
}

// Where each id is now, in a table at least twice the particle count, filled in index order
void b2fBuildIdMap( b2fFluid* f )
{
	int slots = 2, bits = 1;
	while ( slots < 2 * f->count )
	{
		slots *= 2;
		bits += 1;
	}
	f->idMapMask = slots - 1;
	f->idMapShift = 32 - bits;
	memset( f->idMap, 0xFF, (size_t)slots * sizeof( b2fIdSlot ) );
	for ( int i = 0; i < f->count; ++i )
	{
		unsigned int slot = b2fIdHash( f, f->id[i] );
		while ( f->idMap[slot].id != -1 )
		{
			slot = ( slot + 1 ) & (unsigned int)f->idMapMask;
		}
		f->idMap[slot] = ( b2fIdSlot ){ f->id[i], i };
	}
}

void b2fRelaxSprings( b2fFluid* f, b2SubStepContext* context )
{
	b2fBuildIdMap( f );
	// Particles that woke since the last substep, touching an awake body in the contact pass among them, wake their
	// groups before any spring is worked out
	if ( f->sleepOn || f->springAsleep != 0 )
	{
		uint64_t ticks = b2GetTicks();
		b2fSpringGroups( f, false );
		f->profile.sleep += b2GetMilliseconds( ticks );
	}
	b2SubStep_ParallelFor( context, SpringTask, f->activeCount, 256, f );
	b2SubStep_ParallelFor( context, SpringApplyTask, f->activeCount, 2048, f );
}

void b2fPredict( b2fFluid* f, b2SubStepContext* context )
{
	// Just filed, the awake particles are not yet first
	b2SubStep_ParallelFor( context, PredictTask, f->filePending ? f->count : f->activeCount, 1024, f );
}

void b2fRelaxDensity( b2fFluid* f, b2SubStepContext* context )
{
	b2SubStep_ParallelFor( context, DensityTask, f->activeCount + f->shellCount, 256, f );
	b2SubStep_ParallelFor( context, AccumulateTask, f->activeCount, 256, f );
	b2SubStep_ParallelFor( context, ApplyTask, f->activeCount, 2048, f );
}

void b2fUpdateVelocities( b2fFluid* f, b2SubStepContext* context )
{
	b2SubStep_ParallelFor( context, VelocityTask, f->activeCount, 2048, f );
	uint64_t ticks = b2GetTicks();
	b2SubStep_ParallelFor( context, ShellEndTask, f->shellCount, 2048, f );
	// Groups whose particles are all ready fall asleep together, and those of which some woke in this substep wake
	// together, before the next substep works out their springs
	if ( f->springy && ( f->sleepOn || f->springAsleep != 0 ) )
	{
		b2fSpringGroups( f, true );
	}
	if ( f->sleepOn || f->asleepCount > 0 )
	{
		b2fCountAsleep( f );
	}
	f->profile.sleep += b2GetMilliseconds( ticks );
}

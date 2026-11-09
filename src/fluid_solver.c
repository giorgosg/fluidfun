// SPDX-License-Identifier: MIT
// Double density relaxation (Clavet et al. 2005) with viscosity, run as parallel passes over sorted particles. Viscosity
// shares the move pass and the pairs the density pass found. Pressure can be kept from pulling, with cohesion as a
// separate pull.
#include "fluid_internal.h"

// The density excess that pressure acts on. Below rest density only the tensile share of it pulls.
static inline float TensileClamp( float excess, float tensile )
{
	return excess > 0.0f ? excess : tensile * excess;
}

// Densities and pressures for one particle use current positions. Also keeps each neighbour within reach for the move pass.
static inline void Density( b2fFluid* f, int i, float* density, float* nearDensity )
{
	float h = f->radius, h2 = h * h, inv = 1.0f / h;
	b2fCoord x = f->pos[i].x, y = f->pos[i].y;
	float a = 0.0f, b = 0.0f;
	b2fPair* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
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
				pairs[n] = ( b2fPair ){ j, l };
			}
			n += 1;
		}
	}
	f->pairCount[i] = n <= B2F_PAIRS_PER_PARTICLE ? n : -1;
	*density = a + f->wallDensity[i];
	*nearDensity = b + f->wallNearDensity[i];
}

// Shapes cannot move out of the way, so the particle takes the whole of the move. Below rest
// density the pressure part is negative, but a wall does not pull on a particle.
static inline void ApplyWallMoves( b2fFluid* f, int i, float pressure, float nearPressure )
{
	float limit = 0.5f * f->size[i];
	b2fContact* contacts = f->contacts + B2F_CONTACTS_PER_PARTICLE * i;
	for ( int c = 0; c < f->contactCount[i]; ++c )
	{
		float move = pressure * contacts[c].pressureWeight + nearPressure * contacts[c].nearWeight;
		move = move < 0.0f ? 0.0f : ( move > limit ? limit : move );
		contacts[c].move = move;
		f->pos[i].x += move * contacts[c].normalX;
		f->pos[i].y += move * contacts[c].normalY;
	}
}

// With sizes that differ, a pair uses the mean size and weights the other particle by its area, which keeps
// moves equal and opposite in momentum. Each particle's own pressure scales with its own size squared.
static inline void AdaptiveDensity( b2fFluid* f, int i, float* density, float* nearDensity )
{
	b2fCoord x = f->pos[i].x, y = f->pos[i].y;
	float si = f->size[i], a = 0.0f, b = 0.0f;
	b2fPair* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
	int n = 0;
	FOR_NEIGHBOURS( f, i, j )
	{
		float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, sj = f->size[j], h = si + sj;
		if ( d2 < h * h && j != i )
		{
			float mean = 0.5f * h, w = sj * sj / ( mean * mean );
			float l = sqrtf( d2 ), q = 1.0f - l / h;
			a += w * q * q;
			b += w * q * q * q;
			if ( n < B2F_PAIRS_PER_PARTICLE )
			{
				pairs[n] = ( b2fPair ){ j, l };
			}
			n += 1;
		}
	}
	f->pairCount[i] = n <= B2F_PAIRS_PER_PARTICLE ? n : -1;
	*density = a + f->wallDensity[i];
	*nearDensity = b + f->wallNearDensity[i];
}

static void AdaptiveAccumulate( b2fFluid* f, int start, int end )
{
	float k = f->def.stiffness, kn = f->def.nearStiffness, rest = f->restDensity;
	float tensile = f->def.tensilePressure, cohesion = f->def.cohesion;
	float sigma = f->def.linearViscosity, beta = f->def.quadraticViscosity, dt = f->h;
	bool viscous = sigma > 0.0f || beta > 0.0f;
	for ( int i = start; i < end; ++i )
	{
		b2fCoord x = f->pos[i].x, y = f->pos[i].y;
		float si = f->size[i], moveX = 0.0f, moveY = 0.0f;
		float hi2 = 4.0f * si * si;
		float pressure = k * hi2 * TensileClamp( f->rho[i] - rest, tensile ), nearPressure = kn * hi2 * f->rhoNear[i];
		float vx = f->vx[i], vy = f->vy[i], dvx = 0.0f, dvy = 0.0f;
#define B2F_ADAPTIVE_PAIR( j, l )                                                                                    \
	{                                                                                                                  \
		float sj = f->size[j], h = si + sj;                                                                            \
		float q = 1.0f - l / h, mean = 0.5f * h, w = sj * sj / ( mean * mean ), hj2 = 4.0f * sj * sj;                \
		float pj = k * hj2 * TensileClamp( f->rho[j] - rest, tensile ), nj = kn * hj2 * f->rhoNear[j];                \
		float pull = cohesion * h * h * q * ( 1.0f - 2.0f * q );                                                       \
		float d = 0.5f * w * ( ( pressure + pj ) * q + ( nearPressure + nj ) * q * q - pull ) / ( h * l );            \
		moveX -= d * dx;                                                                                               \
		moveY -= d * dy;                                                                                               \
		float u = ( ( vx - f->vx[j] ) * dx + ( vy - f->vy[j] ) * dy ) / l;                                            \
		if ( viscous && u > 0.0f )                                                                                     \
		{                                                                                                              \
			float impulse = 0.5f * w * dt * q * ( sigma * u + beta * u * u ) / l;                                      \
			dvx -= impulse * dx;                                                                                       \
			dvy -= impulse * dy;                                                                                       \
		}                                                                                                              \
	}
		int n = f->pairCount[i];
		if ( n >= 0 )
		{
			const b2fPair* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
			for ( int p = 0; p < n; ++p )
			{
				int j = pairs[p].j;
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, l = pairs[p].l;
				if ( d2 > 1e-12f )
				{
					B2F_ADAPTIVE_PAIR( j, l )
				}
			}
		}
		else
		{
			FOR_NEIGHBOURS( f, i, j )
			{
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, sj = f->size[j], h = si + sj;
				if ( d2 < h * h && d2 > 1e-12f && j != i )
				{
					float l = sqrtf( d2 );
					B2F_ADAPTIVE_PAIR( j, l )
				}
			}
		}
#undef B2F_ADAPTIVE_PAIR
		f->dx[i] = moveX;
		f->dy[i] = moveY;
		f->viscX[i] = dvx;
		f->viscY[i] = dvy;
	}
}

static void DensityTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	if ( f->def.adaptiveRatio > 1.0f )
	{
		for ( int i = start; i < end; ++i )
		{
			AdaptiveDensity( f, i, f->rho + i, f->rhoNear + i );
		}
		return;
	}

	for ( int i = start; i < end; ++i )
	{
		Density( f, i, f->rho + i, f->rhoNear + i );
	}
}

static void AccumulateTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	if ( f->def.adaptiveRatio > 1.0f )
	{
		AdaptiveAccumulate( f, start, end );
		return;
	}

	float h = f->radius, h2 = h * h, inv = 1.0f / h;
	float scale = f->def.stiffness * h, nearScale = f->def.nearStiffness * h, rest = 2.0f * f->restDensity;
	float tensile = f->def.tensilePressure, single = f->restDensity, pullScale = f->def.cohesion * h;
	bool clamped = tensile < 1.0f;
	float sigma = f->def.linearViscosity, beta = f->def.quadraticViscosity, dt = f->h;
	bool viscous = sigma > 0.0f || beta > 0.0f;
	for ( int i = start; i < end; ++i )
	{
		b2fCoord x = f->pos[i].x, y = f->pos[i].y;
		float moveX = 0.0f, moveY = 0.0f;
		float rhoI = f->rho[i], nearI = f->rhoNear[i], pressureI = TensileClamp( rhoI - single, tensile );
		float vx = f->vx[i], vy = f->vy[i], dvx = 0.0f, dvy = 0.0f;
#define B2F_UNIFORM_PAIR( j, l )                                                                                     \
	{                                                                                                                  \
		float q = 1.0f - l * inv;                                                                                      \
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
			const b2fPair* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
			for ( int p = 0; p < n; ++p )
			{
				int j = pairs[p].j;
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, l = pairs[p].l;
				if ( d2 > 1e-12f )
				{
					B2F_UNIFORM_PAIR( j, l )
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
					float l = sqrtf( d2 );
					B2F_UNIFORM_PAIR( j, l )
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

static void ApplyTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float limit = f->moveLimit * f->radius, limit2 = limit * limit, dt = f->h;
	bool adaptive = f->def.adaptiveRatio > 1.0f;
	for ( int i = start; i < end; ++i )
	{
		if ( adaptive )
		{
			limit = f->moveLimit * 2.0f * f->size[i];
			limit2 = limit * limit;
		}

		if ( limit > 0.0f )
		{
			float d2 = f->dx[i] * f->dx[i] + f->dy[i] * f->dy[i];
			if ( d2 > limit2 )
			{
				float scale = limit / sqrtf( d2 );
				f->dx[i] *= scale;
				f->dy[i] *= scale;
			}
		}
		// Viscosity changes velocity, and the position follows it for the rest of the substep. Velocity itself is
		// derived from the change in position at the end of the substep, so only the position needs the change.
		f->pos[i].x += f->dx[i] + dt * f->viscX[i];
		f->pos[i].y += f->dy[i] + dt * f->viscY[i];
		if ( f->contactCount[i] > 0 )
		{
			float h = adaptive ? 2.0f * f->size[i] : f->radius;
			float excess = TensileClamp( f->rho[i] - f->restDensity, f->def.tensilePressure );
			ApplyWallMoves( f, i, f->def.stiffness * h * excess, f->def.nearStiffness * h * f->rhoNear[i] );
		}
	}
}

static void PredictTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->h, gx = h * f->gravityX, gy = h * f->gravityY;
	for ( int i = start; i < end; ++i )
	{
		f->vx[i] += gx;
		f->vy[i] += gy;
		f->prev[i].x = f->pos[i].x;
		f->prev[i].y = f->pos[i].y;
		f->pos[i].x += h * f->vx[i];
		f->pos[i].y += h * f->vy[i];
	}
}

static void VelocityTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->h, inv = 1.0f / h, limit = 0.5f * f->radius, limit2 = limit * limit;
	for ( int i = start; i < end; ++i )
	{
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
	}
}

void b2fPredict( b2fFluid* f, b2SubStepContext* context )
{
	b2SubStep_ParallelFor( context, PredictTask, f->count, 1024, f );
}

void b2fRelaxDensity( b2fFluid* f, b2SubStepContext* context )
{
	b2SubStep_ParallelFor( context, DensityTask, f->count, 512, f );
	b2SubStep_ParallelFor( context, AccumulateTask, f->count, 512, f );
	b2SubStep_ParallelFor( context, ApplyTask, f->count, 2048, f );
}

void b2fUpdateVelocities( b2fFluid* f, b2SubStepContext* context )
{
	b2SubStep_ParallelFor( context, VelocityTask, f->count, 2048, f );
}

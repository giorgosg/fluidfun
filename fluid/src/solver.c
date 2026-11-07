// SPDX-License-Identifier: MIT
// Double density relaxation (Clavet et al. 2005) with viscosity, run as parallel passes over sorted particles.
#include "internal.h"

// Densities and pressures for one particle use current positions.
static inline void Density( const b2fFluid* f, int i, float* density, float* nearDensity )
{
	float h = f->radius, h2 = h * h, inv = 1.0f / h, x = f->x[i], y = f->y[i];
	float a = 0.0f, b = 0.0f;
	FOR_NEIGHBOURS( f, i, j )
	{
		float dx = f->x[j] - x, dy = f->y[j] - y, d2 = dx * dx + dy * dy;
		if ( d2 < h2 && j != i )
		{
			float q = 1.0f - sqrtf( d2 ) * inv;
			float q2 = q * q;
			a += q2;
			b += q2 * q;
		}
	}
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
		f->x[i] += move * contacts[c].normalX;
		f->y[i] += move * contacts[c].normalY;
	}
}

// With sizes that differ, a pair uses the mean size and weights the other particle by its area, which keeps
// moves equal and opposite in momentum. Each particle's own pressure scales with its own size squared.
static inline void AdaptiveDensity( const b2fFluid* f, int i, float* density, float* nearDensity )
{
	float x = f->x[i], y = f->y[i], si = f->size[i], a = 0.0f, b = 0.0f;
	FOR_NEIGHBOURS( f, i, j )
	{
		float dx = f->x[j] - x, dy = f->y[j] - y, d2 = dx * dx + dy * dy, sj = f->size[j], h = si + sj;
		if ( d2 < h * h && j != i )
		{
			float mean = 0.5f * h, w = sj * sj / ( mean * mean );
			float q = 1.0f - sqrtf( d2 ) / h;
			a += w * q * q;
			b += w * q * q * q;
		}
	}
	*density = a + f->wallDensity[i];
	*nearDensity = b + f->wallNearDensity[i];
}

static void AdaptiveAccumulate( b2fFluid* f, int start, int end )
{
	float k = f->def.stiffness, kn = f->def.nearStiffness, rest = f->restDensity;
	for ( int i = start; i < end; ++i )
	{
		float x = f->x[i], y = f->y[i], si = f->size[i], moveX = 0.0f, moveY = 0.0f;
		float hi2 = 4.0f * si * si;
		float pressure = k * hi2 * ( f->rho[i] - rest ), nearPressure = kn * hi2 * f->rhoNear[i];
		FOR_NEIGHBOURS( f, i, j )
		{
			float dx = f->x[j] - x, dy = f->y[j] - y, d2 = dx * dx + dy * dy, sj = f->size[j], h = si + sj;
			if ( d2 < h * h && d2 > 1e-12f && j != i )
			{
				float l = sqrtf( d2 ), q = 1.0f - l / h, mean = 0.5f * h, w = sj * sj / ( mean * mean ), hj2 = 4.0f * sj * sj;
				float pj = k * hj2 * ( f->rho[j] - rest ), nj = kn * hj2 * f->rhoNear[j];
				float d = 0.5f * w * ( ( pressure + pj ) * q + ( nearPressure + nj ) * q * q ) / ( h * l );
				moveX -= d * dx;
				moveY -= d * dy;
			}
		}
		f->dx[i] = moveX;
		f->dy[i] = moveY;
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
	for ( int i = start; i < end; ++i )
	{
		float x = f->x[i], y = f->y[i], moveX = 0.0f, moveY = 0.0f;
		float rhoI = f->rho[i], nearI = f->rhoNear[i];
		FOR_NEIGHBOURS( f, i, j )
		{
			float dx = f->x[j] - x, dy = f->y[j] - y, d2 = dx * dx + dy * dy;
			if ( d2 < h2 && d2 > 1e-12f && j != i )
			{
				float l = sqrtf( d2 ), q = 1.0f - l * inv;
				float d = 0.5f * ( scale * ( rhoI + f->rho[j] - rest ) * q + nearScale * ( nearI + f->rhoNear[j] ) * q * q ) / l;
				moveX -= d * dx;
				moveY -= d * dy;
			}
		}
		f->dx[i] = moveX;
		f->dy[i] = moveY;
	}
}

static void ApplyTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float limit = f->moveLimit * f->radius, limit2 = limit * limit;
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
		f->x[i] += f->dx[i];
		f->y[i] += f->dy[i];
		if ( f->contactCount[i] > 0 )
		{
			float h = adaptive ? 2.0f * f->size[i] : f->radius;
			ApplyWallMoves( f, i, f->def.stiffness * h * ( f->rho[i] - f->restDensity ), f->def.nearStiffness * h * f->rhoNear[i] );
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
		f->px[i] = f->x[i];
		f->py[i] = f->y[i];
		f->x[i] += h * f->vx[i];
		f->y[i] += h * f->vy[i];
	}
}

// Writes each particle's own change only, so the result does not depend on how the loop is split.
static void ViscosityTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float radius = f->radius, h2 = radius * radius, inv = 1.0f / radius;
	float sigma = f->def.linearViscosity, beta = f->def.quadraticViscosity, h = f->h;
	bool adaptive = f->def.adaptiveRatio > 1.0f;
	for ( int i = start; i < end; ++i )
	{
		float x = f->x[i], y = f->y[i], vx = f->vx[i], vy = f->vy[i], dvx = 0.0f, dvy = 0.0f;
		float si = f->size[i];
		FOR_NEIGHBOURS( f, i, j )
		{
			float dx = f->x[j] - x, dy = f->y[j] - y, d2 = dx * dx + dy * dy, w = 1.0f;
			if ( adaptive )
			{
				float sj = f->size[j], mean = 0.5f * ( si + sj );
				radius = si + sj;
				h2 = radius * radius;
				inv = 1.0f / radius;
				w = sj * sj / ( mean * mean );
			}

			if ( d2 < h2 && d2 > 1e-12f && j != i )
			{
				float l = sqrtf( d2 ), q = 1.0f - l * inv;
				float u = ( ( vx - f->vx[j] ) * dx + ( vy - f->vy[j] ) * dy ) / l;
				if ( u > 0.0f )
				{
					float impulse = 0.5f * w * h * q * ( sigma * u + beta * u * u ) / l;
					dvx -= impulse * dx;
					dvy -= impulse * dy;
				}
			}
		}
		f->dx[i] = dvx;
		f->dy[i] = dvy;
	}
}

static void ApplyViscosityTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->h;
	for ( int i = start; i < end; ++i )
	{
		f->vx[i] += f->dx[i];
		f->vy[i] += f->dy[i];
		f->x[i] += h * f->dx[i];
		f->y[i] += h * f->dy[i];
	}
}

static void VelocityTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->h, inv = 1.0f / h, limit = 0.5f * f->radius, limit2 = limit * limit;
	for ( int i = start; i < end; ++i )
	{
		float dx = f->x[i] - f->px[i], dy = f->y[i] - f->py[i], d2 = dx * dx + dy * dy;
		if ( d2 > limit2 )
		{
			float scale = limit / sqrtf( d2 );
			dx *= scale;
			dy *= scale;
			f->x[i] = f->px[i] + dx;
			f->y[i] = f->py[i] + dy;
		}
		f->vx[i] = dx * inv;
		f->vy[i] = dy * inv;
	}
}

void b2fPredict( b2fFluid* f, b2SubStepContext* context )
{
	b2SubStep_ParallelFor( context, PredictTask, f->count, 1024, f );
}

void b2fApplyViscosity( b2fFluid* f, b2SubStepContext* context )
{
	if ( f->def.linearViscosity > 0.0f || f->def.quadraticViscosity > 0.0f )
	{
		b2SubStep_ParallelFor( context, ViscosityTask, f->count, 512, f );
		b2SubStep_ParallelFor( context, ApplyViscosityTask, f->count, 2048, f );
	}
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

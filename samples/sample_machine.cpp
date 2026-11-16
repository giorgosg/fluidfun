// SPDX-License-Identifier: MIT
#include "draw.h"
#include "machine.h"
#include "sample.h"

#include <imgui.h>

// A marble run through sections that each show something: liquids of several kinds, and plain bodies and joints.
// The camera follows whatever is acting. Each section is also a sample by itself, behind a start ramp.
class SampleMachine : public Sample
{
public:
	SampleMachine( SampleContext* context, MachineSectionType only )
		: Sample( context )
	{
		static bool first = true;
		if ( first )
		{
			s_def = DefaultMachineDef();
			first = false;
		}

		MachineDef def = s_def;
		def.only = only;
		CreateMachine( &m_machine, m_worldId, &def );
		if ( m_context->restart == false || s_follow )
		{
			m_context->camera.center = b2ToPos( m_machine.viewCenter );
			m_context->camera.zoom = m_machine.viewHalfHeight;
		}
	}

	~SampleMachine() override
	{
		b2DestroyFluid( m_machine.fluid );
	}

	void Step() override
	{
		if ( m_context->pause == false || m_context->singleStep > 0 )
		{
			StepMachine( &m_machine, 1.0f / m_context->hertz );
		}

		if ( s_follow )
		{
			m_context->camera.center = b2ToPos( m_machine.viewCenter );
			m_context->camera.zoom = m_machine.viewHalfHeight;
		}

		Sample::Step();

		// Water, oil, honey, mercury, jelly and dough. Speed lightens each toward white.
		static const float tints[machine_liquidCount][3] = { { 40.0f, 110.0f, 255.0f },	 { 235.0f, 200.0f, 60.0f }, { 225.0f, 130.0f, 20.0f },
															 { 170.0f, 175.0f, 190.0f }, { 220.0f, 50.0f, 90.0f },	{ 215.0f, 190.0f, 150.0f } };
		b2FluidParticles particles = b2Fluid_GetParticles( m_machine.fluid );
		float pixelsPerMetre = 0.5f * m_context->camera.height / m_context->camera.zoom;
		for ( int i = 0; i < particles.count; ++i )
		{
			float speed = b2MinFloat( b2Length( { particles.velocitiesX[i], particles.velocitiesY[i] } ) / 8.0f, 1.0f );
			const float* tint = tints[particles.liquids[i] % machine_liquidCount];
			uint32_t red = (uint32_t)( tint[0] + ( 255.0f - tint[0] ) * speed );
			uint32_t green = (uint32_t)( tint[1] + ( 255.0f - tint[1] ) * speed );
			uint32_t blue = (uint32_t)( tint[2] + ( 255.0f - tint[2] ) * speed );
			float size = b2MaxFloat( s_dotScale * particles.sizes[i] * pixelsPerMetre, 1.5f );
			DrawPoint( m_context->draw, particles.positions[i], size, b2HexColor( ( red << 16 ) | ( green << 8 ) | blue ) );
		}

		if ( m_machine.finished && s_loop && m_machine.finishedFor > 2.0f )
		{
			m_context->restart = true;
		}
	}

	bool DrawControls() override
	{
		const MachineSection* section = m_machine.sections + m_machine.active;
		ImGui::Text( "%s, step %d", MachineSectionName( section->type ), section->phase );
		ImGui::Text( "Particles %d", b2Fluid_GetParticleCount( m_machine.fluid ) );
		b2FluidProfile profile = b2Fluid_GetProfile( m_machine.fluid );
		ImGui::Text( "Fluid %.2f ms", profile.total );

		ImGui::Checkbox( "Camera follows", &s_follow );
		ImGui::Checkbox( "Start again at the end", &s_loop );
		bool view = ImGui::SliderFloat( "View height", &s_def.viewHalfHeight, 3.0f, 30.0f, "%.1f" );
		view |= ImGui::SliderFloat( "Camera spring Hz", &s_def.viewHertz, 0.1f, 3.0f, "%.2f" );
		if ( view )
		{
			m_machine.def.viewHalfHeight = s_def.viewHalfHeight;
			m_machine.def.viewHertz = s_def.viewHertz;
		}
		ImGui::SliderFloat( "Dot size", &s_dotScale, 0.3f, 2.0f, "%.2f" );

		// One liquid's settings at a time
		static const char* names[machine_liquidCount] = { "Water", "Oil", "Honey", "Mercury", "Jelly", "Dough" };
		ImGui::Combo( "Liquid", &s_liquid, names, machine_liquidCount );
		b2LiquidDef* liquid = s_def.liquids + s_liquid;
		bool changed = false;
		changed |= ImGui::SliderFloat( "Viscosity", &liquid->viscosity, 0.0f, 40.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Cohesion", &liquid->cohesion, 0.0f, 0.5f, "%.3f" );
		changed |= ImGui::SliderFloat( "Adhesion", &liquid->adhesion, 0.0f, 0.5f, "%.3f" );
		changed |= ImGui::SliderFloat( "Friction", &liquid->friction, 0.0f, 0.95f, "%.2f" );
		changed |= ImGui::SliderFloat( "Stiffness", &liquid->tuning.stiffness, 0.0f, 0.2f, "%.3f" );
		changed |= ImGui::SliderFloat( "Spring stiffness", &liquid->springStiffness, 0.0f, 100000.0f, "%.0f", ImGuiSliderFlags_Logarithmic );
		changed |= ImGui::SliderFloat( "Plasticity", &liquid->plasticity, 0.0f, 5.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Yield ratio", &liquid->yieldRatio, 0.0f, 0.5f, "%.2f" );
		changed |= ImGui::SliderFloat( "Adaptive ratio", &liquid->tuning.adaptiveRatio, 1.0f, 30.0f, "%.1f" );
		if ( changed )
		{
			b2Liquid_SetDef( m_machine.liquids[s_liquid], liquid );
		}

		// These need the machine built again
		bool rebuild = ImGui::SliderFloat( "Liquid density", &liquid->density, 0.25f, 20.0f, "%.2f", ImGuiSliderFlags_Logarithmic );
		rebuild |= ImGui::SliderFloat( "Spacing", &s_def.fluid.particleSpacing, 0.04f, 0.2f, "%.3f" );
		rebuild |= ImGui::SliderFloat( "Marble radius", &s_def.marbleRadius, 0.15f, 0.5f, "%.2f" );
		rebuild |= ImGui::SliderFloat( "Marble density", &s_def.marbleDensity, 0.1f, 4.0f, "%.2f" );
		if ( ImGui::Button( "Defaults" ) )
		{
			s_def = DefaultMachineDef();
			rebuild = true;
		}
		if ( rebuild )
		{
			m_context->restart = true;
		}
		return false;
	}

	template <MachineSectionType only>
	static Sample* Create( SampleContext* context )
	{
		return new SampleMachine( context, only );
	}

	static inline MachineDef s_def;
	static inline bool s_follow = true;
	static inline bool s_loop = true;
	static inline float s_dotScale = 1.0f;
	static inline int s_liquid = 0;

	Machine m_machine;
};

static int sampleMachine = RegisterSample( "Machine", "Whole Machine", SampleMachine::Create<section_typeCount> );
static int sampleMachineSluice = RegisterSample( "Machine", "Sluice And Wheel", SampleMachine::Create<section_sluice> );
static int sampleMachineDominoes = RegisterSample( "Machine", "Dominoes", SampleMachine::Create<section_dominoes> );
static int sampleMachineFerry = RegisterSample( "Machine", "Ferry", SampleMachine::Create<section_ferry> );
static int sampleMachineJellyBall = RegisterSample( "Machine", "Jelly Ball", SampleMachine::Create<section_jellyBall> );
static int sampleMachineCradle = RegisterSample( "Machine", "Cradle", SampleMachine::Create<section_cradle> );
static int sampleMachineDough = RegisterSample( "Machine", "Dough And Jelly", SampleMachine::Create<section_jelly> );
static int sampleMachineHoney = RegisterSample( "Machine", "Honey Balance", SampleMachine::Create<section_honey> );
static int sampleMachinePegs = RegisterSample( "Machine", "Peg Board", SampleMachine::Create<section_pegs> );
static int sampleMachineMercury = RegisterSample( "Machine", "Mercury Lift", SampleMachine::Create<section_mercury> );
static int sampleMachineFinale = RegisterSample( "Machine", "Finale", SampleMachine::Create<section_finale> );

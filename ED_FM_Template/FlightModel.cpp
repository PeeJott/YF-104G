#include "FlightModel.h"


FlightModel::FlightModel
(
	State& state,
	Input& input, 
	Engine& engine //NEU 21Feb21
	//der letzte Eintrag hier darf kein Komma haben!
) :
	m_state(state),
	m_input(input),
	m_engine(engine), //NEU 21Feb21

	//-----"init" Tables AeroData_1.h + FlightModel.h-----------
	//-----------------Pitch------------------------------------
	Cmalpha(DAT_Cma_SL, CON_Cma_SL_min, CON_Cma_SL_max),
	Cmde(DAT_Cmde_SL, CON_Cmde_SL_min, CON_Cmde_SL_max),
	Cmq(DAT_Cmq, CON_Cmq_min, CON_Cmq_max),
	Cmadot(DAT_Cmadot_SL, CON_Cmadot_SL_min, CON_Cmadot_SL_max),
	CmM(DAT_CmM, CON_Cmq_min, CON_Cmq_max),
	//----------------DRAG---------------------------------------
	CDmin(DAT_CDmin, CON_CDminmin, CON_CDminmax),
	CDmach(DAT_CDmach_SL, CON_CDmach_SLmin, CON_CDmach_SLmax),
	CDa(DAT_CDa_SL, CON_CDa_SLmin, CON_CDa_SLmax),
	//---------------LIFT----------------------------------------
	CLmax(DAT_CLmax, CON_CLmaxmin, CON_CLmaxmax),
	CLmach(DAT_CLmach_SL, CON_CLmach_SLmin, CON_CLmach_SLmax),
	CLa(DAT_CLa, CON_CLamin, CON_CLamax),
	//--------------ROLL----------------------------------------
	Clb(DAT_Clb_SL, CON_Clb_SLmin, CON_Clb_SLmax),
	Clp(DAT_Clp_SL, CON_Clp_SLmin, CON_Clp_SLmax),
	Clda(DAT_Clda, CON_Cldamin, CON_Cldamax),
	Clr(DAT_Clr_SL, CON_Clr_SLmin, CON_Clr_SLmax),
	Cldr(DAT_Cldr_SL, CON_Cldr_SLmin, CON_Cldr_SLmax),
	//---------------YAW---------------------------------------
	Cnb(DAT_Cnb_SL, CON_Cnb_SLmin, CON_Cnb_SLmax),
	Cndr(DAT_Cndr, CON_Cndrmin, CON_Cndrmax),
	Cnr(DAT_Cnr, CON_Cnrmin, CON_Cnrmax)
	//---------------Thrust------------------------------------
	//PMax(DAT_PMax, CON_PMaxmin, CON_PMaxmax)
	//PFor(DAT_PFor, CON_PFormin, CON_PFormax)
	//der letzte Eintrag darf KEIN Komma haben...



	//genau so für jede DATA Table (DATA, min , max)
{
	//printf("Hello\n");
}

void FlightModel::L_stab()
{
	//set roll moment -- Neu eingefügt am 14.02.2021 PJ-- "-" vor Clr eingefügt, da Clr Daten positiv waren und negativ sein müssten da Dämpfung
	//m_moment.x-- "2 *" vor Clda eingefügt für stärkere Ailerons = schon besser-- "2*" vor Clp eingefügt -- "0,5 *" vor Clb eingefügt
	m_moment.x += m_q * (Clb(m_state.m_mach) * m_state.m_beta + Clda(m_state.m_mach) * m_input.m_roll + Cldr(m_state.m_mach) * m_input.m_yaw)
		+ 0.25 * m_state.m_airDensity * m_scalarVelocity * CON_A * CON_b * CON_b * (2 * Clp(m_state.m_mach) * m_state.m_omega.x + -Clr(m_state.m_mach) * m_state.m_omega.y);
}

void FlightModel::M_stab()
{
	//set pitch moment-- "-" vor Cmde eingefügt, da positiver Wert erwartet--
	m_moment.z += m_k * CON_mac * (Cmalpha(m_state.m_mach) * m_state.m_aoa + -Cmde(m_state.m_mach) * m_input.m_pitch) 
			+ 0.25 * m_state.m_airDensity * m_scalarVelocity * CON_A * CON_mac * CON_mac * (Cmq(m_state.m_mach)*m_state.m_omega.z + Cmadot(m_state.m_mach) * m_aoaDot);
}

void FlightModel::N_stab()
{
	//set yaw moment-- "Cnda * da" ausgelassen, da wegen gegenläufiger ailerons geringfügig (Buch Seite 114) "Cnp * Pstab" ausgelassen, da ggf. unnötig 
	// "-" vor Cndr eingefügt, da "Rudereffektivität" positiv sein müsste-- "-" vor Cnb eingefügt, da Dämpfung-- "2 *" vor Cnr eingefügt für mehr Dämpfung
	//moment.y
	m_moment.y += m_q * (-Cnb(m_state.m_mach) * m_state.m_beta + -Cndr(m_state.m_mach) * -m_input.m_yaw)
		+ 0.25 * m_state.m_airDensity * m_scalarVelocity * CON_A * CON_b * CON_b * (2 * Cnr(m_state.m_mach) * m_state.m_omega.y);
}


void FlightModel::lift()
{
	//set lift -- eingefügt am 16.02. als "Lift = 0.5 * p * V² * s * CL
	//approx m_force.y
	// erster Versuch : m_force.y = m_k * (CLmach(m_state.m_mach) + CLa(m_state.m_aoa)); //Lift ist so schon gut ;-)
	m_force.y += m_k * (CLmach(m_state.m_mach) + (CLa(m_state.m_mach) * m_state.m_aoa));
}

void FlightModel::drag()
{
	//set drag--eingefügt 16.02. es fehlt noch supersonic drag, gear-drag, flap-drag, brake-drag
	//approx m_force.x negative
	//erster Versuch: m_force.x = -(m_k * (CDmach(m_state.m_mach) + CDa(m_state.m_aoa)
		//+ ((CLmach(m_state.m_mach) + CLa(m_state.m_mach)) * (CLmach(m_state.m_mach) + CLa(m_state.m_mach))) / CON_pi * CON_AR * CON_e));
	m_force.x += -m_k * ((CDmin(m_state.m_mach)) + (CDa(m_state.m_mach) * m_state.m_aoa)+ CON_CDeng); // +CDwave + CDi); CDwave und CDi wieder dazu, wenn DRAG geklärt.
}

void FlightModel::sideForce()
{
	//set side force
	//m_force.z
}

void FlightModel::thrustForce()
{
	//set thrust force
	//m_force.x positive
	//m_engine.update(); //neu eingefügt// und wieder zum testen auskommentiert 
	m_force.x += m_engine.getThrust(); //m_engine.getThrust(); //m_engine.m_thrust geht nicht, da m_thrust "private" in engine.h, daher durch Funktion aufrufen
	//printf("vector %d \n", m_force.x); //neu eingebaut für Ausgabe
}

void FlightModel::update(double dt)
{
	m_moment = Vec3();
	m_force = Vec3();

	m_aoaDot = (m_state.m_aoa - m_aoaPrevious) / dt;
	m_aoaPrevious = m_state.m_aoa;

	m_scalarVelocity = magnitude(m_state.m_localAirspeed);
	m_scalarVelocitySquared = m_scalarVelocity * m_scalarVelocity;


	m_k = 0.5 * m_state.m_airDensity * m_scalarVelocitySquared * CON_A;
	m_p = 0.25 * m_state.m_airDensity * m_scalarVelocity * CON_A * CON_b * CON_b;
	m_q = m_k * CON_b;

	m_state.m_mach = m_scalarVelocity / m_state.m_speedOfSound;
	
	M_mcrit = m_state.m_mach / CON_Mcrit; //Mach/Machcrit //neu eingefügt 18.02.2021
	M_mcrit_b = pow(M_mcrit, CON_wdb); //M_mcrit ^ CON_wdb //neu eingefügt 18.02.2021

	CDwave = CON_wda * M_mcrit_b; //neu eingefügt 18.02.2021
	
	CDi = ((CLmach(m_state.m_mach) * CLmach(m_state.m_mach)) / CON_pi * CON_A * CON_e );

	
	L_stab();
	M_stab();
	N_stab();
	lift();
	drag();
	sideForce();
	thrustForce();
}


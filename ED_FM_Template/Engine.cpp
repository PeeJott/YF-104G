#include "Engine.h"

Engine::Engine
(
	State& state,
	Input& input
	
) :
	m_state(state),
	m_input(input),
	//---------------Thrust------------------------------------
	PMax(DAT_PMax, CON_PMaxmin, CON_PMaxmax),
	PFor(DAT_PFor, CON_PFormin, CON_PFormax)
	//der letzte Eintrag darf KEIN Komma haben...
{
	//huhu!!
}

void Engine::zeroInit()
{
	m_scalarVelocity = 0.0;
	m_scalarVelocitySquared = 0.0;
	//-------------Engine Values/Commands----------------------------
	m_thrust = 0.0;
	m_throttle = 0.0;
	m_burner = 0.0;
	m_fuelFlow = 0.0;
	m_correctedFuelFlow = 0.0;
	m_hasFuel = true;
	m_ignitors = false;
}

void Engine::coldInit()
{
	zeroInit();
}

void Engine::hotInit()
{
	zeroInit();

	m_ignitors = true;
	m_rpmNormal = 0.70;
	m_input.m_engine_start = 1;

}

void Engine::airborneInit()
{
	zeroInit();

	m_ignitors = true;
	m_rpmNormal = 0.85;
	m_input.m_engine_start = 1;
}

void Engine::update(double dt)
{
	if (m_input.m_engine_start == 1)
	{
		m_ignitors = true;
	}
	if ((m_input.m_engine_start == 1) && (m_input.m_engine_stop == 1))
	{
		m_ignitors = false;
		m_input.m_engine_start = 0;
		m_input.m_engine_stop = 0;
	}

}

double Engine::updateThrust() //Wenn Veränderungen dann hier verändern NICHT oben!!!!! //dt in die Klammer eingefügt//double zu void mit double dt verändert
{
	m_force = Vec3(); //braucht man hier vielleicht nicht, trotzdem wieder eingesetzt zum testen
	m_throttle = 0.0; //neu eingefügt
	m_thrust = 0.0; //wieder eingefügt nach auskommentierung
	//m_thrust = 150000; //nur zum testen, wieder rausnehmen
	
	m_scalarVelocity = magnitude(m_state.m_localAirspeed);
	m_scalarVelocitySquared = m_scalarVelocity * m_scalarVelocity;
	m_state.m_mach = m_scalarVelocity / m_state.m_speedOfSound;

	double corrThrottle = 0.0;

	if (m_input.m_throttle >= 0.0)
	{
		corrThrottle = (1 - CON_ThrotIDL) * m_input.m_throttle + CON_ThrotIDL;
	}
	else
	{
		corrThrottle = (m_input.m_throttle + 1.0) / 2.0;
	}

	if ((corrThrottle <= 0.85) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_thrust = (corrThrottle * PMax(m_state.m_mach) * (m_state.m_airDensity / CON_sDay_den));
	}
	else if ((corrThrottle > 0.85) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_thrust = (corrThrottle * PFor(m_state.m_mach) * (m_state.m_airDensity / CON_sDay_den));

	}
	else
	{
		m_thrust = 0.0;
	}

	//corrThrottle = -0.55 * m_input.m_throttle + 1.0; //TESTSETTINGS
	//m_thrust = corrThrottle * PFor(m_state.m_mach); //TESTSETTINGS
	
	//printf("Throttle %f \n", corrThrottle);
	//printf("Thrust %f \n", m_thrust); //eingefügt zum testen, ob M_thrust ausgegeben wird...
	//printf("CurrentAirDensity %f \n", m_state.m_airDensity);
	
	return m_thrust;
}

double Engine::FuelFlowUpdate() 
{
	double lowOmegaInertia = 1.0;
	//starting up or restarting
	/*if (getRPMNorm() < 0.50)
	{
		lowOmegaInertia = 7.0;
	}*/

	//normaler Betrieb, die Engine läuft
	//passt aktuell nicht ganz, da die CON-Werte von 100% Mil/AB-Power ausgehen
	//Alter kram der aber funktioniert hat
	/*if ((getRPMNorm() >= 0.70) && (getRPMNorm() < 0.95))
	{
		m_fuelFlow = getRPMNorm() * (CON_CeMax * 3600 * 2.205) ;
	}

	if (getRPMNorm() >= 0.95)
	{
		m_fuelFlow = getRPMNorm() * (CON_CeFor * 3600 * 2.205);
	}
	else
	{
		m_fuelFlow = 1.300;
	}*/
	
	double corrThrottle = 0.0;

	if (m_input.m_throttle >= 0.0)
	{
		corrThrottle = (1 - CON_ThrotIDL) * m_input.m_throttle + CON_ThrotIDL;
	}
	else
	{
		corrThrottle = (m_input.m_throttle + 1.0) / 2.0;
	}
	
	if ((getRPMNorm() == 0.70) && (getRPMNorm() < 0.95) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_fuelFlow = 1500.0 * (m_state.m_airDensity / CON_sDay_den) + (m_state.m_mach * (0.50 * corrThrottle * CON_CeMax * 3600 * 2.205));
	}
	
	else if ((getRPMNorm() > 0.70) && (getRPMNorm() < 0.95) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_fuelFlow = (corrThrottle * (CON_CeMax * 3600 * 2.205) + 1500) * (m_state.m_airDensity / CON_sDay_den) + (m_state.m_mach * ( 0.50 * corrThrottle * CON_CeMax * 3600 * 2.205));
	}
	else if ((getRPMNorm() >= 0.95) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_fuelFlow = (corrThrottle * (CON_CeFor * 3600 * 2.205) + 1500) * (m_state.m_airDensity / CON_sDay_den) + (m_state.m_mach * (0.50 * corrThrottle * CON_CeFor * 3600 * 2.205));
	}
	else
	{
		m_fuelFlow = 0;
	}
 
	return m_fuelFlow;
}


double Engine::updateBurner()
{
	m_burner = 0.0;
	double corrThrottle = 0.0;

	if (m_input.m_throttle >= 0.0)
	{
		corrThrottle = (1 - CON_ThrotIDL) * m_input.m_throttle + CON_ThrotIDL;
	}
	else
	{
		corrThrottle = (m_input.m_throttle + 1.0) / 2.0;
	}

	if ((corrThrottle >= 0.85) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_burner = 1;
	}
	else
	{
		m_burner = 0;
	}
	return m_burner;
}

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

void Engine::update(double dt) //möglicherweise die ganze Funktion DEAD...dann aber auch im Header entfernen!!!!!
{
	//m_force = Vec3(); //braucht man hier vielleicht nicht, trotzdem wieder eingesetzt zum testen
	//m_throttle = 0.0; //neu eingefügt
	//m_thrust = 0.0; //wieder eingefügt nach auskommentierung
	//m_thrust = 150000; //nur zum testen, wieder rausnehmen
	//m_scalarVelocity = magnitude(m_state.m_localAirspeed);
	//m_scalarVelocitySquared = m_scalarVelocity * m_scalarVelocity;
	//m_state.m_mach = m_scalarVelocity / m_state.m_speedOfSound;
	
	//double corrThrottle = 0.0;
	
	/*if (m_input.m_throttle >= 0.0)
	{
		corrThrottle = (1 - CON_ThrotIDL) * m_input.m_throttle + CON_ThrotIDL;
	}
	else
	{
		corrThrottle = (m_input.m_throttle + 1.0) / 2.0;
	}

	if (corrThrottle <= 0.85)
	{
		m_thrust = (corrThrottle * PMax(m_state.m_mach)) * (m_state.m_airDensity / CON_sDay_den);
	}
	else
	{
		m_thrust = (corrThrottle * PFor(m_state.m_mach)) * (m_state.m_airDensity / CON_sDay_den);

	}*/
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

	if (corrThrottle <= 0.85)
	{
		m_thrust = (corrThrottle * PMax(m_state.m_mach) * (m_state.m_airDensity / CON_sDay_den));
	}
	else
	{
		m_thrust = (corrThrottle * PFor(m_state.m_mach) * (m_state.m_airDensity / CON_sDay_den));

	}

	//corrThrottle = -0.55 * m_input.m_throttle + 1.0; //TESTSETTINGS
	//m_thrust = corrThrottle * PFor(m_state.m_mach); //TESTSETTINGS
	
	//printf("Throttle %f \n", corrThrottle);
	//printf("Thrust %f \n", m_thrust); //eingefügt zum testen, ob M_thrust ausgegeben wird...
	//printf("CurrentAirDensity %f \n", m_state.m_airDensity);
	
	return m_thrust;
}
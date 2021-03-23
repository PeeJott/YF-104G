#include "Engine.h"

Engine::Engine
(
	Input& input,
	State& state
) :
	m_input(input),
	m_state(state),
	//---------------Thrust------------------------------------
	PMax(DAT_PMax, CON_PMaxmin, CON_PMaxmax),
	PFor(DAT_PFor, CON_PFormin, CON_PFormax)
	//der letzte Eintrag darf KEIN Komma haben...
{
	//huhu!!
}

void Engine::update(double dt) //möglicherweise die ganze Funktion DEAD...dann aber auch im Header entfernen!!!!!
{
	m_force = Vec3(); //braucht man hier vielleicht nicht, trotzdem wieder eingesetzt zum testen
	m_throttle = 0.0; //neu eingefügt
	m_thrust = 0.0; //wieder eingefügt nach auskommentierung
	m_thrust = 150000; //nur zum testen, wieder rausnehmen
	double corrThrottle = 0.0;
	
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

double Engine::updateThrust() //Wenn Veränderungen dann hier verändern NICHT oben!!!!!
{
	m_force = Vec3(); //braucht man hier vielleicht nicht, trotzdem wieder eingesetzt zum testen
	m_throttle = 0.0; //neu eingefügt
	m_thrust = 0.0; //wieder eingefügt nach auskommentierung
	m_thrust = 150000; //nur zum testen, wieder rausnehmen
	double corrThrottle = 0.0;

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
	return m_thrust;
}
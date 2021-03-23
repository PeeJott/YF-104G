#pragma once
#include <stdio.h>
#include "Table.h"
#include "Vec3.h"
#include "State.h"
#include "Input.h"
#include "AeroData_1.h"
//#include "FlightModel.h" war doppelt in flightmodel (Ringabhängigkeit)

class Engine
{
public:
	Engine(Input& input, State& state);

	void update(double dt); //in der () "double dt" eingefügt, war vorher ohne
	double updateThrust();
	inline const Vec3& getForce() const;

	inline double getThrust();
	// inline void setThrust(double thrust); //auskommentiert zum Angleichen A4 engine.h

	inline void setThrottle(double throttle); //Neu eingefügt nach A4 engine2.h
	
private:
	
	Vec3 m_force;
	Input m_input;
	State m_state;
	//--------------Aerodynamic Values--------------------------------
	double m_scalarVelocity = 0.0;
	double m_scalarVelocitySquared = 0.0;
	//-------------Engine Values/Commands----------------------------
	double m_thrust = 0.0;
	double m_throttle = 0.0; //neu 28.02.2021
	//-------------Thrust Tables init------------------------
	Table PMax;
	Table PFor;
};

const Vec3& Engine::getForce() const
{
	return m_force;
}

double Engine::getThrust()
{
	return m_thrust;
}



//void Engine::setThrust(double thrust) //rauskommentiert zur Angleichung an A4 Engine.h
//{
//	m_thrust = thrust;
//}

void Engine::setThrottle(double throttle) //neu eingefügt aus A4 engine2.h
{
	m_throttle = throttle; //die ganze Klammer neu eingefügt
}

//Engine::Engine -- Rauskommentiert da in "Engine.cpp" verschoben
//(
//	State& state,
//	Input& input
//) :
//	m_state(state),
//	m_input(input),
	//---------------Thrust------------------------------------
//	PMax(DAT_PMax, CON_PMaxmin, CON_PMaxmax),
//	PFor(DAT_PFor, CON_PFormin, CON_PFormax)
	//der letzte Eintrag darf KEIN Komma haben...
//{
	//huhu!!
//}

//void Engine::update(double dt)
//{
//	m_force = Vec3();

//	double corrThrottle = 0.0;
//	if (m_input.m_throttle >= 0.0)
//	{
//		corrThrottle = (1 - CON_ThrotIDL) * m_input.m_throttle + CON_ThrotIDL;
//	}
//	else
//	{
//		corrThrottle = (m_input.m_throttle + 1.0) / 2.0;
//	}

//	double m_thrust = 0.0;
//	if (corrThrottle <= 0.85)
//	{
//		m_thrust = (corrThrottle * PMax(m_state.m_mach)) * (m_state.m_airDensity / CON_sDay_den);
//	}
//	else
//	{
//		m_thrust = (corrThrottle * PFor(m_state.m_mach)) * (m_state.m_airDensity / CON_sDay_den);
//
//	}
//}


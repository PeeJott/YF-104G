#pragma once
#include <stdio.h>
#include "Table.h"
#include "Vec3.h"
#include "State.h"
#include "Input.h"
#include "AeroData_1.h"
#include "BaseComponent.h"

//#include "FlightModel.h" war doppelt in flightmodel (Ringabhängigkeit)

class Engine
{
public:
	Engine(State& state, Input& input);

	//Initialization
	virtual void zeroInit();
	virtual void coldInit();
	virtual void hotInit();
	virtual void airborneInit();

	void update(double dt); //in der () "double dt" eingefügt, war vorher ohne
	double FuelFlowUpdate();
	double updateThrust(); 
	double updateBurner();
	double updateSpool();
	double updateSpoolCold();
	
	//inline double getFuelFlow();//zum testen auskommentiert
	inline void setHasFuel(bool hasFuel);
	//inline void setAirspeed(double airspeed);
	inline void setIgnitors(bool ignitors);

	inline double getRPMNorm();
	inline double getRPM();

	//void updateThrust(double dt);
	inline const Vec3& getForce() const;

	inline double getThrust();
	// inline void setThrust(double thrust); //auskommentiert zum Angleichen A4 engine.h

	inline void setThrottle(double throttle); //Neu eingefügt nach A4 engine2.h
	
private:
	
	Vec3 m_force;
	State& m_state;
	Input& m_input;
	

	//--------------Aerodynamic Values--------------------------------
	double m_scalarVelocity = 0.0;
	double m_scalarVelocitySquared = 0.0;

	//-------------Engine Values/Commands----------------------------
	double m_thrust = 0.0;
	double m_throttle = 0.0;
	double m_burner = 0.0;
	double m_corrAirDensity = 0.0;
	
	double m_correctedFuelFlow = 0.0;
	double m_fuelFlow = 0.0;
	double m_rpmNormal = 0.0;
	double m_spoolFactor = 0.0;
	double m_spoolFactorPrevious = 0.0;
	double m_deltaSpool = 0.0;
	double m_throttleNEW = 0.0;
	double m_newThrottle = 0.0;
	double m_oldThrottle = 0.0;
	double m_deltaSpoolABS = 0.0;
	double m_newSpoolStep = 0.0;
	double m_desiredThrottle = 0.0;
	double m_spoolColdStart = 0.0;
	bool m_hasFuel = true;
	bool m_ignitors = true;
	bool m_started = false;



	//-------------Thrust Tables init------------------------
	Table PMax;
	Table PFor;
	Table CADen;
	Table EngDel;
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

void Engine::setHasFuel(bool hasFuel)
{
	m_hasFuel = hasFuel;
}

//zum Testen verändert
/*double Engine::getFuelFlow()
{
	m_fuelFlow = FuelFlowUpdate();
	return m_fuelFlow;
}*/

void Engine::setIgnitors(bool ignitors)
{
	m_ignitors = ignitors;
}

//erstmal um eine Reaktion zu haben ist sobald ignition an und fuel da RPM auf 70%
double Engine::getRPMNorm()
{
	double RPM_Normal = 0.0;

	//--------------------Alte RPM-Funktion die funktioniert----------------------
	/*double corrThrottle = 0.0;

	if (m_input.m_throttle >= 0.0)
	{
		corrThrottle = (1 - CON_ThrotIDL) * m_input.m_throttle + CON_ThrotIDL;
	}
	else
	{
		corrThrottle = (m_input.m_throttle + 1.0) / 2.0;
	}
	
	if ((m_hasFuel == true) && (m_ignitors == true))
	{
		if (corrThrottle < 0.01)
		{
			RPM_Normal = 0.70;
		}
		if (corrThrottle >= 0.01)
		{
			RPM_Normal = 0.70 + (corrThrottle * 0.40);
		}
		m_rpmNormal = RPM_Normal;
	}
	else
	{
		m_rpmNormal = 0.0;
	}*/
	//------------------ENDE alte Funktion-------------------------------------------
	
	
	
	if ((m_hasFuel == true) && (m_ignitors == true))
	{
		if (m_spoolColdStart < 1)
		{
			RPM_Normal = 0.70 * m_spoolColdStart;
			m_rpmNormal = RPM_Normal;
		}
		
		if (m_spoolColdStart == 1)
		{
			if (updateSpool() < 0.01)
			{
				RPM_Normal = 0.70;
			}
			if (updateSpool() >= 0.01)
			{
				RPM_Normal = 0.70 + (updateSpool() * 0.40);
			}
			
			m_rpmNormal = RPM_Normal;
		}
	}
	else if (((m_hasFuel == false) || (m_ignitors == false)) && (m_spoolColdStart > 0.0))
	{
		RPM_Normal = 0.70 * m_spoolColdStart;
		m_rpmNormal = RPM_Normal;
	}
	else
	{
		m_rpmNormal = 0.0;
	}
	
	return m_rpmNormal; //erstmal um was hier drin zu haben
}

double Engine::getRPM()
{
	
	return 1;
	//return CON_ThrToRPM * updateThrust(); //erstmal um was zu haben
}


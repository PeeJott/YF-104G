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
	PFor(DAT_PFor, CON_PFormin, CON_PFormax),
	CADen(DAT_CADen, CON_CADenMin, CON_CADenMax),
	EngDel(DAT_EngDel, CON_EngDelMin, CON_EngDelMax)
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
	m_corrAirDensity = 0.0;
	m_spoolFactor = 0.0;
	m_spoolFactorPrevious = 0.0;
	m_deltaSpool = 0.0;
	m_throttleNEW = 0.0;
	m_hasFuel = true;
	m_ignitors = false;
	m_newThrottle = 0.0;
	m_oldThrottle = 0.0;
	m_deltaSpoolABS = 0.0;
	m_newSpoolStep = 0.0;
	m_desiredThrottle = 0.0;
	m_spoolColdStart = 0.0;
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
	m_spoolColdStart = 1;

}

void Engine::airborneInit()
{
	zeroInit();

	m_ignitors = true;
	m_rpmNormal = 0.85;
	m_input.m_engine_start = 1;
	m_spoolColdStart = 1;
}

void Engine::update(double dt)
{
	double corrThrottle = 0.0;
	
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

	//---------------OLD corrected Air Density with 2 steps
	/*if ((m_state.m_airDensity < 0.74) && (m_state.m_airDensity > 0.4))
	{
		m_corrAirDensity = m_state.m_airDensity * 0.30;
	}
	else if (m_state.m_airDensity <= 0.4)
	{
		m_corrAirDensity = m_state.m_airDensity * 0.45;
	}
	else
	{
		m_corrAirDensity = 0.0;
	}*/

	//------------NEW corrected Air Density with a lot of steps through lookup----------

	if (m_state.m_airDensity <= 0.88)
	{
		m_corrAirDensity = CADen(m_state.m_airDensity);
	}
	else
	{
		m_corrAirDensity = 0.0;
	}
//--------------END of corrected airdensity----------------------------------------------
//--------------Start of Engine starting up and spooling down----------------------------	
	double spool[51] = { 1.0000, 0.7817, 0.6753, 0.5834, 0.5040, 0.4354, 0.3762, 0.3250, 0.2808, 0.2425, 0.2095, 0.1810, 0.1564, 0.1351, 0.1167, 0.1008, 0.0871, 0.0753, 0.0650, 0.0562, 0.0485, 0.0419, 0.0362, 0.0313, 0.0270, 0.0234, 0.0202, 0.0174, 0.0151, 0.0130, 0.0112, 0.0097, 0.0084, 0.0072, 0.0063, 0.0054, 0.0047, 0.0040, 0.0035, 0.0030, 0.0026, 0.0022, 0.0019, 0.0017, 0.0014, 0.0013, 0.0011, 0.0009, 0.0008, 0.0007, 0.00,};
	
	if ((m_ignitors == true) && (m_hasFuel == true) && (getRPMNorm() == 0.0))
	{
		m_spoolColdStart = 1 * (spool[50]--);
	}
	else if (((m_ignitors == false) || (m_hasFuel == false)) && (getRPMNorm() > 0.0))
	{
		m_spoolColdStart = 1 * (spool[0]++);
	}
	else
	{
		m_spoolColdStart = 1;
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
	
	//--------------Alte Thrust-Funktion die funktioniert-------------------
	/*double corrThrottle = 0.0;

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
		m_thrust = (corrThrottle * PMax(m_state.m_mach)) * (m_state.m_airDensity / CON_sDay_den);
	}
	else if ((corrThrottle > 0.85) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_thrust = (corrThrottle * PFor(m_state.m_mach)) * ((m_state.m_airDensity + m_corrAirDensity) / CON_sDay_den);

	}
	else
	{
		m_thrust = 0.0;
	}*/
	//---------------------Alte Funktion ENDE------------------------------------------



	//corrThrottle = -0.55 * m_input.m_throttle + 1.0; //TESTSETTINGS
	//m_thrust = corrThrottle * PFor(m_state.m_mach); //TESTSETTINGS
	
	//printf("Throttle %f \n", corrThrottle);
	//printf("Thrust %f \n", m_thrust); //eingefügt zum testen, ob M_thrust ausgegeben wird...
	//printf("CurrentAirDensity %f \n", m_state.m_airDensity);
	
	if ((updateSpool() <= 0.85) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_thrust = (updateSpool() * PMax(m_state.m_mach)) * (m_state.m_airDensity / CON_sDay_den);
	}
	else if ((updateSpool() > 0.85) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_thrust = (updateSpool() * PFor(m_state.m_mach)) * ((m_state.m_airDensity + m_corrAirDensity) / CON_sDay_den);

	}
	else
	{
		m_thrust = 0.0;
	}

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
	
	//------------Alte FuelFlow-Funktion die funktioniert----------------
	/*double corrThrottle = 0.0;

	if (m_input.m_throttle >= 0.0)
	{
		corrThrottle = (1 - CON_ThrotIDL) * m_input.m_throttle + CON_ThrotIDL;
	}
	else
	{
		corrThrottle = (m_input.m_throttle + 1.0) / 2.0;
	}
	
	if ((corrThrottle < 0.01 ) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_fuelFlow = 1500.0 * (m_state.m_airDensity / CON_sDay_den) + (m_state.m_mach * (0.60 * corrThrottle * CON_CeMax * 3600 * 2.205));
	}
	
	else if ((corrThrottle >= 0.01) && (corrThrottle < 0.85) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_fuelFlow = (corrThrottle * (CON_CeMax * 3600 * 2.205) + 1500) * (m_state.m_airDensity / CON_sDay_den) + (m_state.m_mach * ( 0.60 * corrThrottle * CON_CeMax * 3600 * 2.205));
	}
	else if ((corrThrottle >= 0.85) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_fuelFlow = (corrThrottle * (CON_CeFor * 3600 * 2.205) + 1500) * (m_state.m_airDensity / CON_sDay_den) + (m_state.m_mach * (0.60 * corrThrottle * CON_CeFor * 3600 * 2.205));
	}
	else
	{
		m_fuelFlow = 0;
	}*/
	//------------------------------------Alte Funktion ENDE--------------------------------------------------------------
	

	if ((updateSpool() < 0.01) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_fuelFlow = 1500.0 * (m_state.m_airDensity / CON_sDay_den) + (m_state.m_mach * (0.60 * updateSpool() * CON_CeMax * 3600 * 2.205));
	}

	else if ((updateSpool() >= 0.01) && (updateSpool() < 0.85) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_fuelFlow = (updateSpool() * (CON_CeMax * 3600 * 2.205) + 1500) * (m_state.m_airDensity / CON_sDay_den) + (m_state.m_mach * (0.60 * updateSpool() * CON_CeMax * 3600 * 2.205));
	}
	else if ((updateSpool() >= 0.85) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_fuelFlow = (updateSpool() * (CON_CeFor * 3600 * 2.205) + 1500) * (m_state.m_airDensity / CON_sDay_den) + (m_state.m_mach * (0.60 * updateSpool() * CON_CeFor * 3600 * 2.205));
	}
	else
	{
		m_fuelFlow = 0;
	}

 
	return m_fuelFlow;
}

double Engine::updateSpool()
{
	double corrThrottle = 0.0;


	if (m_input.m_throttle >= 0.0)
	{
		corrThrottle = (1 - CON_ThrotIDL) * m_input.m_throttle + CON_ThrotIDL;
	}
	else
	{
		corrThrottle = (m_input.m_throttle + 1.0) / 2.0;
	}
	
	//desiredThrottle = corrThrottle;
	m_desiredThrottle = corrThrottle;
	m_deltaSpool = m_desiredThrottle - m_oldThrottle;
	m_deltaSpoolABS = abs(m_deltaSpool);
	m_spoolFactor = EngDel(m_deltaSpoolABS);
	m_newSpoolStep = m_deltaSpool * m_spoolFactor;
	m_newThrottle = m_oldThrottle + m_newSpoolStep;
	m_throttleNEW = m_newThrottle;
	
	m_oldThrottle = m_newThrottle;

	return m_throttleNEW;

	//double spool[50] = { 1.0000, 0.7817, 0.6753, 0.5834, 0.5040, 0.4354, 0.3762, 0.3250, 0.2808, 0.2425, 0.2095, 0.1810, 0.1564, 0.1351, 0.1167, 0.1008, 0.0871, 0.0753, 0.0650, 0.0562, 0.0485, 0.0419, 0.0362, 0.0313, 0.0270, 0.0234, 0.0202, 0.0174, 0.0151, 0.0130, 0.0112, 0.0097, 0.0084, 0.0072, 0.0063, 0.0054, 0.0047, 0.0040, 0.0035, 0.0030, 0.0026, 0.0022, 0.0019, 0.0017, 0.0014, 0.0013, 0.0011, 0.0009, 0.0008, 0.0007,};// voll aufgefüllt
}

double Engine::updateBurner()
{
	m_burner = 0.0;
	
	//------------------Alte Burner-Funktion die funktioniert----------------
	/*double corrThrottle = 0.0;

	if (m_input.m_throttle >= 0.0)
	{
		corrThrottle = ((1 - CON_ThrotIDL) * m_input.m_throttle + CON_ThrotIDL);
	}
	else
	{
		corrThrottle = ((m_input.m_throttle + 1.0) / 2.0);
	}*/

	if ((updateSpool() >= 0.85) && (m_ignitors == true) && (m_hasFuel == true))
	{
		m_burner = 1;
	}
	else
	{
		m_burner = 0;
	}
	return m_burner;
}

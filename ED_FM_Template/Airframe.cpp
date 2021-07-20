#include "Airframe.h"
#include <algorithm>
#include <random>
#include <cstdlib>
#include <ctime>
#include <chrono>

//-----------Neu eingefügt wegen Chrono und Millisekunden---------
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;
//-----------------------------------------------------------------

Airframe::Airframe
(
	State& state,
	Input& input,
	Engine& engine

) :
	m_state(state),
	m_input(input),
	m_engine(engine),
	m_actuatorFlap(0.9),
	m_actuatorAirbrk(1.2),
	m_actuatorGearL(0.2),
	m_actuatorGearN(0.2),
	m_actuatorGearR(0.2),
	m_actuatorHook(0.6),
	m_actuatorNozzle(1.1),
	m_actuatorNosewheel(2.0)

{
	m_integrityElement = new float[(int)Damage::COUNT];
	zeroInit();

	m_damageStack.reserve(10);
}

Airframe::~Airframe()
{
	delete[] m_integrityElement;
}

void Airframe::printDamageState()
{
	printf("===========================================\n");

	printf("            ||\n");
	printf("            ||\n");
	printf("%.1lf %.1lf %.1lf || %.1lf %.1lf %.1lf\n",
		DMG_ELEM(Damage::WING_L_OUT),
		DMG_ELEM(Damage::WING_L_CENTER),
		DMG_ELEM(Damage::WING_L_IN),
		DMG_ELEM(Damage::WING_R_IN),
		DMG_ELEM(Damage::WING_R_CENTER),
		DMG_ELEM(Damage::WING_R_OUT));

	printf("        %.1f || %.1f\n", DMG_ELEM(Damage::AILERON_L), DMG_ELEM(Damage::AILERON_R));
	printf("            ||\n");
	printf("        %.1f || %.1f\n", DMG_ELEM(Damage::STABILIZATOR_L), DMG_ELEM(Damage::STABILIZATOR_R));
	printf("        %.1f || %.1f\n", DMG_ELEM(Damage::ELEVATOR_L), DMG_ELEM(Damage::ELEVATOR_R));
	printf("            %.1f\n", getVertStabDamage());
	printf("            %.1f\n", getRudderDamage());
}

void Airframe::resetDamage()
{
	for (int i = 0; i < (int)Damage::COUNT; i++)
	{
		m_integrityElement[i] = 1.0f;
	}
}

void Airframe::zeroInit()
{
	//---Gear position--------
	m_gearLPosition = 0.0;
	m_gearRPosition = 0.0;
	m_gearNPosition = 0.0;

	m_gearFLamp = 0.0;
	m_gearLLamp = 0.0;
	m_gearRLamp = 0.0;

	m_gearStart = 0.0;

	//------aerodynamic surfaces-------
	m_flapsPosition = 0.0;
	m_speedBrakePosition = 0.0;
	
	m_aileronLeft = 0.0;
	m_aileronRight = 0.0;
	m_stabilizer = 0.0;
	m_rudder = 0.0;
	
	m_hookPosition = 0.0;

	m_noseWheelAngle = 0.0;

	resetDamage();
	
	m_nozzlePosition = 0.0;

	m_int_throttlePos = 0.0;

	//set to "0" after each init
	//m_input.m_brake = 0.0; geht nicht, da jetzt funktion

	m_nwsEngage = 0.0;
	m_chuteState = 0.0;
	m_mass = 1.0;
	m_damageStack.clear();
	
	m_timePassed = 0;

	m_engDmgMulti = 1.0;

	m_desiredAlt = 0.0;
	m_previousAlt = 0.0;
	m_autPilAltEng = 0.0;
	m_altHold = 0.0;
	m_ascHA = 0.0;
	m_decHA = 0.0;
	m_decend = false;
	m_acend = false;
	m_level = false;
	m_acendHoldAngle = false;
	m_decendHoldAngle = false;

	m_flapsLevPos = 0.0;
	m_flapsIndTEPos = 0.0;
	m_flapsIndLEPos = 0.0;

	m_fuelHundred = 0.0;
	m_fuelThousand = 0.0;
	m_fuelDivide = 0.0;

	/*m_elevUP = 0.0;
	m_elevDOWN = 0.0;
	m_ailRIGHT = 0.0;
	m_ailLEFT = 0.0;
	m_rudRIGHT = 0.0;
	m_rudLEFT = 0.0;
	
	m_keyPitch = 0.0;
	m_keyRoll = 0.0;
	m_keyYaw = 0.0;

	m_finalPitch = 0.0;
	m_finalRoll = 0.0;
	m_finalYaw = 0.0;*/

	m_ailDamInd = 0.0;
	m_stabDamInd = 0.0;

}

void Airframe::coldInit()
{
	zeroInit();
	
	//--Ground Init SET Actuator---------
	m_actuatorGearL.groundInit(1.0, 1.0);
	m_actuatorGearN.groundInit(1.0, 1.0);
	m_actuatorGearR.groundInit(1.0, 1.0);
	//---Gear position--------
	m_gearLPosition = 0.0;
	m_gearRPosition = 0.0;
	m_gearNPosition = 0.0;

	//-----gear position for ground start
	m_gearStart = 0.0;
	//m_actuatorGearL = 0.0;
	//m_actuatorGearN = 0.0;
	//m_actuatorGearR = 0.0;
	//m_input.m_gear_toggle = 1.0; auskommentiert weil nicht zeroable da jetzt funktion
	//------aerodynamic surfaces-------
	m_flapsPosition = 0.0;
	m_speedBrakePosition = 0.0;

	m_aileronLeft = 0.0;
	m_aileronRight = 0.0;
	m_stabilizer = 0.0;
	m_rudder = 0.0;

	m_hookPosition = 0.0;

	m_noseWheelAngle = 0.0;

	m_nozzlePosition = 0.0;
	m_int_throttlePos = 0.0;

	//m_input.m_brake = 0.0; geht nicht, da jetzt funktion

	m_nwsEngage = 0.0;
	m_chuteState = 0.0;
	m_mass = 1.0;
	m_timePassed = 0;
}

void Airframe::hotInit()
{
	zeroInit();

	//--Ground Init SET Actuator---------
	m_actuatorGearL.groundInit(1.0, 1.0);
	m_actuatorGearN.groundInit(1.0, 1.0);
	m_actuatorGearR.groundInit(1.0, 1.0);
	//---Gear position--------
	m_gearLPosition = 0.0;
	m_gearRPosition = 0.0;
	m_gearNPosition = 0.0;

	//---Gear position for ground start----------
	m_gearStart = 0.0;
	//m_actuatorGearL = 0.0;
	//m_actuatorGearN = 0.0;
	//m_actuatorGearR = 0.0;
	//m_input.m_gear_toggle = 1.0; nicht mehr setzbar, da jetzt funktion

	//------aerodynamic surfaces-------
	m_flapsPosition = 0.0;
	m_speedBrakePosition = 0.0;

	m_aileronLeft = 0.0;
	m_aileronRight = 0.0;
	m_stabilizer = 0.0;
	m_rudder = 0.0;

	m_hookPosition = 0.0;

	m_noseWheelAngle = 0.0;

	m_nozzlePosition = 0.0;
	m_int_throttlePos = 0.0;

	//m_input.m_brake = 0.0; geht nicht, da jetzt funktion
	
	m_nwsEngage = 0.0;
	m_chuteState = 0.0;
	m_mass = 1.0;
	m_timePassed = 0;
}

void Airframe::airborneInit()
{
	zeroInit();
}


void Airframe::airframeUpdate(double dt)
{
	//printf( "I %lf, L %lf, C %lf, R %lf\n", m_fuel[0], m_fuel[1], m_fuel[2], m_fuel[3] );

	/*if (m_input.m_hook())
	{
		m_hookPosition += dt / m_hookExtendTime;
		m_hookPosition = std::min(m_hookPosition, 1.0);
	}
	else
	{
		m_hookPosition -= dt / m_hookExtendTime;
		m_hookPosition = std::max(m_hookPosition, 0.0);
	}*/

	//printf("LEFT: %lf, CENTRE: %lf, RIGHT: %lf, INTERNAL: %lf\n", m_fuel[Tank::LEFT_EXT], m_fuel[Tank::CENTRE_EXT], m_fuel[Tank::RIGHT_EXT], m_fuel[Tank::INTERNAL]);
	//m_engine.setHasFuel(m_fuel[Tank::INTERNAL] > 20.0);

	//printf( "Compressor Damage %lf, Turbine Damage: %lf\n", getCompressorDamage(), getTurbineDamage() );
	
	//m_engine.setIntegrity(DMG_ELEM(Damage::ENGINE)); NOCH einfügen in der engine-class
	
	m_stabilizer = setStabilizer(dt);
	m_aileronLeft = setAileron(dt);
	m_aileronRight = -m_aileronLeft;
	m_rudder = setRudder(dt);
	
	m_flapsPosition = setFlapsPosition(dt);
	m_speedBrakePosition = setAirbrakePosition(dt);
	
	m_gearLPosition = setGearLPosition(dt);
	m_gearRPosition = setGearRPosition(dt);
	m_gearNPosition = setGearNPosition(dt);

	
	m_hookPosition = setHookPosition(dt);

	m_nozzlePosition = setNozzlePosition(dt);

	m_noseWheelAngle = setNoseWheelAngle(dt);

	//Neuer Test, falls der KeyBind nur beim Drücke "1" ist
	//m_flapsPosition = getFlapsPosition();
	
	//printf("Flp-Down-Value %f \n", m_input.m_flapsdown);
	//printf("Flp-Up-Value %f \n", m_input.m_flapsup);
	//printf("YAW_Value %f \n", m_input.m_flapstgl);
	//printf("Flp-Tgl-Value %f \n", m_input.m_flapstgl);
	
	//---------Engine flame-out function------------------

	if (((getCompressorDamage() < 0.15) || (getTurbineDamage() < 0.15)) && (m_input.getEngineStart() == 1))
	{
		m_engine.m_needRepair = true;
		m_engine.m_needRestart = true; 
	}
	/*else if (((getCompressorDamage() < 0.15) || (getTurbineDamage() < 0.15)) && (m_input.m_engine_start == 0) && (m_input.m_engine_stop == 0))
	{
		m_input.m_engine_stop = 2.0;
		m_input.m_engine_start = 2.0;
	}*/
	else if ((getCompressorDamage() >= 0.65) && (getTurbineDamage() >= 0.65) && (m_engine.m_needRestart == false))
	{
		m_engine.m_needRepair = false;
	}

	//------------------AutoPilot-Update------------------
	autoPilotAltH(dt);

//--------------------KeyCommand-Updates-----------------
	//keyCommandElevator(dt);
	//keyCommandAileron(dt);
	//keyCommandRudder(dt);
	//printf("FuelFlow_Indicator %f \n", fuelFlowIndGaugeUpdate());
	//printf("FuelFlow in lbs/h %f \n", m_engine.FuelFlowUpdate());

//------------------FuelFlow Indicator Update-----------(NEU ggf. überflüssig)
	fuelFlowIndGaugeUpdate();
	//printf("FuelIndicator %f \n", m_fuelThousand);

//----------Brake update---------------(NEU ggf. überflüssig)
	//updateBrake();

//---------Internal Throttle Position update---(NEU, ggf. überflüssig)
	//getIntThrottlePosition();

//----Alle folgenden NEU eingefpgt und von () zu (doubel dt) geändert!!!
	//brkChutePosition();
	//brkChuteSlewY();
	//brkChuteSlewZ();

	//BLCsystem();
	getEngineDamageMult();



}

double Airframe::updateBrake()
{
	m_brakeMoment = 0.0;
	
	/*if (m_input.m_release_brake == 1.0)
	{
		if (m_input.m_brake == 1.0)
		{
			m_input.m_brake = 0.0;
			m_input.m_release_brake = 0.0;
		}
	}*/ //wurde in Input.cpp verschoben "setBrake()"
	
	if (m_input.getBrake() == 1.0)
	{
		m_brakeMoment = 0.66;
	}
	else
	{
		m_brakeMoment = 0.0;
	}
	
	return m_brakeMoment;
}

double Airframe::setNozzlePosition(double dt) //Nozzle-Position 0-10% Thrust open, 11-84% Thrust closed, 85-100% Thrust open
{
	double NozzlePos = 0.0;
	double corrThrottle = 0.0;

	if (m_input.getThrottle() >= 0.0)
	{
		corrThrottle = (1 - CON_ThrotIDL) * m_input.getThrottle() + CON_ThrotIDL;
	}
	else
	{
		corrThrottle = (m_input.getThrottle() + 1.0) / 2.0;
	}

	if (corrThrottle <= 0.10)
	{
		NozzlePos = 0.4;
	}
	else if (corrThrottle >= 0.85)
	{
		NozzlePos = 0.80;
	}
	else
	{
		NozzlePos = 0.2;
	}

	double input = NozzlePos;
	return m_actuatorNozzle.inputUpdate(input, dt);
}

double Airframe::getIntThrottlePosition()
{
	m_int_throttlePos = 0.0;
	double corrThrottle = 0.0;

	if (m_input.getThrottle() >= 0.0)
	{
		corrThrottle = (1.0 - CON_ThrotIDL) * m_input.getThrottle() + CON_ThrotIDL;
	}
	else
	{
		corrThrottle = (m_input.getThrottle() + 1.0) / 2.0;
	}

	m_int_throttlePos = corrThrottle;

	return m_int_throttlePos;
}

double Airframe::NWSstate()
{
	if (m_input.getNWS() == 1.0)
	{
		m_nwsEngage = 1.0;
	}
	else if (m_input.getNWS() == 0.0)
	{
		m_nwsEngage = 0.0;
	}
	else
	{
		m_nwsEngage = 0.0;
	}
	return m_nwsEngage;
}

double Airframe::brkChutePosition()
{
	int timeToGo = 50;

	if ((m_input.getBrkChute() == 1) && (m_timePassed < timeToGo))
	{
		m_chuteState = 0.5;
	}
	if (m_chuteState == 0.5) 
	{
		m_timePassed++;
	}
	if ((m_chuteState == 0.5) && (m_timePassed >= timeToGo)) 
	{
		m_chuteState = 1.0;
		m_timePassed = 55;
	}
	else if (m_input.getBrkChute() == 0.0)
	{
		m_chuteState = 0.0;
		m_timePassed = 0.0;
	}

	return m_chuteState;
}


double Airframe::brkChuteSlewY()
{
	m_chuteSlewY = 0.0;
	double m_chuteDiceRoll = 0.0;
	int dice_roll = 0;
	int dice_roll1 = 0;
	int dice_roll2 = 0;
	double m_mach_speed = 0.0;
	m_mach_speed = m_state.m_mach;
	
	if (m_chuteState != 0)
	{
		if (m_mach_speed >= 0.12)
		{
			/*std::default_random_engine generator;
			std::uniform_int_distribution<int> distribution(1, 19);
			dice_roll = distribution(generator);  // generates number in the range 1-19 */
			
			int mill_seconds = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

			srand(mill_seconds); //vorher srand(timt(0)); das gab die sekunden-ticks zurück, wir brauchen aber millisekunden

			dice_roll = (rand() % 19) + 1;

			if ((dice_roll == 2) || (dice_roll == 1))
			{
				m_chuteDiceRoll = -0.65;	
			}
			if ((dice_roll == 3) || (dice_roll ==4 ))
			{
				m_chuteDiceRoll = -0.55;	
			}
			if ((dice_roll == 5) || (dice_roll == 6))
			{
				m_chuteDiceRoll = -0.40;	
			}
			if ((dice_roll == 7) || (dice_roll == 8))
			{
				m_chuteDiceRoll = -0.19;
			}
			if ((dice_roll == 9) || (dice_roll == 10))
			{
				m_chuteDiceRoll = 0;	
			}
			if ((dice_roll == 11) || (dice_roll == 12))
			{
				m_chuteDiceRoll = 0.19;	
			}
			if ((dice_roll == 13) || (dice_roll == 14))
			{
				m_chuteDiceRoll = 0.40;	
			}
			if ((dice_roll == 15) || (dice_roll == 16))
			{
				m_chuteDiceRoll = 0.55;	
			}
			if (dice_roll >= 17)
			{
				m_chuteDiceRoll = 0.65;	
			}
			
			m_chuteSlewY = m_chuteDiceRoll;
			return m_chuteSlewY;
		}
		if ((m_mach_speed >= 0.05) && (m_mach_speed < 0.11))
		{
			/*std::default_random_engine generator;
			std::uniform_int_distribution<int> distribution(1, 11);
			dice_roll1 = distribution(generator);  // generates number in the range -0.20....0.20*/ 
			
			int mill_seconds = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

			srand(mill_seconds); //vorher srand(timt(0)); das gab die sekunden-ticks zurück, wir brauchen aber millisekunden

			dice_roll1 = (rand() % 11) + 1;

			if (dice_roll1 <= 2)
			{
				m_chuteDiceRoll = -0.35;
			}
			if ((dice_roll1 == 3) || (dice_roll1 == 4))
			{
				m_chuteDiceRoll = -0.15;
			}
			if ((dice_roll1 == 5) || (dice_roll1 == 6))
			{
				m_chuteDiceRoll = -0.05;
			}
			if ((dice_roll1 == 7) || (dice_roll1 == 8))
			{
				m_chuteDiceRoll = 0.10;
			}
			if (dice_roll1 >= 9)
			{
				m_chuteDiceRoll = 0.19;
			}
			m_chuteSlewY = m_chuteDiceRoll;
			return m_chuteSlewY;
		}
		if ((m_mach_speed < 0.05) && (m_mach_speed > 0.00))
		{
			/*std::default_random_engine generator;
			std::uniform_int_distribution<int> distribution(1, 11);
			dice_roll2 = distribution(generator);  // generates number in the range -0.20....0.20*/
			
			int mill_seconds = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

			srand(mill_seconds); //vorher srand(timt(0)); das gab die sekunden-ticks zurück, wir brauchen aber millisekunden


			dice_roll2 = (rand() % 11) + 1;

			if (dice_roll2 <= 2)
			{
				m_chuteDiceRoll = -0.99;
			}
			if ((dice_roll2 == 3) || (dice_roll2 == 4))
			{
				m_chuteDiceRoll = -0.92;
			}
			if ((dice_roll2 == 5) || (dice_roll2 == 6))
			{
				m_chuteDiceRoll = -0.90;
			}
			if ((dice_roll2 == 7) || (dice_roll2 == 8))
			{
				m_chuteDiceRoll = -0.89;
			}
			if (dice_roll2 >= 9)
			{
				m_chuteDiceRoll = -0.87;
			}
			m_chuteSlewY = m_chuteDiceRoll;
			return m_chuteSlewY;
		}
		if (m_mach_speed == 0.00)
		{
		m_chuteSlewY = -1.00;
		return m_chuteSlewY;
		}
	}
}

double Airframe::brkChuteSlewZ()
{
	m_chuteSlewZ = 0.0;
	double m_chuteDiceRoll = 0.0;
	int dice_roll = 0;
	int dice_roll1 = 0;
	int dice_roll2 = 0;
	double m_mach_speed = 0.0;
	m_mach_speed = m_state.m_mach;

	if (m_chuteState != 0)
	{
	
		if (m_mach_speed >= 0.12)
		{
			/*std::default_random_engine generator;
			std::uniform_int_distribution<int> distribution(1, 19);
			dice_roll = distribution(generator);  // generates number in the range -0.75....0.75 
			*/
			int mill_seconds = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

			srand(mill_seconds); //vorher srand(timt(0)); das gab die sekunden-ticks zurück, wir brauchen aber millisekunden

			dice_roll = (rand() % 19) + 1;

			if (dice_roll <= 3)
			{
				m_chuteDiceRoll = -0.65;
				dice_roll = 0;
			}
			if ((dice_roll == 4) || (dice_roll == 5))
			{
				m_chuteDiceRoll = -0.55;
			}
			if ((dice_roll == 6) || (dice_roll == 7))
			{
				m_chuteDiceRoll = -0.40;
			}
			if ((dice_roll == 8) || (dice_roll == 9))
			{
				m_chuteDiceRoll = -0.19;
			}
			if ((dice_roll == 10) || (dice_roll == 11))
			{
				m_chuteDiceRoll = 0;
			}
			if ((dice_roll == 12) || (dice_roll == 13))
			{
				m_chuteDiceRoll = 0.19;
			}
			if ((dice_roll == 14) || (dice_roll == 15))
			{
				m_chuteDiceRoll = 0.40;
			}
			if ((dice_roll == 16) || (dice_roll == 17))
			{
				m_chuteDiceRoll = 0.55;
			}
			if (dice_roll >= 18)
			{
				m_chuteDiceRoll = 0.65;
			}
			m_chuteSlewZ = m_chuteDiceRoll;
			return m_chuteSlewZ;
			}
		if ((m_mach_speed >= 0.05) && (m_mach_speed <= 0.11))
			{
			/*std::default_random_engine generator;
			std::uniform_int_distribution<int> distribution(1, 11);
			dice_roll1 = distribution(generator);  // generates number in the range -0.20....0.20 
			*/

			int mill_seconds = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

			srand(mill_seconds); //vorher srand(timt(0)); das gab die sekunden-ticks zurück, wir brauchen aber millisekunden

			dice_roll1 = (rand() % 11) + 1;

			if (dice_roll1 <= 2)
			{
				m_chuteDiceRoll = -0.25;
			}
			if ((dice_roll1 == 3) || (dice_roll1 == 4))
			{
				m_chuteDiceRoll = -0.15;
			}
			if ((dice_roll1 == 5) || (dice_roll1 == 6))
			{
				m_chuteDiceRoll = -0.00;
			}
			if ((dice_roll1 == 7) || (dice_roll1 == 8))
			{
				m_chuteDiceRoll = 0.15;
			}
			if (dice_roll1 >= 9)
			{
				m_chuteDiceRoll = 0.25;
			}
			m_chuteSlewZ = m_chuteDiceRoll;
			return m_chuteSlewZ;
			}
		if ((m_mach_speed < 0.05) && (m_mach_speed > 0.00))
			{
			/*std::default_random_engine generator;
			std::uniform_int_distribution<int> distribution(1, 11);
			dice_roll2 = distribution(generator);  // generates number in the range -0.20....0.20 
			*/
			int mill_seconds = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count(); //NEU wegen Chrono

			srand(mill_seconds); //vorher srand(timt(0)); das gab die sekunden-ticks zurück, wir brauchen aber millisekunden

			dice_roll2 = (rand() % 11) + 1;

			if (dice_roll2 <= 2)
			{
				m_chuteDiceRoll = -0.09;
			}
			if ((dice_roll2 == 3) || (dice_roll2 == 4))
			{
				m_chuteDiceRoll = -0.05;
			}
			if ((dice_roll2 == 5) || (dice_roll2 == 6))
			{
				m_chuteDiceRoll = 0.00;
			}
			if ((dice_roll2 == 7) || (dice_roll2 == 8))
			{
				m_chuteDiceRoll = 0.05;
			}
			if (dice_roll2 >= 9)
			{
				m_chuteDiceRoll = 0.09;
			}
			m_chuteSlewZ = m_chuteDiceRoll;
			return m_chuteSlewZ;
		}
		if (m_mach_speed == 0.00)
		{
		m_chuteSlewZ = 0.0;
		
		//printf("ChuteSlewZ-Value %f \n", m_chuteSlewZ);

		return m_chuteSlewZ;
		}

	}
}

double Airframe::BLCsystem()
{
	m_blcLift = 0.0;

	if ((getFlapsPosition() == 1.0) && (m_engine.getRPMNorm() >= 0.85))
	{
		m_blcLift = (0.50 * m_engine.getRPMNorm()) * CON_FlpL2;
	}

	else
	{
		m_blcLift = 0.0;
	}
	return m_blcLift;
}

double Airframe::getEngineDamageMult()
{
	if ((getCompressorDamage() <= 0.75) || (getTurbineDamage() <= 0.75))
	{
		m_engDmgMulti = ((getCompressorDamage() + getTurbineDamage()) / 2.0) * 1.15;
	}
	else
	{
		m_engDmgMulti = 1.0;
	}
	return m_engDmgMulti;
}

void Airframe::autoPilotAltH(double dt)
{
	
	m_ascHA = 0.1309; // = 7,5° in Radians
	m_decHA = -0.1309;

	double altHoldCorridorLow = m_altHold * 1.02;
	double altHoldCorridorHigh = m_altHold * 0.98;
	
	if (m_state.m_angle.z > m_ascHA)
	{
		m_acendHoldAngle = true;
	}
	else
	{
		m_acendHoldAngle = false;
	}

	if (m_state.m_angle.z < m_decHA)
	{
		m_decendHoldAngle = true;
	}
	else
	{
		m_decendHoldAngle = false;
	}

	if (m_state.m_angle.z == m_state.m_aoa)
	{
		m_level = true;
	}
	else
	{
		m_level = false;
	}

	if ((m_state.m_angle.z > 0.0) && ((m_state.m_angle.z - m_state.m_aoa) > 0.0))
	{
		m_acend = true;
	}
	else
	{
		m_acend = false;
	}

	if (m_state.m_angle.z < 0.0)
	{
		m_decend = true;
	}
	else
	{
		m_decend = false;
	}
	
	if (m_input.getAutoPEng() == 1.0)
	{
		m_desiredAlt = m_state.m_airDensity;

		if ((m_input.getAutoPEng() == 1.0) && (m_altHold > m_state.m_airDensity) && ((m_state.m_airDensity < altHoldCorridorHigh) || (m_state.m_airDensity > altHoldCorridorLow)))
		{
			if ((m_decendHoldAngle == false) && (m_altHold != 0.0))
			{
				if (m_state.m_angle.z == m_decHA)
				{
					m_pitchAPadj = 0.0;
				}
				if (m_state.m_angle.z > m_decHA)
				{
					m_pitchAPadj = -0.03;
				}
			}

			if ((m_decendHoldAngle == true) && (m_altHold != 0.0))
			{
				if (m_state.m_angle.z < -0.34)
				{
					m_pitchAPadj = 0.05;
				}
				if ((m_state.m_angle.z >= -0.34) && (m_state.m_angle.z < m_decHA))
				{
					m_pitchAPadj = 0.03;
				}
			}
		}
		
		if ((m_input.getAutoPEng() == 1.0) && (m_altHold < m_state.m_airDensity) && ((m_state.m_airDensity < altHoldCorridorHigh) || (m_state.m_airDensity > altHoldCorridorLow)))
		{
			if ((m_acendHoldAngle == false) && (m_altHold != 0.0))
			{
				if (m_state.m_angle.z == m_ascHA)
				{
					m_pitchAPadj = 0.0;
				}
				if (m_state.m_angle.z < m_ascHA)
				{
					m_pitchAPadj = 0.03;
				}
			}

			if ((m_acendHoldAngle == true) && (m_altHold != 0.0))
			{
				if (m_state.m_angle.z > 0.34)
				{
					m_pitchAPadj = -0.05;
				}
				if ((m_state.m_angle.z <= 0.34) && (m_state.m_angle.z > m_ascHA))
				{
					m_pitchAPadj = -0.03;
				}
			}
		}
		
		if ((m_input.getAutoPEng() == 1.0) && ((m_state.m_airDensity >= altHoldCorridorHigh) && (m_state.m_airDensity <= altHoldCorridorLow)))
		{
			if ((m_state.m_angle.z == m_state.m_aoa) && (m_state.m_angle.z >= 0.0))
			{
				m_pitchAPadj = 0.0;
			}
			if ((m_state.m_angle.z > m_state.m_aoa) && (m_state.m_angle.z > 0.0))
			{
				m_pitchAPadj = -0.01;
			}
			if ((m_state.m_angle.z < m_state.m_aoa) && (m_state.m_angle.z <= 0.0))
			{
				m_pitchAPadj = 0.02;
			}
			if (m_state.m_angle.z < m_state.m_aoa)
			{
				m_pitchAPadj = 0.01;
			}
			

		}
		
		if ((m_previousAlt != m_desiredAlt) && (m_previousAlt != 0.0) && (m_input.getAutoPEng() == 1) && (m_altHold == 0.0))
		{
			m_altHold = m_state.m_airDensity;
		}
		else if ((m_altHold != 0.0) && (m_previousAlt != m_desiredAlt) && (m_input.getAutoPEng() == 1.0))
		{

		}

		m_previousAlt = m_desiredAlt;
	}
	else if (m_input.getAutoPEng() == 0.0)
	{
		m_pitchAPadj = 0.0;
		m_previousAlt = 0.0;
		m_altHold = 0.0;
		m_desiredAlt = 0.0;
	}



	/*printf("m_altHold %f \n", m_altHold);
	printf("airDensity %f \n", m_state.m_airDensity);
	printf("Angle %f \n", m_state.m_angle.z);
	printf("AP_Pitch_Adj. %f \n", m_pitchAPadj);
	printf("AoA %f \n", m_state.m_aoa);*/
}

double Airframe::fuelFlowIndGaugeUpdate()
{
	//direct fuelFlowGauge has 1-100 with 12.000 = 100 and 0 = 0; 0-70 = 0 - 6.000 lbs/h; 72,5 = 7.000 lbs/h; 75 = 8.000 lbs/h; 81,25 = 9; 87,5 = 10; 93,75 = 11; 100 = 12

	if (m_engine.FuelFlowUpdate() <= 6000.0)
	{
		m_fuelThousand = m_engine.FuelFlowUpdate() * 0.01166;
	}
	else if ((m_engine.FuelFlowUpdate() > 6000) && (m_engine.FuelFlowUpdate() < 12000.0))
	{
		m_fuelThousand = (m_engine.FuelFlowUpdate() * 0.005) + 40.0;
	}
	else if (m_engine.FuelFlowUpdate() >= 12000.0)
	{
		m_fuelThousand = 100.0;
	}

	return m_fuelThousand * 0.01;
}


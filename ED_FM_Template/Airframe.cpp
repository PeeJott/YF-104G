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
	m_actuatorFlap(1.75),
	m_actuatorAirbrk(1.2),
	m_actuatorGearL(0.6),
	m_actuatorGearN(0.6),
	m_actuatorGearR(0.6),
	m_actuatorHook(0.6),
	m_actuatorNozzle(1.35),
	m_actuatorNosewheel(2.0)
{
	//huhu!!
}

void Airframe::zeroInit()
{
	//---Gear position--------
	m_gearLPosition = 0.0;
	m_gearRPosition = 0.0;
	m_gearNPosition = 0.0;

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

	m_nozzlePosition = 0.0;

	m_int_throttlePos = 0.0;

	//set to "0" after each init
	m_input.m_brake = 0.0;

	m_nwsEngage = 0.0;
	m_chuteState = 0.0;
	m_mass = 1.0;
	m_timePassed = 0.0;
}

void Airframe::coldInit()
{
	//--Ground Init SET Actuator---------
	m_actuatorGearL.groundInit(1.0, 1.0);
	m_actuatorGearN.groundInit(1.0, 1.0);
	m_actuatorGearR.groundInit(1.0, 1.0);
	//---Gear position--------
	m_gearLPosition = 0.0;
	m_gearRPosition = 0.0;
	m_gearNPosition = 0.0;

	//-----gear position for ground start
	m_gearStart = 0;
	//m_actuatorGearL = 0.0;
	//m_actuatorGearN = 0.0;
	//m_actuatorGearR = 0.0;
	m_input.m_gear_toggle = 1;
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

	m_input.m_brake = 0;

	m_nwsEngage = 0.0;
	m_chuteState = 0.0;
	m_mass = 1.0;
	m_timePassed = 0.0;
}

void Airframe::hotInit()
{
	//--Ground Init SET Actuator---------
	m_actuatorGearL.groundInit(1.0, 1.0);
	m_actuatorGearN.groundInit(1.0, 1.0);
	m_actuatorGearR.groundInit(1.0, 1.0);
	//---Gear position--------
	m_gearLPosition = 0.0;
	m_gearRPosition = 0.0;
	m_gearNPosition = 0.0;

	//---Gear position for ground start----------
	m_gearStart = 0;
	//m_actuatorGearL = 0.0;
	//m_actuatorGearN = 0.0;
	//m_actuatorGearR = 0.0;
	m_input.m_gear_toggle = 1;

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

	m_input.m_brake = 0;
	
	m_nwsEngage = 0.0;
	m_chuteState = 0.0;
	m_mass = 1.0;
	m_timePassed = 0.0;
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
}

double Airframe::updateBrake()
{
	m_brakeMoment = 0.0;
	
	if (m_input.m_release_brake == 1)
	{
		if (m_input.m_brake == 1)
		{
			m_input.m_brake = 0;
			m_input.m_release_brake = 0;
		}
	}
	
	if (m_input.m_brake == 1)
	{
		m_brakeMoment = 0.66;
	}
	else
	{
		m_brakeMoment = 0;
	}
	
	return m_brakeMoment;
}

double Airframe::setNozzlePosition(double dt) //Nozzle-Position 0-10% Thrust open, 11-84% Thrust closed, 85-100% Thrust open
{
	double NozzlePos = 0.0;
	double corrThrottle = 0.0;

	if (m_input.m_throttle >= 0.0)
	{
		corrThrottle = (1 - CON_ThrotIDL) * m_input.m_throttle + CON_ThrotIDL;
	}
	else
	{
		corrThrottle = (m_input.m_throttle + 1.0) / 2.0;
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

	if (m_input.m_throttle >= 0.0)
	{
		corrThrottle = (1 - CON_ThrotIDL) * m_input.m_throttle + CON_ThrotIDL;
	}
	else
	{
		corrThrottle = (m_input.m_throttle + 1.0) / 2.0;
	}

	m_int_throttlePos = corrThrottle;

	return m_int_throttlePos;
}

double Airframe::NWSstate()
{
	if (m_input.m_nwsteering == 1)
	{
		m_nwsEngage = 1;
	}
	else
	{
		m_nwsEngage = 0;
	}
	return m_nwsEngage;
}

double Airframe::brkChutePosition()
{
	int timeToGo = 50;

	if ((m_input.m_brkchute == 1) && (m_timePassed < timeToGo))
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
	else if (m_input.m_brkchute == 0)
	{
		m_chuteState = 0.0;
		m_timePassed = 0.0;
	}

	return m_chuteState;
}


double Airframe::brkChuteSlewY()
{
	m_chuteSlewY = 0.0;
	float m_chuteDiceRoll = 0.0;
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
		return m_chuteSlewZ;
		}

	}
}

#include "Airframe.h"
#include <algorithm>


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
	m_actuatorAirbrk(1.3),
	m_actuatorGearL(0.9),
	m_actuatorGearN(0.9),
	m_actuatorGearR(0.6),
	m_actuatorHook(0.6),
	m_actuatorNozzle(1.75),
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

	//set to "0" after each init
	m_input.m_brake = 0.0;

	m_nwsEngage = 0.0;
	m_chuteState = 0.0;
	m_mass = 1.0;
}

void Airframe::coldInit()
{
	//---Gear position--------
	m_gearLPosition = 0.0;
	m_gearRPosition = 0.0;
	m_gearNPosition = 0.0;

	//-----gear position for ground start
	m_gearStart = 1;
	//m_actuatorGearL = 0.0;
	//m_actuatorGearN = 0.0;
	//m_actuatorGearR = 0.0;

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

	m_input.m_brake = 0;

	m_nwsEngage = 0.0;
	m_chuteState = 0.0;
	m_mass = 1.0;
}

void Airframe::hotInit()
{
	//---Gear position--------
	m_gearLPosition = 0.0;
	m_gearRPosition = 0.0;
	m_gearNPosition = 0.0;

	//---Gear position for ground start----------
	m_gearStart = 1;
	//m_actuatorGearL = 0.0;
	//m_actuatorGearN = 0.0;
	//m_actuatorGearR = 0.0;

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

	m_input.m_brake = 0;
	
	m_nwsEngage = 0.0;
	m_chuteState = 0.0;
	m_mass = 1.0;
}

void Airframe::airborneInit()
{
	zeroInit();
}

double Airframe::updateStartGearGO()
{
	if (m_state.m_mach < 0.25)
	{
		if (m_gearStart == 1)
		{
			m_gearStartDown = 1;
		}
	}
	else
	{
		m_gearStartDown = 2;
	}
	printf("m_gearStartDown-Value %f \n", m_gearStartDown);
	
	return m_gearStartDown;
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
	
	if (updateStartGearGO() == 1)
	{
		m_gearLPosition = 1;
	}
	else //if (updateStartGearGO() == 2)
	{
		m_gearLPosition = setGearLPosition(dt);
	}
	
	if (updateStartGearGO() == 1)
	{
		m_gearRPosition = 1;
	}
	else //if (updateStartGearGO() == 2)
	{
		m_gearRPosition = setGearRPosition(dt);
	}
	if (updateStartGearGO() == 1)
	{
		m_gearNPosition = 1;
	}
	else //if (updateStartGearGO() == 2)
	{
		m_gearNPosition = setGearNPosition(dt);
	}
	
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


//!!!!funktionierende Update-Funktion OHNE Acutators!!!
/*double Airframe::updateFlaps()
{
	m_flapsPosition = 0.0;

	//Dieser Abschnitt funktioniert mit der Public-Function in der Airframe.h, aber ohne actuators
	if (m_input.m_flaps_toggle == 0.5)
	{
		m_flapsPosition = 0.5;
	}
	else if (m_input.m_flaps_toggle == 1)
	{
		m_flapsPosition = 1;
	}
	else
	{
		m_flapsPosition = 0;
	}
	

	//printf("Flp_Toggle-Value %f \n", m_input.m_flaps_toggle);
	//printf("Flp-Position-Value %f \n", m_flapsPosition);

	return m_flapsPosition;
}*/
double Airframe::updateBrake()
{
	m_brakeMoment = 0.0;

	if (m_input.m_brake == 1)
	{
		m_brakeMoment = 1;
	}
	else if (m_input.m_brake != 1)
	{
		m_brakeMoment = 0;
	}
	return m_brakeMoment;
}

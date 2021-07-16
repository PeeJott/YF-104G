#include "State.h"

void State::zeroInit()
{

	m_worldPosition = Vec3();
	m_worldVelocity = Vec3();
	m_worldWind = Vec3();
	
	m_angle = Vec3();
	m_omega = Vec3();
	m_omegaDot = Vec3();
	
	m_localSpeed = Vec3();
	m_localAirspeed = Vec3();
	m_localAcceleration = Vec3();
	
	m_com = Vec3();

	m_aoa = 0.0; // ge�ndert von m_aoa; zu m_aoa = 0.0; und initialisiert
	m_beta = 0.0;
	m_mach = 0.0;
	m_speedOfSound = 0.0;
	m_airDensity = 0.0;
	m_temperature = 0.0;
	m_pressure = 0.0;

}

void State::coldInit()
{
	zeroInit();
}

void State::hotInit()
{
	zeroInit();
}

void State::airborneInit()
{
	zeroInit();
}
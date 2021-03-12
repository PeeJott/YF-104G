﻿#pragma once
#ifndef STATE_H
#define STATE_H

#include "Vec3.h"
struct State
{
	State() {}

	inline void setCurrentStateBodyAxis(double aoa, double beta, const Vec3& angle, const Vec3& omega, const Vec3& omegaDot, const Vec3& speed, const Vec3& airspeed, const Vec3& acceleration);
	inline void setCurrentStateWorldAxis( const Vec3& worldPosition, const Vec3& worldVelocity );
	inline void setCurrentAtmosphere( double temperature, double speedOfSound, double density, double pressure, const Vec3& wind );

	inline void setMach( double mach );
	inline void setCOM( const Vec3& com );

	Vec3 m_worldPosition;
	Vec3 m_worldVelocity;
	Vec3 m_worldWind;
	Vec3 m_angle;
	Vec3 m_omega;
	Vec3 m_omegaDot;
	Vec3 m_localSpeed;
	Vec3 m_localAirspeed;
	Vec3 m_localAcceleration;
	Vec3 m_com; //centre of mass
	double m_aoa;
	double m_beta;
	double m_mach;
	double m_speedOfSound;
	double m_airDensity;
	double m_temperature;
	double m_pressure;
};

void State::setCurrentStateBodyAxis(double aoa, double beta, const Vec3& angle, const Vec3& omega, const Vec3& omegaDot, const Vec3& speed, const Vec3& airspeed, const Vec3& acceleration)
{
	m_aoa = aoa;
	m_beta = beta;
	m_angle = angle;
	m_omega = omega;
	m_omegaDot = omegaDot;
	m_localSpeed = speed;
	m_localAirspeed = airspeed;
	m_localAcceleration = acceleration;
}

void State::setCurrentStateWorldAxis( const Vec3& worldPosition, const Vec3& worldVelocity )
{
	m_worldPosition = worldPosition;
	m_worldVelocity = worldVelocity;
}

void State::setCurrentAtmosphere( double temperature, double speedOfSound, double density, double pressure, const Vec3& wind )
{
	m_temperature = temperature;
	m_speedOfSound = speedOfSound;
	m_airDensity = density;
	m_pressure = pressure;
	m_worldWind = wind;
}

inline void State::setMach( double mach )
{
	m_mach = mach;
}

void State::setCOM( const Vec3& com )
{
	m_com = com;
}

#endif
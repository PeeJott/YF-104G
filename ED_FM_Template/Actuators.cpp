#include "Actuators.h"
#include "Maths.h"
#include <cmath>

Actuator::Actuator() : m_actuatorSpeed{ 10.0 }, m_actuatorPos{ 0.0 }, m_actuatorTargetPos{ 0.0 }, m_actuatorFactor{ 1.0 }
{

}

Actuator::Actuator(double speed) : m_actuatorSpeed{ speed }, m_actuatorPos{ 0.0 }, m_actuatorTargetPos{ 0.0 }, m_actuatorFactor{ 1.0 }
{

}

Actuator::~Actuator()
{

}

void Actuator::zeroInit()
{

}
void Actuator::coldInit()
{

}
void Actuator::hotInit()
{

}
void Actuator::airborneInit()
{

}

double Actuator::inputUpdate(double targetPosition, double dt)
{
	m_actuatorTargetPos = targetPosition;

	physicsUpdate(dt);

	return m_actuatorPos;
}

void Actuator::physicsUpdate(double dt)
{
	double speedToTarget = (m_actuatorTargetPos - m_actuatorPos) / dt;


	double actuatorSpeed = 0.0;

	// dependent on aerodynamic load
	if (m_actuatorPos > 0.0)
	{
		if (m_actuatorTargetPos - m_actuatorPos < 0.0)
		{
			actuatorSpeed = m_actuatorSpeed;
		}
		else
		{
			actuatorSpeed = m_actuatorSpeed * m_actuatorFactor;
		}
	}
	else
	{
		if (m_actuatorTargetPos - m_actuatorPos > 0.0)
		{
			actuatorSpeed = m_actuatorSpeed;
		}
		else
		{
			actuatorSpeed = m_actuatorSpeed * m_actuatorFactor;
		}
	}


	if (abs(speedToTarget) <= actuatorSpeed)
	{
		m_actuatorPos = m_actuatorTargetPos;
	}
	else
	{
		m_actuatorPos += copysign(1.0, speedToTarget) * actuatorSpeed * dt;
	}

	m_actuatorPos = clamp(m_actuatorPos, -1.0, 1.0);
}

double Actuator::getPosition()
{
	return m_actuatorPos;
}

void Actuator::setActuatorSpeed(double factor)
{
	m_actuatorFactor = factor;
}
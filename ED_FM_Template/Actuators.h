#pragma once
#include "BaseComponent.h"

class Actuator
{
public:
	Actuator();
	Actuator(double speed);
	~Actuator();

	virtual void Actuator::zeroInit();
	void Actuator::groundInit(double pos, double targetpos); //neu eingef�gt f�r GroundInits
	virtual void Actuator::hotInit();
	virtual void Actuator::airborneInit();

	double inputUpdate(double targetPosition, double dt);
	void physicsUpdate(double dt);
	double getPosition();
	void setActuatorSpeed(double factor);

private:
	double m_actuatorFactor;
	double m_actuatorSpeed;
	double m_actuatorPos;
	double m_actuatorTargetPos;
};
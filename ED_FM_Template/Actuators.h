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

private: // = 0.0; bei allen actuators eingef�gt, vorher fehlte das!!!
	double m_actuatorFactor = 0.0;
	double m_actuatorSpeed = 0.0;
	double m_actuatorPos = 0.0;
	double m_actuatorTargetPos = 0.0;
};
#pragma once
#include <stdio.h>
#include "Table.h"
#include "Vec3.h"
#include "State.h"
#include "Input.h"
#include "AeroData_1.h"
#include "Engine.h"
#include "Maths.h"
#include "Actuators.h"
#include "BaseComponent.h"


class Airframe
{
public:
	Airframe(State& state, Input& input, Engine& engine); 

	//Initialization
	virtual void zeroInit();
	virtual void coldInit();
	virtual void hotInit();
	virtual void airborneInit();

	//---------Setting positions-------------------------------------------------
	//Gear
	inline void setGearLPosition(double position); //for airstart or ground start
	inline void setGearRPosition(double position); //for airstart or ground start
	inline void setGearNPosition(double position); //for airstart or ground start
	
	//Flaps
	inline void setFlapsPosition(double position);
	
	//Airbrake
	inline void setAirbrakePosition(double position);
	
	//Fuel
	//void addFuel(double fuel); //fuel Funktion fehlt aktuell noch

	//Aerodynamic surfaces
	inline double setAileron(double dt);
	inline double setRudder(double dt);
	inline double setStabilizer(double dt);

	//Steering
	inline void setNoseWheelAngle(double angle);

	//---Masses-----
	inline void setMass(double angle);

	//------Getting and returning positions-------------------
	inline double getGearLPosition() const; //returns gear pos
	inline double getGearRPosition() const; //returns gear pos
	inline double getGearNPosition() const; //returns gear pos

	inline double getFlapsPosition() const;
	inline double getSpeedBrakePosition() const;
	
	inline double getHookPosition() const;
	inline double getMass() const;
	inline double getAileron() const;
	inline double getRudder() const;
	inline double getStabilizer() const;

	inline double getNoseWheelAngle() const;

	inline double aileronAngle();
	inline double stabilizerAngle();
	inline double rudderAngle();
	
	void airframeUpdate(double dt);


private:
	Vec3 m_moment;
	Vec3 m_force;
	State& m_state;
	Input& m_input;
	Engine& m_engine; //neu 21Feb21 // wieder rausgenommen

	//Gear
	double m_gearLPosition = 0.0; //0 -> 1
	double m_gearRPosition = 0.0;
	double m_gearNPosition = 0.0;

	//aerodynamic surfaces
	double m_flapsPosition = 0.0;
	double m_speedBrakePosition = 0.0;
	double m_hookPosition = 0.0;

	double m_aileronLeft = 0.0;
	double m_aileronRight = 0.0;
	double m_stabilizer = 0.0;
	double m_rudder = 0.0;

	double m_noseWheelAngle = 0.0;

	Actuator m_actuatorStab; //scheint nur zur optischen "Verschönerung" zu sein, aber egal
	Actuator m_actuatorAil;
	Actuator m_actuatorRud;


	//double m_stabilizerZeroForceDeflection = 0.0;

	double m_mass = 1.0;
};

double Airframe::setAileron(double dt)
{
	double input = m_input.m_roll; // +m_input.m_rollTrim(); // m_rollTrim kommt noch
	return m_actuatorAil.inputUpdate(input, dt);
}

//Folgend Auskommentierungen zum Testen des Stabilizers, da Aileron und Rudder so funktionieren
double Airframe::setStabilizer(double dt)
{
	double input = m_input.m_pitch; // +m_stabilizerZeroForceDeflection;
	return m_actuatorStab.inputUpdate(input, dt);
	
	//double stabilizer = toDegrees(m_stabilizer);
	//double bungeeTrimDeg = (stabilizer + 1.0) / (-13.25) * (-0.65306 - 7.3469) - 0.65306;
	//double bungeeTrimStick = bungeeTrimDeg / 20.0; // transformation from control surface deflection to stick normalized coordinate goes here
	//double speedbrakeTrim = -0.15 * m_speedBrakePosition;
	//m_actuatorStab.setActuatorSpeed(clamp(1.0 - 1.2 * pow(m_state.m_mach, 3.0), 0.1, 1.0));
	//printf("factor: %lf\n", clamp(1.0 - 1.2 * pow(m_state.getMach(), 3.0), 0.1, 1.0));

	//m_stabilizerZeroForceDeflection = bungeeTrimStick + speedbrakeTrim;

}

//den folgenden rausgeworfen, da er wohl einen wie auch immer gearteten Stabilizer bedient
/*double Airframe::setStabilizer(double dt)
{
	double input = m_input.m_pitch; // +m_controls.yawTrim(); Yaw-Trim kommt noch
	return m_actuatorStab.inputUpdate(input, dt);
}*/

double Airframe::setRudder(double dt)
{
	double input = m_input.m_yaw; // +m_controls.yawTrim(); Yaw-Trim kommt noch
	return m_actuatorRud.inputUpdate(input, dt);
}


void Airframe::setFlapsPosition(double position)
{
	m_flapsPosition = position;
}

void Airframe::setGearLPosition(double position)
{
	m_gearLPosition = position;
}

void Airframe::setGearRPosition(double position)
{
	m_gearRPosition = position;
}

void Airframe::setGearNPosition(double position)
{
	m_gearNPosition = position;
}

void Airframe::setAirbrakePosition(double position)
{
	m_speedBrakePosition = position;
}

void Airframe::setMass(double mass)
{
	m_mass = mass;
}

void Airframe::setNoseWheelAngle(double angle)
{
	m_noseWheelAngle = angle;
}

double Airframe::getNoseWheelAngle() const
{
	return m_noseWheelAngle;
}

double Airframe::getGearLPosition() const
{
	return m_gearLPosition;
}

double Airframe::getGearRPosition() const
{
	return m_gearRPosition;
}

double Airframe::getGearNPosition() const
{
	return m_gearNPosition;
}

double Airframe::getFlapsPosition() const
{
	return m_flapsPosition;
}

double Airframe::getSpeedBrakePosition() const
{
	return m_speedBrakePosition;
}

double Airframe::getHookPosition() const
{
	return m_hookPosition;
}

double Airframe::getAileron() const
{
	return m_aileronLeft;
}

double Airframe::getStabilizer() const
{
	return m_stabilizer;
}

double Airframe::getRudder() const
{
	return m_rudder;
}

double Airframe::aileronAngle()
{
	return 	m_aileronLeft > 0.0 ? CON_aitnu * m_aileronLeft : -CON_aitnd * m_aileronLeft;
}

double Airframe::stabilizerAngle()
{
	return m_stabilizer > 0.0 ? -CON_hstdUP * m_stabilizer : CON_hstdDN * m_stabilizer;
}

double Airframe::rudderAngle()
{
	return m_rudder > 0.0 ? CON_RdDefGUR * m_rudder : -CON_RdDefGUL * m_rudder;
}

double Airframe::getMass() const
{
	return m_mass;
}


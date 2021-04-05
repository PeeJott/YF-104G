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
	//inline void setGearLPosition(double position); //for airstart or ground start-OLD
	//inline void setGearRPosition(double position); //for airstart or ground start-OLD
	//inline void setGearNPosition(double position); //for airstart or ground start-OLD
	inline double setGearLPosition(double dt);
	inline double setGearRPosition(double dt);
	inline double setGearNPosition(double dt);
	//Flaps
	//inline void setFlapsPosition(double position); //Ursprünglich funktioniert insges.-OLD
	//inline double setFlapsPosition(double dt); //Neu und ein Versuch-OLD
													 
	//Airbrake
	//inline void setAirbrakePosition(double position);//OLD
	inline double setAirbrakePosition(double dt);
	
	//Fuel
	//void addFuel(double fuel); //fuel Funktion fehlt aktuell noch

	//Aerodynamic surfaces
	inline double setAileron(double dt);
	inline double setRudder(double dt);
	inline double setStabilizer(double dt);
	
	//NEW Flaps and Airbrake
	inline double setFlapsPosition(double dt);

	//Steering
	inline void setNoseWheelAngle(double angle);

	//---Masses-----
	inline void setMass(double angle);

	//------Getting and returning positions-------------------
	inline double getGearLPosition() const; //returns gear pos
	inline double getGearRPosition() const; //returns gear pos
	inline double getGearNPosition() const; //returns gear pos

	inline double getSpeedBrakePosition() const;
	
	inline double getHookPosition() const;
	inline double getMass() const;
	inline double getAileron() const;
	inline double getRudder() const;
	inline double getStabilizer() const;
	inline double getFlapsPosition() const;

	//--------Setting/Getting Angles-------------------------
	inline double getNoseWheelAngle() const;

	inline double aileronAngle();
	inline double stabilizerAngle();
	inline double rudderAngle();
	
	inline double flapsAngle();
	
	inline double gearLAngle();
	inline double gearRAngle();
	inline double gearNAngle();

	inline double airbrakeAngle();
	
	
	void airframeUpdate(double dt);

	//!!! die update Funktion funktioniert, allerdings OHNE Actuators!!!!
	//double updateFlaps();


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
	Actuator m_actuatorFlap;
	Actuator m_actuatorGearL;
	Actuator m_actuatorGearR;
	Actuator m_actuatorGearN;
	Actuator m_actuatorAirbrk;


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

double Airframe::setFlapsPosition(double dt)
{
	double input = m_input.m_flaps_toggle;
	return m_actuatorFlap.inputUpdate(input, dt);
}


/*void Airframe::setFlapsPosition(double position) //ALT aber insges. funktionstüchtig
{
	m_flapsPosition = position;
}*/

//Jetzt mal ganz anders
/*double Airframe::updateFlaps() 
{
	return m_flapsPosition;

	printf("Flp_Position %f \n", m_flapsPosition);
}*/

double Airframe::setGearLPosition(double dt)
{
	double input = m_input.m_gear_toggle;
	return m_actuatorGearL.inputUpdate(input, dt);
}

double Airframe::setGearRPosition(double dt)
{
	double input = m_input.m_gear_toggle;
	return m_actuatorGearR.inputUpdate(input, dt);
}

double Airframe::setGearNPosition(double dt)
{
	double input = m_input.m_gear_toggle;
	return m_actuatorGearN.inputUpdate(input, dt);
}

double Airframe::setAirbrakePosition(double dt)
{
	double input = m_input.m_airbrk;
	return m_actuatorAirbrk.inputUpdate(input, dt);
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

double Airframe::getFlapsPosition() const
{
	return m_flapsPosition;
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

//FlapsAngle scheint egal zu sein, da fester Ausschlag 0-50%-100%
double Airframe::flapsAngle()
{
	return m_flapsPosition > 0.0 ? -CON_teft2 * m_flapsPosition : 0.001 * m_flapsPosition;
}

double Airframe::gearLAngle()
{
	return m_gearLPosition > 0.0 ? 1 * m_gearLPosition : 0.0001 * m_gearLPosition;
}

double Airframe::gearRAngle()
{
	return m_gearRPosition > 0.0 ? 1 * m_gearRPosition : 0.0001 * m_gearRPosition;
}

double Airframe::gearNAngle()
{
	return m_gearNPosition > 0.0 ? 1 * m_gearNPosition : 0.0001 * m_gearNPosition;
}

double Airframe::airbrakeAngle()
{
	return m_speedBrakePosition > 0.0 ? CON_BrkAngl * m_speedBrakePosition : 0.0001 * m_speedBrakePosition;
}

double Airframe::getMass() const
{
	return m_mass;
}


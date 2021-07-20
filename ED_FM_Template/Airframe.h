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
#include "Units.h"

#define DMG_ELEM(v) m_integrityElement[(int)v]

class Airframe
{
public:
	Airframe(State& state, Input& input, Engine& engine); 
	
	~Airframe();

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
	
	//Airbrake
	//inline void setAirbrakePosition(double position);//OLD
	inline double setAirbrakePosition(double dt);
	
	//Fuel
	//void addFuel(double fuel); //fuel Funktion fehlt aktuell noch

	//Aerodynamic surfaces
	inline double setAileron(double dt);
	inline double setRudder(double dt);
	inline double setStabilizer(double dt);
	
	//Flaps
	inline double setFlapsPosition(double dt);

	//Hook
	inline double setHookPosition(double dt);

	//Engine Nozzle
	double setNozzlePosition(double dt); //verschoben nach Airframe CPP wegen der Gr��e

	//intern FC3 Cockpit-Stuff
	double getIntThrottlePosition();

	//Steering
	inline double setNoseWheelAngle(double dt);

	//---Masses-----
	inline void setMass(double angle);

	//------Getting and returning positions-------------------
	inline double getGearLPosition() const; //returns gear pos
	inline double getGearRPosition() const; //returns gear pos
	inline double getGearNPosition() const; //returns gear pos

	inline double getGearLLamp(); //Gear-Lamp-Left
	inline double getGearRLamp(); //Gear-Lamp-right
	inline double getGearFLamp(); //Gear-Lamp-front

	inline double getSpeedBrakePosition() const;
	
	inline double getHookPosition() const;
	inline double getMass() const;
	inline double getAileron() const;
	inline double getRudder() const;
	inline double getStabilizer() const;
	inline double getFlapsPosition() const;

	inline double getNozzlePosition() const;

	double NWSstate();//verschoben nach Airframe CPP

	double BLCsystem();


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
	inline double noseWheelAngle();
	
	
	void airframeUpdate(double dt);

	//!!! die update Funktion funktioniert, allerdings OHNE Actuators!!!!
	//double updateFlaps();

	//NEU UpdateBrake-Funktion
	double updateBrake();
	
	//Brake-Chute-Funktionen
	double brkChutePosition(); //verschoben nach CPP, daher inline gespart
	double brkChuteSlewZ();
	double brkChuteSlewY();

	//Auto-Pilot-Funktionen
	void autoPilotAltH(double dt);
	inline double getAutoPilotAltH();

	//FlapsPosition Handle and Lights
	inline double getFlapLevPos();
	inline double getFlapIndLEPos();
	inline double getFlapIndTEPos();

	//FuelFlow direct gauge-steering and Indicators
	double fuelFlowIndGaugeUpdate();

	//-------Damage Indicators Aileron and Stabilizer-------------
	inline double ailDamageIndicator();
	inline double stabDamageIndicator();

	



//---------Begin of Damage-Stuff--------------
	enum class Damage
	{
		NOSE_CENTER = 0,
		FRONT = 0,
		Line_NOSE = 0,
		NOSE_LEFT_SIDE = 1,
		NOSE_RIGHT_SIDE = 2,
		COCKPIT = 3,
		CABINA = 3,
		CABIN_LEFT_SIDE = 4,
		CABIN_RIGHT_SIDE = 5,
		CABIN_BOTTOM = 6,
		GUN = 7,
		FRONT_GEAR_BOX = 8,
		GEAR_REAR = 8,
		GEAR_C = 8,
		GEAR_F = 8,
		STOIKA_F = 8,
		FUSELAGE_LEFT_SIDE = 9,
		FUSELAGE_RIGHT_SIDE = 10,
		MAIN = 10,
		LINE_MAIN = 10,
		ENGINE = 11,
		ENGINE_L = 11,
		ENGINE_L_VNUTR = 11,
		ENGINE_L_IN = 11,
		ENGINE_R = 12,
		ENGINE_R_VNUTR = 12,
		ENGINE_R_IN = 12,
		MTG_L_BOTTOM = 13,
		MTG_R_BOTTOM = 14,
		LEFT_GEAR_BOX = 15,
		GEAR_L = 15,
		STOIKA_L = 15,
		RIGHT_GEAR_BOX = 16,
		GEAR_R = 16,
		STOIKA_R = 16,
		MTG_L = 17,
		ENGINE_L_VNESHN = 17,
		ENGINE_L_OUT = 17,
		EWU_L = 17,
		MTG_R = 18,
		ENGINE_R_VNESHN = 18,
		ENGINE_R_OUT = 18,
		EWU_R = 18,
		AIR_BRAKE_L = 19,
		AIR_BRAKE_R = 20,
		WING_L_PART_OUT = 21,
		WING_R_PART_OUT = 22,
		WING_L_OUT = 23,
		WING_R_OUT = 24,
		ELERON_L = 25,
		AILERON_L = 25,
		ELERON_R = 26,
		AILERON_R = 26,
		WING_L_PART_CENTER = 27,
		WING_R_PART_CENTER = 28,
		WING_L_CENTER = 29,
		WING_R_CENTER = 30,
		FLAP_L_OUT = 31,
		FLAP_R_OUT = 32,
		WING_L_PART_IN = 33,
		WING_R_PART_IN = 34,
		WING_L_IN = 35,
		WING_L = 35,
		Line_WING_L = 35,
		WING_R_IN = 36,
		WING_R = 36,
		Line_WING_R = 36,
		FLAP_L_IN = 37,
		FLAP_L = 37,
		FLAP_R_IN = 38,
		FLAP_R = 38,
		FIN_L_TOP = 39,
		KEEL_L_OUT = 39,
		KEEL_OUT = 39,
		FIN_R_TOP = 40,
		KEEL_R_OUT = 40,
		FIN_L_CENTER = 41,
		KEEL_L_CENTER = 41,
		KEEL_CENTER = 41,
		FIN_R_CENTER = 42,
		KEEL_R_CENTER = 42,
		FIN_L_BOTTOM = 43,
		KIL_L = 43,
		Line_KIL_L = 43,
		KEEL = 43,
		KEEL_IN = 43,
		KEEL_L = 43,
		KEEL_L_IN = 43,
		FIN_R_BOTTOM = 44,
		KIL_R = 44,
		Line_KIL_R = 44,
		KEEL_R = 44,
		KEEL_R_IN = 44,
		STABILIZER_L_OUT = 45,
		STABILIZATOR_L_OUT = 45,
		STABILIZER_R_OUT = 46,
		STABILIZATOR_R_OUT = 46,
		STABILIZER_L_IN = 47,
		STABILIZATOR_L = 47,
		STABILIZATOR_L01 = 47,
		Line_STABIL_L = 47,
		STABILIZER_R_IN = 48,
		STABILIZATOR_R = 48,
		STABILIZATOR_R01 = 48,
		Line_STABIL_R = 48,
		ELEVATOR_L_OUT = 49,
		ELEVATOR_R_OUT = 50,
		ELEVATOR_L_IN = 51,
		ELEVATOR_L = 51,
		RV_L = 51,
		ELEVATOR_R_IN = 52,
		ELEVATOR_R = 52,
		RV_R = 52,
		RUDDER_L = 53,
		RN_L = 53,
		RUDDER = 53,
		RUDDER_R = 54,
		RN_R = 54,
		TAIL = 55,
		TAIL_LEFT_SIDE = 56,
		TAIL_RIGHT_SIDE = 57,
		TAIL_BOTTOM = 58,
		NOSE_BOTTOM = 59,
		PWD = 60,
		PITOT = 60,
		FUEL_TANK_F = 61,
		FUEL_TANK_LEFT_SIDE = 61,
		FUEL_TANK_B = 62,
		FUEL_TANK_RIGHT_SIDE = 62,
		ROTOR = 63,
		BLADE_1_IN = 64,
		BLADE_1_CENTER = 65,
		BLADE_1_OUT = 66,
		BLADE_2_IN = 67,
		BLADE_2_CENTER = 68,
		BLADE_2_OUT = 69,
		BLADE_3_IN = 70,
		BLADE_3_CENTER = 71,
		BLADE_3_OUT = 72,
		BLADE_4_IN = 73,
		BLADE_4_CENTER = 74,
		BLADE_4_OUT = 75,
		BLADE_5_IN = 76,
		BLADE_5_CENTER = 77,
		BLADE_5_OUT = 78,
		BLADE_6_IN = 79,
		BLADE_6_CENTER = 80,
		BLADE_6_OUT = 81,
		FUSELAGE_BOTTOM = 82,
		WHEEL_F = 83,
		WHEEL_C = 83,
		WHEEL_REAR = 83,
		WHEEL_L = 84,
		WHEEL_R = 85,
		PYLON1 = 86,
		PYLONL = 86,
		PYLON2 = 87,
		PYLONR = 87,
		PYLON3 = 88,
		PYLON4 = 89,
		CREW_1 = 90,
		CREW_2 = 91,
		CREW_3 = 92,
		CREW_4 = 93,
		ARMOR_NOSE_PLATE_LEFT = 94,
		ARMOR_NOSE_PLATE_RIGHT = 95,
		ARMOR_PLATE_LEFT = 96,
		ARMOR_PLATE_RIGHT = 97,
		HOOK = 98,
		FUSELAGE_TOP = 99,
		TAIL_TOP = 100,
		FLAP_L_CENTER = 101,
		FLAP_R_CENTER = 102,
		ENGINE_1 = 103,
		ENGINE_2 = 104,
		ENGINE_3 = 105,
		ENGINE_4 = 106,
		ENGINE_5 = 107,
		ENGINE_6 = 108,
		ENGINE_7 = 109,
		ENGINE_8 = 110,
		COUNT = 111,
	};

	struct DamageDelta
	{
		Damage m_element;
		float m_delta;
	};

	inline void setIntegrityElement(Damage element, float integrity);
	inline float getIntegrityElement(Damage element);
	inline void setDamageDelta(Damage element, float delta);
	inline bool processDamageStack(Damage& element, float& damage);
	void resetDamage();
	void printDamageState();
	
	inline float getLWingDamage() const;
	inline float getRWingDamage() const;
	
	inline float getAileronDamage() const;

	inline float getVertStabDamage() const;
	inline float getRudderDamage() const;

	inline float getHoriStabDamage() const;

	inline float getCompressorDamage() const;
	inline float getTurbineDamage() const;
	inline float getSpeedbrakeDamage() const;
	inline float getFlapDamage() const;

	double getEngineDamageMult();
	//void engineFlameOut();

	inline double getDamageElement(Damage element) const;

private:
	Vec3 m_moment;
	Vec3 m_force;
	State& m_state;
	Input& m_input;
	Engine& m_engine; //neu 21Feb21 // wieder rausgenommen
	std::vector<DamageDelta> m_damageStack;

	//Gear
	double m_gearLPosition = 0.0; //0 -> 1
	double m_gearRPosition = 0.0;
	double m_gearNPosition = 0.0;

	double m_gearLLamp = 0.0;
	double m_gearRLamp = 0.0;
	double m_gearFLamp = 0.0;

	//modification variable for Ground-Start
	double m_gearStart = 0.0;
	double m_gearStartDown = 0.0;

	//aerodynamic surfaces
	double m_flapsPosition = 0.0;
	double m_speedBrakePosition = 0.0;
	double m_hookPosition = 0.0;

	double m_aileronLeft = 0.0;
	double m_aileronRight = 0.0;
	double m_stabilizer = 0.0;
	double m_rudder = 0.0;

	double m_noseWheelAngle = 0.0;

	double m_nozzlePosition = 0.0;

	double m_int_throttlePos = 0.0;

	double m_engDmgMulti = 0.0;

	//--------Brk-Chute Stuff-------------------------
	double m_brakeMoment = 0.0;
	double m_chuteState = 0.0;
	double m_nwsEngage = 0.0;
	double m_chuteSlewY = 0.0;
	double m_chuteSlewZ = 0.0;
	int m_timePassed = 0;

	//---------AutoPilot Stuff Alt-Hold--------------
	double m_desiredAlt = 0.0;
	double m_previousAlt = 0.0;
	double m_autPilAltEng = 0.0;
	double m_altHold = 0.0;
	double m_pitchAPadj = 0.0;
	double m_ascHA = 0.0;
	double m_decHA = 0.0;
	bool m_decend = false;
	bool m_acend = false;
	bool m_level = false;
	bool m_acendHoldAngle = false;
	bool m_decendHoldAngle = false;
	//double m_speedPrevious = 0.0;

	//-----------Flaps and Gear Systems and Indicators------------
	double m_blcLift = 0.0;
	double m_flapsLevPos = 0.0;
	double m_flapsIndTEPos = 0.0;
	double m_flapsIndLEPos = 0.0;

	//-----------FuelFlow Indicator---------------------------------
	double m_fuelHundred = 0.0;
	double m_fuelThousand = 0.0;
	double m_fuelDivide = 0.0;

	//-----------Damage Indicator Variables--------------------------
	double m_ailDamInd = 0.0;
	double m_stabDamInd = 0.0;

	//---------------Actuators--------------------------------------

	Actuator m_actuatorStab; //scheint nur zur optischen "Versch�nerung" zu sein, aber egal
	Actuator m_actuatorAil;
	Actuator m_actuatorRud;
	Actuator m_actuatorFlap;
	Actuator m_actuatorGearL;
	Actuator m_actuatorGearR;
	Actuator m_actuatorGearN;
	Actuator m_actuatorAirbrk;
	Actuator m_actuatorHook;
	Actuator m_actuatorNozzle;
	Actuator m_actuatorNosewheel;


	//double m_stabilizerZeroForceDeflection = 0.0;

	double m_mass = 1.0;

	float* m_integrityElement;

};

double Airframe::setAileron(double dt)
{
	double input = m_input.getRoll(); // +m_input.m_rollTrim(); // m_rollTrim kommt noch
	return m_actuatorAil.inputUpdate(input, dt);
}

//Folgend Auskommentierungen zum Testen des Stabilizers, da Aileron und Rudder so funktionieren
double Airframe::setStabilizer(double dt)
{
	double input = m_input.getPitch();//m_input.getPitch(); // +m_stabilizerZeroForceDeflection;
	return m_actuatorStab.inputUpdate(input, dt);
	
}

double Airframe::setRudder(double dt)
{
	double input = m_input.getYaw(); // +m_controls.yawTrim(); Yaw-Trim kommt noch
	return m_actuatorRud.inputUpdate(input, dt);
}

double Airframe::setFlapsPosition(double dt)
{
	double input = m_input.getFlapsToggle();
	return m_actuatorFlap.inputUpdate(input, dt);
}


/*void Airframe::setFlapsPosition(double position) //ALT aber insges. funktionst�chtig
{
	m_flapsPosition = position;
}*/

double Airframe::setGearLPosition(double dt)
{
	double input = m_input.getGearToggle(); 
	return m_actuatorGearL.inputUpdate(input, dt);
}

double Airframe::setGearRPosition(double dt)
{
	double input = m_input.getGearToggle();
	return m_actuatorGearR.inputUpdate(input, dt);
}

double Airframe::setGearNPosition(double dt)
{	
	double input = m_input.getGearToggle();
	return m_actuatorGearN.inputUpdate(input, dt);
}

double Airframe::setAirbrakePosition(double dt)
{
	double input = m_input.getAirbrake();
	return m_actuatorAirbrk.inputUpdate(input, dt);
}

double Airframe::setHookPosition(double dt)
{
	double input = m_input.getHookToggle();
	return m_actuatorHook.inputUpdate(input, dt);
}

void Airframe::setMass(double mass)
{
	m_mass = mass;
}

double Airframe::setNoseWheelAngle(double dt)
{
	double input = m_input.getYaw();
	return m_actuatorNosewheel.inputUpdate(input, dt);
}

double Airframe::getNoseWheelAngle() const
{
	return -m_noseWheelAngle;
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

// Neu eingef�gt den Lampen-Kram zur directen Steuerung der FC-3 Cockpit-Args
double Airframe::getGearLLamp()
{
	if (getGearLPosition() == 1.0)
	{
		m_gearLLamp = 1.0;
	}
	else
	{
		m_gearLLamp = 0.0;
	}

	return m_gearLLamp;
}

double Airframe::getGearRLamp()
{
	if (getGearRPosition() == 1.0)
	{
		m_gearRLamp = 1.0;
	}
	else
	{
		m_gearRLamp = 0.0;
	}

	return m_gearRLamp;
}

double Airframe::getGearFLamp()
{
	if (getGearNPosition() == 1.0)
	{
		m_gearFLamp = 1.0;
	}
	else
	{
		m_gearFLamp = 0.0;
	}

	return m_gearFLamp;
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

double Airframe::getNozzlePosition() const
{
	return m_nozzlePosition;
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

double Airframe::noseWheelAngle()
{
	return m_noseWheelAngle > 0.0 ? CON_NSWL * m_noseWheelAngle : -CON_NSWR * m_noseWheelAngle;
}

double Airframe::getMass() const
{
	return m_mass;
}

//------------------Flaps Lever and Flaps-Indicators--------------------------
//--------------Flaps Lever--------------------------------------------------
double Airframe::getFlapLevPos()
{
	if (m_input.getFlapsToggle() == 0.0)
	{
		m_flapsLevPos = 0.0;
	}
	else if (m_input.getFlapsToggle() == 0.5)
	{
		m_flapsLevPos = 0.5;
	}

	else if (m_input.getFlapsToggle() == 1.0)
	{
		m_flapsLevPos = 1.0;
	}

	return m_flapsLevPos;
}

double Airframe::getFlapIndLEPos()
{
	if (getFlapsPosition() == 0.0)
	{
		m_flapsIndLEPos = 0.0;
	}
	else if (getFlapsPosition() == 0.5)
	{
		m_flapsIndLEPos = 0.5;
	}
	else if (getFlapsPosition() == 1.0)
	{
		m_flapsIndLEPos = 1.0;
	}
	else
	{
		m_flapsIndLEPos = 0.3;
	}
	return m_flapsIndLEPos;
}

double Airframe::getFlapIndTEPos()
{
	if (getFlapsPosition() == 0.0)
	{
		m_flapsIndTEPos = 0.0; //UP
	}
	else if (getFlapsPosition() == 0.5)
	{
		m_flapsIndTEPos = 0.5; //TakeOff
	}
	else if (getFlapsPosition() == 1.0)
	{
		m_flapsIndTEPos = 1.0; //Land
	}
	else
	{
		m_flapsIndTEPos = 0.3;
	}

	return m_flapsIndTEPos;
}

double Airframe::ailDamageIndicator()
{
	if (getAileronDamage() <= 0.5)
	{
		m_ailDamInd = 1.0;
	}
	else
	{
		m_ailDamInd = 0.0;
	}
	
	return m_ailDamInd;
}

double Airframe::stabDamageIndicator()
{
	if (getHoriStabDamage() <= 0.5)
	{
		m_stabDamInd = 1.0;
	}
	else
	{
		m_stabDamInd = 0.0;
	}

	return m_stabDamInd;
}

//------------AutoPilot-Stuff-----------------------------
double Airframe::getAutoPilotAltH()
{
	return m_pitchAPadj;
}

//----------Damage-Stuff-------------------------------------

double Airframe::getDamageElement(Damage element) const
{
	return DMG_ELEM(element);
}

inline void Airframe::setIntegrityElement(Damage element, float integrity)
{
	m_integrityElement[(int)element] = integrity;
}

inline float Airframe::getIntegrityElement(Damage element)
{
	return m_integrityElement[(int)element];
}

void Airframe::setDamageDelta(Damage element, float delta)
{
	DamageDelta d;
	d.m_delta = delta;
	d.m_element = element;
	m_damageStack.push_back(d);
}

bool Airframe::processDamageStack(Damage& element, float& damage)
{
	if (m_damageStack.empty())
		return false;

	DamageDelta delta = m_damageStack.back();
	m_damageStack.pop_back();


	m_integrityElement[(int)delta.m_element] -= delta.m_delta;
	m_integrityElement[(int)delta.m_element] = clamp(m_integrityElement[(int)delta.m_element], 0.0, 1.0);

	element = delta.m_element;
	damage = m_integrityElement[(int)delta.m_element];

	return true;
}

inline float Airframe::getLWingDamage() const
{
	return (DMG_ELEM(Damage::WING_L_IN) + DMG_ELEM(Damage::WING_L_CENTER) + DMG_ELEM(Damage::WING_L_OUT)) / 3.0;
}

inline float Airframe::getRWingDamage() const
{
	return (DMG_ELEM(Damage::WING_R_IN) + DMG_ELEM(Damage::WING_R_CENTER) + DMG_ELEM(Damage::WING_R_OUT)) / 3.0;
}

inline float Airframe::getAileronDamage() const
{
	return (DMG_ELEM(Damage::AILERON_L) + DMG_ELEM(Damage::AILERON_R)) / 2.0;
}

inline float Airframe::getVertStabDamage() const
{
	return (DMG_ELEM(Damage::FIN_L_TOP) + DMG_ELEM(Damage::FIN_L_BOTTOM)) / 2.0;
}

inline float Airframe::getRudderDamage() const
{
	return DMG_ELEM(Damage::RUDDER);
}

inline float Airframe::getHoriStabDamage() const
{
	return (DMG_ELEM(Damage::STABILIZATOR_L) + DMG_ELEM(Damage::STABILIZATOR_R)) / 2.0;
}

inline float Airframe::getCompressorDamage() const
{
	return 1.0 - clamp(2.0 - DMG_ELEM(Damage::MTG_L) - DMG_ELEM(Damage::MTG_R), 0.0, 1.0);
}
inline float Airframe::getTurbineDamage() const
{
	return 1.0 - clamp(2.0 - DMG_ELEM(Damage::ENGINE_L) - DMG_ELEM(Damage::ENGINE_R), 0.0, 1.0);
}

inline float Airframe::getSpeedbrakeDamage() const
{
	return (DMG_ELEM(Damage::AIR_BRAKE_L) + DMG_ELEM(Damage::AIR_BRAKE_R)) / 2.0;
}
inline float Airframe::getFlapDamage() const
{
	return (DMG_ELEM(Damage::FLAP_L) + DMG_ELEM(Damage::FLAP_R)) / 2.0;
}



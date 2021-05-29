#pragma once
#include "BaseComponent.h"

enum Command
{
	COMMAND_PITCH = 2001,
	COMMAND_ROLL = 2002,
	COMMAND_THROTTLE = 2004,
	COMMAND_YAW = 2003, //eingefügt 16.02. PJ
	COMMAND_TRIMM_UP = 95,
	COMMAND_TRIMM_DOWN = 96,
	COMMAND_TRIMM_AIL_L = 93,
	COMMAND_TRIMM_AIL_R = 94,
	COMMAND_FLAPS_TOGGLE = 72,
	COMMAND_GEAR_TOGGLE = 68, //68
	COMMAND_GEAR_UP = 430, //
	COMMAND_GEAR_DOWN = 431, //
	COMMAND_BRAKE = 74,//war 2111// Konsole gab 74 und 75 aus bei "W"
	COMMAND_RELEASE_BRAKE = 75, //NEU eingefügt und ein versuch wegen oben
	COMMAND_LEFT_BRAKE = 3162,
	COMMAND_RIGHT_BRAKE = 3163,
	//COMMAND_FLAPS_INCREASE = 10001,
	//COMMAND_FLAPS_DECREASE = 10002,
	COMMAND_FLAPS_DOWN = 145,
	COMMAND_FLAPS_UP = 146,
	COMMAND_AIRBRAKE = 73, // Aus Konsole abgelesen
	COMMAND_AIRBRAKE_EXTEND = 147,
	COMMAND_AIRBRAKE_RETRACT = 148,
	COMMAND_HOOK_TOGGLE = 69,
	COMMAND_NOSEWHEEL_STEERING = 606, //Neu aus Konsole abgelesen
	//COMMAND_NOSEWHEEL_STEERING_ENGAGE = 3133,
	//COMMMAND_NOSEWHEEL_STEERING_DISENGAGE = 3134,
	//COMMAND_NOSEWHEEL_RANGE = 562, //aus Konsole abgelesen
	//COMMAND_STARTER_BUTTON = 3013,
	//COMMAND_THROTTLE_DETEND = 3087,
	COMMAND_BRAKE_CHUTE = 76, //aus der Konsole abgelesen
	COMMAND_ENGINE_START = 309,
	COMMAND_ENGINE_STOP = 310,

						//--------------------------------------

};


class Input
{
	public:
		virtual void zeroInit()
		{
			m_pitch = 0.0;
			m_roll = 0.0;
			m_yaw = 0.0;
			m_throttle = 0.0;
			m_trimm_up = 0.0;
			m_trimm_down = 0.0;
			m_trimm_ail_l = 0.0;
			m_trimm_ail_r = 0.0;
			m_flaps_toggle = 0.0;
			m_gear_toggle = 0.0;
			m_gearup = 0.0;
			m_geardown = 0.0;
			m_brake = 0.0;
			m_release_brake = 0.0;
			m_brakeDuration = 0.0;
			m_leftbrake = 0.0;
			m_rightbrake = 0.0;
			m_flapsinc = 0.0;
			m_flapsdec = 0.0;
			m_flapsdown = 0.0;
			m_flapsup = 0.0;
			m_airbrk = 0.0;
			m_airbrkext = 0.0;
			m_airbrkret = 0.0;
			m_hooktgl = 0.0;
			m_nwsteering = 0.0;
		}

		
		virtual void coldInit()
		{
			zeroInit();
		}

		virtual void hotInit()
		{
			zeroInit();
		}

		virtual void airborneInit()
		{
			zeroInit();
		}

	double m_pitch = 0.0;
	double m_roll = 0.0;
	double m_yaw = 0.0;
	double m_throttle = 0.0;
	double m_trimm_up = 0.0;
	double m_trimm_down = 0.0;
	double m_trimm_ail_l = 0.0;
	double m_trimm_ail_r = 0.0;
	double m_flaps_toggle = 0.0;
	double m_gear_toggle = 0.0;
	double m_gearup = 0.0;
	double m_geardown = 0.0;
	double m_brake = 0.0;
	double m_release_brake = 0.0;
	double m_brakeDuration = 0.0;
	double m_leftbrake = 0.0;
	double m_rightbrake = 0.0;
	double m_flapsinc = 0.0;
	double m_flapsdec = 0.0;
	double m_flapsdown = 0.0;
	double m_flapsup = 0.0;
	double m_airbrk = 0.0;
	double m_airbrkext = 0.0;
	double m_airbrkret = 0.0;
	double m_hooktgl = 0.0;
	double m_nwsteering = 0.0;
	//double m_nwsteeringeng;
	//double m_nwsteeringdiseng;
	//double m_starterbutton;
	//double m_trhottledet;
	double m_brkchute = 0.0;
	double m_engine_start = 0.0;
	double m_engine_stop = 0.0;
};
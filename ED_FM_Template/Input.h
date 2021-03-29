#pragma once

enum Command
{
	COMMAND_PITCH = 2001,
	COMMAND_ROLL = 2002,
	COMMAND_THROTTLE = 2004,
	COMMAND_YAW = 2003, //eingefügt 16.02. PJ
	COMMAND_GEAR_TOGGLE = 68, //68
	COMMAND_GEAR_UP = 430, //
	COMMAND_GEAR_DOWN = 431, //
	COMAND_BRAKE = 2111,
	COMMAND_LEFT_BRAKE = 3162,
	COMMAND_RIGHT_BRAKE = 3163,
	COMMAND_FLAPS_INCREASE = 10001,
	COMMAND_FLAPS_DECREASE = 10002,
	COMMAND_FLAPS_DOWN = 145,
	COMMAND_FLAPS_UP = 146,
	COMMAND_FLAPS_TOGGLE = 72,
	COMMAND_AIRBRAKE_EXTEND = 147,
	COMMAND_AIRBRAKE_RETRACT = 148,
	COMMAND_HOOK_TOGGLE = 69,
	COMMAND_NOSEWHEEL_STEERING_ENGAGE = 3133,
	COMMMAND_NOSEWHEEL_STEERING_DISENGAGE = 3134,
	COMMAND_STARTER_BUTTON = 3013,
	COMMAND_THROTTLE_DETEND = 3087,
						//--------------------------------------

};



struct Input
{
	double m_pitch;
	double m_roll;
	double m_yaw;
	double m_throttle;
	double m_geartoggle;
	double m_gearup;
	double m_geardown;
	double m_brake;
	double m_leftbrake;
	double m_rightbrake;
	double m_flapsinc;
	double m_flapsdec;
	double m_flapsdown;
	double m_flapsup;
	double m_flapstgl;
	double m_airbrkext;
	double m_airbrkret;
	double m_hooktgl;
	double m_nswsteeringeng;
	double m_nswsteeringdiseng;
	double m_starterbutton;
	double m_trhottledet;
};
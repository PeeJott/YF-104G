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
	COMMAND_AUTOPILOT_ENG = 389,
	COMMAND_LIGHT_TOGGLE = 328,
	
	COMMAND_ELEV_UP_GO = 195,
	COMMAND_ELEV_UP_STOP = 196,
	COMMAND_ELEV_DOWN_GO = 193,
	COMMAND_ELEV_DOWN_STOP = 194,
	COMMAND_RUD_RIGHT_GO = 203,
	COMMAND_RUD_RIGHT_STOP = 204,
	COMMAND_RUD_LEFT_GO = 201,
	COMMAND_RUD_LEFT_STOP = 202,
	COMMAND_AIL_RIGHT_GO = 199,
	COMMAND_AIL_RIGHT_STOP = 200,
	COMMAND_AIL_LEFT_GO = 197,
	COMMAND_AIL_LEFT_STOP = 198,


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
			m_throttle = -1.0;
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
			m_autoPilotEng = 0.0;
			m_light_toggle = 0.0;

			m_elev_up_go = 0.0;
			m_elev_up_stop = 0.0;
			m_elev_down_go = 0.0;
			m_elev_down_stop = 0.0;
			m_rudder_right_go = 0.0;
			m_rudder_right_stop = 0.0;
			m_rudder_left_go = 0.0;
			m_rudder_left_stop = 0.0;
			m_ail_right_go = 0.0;
			m_ail_right_stop = 0.0;
			m_ail_left_go = 0.0;
			m_ail_left_stop = 0.0;

		}

		
		virtual void coldInit()
		{
			zeroInit();
			m_gear_toggle = 1.0;
		}

		virtual void hotInit()
		{
			zeroInit();
			m_engine_start = 1.0;
			m_gear_toggle = 1.0;
		}

		virtual void airborneInit()
		{
			zeroInit();
			m_engine_start = 1.0;
		}

		inline const double pitch(double value)
		{
			m_pitch = value;
			return m_pitch;
		}

		inline const double roll(double value)
		{
			m_roll = value;
			return m_roll;
		}

		inline const double throttle(double value)
		{
			m_throttle = value;
			return m_throttle;
		}

		inline const double yaw(double value)
		{
			m_yaw = value;
			return m_yaw;
		}

		inline const double trimmUP()
		{
			if (m_trimm_up == 0.0)
			{
				m_trimm_up = 0.0002;
			}
			else if (m_trimm_up != 0.0)
			{
				m_trimm_up += 0.0002;
			}

			return m_trimm_up;
		}

		inline const double trimmDOWN()
		{
			if (m_trimm_down == 0.0)
			{
				m_trimm_down = 0.0002;
			}
			else if (m_trimm_down != 0.0)
			{
				m_trimm_down += 0.0002;
			}
			return m_trimm_down;
		}

		inline const double trimmAilL()
		{
			if (m_trimm_ail_l == 0.0)
			{
				m_trimm_ail_l = 0.0002;
			}
			else if (m_trimm_ail_l != 0.0)
			{
				m_trimm_ail_l += 0.0002;
			}

			return m_trimm_ail_l;
		}

		inline const double trimmAilR()
		{
			if (m_trimm_ail_r == 0.0)
			{
				m_trimm_ail_r = 0.0002;
			}
			else if (m_trimm_ail_r != 0.0)
			{
				m_trimm_ail_r += 0.0002;
			}
			return m_trimm_ail_r;
		}

		inline const double gearToggle()
		{
			if (m_gear_toggle == 0.0)
			{
				m_gear_toggle = 1.0;
			}
			else
			{
				m_gear_toggle = 0.0;
			}
			return m_gear_toggle;
		}

		inline const double gearUP()
		{
			if (m_gearup == 0.0)
			{
				m_gear_toggle = 0.0;
			}
			else
			{
				m_gearup = 0.0;
			}
			return m_gearup;
		}

		inline const double gearDOWN()
		{
			if (m_geardown == 0.0)
			{
				m_gear_toggle = 1.0;
			}
			else
			{
				m_geardown = 0.0;
			}

			return m_geardown;
		}

		inline const double brake()
		{
			if (m_brake == 0.0)
			{
				m_brake = 1.0;
			}
			else
			{
				m_brake = 0.0;
			}
			return m_brake;
		}

		inline const double releaseBrake()
		{
			if (m_release_brake == 0.0)
			{
				m_release_brake = 1.0;
			}
			else
			{
				m_release_brake = 0.0;
			}
			return m_release_brake;
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
	double m_autoPilotEng = 0.0;
	double m_light_toggle = 0.0;
	
	double m_elev_up_go = 0.0;
	double m_elev_up_stop = 0.0;
	double m_elev_down_go = 0.0;
	double m_elev_down_stop = 0.0;
	double m_rudder_right_go = 0.0;
	double m_rudder_right_stop = 0.0;
	double m_rudder_left_go = 0.0;
	double m_rudder_left_stop = 0.0;
	double m_ail_right_go = 0.0;
	double m_ail_right_stop = 0.0;
	double m_ail_left_go = 0.0;
	double m_ail_left_stop = 0.0;
};
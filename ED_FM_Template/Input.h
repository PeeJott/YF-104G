﻿#pragma once
#include "BaseComponent.h"

//=========================================================================//
//
//		FILE NAME	: Input.h
//		AUTHOR		: Joshua Nelson & Paul Stich
//		Copyright	: Joshua Nelson & Paul Stich
//		DATE		: August 2021
//
//		DESCRIPTION	: Class for Input-handling.  
//					  
//					  
//================================ Includes ===============================//
//=========================================================================//

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
	COMMAND_ELECTRIC_SYSTEM = 315,

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

	COMMAND_CROSSHAIR_UP = 302,
	COMMAND_CROSSHAIR_DOWN = 303,
	COMMAND_CROSSHAIR_LEFT = 562,
	COMMAND_CROSSHAIR_RIGHT = 301,

	COMMAND_HUD_DARK = 747,

	COMMAND_SIGHT_HORIZONTAL = 2142,//eigentlich 2142
	COMMAND_SIGHT_VERTICAL = 2143,//eigentlich 2143


	//--------------------------------------

};


class Input
{
	
	public:
		
		Input(); //NEU eingefügt, wenn man Input.cpp baut, sonst raus!

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
			m_electricSystem = 0.0;

			m_crossHUp = 0.0;
			m_crossHDown = 0.0;
			m_crossHLeft = 0.0;
			m_crossHRight = 0.0;

			m_hudDark = 50.0;

			m_sightVertical = 0.0;
			m_sightHorizontal = 0.0;

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

			m_elevUP = 0.0;
			m_elevDOWN = 0.0;
			m_rudRIGHT = 0.0;
			m_rudLEFT = 0.0;
			m_ailRIGHT = 0.0;
			m_ailLEFT = 0.0;

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
			m_electricSystem = 1.0;
		}

		virtual void airborneInit()
		{
			zeroInit();
			m_engine_start = 1.0;
			m_electricSystem = 1.0;
		}

		inline const void pitch(double value) //von "inline const double" zu "inline const void
		{
			m_pitch = value;
			//return m_pitch;
		}

		inline const double getPitch() //ob das geht werden wir mal testen...wenn nicht, checken wo ich überall m_pitch zu getPitch verändert habe
		{
			return m_pitch;
		}

		inline const void roll(double value)
		{
			m_roll = value;
			//return m_roll;
		}

		inline const double getRoll()
		{
			return m_roll;
		}

		inline const void throttle(double value)
		{
			m_throttle = value;
			//return m_throttle;
		}

		inline const double getThrottle()
		{
			return m_throttle;
		}

		inline const void yaw(double value)
		{
			m_yaw = value;
			//return m_yaw;
		}

		inline const double getYaw()
		{
			return m_yaw;
		}

		inline const void trimmUP()
		{
			if (m_trimm_up == 0.0)
			{
				m_trimm_up = 0.0002;
			}
			else if (m_trimm_up != 0.0)
			{
				m_trimm_up += 0.0002;
			}

			//return m_trimm_up;
		}

		inline const double getTrimmUp()
		{
			return m_trimm_up;
		}

		inline const void trimmDOWN()
		{
			if (m_trimm_down == 0.0)
			{
				m_trimm_down = 0.0002;
			}
			else if (m_trimm_down != 0.0)
			{
				m_trimm_down += 0.0002;
			}
			//return m_trimm_down;
		}

		inline const double getTrimmDown()
		{
			return m_trimm_down;
		}

		inline const void trimmAilL()
		{
			if (m_trimm_ail_l == 0.0)
			{
				m_trimm_ail_l = 0.0002;
			}
			else if (m_trimm_ail_l != 0.0)
			{
				m_trimm_ail_l += 0.0002;
			}

			//return m_trimm_ail_l;
		}

		inline const double getTrimmAilL()
		{
			return m_trimm_ail_l;
		}


		inline const void trimmAilR()
		{
			if (m_trimm_ail_r == 0.0)
			{
				m_trimm_ail_r = 0.0002;
			}
			else if (m_trimm_ail_r != 0.0)
			{
				m_trimm_ail_r += 0.0002;
			}
			//return m_trimm_ail_r;
		}

		inline const double getTrimmAilR()
		{
			return m_trimm_ail_r;
		}

		inline const void gearToggle()
		{
			if (m_gear_toggle == 0.0)
			{
				m_gear_toggle = 1.0;
			}
			else
			{
				m_gear_toggle = 0.0;
			}
			//return m_gear_toggle;
		}

		inline const double getGearToggle()
		{
			return m_gear_toggle;
		}

		inline const void gearUP()
		{
			
			if (m_gearup == 0.0)
			{
				m_gear_toggle = 0.0;
			}
			else
			{
				m_gearup = 0.0;
			}
			//return m_gearup;
		}

		inline const void gearDOWN()
		{
			if (m_geardown == 0.0)
			{
				m_gear_toggle = 1.0;
			}
			else
			{
				m_geardown = 0.0;
			}

			//return m_geardown;
		}

		inline const void brake()
		{
			if (m_brake == 0.0)
			{
				m_brake = 1.0;
			}
			else
			{
				m_brake = 0.0;
			}
			//return m_brake;
		}

		inline const double getBrake()
		{
			return m_brake;
		}

		inline const void releaseBrake()
		{
			if (m_release_brake == 0.0)
			{
				m_release_brake = 1.0;
			}
			else
			{
				m_release_brake = 0.0;
			}
			//return m_release_brake;
		}

		inline const double getReleaseBrake()
		{
			return m_release_brake;
		}

		inline const void flapsDown()
		{
			if (m_flapsdown == 0.0)
			{
				m_flaps_toggle = 1.0;
			}
			else
			{
				m_flapsdown = 0.0;
			}
			//return m_flapsdown;
		}

		inline const void flapsToggle()
		{
			if (m_flaps_toggle == 0.0)
			{
				m_flaps_toggle = 0.5;
			}
			else if (m_flaps_toggle == 0.5)
			{
				m_flaps_toggle = 1.0;
			}
			else
			{
				m_flaps_toggle = 0.0;
			}

			//return m_flaps_toggle;
		}
		
		inline const double getFlapsToggle()
		{
			return m_flaps_toggle;
		}

		inline const void flapsUp()
		{
			if (m_flapsup == 0.0)
			{
				m_flaps_toggle = 0.0;
			}
			else
			{
				m_flapsup = 0.0;
			}
			//return m_flapsup;
		}

		inline const void airbrake()
		{
			if (m_airbrk == 0.0)
			{
				m_airbrk = 1.0;
			}
			else
			{
				m_airbrk = 0.0;
			}
			//return m_airbrk;
		}

		inline const double getAirbrake()
		{
			return m_airbrk;
		}

		inline const void airbrakeExt()
		{
			if (m_airbrkext == 0.0)
			{
				m_airbrk = 1.0;
			}
			else
			{
				m_airbrkext = 0.0;
			}
			//return m_airbrkext;
		}

		inline const void airbrakeRet()
		{
			if (m_airbrkret == 0.0)
			{
				m_airbrk = 0.0;
			}
			else
			{
				m_airbrkret = 0.0;
			}
			//return m_airbrkret;
		}

		inline const void hookToggle()
		{
			if (m_hooktgl == 0.0)
			{
				m_hooktgl = 1.0;
			}
			else
			{
				m_hooktgl = 0.0;
			}
			//return m_hooktgl;
		}

		inline const double getHookToggle()
		{
			return m_hooktgl;
		}

		inline const void nwSteering()
		{
			if (m_nwsteering == 0.0)
			{
				m_nwsteering = 1.0;
			}
			else
			{
				m_nwsteering = 0.0;
			}
			//return m_nwsteering;
		}

		inline const double getNWS()
		{
			return m_nwsteering;
		}

		inline const void brakeChute()
		{
			if (m_brkchute == 0.0)
			{
				m_brkchute = 1.0;
			}
			else
			{
				m_brkchute = 0.0;
			}
			//return m_brkchute;
		}

		inline const double getBrkChute()
		{
			return m_brkchute;
		}

		inline const void engineStart()
		{
			if (m_engine_start == 0.0)
			{
				m_engine_stop = 0.0;
				m_engine_start = 1.0;
			}
			else
			{
				m_engine_start = 0.0;
			}
			//return m_engine_start;
		}

		inline const double getEngineStart()
		{
			return m_engine_start;
		}

		inline const void engineStop()
		{
			if (m_engine_stop == 0.0)
			{
				m_engine_start = 0.0;
				m_engine_stop = 1.0;
			}
			
			//return m_engine_stop;
		}

		inline const double getEngineStop()
		{
			return m_engine_stop;
		}

		inline const void autoPilotEng()
		{
			if (m_autoPilotEng == 0.0)
			{
				m_autoPilotEng = 1.0;
			}
			else
			{
				m_autoPilotEng = 0.0;
			}

			//return m_autoPilotEng;
		}

		inline const double getAutoPEng()
		{
			return m_autoPilotEng;
		}

		inline const void lightToggle()
		{
			if (m_light_toggle == 0.0)
			{
				m_light_toggle = 0.5;
			}
			else if (m_light_toggle == 0.5)
			{
				m_light_toggle = 1.0;
			}
			else
			{
				m_light_toggle = 0.0;
			}

			//return m_light_toggle;
		}

		inline const double getLightToggle()
		{
			return m_light_toggle;
		}

		inline const void electricSystem()
		{
			if (m_electricSystem == 0.0)
			{
				m_electricSystem = 1.0;
			}
			else
			{
				m_electricSystem = 0.0;
			}
		}

		inline const double getElectricSystem()
		{
			return m_electricSystem;
		}

//----------Crosshair-Test-Functions-------------

		inline const void crossHRight()
		{
			if (m_crossHRight == 0.0)
			{
				m_crossHRight = 0.05;
			}
			else if (m_crossHRight >= 0.05)
			{
				m_crossHRight += 0.05;
			}
		}

		void resetCrossHairs();

		inline const double getCrossHRight()
		{
			return m_crossHRight;
		}


		inline const void crossHLeft()
		{
			if (m_crossHLeft == 0.0)
			{
				m_crossHLeft = -0.05;
			}
			else if (m_crossHLeft <= -0.05)
			{
				m_crossHLeft -= 0.05;
			}
		}

		inline const double getCrossHLeft()
		{
			return m_crossHLeft;
		}

		inline const void crossHUp()
		{
			if (m_crossHUp == 0.0)
			{
				m_crossHUp = +0.05;
			}
			else if (m_crossHUp >= 0.05)
			{
				m_crossHUp += 0.05;
			}
		}

		inline const double getCrossHUp()
		{
			return m_crossHUp;
		}

		inline const void crossHDown()
		{
			if (m_crossHDown == 0.0)
			{
				m_crossHDown = -0.05;
			}
			else if (m_crossHDown <= -0.05)
			{
				m_crossHDown -= 0.05;
			}
		}

		inline const double getCrossHDown()
		{
			return m_crossHDown;
		}

		inline const void hudDark()
		{
			if (m_hudDark == 0.0)
			{
				m_hudDark = 50.0;
			}
		}

		inline const double getHudDark()
		{
			return m_hudDark;
		}

		inline const void sightHorizontal(double value)//eigentlich (double value)
		{
			m_sightHorizontal = value;
		}

		inline const double getSightHorizontal()
		{
			return -(m_sightHorizontal);
		}

		inline const void sightVertical(double value)
		{
			m_sightVertical = value;
		}

		inline const double getSightVertical()
		{
			return m_sightVertical;
		}

//---------Key Command Functions-----------------
		inline const double elevUpGO()
		{
			if (m_elev_up_go == 0.0)
			{
				m_elev_up_go = 1.0;
			}
			else
			{
				m_elev_up_go = 0.0;
			}
			return m_elev_up_go;
		}

		inline const double elevUpStop()
		{
			if (m_elev_up_stop == 0.0)
			{
				m_elev_up_stop = 1.0;
			}
			else
			{
				m_elev_up_stop = 0.0;
			}
			return m_elev_up_stop;
		}

		inline const double elevDownGO()
		{
			if (m_elev_down_go == 0.0)
			{
				m_elev_down_go = 1.0;
			}
			else
			{
				m_elev_down_go = 0.0;
			}
			return m_elev_down_go;
		}

		inline const double elevDownStop()
		{
			if (m_elev_down_stop == 0.0)
			{
				m_elev_down_stop = 1.0;
			}
			else
			{
				m_elev_down_stop = 0.0;
			}
			return m_elev_down_stop;
		}

		inline const double rudLeftGO()
		{
			if (m_rudder_left_go == 0.0)
			{
				m_rudder_left_go = 1.0;
			}
			else
			{
				m_rudder_left_go = 0.0;
			}
			return m_rudder_left_go;
		}

		inline const double rudLeftStop()
		{
			if (m_rudder_left_stop == 0.0)
			{
				m_rudder_left_stop = 1.0;
			}
			else
			{
				m_rudder_left_stop = 0.0;
			}
			return m_rudder_left_stop;
		}
		
		inline const double rudRightGO()
		{
			if (m_rudder_right_go == 0.0)
			{
				m_rudder_right_go = 1.0;
			}
			else
			{
				m_rudder_right_go = 0.0;
			}
			return m_rudder_right_go;
		}

		inline const double rudRightStop()
		{
			if (m_rudder_right_stop == 0.0)
			{
				m_rudder_right_stop = 1.0;
			}
			else
			{
				m_rudder_right_stop = 0.0;
			}
			return m_rudder_right_stop;
		}

		inline const double ailRightGO()
		{
			if (m_ail_right_go == 0.0)
			{
				m_ail_right_go = 1.0;
			}
			else
			{
				m_ail_right_go = 0.0;
			}
			return m_ail_right_go;
		}

		inline const double ailRightStop()
		{
			if (m_ail_right_stop == 0.0)
			{
				m_ail_right_stop = 1.0;
			}
			else
			{
				m_ail_right_stop = 0.0;
			}
			return m_ail_right_stop;
		}

		inline const double ailLeftGO()
		{
			if (m_ail_left_go == 0.0)
			{
				m_ail_left_go = 1.0;
			}
			else
			{
				m_ail_left_go = 0.0;
			}
			return m_ail_left_go;
		}

		inline const double ailLeftStop()
		{
			if (m_ail_left_stop == 0.0)
			{
				m_ail_left_stop = 1.0;
			}
			else
			{
				m_ail_left_stop = 0.0;
			}
			return m_ail_left_stop;
		}


		//----------Input Update-------------------------
		void inputUpdate(double dt);
		
		//--------Setter-Functions for Key-Commands-------
		void setKeyPitch();
		void setKeyRoll();
		void setKeyYaw();

		//---------Setter-Funktion for Brake/Release Brake-------
		void setBrake();

		/*//--------Getter-Functions for Key-Commands-------
		inline double getElevUpGO();
		inline double getElevUpStop();
		inline double getElevDownGO();
		inline double getElevDownStop();
		
		inline double getRudRightGO();
		inline double getRudRightStop();
		inline double getRudLeftGO();
		inline double getRudLeftStop();

		inline double getAilRightGO();
		inline double getAilRightStop();
		inline double getAilLeftGO();
		inline double getAilLeftStop();*/


		
	//die auskommentierten Variablen sind jetzt alle Private	
	//double m_pitch = 0.0;
	//double m_roll = 0.0;
	//double m_yaw = 0.0;
	//double m_throttle = 0.0;
	//double m_trimm_up = 0.0;
	//double m_trimm_down = 0.0;
	//double m_trimm_ail_l = 0.0;
	//double m_trimm_ail_r = 0.0;
	//double m_flaps_toggle = 0.0;
	//double m_gear_toggle = 0.0;
	//double m_gearup = 0.0;
	//double m_geardown = 0.0;
	//double m_brake = 0.0;
	//double m_release_brake = 0.0;
	double m_brakeDuration = 0.0;
	double m_leftbrake = 0.0;
	double m_rightbrake = 0.0;
	double m_flapsinc = 0.0;
	double m_flapsdec = 0.0;
	//double m_flapsdown = 0.0;
	//double m_flapsup = 0.0;
	//double m_airbrk = 0.0;
	//double m_airbrkext = 0.0;
	//double m_airbrkret = 0.0;
	//double m_hooktgl = 0.0;
	//double m_nwsteering = 0.0;
	//double m_nwsteeringeng;
	//double m_nwsteeringdiseng;
	//double m_starterbutton;
	//double m_trhottledet;
	//double m_brkchute = 0.0;
	//double m_engine_start = 0.0;
	//double m_engine_stop = 0.0;
	//double m_autoPilotEng = 0.0;
	//double m_light_toggle = 0.0;
	
	//double m_elev_up_go = 0.0;
	//double m_elev_up_stop = 0.0;
	//double m_elev_down_go = 0.0;
	//double m_elev_down_stop = 0.0;
	
	//double m_rudder_right_go = 0.0;
	//double m_rudder_right_stop = 0.0;
	//double m_rudder_left_go = 0.0;
	//double m_rudder_left_stop = 0.0;
	
	//double m_ail_right_go = 0.0;
	//double m_ail_right_stop = 0.0;
	//double m_ail_left_go = 0.0;
	//double m_ail_left_stop = 0.0;

	private:
		
	double m_pitch = 0.0;
	double m_roll = 0.0;
	double m_yaw = 0.0;
	double m_throttle = 0.0;

	double m_trimm_up = 0.0;
	double m_trimm_down = 0.0;
	double m_trimm_ail_l = 0.0;
	double m_trimm_ail_r = 0.0;

	double m_flaps_toggle = 0.0;
	double m_flapsdown = 0.0;
	double m_flapsup = 0.0;
	
	double m_gear_toggle = 0.0;
	double m_gearup = 0.0;
	double m_geardown = 0.0;
	
	double m_brake = 0.0;
	double m_release_brake = 0.0;

	double m_airbrk = 0.0;
	double m_airbrkext = 0.0;
	double m_airbrkret = 0.0;

	double m_hooktgl = 0.0;

	double m_brkchute = 0.0;

	double m_nwsteering = 0.0;

	double m_autoPilotEng = 0.0;

	double m_light_toggle = 0.0;

	double m_engine_start = 0.0;
	double m_engine_stop = 0.0;

	double m_electricSystem = 0.0;

	double m_crossHUp = 0.0;
	double m_crossHDown = 0.0;
	double m_crossHLeft = 0.0;
	double m_crossHRight = 0.0;
	double m_hudDark = 0.0;

	double m_sightVertical = 0.0;
	double m_sightHorizontal = 0.0;
	
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

	double m_elevUP = 0.0;
	double m_elevDOWN = 0.0;
	double m_rudRIGHT = 0.0;
	double m_rudLEFT = 0.0;
	double m_ailRIGHT = 0.0;
	double m_ailLEFT = 0.0;

};


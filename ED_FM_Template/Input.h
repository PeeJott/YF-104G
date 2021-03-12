#pragma once

enum Command
{
	COMMAND_PITCH = 2001,
	COMMAND_ROLL = 2002,
	COMMAND_THROTTLE = 2004,
	COMMAND_YAW = 2003, //eingefügt 16.02. PJ
	//--------------------------------------

};



struct Input
{
	double m_pitch;
	double m_roll;
	double m_yaw;
	double m_throttle;
};
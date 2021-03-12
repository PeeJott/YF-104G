// ED_FM_Template.cpp : Defines the exported functions for the DLL application.
#include "Vec3.h"
#include "stdafx.h"
#include "ED_FM_Template.h"
#include "ED_FM_Utility.h"
#include "FlightModel.h"
#include "State.h"
#include "Engine.h"
#include <Math.h>
#include <stdio.h>
#include <string>
#include "Input.h"


//Vec3	common_moment;
//Vec3	common_force;
//Vec3    center_of_gravity;
//Vec3	wind;
//Vec3	velocity_world_cs;
//double  throttle		  = 0;
//double  stick_roll		  = 0;
//double  stick_pitch		  = 0;

//double  internal_fuel     = 0;
//double  fuel_consumption_since_last_time  = 0;
//double  atmosphere_density = 0;
//double  aoa = 0;
//double  speed_of_sound = 320;

//============================= Statics ===================================//
static Input s_input;
static State s_state;
static Engine s_engine(s_input, s_state); //NEU (s_input, s_state)// !!WICHTIG!! überall muss die Reihenfolge Input/State/Engine/Flightmodel sein, NICHT andersrum
static FlightModel s_flightModel(s_state, s_input, s_engine); 

//=========================================================================//

void ed_fm_add_local_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{
	x = s_flightModel.getForce().x;
	y = s_flightModel.getForce().y;
	z = s_flightModel.getForce().z;

	pos_x = s_state.m_com.x;
	pos_y = s_state.m_com.y;
	pos_z = s_state.m_com.z;
}

void ed_fm_add_global_force(double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z)
{
	
}

void ed_fm_add_global_moment(double & x,double &y,double &z)
{

}

void ed_fm_add_local_moment(double& x,double& y,double& z)
{
	x = s_flightModel.getMoment().x;
	y = s_flightModel.getMoment().y;
	z = s_flightModel.getMoment().z;
}

void ed_fm_simulate(double dt)
{
	s_flightModel.update(dt);
	s_engine.update(dt); //neu eingefügt, damit die Engine-Class auch "geupdated" wird, da sie ungleich der Flightmodel-Class ist
}

void ed_fm_set_atmosphere(double h,//altitude above sea level
							double t,//current atmosphere temperature , Kelwins
							double a,//speed of sound
							double ro,// atmosphere density
							double p,// atmosphere pressure
							double wind_vx,//components of velocity vector, including turbulence in world coordinate system
							double wind_vy,//components of velocity vector, including turbulence in world coordinate system
							double wind_vz //components of velocity vector, including turbulence in world coordinate system
						)
{
	//void State::setCurrentAtmosphere( double temperature, double speedOfSound, double density, double pressure, const Vec3& wind )

	s_state.setCurrentAtmosphere(t, a, ro, p, Vec3(wind_vx, wind_vy, wind_vz));
}
/*
called before simulation to set up your environment for the next step
*/
void ed_fm_set_current_mass_state (double mass,
									double center_of_mass_x,
									double center_of_mass_y,
									double center_of_mass_z,
									double moment_of_inertia_x,
									double moment_of_inertia_y,
									double moment_of_inertia_z
									)
{
	s_state.setCOM(Vec3(center_of_mass_x, center_of_mass_y, center_of_mass_z));
}
/*
called before simulation to set up your environment for the next step
*/
void ed_fm_set_current_state (double ax,//linear acceleration component in world coordinate system
							double ay,//linear acceleration component in world coordinate system
							double az,//linear acceleration component in world coordinate system
							double vx,//linear velocity component in world coordinate system
							double vy,//linear velocity component in world coordinate system
							double vz,//linear velocity component in world coordinate system
							double px,//center of the body position in world coordinate system
							double py,//center of the body position in world coordinate system
							double pz,//center of the body position in world coordinate system
							double omegadotx,//angular accelearation components in world coordinate system
							double omegadoty,//angular accelearation components in world coordinate system
							double omegadotz,//angular accelearation components in world coordinate system
							double omegax,//angular velocity components in world coordinate system
							double omegay,//angular velocity components in world coordinate system
							double omegaz,//angular velocity components in world coordinate system
							double quaternion_x,//orientation quaternion components in world coordinate system
							double quaternion_y,//orientation quaternion components in world coordinate system
							double quaternion_z,//orientation quaternion components in world coordinate system
							double quaternion_w //orientation quaternion components in world coordinate system
							)
{
	s_state.setCurrentStateWorldAxis(
		Vec3(px, py, pz),
		Vec3(vx, vy, vz));
}


void ed_fm_set_current_state_body_axis(
	double ax,//linear acceleration component in body coordinate system
	double ay,//linear acceleration component in body coordinate system
	double az,//linear acceleration component in body coordinate system
	double vx,//linear velocity component in body coordinate system
	double vy,//linear velocity component in body coordinate system
	double vz,//linear velocity component in body coordinate system
	double wind_vx,//wind linear velocity component in body coordinate system
	double wind_vy,//wind linear velocity component in body coordinate system
	double wind_vz,//wind linear velocity component in body coordinate system

	double omegadotx,//angular accelearation components in body coordinate system
	double omegadoty,//angular accelearation components in body coordinate system
	double omegadotz,//angular accelearation components in body coordinate system
	double omegax,//angular velocity components in body coordinate system
	double omegay,//angular velocity components in body coordinate system
	double omegaz,//angular velocity components in body coordinate system
	double yaw,  //radians
	double pitch,//radians
	double roll, //radians
	double common_angle_of_attack, //AoA radians
	double common_angle_of_slide   //AoS radians
	)
{
	//void State::setCurrentStateBodyAxis(double aoa, double beta, const Vec3& angle, const Vec3& omega, const Vec3& omegaDot, const Vec3& speed, const Vec3& airspeed, const Vec3& acceleration)

	Vec3 velocity(vx, vy, vz);
	Vec3 windVelocity(wind_vx, wind_vy, wind_vz);

	s_state.setCurrentStateBodyAxis(
		common_angle_of_attack,
		common_angle_of_slide,
		Vec3(roll, yaw, pitch),
		Vec3(omegax, omegay, omegaz),
		Vec3(omegadotx, omegadoty, omegadotz),
		velocity,
		velocity - windVelocity,
		Vec3(ax, ay, az)
	);
}
/*
input handling
*/
void ed_fm_set_command(int command,
	float value)
{
	switch (command)
	{
	case COMMAND_PITCH:
		s_input.m_pitch = value;
		break;
	case COMMAND_ROLL:
		s_input.m_roll = value;
		break;
	case COMMAND_THROTTLE:
		s_input.m_throttle = value; // 0.65 * (-value + 1.0); auskommentiert und durch "value" ersetzt zur Angleichung an A4 Scooter.cpp
		break;
	case COMMAND_YAW:
		s_input.m_yaw = value;
		break;
	default:
		printf("number %d: %l f\n", command, value); //neu eingefügt um "unbekannte" Kommandos zur Konsole auszugeben
	}
}
/*
	Mass handling 

	will be called  after ed_fm_simulate :
	you should collect mass changes in ed_fm_simulate 

	double delta_mass = 0;
	double x = 0;
	double y = 0; 
	double z = 0;
	double piece_of_mass_MOI_x = 0;
	double piece_of_mass_MOI_y = 0; 
	double piece_of_mass_MOI_z = 0;
 
	//
	while (ed_fm_change_mass(delta_mass,x,y,z,piece_of_mass_MOI_x,piece_of_mass_MOI_y,piece_of_mass_MOI_z))
	{
	//internal DCS calculations for changing mass, center of gravity,  and moments of inertia
	}
*/
bool ed_fm_change_mass  (double & delta_mass,
						double & delta_mass_pos_x,
						double & delta_mass_pos_y,
						double & delta_mass_pos_z,
						double & delta_mass_moment_of_inertia_x,
						double & delta_mass_moment_of_inertia_y,
						double & delta_mass_moment_of_inertia_z
						)
{
	return false;
	//if (fuel_consumption_since_last_time > 0)
	//{
	//	delta_mass		 = fuel_consumption_since_last_time;
	//	delta_mass_pos_x = -1.0;
	//	delta_mass_pos_y =  1.0;
	//	delta_mass_pos_z =  0;

	//	delta_mass_moment_of_inertia_x	= 0;
	//	delta_mass_moment_of_inertia_y	= 0;
	//	delta_mass_moment_of_inertia_z	= 0;

	//	fuel_consumption_since_last_time = 0; // set it 0 to avoid infinite loop, because it called in cycle 
	//	// better to use stack like structure for mass changing 
	//	return true;
	//}
	//else 
	//{
	//	return false;
	//}
}
/*
	set internal fuel volume , init function, called on object creation and for refueling , 
	you should distribute it inside at different fuel tanks
*/
void   ed_fm_set_internal_fuel(double fuel)
{
	//internal_fuel = fuel;
}
/*
	get internal fuel volume 
*/
double ed_fm_get_internal_fuel()
{
	return 1.0;
}
/*
	set external fuel volume for each payload station , called for weapon init and on reload
*/
void  ed_fm_set_external_fuel (int	 station,
								double fuel,
								double x,
								double y,
								double z)
{

}
/*
	get external fuel volume 
*/
double ed_fm_get_external_fuel ()
{
	return 0;
}

void ed_fm_set_draw_args (EdDrawArgument * drawargs,size_t size)
{
	//drawargs[28].f   = (float)throttle;
	//drawargs[29].f   = (float)throttle;

	//if (size > 616)
	//{	
	//	drawargs[611].f = drawargs[0].f;
	//	drawargs[614].f = drawargs[3].f;
	//	drawargs[616].f = drawargs[5].f;
	//}

}


void ed_fm_configure(const char * cfg_path)
{

}

double test_gear_state = 0;
double ed_fm_get_param(unsigned index)
{
	
switch (index)
{
case ED_FM_SUSPENSION_0_GEAR_POST_STATE:
case ED_FM_SUSPENSION_0_DOWN_LOCK:
case ED_FM_SUSPENSION_0_UP_LOCK:
	return 1.0;

case ED_FM_SUSPENSION_1_GEAR_POST_STATE:
case ED_FM_SUSPENSION_1_DOWN_LOCK:
case ED_FM_SUSPENSION_1_UP_LOCK:
	return 1.0;

case ED_FM_SUSPENSION_2_GEAR_POST_STATE:
case ED_FM_SUSPENSION_2_DOWN_LOCK:
case ED_FM_SUSPENSION_2_UP_LOCK:
	
return 1.0;
}

	return 0;

}


void ed_fm_cold_start()
{

}

void ed_fm_hot_start()
{

}

void ed_fm_hot_start_in_air()
{

}

bool ed_fm_add_local_force_component( double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z )
{
	return false;
}

bool ed_fm_add_global_force_component( double & x,double &y,double &z,double & pos_x,double & pos_y,double & pos_z )
{
	return false;
}

bool ed_fm_add_local_moment_component( double & x,double &y,double &z )
{
	return false;
}

bool ed_fm_add_global_moment_component( double & x,double &y,double &z )
{
	return false;
}


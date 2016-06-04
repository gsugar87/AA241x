/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file aa241x_fw_control.cpp
 *
 * Secondary file to the fixedwing controller containing the
 * control law for the AA241x class.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */

#include <uORB/uORB.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

// needed for variable names
using namespace aa241x_high;

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */


float altitude_desired = 0.0f;
float velocity_desired = 0.0f;
float last_man_throttle_in = 0.0f;

float dt = 0.0f;

const float PI_F = 4.0f*atanf(1.0f);

void velocity_control(){
	if(low_data.isLine){
		if(aah_parameters.throttle_line > 1.0f){throttle_servo_out =  last_man_throttle_in;}
		else 								   {throttle_servo_out =  aah_parameters.throttle_line;}
	}
	else{
		if(aah_parameters.throttle_circle > 1.0f){throttle_servo_out =  last_man_throttle_in;}
		else 									 {throttle_servo_out =  aah_parameters.throttle_circle;}
	}
}

void pitch_control(float pitch_desired){
	float delta_pitch = pitch_desired - pitch;
    float elevator_desired = (aah_parameters.proportional_pitch_gain)*(delta_pitch);
    elevator_desired+=aah_parameters.elevator_trim;

    elevator_desired+=aah_parameters.derivative_pitch_gain*(-pitch_rate);
	
	if(aah_parameters.invert_ele_servo>0)	{pitch_servo_out =  math::constrain(elevator_desired, -1.0f, 1.0f);}
	else 									{pitch_servo_out = -math::constrain(elevator_desired, -1.0f, 1.0f);}
}

void altitude_control_circle(float altitude_desired){
	float delta_altitude = altitude_desired + position_D_gps;
	float roll_desired = aah_parameters.proportional_altitude_gain_circle*delta_altitude +
						 aah_parameters.derivative_altitude_gain_circle*vel_D +
						 aah_parameters.circle_roll_trim;

	roll_desired = math::constrain(roll_desired, -aah_parameters.roll_lim*PI_F/180.0f, aah_parameters.roll_lim*PI_F/180.0f);
	high_data.target_roll = math::constrain(roll_desired*180.0f/PI_F, -aah_parameters.roll_lim, aah_parameters.roll_lim);
	roll_control(roll_desired);
}

void altitude_control(float altitude_desired){
	float delta_altitude = altitude_desired - (-position_D_gps);
    float pitch_desired = aah_parameters.proportional_altitude_gain*(delta_altitude);

	pitch_desired+=aah_parameters.derivative_altitude_gain*(vel_D);

    pitch_control(math::constrain(pitch_desired, -PI_F*5.0f/18.0f, PI_F*5.0f/18.0f));
}

void roll_control(float roll_desired){
	float delta_roll = roll_desired - roll;
    float aileron_desired = aah_parameters.proportional_roll_gain*(delta_roll);
    aileron_desired+=aah_parameters.aileron_trim;

	aileron_desired+=aah_parameters.derivative_roll_gain*(-roll_rate);
  	
  	if(aah_parameters.invert_ail_servo>0)	{roll_servo_out =  math::constrain(aileron_desired, -1.0f, 1.0f);}
	else 									{roll_servo_out = -math::constrain(aileron_desired, -1.0f, 1.0f);}
}

void yaw_control_circle(float yaw_desired){
	float delta_yaw = yaw_desired - yaw;
	if(delta_yaw >= PI_F)			{delta_yaw -= 2.0f*PI_F;}
	else if(delta_yaw <= -PI_F)		{delta_yaw += 2.0f*PI_F;}

	float pitch_desired = delta_yaw*aah_parameters.proportional_yaw_gain_circle +
			              yaw_rate*aah_parameters.derivative_yaw_gain_circle +
				          aah_parameters.circle_pitch_trim;
	pitch_desired = math::constrain(pitch_desired, -abs(aah_parameters.pitch_maxmin_circle)*PI_F/180.0f,
									abs(aah_parameters.pitch_maxmin_circle)*PI_F/180.0f);
	pitch_control(pitch_desired);
}

void yaw_control(float yaw_desired){
	float delta_yaw = yaw_desired - yaw;
	if(delta_yaw >= PI_F)			{delta_yaw -= 2.0f*PI_F;}
	else if(delta_yaw <= -PI_F)		{delta_yaw += 2.0f*PI_F;}

	float roll_desired = aah_parameters.proportional_yaw_gain*(delta_yaw);
    
    roll_desired += aah_parameters.derivative_yaw_gain*(-yaw_rate);

    high_data.target_roll = math::constrain(roll_desired, -aah_parameters.roll_lim, aah_parameters.roll_lim);

    roll_control(math::constrain(roll_desired, -aah_parameters.roll_lim*PI_F/180.0f, aah_parameters.roll_lim*PI_F/180.0f));
}

void rudder_control_circle(){
	if(aah_parameters.invert_rud_servo>0)	{yaw_servo_out =  math::constrain(aah_parameters.proportional_rudder_gain*sinf(roll), -1.0f, 1.0f);}
	else 									{yaw_servo_out = -math::constrain(aah_parameters.proportional_rudder_gain*sinf(roll), -1.0f, 1.0f);}
}

void rudder_control_line(){
	yaw_servo_out = 0;
}

float getMissionYaw(){
	return atan2(low_data.next_E-position_E,low_data.next_N-position_N);
}


void radius_control(){
	float delta_east = position_E-low_data.centerE;
	float delta_north = position_N-low_data.centerN;
	float distance_to_center = sqrtf(pow(delta_east,2)+pow(delta_north,2));
	float delta_radius = distance_to_center-low_data.radius;
	float delta_radius_derivative = (vel_N*delta_north + vel_E*delta_east)/distance_to_center;
	float delta_yaw = aah_parameters.proportional_radius_gain_circle*delta_radius +
					  aah_parameters.derivative_radius_gain_circle*delta_radius_derivative;
	delta_yaw = math::constrain(delta_yaw, -PI_F*0.5f,PI_F*0.5f);
	float yaw_perp = getMissionYaw();
	float yaw_desired = yaw_perp+delta_yaw;
	yaw_control_circle(yaw_desired);
}

//void radius_control(float yaw_perp){
//	float delta_dist = sqrtf(pow((low_data.centerE-position_E),2)+pow((low_data.centerN-position_N),2))-low_data.radius;
//	float yaw_desired = yaw_perp + math::constrain(delta_dist*aah_parameters.proportional_rad_err_gain, -PI_F*.5f,PI_F*.5f);
//
//	if(yaw_desired >= PI_F)			{yaw_desired -= 2.0f*PI_F;}
//	else if(yaw_desired <= -PI_F)	{yaw_desired += 2.0f*PI_F;}
//	yaw_control(yaw_desired);
//}
//
//void radius_control_with_roll(){
//	float delta_dist = sqrtf(pow((low_data.centerE-position_E),2)+pow((low_data.centerN-position_N),2))-low_data.radius;
//
//	float velocity_filtered = sqrtf(pow(vel_N,2) + pow(vel_E,2));
//	float roll_desired =delta_dist*aah_parameters.proportional_rad_err_gain+float(atan2(pow(velocity_filtered,2),(low_data.radius*9.81f)));
//	if(roll_desired >= PI_F)		{roll_desired -= 2.0f*PI_F;}
//	else if(roll_desired <= -PI_F)	{roll_desired += 2.0f*PI_F;}
//	roll_control(math::constrain(roll_desired, -aah_parameters.roll_lim, aah_parameters.roll_lim));
//}

void flight_control() {
	dt = hrt_absolute_time() - previous_loop_timestamp;
	if (dt > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop, 
																	 //	should only occur on first engagement since this is 59Hz loop
		//yaw_desired = yaw; 							// yaw_desired already defined in aa241x_high_aux.h
		// float altitude_desired = position_D_baro; 		// altitude_desired needs to be declared
		altitude_desired = (-position_D_gps);
        velocity_desired = ground_speed;
        yaw_desired = yaw;

        last_man_throttle_in = man_throttle_in;

	}

	// TODO: write all of your flight control here...
	high_data.test_case=aah_parameters.test_case;
	high_data.command_altitude = aah_parameters.command_alt;
	//Step into to altitude
	if(aah_parameters.test_case<=0.0f){
        velocity_control();
        if(aah_parameters.command_alt<=0.0f){
	        altitude_control(altitude_desired);
    	}else{
    		altitude_control(aah_parameters.command_alt);
    	}

		roll_control(man_roll_in*PI_F*.5f);
		yaw_servo_out = man_yaw_in;
	}
	//Auto-Velocity
	else if(aah_parameters.test_case<=1.0f){
        velocity_control();
        pitch_servo_out = -man_pitch_in;
		roll_servo_out = man_roll_in;
		yaw_servo_out = man_yaw_in;
	}
	//Auto-Pitch
	else if(aah_parameters.test_case<=2.0f){
        throttle_servo_out = man_throttle_in;
        pitch_control(man_pitch_in*PI_F*.5f);
		roll_servo_out = man_roll_in;
		yaw_servo_out = man_yaw_in;
	}
	//Auto-Altitude
	else if(aah_parameters.test_case<=3.0f){
        velocity_control();
        altitude_control(altitude_desired);
		roll_servo_out = man_roll_in;
		yaw_servo_out = man_yaw_in;
	}
	//Auto-Roll
	else if(aah_parameters.test_case<=4.0f){
        throttle_servo_out = man_throttle_in;
		pitch_servo_out = -man_pitch_in;
		roll_control(man_roll_in*PI_F*.5f);
		yaw_servo_out = man_yaw_in;
	}
	//Auto-Yaw
	else if(aah_parameters.test_case<=5.0f){
        throttle_servo_out = man_throttle_in;
        pitch_servo_out = -man_pitch_in;
		yaw_control(yaw_desired);
		yaw_servo_out = man_yaw_in;
	}
	//Set-Yaw
	else if(aah_parameters.test_case<=6.0f){
        throttle_servo_out = man_throttle_in;
		pitch_servo_out = -man_pitch_in;
        yaw_control(aah_parameters.set_yaw);
		yaw_servo_out = man_yaw_in;
	}
	//Auto-Rudder
	else if(aah_parameters.test_case<=7.0f){
		throttle_servo_out = man_throttle_in;
		roll_servo_out = man_roll_in;
        pitch_servo_out = -man_pitch_in;
        rudder_control_circle();
	}
	//FULL MISSION
	else{
		velocity_control();
		if (low_data.isLine){
			// We are in a Line
			altitude_control(low_data.next_Alt);
			rudder_control_line();
			if(low_data.race_complete<=0.0f){
				yaw_desired = getMissionYaw();
			}
			yaw_control(yaw_desired);
		}
		else{
			// We are in a Circle
			rudder_control_circle();
			altitude_control_circle(low_data.next_Alt);
			radius_control();
		}
	}
}

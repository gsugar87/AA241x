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
float start_circle_time = 0.0f;

float dt = 0.0f;

const float PI_F = 4.0f*atanf(1.0f);

void velocity_control_line(){
	if(aah_parameters.throttle_line > 1.0f){throttle_servo_out =  last_man_throttle_in;}
	else 								   {throttle_servo_out =  aah_parameters.throttle_line;}
}

void pitch_control_line(float pitch_desired){
	float delta_pitch = pitch_desired - pitch;
    float elevator_desired = aah_parameters.proportional_pitch_gain*delta_pitch +
    						 aah_parameters.elevator_trim +
    						 aah_parameters.derivative_pitch_gain*(-pitch_rate);
	
	if(aah_parameters.invert_ele_servo>0)	{pitch_servo_out =  math::constrain(elevator_desired, -1.0f, 1.0f);}
	else 									{pitch_servo_out = -math::constrain(elevator_desired, -1.0f, 1.0f);}
}

void altitude_control_line(float altitude_desired){
	float delta_altitude = altitude_desired - (-position_D_gps);
    float pitch_desired = 	aah_parameters.proportional_altitude_gain*delta_altitude +
							aah_parameters.derivative_altitude_gain*vel_D;
	pitch_desired = math::constrain(pitch_desired, -PI_F*5.0f/18.0f, PI_F*5.0f/18.0f);
	high_data.target_pitch = pitch_desired;
    pitch_control_line(pitch_desired);
}

void yaw_control_line(float yaw_desired){
	high_data.target_yaw = yaw_desired;
	float delta_yaw = yaw_desired - yaw;
	if(delta_yaw >= PI_F)			{delta_yaw -= 2.0f*PI_F;}
	else if(delta_yaw <= -PI_F)		{delta_yaw += 2.0f*PI_F;}

	float roll_desired = 	aah_parameters.proportional_yaw_gain*delta_yaw +
							aah_parameters.derivative_yaw_gain*(-yaw_rate);
    roll_desired = math::constrain(roll_desired, -aah_parameters.roll_lim*PI_F/180.0f, 
    								aah_parameters.roll_lim*PI_F/180.0f);
	high_data.target_roll = roll_desired;
    roll_control(roll_desired);
}

void rudder_control_line(){
	yaw_servo_out = 0;
}

void line_control(){ 
	velocity_control_line();
	altitude_control_line(low_data.next_Alt);
	rudder_control_line();
	if(low_data.race_complete<=0.0f){
		yaw_desired = getMissionYaw();
	}
	yaw_control_line(yaw_desired);
}




void velocity_control_circle(){
	if(aah_parameters.throttle_circle > 1.0f){throttle_servo_out =  last_man_throttle_in;}
	else 									 {throttle_servo_out =  aah_parameters.throttle_circle;}
}

void pitch_control_circle(float pitch_desired){
	float delta_pitch = pitch_desired - pitch;
    float roll_desired = aah_parameters.proportional_pitch_gain_circle*delta_pitch + 
    					 aah_parameters.derivative_pitch_gain_circle*(-pitch_rate) +
    					 aah_parameters.circle_roll_trim*PI_F/180.0f;
	roll_desired = math::constrain(roll_desired, -aah_parameters.roll_lim*PI_F/180.0f, 
    								aah_parameters.roll_lim*PI_F/180.0f);
	high_data.target_roll = roll_desired;
	roll_control(roll_desired);
}

void altitude_control_circle(float altitude_desired){
	float delta_altitude = altitude_desired - (-position_D_gps);
	float pitch_desired = aah_parameters.proportional_altitude_gain_circle*delta_altitude +
						  aah_parameters.derivative_altitude_gain_circle*vel_D;
	pitch_desired = math::constrain(pitch_desired, -aah_parameters.pitch_maxmin_circle*PI_F/180.0f, 
									 aah_parameters.pitch_maxmin_circle*PI_F/180.0f);
	high_data.target_pitch= pitch_desired;
	pitch_control_circle(pitch_desired);
}

void yaw_control_circle(float yaw_desired){
	high_data.target_yaw = yaw_desired;
	float delta_yaw = yaw_desired - yaw;
	if(delta_yaw >= PI_F)			{delta_yaw -= 2.0f*PI_F;}
	else if(delta_yaw <= -PI_F)		{delta_yaw += 2.0f*PI_F;}

	float elevator_desired = aah_parameters.proportional_yaw_gain_circle*delta_yaw +
			              	 aah_parameters.derivative_yaw_gain_circle*(-yaw_rate) +
				          	 aah_parameters.circle_elev_trim;
	elevator_desired *= sinf(roll);
	if(aah_parameters.invert_ele_servo>0)	{pitch_servo_out =  math::constrain(elevator_desired, aah_parameters.elevator_min_circle, 1.0f);}
	else 									{pitch_servo_out = -math::constrain(elevator_desired, aah_parameters.elevator_min_circle, 1.0f);}
}

void rudder_control_circle(){
	if(aah_parameters.invert_rud_servo>0)	{yaw_servo_out =  math::constrain(aah_parameters.proportional_rudder_gain*sinf(roll), -1.0f, 1.0f);}
	else 									{yaw_servo_out = -math::constrain(aah_parameters.proportional_rudder_gain*sinf(roll), -1.0f, 1.0f);}
}

void roll_control(float roll_desired){
	float delta_roll = roll_desired - roll;
    float aileron_desired = aah_parameters.proportional_roll_gain*delta_roll +
							aah_parameters.derivative_roll_gain*(-roll_rate) + 
							aah_parameters.aileron_trim;
  	if(aah_parameters.invert_ail_servo>0)	{roll_servo_out =  math::constrain(aileron_desired, -1.0f, 1.0f);}
	else 									{roll_servo_out = -math::constrain(aileron_desired, -1.0f, 1.0f);}
}

float getMissionYaw(){
	return atan2(low_data.next_E-position_E,low_data.next_N-position_N);
}

//Old Radius Control
/*
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
*/
void radius_control(){
	/*
	float delta_east = position_E-low_data.centerE;
	float delta_north = position_N-low_data.centerN;
	float distance_to_center = sqrtf(pow(delta_east,2)+pow(delta_north,2));
	//float delta_radius = distance_to_center-low_data.radius;
	float delta_radius_derivative = (vel_N*delta_north + vel_E*delta_east)/distance_to_center;
	float elevator_desired = aah_parameters.proportional_radius_gain_circle*distance_to_center -
			              	 aah_parameters.derivative_radius_gain_circle*delta_radius_derivative +
							 aah_parameters.circle_elev_trim;
	*/
	if(hrt_absolute_time()- low_data.circle_start_time > aah_parameters.circle_elev_delay*1000000.0f){
		float delta_yaw =getMissionYaw()-yaw;
	    if(delta_yaw >= PI_F)			{delta_yaw -= 2.0f*PI_F;}
		else if(delta_yaw <= -PI_F)		{delta_yaw += 2.0f*PI_F;}

		float elevator_desired = aah_parameters.proportional_radius_gain_circle*delta_yaw +
				              	 aah_parameters.derivative_radius_gain_circle*(-yaw_rate) +
								 aah_parameters.circle_elev_trim;

		if(aah_parameters.invert_ele_servo>0)	{pitch_servo_out =  math::constrain(elevator_desired, aah_parameters.elevator_min_circle, 1.0f);}
		else 									{pitch_servo_out = -math::constrain(elevator_desired, aah_parameters.elevator_min_circle, 1.0f);}
	}	
}



void flight_control() {
	dt = hrt_absolute_time() - previous_loop_timestamp;
	if (dt > 500000.0f) { // Run if more than 0.5 seconds have passes since last loop, 
																	 //	should only occur on first engagement since this is 59Hz loop
		//yaw_desired = yaw; 							// yaw_desired already defined in aa241x_high_aux.h
		// float altitude_desired = position_D_baro; 		// altitude_desired needs to be declared
		if(aah_parameters.command_alt<=0){
			altitude_desired = (-position_D_gps);
		}else{
			altitude_desired = aah_parameters.command_alt;
		}
        velocity_desired = ground_speed;
        yaw_desired = yaw;

        last_man_throttle_in = man_throttle_in;

	}

	// TODO: write all of your flight control here...
	high_data.test_case=aah_parameters.test_case;
	high_data.command_altitude = altitude_desired;
	//Step into to altitude
	if(aah_parameters.test_case<=0.0f){
        velocity_control_line();
        altitude_control_line(altitude_desired);
		roll_control(man_roll_in*PI_F*.5f);
		yaw_servo_out = man_yaw_in;
	}
	//Auto-Velocity
	else if(aah_parameters.test_case<=1.0f){
        velocity_control_line();
        pitch_servo_out = -man_pitch_in;
		roll_servo_out = man_roll_in;
		yaw_servo_out = man_yaw_in;
	}
	//Auto-Pitch
	else if(aah_parameters.test_case<=2.0f){
        throttle_servo_out = man_throttle_in;
        pitch_control_line(man_pitch_in*PI_F*.5f);
		roll_servo_out = man_roll_in;
		yaw_servo_out = man_yaw_in;
	}
	//Auto-Altitude
	else if(aah_parameters.test_case<=3.0f){
        velocity_control_line();
        altitude_control_line(altitude_desired);
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
		yaw_control_line(yaw_desired);
		yaw_servo_out = man_yaw_in;
	}
	//Set-Yaw
	else if(aah_parameters.test_case<=6.0f){
        throttle_servo_out = man_throttle_in;
		pitch_servo_out = -man_pitch_in;
        yaw_control_line(aah_parameters.set_yaw);
		yaw_servo_out = man_yaw_in;
	}
	//Auto-Rudder
	else if(aah_parameters.test_case<=7.0f){
		throttle_servo_out = man_throttle_in;
		roll_servo_out = man_roll_in;
        pitch_servo_out = -man_pitch_in;
        rudder_control_circle();
	}

	//Circle Following Tests
	//Manual Roll, Auto Radius, Tweek Radius and Yaw Params
	else if(aah_parameters.test_case<=8.0f){
		if (low_data.isLine){
			// We are in a Line
			line_control();
		}
		else{
			velocity_control_circle();
			rudder_control_circle();
			roll_control(man_roll_in*PI_F*.2f+aah_parameters.circle_roll_trim*PI_F/180.0f);
			radius_control();
		}
	}
	//Manual Pitch, Auto Radius, Tweek Pitch Params
	else if(aah_parameters.test_case<=9.0f){
		if (low_data.isLine){
			// We are in a Line
			line_control();
		}
		else{
			velocity_control_circle();
			rudder_control_circle();
			pitch_control_circle(man_pitch_in*PI_F*.5f);
			radius_control();
		}
	}
	//Auto 
	else if(aah_parameters.test_case<=10.0f){
		if (low_data.isLine){
			// We are in a Line
			line_control();
		}
		else{
			velocity_control_circle();
			rudder_control_circle();
			altitude_control_circle(aah_parameters.command_alt);
			radius_control();
		}
	}
	else if(aah_parameters.test_case<=11.0f){
		velocity_control_circle();
		rudder_control_circle();
		altitude_control_circle(altitude_desired);
		radius_control();
	}
	//FULL MISSION
	else{
		if (low_data.isLine){
			// We are in a Line
			line_control();
		}
		else{
			// We are in a Circle
			velocity_control_circle();
			rudder_control_circle();
			altitude_control_circle(low_data.next_Alt);
			radius_control();
		}
	}
}

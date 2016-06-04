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
 * @file aa241x_fw_control_params.h
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */
#pragma once

#ifndef AA241X_FW_CONTROL_PARAMS_H_
#define AA241X_FW_CONTROL_PARAMS_H_


#include <systemlib/param/param.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Struct of all of the custom parameters.
 *
 * Please make sure to add a variable for each of your newly defined
 * parameters here.
 */
struct aah_params {

	// TODO: add custom parameter variable names here......
	float proportional_pitch_gain;
	float derivative_pitch_gain;
	
	float proportional_altitude_gain;
	float derivative_altitude_gain;
	float proportional_altitude_gain_cicle;

	float proportional_yaw_gain;
	float derivative_yaw_gain;

	float proportional_roll_gain;
	float derivative_roll_gain;
	float roll_lim;

	float proportional_rudder_gain;

	float proportional_rad_err_gain;

	float throttle_line;
	float throttle_circle;

	float elevator_trim;
	float aileron_trim;
	float rudder_trim;

	float set_yaw;

	float test_case;

	float invert_ail_servo;
	float invert_rud_servo;
	float invert_ele_servo;
	float radius_control_by_roll;
	float command_alt;
	float circle_roll_trim;
	float circle_pitch_trim;
	
	float derivative_altitude_gain_circle;
	float proportional_altitude_gain_circle;
	float derivative_radius_gain_circle;
	float proportional_radius_gain_circle;
	float derivative_yaw_gain_circle;
	float proportional_yaw_gain_circle;

	
	float pitch_maxmin_circle;
	float delta_yaw_maxmin_circle;
};


/**
 * Struct of handles to all of the custom parameters.
 *
 *  Please make sure to add a variable for each of your newly
 *  defined parameters here.
 *
 *  NOTE: these variable names can be the same as the ones above
 *  (makes life easier if they are)
 */
struct aah_param_handles {

	// TODO: add custom parameter variable names here.......
	param_t proportional_pitch_gain;
	param_t derivative_pitch_gain;

	param_t proportional_altitude_gain;
	param_t derivative_altitude_gain;
	param_t proportional_altitude_gain_cicle;

	param_t proportional_yaw_gain;
	param_t derivative_yaw_gain;

	param_t proportional_roll_gain;
	param_t derivative_roll_gain;
	param_t roll_lim;
	
	param_t proportional_rudder_gain;

	param_t proportional_rad_err_gain;

	param_t throttle_line;
	param_t throttle_circle;

	param_t elevator_trim;
	param_t aileron_trim;
	param_t rudder_trim;

	param_t set_yaw;

	param_t test_case;

	param_t invert_ail_servo;
	param_t invert_rud_servo;
	param_t invert_ele_servo;
	
	param_t radius_control_by_roll;
	param_t command_alt;
	
	param_t circle_roll_trim;
	param_t circle_pitch_trim;
	
	param_t derivative_altitude_gain_circle;
	param_t proportional_altitude_gain_circle;
	param_t derivative_radius_gain_circle;
	param_t proportional_radius_gain_circle;
	param_t derivative_yaw_gain_circle;
	param_t proportional_yaw_gain_circle;
	
	
	param_t pitch_maxmin_circle;
	param_t delta_yaw_maxmin_circle;
};

/**
 * Initialize all parameter handles and values
 *
 */
int aah_parameters_init(struct aah_param_handles *h);

/**
 * Update all parameters
 *
 */
int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p);

#ifdef __cplusplus
}
#endif




#endif /* AA241X_FW_CONTROL_PARAMS_H_ */

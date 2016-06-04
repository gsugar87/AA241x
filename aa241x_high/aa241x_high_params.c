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
 * @file aa241x_fw_control_params.c
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */

#include "aa241x_high_params.h"



/*
 *  controller parameters, use max. 15 characters for param name!
 *
 */


/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
//PARAM_DEFINE_FLOAT(AAH_PROPROLLGAIN, 1.0f);

// TODO: define custom parameters here
PARAM_DEFINE_FLOAT(AAH_K_ALT_P, 0.25f);
PARAM_DEFINE_FLOAT(AAH_K_ALT_D, 0.025f);

PARAM_DEFINE_FLOAT(AAH_K_PIT_P, 0.7f);
PARAM_DEFINE_FLOAT(AAH_K_PIT_D, 0.0f);

PARAM_DEFINE_FLOAT(AAH_K_YAW_P, 1.6f);
PARAM_DEFINE_FLOAT(AAH_K_YAW_D, 0.18f);

PARAM_DEFINE_FLOAT(AAH_K_ROLL_P, 0.5f);
PARAM_DEFINE_FLOAT(AAH_K_ROLL_D, 0.0f);
PARAM_DEFINE_FLOAT(AAH_ROLL_LIM, 70.0f);

PARAM_DEFINE_FLOAT(AAH_K_RUD_P, 0.4f);

PARAM_DEFINE_FLOAT(AAH_THRO_LINE, 1.0f);
PARAM_DEFINE_FLOAT(AAH_THRO_CIRC, 0.7f);

PARAM_DEFINE_FLOAT(AAH_TRIM_AILE, 0.0055f);
PARAM_DEFINE_FLOAT(AAH_TRIM_ELEV, 0.0055f);
PARAM_DEFINE_FLOAT(AAH_TRIM_RUDD, 0.0f);

PARAM_DEFINE_FLOAT(TEST_YAW, 0.0f);

PARAM_DEFINE_FLOAT(TEST_CASE, 11.0f);

PARAM_DEFINE_FLOAT(INVERT_AIL_SERV, 0.0f);
PARAM_DEFINE_FLOAT(INVERT_RUD_SERV, 0.0f);
PARAM_DEFINE_FLOAT(INVERT_ELE_SERV, 0.0f);

PARAM_DEFINE_FLOAT(AAH_COMMAND_ALT,0.0f);

PARAM_DEFINE_FLOAT(AAH_C_ROLL_TRM,0.0f);
PARAM_DEFINE_FLOAT(AAH_C_PIT_TRM,0.0f);
PARAM_DEFINE_FLOAT(AAH_C_ALT_P,0.0f);
PARAM_DEFINE_FLOAT(AAH_C_ALT_D,0.0f);
PARAM_DEFINE_FLOAT(AAH_C_RAD_P,0.0f);
PARAM_DEFINE_FLOAT(AAH_C_RAD_D,0.0f);
PARAM_DEFINE_FLOAT(AAH_C_YAW_P,0.0f);
PARAM_DEFINE_FLOAT(AAH_C_YAW_D,0.0f);
PARAM_DEFINE_FLOAT(AAH_C_PIT_MAX,0.0f);
PARAM_DEFINE_FLOAT(AAH_C_YAW_MAX,0.0f);

int aah_parameters_init(struct aah_param_handles *h)
{

	/* for each of your custom parameters, make sure to define a corresponding
	 * variable in the aa_param_handles struct and the aa_params struct these
	 * structs can be found in the aa241x_fw_control_params.h file
	 *
	 * NOTE: the string passed to param_find is the same as the name provided
	 * in the above PARAM_DEFINE_FLOAT
	 */

	h->proportional_altitude_gain 	= param_find("AAH_K_ALT_P");
	h->derivative_altitude_gain 	= param_find("AAH_K_ALT_D");

	h->proportional_pitch_gain 		= param_find("AAH_K_PIT_P");
	h->derivative_pitch_gain		= param_find("AAH_K_PIT_D");


	h->proportional_yaw_gain 		= param_find("AAH_K_YAW_P");
	h->derivative_yaw_gain 			= param_find("AAH_K_YAW_D");

	h->proportional_roll_gain 		= param_find("AAH_K_ROLL_P");
	h->derivative_roll_gain			= param_find("AAH_K_ROLL_D");
	h->roll_lim 					= param_find("AAH_ROLL_LIM");

	h->proportional_rudder_gain     = param_find("AAH_K_RUD_P");

	h->throttle_line				= param_find("AAH_THRO_LINE");
	h->throttle_circle 				= param_find("AAH_THRO_CIRC");

	h->elevator_trim 				= param_find("AAH_TRIM_AILE");
	h->aileron_trim 				= param_find("AAH_TRIM_ELEV");
	h->rudder_trim 					= param_find("AAH_TRIM_RUDD");

	h->set_yaw  					= param_find("TEST_YAW");

	h->test_case 					= param_find("TEST_CASE");

	h->invert_ail_servo 			= param_find("INVERT_AIL_SERV");
	h->invert_rud_servo 			= param_find("INVERT_RUD_SERV");
	h->invert_ele_servo 			= param_find("INVERT_ELE_SERV");
	
	h->command_alt   				= param_find("AAH_COMMAND_ALT");
	
	h->circle_roll_trim				= param_find("AAH_C_ROLL_TRM");
	h->circle_pitch_trim			= param_find("AAH_C_PIT_TRM");
	
	h->derivative_altitude_gain_circle		= param_find("AAH_C_ALT_D");
	h->proportional_altitude_gain_circle	= param_find("AAH_C_ALT_P");

	h->derivative_radius_gain_circle		= param_find("AAH_C_RAD_D");
	h->proportional_radius_gain_circle		= param_find("AAH_C_RAD_P");	
	
	h->derivative_yaw_gain_circle		= param_find("AAH_C_YAW_D");
	h->proportional_yaw_gain_circle		= param_find("AAH_C_YAW_P");
	
	h->pitch_maxmin_circle				= param_find("AAH_C_PIT_MAX");
	h->delta_yaw_maxmin_circle			= param_find("AAH_C_YAW_MAX");
	// TODO: add the above line for each of your custom parameters........

	return OK;
}

int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p)
{
	// for each of your custom parameters, make sure to add this line with
	// the corresponding variable name

	// TODO: add the above line for each of your custom parameters.....
	param_get(h->proportional_altitude_gain, &(p->proportional_altitude_gain));
	param_get(h->derivative_altitude_gain,   &(p->derivative_altitude_gain));
	
	param_get(h->proportional_pitch_gain,    &(p->proportional_pitch_gain));
	param_get(h->derivative_pitch_gain,      &(p->derivative_pitch_gain));

	param_get(h->proportional_yaw_gain,      &(p->proportional_yaw_gain));
	param_get(h->derivative_yaw_gain,      	 &(p->derivative_yaw_gain));
	
	param_get(h->proportional_roll_gain,     &(p->proportional_roll_gain));
	param_get(h->derivative_roll_gain,       &(p->derivative_roll_gain));
	param_get(h->roll_lim,    				 &(p->roll_lim));
	
	param_get(h->proportional_rudder_gain,   &(p->proportional_rudder_gain));

	param_get(h->throttle_line,				 &(p->throttle_line));
	param_get(h->throttle_circle,			 &(p->throttle_circle));

	param_get(h->elevator_trim,				 &(p->elevator_trim));
	param_get(h->aileron_trim,				 &(p->aileron_trim));
	param_get(h->rudder_trim,				 &(p->rudder_trim));

	param_get(h->set_yaw,					 &(p->set_yaw));

	param_get(h->test_case,     			 &(p->test_case));

	param_get(h->invert_ail_servo,			 &(p->invert_ail_servo));
	param_get(h->invert_rud_servo,			 &(p->invert_rud_servo));
	param_get(h->invert_ele_servo,			 &(p->invert_ele_servo));
	
	param_get(h->command_alt, 				 &(p->command_alt));
	
	param_get(h->circle_roll_trim, 			 &(p->circle_roll_trim));
	param_get(h->circle_pitch_trim, 		 &(p->circle_pitch_trim));
	
	param_get(h->derivative_altitude_gain_circle, 		 	&(p->derivative_altitude_gain_circle));
	param_get(h->proportional_altitude_gain_circle, 		&(p->proportional_altitude_gain_circle));
	param_get(h->derivative_radius_gain_circle, 		 	&(p->derivative_radius_gain_circle));
	param_get(h->proportional_radius_gain_circle, 			&(p->proportional_radius_gain_circle));
	param_get(h->derivative_yaw_gain_circle, 		 		&(p->derivative_yaw_gain_circle));
	param_get(h->proportional_yaw_gain_circle, 				&(p->proportional_yaw_gain_circle));
	
	param_get(h->pitch_maxmin_circle, 		 			&(p->pitch_maxmin_circle));
	param_get(h->delta_yaw_maxmin_circle, 				&(p->delta_yaw_maxmin_circle));
	

	
	return OK;
}

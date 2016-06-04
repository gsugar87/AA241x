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
 * @file aa241x_low_params.c
 *
 * Definition of custom parameters for low priority controller.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */

#include "aa241x_low_params.h"



/*
 * controller parameters, use max. 15 characters for param name!
 *
 */

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAL_EXAMPLE and will be in the AAL dropdown.  Make sure to always
 * start your parameters with AAL to have them all in one place.
 *
 * The default value of this float parameter will be 10.0.
 *
 * @unit meter 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x Low Params			(always include this)
 */
PARAM_DEFINE_FLOAT(PATH_NUMBER,1.0f);
PARAM_DEFINE_FLOAT(OVERRIDE_TURN,0.0f);
PARAM_DEFINE_FLOAT(DT_MAX,0.0f);
PARAM_DEFINE_FLOAT(DT_PIVOT,0.0f);
PARAM_DEFINE_FLOAT(DRAD_CIRCLE,0.0f);
PARAM_DEFINE_FLOAT(RADIUS_PATH,0.0f);
PARAM_DEFINE_FLOAT(RADIUS_TARGET,0.0f);
PARAM_DEFINE_FLOAT(BLINE_SIDE_BUFF,5.0f);
PARAM_DEFINE_FLOAT(BLINE_THETA,35.0f);
PARAM_DEFINE_FLOAT(BLINE_START_BUFF,10.0f);
PARAM_DEFINE_FLOAT(LINE_CIRC_BUFF,2.0f);

PARAM_DEFINE_FLOAT(ALT_CLIMB_START, 10.0f);
PARAM_DEFINE_FLOAT(ALT_CLIMB_HEIGHT,10.0f);
PARAM_DEFINE_FLOAT(ALT_0_START,55.0f);
PARAM_DEFINE_FLOAT(ALT_1_PYLON,51.0f);
PARAM_DEFINE_FLOAT(ALT_2_PYLON,49.0f);
PARAM_DEFINE_FLOAT(ALT_3_END,45.0f);

PARAM_DEFINE_FLOAT(ALT_CLIMB_MODE, 0.0f);

// TODO: define custom parameters here


int aal_parameters_init(struct aal_param_handles *h)
{

	/* for each of your custom parameters, make sure to define a corresponding
	 * variable in the aa_param_handles struct and the aa_params struct these
	 * structs can be found in the aa241x_fw_control_params.h file
	 *
	 * NOTE: the string passed to param_find is the same as the name provided
	 * in the above PARAM_DEFINE_FLOAT
	 */
    h->followLineMode       =   param_find("PATH_NUMBER");
	h->override_turn       	=   param_find("OVERRIDE_TURN");
	h->maximum_dt       	=   param_find("DT_MAX");
	h->pivot_dt       		=   param_find("DT_PIVOT");
	h->dTheta 				=   param_find("DRAD_CIRCLE");
	h->target_radius 		= 	param_find("RADIUS_TARGET");
	h->path_radius 			= 	param_find("RADIUS_PATH");
	h->base_buff 			=   param_find("BLINE_SIDE_BUFF");
	h->base_theta			=	param_find("BLINE_THETA");
	h->prestart_dist		=	param_find("BLINE_START_BUFF");
	h->alt_climb_start		=	param_find("ALT_CLIMB_START");
	h->alt_climb_height		=	param_find("ALT_CLIMB_HEIGHT");
	h->alt_0				=	param_find("ALT_0_START");
	h->alt_1				=	param_find("ALT_1_PYLON");
	h->alt_2				=	param_find("ALT_2_PYLON");
	h->alt_3				=	param_find("ALT_3_END");
	h->alt_climb_mode		=	param_find("ALT_CLIMB_MODE");
	// TODO: add the above line for each of your custom parameters........
	return OK;
}

int aal_parameters_update(const struct aal_param_handles *h, struct aal_params *p)
{
	// for each of your custom parameters, make sure to add this line with
	// the corresponding variable name
	param_get(h->closeness_threshold, 	&(p->closeness_threshold));
    param_get(h->followLineMode, 		&(p->followLineMode));
    param_get(h->rotAngle, 				&(p->rotAngle));
    param_get(h->override_turn, 		&(p->override_turn));
	param_get(h->maximum_dt, 			&(p->maximum_dt));
	param_get(h->pivot_dt, 				&(p->pivot_dt));
	param_get(h->dTheta, 				&(p->dTheta));
	param_get(h->target_radius, 		&(p->target_radius));
	param_get(h->path_radius, 			&(p->path_radius));
	param_get(h->base_buff, 			&(p->base_buff));
	param_get(h->base_theta, 			&(p->base_theta));
	param_get(h->prestart_dist, 		&(p->prestart_dist));
	param_get(h->alt_climb_start, 		&(p->alt_climb_start));
	param_get(h->alt_climb_height, 		&(p->alt_climb_height));
	param_get(h->alt_0, 				&(p->alt_0));
	param_get(h->alt_1, 				&(p->alt_1));
	param_get(h->alt_2, 				&(p->alt_2));
	param_get(h->alt_3,					&(p->alt_3));
	param_get(h->alt_climb_mode, 		&(p->alt_climb_mode));
	// TODO: add the above line for each of your custom parameters.....

	return OK;
}

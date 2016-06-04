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
 * @file aa241x_high_data_struct.h
 *
 * Structure for the data to be sent from this module
 * (the low priority module) to the high priority module.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */

#ifndef AA241X_LOW_DATA_STRUCT_H_
#define AA241X_LOW_DATA_STRUCT_H_

/**
 * This is the structure that will contain the data to be sent
 * from the low priority thread to the high priority thread.
 *
 * NOTE: do not edit the define name (LOW_FIELD#), but please
 * do change variable_name# to be your desired variable names.
 *
 * NOTE: any fields you add to this structure will NOT be logged
 * or sent to the ground station, therefore it is recommended
 * you DO NOT ADD FIELDS TO THIS STRUCT.
 *
 * You may also change the data type of fields to one of the following:
 * boolean, int (of any size and type)
 * Though note that all the data will be logged as a float.
 *
 */

// list of variable names
#define LOW_FIELD1 next_N;		/**< change variable_name1 to the desired variable name */
#define LOW_FIELD2 next_E;
#define LOW_FIELD3 next_Alt;
#define LOW_FIELD4 race_complete;
#define LOW_FIELD5 line_index;
#define LOW_FIELD6 distBtwPlanes;
#define LOW_FIELD7 dT;
#define LOW_FIELD8 isLine;
#define LOW_FIELD9 centerN;
#define LOW_FIELD10 centerE;
#define LOW_FIELD11 radius;
#define LOW_FIELD12 variable_name12;
#define LOW_FIELD13 variable_name13;
#define LOW_FIELD14 variable_name14;
#define LOW_FIELD15 variable_name15;
#define LOW_FIELD16 variable_name16;

/**
 * This string is a list of labels for each field in the log file.
 *
 * Feel free to edit f## to the desired name, but please note length of each
 * label must be less than 3 char per label!!!!
 *
 * THIS STRING MUST BE < 64 CHARS LONG AND CONTAIN 16 LABELS, IF IT DOES NOT
 * MEET THESE REQUIREMENTS, LOGGING WILL FAIL!!!!!!
 *
 */
#define LOW_DATA_LABELS "Nwa,Ewa,Awa,Rc,Lin,CrD,dT,isL,ceN,ceE,rad,f12,f13,f14,f15,f16"

struct aa241x_low_data_s {
	float LOW_FIELD1;
	float LOW_FIELD2;
	float LOW_FIELD3;
	int   LOW_FIELD4;
	int   LOW_FIELD5;
	float LOW_FIELD6;
	float LOW_FIELD7;
	int   LOW_FIELD8;
	float LOW_FIELD9;
	float LOW_FIELD10;
	float LOW_FIELD11;
	float LOW_FIELD12;
	float LOW_FIELD13;
	float LOW_FIELD14;
	float LOW_FIELD15;
	float LOW_FIELD16;
};


#endif /* AA241X_LOW_DATA_STRUCT_H_ */

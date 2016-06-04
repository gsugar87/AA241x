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
 * @file aa241x_low.cpp
 *
 * Secondary control law file for AA241x.  Contains control/navigation
 * logic to be executed with a lower priority, and "slowly"
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */


// include header file
#include "aa241x_low_control_law.h"
#include "aa241x_low_aux.h"

#include <uORB/uORB.h>
#include "racepath.h"

using namespace aa241x_low;

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 *
 * This loop executes at ~50Hz, but is not guaranteed to be 50Hz every time.
 */

const float PI_F = 4.0f*atanf(1.0f);

float nCoeff = 0.0f;
float eCoeff = 0.0f;

float originN     = 0.0f;
float originE     = 0.0f;

float centerN     = 0.0f;
float centerE     = 0.0f;

float startAlt = 0.0f;

float endN = 0.0f;
float endE = 0.0f;
float endAlt = 0.0f;

int pathNumber = 0;

float* lineStartingPoints = {};
int* isLinePts = {};
int n_numOfPts;

int currentTurn = 0;



float getLinePercentage(float lengthTraveled){
	if(isLinePts[low_data.line_index]){
		float lineLength = findDist3D(originN, endN, originE, endE);
		return lengthTraveled/lineLength*100.0f;
	}
	else{
		return 0.0f;
	}
}

float getLineAltDesired(float percentage){
	/*
	LOGIC OF DECIDING THE DESIRED ALTITUDE HERE IS JUST AN EXAMPLE.
	THIS CODE IS TO BE CHANGED.
	*/
	float altDesired;
    float percentage_start = aal_parameters.alt_climb_start;
    //MODE = 0.0f, Linear
    if(aal_parameters.alt_climb_mode <= 0.0f){
    	if (percentage > percentage_start){
    		altDesired = startAlt + (percentage-percentage_start)/(100.0f-percentage_start)*(endAlt-startAlt);
    	}
    	else{
    		altDesired = startAlt;
    	}
    }
    //MODE = 1.0f, Quadratic
    else{
        if (percentage > percentage_start){
            float a = (endAlt - startAlt)/float(pow(100.0f-percentage_start,2));
            altDesired = startAlt + a*float(pow(percentage - percentage_start,2));
        }
        else{
            altDesired = startAlt;
        }
    }
	return altDesired;
}

//Indeces = [N,E];
float * calcTestCircle(float *pylon1){
    float radius = aal_parameters.path_radius;
    float start[2];
    static float racePath[6*3];
    start[0] = position_N;
    start[1] = position_E;
    
    float pylon1_to_start_dist = findDist3D(start[0],pylon1[0],start[1],pylon1[1]);
    float angle_pylon1 = atan2(pylon1[0]-start[0],pylon1[1]-start[1]);
    float angle_pylon1_to_inter1 = asin(radius/pylon1_to_start_dist);
    float start_to_inter1_dist = sqrtf(pow(pylon1_to_start_dist,2)-pow(radius,2));
    float target_angle_1 = angle_pylon1+angle_pylon1_to_inter1;
    
    //Plane Current to Inter1
    racePath[0] = start[0];
    racePath[1] = start[1];
    racePath[2] = mission_parameters.max_alt-5;
    racePath[3] = start[0] + sinf(target_angle_1)*start_to_inter1_dist;
    racePath[4] = start[1] + cosf(target_angle_1)*start_to_inter1_dist;
    racePath[5] = mission_parameters.max_alt-5;

    //Center
    racePath[6+0] = pylon1[0];
    racePath[6+1] = pylon1[1];
    racePath[6+2] = mission_parameters.max_alt-5;
    racePath[6+3] = pylon1[0];
    racePath[6+4] = pylon1[1];
    racePath[6+5] = mission_parameters.max_alt-5;

    //Dummy Line
    racePath[6*2+0] = pylon1[0];
    racePath[6*2+1] = pylon1[1];
    racePath[6*2+2] = mission_parameters.max_alt-5;
    racePath[6*2+3] = pylon1[0];
    racePath[6*2+4] = pylon1[1];
    racePath[6*2+5] = mission_parameters.max_alt-5;

    return racePath;
}

float * calcRacePath(float *pylonStart, float *pylon1, float *pylon2){
    float radius = aal_parameters.path_radius;
    float start[2];
    static float racePath[6*13];

    //Current to Prestart Pt1
    racePath[6*0+0] = position_N;
    racePath[6*0+1] = position_E;
    racePath[6*0+2] = mission_parameters.max_alt-5;
    racePath[6*0+3] = 50.0f;
    racePath[6*0+4] = -150.0f;
    racePath[6*0+5] = mission_parameters.max_alt-5;
    
    //Prestart Pt1 to Prestart Pt2
    racePath[6*1+0] = 50.0f;
    racePath[6*1+1] = -150.0f;
    racePath[6*1+2] = mission_parameters.max_alt-5;
    racePath[6*1+3] = 120.0f;
    racePath[6*1+4] = -150.0f;
    racePath[6*1+5] = mission_parameters.max_alt-5;

    //Prestart Pt2 to Prestart Pt3
    racePath[6*2+0] = 120.0f;
    racePath[6*2+1] = -150.0f;
    racePath[6*2+2] = mission_parameters.max_alt-5;
    racePath[6*2+3] = 150.0f;
    racePath[6*2+4] = -125.0f;
    racePath[6*2+5] = mission_parameters.max_alt-5;

    // Get preStarting Point to Start Point
    float buffer = aal_parameters.base_buff;
    float base_angle = aal_parameters.base_theta;
    
    // Get start point based on the starting line
    start[0] =  pylonStart[0] + (25-buffer)*sinf(base_angle*PI_F/180); 
    start[1] =  pylonStart[1] + (25-buffer)*cosf(base_angle*PI_F/180);
    
    
    float pylon1_to_start_dist = findDist3D(start[0],pylon1[0],start[1],pylon1[1]);
    float angle_pylon1 = atan2(pylon1[0]-start[0],pylon1[1]-start[1]);
    float angle_pylon1_to_inter1 = asin(radius/pylon1_to_start_dist);
    float start_to_inter1_dist = sqrtf(pow(pylon1_to_start_dist,2)-pow(radius,2));
    float target_angle_1 = angle_pylon1+angle_pylon1_to_inter1;
    
    //Prestart Pt3 to Prestart Pt4
    racePath[6*3+0] = 150.0f;
    racePath[6*3+1] = -125.0f;
    racePath[6*3+2] = mission_parameters.max_alt-5;
    racePath[6*3+3] = start[0] - sinf(target_angle_1)*aal_parameters.prestart_dist;
    racePath[6*3+4] = start[1] - cosf(target_angle_1)*aal_parameters.prestart_dist;
    racePath[6*3+5] = mission_parameters.max_alt-5;
    
    // Get Prestart Pt4 to Start Point
    racePath[6*4+0] = start[0] - sinf(target_angle_1)*aal_parameters.prestart_dist;
    racePath[6*4+1] = start[1] - cosf(target_angle_1)*aal_parameters.prestart_dist;
    racePath[6*4+2] = mission_parameters.max_alt-5;
    racePath[6*4+3] = start[0];
    racePath[6*4+4] = start[1];
    racePath[6*4+5] = mission_parameters.max_alt-5;

    // Get Start to Inter1
    racePath[6*5+0] = start[0];
    racePath[6*5+1] = start[1];
    racePath[6*5+2] = mission_parameters.min_alt+10;
    racePath[6*5+3] = start[0]+sinf(target_angle_1)*start_to_inter1_dist;
    racePath[6*5+4] = start[1]+cosf(target_angle_1)*start_to_inter1_dist;
    racePath[6*5+5] = mission_parameters.min_alt+10;

    //Get Center1
    racePath[6*6+0] = pylon1[0];
    racePath[6*6+1] = pylon1[1];
    racePath[6*6+2] = mission_parameters.min_alt+10;
    racePath[6*6+3] = pylon1[0];
    racePath[6*6+4] = pylon1[1];
    racePath[6*6+5] = mission_parameters.min_alt+10;

    //Inter2a to Inter2b
    float * pylon1_to_pylon2_coeffArray = calculateLineCoeffs(pylon1[0],pylon2[0],pylon1[1],pylon2[1], true);
    float pylon1_to_pylon2_nCoeff = pylon1_to_pylon2_coeffArray[0];
    float pylon1_to_pylon2_eCoeff = pylon1_to_pylon2_coeffArray[1];
    racePath[6*7+0] = pylon1[0]+radius*pylon1_to_pylon2_eCoeff;
    racePath[6*7+1] = pylon1[1]-radius*pylon1_to_pylon2_nCoeff;
    racePath[6*7+2] = mission_parameters.min_alt+10;
    racePath[6*7+3] = pylon2[0]+radius*pylon1_to_pylon2_eCoeff;
    racePath[6*7+4] = pylon2[1]-radius*pylon1_to_pylon2_nCoeff;
    racePath[6*7+5] = mission_parameters.min_alt+10;

    //Get Center2
    racePath[6*8+0] = pylon2[0];
    racePath[6*8+1] = pylon2[1];
    racePath[6*8+2] = mission_parameters.min_alt+10;
    racePath[6*8+3] = pylon2[0];
    racePath[6*8+4] = pylon2[1];
    racePath[6*8+5] = mission_parameters.min_alt+10;

    //Get Inter2 to end
    float endPoint[2];
    endPoint[0] =  pylonStart[0] - (25-buffer)*sinf(base_angle*PI_F/180); 
    endPoint[1] =  pylonStart[1] - (25-buffer)*cosf(base_angle*PI_F/180);
    
    float pylon2_to_end_dist = findDist3D(endPoint[0],pylon2[0],endPoint[1],pylon2[1]);
    float angle_pylon2 = atan2(pylon2[0]-endPoint[0],pylon2[1]-endPoint[1]);
    float angle_pylon2_to_inter2 = asin(radius/pylon2_to_end_dist);
    float end_to_inter2_dist = sqrtf(pow(pylon2_to_end_dist,2)-pow(radius,2));
    float target_angle_2 = angle_pylon2-angle_pylon2_to_inter2;
    
    racePath[6*9+0] = endPoint[0]+sinf(target_angle_2)*end_to_inter2_dist;
    racePath[6*9+1] = endPoint[1]+cosf(target_angle_2)*end_to_inter2_dist;
    racePath[6*9+2] = mission_parameters.min_alt+10;
    racePath[6*9+3] = endPoint[0];
    racePath[6*9+4] = endPoint[1];
    racePath[6*9+5] = mission_parameters.min_alt+10;

    //End to PostEnd1
    racePath[6*10+0] = endPoint[0];
    racePath[6*10+1] = endPoint[1];
    racePath[6*10+2] = mission_parameters.min_alt+10;
    racePath[6*10+3] = 160.0f;
    racePath[6*10+4] = -100.0f;
    racePath[6*10+5] = mission_parameters.min_alt+10;

    //PostEnd1 to PostEnd2
    racePath[6*11+0] = 160.0f;
    racePath[6*11+1] = -100.0f;
    racePath[6*11+2] = mission_parameters.min_alt+10;
    racePath[6*11+3] = 180.0f;
    racePath[6*11+4] = -80.0f;
    racePath[6*11+5] = mission_parameters.min_alt+10;

    //PostEnd2 to PostEnd3
    racePath[6*12+0] = 180.0f;
    racePath[6*12+1] = -80.0f;
    racePath[6*12+2] = mission_parameters.min_alt+10;
    racePath[6*12+3] = 160.0f;
    racePath[6*12+4] = -60.0f;
    racePath[6*12+5] = mission_parameters.min_alt+10;
    
    return racePath;
}

void initializeParameters(){
    pathNumber = aal_parameters.followLineMode;
    //Durand
    if (pathNumber == 1){
        lineStartingPoints = lineStartingPoint1;
        isLinePts = isLinePt1;
        n_numOfPts = n_numOfPts1;
    }
    //Lake Lag Test
    else if(pathNumber == 2){
        lineStartingPoints = lineStartingPoint2;
        isLinePts = isLinePt2;
        n_numOfPts = n_numOfPts2;
    }
    //Circle Durand
    else if (pathNumber == 3){
        lineStartingPoints = lineStartingPoint3;
        isLinePts = isLinePt3;
        n_numOfPts = n_numOfPts3;
    }
    //Full Mission Old
    else if (pathNumber == 4){
        lineStartingPoints = lineStartingPoint4;
        isLinePts = isLinePt4;
        n_numOfPts = n_numOfPts4;
    }
    //Generate Mission Paths at Durand
    else if(pathNumber == 5){
        float start[] = {485.0,310.0};
        float pylon1[] = {500.0,312.0};
        float pylon2[] = {490.0,322.0};
        lineStartingPoints = calcRacePath(start, pylon1, pylon2);
        isLinePts = isLinePtMission;
        n_numOfPts = 13;
    }
    //Generate Test Circle at Lag
    else if(pathNumber == 6){
        float pylon1[] = {20,90};
        lineStartingPoints = calcTestCircle(pylon1);
        isLinePts = isLinePt6;
        n_numOfPts = 3;
    }
    //Generate Mission Paths at Lag
    else{
        float start[] = {150,-96};
        float pylon1[] = {47,125};
        float pylon2[] = {-93,-75};
        lineStartingPoints = calcRacePath(start, pylon1, pylon2);
        isLinePts = isLinePtMission;
        n_numOfPts = 13;
    }
    low_data.line_index = 0;
    low_data.race_complete = 0.0f;
    low_data.isLine = isLinePts[0];

    if(isLinePts[0]){
        updateLineParameters();
    }else{
        updateCircleParameters();
    }
}


/**
 * @brief findDist3D: finds the 3D distance between two points.
 * @param x1
 * @param x2
 * @param y1
 * @param y2
 * @param z1
 * @param z2
 * @return dist: distance to be returned.
 */
float findDist3D(float x1,float x2, float y1, float y2){
    float dist;
    dist = sqrtf(pow((x1-x2),2)+pow((y1-y2),2));
    return dist;
}

/**
 * @brief mergeLineByEqn: it updates the next point the aircraft should head to.
 * @param dT: the distance between the point that intersects with plane and the point.
 */
void mergeLineByEqn()
{
    // defining a line
    float pointParamOnPlane=(nCoeff*(position_N - originN) + eCoeff*(position_E - originE))/float(pow(nCoeff,2.0f)+pow(eCoeff,2.0f));
	
    
    // Need to get pointParamOnPlane
    float nDist = originN + nCoeff*pointParamOnPlane - position_N;
    float eDist = originE + eCoeff*pointParamOnPlane - position_E;
    //float dDist = originAlt + altCoeff*pointParamOnPlane - (-position_D_gps);
    float distBtwPlanes = sqrtf(pow(nDist,2) + pow(eDist,2));
    low_data.distBtwPlanes = distBtwPlanes;
	
	// getting percentage of the line traveled
	float linePercent = getLinePercentage(pointParamOnPlane);
	// get altitude desired based on the percentage
	float altDesired = getLineAltDesired(linePercent);
	
	//dT = 1/(distBtwPlanes+aal_parameters.manualDT);
    float dT;
	
	float pivotDT = aal_parameters.pivot_dt;
	float maxDT = aal_parameters.maximum_dt;
	if (pivotDT>=maxDT) {
		pivotDT = maxDT - 0.01f;
	}
	float baseLength = float(pow(pivotDT,2.0f))*(maxDT)/(maxDT - pivotDT);
	float epsilon = float(pow(pivotDT,2.0f))/(maxDT - pivotDT);
	dT = baseLength/(distBtwPlanes+epsilon);
    low_data.dT = dT;

    low_data.next_N = originN + nCoeff*(pointParamOnPlane + dT);
    low_data.next_E = originE + eCoeff*(pointParamOnPlane + dT);
    low_data.next_Alt = altDesired;
}

/**
 * @brief mergeCircleByEqn: it updates the next point the aircraft should head to.
 */
void mergeCircleByEqn()
{
    if(high_data.radius_control_mode <= 1.0f){
        // defining a line from plane to circle center
        float * plane_to_center_coeffArray = calculateLineCoeffs(position_N, centerN, position_E, centerE,true);
        float plane_to_center_nCoeff = plane_to_center_coeffArray[0];
        float plane_to_center_eCoeff = plane_to_center_coeffArray[1];
		
        //Get Perpindicular
        nCoeff = plane_to_center_eCoeff;
        eCoeff = -plane_to_center_nCoeff;

        low_data.next_N = position_N + nCoeff;
        low_data.next_E = position_E + eCoeff;
        low_data.next_Alt = endAlt;
    }
    else{
        float angle = atan2(position_N-centerN, position_E-centerE);
        low_data.next_N = centerN + aal_parameters.target_radius*sinf(angle-aal_parameters.dTheta);
        low_data.next_E = centerE + aal_parameters.target_radius*cosf(angle-aal_parameters.dTheta);
        low_data.next_Alt = endAlt;
    }
    
}

/**
 * @brief calculateLineParams: it finds the parameters for a line defined by two points
 * @param x1: first point, x location
 * @param x2: second point, x location
 * @param y1: first point, y location
 * @param y2: second point, y location
 * @param z1: sfirst point, z location
 * @param z2: second point, z location
 * @return: returns an array of parameters for each x, y, z direcitons
 */
float* calculateLineCoeffs(float x1, float x2, float y1, float y2, bool unitVector){
    static float coeffArray[2];
    float dx = x2 - x1;
    float dy = y2 - y1;
    if(unitVector){
        float dist = findDist3D(x1,x2,y1,y2);
        dx = dx/dist;
        dy = dy/dist;
    }
    coeffArray[0] = dx;
    coeffArray[1] = dy;
    return coeffArray;
}

bool transitionLine(){
    //Only check if your still have more points to go
    if(low_data.line_index < n_numOfPts){
        //The Transition is a Line to Line
        if(isLinePts[low_data.line_index] & isLinePts[low_data.line_index+1] & (low_data.line_index < n_numOfPts - 1)){
            float nextendN = lineStartingPoints[(low_data.line_index+1)*6 + 3];
            float nextendE = lineStartingPoints[(low_data.line_index+1)*6 + 4];

            float* new_coeffArray = calculateLineCoeffs(endN,nextendN,endE,nextendE,true);
            float bound_tan_nCoeff = (new_coeffArray[0]+nCoeff)/2;
            float bound_tan_eCoeff = (new_coeffArray[1]+eCoeff)/2;

            float* end_to_plane_coeffArray = calculateLineCoeffs(endN,position_N,endE,position_E,false);
            float end_to_plane_nCoeff = end_to_plane_coeffArray[0];
            float end_to_plane_eCoeff = end_to_plane_coeffArray[1];

            float dot_prod = bound_tan_nCoeff*end_to_plane_nCoeff + bound_tan_eCoeff*end_to_plane_eCoeff;
            return dot_prod > 0;
        }
        //The Transition is Line to Circle
        else if(isLinePts[low_data.line_index]){
            float* end_to_plane_coeffArray = calculateLineCoeffs(endN,position_N,endE,position_E,false);
            float end_to_plane_nCoeff = end_to_plane_coeffArray[0];
            float end_to_plane_eCoeff = end_to_plane_coeffArray[1];

            float dot_prod = nCoeff*end_to_plane_nCoeff + eCoeff*end_to_plane_eCoeff;
            return dot_prod > 0;
        }
        //The Transition if Circle to Line
        else{
            return turn_num > currentTurn || aal_parameters.override_turn > 0;
        }
    }
    else{
        low_data.race_complete = 1.0f;
        return false;
    }
}

void updateLineParameters(){
    originN = lineStartingPoints[low_data.line_index*6];
    originE = lineStartingPoints[low_data.line_index*6+1];
    
	startAlt = -position_D_gps;
	
    endN    = lineStartingPoints[low_data.line_index*6+3];
    endE    = lineStartingPoints[low_data.line_index*6+4];
    endAlt  = lineStartingPoints[low_data.line_index*6+5];

    float*  coeffArray = calculateLineCoeffs(originN,endN,originE,endE,true);
    nCoeff = coeffArray[0];
    eCoeff = coeffArray[1];
}

void updateCircleParameters(){
    centerN = lineStartingPoints[low_data.line_index*6];
    centerE = lineStartingPoints[low_data.line_index*6+1];
    endAlt =  lineStartingPoints[low_data.line_index*6+2];

    low_data.centerN = centerN;
    low_data.centerE = centerE;

    currentTurn = turn_num;
}

void missionControl(){
    if(transitionLine()){
        low_data.line_index++;
        if(isLinePts[low_data.line_index]){
            low_data.isLine = 1;
            updateLineParameters();
            mergeLineByEqn();
        }else{
            low_data.isLine = 0;
            updateCircleParameters();
            mergeCircleByEqn();
        }
    }else{
        if(isLinePts[low_data.line_index]){
            mergeLineByEqn();
        }else{
            mergeCircleByEqn();
        }
    }
}

void low_loop()
{
    low_data.radius = aal_parameters.target_radius;
    if(timestamp - previous_loop_timestamp >= 500000.0f)
    {
        initializeParameters();
    }
    if(aal_parameters.followLineMode > 0){
        missionControl();
    }
}


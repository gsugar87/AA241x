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
int sectionIndex = 1;

float nCoeff = 0.0f;
float eCoeff = 0.0f;
float altCoeff = 0.0f;

float originN     = 0.0f;
float originE     = 0.0f;
float originAlt   = 0.0f;

float endN = 0.0f;
float endE = 0.0f;
float endAlt = 0.0f;

float pathNumber = 0.0f;

void initializeParameters(){
    pathNumber = aal_parameters.followLineMode;
    if (pathNumber == 1){
        endN = lineStartingPoint1[0];
        endE = lineStartingPoint1[1];
        endAlt = lineStartingPoint1[2];
    }else{
        endN = lineStartingPoint2[0];
        endE = lineStartingPoint2[1];
        endAlt = lineStartingPoint2[2];
    }
    sectionIndex =1;
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
float findDist3D(float x1,float x2, float y1, float y2, float z1, float z2){
    float dist;
    dist = sqrtf(pow((x1-x2),2)+pow((y1-y2),2)+pow((z1-z2),2));
    return dist;
}

/**
 * @brief mergeLineByEqn: it updates the next point the aircraft should head to.
 * @param dT: the distance between the point that intersects with plane and the point.
 */
void mergeLineByEqn(float nCoeff, float eCoeff, float altCoeff)
{
    // defining a line
    float pointParamOnPlane=(nCoeff*(position_N - originN) + eCoeff*(position_E - originE) + altCoeff*(-position_D_gps - originAlt))/float(pow(nCoeff,2.0f)+pow(eCoeff,2.0f)+pow(altCoeff,2.0f));

    //Determine how far down line you want to go
    float dT;
    // Need to get pointParamOnPlane
    float nDist = originN + nCoeff*pointParamOnPlane - position_N;
    float eDist = originE + eCoeff*pointParamOnPlane - position_E;
    //float dDist = originAlt + altCoeff*pointParamOnPlane - (-position_D_gps);
    float distBtwPlanes = sqrtf(pow(nDist,2) + pow(eDist,2));
    low_data.distBtwPlanes = distBtwPlanes;

    dT = 1/(distBtwPlanes+aal_parameters.manualDT);
    low_data.dT = dT;

    low_data.next_N = originN + nCoeff*(pointParamOnPlane + dT);
    low_data.next_E = originE + eCoeff*(pointParamOnPlane + dT);
    low_data.next_Alt = originAlt + altCoeff*(pointParamOnPlane + dT);
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
float* calculateLineCoeffs(float x1, float x2, float y1, float y2, float z1, float z2){
    float coeffArray[3];
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    float dist = findDist3D(x1,x2,y1,y2,z1,z2);
    coeffArray[0] = dx/dist;
    coeffArray[1] = dy/dist;
    coeffArray[2] = dz/dist;
    return coeffArray;
}

void missionControl(sectionIndex){
    if(transitionLine(sectionIndex)){
        sectionIndex ++;
        if(isLinePt1[sectionIndex-1]){
            updateLineParameters();
            mergeLineByEqn(nCoeff,eCoeff,altCoeff);
        }
    }else{
        if(isLinePt1[sectionIndex - 1]){
            mergeLineByEqn(nCoeff,eCoeff,altCoeff);
        }
    }
}


void updateLineParameters(){
    float * coeffArray;
    float n1, n2, e1, e2, d1,d2;
    if (pathNumber == 1){
        n1 = lineStartingPoint1[(sectionIndex-1)*3];
        e1 = lineStartingPoint1[(sectionIndex-1)*3 + 1];
        d1 = lineStartingPoint1[(sectionIndex-1)*3 + 2];
        n2 = lineStartingPoint1[sectionIndex*3];
        e2 = lineStartingPoint1[sectionIndex*3+1];
        d2 = lineStartingPoint1[sectionIndex*3+2];
    }else{
        n1 = lineStartingPoint2[(sectionIndex-1)*3];
        e1 = lineStartingPoint2[(sectionIndex-1)*3 + 1];
        d1 = lineStartingPoint2[(sectionIndex-1)*3 + 2];
        n2 = lineStartingPoint2[sectionIndex*3];
        e2 = lineStartingPoint2[sectionIndex*3+1];
        d2 = lineStartingPoint2[sectionIndex*3+2];
    }
    originN = n1;
    originE = e1;
    originAlt =d1;
    coeffArray = calculateLineCoeffs(x1,x2,y1,y2,z1,z2);
    nCoeff = coeffArray[0];
    eCoeff = coeffArray[1];
    altCoeff = coeffArray[2];
}

void low_loop()
{
    if(timestamp - previous_loop_timestamp >= 500000.0f)
    {
        low_data.race_complete = 0.0f;
        low_data.line_index = 0;
        initializeParameters();
    }
    if(aal_parameters.followLineMode > 0){
        missionControl(sectionIndex);
    }
}


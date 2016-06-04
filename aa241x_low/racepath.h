#ifndef RACEPATH_H
#define RACEPATH_H

/***************** Line following arrays *************************/
//Lake Lag Test 45
int n_numOfPts1 = 1;
float lineStartingPoint1[] = {
    20.0,140.0,3.5,              -50.0,70.0,3.5,
    -50.0,70.0,3.5,              -50.0,-30.0,3.5
};
int isLinePt1[] = {
	1,
    1
};

//Lake Lag Test 15
int n_numOfPts2 = 2;
float lineStartingPoint2[] = {
    20.0,140.0,3.5,              -5.9,43.4,3.5,
    -5.9,43.4,3.5,              -5.9,-60.0,3.5
};
int isLinePt2[] = {
	1,
    1
};

//Line Durand
int n_numOfPts3 = 3;
float lineStartingPoint3[] = {
    477.0,295.0,10.5,            476.0,295.0,10.5,      
    471.0,295.0,10.5,            471.0,295.0,10.5,
    471.0,295.0,10.5,            500.0,312.0,20.0
};
int isLinePt3[] = {
    1,
    0,
    1
};

//Full Mission
int n_numOfPts4 = 5;
float lineStartingPoint4[] = {
    150.00,-96.00,40.00,    65.76,131.93,40.00,
    47.00,125.00,40.00,     47.00,125.00,40.00,
    30.62,136.47,40.00,     -109.38,-63.53,40.00,
    -93.00,-75.00,40.00,    -93.00,-75.00,40.00,
    -93.08,-95.00,40.00,    150.00,-96.00,40.00
};
int isLinePt4[] = {
    1,
    0,
    1,
    0,
    1
};
int isLinePtMission[] = {
    1,1,1,1,1,
    1,0,1,0,1,
    1,1,1
};
int isLinePt6[] = {
    1,
    0,
    1
};

/***************** End of Line Following arrays ******************/
#endif

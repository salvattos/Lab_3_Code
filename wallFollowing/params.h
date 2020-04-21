#ifndef __PARAMS_H
#define __PARAMS_H

//PID gains -- you can edit the defaults later, after you tune the motors
float Kp = 15;
float Ki = 1.5;
int bufferCount[100] = {0};
int bufferCountRight[100] = {0};
int bufferIndexRight = 0;

int bufferCountLeft[100] = {0};
int bufferIndexLeft = 0;

float Kd = 0;


float distKp = 12;
float distKi = 1.2;
int errorIndex = 0;
static int distSum = 0;
int errorCount[100] = {0};

/*
 * target wheel speeds; these are in encoder ticks per PID loop!
 * 
 * Even though ticks/loop are always reported in integer number of ticks
 * we use a float here so that the target can be fractional. In practice, 
 * the fraction will cause the error to be sometimes a fraction positive
 * and sometimes a fraction negative, but they'll all wash out in the end.
 */

static int leftTarget = 0;
static int rightTarget = 0;
const int baseRight = 30;
const int baseLeft = 30;

int targetLeft = 50;
int targetRight = 50;

int targetDist = 30; // distance fomr wall in cm
static float currentDist = 0; //current distance form wall
static int offset = 0;


#endif

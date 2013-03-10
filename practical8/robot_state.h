#ifndef _STATE_H
#define _STATE_H

#include "space.h"
#include "conf.txt"

// global encodings target...
int gl_encodings = 0;

typedef struct {
	Point position;

} State;


void stop()
{
	motor[LEFT_WHEEL] = 0;
	motor[RIGHT_WHEEL] = 0;
  wait1Msec(100);
}

void state_init() {
	nMotorEncoder[SONAR_M] = 0;

	nMotorPIDSpeedCtrl[SONAR_M] = mtrSpeedReg;
}
// specify the rotation direction with 1 or -1
void sonar_set_rotation(int dir, int speed) {
	motor[motorB] = speed * dir;
}

void sonar_stop() {motor[SONAR_M] = 0;}

void sonar_rotate_encod_sync(int encodings) {
	nMotorEncoderTarget[SONAR_M] = encodings;
	//int delta = encodings - nMotorEncoder[SONAR_M];
	int delta = encodings;
	sonar_set_rotation(delta / abs(delta), false);
	while(nMotorRunState[SONAR_M] != runStateIdle)  {
		//matah
	}
	sonar_stop();
}
task rotate_encod() {
	sonar_rotate_encod_sync(gl_encodings);
}

/* this is async */
void sonar_rotate_encod(int encodings) {
	gl_encodings = encodings;
	StartTask(rotate_encod);
}

void rotate(int degs) {
	int dir = 1;
	if (degs < 0) {
		dir = -1;
		degs = -degs;
	}
	// Inverse the motors speed

  motor[LEFT_WHEEL] 	= dir * ROTATE_POWER;
  motor[RIGHT_WHEEL] 	= -dir * ROTATE_POWER;
  wait1Msec(degs * 1.0 / 90 * ROTATE_90_TIME);
  stop();
}

void droid_rotate(int degs) {
	rotate(degs);
}


void droid_move(int power) {
	motor[LEFT_WHEEL] = power;
	motor[RIGHT_WHEEL] = power;
}


#endif

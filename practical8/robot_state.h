#ifndef _STATE_H
#define _STATE_H

#include "space.h"
#include "conf.txt"

// global encodings target...
int gl_encodings = 0;
bool gl_fine_tune = false;
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


/* SONAR MOVEMENT FUNCTIONS */
void sonar_fix_at_sync(int encodings_value, bool run_slow) {
	int current = nMotorEncoder[SONAR_M];
	int rotation_dir = encodings_value - current;
	rotation_dir = rotation_dir / abs(rotation_dir);
	if (run_slow)
  	sonar_set_rotation(rotation_dir, SONAR_SPEED_SLOW);
 	else
 		sonar_set_rotation(rotation_dir, SONAR_SPEED_FAST);
	while (nMotorEncoder[SONAR_M] != encodings_value) {}
	sonar_stop();
}

//public
void sonar_move_at_sync(int encodings, bool fine_tune) {
	sonar_fix_at_sync(encodings, false);
	if (fine_tune) {
	  wait1Msec(100);
		sonar_fix_at_sync(encodings, true);
	}
}

task move_at_task() {
	sonar_move_at_sync(gl_encodings, gl_fine_tune);
	gl_encodings = 0;
	gl_fine_tune = false;
}

void sonar_move_at_async(int encodings_value, bool fine_tune) {
	gl_encodings = encodings_value;
	gl_fine_tune = fine_tune;
	StartTask(move_at_task);
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

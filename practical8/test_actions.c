
#pragma config(Sensor, S3,     sonar,          sensorSONAR)
#include "actions.h"

task main()
{
	int value = 1;
	wait1Msec(1000);
	nxtDisplayString(1, "Sonar : %d", OUTSIDE_OF_CUBICLE_THRESH);

	state_init();
	//sonar_rotate_encod(360);
	//wait1Msec(6000);
	//find_cubicle_exit();

	bFloatDuringInactiveMotorPWM = false;
	int left = 0;
	int right = 0;
	get_left_right(&left, &right);
	writeDebugStream("left right encoder: %d %d\n", left, right );
	sonar_move_at_sync(0, true);
	/*
	sonar_rotate_encod_sync(90);
	*/


	/*
	sonar_move_at_sync(-45,false);
	writeDebugStream("Immediate sonar motor encod: %d\n", nMotorEncoder[SONAR_M]);
	wait1Msec(1000);
	writeDebugStream("sonar motor encod: %d\n", nMotorEncoder[SONAR_M]);

	sonar_move_at_sync(45,false);
	wait1Msec(1000);
	writeDebugStream("sonar motor encod: %d\n", nMotorEncoder[SONAR_M]);
	writeDebugStream("Correcting the sonar to be into position");
  sonar_fix_at_sync(45,true);
  */

	wait1Msec(10000);
	//droid_rotate(90);
}

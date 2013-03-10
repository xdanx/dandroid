#pragma config(Sensor, S3,     sonar,          sensorSONAR)
#include "actions.h"
#include "robot_state.h"
task main()
{
	writeDebugStream("-----------started big main----------\n");

  state_init();
	bFloatDuringInactiveMotorPWM = false;

	int cub = escape_cubicle();

	if (cub == 1) {
		// 1 -> 2 -> 3 -> 1

	} else if (cub == 2) {
		// 2 -> 1 -> 3 -> 2
		droid_rotate(-90);
		// get in wall follow range \
		// wall follow
		//follow_wall(21,

	} else if (cub == 3) {
		// 3 -> 2 -> 1 -> 3

	} else {
		// better not get here
		stop();
	}
}

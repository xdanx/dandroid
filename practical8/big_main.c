#pragma config(Sensor, S3,     sonar,          sensorSONAR)
#include "actions.h"
#include "robot_state.h"
task main()
{
	writeDebugStream("-----------started big bothar fucker----------\n");

  state_init();
	bFloatDuringInactiveMotorPWM = false;

	int cub = escape_cubicle();



	if (cub == 1) {
		// 1 -> 2 -> 3 -> 1

		writeDebugStream("I just left cubicle 1\n");
		// Rotate 90 to right
		droid_rotate(90);

		// Fine move the sonar to the left
		sonar_move_at_async(90,true);
		// get to wall follow range.
		droid_move(DEFAULT_POWER);
		wait1Msec(1500);
		// Wall follow from 1 to 2
		follow_wall(21, 1, false);
		enter_cubicle(1);
		// exit cube
		droid_exit_cube();
		droid_rotate(-90);

		// Fine move the sonar to the left
		sonar_move_at_async(90, true);
		// get to wall follow range.
		droid_move(DEFAULT_POWER);
		wait1Msec(1500);
		// Wall follow from 2 to 3
		follow_wall(21, 1, false);
		enter_cubicle(1);
		// exit cube
		droid_exit_cube();
		droid_rotate(90);

		// Fine move the sonar to the right
		sonar_move_at_async(-90,true);
		// get to wall follow range.
		droid_move(DEFAULT_POWER);
		wait1Msec(1500);

		// Wall follow from 3 to 1
		follow_wall(21, -1, true);
		enter_cubicle(-1);
		// We're done!

	} else if (cub == 2) {
		writeDebugStream("I just left cubicle 2\n");
		// 2 -> 1 -> 3 -> 2

		// Rotate 90 to right
		droid_rotate(-90);
		// Fine move the sonar to the right
		sonar_move_at_async(-90,true)
		// get to wall follow range.
		droid_move(DEFAULT_POWER);
		wait1Msec(1500);
		// Wall follow from 2 to 1
		follow_wall(21,-1, false);
		enter_cubicle(-1);
		// exit cube
		droid_exit_cube();
		droid_rotate(-90);

		// Fine move the sonar back to left
		sonar_move_at_async(90,true);
		// get to wall follow range.
		droid_move(DEFAULT_POWER);
		wait1Msec(1500);
		// Wall follow from 1 to 3
		follow_wall(21,1,true);
		enter_cubicle(1);
		// exit cube
		droid_exit_cube();
		droid_rotate(90);



		// Fine move the sonar back to left
		sonar_move_at_async(-90,true);
		// get to wall follow range.
		droid_move(DEFAULT_POWER);
		wait1Msec(1500);
		// Wall follow from 3 to 2
		follow_wall(21, -1, false);
		enter_cubicle(-1);
		// We're done!


	} else if (cub == 3) {
		// 3 -> 2 -> 1 -> 3

		// Rotate 90 to right
		droid_rotate(-90);

		// Fine move the sonar to the right
		sonar_move_at_async(-90,true);
		// get to wall follow range.
		droid_move(DEFAULT_POWER);
		wait1Msec(1500);
		// Wall follow from 3 to 2
		follow_wall(21,-1, false);
		enter_cubicle(-1);
		// exit cube
		droid_exit_cube();
		droid_rotate(90);


		// Fine move the sonar to the right
		sonar_move_at_async(-90,true);
		// get to wall follow range.
		droid_move(DEFAULT_POWER);
		wait1Msec(1500);
		// Wall follow from 2 to 1
		follow_wall(21,-1,false);
		enter_cubicle(-1);
		// exit cube
		droid_exit_cube();
		droid_rotate(-90);

		// Fine move the sonar to the right
		sonar_move_at_async(90,true);
		// get to wall follow range.
		droid_move(DEFAULT_POWER);
		wait1Msec(1500);
		// Wall follow from 1 to 3
		follow_wall(21, 1, true);
		enter_cubicle(1);
		// We're done!

	} else {
		// better not get here
		writeDebugStream("UNREACHABLE CODE REACHED\n");
		StopAllTasks();
	}
}

#ifndef _ACTIONS_H
#define _ACTIONS_H

#include "conf.txt"
#include "space.h"
#include "robot_state.h"
#include "logic.h"
#include "test.c"

// distances

#define WAYPOINT_TO_OUTER_WALL 63
#define OUTSIDE_OF_CUBICLE_THRESH 33  // when sonar reports this i means robot is looking outside the cubicle; diag is 29.69
#define TUNNEL_WIDTH 42

//struct Wall { int Ax, Ay, Bx, By};

int read_sonar() {
		return SensorValue[sonar];
}

void find_cubicle_exit() {
	// rotate sonar till you look outside
	// TODO: special case: iullf it points outside but then looks at inside of crap; just make it turn back
	int rotation_dir = 1;
	sonar_set_rotation(rotation_dir, SONAR_SPEED_SLOW);
	while (true) {
		//wait1Msec(30);
		if (read_sonar() > OUTSIDE_OF_CUBICLE_THRESH) break;
	}
	sonar_stop();


	int prevReading = read_sonar();
	sonar_set_rotation(rotation_dir, SONAR_SPEED_SLOW);
	bool dec_started = false; // distance decrease started

  // rotate sonar till you minimize outside distance
  while (true) {
		//wait1Msec(30);
		int currentReading = read_sonar();
		if (!dec_started) {
			if (currentReading < 	prevReading) dec_started = true;
		} else {
			// when your getting further away from that reading
			if (currentReading > 	prevReading) break;
	  }
		prevReading = currentReading;
	}
	writeDebugStream("find cubicle exit: encodings: %d\n", nMotorEncoder[SONAR_M]);
	sonar_stop();
	rotation_dir = -1;
	sonar_set_rotation(rotation_dir, SONAR_SPEED_SLOW);

	wait1Msec(200);

	/*
	while (true) { // try to get back to the smallest reading
		//wait1Msec(30);
		if(read_sonar() == prevReading) break;
	}
	*/

	sonar_stop();

	wait1Msec(100);
	int rot_degs = nMotorEncoder[SONAR_M];
	writeDebugStream("Find cubicle exit: Rotating for %d\n",rot_degs);

	// rotate robot in the same direction and reposition the sonar
	sonar_move_at_async(0, true); // in parallel
	droid_rotate(rot_degs); //sync

	//wait1Msec(10000);
}

// PRE: sonar is at 0 encodings
void step_out_of_cubicle() {
	droid_move(30);
	// perhaps here already start looking left or right to see where you are by the time you reach the middle
	while (true) {
		wait1Msec(30);
		int reading = read_sonar();
		if (reading < TUNNEL_WIDTH  / 2) break; // you are out in the tunnel
	}
}



// PRE: sonar pointing forward
void get_left_right(int* left, int* right) {
	sonar_move_at_sync(90, false); //this turns it to the right
	nxtDisplayString(1, "Rotation : %d \n", nMotorEncoder[SONAR_M]);
	wait1Msec(100);
	*right = read_sonar();
	sonar_move_at_sync(-90, false); //this turns it to the left
	nxtDisplayString(1, "Rotation : %d \n", nMotorEncoder[SONAR_M]);
	wait1Msec(100);
	*left = read_sonar();
}

int escape_cubicle() {
	writeDebugStream("---- Entering escape cubicle ---- \n");
	find_cubicle_exit();
	// sonar MUST be at 0 at this point
	step_out_of_cubicle();
	// still moving at this point !!

	stop();
	int left, right;
	get_left_right(&left, &right);
	int cub = get_cubicle_index(left, right);

	// look forward quickly and stop so you don't bump into the wall !!!
	sonar_move_at_sync(0, true);
	//while (read_sonar() > TUNNEL_WIDTH / 2) {/*wait for it ...*/}
	writeDebugStream("the cub is %d\n", cub);
	stop();

	return cub;
	//droid_rotate(90); //sync
}




#endif

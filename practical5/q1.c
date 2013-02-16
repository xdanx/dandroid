#include "conf.txt"
#include "particleDataStructures.c"
#include "sample.h"

/* Variables used in our program */

int robotState = MOVE_STATE;
int leftEncodings = 0;
int rightEncodings = 0;


typedef struct {
	float x;
	float y;
	float angle; // -pi < angle < pi, 0 when robot pointed towards x+
} Position;

Position position;

/* End variables used in our program */
/* Functions used in our program */

void stop()
{
	motor[LEFT_WHEEL] = 0;
	motor[RIGHT_WHEEL] = 0;
  wait1Msec(1000);
}

/* move time in millis based on K constant */
float get_move_time(float power, int distance) {
	return  (distance * 1.0) / (K * power);
}

void move_forward(int power, float distance) {
	robotState = MOVE_STATE;

	motor[LEFT_WHEEL] = power;
	motor[RIGHT_WHEEL] = power;
	float time = get_move_time(power, distance);
	wait1Msec(time);
	stop();
}

/* Debugging variables */
void clear_debug_stats()
{
	nMotorEncoder[LEFT_WHEEL] = 0;
	nMotorEncoder[RIGHT_WHEEL] = 0;
}

void print_debug_stats()
{
	writeDebugStream("LEFT_WHEEL encoder: %d\n", nMotorEncoder[LEFT_WHEEL] );
	writeDebugStream("RIGHT_WHEEL encoder: %d\n", nMotorEncoder[RIGHT_WHEEL] );
	//clearDebugStats();
}
/* End debugging variables */

/* Distance related functions  */
void position_add_distance(Position* p, float distance) {

	p->x += distance * cosDegrees(p->angle);
	p->y += distance * sinDegrees(p->angle);
	return;
}

void position_add_angle(Position* p, float deltaAngle) {
	const float upper = 180;
	const float lower = -180;


	p->angle += deltaAngle;
	if (p->angle > upper) p->angle -= 360;
	if (p->angle < lower) p->angle += 360;
}

void rotate(int degs) {
	robotState = ROTATE_STATE;
	int dir = 1;
	if (degs < 0) {
		dir = -1;
		degs = -degs;
	}

  motor[LEFT_WHEEL] 	= dir * ROTATE_POWER;
  motor[RIGHT_WHEEL] 	= -dir * ROTATE_POWER;
  wait1Msec(degs * 1.0 / 90 * ROTATE_90_TIME);
  stop();
}


/* functions related to points */

void print_10_points()
{
	int i;
	for (i=0; i<10; ++i)
		writeDebugStream("x:%f y:%f theta:%f\n",xArray[i], yArray[i], thetaArray[i]);
	writeDebugStream("-------------\n");
}

void points_update(Position* p, float distance, int state) {
	int i;
	float e, f;
	switch (state) {
		case MOVE_STATE:
			for(i=0; i<NUMBER_OF_PARTICLES; i++) {
				e = sampleGaussian(0.0, 0.881);
				f = sampleGaussian(0.0, 0.881);
				//writeDebugStream("Our random vars are: e=%f f=%f\n",e,f);
				xArray[i] = p->x + (distance + e) * cosDegrees(p->angle);
				yArray[i] = p->y + (distance + e) * cosDegrees(p->angle);

				thetaArray[i] = p->angle + f;
			}
			print_10_points();
			break;
		case ROTATE_STATE:
			print_10_points();
			for(i=0; i<NUMBER_OF_PARTICLES; i++) {
				e = sampleGaussian(0.0, 0.881);
				// Theta is added 90 degrees when is updated with update_position
				// we just add the error here.
				thetaArray[i] = p->angle + e;
			}
			print_10_points();
			break;
	}
}



/* End functions related to points */
/* Start task functions */

task vehicle_draw_position() {
	while (true) {
		wait1Msec(10);
		nxtSetPixel(15 + (int)(position.x), 15 + (int)(position.y));
	}
}

task vehicle_compute_position() {
	while (true) {
		wait1Msec(10);
		int curLeft = nMotorEncoder[LEFT_WHEEL];
		int curRight = nMotorEncoder[RIGHT_WHEEL];
		int deltaLeft = curLeft - leftEncodings;
		int deltaRight = curRight - rightEncodings;

		switch (robotState) {
			case MOVE_STATE:
				position_add_distance(&position, deltaLeft / ENCODINGS_PER_CM);
				break;
			case ROTATE_STATE:
			  // The angle will be added by the rotate function
				//addAngle(&position, deltaLeft / ENCODINGS_PER_DEGREE);
				break;
		}

		leftEncodings = curLeft;
		rightEncodings = curRight;
	}
}

/* End tasks */

task main() {

	clearDebugStream();
	position.x = 0;
	position.y = 0;
	position.angle = 0;
	nMotorEncoder[LEFT_WHEEL] = 0;
	nMotorEncoder[RIGHT_WHEEL] = 0;

	StartTask(vehicle_compute_position);
	StartTask(vehicle_draw_position);

	int i,j;
	int loops = 4;
	int segments = 4;
	int milage = 0;

	for (i=0 ;i < loops; ++i) {

		for (j=0; j < segments; ++j)
		{
			move_forward(MOVE_POWER,10);
			eraseDisplay();
			// The robot moved to the new segment
			points_update(position, milage, MOVE_STATE);
			drawParticles();

			milage += 10;
		}

	  rotate(90);
	  position_add_angle(position, 90);
	  points_update(position, milage, ROTATE_STATE);
  }

  StopTask(vehicle_compute_position);
  StopTask(vehicle_draw_position);

  wait10Msec(6000); // wait 1MIN


}

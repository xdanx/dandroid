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

	p->angle += deltaAngle;
	if (p->angle > THETA_UPPER) p->angle -= 360;
	if (p->angle < THETA_LOWER) p->angle += 360;
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
				writeDebugStream("Our random vars are: e=%f f=%f\n",e,f);
				xArray[i] = xArray[i] + (distance + e) * cosDegrees(thetaArray[i]);
				yArray[i] = yArray[i] + (distance + e) * sinDegrees(thetaArray[i]);
				thetaArray[i] = thetaArray[i] + f;
			}
			print_10_points();
			break;
		case ROTATE_STATE:
			print_10_points();
			for( i=0; i<NUMBER_OF_PARTICLES; i++) {

				e = sampleGaussian(0.0, 0.881);
				thetaArray[i] += 90 + e;
				if (thetaArray[i] > THETA_UPPER) thetaArray[i] -= 360;
				if (thetaArray[i] < THETA_LOWER) thetaArray[i] += 360;
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
		nxtSetPixel(PRINT_OFFSET_X + (int)(position.x), PRINT_OFFSET_Y + (int)(position.y));
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

	// Initial points update
	points_update(position, milage, MOVE_STATE);
	drawParticles();
	for (i=0 ;i < loops; ++i) {

		for (j=0; j < segments; ++j)
		{
			//eraseDisplay();
			milage = 10;
			move_forward(MOVE_POWER, milage);

			// The robot moved to the new segment
			points_update(position, milage, MOVE_STATE);
			drawParticles();

		}

	  rotate(90);
	  position_add_angle(position, 90);
	  points_update(position, milage, ROTATE_STATE);
  }

  StopTask(vehicle_compute_position);
  StopTask(vehicle_draw_position);

  wait10Msec(6000); // wait 1MIN


}

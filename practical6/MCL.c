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
//*
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

void move_for_duration(int power, int time) {
	motor[LEFT_WHEEL] = power;
	motor[RIGHT_WHEEL] = power;
	wait1Msec(time);
	stop();
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

float normalize_angle_value(float angle)
{
	if (angle > THETA_UPPER)
		return angle - THETA_NORMALIZER;
	if (angle < THETA_LOWER)
		return angle + THETA_NORMALIZER;

	return angle;
}


void position_add_distance(Position* p, float distance) {

	p->x += distance * cosDegrees(p->angle);
	p->y += distance * sinDegrees(p->angle);
	return;
}

void position_add_angle(Position* p, float deltaAngle) {

	//p->angle += deltaAngle;
	writeDebugStream("Position_add_angle: Pos angle:%f adding angle:%f\n",p->angle, deltaAngle);
	p->angle = normalize_angle_value(p->angle + deltaAngle;
	writeDebugStream("Position_add_angle: Pos angle:%f\n",p->angle);
}

void rotate(float degs) {
	robotState = ROTATE_STATE;

	/* Might go wrong, opposite to what we have in our debug msg */
	int dir = (degs >= 0) - (degs < 0);
  motor[LEFT_WHEEL] 	= dir * ROTATE_POWER;
  motor[RIGHT_WHEEL] 	= -dir * ROTATE_POWER;
  wait1Msec(abs(degs) / 90 * ROTATE_90_TIME);
 	// End here

  position_add_angle(position, degs);
  stop();
}



/* functions related to points */

void print_10_points()
{
	int i;
	for (i=0; i<10; ++i)
		writeDebugStream("x:%f y:%f theta:%f cos:%f, sin: %f\n",xArray[i], yArray[i], thetaArray[i],
																	cosDegrees(thetaArray[i]), sinDegrees(thetaArray[i]));
	writeDebugStream("-------------\n");
}

void points_update(float value, int state) {
	// Value is distance in cm when state is MOVE_STATE
	//and degrees when state is ROTATE_STATE
	int i;
	float e, f;

	switch (state) {
		case MOVE_STATE:
			for(i=0; i<NUMBER_OF_PARTICLES; i++) {
				e = sampleGaussian(0.0, 0.881);
				f = sampleGaussian(0.0, 0.881);
				//writeDebugStream("Our random vars are: e=%f f=%f\n",e,f);
				xArray[i] = xArray[i] + (value + e) * cosDegrees(thetaArray[i]);
				yArray[i] = yArray[i] + (value + e) * sinDegrees(thetaArray[i]);
				thetaArray[i] = thetaArray[i] + f;
			}
			break;
		case ROTATE_STATE:
			for( i=0; i<NUMBER_OF_PARTICLES; i++) {

				e = sampleGaussian(0.0, 0.881);
				//thetaArray[i] += value + e;
				thetaArray[i] = normalize_angle_value(thetaArray[i] + value + e);
			}
			break;
	}
}

void navigate_to_waypoint(float x, float y)
{
	float med_x = 0, med_y = 0, med_theta = 0;
	float weight = 1,i;

	// estimate current posistion
	for (i=0; i < NUMBER_OF_PARTICLES; ++i)
	{
		// Next time we will use , now it is just hard-coded.
		// med_x += weightArray[i] * xArray[i];
		med_x += xArray[i];
		med_y += yArray[i];
		med_theta += thetaArray[i];
	}
	// Now we need to get the correct value
	med_x = med_x / NUMBER_OF_PARTICLES;
	med_y = med_y / NUMBER_OF_PARTICLES;
	med_theta = med_theta / NUMBER_OF_PARTICLES;

	writeDebugStream("Averages: x: %f y:%f theta:%f\n", med_x, med_y, med_theta);

	// calculate difference
	float dif_x = x - med_x; // dest - curr_pos
	float dif_y = y - med_y;

	writeDebugStream("Dif_x: %f, dif_y: %f\n",dif_x, dif_y);
	float rotate_degs;// = atan(dif_y / dif_x) * 180.0 / PI; // get the nr of degrees we want to turn. But in which direction ?!

	// If both are negative, then we need to add the - to the rotate_degrees
	if ( dif_x > 0)
		rotate_degs = atan (dif_y / dif_x);
	else if (dif_y >=0 && dif_x < 0)
		rotate_degs = atan( dif_y / dif_x) + PI;
	else if (dif_y < 0 && dif_x < 0)
		rotate_degs = atan (dif_y / dif_x) - PI;
	else if (dif_y > 0 && dif_x == 0)
		rotate_degs = PI;
	else if (dif_y < 0 && dif_x == 0)
		rotate_degs = -PI;
	else
		rotate_degs = 0;

	rotate_degs = rotate_degs * 180.0 / PI;
	rotate_degs = normalize_angle_value(rotate_degs - med_theta);
	writeDebugStream("Rotate angle: %f\n",rotate_degs);

	// Rotate towards the correct position
	rotate(rotate_degs);
	points_update(rotate_degs, ROTATE_STATE);

	// move forward
	float move_distance = sqrt(dif_x * dif_x + dif_y * dif_y);
	writeDebugStream("Moving distance: %f\n", move_distance);
	move_forward(MOVE_POWER, move_distance);

	// Update the position to the new points
	points_update(move_distance, MOVE_STATE);
}

/* End functions related to points */
/* Start debug functions */
// Print the last 10 cumulative weight arrays
void print_10_cwa()
{
	writeDebugStream("Printing weights...\n");
	int i;
	for (i=NUMBER_OF_PARTICLES-11; i<NUMBER_OF_PARTICLES; ++i)
		writeDebugStream("wa[%d]: %f - cwa[%d]: %f",i,weightArray[i], i,cumulativeWeightArray[i]);
	writeDebugStream("-------------\n");
}
/* End debug functions */
/* Start task functions */

task vehicle_draw_position() {
	while (true) {
		wait1Msec(10);
		nxtSetPixel(20 + (int)(position.x / DISPLAY_SCALE), PRINT_OFFSET_Y + 20 + (int)(position.y / DISPLAY_SCALE));
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


float calculate_likelihood(float x, float y, float theta, float z)
{
	int i = 0, mind = 0;
	//find wall intersection
	for (i = 0; i < NUMBER_OF_WALLS; i++)
	{


	}

}


void calculate_cumulative_array()
{
	int i,j;
	// Creating the cumulative Weight Array
	for (i=0; i<NUMBER_OF_PARTICLES; ++i)
		for(j=0; j<= i; ++j)
			cumulativeWeightArray[i] += weightArray[j];


	print_10_cwa();


}

void normalise_weight_array ()
{
	print_10_cwa ();

	int i;
	float sum = 0;

	for (i = 0; i < NUMBER_OF_PARTICLES; ++i)
		sum += weightArray[i];

	for (i = 0; i < NUMBER_OF_PARTICLES; ++i)
		weightArray[i] /= sum;

	print_10_cwa ();
}

int binary_search (float target)
{
	int begin = 0;
	int end = NUMBER_OF_PARTICLES - 1;
	int mid;

	while (begin <= end)
	{
		mid = (begin + end) / 2;
		if (target > cumulativeWeightArray[mid])
			begin = mid + 1;
		else if (target < cumulativeWeightArray[mid])
			end = mid - 1;
		else
			return mid;
	}

	return begin;
}

void resample()
{
	// Normalise weight array
	normalise_weight_array ();

	// Calculate cumulative array
	calculate_cumulative_array ();

	// Reset weights to 1/N
	int i;
	for (i = 0; i < NUMBER_OF_PARTICLES; ++i)
		weightArray[i] = 1/NUMBER_OF_PARTICLES;

	// Resample
	for (i = 0; i < NUMBER_OF_PARTICLES; ++i)
	{
		// Generate random number between 0 and 1
		unsigned int random100 = rand() % 100;
		float random1 = random100 / 100;

		// Search value of random1 in cumulativeWeightArray
		int index = binary_search (random1);

		// Copy new particle
		xArrayCopy[index] = xArray[index];
		yArrayCopy[index] = yArray[index];
	  thetaArrayCopy[index] = thetaArray[index];
	}

	// Update spacial distribution
	for (i = 0; i < NUMBER_OF_PARTICLES; ++i)
	{
		xArray[i] = xArrayCopy[i];
		yArray[i] = yArrayCopy[i];
		thetaArray[i] = thetaArrayCopy[i];
	}
}


task main() {

	clearDebugStream();
	position.x = 0;
	position.y = 0;
	position.angle = 0;
	nMotorEncoder[LEFT_WHEEL] = 0;
	nMotorEncoder[RIGHT_WHEEL] = 0;

	StartTask(vehicle_compute_position);
	StartTask(vehicle_draw_position);

	navigate_to_waypoint(50,50);
	navigate_to_waypoint(50,-20);
	navigate_to_waypoint(0,0);

  StopTask(vehicle_compute_position);
  StopTask(vehicle_draw_position);

  wait10Msec(60000); // wait 1MIN

}

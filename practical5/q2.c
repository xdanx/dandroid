#include "conf.txt"
#include "particleDataStructures.c"
#include "sample.h"

/* Variables used in our program */

bool synch = true;
int robotState = MOVE_STATE;
int leftEncodings = 0;
int rightEncodings = 0;

int milage = 0;

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
float getMoveTime(float power, int distance) {
	return  (distance * 1.0) / (K * power);
}


void move(int power, float distance) {
	robotState = MOVE_STATE;
	motor[LEFT_WHEEL] = power;
	motor[RIGHT_WHEEL] = power;
	float time = getMoveTime(power, distance);
	wait1Msec(time);
	stop();
}

void shoot()
{
	motor[motorA] = 0;
  motor[motorC] = 0;
  motor[motorB] = 100;
  wait1Msec(4000);
}


void clearDebugStats()
{
	nMotorEncoder[LEFT_WHEEL] = 0;
	nMotorEncoder[RIGHT_WHEEL] = 0;
}

void printDebugStats()
{
	writeDebugStream("LEFT_WHEEL encoder: %d\n", nMotorEncoder[LEFT_WHEEL] );
	writeDebugStream("RIGHT_WHEEL encoder: %d\n", nMotorEncoder[RIGHT_WHEEL] );
	//clearDebugStats();
}

void addDistance(Position* p, float distance) {
	int i=0;

	p->x += distance * cosDegrees(p->angle);
	p->y += distance * sinDegrees(p->angle);
	return;
}

void addAngle(Position* p, float deltaAngle) {
	const float upper = 180;
	const float lower = -180;
	float e;
	int i=0;
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

void updateArrays(Position* p, float distance, int state) {
	int i;
	float e;
	switch (state) {
		case MOVE_STATE:
			for(i=0; i<NUMBER_OF_PARTICLES; i++) {
				e = sampleGaussian(0.0, 0.881);
				xArray[i] = p->x + (distance + e) * cosDegrees(p->angle);
				e = sampleGaussian(0.0, 0.881);
				yArray[i] = p->y + (distance+e) * cosDegrees(p->angle);
				e = sampleGaussian(0.0, 0.881);
				thetaArray[i] = p->angle + e;
			}
			break;
		case ROTATE_STATE:
			for(i=0; i<NUMBER_OF_PARTICLES; i++) {
				e = sampleGaussian(0.0, 0.881);
				thetaArray[i] = p->angle + 90 + e;
			}
			break;
	}
}


void print_points()
{
	int mileage = 0, i,j;
	// We are making 4 corners
	for(i=1; i<=4; ++i)
	{
		// Each straight line, we are marking each 4 points ( each 10 cm out of 40)
		for(j=1; j<=4; ++j)
		{
			updateArrays(&position, mileage, MOVE_STATE);
			addDistance(&position, mileage);
			mileage += 10;
			drawParticles();
		}
		addAngle(&position, 90);
		updateArrays(&position, mileage,ROTATE_STATE);
	}

}


task drawPosition() {
	while (true) {
		wait1Msec(10);
		nxtSetPixel(15 + (int)(position.x), 15 + (int)(position.y));
	}
}

task drawScatter() {
	while(true) {
		wait1Msec(1000);
		drawParticles();
	}
}

task main() {
	position.x = 0;
	position.y = 0;
	position.angle = 0;
	nMotorEncoder[LEFT_WHEEL] = 0;
	nMotorEncoder[RIGHT_WHEEL] = 0;

	print_points();

	wait1Msec(90000);
	return;



	/*
	StartTask(computePosition);
	StartTask(drawPosition);
	StartTask(drawScatter);

	int i = 0;
	int loops = 4;
	int endRotation = 0;
	for (i=0 ;i < loops; ++i) {
		move(MOVE_POWER,40);
		printDebugStats();
	  rotate(90);
	  printDebugStats();
  }

  StopTask(computePosition);
  StopTask(drawPosition);
  StopTask(drawScatter);
  wait10Msec(6000); // wait 1MIN
  */

}

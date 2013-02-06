#pragma config(Sensor, S1,     touchRight,        sensorTouch)
#pragma config(Sensor, S2,		 touchLeft, 				sensorTouch)
#pragma config(Sensor, S3,     lightRight,       sensorLightActive)
#pragma config(Sensor, S4,     lightLeft,       sensorLightActive)
/* constants */
int ROTATE_90_TIME = 1210;
int ROTATE_POWER = 25;
int MOVE_POWER = 10;
const int DISPLAY_SCALE = 10;
ee
const float K = 0.00036; // for distance in cm, and time in milliseconds
const float ENCODINGS_PER_CM = 27.5;
const float ENCODINGS_PER_DEGREE = 3;

int LEFT_WHEEL = motorA;
int RIGHT_WHEEL = motorC;
int GUN = motorB;


#define MOVE_STATE 0
#define ROTATE_STATE 1
/* end constants */

// global vars
bool synch = true;
int robotState = MOVE_STATE;
int leftEncodings = 0;
int rightEncodings = 0;
int dir = 0;

typedef struct {
	float x;
	float y;
	float angle; // -pi < angle < pi, 0 when robot pointed towards x+
} Position;

Position position;


void stop() {
	motor[LEFT_WHEEL] = 0;
	motor[RIGHT_WHEEL] = 0;
  // Now motors are synched
	//motor[RIGHT_WHEEL] = 0;
  wait1Msec(2000);
}

/* move time in millis based on K constant */
float getMoveTime(float power, int distance) {
	return  (distance * 1.0) / (K * power);
}

void move(int power, float distance) {
	robotState = MOVE_STATE;
	motor[LEFT_WHEEL] = power;
	motor[RIGHT_WHEEL] = power;
	float time = getMoveTime(abs(power), distance);
	wait1Msec(time);
	stop();
}

void rotate(int degs) {
	robotState = ROTATE_STATE;
	int dir = 1;
	if (degs < 0) {
		dir = -1;
		degs = -degs;
	}
	// Inverse the motors speed

	if (synch) nSyncedTurnRatio = -100;
  motor[LEFT_WHEEL] 	= dir * ROTATE_POWER;
  motor[RIGHT_WHEEL] 	= -dir * ROTATE_POWER;
  wait1Msec(degs * 1.0 / 90 * ROTATE_90_TIME);
  if (synch) nSyncedTurnRatio = 100;

  stop();
}

void shoot()
{
	motor[motorA] = 0;
  motor[motorC] = 0;
  motor[motorB] = 100;
  wait1Msec(4000);
}


void move_k(int speed, int time)
{
	motor[motorA] = speed;
	wait1Msec(time);
	stop();
}


void clearDebugStats()
{
	//nMotorEncoder[LEFT_WHEEL] = 0;
	//nMotorEncoder[RIGHT_WHEEL] = 0;
}

void printDebugStats()
{
	writeDebugStream("LEFT_WHEEL encoder: %d\n", nMotorEncoder[LEFT_WHEEL] );
	writeDebugStream("RIGHT_WHEEL encoder: %d\n", nMotorEncoder[RIGHT_WHEEL] );
	clearDebugStats();
}

void addDistance(Position* p, float distance) {
	p->x += distance * cosDegrees(p->angle);
	p->y += distance * sinDegrees(p->angle);
	return;
}

void addAngle(Position* p, float deltaAngle) {
	const float upper = 180;
	const float lower = -180;
	p->angle += deltaAngle;
	if (p->angle > upper) p->angle -= 360;
	if (p->angle < lower) p->angle += 360;
}

#define TRU true

task avoid_obstacle() {

	stop();
	motor[LEFT_WHEEL] = -25;
	motor[RIGHT_WHEEL] = -25;
	wait1Msec(1000);
	stop();

	if(dir == 0) {
	//hit was center
		rotate(45);

	} else if(dir == 1) {
	//hit was right
		rotate(45);

	} else {
	//hit was left
		rotate(-45);

		}
}

task light_at_the_end_of_the_tunnel() {



}

task move_forever () {
	move(25, 1000000000);
}


task main() {

	//StartTask(move_forever);
	//StartTask(avoid_obstacle);
	while (TRU) {

		if(SensorValue[touchRight] == 1 && SensorValue[touchLeft] == 1){
			dir = 0;
			StopTask(move_forever);
			StartTask(avoid_obstacle);
		}
		if(SensorValue[touchRight] == 1) {
			dir = 1;
			StopTask(move_forever);
			4//StartTask(avoid_obstacle);
		}
		if(SensorValue[touchLeft] == 1) {
			dir = 2;
			StopTask(move_forever);
			StartTask(avoid_obstacle);
		}
		if(TRU) {
				StopTask(avoid_obstacle);
				StartTask(move_forever);
		}
	}
	wait1Msec(1000000000000);

++++++}

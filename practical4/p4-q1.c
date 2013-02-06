#pragma config(Sensor, S1,     touchRight,     sensorTouch)
#pragma config(Sensor, S2,     lightLeft,      sensorLightInactive)
#pragma config(Sensor, S3,     lightRight,     sensorCOLORNONE)
#pragma config(Sensor, S4,     touchLeft,      sensorTouch)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/* constants */
int ROTATE_90_TIME = 1210;
int ROTATE_POWER = 25;
int MOVE_POWER = 10;
const int DISPLAY_SCALE = 10;

const float K = 0.00036; // for distance in cm, and time in milliseconds

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

/* Values read by the sensors */

int bumper_left = 0;
int bumper_right = 0;
int light_left = 0;
int light_right = 0;


/* Starting functions needed for controlling the robot */
void stop() {
	motor[LEFT_WHEEL] = 0;
	motor[RIGHT_WHEEL] = 0;
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

  motor[LEFT_WHEEL] 	= dir * ROTATE_POWER;
  motor[RIGHT_WHEEL] 	= -dir * ROTATE_POWER;
  wait1Msec(degs * 1.0 / 90 * ROTATE_90_TIME);

  stop();
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

/* End functions used to move robots */

#define TRU true

void avoid_obstacle() {

	writeDebugStream("DEBUG: Started task avoid_obstacle\n");


	// Read the values reported by the sensors
	int left_bump = SensorValue[touchLeft];
	int right_bump = SensorValue[touchRight];

	// First of all, emergency stop
	stop();

	if ( left_bump == 1 && right_bump == 1 ) {
		//hit was center
		move(-25,10); // move back 10cm
		rotate(60); // rotate left 60 degrees
		move(25, 20); // go in front
		rotate(-60); // go back into position

	} else if (right_bump == 1) {
		//hit was right
		move(-25,5); // move back 10cm
		rotate(30); // rotate left 30 degrees
		move(25, 10); // go in front
		rotate(-30); // go back into position

	} else {
		//hit was left
		move(-25,5); // move back 10cm
		rotate(-30); // rotate right 30 degrees
		move(25, 10); // go in front
		rotate(30); // go back into position

		}
}



task move_forever () {
	writeDebugStream("DEBUG: Started task move_forever\n");
	move(15, 1000000000);
}

int bumped()
{
	// Return if either left or right bump
	return SensorValue[touchLeft] || SensorValue[touchRight];
}


task main() {


	//StartTask(update_sensors_value);
	StartTask(move_forever);

	while (TRU)
	{
		if ( bumped() )
		{

			// Can either Stop the move forward
			// task or have a higher priority task.
			StopTask(move_forever);
			wait1Msec(200);
			// Start the avoid obstacle turn
			avoid_obstacle();
			// Exit the program
			return;
		}
	}

}

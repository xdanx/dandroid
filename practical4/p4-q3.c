#pragma config(Sensor, S1,     touchRight,     sensorTouch)
#pragma config(Sensor, S2,     lightRight,     sensorLightInactive)
#pragma config(Sensor, S3,     lightLeft,      sensorCOLORNONE)
#pragma config(Sensor, S4,     touchLeft,      sensorTouch)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/* constants */
int ROTATE_90_TIME = 1210;
int ROTATE_POWER = 25;
int MOVE_POWER = 10;

const float K = 0.00036; // for distance in cm, and time in milliseconds

int LEFT_WHEEL = motorA;
int RIGHT_WHEEL = motorC;
int GUN = motorB;


#define MOVE_STATE 0
#define ROTATE_STATE 1
/* end constants */

// global vars

int robotState = MOVE_STATE;
int leftEncodings = 0;
int rightEncodings = 0;
int dir = 0;


/* Starting functions needed for interacting with the robot */
void stop() {
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
	float time = getMoveTime(abs(power), distance);
	wait1Msec(time);
	stop();
}

// This function gets the argument, to see in which direction the
// robot should move, 1 or -1

void rotate(int degs) {
	int dir = 1;
	if (degs < 0) {
		dir = -1;
		degs = -degs;
	}

	// Inverse the motors speed
  motor[LEFT_WHEEL] 	= dir * ROTATE_POWER;
  motor[RIGHT_WHEEL] 	= -dir * ROTATE_POWER;
}

void dbg(char* s)
{
	writeDebugStream("DEBUG: %s\n",s);
}

#define LEFT_LIGHT_CORRECTION 0.8
#define RIGHT_LIGHT_CORRECTION 1.4

int get_left_light_sensor()
{
	// This sensor needs correction to be as the other one
	return SensorValue[lightLeft] * LEFT_LIGHT_CORRECTION;
}
int get_right_light_sensor()
{
	// This sensor is returned as it is
	return SensorValue[lightRight] * RIGHT_LIGHT_CORRECTION;
}

void print_sensors()
{
	writeDebugStream("Left: %d - Right: %d\n", get_left_light_sensor(), get_right_light_sensor());
	return;
}

/* End functions used to interact with the robot */

#define TRU true
#define ROTATE_LEFT 1
#define ROTATE_RIGHT -1

/* Our own functions here */

#define ARRIVED_THRESHOLD 40
#define CENTERED_THRESHOLD 35
#define CAPTURE_THRESHOLD 25

// Goof for next task
/*
int light_arrived()
{
	return ( (SensorValue[lightLeft] - SensorValue[lightRight]) < CENTERED_THRESHOLD &&
				  SensorValue[lightLeft] > ARRIVED_THRESHOLD && SensorValue[lightRight] > ARRIVED_THRESHOLD );

}
*/

int light_centered()
{
	print_sensors();
	return ( ( get_left_light_sensor() - get_right_light_sensor()) < CENTERED_THRESHOLD &&
				  get_left_light_sensor() > CAPTURE_THRESHOLD && get_right_light_sensor() > CAPTURE_THRESHOLD );
}

int light_unbalanced()
{
	return (get_left_light_sensor() > CAPTURE_THRESHOLD || get_right_light_sensor() >  CAPTURE_THRESHOLD );
}


task balance_light()
{
	dbg("Entered balance_light");
	print_sensors();
	// In this task, the light_unbalanced() condition holds
	if ( get_left_light_sensor() > get_right_light_sensor() )
		rotate(ROTATE_LEFT);
	else
		rotate(ROTATE_RIGHT);

}

task find_light()
{
	dbg("Entered find_light");
	rotate(ROTATE_LEFT);
}


task main()
{

	//void *curr_task = null;

	while (TRU)
	{
		//dbg("Starting while loop");
		wait1Msec(1000);
		if ( light_centered() )
		{
			dbg("Entered light centered");
			stop();
			continue;
		}
		else if ( light_unbalanced() )
		{
			// Here one sensor detects light, the other doesn't
			StartTask(balance_light);
		}
		else
		{
			// We are not finding light, just look for a source of light
			StartTask(find_light);
		}
	wait1Msec(1000);
	}


}

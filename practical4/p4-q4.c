#pragma config(Sensor, S1,     touchRight,     sensorTouch)
#pragma config(Sensor, S2,     lightRight,     sensorLightInactive)
#pragma config(Sensor, S3,     lightLeft,      sensorCOLORNONE)
#pragma config(Sensor, S4,     touchLeft,      sensorTouch)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "conf.txt"

/* constants */
int LEFT_WHEEL = motorA;
int RIGHT_WHEEL = motorC;
int GUN = motorB;
/* end constants */

/* Starting functions needed for interacting with the robot */
void stop() {
	motor[LEFT_WHEEL] = 0;
	motor[RIGHT_WHEEL] = 0;
  wait1Msec(1000);
}

void move_forward(int power) {
	motor[LEFT_WHEEL] = power;
	motor[RIGHT_WHEEL] = power;
}


void move_back(int power) {
	motor[LEFT_WHEEL] = -power;
	motor[RIGHT_WHEEL] = -power;

	wait1Msec(3000);

	stop();
}

// This function gets the argument, to see in which direction the
// robot should move, 1 or -1

void rotate(int degs, int power) {
	int dir = 1;
	if (degs < 0) {
		dir = -1;
		degs = -degs;
	}

	// Inverse the motors speed
  motor[LEFT_WHEEL] 	= dir * power;
  motor[RIGHT_WHEEL] 	= -dir * power;
}

void right_curve(int power) {
	motor[LEFT_WHEEL] = power;
	motor[RIGHT_WHEEL] = 1.8 * power;

	wait1Msec(4000);

	stop();
}

void left_curve(int power) {
	motor[LEFT_WHEEL] = 1.8 * power;
	motor[RIGHT_WHEEL] = power;

	wait1Msec(4000);
	stop();
}


void dbg(char* s)
{
	//return;
	writeDebugStream("DEBUG: %s\n",s);
}


int get_left_light_sensor()
{
	// This sensor needs correction to be as the other one
	return SensorValue[lightLeft];
}
int get_right_light_sensor()
{
	// This sensor is returned as it is
	return SensorValue[lightRight];
}

void print_sensors()
{
	writeDebugStream("Left: %d - Right: %d\n", get_left_light_sensor(), get_right_light_sensor());
}

/* End functions used to interact with the robot */

/* Our own functions here */

int light_arrived()
{
	return (			get_left_light_sensor() 	>= LEFT_ARRIVED_THRESHOLD 		&&
								get_right_light_sensor()  >= RIGHT_ARRIVED_THRESHOLD	) &&
					abs(	get_left_light_sensor() 	-  LEFT_ARRIVED_THRESHOLD
							- get_right_light_sensor() 	+  RIGHT_ARRIVED_THRESHOLD ) <= CENTERED_THRESHOLD;
}
int light_balanced()
{
	return (			get_left_light_sensor() 	>= LEFT_CAPTURE_THRESHOLD 		&&
								get_right_light_sensor() 	>= RIGHT_CAPTURE_THRESHOLD 	) &&
					abs(	get_left_light_sensor() 	-  LEFT_CAPTURE_THRESHOLD
							- get_right_light_sensor()	+	 RIGHT_CAPTURE_THRESHOLD	) <= CENTERED_THRESHOLD;
}

int light_unbalanced()
{
	return (get_left_light_sensor() >= LEFT_CAPTURE_THRESHOLD || get_right_light_sensor() >=  RIGHT_CAPTURE_THRESHOLD );
}

int bumped()
{
	// Return if either left or right bump
	return SensorValue[touchLeft] || SensorValue[touchRight];
}

void avoid_obstacle()
{
	dbg("DEBUG: Started task avoid_obstacle\n");

	// Read the values reported by the sensors
	int left_bump = SensorValue[touchLeft];
	int right_bump = SensorValue[touchRight];

	dbg("Moving back");
	//reverse
	move_back(DEFAULT_POWER);

	if ( left_bump == 1 && right_bump == 1 )
	{
		dbg("Hit was centered");
		//hit was center
		left_curve(DEFAULT_POWER);

	} else if (right_bump == 1)
	{
		dbg("Hit was right");
		//hit was right
		left_curve(DEFAULT_POWER);

	} else
	{
		dbg("Hit was left");
		//hit was left
		right_curve(DEFAULT_POWER);

	}
}

void balance_light()
{
	print_sensors();
	// In this task, the light_unbalanced() condition holds
	if ( 	get_left_light_sensor()  - LEFT_CAPTURE_THRESHOLD >
				get_right_light_sensor() - RIGHT_CAPTURE_THRESHOLD )
		rotate(ROTATE_LEFT,BALANCE_ROTATE_POWER);
	else
		rotate(ROTATE_RIGHT,BALANCE_ROTATE_POWER);

}

void find_light()
{
	rotate(ROTATE_LEFT,FIND_ROTATE_POWER);
}

void print_thresholds()
{
	writeDebugStream("Arr left: %d, Arr right: %d\n", LEFT_ARRIVED_THRESHOLD, RIGHT_ARRIVED_THRESHOLD);
	writeDebugStream("Cap left: %d, Cap right: %d\n", LEFT_CAPTURE_THRESHOLD, RIGHT_CAPTURE_THRESHOLD);
}
void set_automatic_thresholds()
{

	dbg("Need to wait 1s for the RGB Sensor");
	// RGB Sensor takes more time to be initialised
	wait1Msec(2000);

	int lightValueLeft = 0;
	int lightValueRight = 0;
	int darkValueLeft = 0;
	int darkValueRight = 0;

	lightValueLeft  = get_left_light_sensor();
	lightValueRight = get_right_light_sensor();

	print_sensors();
	LEFT_ARRIVED_THRESHOLD = lightValueLeft - 10;
	RIGHT_ARRIVED_THRESHOLD = lightValueRight - 10;


	dbg("remove light please");
	wait1Msec(3000);

	darkValueLeft = get_left_light_sensor();
	darkValueRight = get_right_light_sensor();

	LEFT_CAPTURE_THRESHOLD  = (lightValueLeft   + darkValueLeft )/2;
	RIGHT_CAPTURE_THRESHOLD = (lightValueRight  + darkValueRight)/2;

}

void set_manual_thresholds()
{
	LEFT_ARRIVED_THRESHOLD 	= 47;
	RIGHT_ARRIVED_THRESHOLD = 47;

	RIGHT_CAPTURE_THRESHOLD = 23;
	LEFT_CAPTURE_THRESHOLD  = 23;
}

/* End own functions here */

task main()
{
	clearDebugStream();

	//set_automatic_thresholds();
	set_manual_thresholds();

	print_thresholds();
	wait1Msec(1000);

	while (TRU)
	{
		//wait1Msec(2000);

		if ( bumped() )
		{
			dbg("Entered bumped");
			stop();
			avoid_obstacle();
		}
		else if ( light_arrived() )
		{
			stop();
			dbg("Entered light_arrived");
			continue; // return;
		}
		else if ( light_balanced() )
		{
			dbg("Entered light_balanced");
			move_forward(DEFAULT_POWER);
		}

		else if ( light_unbalanced() )
		{
			dbg("Entered light_unbalanced");
			// Here one sensor detects light, the other doesn't
			balance_light();
		}
		else
		{
			dbg("Entered find_light");
			// We are not finding light, just look for a source of light
			find_light();
		}

	}


}

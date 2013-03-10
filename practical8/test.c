// global vars


void clearDebugStats()
{
	//nMotorEncoder[LEFT_WHEEL] = 0;
	//nMotorEncoder[RIGHT_WHEEL] = 0;
}

void enter_cubicle(int dir) {

	motor[RIGHT_WHEEL] = DEFAULT_POWER;
	motor[LEFT_WHEEL] = DEFAULT_POWER;
	int wheel;
	float radius = 21;
	float w = 17.7;

	float k = (radius - (w / 2.0))/(radius + (w / 2.0));

	wheel = (dir == -1 ? RIGHT_WHEEL : LEFT_WHEEL);

	motor[wheel] *= k;

	wait1Msec(1025.65);

	motor[RIGHT_WHEEL] = DEFAULT_POWER;
	motor[LEFT_WHEEL] = DEFAULT_POWER;

	wait1Msec(300);
}

// dir = -1 (going east)
// dir = 1 (going west)
void follow_wall(int distance, int dir, bool skip) {

	int wheel, diff = 0, current_distance = 0, correction = 0;
	int count = 0;
	float k = 1.2;

	current_distance = SensorValue[sonar];

	while(current_distance < 60 || current_distance > 200) {
		count++;
		motor[RIGHT_WHEEL] = 80;
		motor[LEFT_WHEEL] = 80;

		// filter garbage readings
		if (current_distance > 200) {
			current_distance = SensorValue[sonar];
			continue;
		}

		writeDebugStream("Wheel %d, motor 1: %d, motor 2: %d \n", wheel, motor[RIGHT_WHEEL], motor[LEFT_WHEEL]);
		//wait1Msec(10);
		writeDebugStream("Sensor reading: %d \n", current_distance);
		//wait1Msec(10);

		//if (count%5 == 0) {
		diff = distance - current_distance;

		if(abs(diff) <= 2) {
			diff =  0;
		}

		// set power of motor to turn towards wall
		correction = (int) (-k * diff);

		motor[RIGHT_WHEEL] -= correction;
		motor[LEFT_WHEEL] += correction;

		writeDebugStream("Value of diff: %d , correction: %d, wheel: %d \n", diff, correction, wheel);
		//wait1Msec(10);
		writeDebugStream("Wheel %d, motor 1: %d, motor 2: %d \n", wheel, motor[RIGHT_WHEEL], motor[LEFT_WHEEL]);
		//wait1Msec(10);

	//	}
		//wait1Msec(50);
		current_distance = SensorValue[sonar];
	}

	//writeDebugStream("Wheel %d, motor 1: %d, motor 2: %d \n", wheel, motor[RIGHT_WHEEL], motor[LEFT_WHEEL]);
	//wait1Msec(2000);

}

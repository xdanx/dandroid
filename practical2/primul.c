int ROTATE_90_TIME = 1210;
int ROTATE_POWER = 25;
int MOVE_POWER = 10;
const float K = 0.00037; // for distance in cm, and time in milliseconds

int LEFT_WHEEL = motorA;
int RIGHT_WHEEL = motorC;
int GUN = motorB;
bool synch = false;


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
	motor[LEFT_WHEEL] = power;
	motor[RIGHT_WHEEL] = power;
	float time = getMoveTime(power, distance);
	wait1Msec(time);
	stop();
}

void rotate(int degs) {
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



void moveForward40() {
	move(MOVE_POWER, 40);
}

void moveBackward40() {
	move(-MOVE_POWER, 40);
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
	nMotorEncoder[LEFT_WHEEL] = 0;
	nMotorEncoder[RIGHT_WHEEL] = 0;

}

void printDebugStats()
{
	writeDebugStream("LEFT_WHEEL encoder: %d\n", nMotorEncoder[LEFT_WHEEL] );
	writeDebugStream("RIGHT_WHEEL encoder: %d\n", nMotorEncoder[RIGHT_WHEEL] );
	clearDebugStats();
}


task main()
{

	nMotorEncoder[LEFT_WHEEL] = 0;
	nMotorEncoder[RIGHT_WHEEL] = 0;

	if (synch) {

		//nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
		//nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;

	// A is master and C is slave
		nSyncedMotors = synchAC;
  	nSyncedTurnRatio = 100;
	}
  //move(25, 40);
  //moveForward40();
  //rotate(90);

  //move_k(60, 2500);

	//return;

 // moveForward40();


	int i = 0;
	int loops = 4;
	int endRotation = 0;
	for (i=0 ;i < loops; ++i) {
		moveForward40();
		printDebugStats();
	  rotate(90);
	  printDebugStats();
  }
}

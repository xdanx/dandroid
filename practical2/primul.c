
/* constants */
int ROTATE_90_TIME = 1108;
int ROTATE_POWER = 25;
int MOVE_POWER = 10;
const int DISPLAY_SCALE = 10;

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
	float time = getMoveTime(power, distance);
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

// 0.00045
// 0.000468
// 0.000461

void move_k(int speed, int time)
{
	motor[motorA] = speed;
	motor[motorC] = speed;
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

task computePosition() {
	while (true) {
		wait1Msec(10);
		int curLeft = nMotorEncoder[LEFT_WHEEL];
		int curRight = nMotorEncoder[RIGHT_WHEEL];
		int deltaLeft = curLeft - leftEncodings;
		int deltaRight = curRight - rightEncodings;
		switch (robotState) {
			case MOVE_STATE:
				addDistance(&position, deltaLeft / ENCODINGS_PER_CM);
				break;
			case ROTATE_STATE:
			  addAngle(&position, deltaLeft / ENCODINGS_PER_DEGREE);
				break;
		}

		leftEncodings = curLeft;
		rightEncodings = curRight;
	}
}

task drawPosition() {
	while (true) {
		wait1Msec(10);
		nxtSetPixel(15 + (int)(position.x),15 + (int)(position.y));
	}
}

task main()
{
	//move_k(30, 4000);
	rotate(90);
}

/*task main() {
	position.x = 0;
	position.y = 0;
	position.angle = 0;
	nMotorEncoder[LEFT_WHEEL] = 0;
	nMotorEncoder[RIGHT_WHEEL] = 0;
	if (synch) {
		nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
		nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;

	// A is master and C is slave
		nSyncedMotors = synchAC;
  	nSyncedTurnRatio = 100;
	}
  //move(25, 40);
  //moveForward40();

  //move_k(60, 2500);

	//return;

 // moveForward40();

	StartTask(computePosition);
	StartTask(drawPosition);


	int i = 0;
	int loops = 4;
	int endRotation = 0;
	for (i=0 ;i < loops; ++i) {
		moveForward40();
		printDebugStats();
	  rotate(90);
	  printDebugStats();
  }

  StopTask(computePosition);
  wait10Msec(6000); // wait 1MIN
  StopTask(drawPosition);
}*/

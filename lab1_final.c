#define PI 3.1416

#define GLITCH_ANGLE 4
#define R_WHEEL 28.2
#define R_AXLE 57.4


void forward400mm();
void backward400mm();

void left90deg();
void right90deg();


// state vars
float x = 0;	// mm
float y = 0;	// mm
float th = 0;	// rad


task main()
{
	nPidUpdateInterval = 20;
	wait1Msec(1000);


	//left90deg();
	//left90deg();
	//left90deg();
	//left90deg();

	// square movement
	forward400mm();
	left90deg();
	forward400mm();
	left90deg();
	forward400mm();
	left90deg();
	forward400mm();
	left90deg();


	/*
	backward400mm();
	right90deg();
		backward400mm();
	right90deg();
		backward400mm();
	right90deg();
		backward400mm();
	right90deg();*/
}

// can be easily parameterised for any distance in mm (calculation done at runtime)
void forward400mm()
{
  nMotorEncoder[motorB] = 0;
  nMotorEncoder[motorC] = 0;

  // feed forward pre-glitch due to slave wheel's controller responding later (low D factor in PID?)
  nSyncedMotors = synchBC;
  nSyncedTurnRatio = 0;

  nMotorEncoderTarget[motorB] = -GLITCH_ANGLE;
  motor[motorB] = -30;

  while(nMotorRunState[motorB] != runStateIdle){}

	// movement in a line
  nMotorEncoderTarget[motorB] = (int)(400/(2*PI*R_WHEEL))*360; //812ish
  nSyncedTurnRatio = 100;
  motor[motorB] = 30;

  // state update loop while waiting for finish movement
  int lastEncoderValB = 0;
  int lastEncoderValC = 0;
  int d = 0;
  int deltaRotAngleDegB = 0;
  int deltaRotAngleDegC = 0;

  // update state estimate
  while(nMotorRunState[motorB] != runStateIdle)
  {
  	deltaRotAngleDegB = nMotorEncoder[motorB]-lastEncoderValB;

  	if (deltaRotAngleDegB > 20)
  	{
  		deltaRotAngleDegC = nMotorEncoder[motorC]-lastEncoderValC;

 			lastEncoderValB = nMotorEncoder[motorB];
			lastEncoderValC = nMotorEncoder[motorC];

  		d = ((deltaRotAngleDegB*R_WHEEL*(PI/180))+(deltaRotAngleDegC*R_WHEEL*(PI/180)))/2; // in mm

  		x = x + ((float) d)*cos(th);
  		y = y + ((float) d)*sin(th);
  		th = th + (deltaRotAngleDegB - deltaRotAngleDegC)/2*(PI/180)*R_WHEEL/R_AXLE;

			nxtSetPixel(10+(int)(x/10), 10+(int)(y/10));
  	}
  }

  // post-glitch
  nSyncedMotors = synchNone;
  nMotorEncoderTarget[motorC] = -GLITCH_ANGLE;
  motor[motorC] = -30;
  while(nMotorRunState[motorC] != runStateIdle){}

  // finish state calculation
  deltaRotAngleDegB = nMotorEncoder[motorB]-lastEncoderValB;
  deltaRotAngleDegC = nMotorEncoder[motorC]-lastEncoderValC;
  d = ((deltaRotAngleDegB*R_WHEEL*(PI/180))+(deltaRotAngleDegC*R_WHEEL*(PI/180)))/2; // distance in mm

  x = x + ((float) d)*cos(th);
  y = y + ((float) d)*sin(th);
  th = th + (deltaRotAngleDegB - deltaRotAngleDegC)/2*(PI/180)*R_WHEEL/R_AXLE;

	nxtSetPixel(10+(int)(x/10), 10+(int)(y/10));
}

///////////////////////////////////////////////////////////////////////////////////

// can be easily parameterised for any distance in mm (calculation done at runtime)
void backward400mm()
{
  nMotorEncoder[motorB] = 0;
  nMotorEncoder[motorC] = 0;

  // feed forward pre-glitch due to slave wheel's controller responding later (low D factor in PID?)
  nSyncedMotors = synchBC;
  nSyncedTurnRatio = 0;

  nMotorEncoderTarget[motorB] = GLITCH_ANGLE;
  motor[motorB] = 30;

  while(nMotorRunState[motorB] != runStateIdle){}

	// movement in a line
  nMotorEncoderTarget[motorB] = (-1)*(int)(400/(2*PI*R_WHEEL))*360; //812ish
  nSyncedTurnRatio = 100;
  motor[motorB] = -30;

  // state update loop while waiting for finish movement
  int lastEncoderValB = 0;
  int lastEncoderValC = 0;
  int d = 0;
  int deltaRotAngleDegB = 0;
  int deltaRotAngleDegC = 0;

  // update state estimate
  while(nMotorRunState[motorB] != runStateIdle)
  {
  	deltaRotAngleDegB = nMotorEncoder[motorB]-lastEncoderValB;

  	if (deltaRotAngleDegB < -20)
  	{
  		deltaRotAngleDegC = nMotorEncoder[motorC]-lastEncoderValC;

 			lastEncoderValB = nMotorEncoder[motorB];
			lastEncoderValC = nMotorEncoder[motorC];

  		d = ((deltaRotAngleDegB*R_WHEEL*(PI/180))+(deltaRotAngleDegC*R_WHEEL*(PI/180)))/2; // in mm

  		x = x + ((float) d)*cos(th);
  		y = y + ((float) d)*sin(th);
  		th = th + (deltaRotAngleDegB - deltaRotAngleDegC)/2*(PI/180)*R_WHEEL/R_AXLE;

			nxtSetPixel(10+(int)(x/10), 10+(int)(y/10));
  	}
  }

  // post-glitch
  nSyncedMotors = synchNone;
  nMotorEncoderTarget[motorC] = GLITCH_ANGLE;
  motor[motorC] = 30;
  while(nMotorRunState[motorC] != runStateIdle){}

  // finish state calculation
  deltaRotAngleDegB = nMotorEncoder[motorB]-lastEncoderValB;
  deltaRotAngleDegC = nMotorEncoder[motorC]-lastEncoderValC;
  d = ((deltaRotAngleDegB*R_WHEEL*(PI/180))+(deltaRotAngleDegC*R_WHEEL*(PI/180)))/2; // distance in mm

  x = x + ((float) d)*cos(th);
  y = y + ((float) d)*sin(th);
  th = th + (deltaRotAngleDegB - deltaRotAngleDegC)/2*(PI/180)*R_WHEEL/R_AXLE;

	nxtSetPixel(10+(int)(x/10), 10+(int)(y/10));
}

///////////////////////////////////////////////////////////////////////////////////

void left90deg()
{
	int deltaRotAngleDegB = 0;
	int deltaRotAngleDegC = 0;
	int savedEncoderValB = nMotorEncoder[motorB];
	int savedEncoderValC = nMotorEncoder[motorC];

  // feed forward pre-glitch
  nSyncedMotors = synchBC;
  nSyncedTurnRatio = 0;
  nMotorEncoderTarget[motorB] = -GLITCH_ANGLE;
  motor[motorB] = -30;
  while(nMotorRunState[motorB] != runStateIdle){}

	// rotation
  nMotorEncoderTarget[motorB] = (int) (R_AXLE/R_WHEEL)*90 + 6; // (pi/2 * r_a) = (th_w * r_w) // +6 is unaccounted linearity (wheel slack effect on deglitch?)
  nSyncedTurnRatio = -100;
  motor[motorB] = 30;
  while(nMotorRunState[motorB] != runStateIdle){}

  nSyncedMotors = synchNone;

  // post-glitch
  nMotorEncoderTarget[motorC] = -GLITCH_ANGLE;
  motor[motorC] = -30;
  while(nMotorRunState[motorC] != runStateIdle){}

	// state calc
  deltaRotAngleDegB = nMotorEncoder[motorB]-savedEncoderValB;
  deltaRotAngleDegC = nMotorEncoder[motorC]-savedEncoderValC;
  th = th + (deltaRotAngleDegB - deltaRotAngleDegC)/2*(PI/180)*R_WHEEL/R_AXLE;
}

///////////////////////////////////////////////////////////////////////////////////

void right90deg()
{
	int deltaRotAngleDegB = 0;
	int deltaRotAngleDegC = 0;
	int savedEncoderValB = nMotorEncoder[motorB];
	int savedEncoderValC = nMotorEncoder[motorC];

  // feed forward pre-glitch
  nSyncedMotors = synchBC;
  nSyncedTurnRatio = 0;
  nMotorEncoderTarget[motorB] = GLITCH_ANGLE;
  motor[motorB] = 30;
  while(nMotorRunState[motorB] != runStateIdle){}

	// rotation
  nMotorEncoderTarget[motorB] = (-1)*(int)(R_AXLE/R_WHEEL)*90 - 6; // (-pi/2 * r_a) = (th_w * r_w) // -6 is unaccounted linearity (wheel slack effect on deglitch?)
  nSyncedTurnRatio = -100;
  motor[motorB] = -30;
  while(nMotorRunState[motorB] != runStateIdle){}

  nSyncedMotors = synchNone;

  // post-glitch
  nMotorEncoderTarget[motorC] = GLITCH_ANGLE;
  motor[motorC] = 30;
  while(nMotorRunState[motorC] != runStateIdle){}

	// state calc
  deltaRotAngleDegB = nMotorEncoder[motorB]-savedEncoderValB;
  deltaRotAngleDegC = nMotorEncoder[motorC]-savedEncoderValC;
  th = th + (deltaRotAngleDegB - deltaRotAngleDegC)/2*(PI/180)*R_WHEEL/R_AXLE;
}

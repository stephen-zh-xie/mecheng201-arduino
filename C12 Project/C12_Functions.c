#pragma systemFile

void armTime(float armPower, int time)
{
	armPower = saturate(armPower, -100, 100);
	motorPower(ArmMotor, armPower);
	delay(time);
	motorPower(ArmMotor, 0);
}

/* void waitForStart()
{
	int result = 0;
	while (result == 0) {
		delay(50);
		result = readSensor(StartButton);
	}
	delay(800);
} */

int count_to_mm(int encoderCount){
	int distanceTravelled = 0;

	distanceTravelled = (encoderCount/360.0) * 3.0 / 5.0 * drivingWheelDiameter*PI;

	return distanceTravelled;
}

void driveDistance(int distance, float power){
	int forward;
	int leftEncoder;
	int rightEncoder;
	int currentDistance = 0;
	int leftPower;
	int rightPower;

	setSensor(LeftEnc,0);
	setSensor(RightEnc,0);

	if(distance > 0){
		forward = 1;
	}else{
		forward = -1;
	}

	if(power < 0){
		power = power * -1;
	}
	power = saturate(power,0,100);
	power = power/100 * 127;

	leftPower = power;
	rightPower = power;

	motorPower(LeftMotor,power*forward);
	motorPower(RightMotor,power*forward);

	while((currentDistance * forward) < (distance * forward)){
		delay(50);
		leftEncoder = readSensor(LeftEnc);
		rightEncoder = readSensor(RightEnc);
		currentDistance = count_to_mm( ((leftEncoder+rightEncoder)/2) );
	}

	motorPower(LeftMotor,0);
	motorPower(RightMotor,0);
}
void turn(float degrees, float Kp){
	signed int PwrLeft = 25; //FOR NOW
	signed int PwrRight = 25; // FOR NOW
	int rotation;
	//+ve = CLOCKWISE
	int encoderLeftV;
	int encoderRightV;
	int average;
	int circumfrence = (robotWidth-wheelWidth)*PI;
	float arc;
	float arcDegrees;

	if(degrees >= 0){
		rotation = 1;
	}else{
		rotation = -1;
	}

	setSensor(LeftEnc,0);
	setSensor(RightEnc,0);

	do{
		encoderLeftV = readSensor(LeftEnc);
		encoderRightV = readSensor(RightEnc);

		encoderLeftV = 	encoderLeftV * rotation;
		encoderRightV = encoderRightV * -1 * rotation;

		printf("PwrLeft: %d",PwrRight);
		printf("PwrRight: %d",PwrLeft);

		average = (encoderLeftV + encoderRightV)/2;

		arc = count_to_mm(average);
		arcDegrees = (arc/circumfrence) * 360;
		printf("%d",arc);

		printf("%d",arcDegrees);

		PwrLeft = PwrLeft + (average - encoderLeftV)*Kp;
		PwrRight = PwrRight + (average - encoderRightV)*Kp;

		PwrLeft = saturate(PwrLeft,20,100);
		PwrRight = saturate(PwrRight,20,100);

		motorPower(LeftMotor, (PwrLeft * rotation));
		motorPower(RightMotor, (PwrRight * -1 * rotation));

		delay(100);

	}while(arcDegrees < degrees*rotation);

	motorPower(LeftMotor, 0);
	motorPower(RightMotor, 0);
}
void driveStraight(int distance, float power) {
	// Slightly altered copy of the driveDistance function. Not good etiquette, I know, but I'm too lazy to do it properly.
	// The important lines are Kp = 0.01, leftPower = stuff and rightPower = stuff. That can be spliced into other functions, like light-based ones.
	// Note all values (and the code itself, for that matter) are untested.

	int forward;
	int leftEncoder;
	int rightEncoder;
	int currentDistance = 0;
	int leftPower;
	int rightPower;
	int Kp = 1;

	setSensor(LeftEnc, 0);
	setSensor(RightEnc, 0);

	if (distance > 0) {
		forward = 1;
	}
	else {
		forward = -1;
	}

	if (power < 0) {
		power = power * -1;
	}
	power = saturate(power, 0, 100);
	power = power / 100 * 127;

	leftPower = power;
	rightPower = power;

	motorPower(LeftMotor, power * forward);
	motorPower(RightMotor, power * forward);

	while ((currentDistance * forward) < (distance * forward)) {
		delay(50);
		leftEncoder = readSensor(LeftEnc);
		rightEncoder = readSensor(RightEnc);
		currentDistance = count_to_mm(((leftEncoder + rightEncoder) / 2));

		leftPower = power - Kp * forward * (leftEncoder - rightEncoder);
		rightPower = power - Kp * forward * (rightEncoder - leftEncoder);

		// Adjust actual motor speed
		motorPower(LeftMotor, leftPower * forward);
		motorPower(RightMotor, rightPower * forward);
	}

	motorPower(LeftMotor, 0);
	motorPower(RightMotor, 0);
}
void driveStraightToLine(int distance, float power) {
// Slightly altered copy of the driveDistance function. Not good etiquette, I know, but I'm too lazy to do it properly.
// The important lines are Kp = 0.01, leftPower = stuff and rightPower = stuff. That can be spliced into other functions, like light-based ones.
// Note all values (and the code itself, for that matter) are untested.

	int forward;
	int leftEncoder;
	int rightEncoder;
	int leftPower;
	int rightPower;
	int Kp = 1;
	int onLine = 0;
	int sensorL;
	int sensorM;
	int sensorR;

	setSensor(LeftEnc, 0);
	setSensor(RightEnc, 0);

	if (distance == 1) {
		forward = 1;
	}
	else {
		forward = -1;
	}

	if (power < 0) {
		power = power * -1;
	}
	power = saturate(power, 0, 100);
	power = power / 100 * 127;

	leftPower = power;
	rightPower = power;

	motorPower(LeftMotor, power * forward);
	motorPower(RightMotor, power * forward);

	while (!onLine) {
		delay(50);

		sensorL = readSensor(LeftLight);
		sensorM = readSensor(MidLight);
		sensorR = readSensor(RightLight);

		if((sensorL>=BLACKMIN) && (sensorL<=BLACKMAX)){
			onLine = 1;
		}else if((sensorM>=BLACKMIN) && (sensorM<=BLACKMAX)){
			onLine = 1;
		}else if((sensorR>=BLACKMIN) && (sensorR<=BLACKMAX)){
			onLine = 1;
		}

		leftEncoder = readSensor(LeftEnc);
		rightEncoder = readSensor(RightEnc);

		leftPower = power - Kp * forward * (leftEncoder - rightEncoder);
		rightPower = power - Kp * forward * (rightEncoder - leftEncoder);

		// Adjust actual motor speed
		motorPower(LeftMotor, leftPower * forward);
		motorPower(RightMotor, rightPower * forward);
	}

	motorPower(LeftMotor, 0);
	motorPower(RightMotor, 0);
}

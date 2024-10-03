#include "findArmAngle.c"

void rotateArm(int angle, int tolerance) {
	// Rotates the arm to a specific angle
	// Zero is at the horizontal, so I'll use the findArmAngle function to control position
	// Assume it's already been calibrated
	// Uses a PI controller
	// Everything in degrees (time is in milliseconds)

	// Initialise variables
	int angleError = 0;
	int angleTotal = 0;
	int anglePower = 0;
	int timeInArea = 0;
	int timeRequiredArea = 40;
	int currentEncoder;
	int saturateLower = 1;
	int saturateUpper = 125;

	// Initialise kp and ki values - untested
	float KP = 1;
	float KI = 0.3;

	// Initialise the error
	currentEncoder = readSensor(EncoderArm);
	currentEncoder = findArmAngle(currentEncoder);
	angleError = angle - currentEncoder;

	// Begin loop: Have you been within tolerance of target for some time?
	while ( (angleError > tolerance) || (angleError < -tolerance) || (timeInArea < timeRequiredArea) ) {
		// Loop begins: read angle and set error
		currentEncoder = readSensor(EncoderArm);
		currentEncoder = findArmAngle(currentEncoder);
		angleError = angle - currentEncoder;
		// If angleError isn't so large that it's over the saturation limit, add error to angle total for I part of PI
		if (angleError > (-saturateUpper / KP) && angleError < (saturateUpper / KP)) {
			angleTotal = angleTotal + angleError;
		}
		// Calculate total power
		anglePower = KP * angleError + KI * angleTotal;

		// Saturate power based on error (either forward or backward)
		if (angleError > 0) {
			anglePower = saturate(anglePower, saturateLower, saturateUpper);
		} else if (angleError < 0) {
			anglePower = saturate(anglePower, -saturateUpper, -saturateLower);
		}

		// Send power to arm motor
		motorPower(ArmMotor, anglePower);

		// If the arm is within the tolerance, we increment time
		// This is to prevent it from stopping suddenly and retaining momentum.
		if ( (angleError < tolerance) && (angleError > -tolerance) ) {
			timeInArea = timeInArea + 1;
		}

		// The loop must keep running - we add a 50ms delay before the next iteration
		delay(50);

		datalogDataGroupStart();
		datalogAddValue(0,angleError);
		datalogAddValue(1,anglePower);
		datalogDataGroupEnd();
	}

	// Reset motors
	motorPower(ArmMotor, 0);
}

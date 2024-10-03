#pragma systemFile
#include "C12_Functions.c"

void driveToObjectPI(int distance, int tolerance) {
	// Uses two PI controllers
	// one to control the distance to the object, one to control the turning
	// distance and tolerance in millimetres

	// Initialise variables first
	int distanceError = 0;
	int distanceTotal = 0;
	int distancePower = 0;
	int sonarValue;
	int turnError = 0;
	int turnTotal = 0;
	int turnPower = 0;
	int encoderLeft = 0;
	int encoderRight = 0;
	int encoderAverage = 0;
	int encoderDifference = 0;
	int timeInArea = 0;
	int timeRequiredArea = 20;
	int sonar = -1;
	int saturateLower = 20;
	int saturateUpper = 40;

	// Initialise kp and ki values - experimental
	float distanceKP = 0.5;
	float distanceKI = 0.005;
	float turnKP = 0.8;
	float turnKI = 0.1;

	// Reset sensors, set distanceError to initial value
	setSensor(LeftEnc, 0);
	setSensor(RightEnc, 0);
	distancePower = 25; // keep constant for first sonar loop

	// Begin first loop: Can you spot the object? Note that it is assumed to be straight ahead
	while (sonar == -1) {
		// Loop begins: get encoder count
		encoderLeft = readSensor(LeftEnc);
		encoderRight = readSensor(RightEnc);
		encoderAverage = (encoderLeft + encoderRight) / 2;
		encoderDifference = (encoderLeft - encoderRight);

		// Calculate turnError in millimetres, add to turnTotal
		turnError = encoderDifference;
		turnTotal = turnTotal + turnError;

		// Calculate turn power - doesn't need saturation due to (hopefully) low values
		turnPower = turnKP * turnError + turnKI * turnTotal;

		// Set motor power to constant + turn effect
		motorPower(LeftMotor, (distancePower - turnPower));
		motorPower(RightMotor, (distancePower + turnPower));

		// Check the sonar again
		sonar = readSensor(sonarSensor);

		// Wait 50 milliseconds
		delay(50);
	}

	// Begin second loop: Have you been within tolerance of distance from object for some time?
	while (distanceError > tolerance || distanceError < -tolerance || timeInArea < timeRequiredArea) {
		// Loop begins: get encoder count
		encoderLeft = readSensor(LeftEnc);
		encoderRight = readSensor(RightEnc);
		encoderAverage = (encoderLeft + encoderRight) / 2;
		encoderDifference = (encoderLeft - encoderRight);

		// Calculate distanceError in millimetres
		sonarValue = readSensor(sonarSensor);
		if(sonarValue != -1){
			distanceError = sonarValue - distance;
		}
		// If distanceError is between -125 and 125 (the saturation limits) then add to distanceTotal
		// So that it doesn't add up massive numbers and cause major integrator windup
		if (distanceError > (-saturateUpper / distanceKP) && distanceError < (saturateUpper / distanceKP)) {
			distanceTotal = distanceTotal + distanceError;
		}
		// Calculate turnError in millimetres, add to turnTotal
		// Doesn't need the integrator windup preventor, since error should stay almost at zero
		turnError = encoderDifference;
		turnTotal = turnTotal + turnError;
		// Calculate resulting distance power
		distancePower = distanceKP * distanceError + distanceKI * distanceTotal;

		// Saturate distancePower based on distanceError (either forward or backward)
		if (distanceError > 0) {
			distancePower = saturate(distancePower, saturateLower, saturateUpper);
		} else if (distanceError < 0) {
			distancePower = saturate(distancePower, -saturateUpper, -saturateLower);
		}

		// Calculate turn power - doesn't need saturation due to (hopefully) low values
		turnPower = turnKP * turnError + turnKI * turnTotal;


		// Now we actually set the motor power
		motorPower(LeftMotor, (distancePower - turnPower));
		motorPower(RightMotor, (distancePower + turnPower));

		// If the robot is within the tolerance, we increment time
		// This is to prevent it from stopping suddenly and retaining momentum.
		if (distanceError < tolerance && distanceError > -tolerance) {
			timeInArea = timeInArea + 1;
		}
		// The loop must keep running - we add a 50ms delay before the next iteration
		delay(50);

		datalogDataGroupStart();
		datalogAddValue(0,distanceError);
		datalogAddValue(1,distanceKP * distanceError);
		datalogAddValue(2,distanceKI * distanceTotal);
		datalogAddValue(3,sonarValue);
		datalogAddValue(4,timeInArea);
		datalogDataGroupEnd();
	}

	// Reset motors
	motorPower(LeftMotor, 0);
	motorPower(RightMotor, 0);
}

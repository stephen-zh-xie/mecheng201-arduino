#pragma systemFile
#include "C12_Functions.c"

void driveStraightPI(int distance, int tolerance) {
	// Uses two PI controllers
	// one to control the distance to the target, one to control the turning
	// distance and tolerance in millimetres

	// Initialise variables first
	int distanceError = 0;
	int distanceTotal = 0;
	int distancePower = 0;
	int turnError = 0;
	int turnTotal = 0;
	int turnPower = 0;
	int encoderLeft = 0;
	int encoderRight = 0;
	int encoderAverage = 0;
	int encoderDifference = 0;
	int saturateUpper = 11;
	int saturateLower = 125;
	int timeInArea = 0;
	int timeRequiredArea = 40;

	// Initialise kp and ki values - experimental
	float distanceKP = 0.5;
	float distanceKI = 0.000;
	float turnKP = 0.5;
	float turnKI = 0.00;

	// Reset sensors, set distanceError to initial value
	setSensor(LeftEnc, 0);
	setSensor(RightEnc, 0);
	distanceError = distance; // we are 'distance' mm away from target position

	// Begin loop: Have you been within tolerance of target for some time?
	while ( (distanceError > tolerance) || (distanceError < -tolerance) || (timeInArea < timeRequiredArea) ) {
		// Loop begins: get encoder count
		encoderLeft = readSensor(LeftEnc);
		encoderRight = readSensor(RightEnc);
		encoderAverage = (encoderLeft + encoderRight) / 2;
		encoderDifference = (encoderLeft - encoderRight);

		// Calculate distanceError in millimetres
		distanceError = distance - count_to_mm(encoderAverage);
		// If distanceError is between -125/kp and 125/kp (the saturation limits) then add to distanceTotal
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
		if ( (distanceError < tolerance) && (distanceError > -tolerance) ) {
			timeInArea = timeInArea + 1;
		}

		// The loop must keep running - we add a 50ms delay before the next iteration
		delay(50);

		datalogDataGroupStart();
		datalogAddValue(0,turnPower);
		datalogAddValue(1,turnError);
		datalogAddValue(2,turnTotal);
		datalogAddValue(3,distancePower);
		datalogAddValue(4,distanceError);
		datalogAddValue(5,distanceTotal);
		datalogDataGroupEnd();
	}

	// Reset motors
	motorPower(LeftMotor, 0);
	motorPower(RightMotor, 0);
}

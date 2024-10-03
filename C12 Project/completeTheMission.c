
void completeTheMission()
{
	int distanceTravelled;

	armDown(50);
	resetArmAngle();

	rotateArm(40,1);

	driveToObjectPI(376, 1);

	distanceTravelled = readSensor(RightEnc);
	distanceTravelled = count_to_mm(distanceTravelled);

	rotateArm(-7, 1);

	driveStraightPI(10, 1);
	distanceTravelled = distanceTravelled + 10;

	rotateArm(40, 1);

	distanceTravelled = readSensor(RightEnc);
	distanceTravelled = count_to_mm(distanceTravelled);

	driveStraightPI(-distanceTravelled, 5);
	distanceTravelled = 0;

	turnPI(-90);
	driveStraightPI(713, 3);
	turnPI(80);

	followLine();

	driveStraightPI(348, 3);
	rotateArm(-7, 1);

	driveStraightPI(-20, 3);
	rotateArm(40,5);

	turnPI(90);

	driveStraightPI(1385, 5);

	turnPI(90);

	driveStraightPI(328, 10);
}

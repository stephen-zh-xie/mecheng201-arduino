
int findArmAngle(float encoder){
	// Takes an encoder count and returns the angle of the arm, with zero being horizontal
	// 5050 encoder counts for a single arm revolution
	// Assume the arm encoder's already been reset, and zero is at the bottom
	float bottomAngle = -29; // In millimetres
	float armAngle;

	// To find angle above bottom, just divide by 5050 and multiply by 360
	// So I'll just add the (negative) bottom angle to get the horizontal at zero
	armAngle = ( encoder * 360 / 5050 ) + bottomAngle;
	return armAngle;
}

void resetArmAngle() {
	armDown(50);
	setSensor(armEncoder, 0);
}

#pragma systemFile

#define printf writeDebugStreamLine


//************************************************************************************
// ROBOTC MERGER code added below

//This file is to offer portability between code in RobotC and PROS
//The following functions are compatible on both PROS and RobotC
//Though they may be implemented differently by the respective library.
//The functions are simply 'wrappers' for native RobotC functions which are then called accordingly

//macros for SENSORS
#define MidLight	 	0
#define LeftLight 		1
#define RightLight	 	2
//#define StartButton 	3	// select (dgtl 2)
//#define StopButton 		4 // confirm (dgtl 1)
#define RightEnc 		5
#define LeftEnc 		6
#define LowArmLimit 	7
//#define HighArmLimit 	8
#define EncoderArm		9
#define sonarSensor		10
//#define rightLED		11
//#define leftLED			12


//macros for MOTORS
#define LeftMotor 		0
#define RightMotor 		1
#define ArmMotor 		2

//macros Timers
#define T_1 0 //T1
#define T_2 1 //T2
#define T_3 2 //T3
#define T_4 3 //T4

//macros For Tasks to be used on RobotC similar to how they are used on PROS
#define Task_1 Task1
#define Task_2 Task2
#define Task_3 Task3

//Function prototypes
void motorPower(int motorSelect,int power);//set motor power
int getMotorPower(int motorSelect);//read motor power value
int readSensor(int sensorSelect);//read sensor value
void setSensor(int sensorSelect, int sValue);//set or reset sensor/component
void resetTimer(int timerName);//resetting timer
int readTimer(int timerName);//reading timer time
float saturate (float input, float lower, float upper);//saturates the power sent to motors

// defining a golab variable for power imbalance
int powerImbalance;

int _armEncoderOffset = 0;	// an offset to help reset the encoder as it doesn't work in RVW

void initialise(int robotNum, int ideal){
	// Generating random motor power imbalance
	srand(robotNum);	// setting the random number seed generator
	int direction=rand();
	if(ideal == 1){
		powerImbalance = 0;
	}
	else{
		direction = rand();
		powerImbalance = sgn(direction%200-100)*(2+rand()%3);
	}
}

//MOTORS - Setting Power
void motorPower(int motorSelect,int power)
{
	//saturate the power being sent to the motors using saturate function
	int outPower = (int) saturate(power,-110,110);

	//switch case statement checking which motor is selected
	switch (motorSelect){
	case 0:
		if(outPower==0){
			motor[_motorLeft] = outPower; //sets Left Motor to power level
		}
		else{
			motor[_motorLeft] = outPower+powerImbalance; //sets Left Motor to power level
		}
		break;
	case 1:
		if(outPower==0){
			motor[_motorRight] = outPower;//sets Right Motor to power level
		}
		else{
			motor[_motorRight] = outPower-powerImbalance; //sets Right Motor to power level
		}
		break;
	case 2: //Arm motor, first checking limit switches
		motor[_motorArm] = outPower; //sets Arm Motor to power level
		break;
	default:
		writeDebugStreamLine("Unrecognised Motor Selected"); //invalid motor/input selected
		break;
	}
}
//MOTORS - Getting Power
int getMotorPower(int motorSelect)
{
	int powerValue;
	//switch case statement checking which motor is selected
	switch (motorSelect){
	case 0:
		powerValue = motor[_motorLeft]; //reads the Left Motor power value
		break;
	case 1:
		powerValue = motor[_motorRight]; //reads the Right Motor power value
		break;
	case 2:
		powerValue = motor[_motorArm]; //reads the Arm Motor power value
		break;
	default:
		writeDebugStreamLine("Unrecognised Motor Selected"); //invalid motor/input selected
		break;
	}
	return powerValue; // return the motor power
}

//SENSORS - Reading Sensor Values
int readSensor(int sensorSelect)
{
	int result;
	//switch case statement checking which sensor is selected
	switch (sensorSelect){
	case 0:
		result = SensorValue [ _lightMid ]; //read Middle Light Sensor
		break;
	case 1:
		result = SensorValue [ _lightLeft ]; //read Left Light Sensor
		break;
	case 2:
		result = SensorValue [ _lightRight ]; //read Right Light Sensor
		break;
	case 5:
		result = SensorValue [ _encRight ]; //read Right wheel Encoder
		break;
	case 6:
		result = SensorValue [ _encLeft ]; //read Left wheel Encoder
		break;
	case 7:
		result = SensorValue [ _armLimit_low ]; //read Lower Limit Switch on Arm
		break;
	case 9:
		result = SensorValue [ _armEncoder ]+_armEncoderOffset; //read arm encoder sensor
		break;
	case 10:
		result = SensorValue [ _sonar ]; //read ultrasonic sensor
		break;
	default:
		writeDebugStreamLine("Unrecognised Sensor Selected"); //invalid sensor selected
		break;
	}
	return result; //return the sensor reading
}

//SENSORS - Setting Sensor Values
void setSensor(int sensorSelect, int sValue)
{
	//switch case statement checking which sensor is selected
	switch (sensorSelect){
	case 5:
		SensorValue [ _encRight ] = sValue; //Set/Reset Right Encoder
		break;
	case 6:
		SensorValue [ _encLeft ] = sValue; //Set/Reset Left Encoder
		break;
	case 9:
		//SensorValue [ _armEncoder ] = sValue; //Set/Reset arm Encoder
		_armEncoderOffset = sValue - readSensor(EncoderArm); 	// to set the encoder to sValue when reading
		//writeDebugStreamLine("offset = %d",_armEncoderOffset);
		break;
	default:
		writeDebugStreamLine("Unrecognised Sensor Selected"); //invalid sensor selected (some sensors cannot be 'set')
		break;
	}
}

//RESET TIMERS
void resetTimer(int timerName)
{
	//switch case statement checking which Timer is selected
	switch (timerName){
	case 0:
		clearTimer(T1);//T1 is reset
		break;
	case 1:
		clearTimer(T2);//T2 is reset
		break;
	case 2:
		clearTimer(T3);//T3 is reset
		break;
	case 3:
		clearTimer(T4);//T4 is reset
		break;
	default:
		writeDebugStreamLine("Unrecognised Timer Selected"); //Invalid timer selected
		break;
	}
}

// GET TIMER COUNT
int readTimer(int timerName)
{
	//switch case statement checking which Timer is selected
	int timereturn;
	switch (timerName){
	case 0:
		timereturn = time1[T1];//returns value of T1
		break;
	case 1:
		timereturn = time1[T2];//returns value of T2
		break;
	case 2:
		timereturn = time1[T3];//returns value of T3
		break;
	case 3:
		timereturn = time1[T4];//returns value of T4
		break;
	default:
		writeDebugStreamLine("Unrecognised Timer Selected"); //Invalid timer selected
		break;
	}
	return timereturn; // return the count of the selected timer
}

// This function saturates an input number to be between the lower and upper limits defined by the user.
float saturate (float input, float lower, float upper){

	if (input > upper){ //if input is greater than upper limit, return upper limit
		return upper;
	}
	else if (input < lower){ //if input is less than lower limit, return lower limit
		return lower;
	}
	return input;
}

//--------------------------------- Arm stuff

// Predefined arm movement code
int arm_State = 0;				// a global variable that defines the state of the robot arm (-1 at lower limit, and 0 for anywhere else)
int checkArmRunning = 0;	// a golbal status variable used to indicate whether the checkArm task is running or not


void armDown(float percentPower);
void armUp(float percentPower);

// This task should always be running in parallel to the main task.
// It constantly monitors the arm limit switche and stops the arm
// motor if it's commanded to move in the direction where a limit switch
// is activated.
task checkArm(){
	checkArmRunning = 1;
	int arm_motorPwr;
	while(1){
		arm_motorPwr = motor[_motorArm];	// Check what the motor is doing
		if(SensorValue[_armLimit_low]){	// Check lower limit switch
			arm_State = -1;			// update arm_State (-1 = at lower limit)
			if (arm_motorPwr <0){motor[_motorArm] = 0;}	// stop if commaded to lower the arm
		}
		else{
			arm_State = 0; // update arm_State (0 = not at the lowest position)
		}
		wait1Msec(20); // repeat this loop approx every 20 ms (50Hz)
	}
}


// Function for lowering the arm all the way down until it hits the bottom limit switch
// Input is from 0 to 100 percent of motor power
void armDown (float percentPower){
	// Check if checkArm is running. if not, start it.
	if(!checkArmRunning	){startTask(checkArm);}
	// use absolute to prevent moving in the wrong direction
	int absPower = abs(round(percentPower/100.0*127));
	// Saturation
	if (absPower > 127) {
		absPower = 127;
	}
	motor[_motorArm] = -absPower;
	while(arm_State>-1){}
	motor[_motorArm] = 0; // should be redundant but just to be safe.
	wait1Msec(1000);
}

// Function for raising the arm all the way up until the arm encoder stops counting
// Input is from 0 to 100 percent of motor power
void armUp (float percentPower){
	// Check if checkArm is running. if not, start it.
	if(!checkArmRunning	){startTask(checkArm);}
	// use absolute to prevent moving in the wrong direction
	int absPower = abs(round(percentPower/100.0*127));
	// Saturation
	if (absPower > 127) {
		absPower = 127;
	}
	int EncoderReading = readSensor(EncoderArm);
	int PrevEncoderReading = EncoderReading -10; // just creating an offser to enter the loop
	motor[_motorArm] = absPower;
	while(abs(EncoderReading-PrevEncoderReading)>0){ // while the previous different between the previous encoder reading and the current one is not small, keep going
		PrevEncoderReading = EncoderReading;
		delay(20);
		EncoderReading = readSensor(EncoderArm);
	}
	motor[_motorArm] = 0; // should be redundant but just to be safe.
	wait1Msec(1000);
}

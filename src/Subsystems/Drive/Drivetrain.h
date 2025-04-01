#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include "Motor.h"
struct PIDController {
	float kP;
	float kI;
	float kD;
};

struct DrivetrainConfiguration {
	float maxRPM;
	float maxTurnRPM;
	int encoderCountsPerCell;
	int wallThreshold;	// in mm.

	PIDController distancePID;
	PIDController turnPID;

	int yawErrorThrewshold;
	int distanceErrorThreshold;
};

class Drivetrain {
   public:
	Drivetrain(const DrivetrainConfiguration& config, Motor* leftMotor, Motor* rightMotor);

	void driveForward();
	void stop();

	void driveForwardDistance(int distanceCounts);

	void rotateBy(int angleDegrees);
	void setAbsoluteHeading(int headingDegrees);

	void positionDrive(int offsetX, int offsetY);

	bool isWallAhead();

	void controlLoop();

   private:
	DrivetrainConfiguration config;

	Motor* leftMotor;
	Motor* rightMotor;

	Encoder* leftEncoder;
	Encoder* rightEncoder;

	volatile float currentRPM;
	volatile int currentYaw;
	volatile int desiredYaw;
	volatile bool isTurning;
	volatile bool isMoving;

	float turningIntegral;
	float turningLastError;
	float turningDerivative;
	float distanceIntegralL;
	float distanceLastErrorL;
	float distanceDerivativeL;
	float distanceIntegralR;
	float distanceLastErrorR;
	float distanceDerivativeR;

	absolute_time_t lastUpdateTime;
	double oldEncoderCountL;
	double oldEncoderCountR;

	// Helper methods.
	int positiveMod(int a, int b);
	int getAverageEncoderCount();

	// ToF distance calculations in mm.
	int readToFLeft();
	int readToFRight();
	int readToFFront();

	void executeTurningControl();
	void executeDistanceControl(int targetCounts);

	// IMU support
	volatile uint8_t imuBuffer[19];
	volatile int imuBufferIndex = 0;
	void initIMU();
	int getCurrentYaw() const;
	static void imuInterruptHandler();
	void updateIMU();
};
#endif
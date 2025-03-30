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
	float distanceIntegral;
	float distanceLastError;
	float distanceDerivative;

	std::chrono::steady_clock::time_point lastUpdateTime;

	// Helper methods.
	int positiveMod(int a, int b);
	int getAverageEncoderCount();
	float elapsedTimeSeconds();

	// ToF distance calculations in mm.
	int readToFLeft();
	int readToFRight();
	void updateIMU();

	void executeTurningControl();
	void executeDistanceControl(int targetCounts);
};
#endif
#include "Drivetrain.h"

Drivetrain::Drivetrain(const DrivetrainConfiguration& config, Motor* leftMotor, Motor* rightMotor) {
	this->config = config;
	this->leftMotor = leftMotor;
	this->rightMotor = rightMotor;

	leftEncoder = leftMotor->getEncoder();
	rightEncoder = rightMotor->getEncoder();

	currentRPM = config.maxRPM;
	currentYaw = desiredYaw = 0;
	isMoving = isTurning = false;
	turningIntegral = distanceIntegral = 0.0f;
	distanceLastError = distanceDerivative = 0.0f;

	lastUpdateTime = std::chrono::steady_clock::now();
}

int Drivetrain::positiveMod(int a, int b) {
	return (a % b + b) % b;
}

int Drivetrain::getAverageEncoderCount() {
	leftEncoder->update();
	rightEncoder->update();
	return (leftEncoder->getCount() + rightEncoder->getCount()) / 2;
}

float Drivetrain::elapsedTimeSeconds() {
	auto now = std::chrono::steady_clock::now();
	float dt = std::chrono::duration<float>(now - lastUpdateTime).count();
	lastUpdateTime = now;
	return dt;	// In seconds.
}

int Drivetrain::readToFLeft() {
	return 100;	 // FIXME
}

int Drivetrain::readToFRight() {
	return 100;	 // FIXME
}

void Drivetrain::updateIMU() {
	// FIXME
}

void Drivetrain::driveForward() {
	leftMotor->setThrottle(currentRPM / leftMotor->getMaxRPM());
	rightMotor->setThrottle(currentRPM / rightMotor->getMaxRPM());
	isMoving = true;
}

void Drivetrain::stop() {
	leftMotor->stop();
	rightMotor->stop();
	isMoving = false;
	isTurning = false;
}

void Drivetrain::driveForwardDistance(int cellCount) {
	float currPos = getAverageEncoderCount();
	float targetPos = currPos + cellCount * config.encoderCountsPerCell;
	distanceIntegral = distanceLastError = distanceDerivative = 0.0f;
	driveForward();

	while (isMoving) {
		int32_t leftRPM = leftMotor->getCurrentRPM();
		int32_t rightRPM = rightMotor->getCurrentRPM();
		int32_t leftPos = leftMotor->getCurrentPosition();
		int32_t rightPos = rightMotor->getCurrentPosition();
		printf("LRPM=%d RRPM=%d LP=%d RP=%d\n", leftRPM, rightRPM, leftPos, rightPos);
        
		float dt = elapsedTimeSeconds();
		int error = targetPos - (currPos + getAverageEncoderCount());
		distanceIntegral += error * dt;
		distanceDerivative = (error - distanceLastError) / dt;
		distanceLastError = error;

		float controlSignal = config.distancePID.kP * error + config.distancePID.kI * distanceIntegral + config.distancePID.kD * distanceDerivative;

		float adjustedRPM = std::min(currentRPM + controlSignal, config.maxRPM);
		leftMotor->setThrottle(adjustedRPM / leftMotor->getMaxRPM());
		rightMotor->setThrottle(adjustedRPM / rightMotor->getMaxRPM());

		if (fabs(error) < config.distanceErrorThreshold) {
			stop();
		}
	}
}

void Drivetrain::rotateBy(int angleDegrees) {
	stop();
	updateIMU();
	desiredYaw = positiveMod(currentYaw + angleDegrees, 360);
	turningIntegral = turningLastError = turningDerivative = 0.0f;
	isTurning = true;

	while (isTurning) {
		executeTurningControl();
	}
	stop();
	turningIntegral = 0.0f;
}

void Drivetrain::setAbsoluteHeading(int headingDegrees) {
	int angleDifference = headingDegrees - currentYaw;
	// Make it [-180, 180].
	(angleDifference >= 180) ? angleDifference -= 360 : angleDifference += 360;
	rotateBy(angleDifference);
}

// // We won't be needing this for now!
// void Drivetrain::positionDrive(int offsetX, int offsetY) {
// 	if (offsetY != 0) {
// 		driveForwardDistance(abs(offsetY) * config.encoderCountsPerCell);
// 		// FIXME: We could add a reverse distance!
// 	}

// 	if (offsetX != 0) {
// 		if (offsetX > 0) {
// 			rotateBy(90);
// 			driveForwardDistance(abs(offsetX) * config.encoderCountsPerCell);
// 		} else {
// 			rotateBy(-90);
// 			driveForwardDistance(abs(offsetX) * config.encoderCountsPerCell);
// 		}
// 	}
// }

void Drivetrain::executeTurningControl() {
	updateIMU();
	float dt = elapsedTimeSeconds();
	int error = desiredYaw - currentYaw;

	if (error > 180) {
		error -= 360;
	} else if (error < -180) {
		error += 360;
	}

	turningIntegral += error * dt;
	turningDerivative = (error - turningLastError) / dt;
	turningLastError = error;
	float controlSignal = config.turnPID.kP * error + config.turnPID.kI * turningIntegral + config.turnPID.kD * turningDerivative;

	if (abs(error) < config.yawErrorThrewshold) {
		isTurning = false;
		stop();
		return;
	}

	if (error > 0) {
		leftMotor->setThrottle(controlSignal / leftMotor->getMaxRPM());
		rightMotor->setThrottle(-controlSignal / rightMotor->getMaxRPM());
	} else {
		leftMotor->setThrottle(-controlSignal / leftMotor->getMaxRPM());
		rightMotor->setThrottle(controlSignal / rightMotor->getMaxRPM());
	}
}

bool Drivetrain::isWallAhead() {
	int leftDistance = readToFLeft();
	int rightDistance = readToFRight();

	return (leftDistance < config.wallThreshold || rightDistance < config.wallThreshold);
}

void Drivetrain::controlLoop() {
	updateIMU();
	leftEncoder->update();
	rightEncoder->update();

	if (isTurning) {
		executeTurningControl();
	}

	if (isMoving && isWallAhead()) {
		stop();
	}
}
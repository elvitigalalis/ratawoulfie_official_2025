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
	distanceIntegralL = distanceLastErrorL = distanceDerivativeL = 0.0f;
	distanceIntegralR = distanceLastErrorR = distanceDerivativeR = 0.0f;
	turningIntegral = turningLastError = turningDerivative = 0.0f;

	lastUpdateTime = get_absolute_time();
	oldEncoderCountL = 0;
	oldEncoderCountR = 0;
	initIMU();
}

void Drivetrain::initIMU() {
	// Initialize UART for IMU communication
	uart_init(UART_IMU, BAUD_RATE);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	uart_set_baudrate(UART_IMU, BAUD_RATE);
	uart_set_hw_flow(UART_IMU, false, false);
	uart_set_format(UART_IMU, DATA_BITS, STOP_BITS, PARITY);
	uart_set_fifo_enabled(UART_IMU, true);

	// Set up IMU UART interrupt (assumes UART0 is used for IMU)
	irq_set_enabled(UART0_IRQ, true);
	irq_set_exclusive_handler(UART0_IRQ, &Drivetrain::imuInterruptHandler);
	uart_get_hw(UART_IMU)->imsc = (bool_to_bit(true) << UART_UARTIMSC_RTIM_LSB);

	imuBufferIndex = 0;
}

int Drivetrain::positiveMod(int a, int b) {
	return (a % b + b) % b;
}

int Drivetrain::getAverageEncoderCount() {
	leftEncoder->update();
	rightEncoder->update();
	return (leftEncoder->getCount() + rightEncoder->getCount()) / 2;
}

int Drivetrain::readToFLeft() {
	return 100;	 // FIXME
}

int Drivetrain::readToFRight() {
	return 100;	 // FIXME
}

void Drivetrain::updateIMU() {
	uart_inst_t* UART_IMU = uart0;
	while (uart_is_readable(UART_IMU)) {
		uint8_t byte = uart_getc(UART_IMU);
		imuBuffer[imuBufferIndex++] = byte;
		// We have a full packet! (19 bytes)
		if (imuBufferIndex == 19) {
			uint8_t checksum = 0;
			// 2-->14
			for (int i = 2; i < 15; i++) {
				checksum += imuBuffer[i];
			}

			// Check if checksum is the same
			if (checksum == imuBuffer[18]) {
				int16_t yaw = (imuBuffer[4] << 8) | imuBuffer[5];
				printf("Yaw: %d\n", yaw);
                currentYaw = yaw;
			}
			imuBufferIndex = 0;	 // Resets the index for next packet of information.
		}
	}
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
	distanceIntegralL = distanceLastErrorL = distanceDerivativeL = distanceIntegralR = distanceLastErrorR = distanceDerivativeR = 0.0f;
	driveForward();

	while (isMoving) {
		int32_t leftRPM = leftMotor->getCurrentRPM();
		int32_t rightRPM = rightMotor->getCurrentRPM();
		int32_t leftPos = leftMotor->getCurrentPosition();
		int32_t rightPos = rightMotor->getCurrentPosition();
		printf("LRPM=%d RRPM=%d LP=%d RP=%d\n", leftRPM, rightRPM, leftPos, rightPos);

		float dt = absolute_time_diff_us(lastUpdateTime, get_absolute_time()) / 1000000.0f;
		int errorL = targetPos - leftMotor->getEncoder()->getCount();
		int errorR = targetPos - rightMotor->getEncoder()->getCount();

		// printf("Error: %i\n", (int32_t)error);
		distanceIntegralL += errorL * dt;
		distanceDerivativeL = (errorL - distanceLastErrorL) / dt;
		distanceLastErrorL = errorL;
		distanceIntegralR += errorR * dt;
		if (distanceIntegralR > 100) {
			distanceIntegralR = 100;
		}
		if (distanceIntegralR > 100) {
			distanceIntegralR = 100;
		}
		distanceDerivativeR = (errorR - distanceLastErrorR) / dt;
		distanceLastErrorR = errorR;
		float controlSignalL =
			config.distancePID.kP * errorL + config.distancePID.kI * distanceIntegralL + config.distancePID.kD * distanceDerivativeL;
		float controlSignalR =
			config.distancePID.kP * errorR + config.distancePID.kI * distanceIntegralR + config.distancePID.kD * distanceDerivativeR;

		float adjustedRPML = std::min(currentRPM + controlSignalL, config.maxRPM);
		float adjustedRPMR = std::min(currentRPM + controlSignalR, config.maxRPM);
		leftMotor->setThrottle(adjustedRPML / leftMotor->getMaxRPM());
		rightMotor->setThrottle(adjustedRPMR / rightMotor->getMaxRPM());

		if (fabs(errorL) < config.distanceErrorThreshold && fabs(errorR) < config.distanceErrorThreshold) {
			printf("Reached target position: %i\n", (int32_t)targetPos);
			stop();
			sleep_ms(2000);	 // FIXME: Remove this
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
	// turningIntegralR = turningIntegralL = 0.0f;
	// turningLastErrorR = turningLastErrorL = 0.0f;
	// turningDerivativeR = turningDerivativeL = 0.0f;
}

int Drivetrain::getCurrentYaw() const {
    return currentYaw;
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
	float dt = 0;  //FIXME elapsedTime
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
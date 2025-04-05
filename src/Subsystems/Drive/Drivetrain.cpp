#include "Drivetrain.h"

Drivetrain* Drivetrain::imuInstance = nullptr;

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
	// initIMU(); Uncomment this if you want to test this.
}

void Drivetrain::initIMU() {
	// Set the static instance pointer for use in the interrupt handler.
	// imuInstance = this;

	// Initialize UART for IMU communication.
	uart_init(UART_IMU, BAUD_RATE);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	uart_set_baudrate(UART_IMU, BAUD_RATE);
	uart_set_hw_flow(UART_IMU, false, false);
	uart_set_format(UART_IMU, DATA_BITS, STOP_BITS, PARITY);
	// uart_set_fifo_enabled(UART_IMU, true);

	// handleIMUInterrupt();
	// Set up IMU UART interrupt.
	// irq_set_exclusive_handler(UART0_IRQ, Drivetrain::imuInterruptHandler);
	// irq_set_enabled(UART0_IRQ, true);

	// Enable the UART receive timeout interrupt.
	// uart_get_hw(UART_IMU)->imsc = (1 << UART_UARTIMSC_RTIM_LSB);

	// Initialize the buffer index.
	imuBufferIndex = 0;
}

void Drivetrain::imuInterruptHandler() {
	// Static interrupt handler that delegates processing to the instance method.
	if (imuInstance) {
		imuInstance->handleIMUInterrupt();
	}
}

void Drivetrain::handleIMUInterrupt() {
	// printf("IMU Interrupt Triggered\n");
	// Read all available bytes from the IMU UART.
	while (uart_is_readable(UART_IMU)) {
		// printf("Reading IMU data\n");
		uint8_t byte = uart_getc(UART_IMU);
		imuBuffer[imuBufferIndex++] = byte;
		printf("Byte %d: %d\n", imuBufferIndex, byte);
		// Once a full packet (19 bytes) is received, process it.
		if (imuBufferIndex == 19) {
			processIMUPacket();
			imuBufferIndex = 0;	 // Reset index for next packet.
		}
	}
}

void Drivetrain::processIMUPacket() {
	// Calculate the checksum over bytes 2 to 14.
	uint8_t checksum = 0;
	for (int i = 2; i < 15; i++) {
		checksum += imuBuffer[i];
	}

	// Verify the checksum.
	if (checksum == imuBuffer[18]) {
		// Extract yaw from bytes 4 and 3.
		int16_t yaw = (imuBuffer[4] << 8) | imuBuffer[3];
		printf("Yaw: %d\n", yaw);
		currentYaw = yaw;
	} else {
		printf("IMU checksum error: calculated %d, expected %d\n", checksum, imuBuffer[18]);
	}
}


// I2C IMU functions

// Initialise Accelerometer Function
void Drivetrain::accel_init(void){
    // Check to see if connection is correct
    sleep_ms(1000); // Add a short delay to help BNO005 boot up
    uint8_t reg = 0x00;
    uint8_t chipID[1];
    i2c_write_blocking(I2C_PORT, addr, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr, chipID, 1, false);

    if(chipID[0] != 0xA0){
        while(1){
            printf("Chip ID Not Correct - Check Connection!");
            sleep_ms(5000);
        }
    }

    // Use internal oscillator
    uint8_t data[2];
    data[0] = 0x3F;
    data[1] = 0x40;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);

    // Reset all interrupt status bits
    data[0] = 0x3F;
    data[1] = 0x01;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);

    // Configure Power Mode
    data[0] = 0x3E;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(50);

    // Defaul Axis Configuration
    data[0] = 0x41;
    data[1] = 0x24;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);

    // Default Axis Signs
    data[0] = 0x42;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);

    // Set units to m/s^2
    data[0] = 0x3B;
    data[1] = 0b0001000;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(30);

    // Set operation to acceleration only
    data[0] = 0x3D;
    data[1] = 0x0C;
    i2c_write_blocking(I2C_PORT, addr, data, 2, true);
    sleep_ms(100);
}

void Drivetrain::IMUinit(void){
    stdio_init_all(); // Initialise STD I/O for printing over serial

    // Configure the I2C Communication
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    // Call accelerometer initialisation function
    accel_init();

    
	IMUloop();
    
}

void Drivetrain::IMUloop(void) {
	uint8_t accel[6]; // Store data from the 6 acceleration registers
    int16_t accelX, accelY, accelZ; // Combined 3 axis data
    float f_accelX, f_accelY, f_accelZ; // Float type of acceleration data
    uint8_t val = 0x08; // Start register address
	// Infinite Loop
    while(1){
        i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
        i2c_read_blocking(I2C_PORT, addr, accel, 6, false);

        accelX = ((accel[1]<<8) | accel[0]);
        accelY = ((accel[3]<<8) | accel[2]);
        accelZ = ((accel[5]<<8) | accel[4]);

        f_accelX = accelX / 100.00;
        f_accelY = accelY / 100.00;
        f_accelZ = accelZ / 100.00;

        // Print to serial monitor
        printf("X: %6.2f    Y: %6.2f    Z: %6.2f\n", f_accelX, f_accelY, f_accelZ);
        sleep_ms(300);
    }
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
		if (distanceIntegralL > 100) {
			distanceIntegralL = 100;
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

		lastUpdateTime = get_absolute_time();

		if (fabs(errorL) < config.distanceErrorThreshold && fabs(errorR) < config.distanceErrorThreshold) {
			printf("Reached target position: %i\n", (int32_t)targetPos);
			stop();
			sleep_ms(2000);	 // FIXME: Remove this
		}
	}
}

void Drivetrain::rotateBy(int angleDegrees) {
	stop();
	// updateIMU();
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
	uint64_t currentTime = get_absolute_time();
	float dt = absolute_time_diff_us(lastUpdateTime, currentTime) / 1000000.0f;
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
	// updateIMU();
	leftEncoder->update();
	rightEncoder->update();

	if (isTurning) {
		executeTurningControl();
	}

	if (isMoving && isWallAhead()) {
		stop();
	}
}

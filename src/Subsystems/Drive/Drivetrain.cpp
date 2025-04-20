#include "Drivetrain.h"

Drivetrain* Drivetrain::imuInstance = nullptr;

Drivetrain::Drivetrain(const DrivetrainConfiguration& config, Motor* leftMotor, Motor* rightMotor) {
    this->config = config;
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;

    leftEncoder = leftMotor->getEncoder();
    rightEncoder = rightMotor->getEncoder();

    // currentRPM = 0.0f;
    currentYaw = desiredYaw = 0;
    isMoving = isTurning = false;
    distanceIntegralL = distanceLastErrorL = distanceDerivativeL = 0.0f;
    distanceIntegralR = distanceLastErrorR = distanceDerivativeR = 0.0f;
    turningIntegral = turningLastError = turningDerivative = 0.0f;

    lastUpdateTime = get_absolute_time();
    oldEncoderCountL = 0;
    oldEncoderCountR = 0;
    initIMU();
    // initToF();
}

void Drivetrain::initIMU() {
    // Set the static instance pointer for use in the interrupt handler.
    imuInstance = this;

    // Initialize UART for IMU communication.
    uart_init(UART_IMU, BAUD_RATE);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_baudrate(UART_IMU, BAUD_RATE);
    uart_set_hw_flow(UART_IMU, false, false);
    uart_set_format(UART_IMU, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_IMU, true);

    // Set up IMU UART interrupt.
    irq_set_exclusive_handler(UART1_IRQ, Drivetrain::imuInterruptHandler);
    irq_set_enabled(UART1_IRQ, true);

    // Enable the UART receive timeout interrupt.
    uart_get_hw(UART_IMU)->imsc = (1 << UART_UARTIMSC_RTIM_LSB);

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
    // Read all available bytes from the IMU UART.
    while (uart_is_readable(UART_IMU)) {
        uint8_t byte = uart_getc(UART_IMU);
        imuBuffer[imuBufferIndex++] = byte;

        // Once a full packet (19 bytes) is received, process it.
        if (imuBufferIndex == 19) {
            processIMUPacket();
            imuBufferIndex = 0;  // Reset index for next packet.
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
        printf("Yaw: %f\n", yaw / 100.0f);
        currentYaw = yaw / 100.0f;
    } else {
        printf("IMU checksum error: calculated %d, expected %d\n", checksum, imuBuffer[18]);
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

void Drivetrain::initToF() {
    gpio_init(XSHUT_FRONT);
    gpio_set_dir(XSHUT_FRONT, GPIO_OUT);
    gpio_init(XSHUT_LEFT);
    gpio_set_dir(XSHUT_LEFT, GPIO_OUT);
    gpio_init(XSHUT_RIGHT);
    gpio_set_dir(XSHUT_RIGHT, GPIO_OUT);

    // Reset all sensors
    gpio_put(XSHUT_FRONT, 0);
    gpio_put(XSHUT_LEFT, 0);
    gpio_put(XSHUT_RIGHT, 0);
    sleep_ms(100);

    // Initialize front sensor
    gpio_put(XSHUT_FRONT, 1);
    sleep_ms(200);
    frontTOF.I2cDevAddr = 0x29;
    frontTOF.comms_type = 1;
    frontTOF.comms_speed_khz = 400;
    VL53L0X_dev_i2c_default_initialise(&frontTOF, VL53L0X_DEFAULT_MODE);
    int status = VL53L0X_SetDeviceAddress(&frontTOF, 0x30);
    printf("Front sensor address set status: %d\n", status);
    frontTOF.I2cDevAddr = 0x30;
    VL53L0X_SetDeviceMode(&frontTOF, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    VL53L0X_StartMeasurement(&frontTOF);
    sleep_ms(100);

    // Initialize left sensor
    gpio_put(XSHUT_LEFT, 1);
    sleep_ms(200);
    leftTOF.I2cDevAddr = 0x29;
    leftTOF.comms_type = 1;
    leftTOF.comms_speed_khz = 400;
    VL53L0X_dev_i2c_default_initialise(&leftTOF, VL53L0X_DEFAULT_MODE);
    status = VL53L0X_SetDeviceAddress(&leftTOF, 0x31);
    printf("Left sensor address set status: %d\n", status);
    leftTOF.I2cDevAddr = 0x31;
    VL53L0X_SetDeviceMode(&leftTOF, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    VL53L0X_StartMeasurement(&leftTOF);
    sleep_ms(100);

    // Initialize right sensor
    gpio_put(XSHUT_RIGHT, 1);
    sleep_ms(200);
    rightTOF.I2cDevAddr = 0x29;
    rightTOF.comms_type = 1;
    rightTOF.comms_speed_khz = 400;
    VL53L0X_dev_i2c_default_initialise(&rightTOF, VL53L0X_DEFAULT_MODE);
    status = VL53L0X_SetDeviceAddress(&rightTOF, 0x32);
    printf("Right sensor address set status: %d\n", status);
    rightTOF.I2cDevAddr = 0x32;
    VL53L0X_SetDeviceMode(&rightTOF, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    VL53L0X_StartMeasurement(&rightTOF);
    sleep_ms(100);
}

float Drivetrain::checkFrontWallDistance() {
    VL53L0X_RangingMeasurementData_t distance;
    VL53L0X_GetRangingMeasurementData(&frontTOF, &distance);
    VL53L0X_ClearInterruptMask(&frontTOF, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    return distance.RangeMilliMeter;
}

float Drivetrain::checkLeftWallDistance() {
    VL53L0X_RangingMeasurementData_t distance;
    VL53L0X_GetRangingMeasurementData(&leftTOF, &distance);
    VL53L0X_ClearInterruptMask(&leftTOF, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    return distance.RangeMilliMeter;
}

float Drivetrain::checkRightWallDistance() {
    VL53L0X_RangingMeasurementData_t distance;
    VL53L0X_GetRangingMeasurementData(&rightTOF, &distance);
    VL53L0X_ClearInterruptMask(&rightTOF, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    return distance.RangeMilliMeter;
}

bool Drivetrain::checkFrontWall() {
    return checkFrontWallDistance() < WALLCUTOFF;
}

bool Drivetrain::checkLeftWall() {
    return checkLeftWallDistance() < WALLCUTOFF;
}

bool Drivetrain::checkRightWall() {
    return checkRightWallDistance() < WALLCUTOFF;
}

void Drivetrain::driveForward() {
    leftMotor->setRPM(currentRPM);
    rightMotor->setRPM(currentRPM);
    isMoving = true;
}

void Drivetrain::stop() {
    leftMotor->stop();
    rightMotor->stop();
    isMoving = false;
    isTurning = false;
}

void Drivetrain::driveForwardDistance(float cellCount) {
    float targetPosL = leftMotor->getEncoder()->getCount() + cellCount * config.encoderCountsPerCell;
    float targetPosR = rightMotor->getEncoder()->getCount() + cellCount * config.encoderCountsPerCell;
    distanceIntegralL = distanceLastErrorL = distanceDerivativeL = distanceIntegralR = distanceLastErrorR = distanceDerivativeR = 0.0f;
    driveForward();
    absolute_time_t now2 = get_absolute_time();
    while (isMoving) {
        // printf("LRPM=%d RRPM=%d LP=%d RP=%d\n", leftRPM, rightRPM, leftPos, rightPos);

        float dt = absolute_time_diff_us(lastUpdateTime, get_absolute_time()) / 1000000.0f;
        if (dt <= 0.1f) {
            continue;
        }
        int errorL = (targetPosL - leftMotor->getEncoder()->getCount());
        int errorR = (targetPosR - rightMotor->getEncoder()->getCount());

        LOG_DEBUG(std::to_string(leftMotor->getCurrentRPM()) + "," + std::to_string(absolute_time_diff_us(now2, get_absolute_time()) / 1e6f) + "," +
                  std::to_string(rightMotor->getCurrentRPM()));
        // LOG_DEBUG(std::to_string(leftMotor->getCurrentRPM()) + "," + std::to_string(absolute_time_diff_us(now2, get_absolute_time()) / 1e6f) + "," +
        //           std::to_string(rightMotor->getCurrentRPM()));
        // printf("ErrorLR=%f\n", (errorL + errorR) / 2.0 / config.encoderCountsPerCell);

        // Left
        //  printf("Error: %i\n", (int32_t)error);
        // distanceIntegralL += errorL * dt;
        // if (distanceIntegralL > 100) {
        //     distanceIntegralL = 100;
        // }
        distanceDerivativeL = (errorL - distanceLastErrorL) / dt;
        distanceLastErrorL = errorL;
        float controlSignalL;
        if (fabs(errorL) < 50) {
            controlSignalL = 0.0f;
        } else if (errorL < 0) {
            controlSignalL = -150.0f;
            // std::min(-150.0f, config.distancePID.kP * errorL + config.distancePID.kI * distanceIntegralL + config.distancePID.kD * distanceDerivativeL);
        } else {
            controlSignalL = 150.0f;
            // std::max(150.0f, config.distancePID.kP * errorL + config.distancePID.kI * distanceIntegralL + config.distancePID.kD * distanceDerivativeL);
        }

        // Right
        // distanceIntegralR += errorR * dt;
        // if (distanceIntegralR > 100) {
        //     distanceIntegralR = 100;
        // }
        distanceDerivativeR = (errorR - distanceLastErrorR) / dt;
        distanceLastErrorR = errorR;
        float controlSignalR;
        if (fabs(errorR) < 50) {
            controlSignalR = 0.0f;
        } else if (errorR < 0) {
            controlSignalR = -150.0f;
            // std::min(-150.0f, config.distancePID.kP * errorR + config.distancePID.kI * distanceIntegralR + config.distancePID.kD * distanceDerivativeR);
        } else {
            controlSignalR = 150.0f;
            // std::max(150.0f, config.distancePID.kP * errorR + config.distancePID.kI * distanceIntegralR + config.distancePID.kD * distanceDerivativeR);
        }

        // printf("Control SignalL: %f\n", controlSignalL);
        float adjustedRPML = std::min(std::max(controlSignalL, -config.maxRPM), config.maxRPM);

        // printf("Control SignalR: %f\n", controlSignalR);
        float adjustedRPMR = std::min(std::max(controlSignalR, -config.maxRPM), config.maxRPM);

        // adjustedRPML = 200.0f;
        // adjustedRPMR = 200.0f;

        // Adjusts RPM of either motor if angle is off
        float error = abs(currentYaw - desiredYaw);
        if (error > 180) {
            error = 360 - error;
        }
        bool right = false;
        if (currentYaw - desiredYaw > 0) {
            right = false;
        } else {
            right = true;
        }
        if (abs(currentYaw - desiredYaw) > 180) {
            right = !right;
        }
        if (right) {
            error *= -1;
        }
        leftMotor->setRPM(std::min(adjustedRPML - 1.0f * error, (adjustedRPML + 100)));
        rightMotor->setRPM(std::min(adjustedRPMR + 1.0f * error, (adjustedRPMR + 100)));

        // printf("Adjusted RPML: %f\n", adjustedRPML);
        // printf("Adjusted RPMR: %f\n", adjustedRPMR);
        // leftMotor->setRPM(adjustedRPML);
        // rightMotor->setRPM(adjustedRPMR);
        lastUpdateTime = get_absolute_time();

        if (fabs(errorL) < config.distanceErrorThreshold) {
            leftMotor->stop();
        }
        if (fabs(errorR) < config.distanceErrorThreshold) {
            // printf("Reached target position: %i\n", (int32_t)targetPos);
            rightMotor->stop();
        }
        if (fabs(errorL) < config.distanceErrorThreshold && fabs(errorR) < config.distanceErrorThreshold) {
            break;
        }
        // LOG_DEBUG("Pos of Left Motor: " + std::to_string(leftMotor->getCurrentPosition()));
        // LOG_DEBUG("Pos of Right Motor: " + std::to_string(rightMotor->getCurrentPosition()));

        // sleep_ms(15);  // FIXME
    }
}

void Drivetrain::rotateBy(int angleDegrees) {
    stop();
    // updateIMU();
    if (currentYaw + angleDegrees > 180) {
        angleDegrees -= 360;
    } else if (currentYaw + angleDegrees < -180) {
        angleDegrees += 360;
    }
    desiredYaw = angleDegrees;
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

void Drivetrain::turnLeft() {
    uint64_t currentTime = get_absolute_time();
    float dt = absolute_time_diff_us(lastUpdateTime, currentTime) / 1000000.0f;
    desiredYaw = currentYaw - 90;
    if (desiredYaw < -180) {
        desiredYaw += 360;
    } else if (desiredYaw > 180) {
        desiredYaw -= 360;
    }
    int error = desiredYaw - currentYaw;

    // if (error > 180) {
    //     error -= 360;
    // } else if (error < -180) {
    //     error += 360;
    // }

    turningIntegral += error * dt;
    turningDerivative = (error - turningLastError) / dt;
    turningLastError = error;
    float controlSignal = config.turnPID.kP * error + config.turnPID.kI * turningIntegral + config.turnPID.kD * turningDerivative;

    if (abs(error) < config.yawErrorThrewshold) {
        isTurning = false;
        stop();
        return;
    }

    leftMotor->setRPM(controlSignal);
    rightMotor->setRPM(controlSignal / 5);
}

void Drivetrain::turnRight() {
    uint64_t currentTime = get_absolute_time();
    float dt = absolute_time_diff_us(lastUpdateTime, currentTime) / 1000000.0f;
    desiredYaw = currentYaw - 90;
    if (desiredYaw < -180) {
        desiredYaw += 360;
    } else if (desiredYaw > 180) {
        desiredYaw -= 360;
    }
    int error = desiredYaw - currentYaw;

    // if (error > 180) {
    //     error -= 360;
    // } else if (error < -180) {
    //     error += 360;
    // }

    turningIntegral += error * dt;
    turningDerivative = (error - turningLastError) / dt;
    turningLastError = error;
    float controlSignal = config.turnPID.kP * error + config.turnPID.kI * turningIntegral + config.turnPID.kD * turningDerivative;

    if (abs(error) < config.yawErrorThrewshold) {
        isTurning = false;
        stop();
        return;
    }

    leftMotor->setRPM(controlSignal / 5);
    rightMotor->setRPM(controlSignal);
}

void Drivetrain::setAbsoluteHeading(int headingDegrees) {
    int angleDifference = headingDegrees - currentYaw;
    // Make it [-180, 180].
    if (angleDifference >= 180) {
        angleDifference -= 360;
    } else if (angleDifference < -180) {
        angleDifference += 360;
    }
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
    LOG_DEBUG(std::to_string(currentYaw));

    // if (error > 180) {
    //     error -= 360;
    // } else if (error < -180) {
    //     error += 360;
    // }

    turningIntegral += error * dt;
    turningDerivative = (error - turningLastError) / dt;
    turningLastError = error;
    float controlSignal = config.turnPID.kP * error + config.turnPID.kI * turningIntegral + config.turnPID.kD * turningDerivative + 50.0f;

    if (abs(error) < config.yawErrorThrewshold) {
        isTurning = false;
        stop();
        return;
    }

    // LOG_DEBUG("Pos of Left Motor: " + std::to_string(leftMotor->getCurrentPosition()));
    // LOG_DEBUG("Pos of Right Motor: " + std::to_string(rightMotor->getCurrentPosition()));
    // LOG_DEBUG("Control signal" + std::to_string(controlSignal));
    leftMotor->setRPM(controlSignal);
    rightMotor->setRPM(-1.0f * controlSignal);
    // sleep_ms(15);
}

void Drivetrain::controlLoop() {
    leftEncoder->update();
    rightEncoder->update();

    if (isTurning) {
        executeTurningControl();
    }

    if (isMoving && isWallAhead()) {
        stop();
    }
}
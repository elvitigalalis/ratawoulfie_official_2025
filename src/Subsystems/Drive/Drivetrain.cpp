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
	imuInstance = this;

	// Initialize UART for IMU communication.
	uart_init(UART_IMU, BAUD_RATE);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	uart_set_baudrate(UART_IMU, BAUD_RATE);
	uart_set_hw_flow(UART_IMU, false, false);
	uart_set_format(UART_IMU, DATA_BITS, STOP_BITS, PARITY);
	uart_set_fifo_enabled(UART_IMU, true);

	// Set up IMU UART interrupt.
	irq_set_exclusive_handler(UART0_IRQ, Drivetrain::imuInterruptHandler);
	irq_set_enabled(UART0_IRQ, true);

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

// Lidar Functions
void Drivetrain::lidarInterruptHandler() {
	// Static interrupt handler that delegates processing to the instance method.
	if (lidarInstance) {
		lidarInstance->readLidar();
	}
}

//FIFO Queue: Add's Data
void Drivetrain::queue(uint8_t data){

if (count < BUFFER_SIZE){
	fifoQueue[tail] = data;
	tail = (tail + 1) % BUFFER_SIZE;
	count++;
} else {                                                                   
	printf("FIFO Overflow. Data Lost.\n");
}
}

//FIFO Queue: Remove's and Returns old data
int Drivetrain::dequeue(){
	if (count > 0){
		uint8_t data = fifoQueue[head];
		head = (head + 1) % BUFFER_SIZE;
		count--;
		return data;
	}

	return -1; //FIFO empty
}


//reads LIDAR data
// Static
void Drivetrain::readLidar(){
	while(uart_is_readable(UART_LIDAR)){
		uint8_t recievedByte = uart_getc(UART_LIDAR);
		queue(recievedByte);
	}
}

//LIDAR initialization
void Drivetrain::initLidar(){
    uart_init(UART_LIDAR, BAUD_RATE);  //initialize UART with baud rate
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART); //configures txPin
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART); //configures rxPin
    uart_set_hw_flow(UART_LIDAR, false, false); //disables RTS and CTS (flow control not required)
    uart_set_format(UART_LIDAR, 8, 1, UART_PARITY_NONE); // sets data to 8 bits, 1 stop bit per frame, turns off parity
    uart_set_fifo_enabled(UART_LIDAR, false); //disables FirstIn, FirstOut buffer (data will be processed as it arrives w/o hardware buffer)

    irq_set_exclusive_handler(21, Drivetrain::lidarInterruptHandler);  //interrupt
    irq_set_enabled(21, true);
    uart_set_irq_enables(UART_LIDAR, true, false);
    printf("LIDAR Sensors Initialized.\n");
}

//process LIDAR data (extract distances)
void Drivetrain::processLidarData(){
    printf("\nLIDAR Distance Data:\n");

    for(int i = 10; i < 330; i +=2){
        int byte1 = (int) dequeue();
        int byte2 = (int) dequeue();
        if(byte1 == -1 || byte2 == -1) break; //check if data was retrieved

        int distance = (byte1 | (byte2 << 8)) & 0x01FF;
        int lidarPointIndex = (1-8) / 2;

        if (lidarPointIndex < 81) {
            lidarPointIndex = 80 - lidarPointIndex;
        } else {
            lidarPointIndex = -lidarPointIndex + 240;
        }

        int address = fifoQueue[4]; //makes sure address is correct

        if (address == 0x1)
            lidarPoints[159 - (lidarPointIndex - 1)] = distance;
            if (address == 0x2)
            lidarPoints[320 - lidarPointIndex] = distance;
    }

    
    //display data 
    for(int i = 0; i < 360; i++){
        printf("Angle %d: %d mm\n", i, lidarPoints[i]);

    }

}


//data translation to messages
void Drivetrain::translateDataToMessage(int distance){
    if(distance == 0){
        printf("INVALID READING\n");
    }
    else if(distance == 0xFFFF){
        printf("OUT OF DETECTION RANGE\n");
    }
    else if (distance < 200){
        printf("Object is extremely close. (%d cm)\n", distance / 10.0);
    }
    else if(distance < 500) {
        printf("Object detected nearby. (%d cm)\n", distance / 10.0);
    }
    else if(distance < 1500) {
        printf("Object detected at %d cm\n", distance / 10.0);
    }
    else{
        printf("No obstacles detected.\n");
    }
}

int Drivetrain::lidarMain() { //TODO: This is the format for the test code put this in the main drivetrain loop
    stdio_init_all();
    sleep_ms(1000);
    initLidar();

    uint8_t command[9] = {0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x63, 0x00, 0x00, 0x63};
    uart_write_blocking(UART_LIDAR, command, 9);

    while(true) {   
        
        if (count > 329) { // needs enough data before processing
            processLidarData();
        }
        sleep_ms(10);
       
    }
    return 0;   //displays when program has executed sucessfully
}

void Drivetrain::cartesianConvert(){
    /*
        Takes 320 LIDAR points and converts them to 
        x, y coordinates.
    */
    for (int i = 0; i < 80; i++){
        int r = lidarPoints[i];
        if (r == 0){
            xcor[i] = 0;
            ycor[i] = 0;
        }
        else{
            float theta = 2 * 3.1415 * (52.5 / 79 * i - 52.5)/ 360;
            xcor[i] = r * cos(theta);
            ycor[i] = r * sin(theta);
        }
    }   
    for (int i = 80; i < 160; i++){
        int r = lidarPoints[i];
        if (r == 0){
            xcor[i] = 0;
            ycor[i] = 0;
        }
        else{
            float theta = 2 * 3.1415 * (52.5 / 79 * (i - 80))/ 360;
            xcor[i] = r * cos(theta);
            ycor[i] = r * sin(theta);
        }
    }
    for (int i = 160; i < 240; i++){
        int r = lidarPoints[i];
        if (r == 0){
            xcor[i] = 0;
            ycor[i] = 0;
        } else {
            float theta = 2 * 3.1415 * (180 - 52.5 / 79 * (239 - i))/ 360;
            xcor[i] = r * cos(theta);
            ycor[i] = r * sin(theta);
        }
    }
    for (int i = 240; i < 320; i++){
        int r = lidarPoints[i];
        if (r == 0){
            xcor[i] = 0;
            ycor[i] = 0;
        } else {
            float theta = 2 * 3.1415 * (180 + 52.5 / 79 * (i - 240))/ 360;
            xcor[i] = r * cos(theta);
            ycor[i] = r * sin(theta);
        }
    }
}

float Drivetrain::checkFrontWallDistance(){
    /*
        Takes LIDAR points 150-170 (front points) and averages the 
        y distance.
    */
    int pointOne = 150; // first point corresponding to front wall (position in array)
    int pointTwo = 170; // last point corresponding to front wall
    int sum = 0;
    int zeroCount = 0;
    cartesianConvert();
    for (int i = pointOne; i < pointTwo + 1; i++){
        if (ycor[i] == 0){
            zeroCount += 1;
        } else {sum += ycor[i];}
    }
    return sum / (float(pointTwo - pointOne + 1) - zeroCount);
}

bool Drivetrain::checkFrontWall(){
    /*
        Counts number of points within a space in front of the mouse to determine if there's a wall.
    */
    cartesianConvert();
    int count = 0;
    for (int i = 0; i < 320; i++){
        if (ycor[i] < 70 && ycor[i] > 20){
            if (xcor[i] < 65 && xcor[i] > -65){
                count += 1;
            }
        }
    }
    return count > 45;
}

bool Drivetrain::checkRightWall(){
    cartesianConvert();
    int count = 0;
    for (int i = 0; i < 320; i++){
        if (ycor[i] < 130 && ycor[i] > -60){
            if (xcor[i] < 90 && xcor[i] > 30){
                count += 1;
            }
        }
    }
    return count > 50;
}

bool Drivetrain::checkLeftWall(){
    cartesianConvert();
    int count = 0;
    for (int i = 0; i < 320; i++){
        if (ycor[i] < 130 && ycor[i] > -60){
            if (xcor[i] < -30 && xcor[i] > -90){
                count += 1;
            }
        }
    }
    return count > 50;
}

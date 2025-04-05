#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include "Motor.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/i2c.h"

#define MULTICORE // Comment out if not using multicore
// #ifdef MULTICORE
// mutex_t imu_mutex;
// #endif

#ifdef MULTICORE
extern mutex_t imu_mutex;
#define IMU_LOCK() mutex_enter_blocking(&imu_mutex)
#define IMU_UNLOCK() mutex_exit(&imu_mutex)
#define IMU_MUTEX_INIT() mutex_init(&imu_mutex)
#else
#define IMU_LOCK()
#define IMU_UNLOCK() 
#define IMU_MUTEX_INIT()
#endif

#define UART_IMU uart1 //uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 8 //0
#define UART_RX_PIN 9 //1
#define I2C_PORT i2c0

static int addr = 0x28;

// UART receive timeout interrupt bit.
// #define UART_UARTIMSC_RTIM_LSB 6
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

	// Lidar Public Functions 
	// TODO Overload the following functions to have a definition for Lidar and Tof
	bool checkLeftWall();
	bool checkRightWall();
	bool checkFrontWall();

	// IMU support
	volatile uint8_t imuBuffer[19];	 // Buffer to hold IMU packet data.
	volatile int imuBufferIndex;	 // Index for the IMU buffer.

	void initIMU();
	void handleIMUInterrupt();
	void processIMUPacket();
	void IMUinit(void);
	void accel_init(void);
	void IMUloop(void);

   private:
	DrivetrainConfiguration config;
	static Drivetrain* imuInstance;
	static Drivetrain* lidarInstance;

	Motor* leftMotor;
	Motor* rightMotor;

	Encoder* leftEncoder;
	Encoder* rightEncoder;

	// LIDAR Data
	// volatile char buffer[600]; // Buffer for LIDAR readings
	// volatile int bufferIndex = 0;
	// int16_t lidarPoints[320]; // Stores LIDAR data for processing
	// volatile int lidarPointIndex;
	// float frontWallDist = 10000; // Distance wall is from mouse
	//define necessary values
	// #define UART_LIDAR uart1
	// #define BAUD_RATE 921600
	// #define UART_TX_PIN 4 //txPin (system serial port output)
	// #define UART_RX_PIN 5 //rxPin (system serial port input)


	// /*the buffer will store recieved bytes from the sensor via UART
	// the index variable will keep track of where to store the next byte;
	// once buffer is full, it will reset to 0. */
	// #define BUFFER_SIZE 330  //modify as needed (if all data doesn't show, increase value)
	// uint8_t fifoQueue[BUFFER_SIZE]; // TODO: add constants
	// int head = 0, tail = 0, count = 0;  //FIFO
	// int lidarPoints[360];   //stores distances at different angles
	// float xcor[320]; // Arrays for processed LIDAR data
	// float ycor[320];


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

	// Lidar helper methods
	static void lidarInterruptHandler();
	void queue(uint8_t data);
	int dequeue();
	void readLidar();
	void initLidar();
	void processLidarData();
	void translateDataToMessage(int distance);
	int lidarMain();
	void cartesianConvert(); // Turning LIDAR values to x, y coordinates
	float checkFrontWallDistance();

	void executeTurningControl();
	void executeDistanceControl(int targetCounts);

	// // IMU support
	// volatile uint8_t imuBuffer[19];	 // Buffer to hold IMU packet data.
	// volatile int imuBufferIndex;	 // Index for the IMU buffer.
	// void initIMU();
	static void imuInterruptHandler();
	// void handleIMUInterrupt();
	// void processIMUPacket();
};
#endif
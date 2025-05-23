#include "Constants.h"

namespace Constants {

namespace RobotConstants {

const int leftMotorPin1 = 19;
const int leftMotorPin2 = leftMotorPin1 - 1;
const int leftMotorEncoderPin1 = leftMotorPin1 + 1;
const int leftMotorEncoderPin2 = leftMotorEncoderPin1 + 1;

const int rightMotorPin1 = 6;
const int rightMotorPin2 = rightMotorPin1 + 1;
const int rightMotorEncoderPin1 = rightMotorPin2 + 1;
const int rightMotorEncoderPin2 = rightMotorEncoderPin1 + 1;
// const int rightMotorEncoderPin1 = rightMotorPin2 + 2;
// const int rightMotorEncoderPin2 = rightMotorPin2 + 1;

const float eventsPerRev = 1400.0f;	// Amount of encoder ticks per revolution.
const float maxRPM = 200.0f;

// FOR MOTORS (RPM TO PWM)
const float kP = 0.0f;
const float kI = 0.0f;
const float kD = 0.0f;

const float PIDtime = 50.0f; // in ms.

// DrivetrainConfiguration config = [] {
// 	DrivetrainConfiguration cfg;
// 	cfg.maxRPM = 200.0f;
// 	cfg.maxTurnRPM = 200.0f;
// 	cfg.encoderCountsPerCell = 635;	 // 180 / (32.5 mm (wheel diameter) * 3.14 (pi)) * 360 (encoder counts per rev). = 634.6609.
// 	cfg.wallThreshold = 50;			 // mm.
// 	cfg.distancePID = {0.01f, 0.0f, 0.0f};
// 	cfg.turnPID = {0.1f, 0.0f, 0.0f};
// 	cfg.yawErrorThrewshold = 3;
// 	cfg.distanceErrorThreshold = 5;
// 	return cfg;
// }();
}  // namespace RobotConstants

namespace MouseConstants {
const string mouseName = "Ratawoulfie";
const vector<array<int, 2>> possibleMouseDirections = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
const array<int, 2> startingMousePosition = {0, 0};
const array<int, 2> startingMouseDirection = {0, 1};
}  // namespace MouseConstants

namespace uartConstants {}
namespace MazeConstants {
const int numRows = 16;
const int numCols = 16;
const int goalX = 8;
const int goalY = 8;

vector<array<int, 2>> getGoalCells() {
	return {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
}

const char startCellColor = 'B';
const string startCellText = "Start";
const char goalCellColor = 'G';
const string goalCellText = "Goal";
const char goalPathColor = 'C';
const char returnPathColor = 'Y';

bool showGrid = true;
bool showPath = true;
}  // namespace MazeConstants
}  // namespace Constants
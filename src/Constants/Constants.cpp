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

const float eventsPerRev = 360.0f;	// Amount of encoder ticks per revolution.
const float maxRPM = 400.0f;

const float kP = 10.0f;
const float kI = 1.0f;
const float kD = 0.5f;
}  // namespace RobotConstants

namespace MouseConstants {
const string mouseName = "Ratawoulfie";
const vector<array<int, 2>> possibleMouseDirections = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
const array<int, 2> startingMousePosition = {0, 0};
const array<int, 2> startingMouseDirection = {0, 1};
}  // namespace MouseConstants

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
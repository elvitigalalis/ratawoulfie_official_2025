#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <array>
#include <string>
#include <vector>

#include "../Subsystems/Drive/Drivetrain.h"

using std::array;
using std::string;
using std::vector;
namespace Constants {

namespace RobotConstants {
extern const int leftMotorPin1, leftMotorPin2, leftMotorEncoderPin1, leftMotorEncoderPin2;
extern const int rightMotorPin1, rightMotorPin2, rightMotorEncoderPin1, rightMotorEncoderPin2;

extern const float eventsPerRev;  // Amount of encoder ticks per revolution.
extern const float maxRPM;

extern const float kP, kI, kD;

extern const float PIDtime;

// extern DrivetrainConfiguration config;

}  // namespace RobotConstants

namespace MouseConstants {
extern const string mouseName;
extern const vector<array<int, 2>> possibleMouseDirections;
extern const array<int, 2> startingMousePosition;
extern const array<int, 2> startingMouseDirection;
}  // namespace MouseConstants

namespace MazeConstants {
extern const int numRows, numCols;
extern const int goalX, goalY;

vector<array<int, 2>> getGoalCells();

extern const char startCellColor;
extern const string startCellText;

extern const char goalCellColor;
extern const string goalCellText;

extern const char goalPathColor;
extern const char returnPathColor;

extern bool showGrid;
extern bool showPath;
}  // namespace MazeConstants
}  // namespace Constants

#endif
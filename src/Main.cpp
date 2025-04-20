#include "Main.h"

MouseLocal* mousePtr;
API* apiPtr;
AStar* aStarPtr;
FrontierBased* frontierBasedPtr;

int main() {
    stdio_init_all();
    sleep_ms(3000);  // Wait for the serial console to open
    Motor leftMotor(Constants::RobotConstants::leftMotorPin1, Constants::RobotConstants::leftMotorPin2, Constants::RobotConstants::leftMotorEncoderPin1,
                    Constants::RobotConstants::leftMotorEncoderPin2, Constants::RobotConstants::eventsPerRev, Constants::RobotConstants::maxRPM, true);

    Motor rightMotor(Constants::RobotConstants::rightMotorPin1, Constants::RobotConstants::rightMotorPin2, Constants::RobotConstants::rightMotorEncoderPin1,
                     Constants::RobotConstants::rightMotorEncoderPin2, Constants::RobotConstants::eventsPerRev, Constants::RobotConstants::maxRPM);

    // leftMotor.setPIDVariables(0.015f, 0.0f, 0.001f);
    // leftMotor.setPIDVariables(0.016f, 0.0f, 0.0015f);
    // leftMotor.setPIDVariables(0.016f, 0.00f, 0.00155f);
    // leftMotor.setPIDVariables(0.016f, 0.01f, 0.00155f);
    // leftMotor.setPIDVariables(0.016f / 2.0f, 0.05f, 0.0f);
    // leftMotor.setPIDVariables(0.005f, 0.0045f, 0.0f);
    // leftMotor.setPIDVariables(0.005f, 0.0045f, 0.0f);
    // leftMotor.setPIDVariables(0.0150f / 2.0f, 0.1f / 100.0f, 0.1f / 100.0f);
    // leftMotor.setPIDVariables(0.005f, 0.0020f, 0.000675f);

    // rightMotor.setPIDVariables(0.0055f, 0.0025f, 0.000675f);

    // leftMotor.setPIDVariables(0.01f, 0.0059f, 0.0f);

    // rightMotor.setPIDVariables(0.0085f, 0.00775f, 0.0f);

    // leftMotor.setPIDVariables(0.0105f, 0.00677f, 0.000675f);
    leftMotor.setPIDVariables(0.0095f, 0.00677f, 0.000675f);

    rightMotor.setPIDVariables(0.0085f, 0.0075f, 0.000675f);
    DrivetrainConfiguration config = [] {
        DrivetrainConfiguration cfg;
        cfg.maxRPM = 250.0f;
        cfg.maxTurnRPM = 150.0f;
        cfg.encoderCountsPerCell = 2006 * 1.9f / 1.8f;  // 180mm / (40.0 mm (wheel diameter) * 3.14 (pi)) * 360 (encoder counts per rev). = 634.6609.
        cfg.wallThreshold = 50;                         // mm.
        cfg.distancePID = {0.0250f, 0.0f, 0.0675f};
        cfg.turnPID = {0.5f, 0.0f, 0.0f};
        cfg.yawErrorThrewshold = 3;
        cfg.distanceErrorThreshold = 200.0f;
        return cfg;
    }();

    //         cfg.distancePID = {0.0250f, 0.0f, 0.0675f};

    Drivetrain drivetrain(config, &leftMotor, &rightMotor);

    try {
        sleep_ms(1000);

        mousePtr = new MouseLocal();
        apiPtr = new API(mousePtr, &drivetrain);
        aStarPtr = new AStar();
        frontierBasedPtr = new FrontierBased();
        // sleep_ms(4000);

        // absolute_time_t now2 = get_absolute_time();
        // for (int i = 1; i <= 5; i++) {
        drivetrain.initIMU();
        // drivetrain.initToF();

        // drivetrain.driveForwardDistance(0.4f); // 90mm = to the center of the cell.
        apiPtr->moveForward(4);
        apiPtr->turnRight();
        apiPtr->moveForward(1);
        // }
        sleep_ms(3000);

        // leftMotor.setRPM(-150.0f);
        // rightMotor.setRPM(-150.0f);
        // absolute_time_t now = get_absolute_time();
        // while (1) {
        //     leftMotor.updatePWM();
        //     rightMotor.updatePWM();
        //     LOG_DEBUG("Pos of Left Motor: " + std::to_string(leftMotor.getCurrentPosition()));
        //     LOG_DEBUG("Pos of Right Motor: " + std::to_string(rightMotor.getCurrentPosition()));
        //     // LOG_DEBUG("LRPM: " + std::to_string(leftMotor.getCurrentRPM()) + " RRPM: " + std::to_string(rightMotor.getCurrentRPM()));
        //     // LOG_DEBUG(std::to_string(leftMotor.getCurrentRPM()) + "," + std::to_string(absolute_time_diff_us(now2, get_absolute_time()) / 1e6f) + "," +
        //     //   std::to_string(rightMotor.getCurrentRPM()));
        //     // LOG_DEBUG("RPM Diff (Left to Right): " + std::to_string(leftMotor.getCurrentRPM() - rightMotor.getCurrentRPM()));
        //     sleep_ms(15);
        // LOG_DEBUG("Left=" + std : to_string(drivetrain.checkLeftWall()) + "Front=" + std : to_string(drivetrain.checkFrontWall()) +
                //   "Right=" + std : to_string(drivetrain.checkRightWall()));
        // }

        // for (int i = 1; i < 3; i++) {
        //     sleep_ms(1000);
        //     leftMotor.setRPM(150.0f * i);
        //     rightMotor.setRPM(150.0f * i);
        //     // float rpmL = measureSteadyStateRPM(leftMotor);
        //     // float rpmR = measureSteadyStateRPM(rightMotor);
        //     absolute_time_t now = get_absolute_time();
        //     while (absolute_time_diff_us(now, get_absolute_time()) < 10 * 1e6f) {
        //         leftMotor.updatePWM();
        //         rightMotor.updatePWM();
        //         // LOG_DEBUG("LRPM: " + std::to_string(leftMotor.getCurrentRPM()) + " RRPM: " + std::to_string(rightMotor.getCurrentRPM()));
        //         LOG_DEBUG(std::to_string(leftMotor.getCurrentRPM()) + "," + std::to_string(absolute_time_diff_us(now2, get_absolute_time()) / 1e6f) + "," +
        //                   std::to_string(rightMotor.getCurrentRPM()));

        //         // LOG_DEBUG("RPM Diff (Left to Right): " + std::to_string(leftMotor.getCurrentRPM() - rightMotor.getCurrentRPM()));
        //         sleep_ms(100);
        //     }

        //     sleep_ms(1000);
        //     now = get_absolute_time();
        //     leftMotor.setRPM(100.0f * i);
        //     rightMotor.setRPM(100.0f * i);

        //     while (absolute_time_diff_us(now, get_absolute_time()) < 10 * 1e6f) {
        //         leftMotor.updatePWM();
        //         rightMotor.updatePWM();

        //         LOG_DEBUG(std::to_string(leftMotor.getCurrentRPM()) + "," + std::to_string(absolute_time_diff_us(now2, get_absolute_time()) / 1e6f) + "," +
        //                   std::to_string(rightMotor.getCurrentRPM()));
        //         sleep_ms(100);
        //     }

        //     sleep_ms(1000);
        //     now = get_absolute_time();
        //     leftMotor.setRPM(200.0f * i);
        //     rightMotor.setRPM(200.0f * i);

        //     while (absolute_time_diff_us(now, get_absolute_time()) < 10 * 1e6f) {
        //         leftMotor.updatePWM();
        //         rightMotor.updatePWM();

        //         LOG_DEBUG(std::to_string(leftMotor.getCurrentRPM()) + "," + std::to_string(absolute_time_diff_us(now2, get_absolute_time()) / 1e6f) + "," +
        //                   std::to_string(rightMotor.getCurrentRPM()));

        //         sleep_ms(100);
        //     }
        // }
        // leftMotor.stop();

        // float intendedRPM = 200.0f;
        // leftMotor.setRPM(intendedRPM);
        // rightMotor.setRPM(intendedRPM);
        // while(fabs(leftMotor.getCurrentRPM() - intendedRPM) > 0.1f || fabs(rightMotor.getCurrentRPM() - intendedRPM) > 0.1f) {
        //     sleep_ms(200);
        //     leftMotor.updatePWM();
        //     rightMotor.updatePWM();
        //     LOG_DEBUG("LRPM: " + std::to_string(leftMotor.getCurrentRPM()) + ", RRPM: " + std::to_string(rightMotor.getCurrentRPM()));
        // }
        // drivetrain.driveForwardDistance(0.7f);
        // auto feedforwardConstants = calculateFeedforwardConstants(leftMotor, rightMotor);

        // vector<Cell*> startCells = vector<Cell*>{&mousePtr->getMousePosition()};
        // vector<Cell*> goalCells = mousePtr->getGoalCells();
        // // Explore maze using frontier-based search.

        // setUp(startCells, goalCells);
        // printf("%s", mousePtr->localMazeToString().c_str());
        // frontierBasedPtr->explore(*mousePtr, *apiPtr, false);

    } catch (const std::runtime_error& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    return 0;
}

// Wait for a motor to reach a steady state and then return its stable RPM.
// Note: This function can be used for either motor.
float measureSteadyStateRPM(Motor& motor) {
    sleep_ms(200);                 // initial delay to let the motor respond
    const float threshold = 0.1f;  // RPM change tolerance for steady state
    const int stableCountTarget = 5;
    int stableCount = 0;
    motor.updateEncoder();
    float lastRPM = motor.getCurrentRPM();

    while (stableCount < stableCountTarget) {
        sleep_ms(100);  // poll every 100ms
        motor.updateEncoder();
        float currentRPM = motor.getCurrentRPM();
        if (fabs(currentRPM - lastRPM) < threshold) {
            stableCount++;
        } else {
            stableCount = 0;  // restart check if RPM fluctuates
        }
        lastRPM = currentRPM;
    }
    return lastRPM;
}

// This function calculates feedforward constants for both left and right motors.
// It returns a pair: first = left motor constants, second = right motor constants.
std::pair<FeedforwardConstants, FeedforwardConstants> calculateFeedforwardConstants(Motor& leftMotor, Motor& rightMotor) {
    std::vector<float> voltages;   // same applied voltage for both motors
    std::vector<float> leftRPMs;   // measured steady-state RPMs for left motor
    std::vector<float> rightRPMs;  // measured steady-state RPMs for right motor

    // Loop through a series of voltage steps (e.g., 0.5V to 5V)
    for (int i = 1; i <= 25; i++) {
        float appliedVoltage = 0.2f * i;

        // Apply the voltage to both motors simultaneously.
        leftMotor.setVoltage(appliedVoltage, true);
        rightMotor.setVoltage(appliedVoltage, true);

        // Give time for the motors to respond.
        sleep_ms(500);

        // Measure each motor's steady-state RPM.
        float measuredLeftRPM = measureSteadyStateRPM(leftMotor);
        float measuredRightRPM = measureSteadyStateRPM(rightMotor);

        // Log the results
        LOG_DEBUG("Voltage: " + std::to_string(appliedVoltage) + " V, Left RPM: " + std::to_string(measuredLeftRPM) +
                  ", Right RPM: " + std::to_string(measuredRightRPM));

        // Record the data points.
        voltages.push_back(appliedVoltage);
        leftRPMs.push_back(measuredLeftRPM);
        rightRPMs.push_back(measuredRightRPM);

        // Stop motors between tests to allow them to settle.
        leftMotor.setVoltage(0, true);
        rightMotor.setVoltage(0, true);
        sleep_ms(500);
    }

    // Now perform linear regression for each motor to fit the model: Voltage = kS + kV * RPM

    auto linearRegression = [&](const std::vector<float>& rpms) -> FeedforwardConstants {
        float sumRPM = 0.0f, sumVoltage = 0.0f;
        int n = voltages.size();
        for (int i = 0; i < n; i++) {
            sumRPM += rpms[i];
            sumVoltage += voltages[i];
        }
        float meanRPM = sumRPM / n;
        float meanVoltage = sumVoltage / n;

        float numerator = 0.0f;
        float denominator = 0.0f;
        for (int i = 0; i < n; i++) {
            numerator += (rpms[i] - meanRPM) * (voltages[i] - meanVoltage);
            denominator += (rpms[i] - meanRPM) * (rpms[i] - meanRPM);
        }
        FeedforwardConstants constants;
        constants.kV = numerator / denominator;               // Slope
        constants.kS = meanVoltage - constants.kV * meanRPM;  // Intercept

        return constants;
    };

    FeedforwardConstants leftConstants = linearRegression(leftRPMs);
    FeedforwardConstants rightConstants = linearRegression(rightRPMs);

    // Log the calculated constants for each motor.
    while (1) {
        LOG_DEBUG("Left Motor Feedforward: kS = " + std::to_string(leftConstants.kS) + ", kV = " + std::to_string(leftConstants.kV));
        LOG_DEBUG("Right Motor Feedforward: kS = " + std::to_string(rightConstants.kS) + ", kV = " + std::to_string(rightConstants.kV));
        sleep_ms(100);
    }

    // Return a pair with left constants first, right constants second.
    return std::make_pair(leftConstants, rightConstants);
}

void setUp(const vector<Cell*>& goalCells) {
    setUp(vector<Cell*>{&mousePtr->getMousePosition()}, goalCells);
}

void setUp(const vector<Cell*>& startCells, const vector<Cell*>& goalCells) {
    apiPtr->clearAllColor();
    apiPtr->clearAllText();

    // Adds boundary mazes.
    for (int i = 0; i < Constants::MazeConstants::numCols; i++) {
        apiPtr->setWall(i, 0, "s");                                      // Bottom edge
        apiPtr->setWall(i, Constants::MazeConstants::numRows - 1, "n");  // Top edge
    }
    for (int j = 0; j < Constants::MazeConstants::numRows; j++) {
        apiPtr->setWall(0, j, "w");                                      // Left edge
        apiPtr->setWall(Constants::MazeConstants::numCols - 1, j, "e");  // Right edge
    }

    // Adds grid labels.
    if (Constants::MazeConstants::showGrid) {
        for (int i = 0; i < Constants::MazeConstants::numCols; i++) {
            for (int j = 0; j < Constants::MazeConstants::numRows; j++) {
                apiPtr->setText(i, j, std::to_string(i) + "," + std::to_string(j));
            }
        }
    }

    LOG_WARN("Running " + Constants::MouseConstants::mouseName + "...");

    // Adds color/text to start and goal cells.
    for (const auto& startCell : startCells) {
        apiPtr->setColor(startCell->getX(), startCell->getY(), Constants::MazeConstants::startCellColor);
        apiPtr->setText(startCell->getX(), startCell->getY(), Constants::MazeConstants::startCellText);
    }
    for (const auto& goalCell : goalCells) {
        apiPtr->setColor(goalCell->getX(), goalCell->getY(), Constants::MazeConstants::goalCellColor);
        apiPtr->setText(goalCell->getX(), goalCell->getY(), Constants::MazeConstants::goalCellText);
    }
}

void sleepFor(int milliseconds) {
    // Sleeps for a specified number of milliseconds.
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void setAllExplored(MouseLocal* mouse) {
    // Sets all cells to explored.
    for (int i = 0; i < Constants::MazeConstants::numCols; i++) {
        for (int j = 0; j < Constants::MazeConstants::numRows; j++) {
            mouse->getCell(i, j).setIsExplored(true);
        }
    }
}

vector<Cell*> getBestAlgorithmPath(AStar* aStar, vector<Cell*>& goalCells, bool diagonalsAllowed, bool avoidGoalCells) {
    vector<Cell*> bestPath;
    double bestPathCost = std::numeric_limits<double>::max();

    // Returns the least cost path to the goal cells.
    for (const auto& goal : goalCells) {
        vector<Cell*> path = aStar->findAStarPath(*mousePtr, *goal, diagonalsAllowed, avoidGoalCells);
        if (!path.empty()) {
            double cost = goal->getTotalCost();
            if (cost < bestPathCost) {
                bestPath.clear();
                for (const auto& cellPtr : path) {
                    bestPath.push_back(cellPtr);
                }
                bestPathCost = cost;
            }
        }
    }
    return bestPath;
}

void turnMouseToNextCell(const Cell& currentCell, const Cell& nextCell) {
    array<int, 2> halfSteps = mousePtr->obtainHalfStepCount(mousePtr->getDirBetweenCells(currentCell, nextCell));

    // Turns mouse to face next cell.
    for (int i = 0; i < halfSteps[0]; i++) {
        if (halfSteps[1] == -1) {
            apiPtr->turnLeft45();
        } else {
            apiPtr->turnRight45();
        }
    }
}

enum class MovementBlock { RFLF, LFRF, RFRF, LFLF, FLF, FRF, F, RF, LF, L, R, DEFAULT };

MovementBlock parseMovementBlock(const string& block) {
    // Maps movement block strings to MovementBlock values.
    static const unordered_map<string, MovementBlock> movementMap = {
        {"RFLF", MovementBlock::RFLF}, {"LFRF", MovementBlock::LFRF}, {"RFRF", MovementBlock::RFRF}, {"LFLF", MovementBlock::LFLF},
        {"FLF", MovementBlock::FLF},   {"FRF", MovementBlock::FRF},   {"F", MovementBlock::F},       {"RF", MovementBlock::RF},
        {"LF", MovementBlock::LF},     {"L", MovementBlock::L},       {"R", MovementBlock::R}};
    auto it = movementMap.find(block);
    if (it != movementMap.end())
        return it->second;
    return MovementBlock::DEFAULT;
}

string unparseMovementBlock(const MovementBlock block) {
    // Maps MovementBlock values to movement block strings.
    static const unordered_map<MovementBlock, string> movementMap = {
        {MovementBlock::RFLF, "RFLF"}, {MovementBlock::LFRF, "LFRF"}, {MovementBlock::RFRF, "RFRF"}, {MovementBlock::LFLF, "LFLF"},
        {MovementBlock::FLF, "FLF"},   {MovementBlock::FRF, "FRF"},   {MovementBlock::F, "F"},       {MovementBlock::RF, "RF"},
        {MovementBlock::LF, "LF"},     {MovementBlock::L, "L"},       {MovementBlock::R, "R"}};
    auto it = movementMap.find(block);
    if (it != movementMap.end())
        return it->second;
    return "";
}

vector<string> splitPath(const string& path) {
    // Splits the path strings into individual movements ("vector-ize"/"split").
    vector<string> movements;
    stringstream ss(path);
    string token;
    while (getline(ss, token, '#')) {
        if (!token.empty())
            movements.push_back(token);
    }
    return movements;
}

void executeSequence(const string& seq, ostringstream& diagPath) {
    diagPath << seq;
    stringstream ss(seq);
    string token;
    // Executes a command sequence.
    while (getline(ss, token, '#')) {
        if (token.empty()) {
            continue;
        }
        if (token == "R") {
            apiPtr->turnRight();
        } else if (token == "L") {
            apiPtr->turnLeft();
        } else if (token == "F") {
            apiPtr->moveForward();
        } else if (token == "R45") {
            apiPtr->turnRight45();
        } else if (token == "L45") {
            apiPtr->turnLeft45();
        } else if (token == "FH") {
            apiPtr->moveForwardHalf();
        }
    }
}

void performSequence(const string& seq, ostringstream& diagPath, bool localMove = false) {
    // Executes and updates mouse position for combo moves.
    if (localMove) {
        LOG_DEBUG("Performing Sequence: " + seq);
    }
    executeSequence(seq, diagPath);
    if (localMove) {
        mousePtr->moveForwardLocal();
    }
}

void executeIndividualMovement(const vector<string>& movementsSequence, int& i, ostringstream& diagPath, MovementBlock& prevBlockType, Cell& currCell,
                               bool ignoreRFLF = false) {
    MovementBlock threeMovementBlock = MovementBlock::DEFAULT;
    MovementBlock twoMovementBlock = MovementBlock::DEFAULT;
    MovementBlock nextMovementBlock = MovementBlock::DEFAULT;

    if (i + 2 < static_cast<int>(movementsSequence.size())) {
        threeMovementBlock = parseMovementBlock(movementsSequence[i] + movementsSequence[i + 1] + movementsSequence[i + 2]);
    }
    if (i + 1 < static_cast<int>(movementsSequence.size())) {
        twoMovementBlock = parseMovementBlock(movementsSequence[i] + movementsSequence[i + 1]);
    }
    nextMovementBlock = parseMovementBlock(movementsSequence[i]);
    if (!ignoreRFLF) {
        if (prevBlockType == MovementBlock::RFLF || prevBlockType == MovementBlock::LFLF) {
            if (twoMovementBlock == MovementBlock::RF) {
                performSequence("FH#R45#FH#", diagPath, true);
                prevBlockType = twoMovementBlock;
                i++;
            } else if (twoMovementBlock == MovementBlock::LF) {
                performSequence("L#FH#L45#FH#", diagPath, true);
                prevBlockType = twoMovementBlock;
                i++;
            } else if (threeMovementBlock == MovementBlock::FLF) {
                performSequence("L45#F#L45#FH#L45#FH#", diagPath, true);
                prevBlockType = MovementBlock::F;
                i += 2;
            } else {
                performSequence("L45#FH#", diagPath);
                executeIndividualMovement(movementsSequence, i, diagPath, prevBlockType, currCell, true);
            }
            return;
        } else if (prevBlockType == MovementBlock::LFRF || prevBlockType == MovementBlock::RFRF) {
            if (twoMovementBlock == MovementBlock::LF) {
                performSequence("FH#L45#FH#", diagPath, true);
                prevBlockType = twoMovementBlock;
                i++;
            } else if (twoMovementBlock == MovementBlock::RF) {
                performSequence("R#FH#R45#FH#", diagPath, true);
                prevBlockType = twoMovementBlock;
                i++;
            } else if (threeMovementBlock == MovementBlock::FRF) {
                performSequence("R45#F#R45#FH#R45#FH#", diagPath, true);
                prevBlockType = MovementBlock::F;
                i += 2;
            } else {
                performSequence("R45#FH#", diagPath);
                executeIndividualMovement(movementsSequence, i, diagPath, prevBlockType, currCell, true);
            }
            return;
        }
    }
    if (nextMovementBlock == MovementBlock::R) {
        performSequence("R#", diagPath);
    } else if (nextMovementBlock == MovementBlock::L) {
        performSequence("L#", diagPath);
    } else if (nextMovementBlock == MovementBlock::F) {
        performSequence("F#", diagPath);
    }
    prevBlockType = nextMovementBlock;
    currCell = mousePtr->getMousePosition();
    // LOG_INFO("Step " + to_string(i) + ": " + currCell.toString());
}

void executeIndividualMovement(string movement, Cell& currCell) {
    vector<string> movementsSequence = {movement};
    ostringstream path = ostringstream();
    MovementBlock prevBlockType = MovementBlock::DEFAULT;
    int i = 0;
    executeIndividualMovement(movementsSequence, i, path, prevBlockType, currCell);
}

void correctForEdge(ostringstream& diagPath, MovementBlock prevBlockType) {
    LOG_DEBUG("Correcting for edge.");
    LOG_DEBUG("Previous Block: " + unparseMovementBlock(prevBlockType));
    if (prevBlockType == MovementBlock::RFLF || prevBlockType == MovementBlock::LFLF) {
        performSequence("L45#FH#", diagPath);
    } else if (prevBlockType == MovementBlock::LFRF || prevBlockType == MovementBlock::RFRF) {
        performSequence("R45#FH#", diagPath);
    }
}

string diagonalizeAndRun(Cell& currCell, const string& path) {
    ostringstream diagPath;
    vector<string> movementsSequence = splitPath(path);
    MovementBlock blockType;
    MovementBlock prevBlockType = MovementBlock::DEFAULT;
    int i = 0;

    while (i < static_cast<int>(movementsSequence.size())) {
        currCell = mousePtr->getMousePosition();
        LOG_DEBUG("Step " + to_string(i) + ": " + currCell.toString());

        // Check for a forward combo: "F" followed by a 4-move combo block.
        if (movementsSequence[i] == "F" && (i + 4 < static_cast<int>(movementsSequence.size()))) {
            i++;
            MovementBlock nextMovementBlock =
                parseMovementBlock(movementsSequence[i] + movementsSequence[i + 1] + movementsSequence[i + 2] + movementsSequence[i + 3]);
            if (nextMovementBlock == MovementBlock::RFRF || nextMovementBlock == MovementBlock::LFLF || nextMovementBlock == MovementBlock::RFLF ||
                nextMovementBlock == MovementBlock::LFRF) {
                if (prevBlockType != MovementBlock::RFRF && prevBlockType != MovementBlock::LFLF && prevBlockType != MovementBlock::RFLF &&
                    prevBlockType != MovementBlock::LFRF) {
                    performSequence("FH#", diagPath, true);
                } else {
                    if (prevBlockType == MovementBlock::RFLF || prevBlockType == MovementBlock::LFLF) {
                        performSequence("L45#F#", diagPath);
                    } else if (prevBlockType == MovementBlock::LFRF || prevBlockType == MovementBlock::RFRF) {
                        performSequence("R45#F#", diagPath);
                    }
                }
                prevBlockType = MovementBlock::F;
            } else {
                i--;
            }
        }

        // Attempt to form a 4-move combo block if possible.
        if (i + 3 < static_cast<int>(movementsSequence.size())) {
            blockType = parseMovementBlock(movementsSequence[i] + movementsSequence[i + 1] + movementsSequence[i + 2] + movementsSequence[i + 3]);
            LOG_DEBUG("Block Type: " + movementsSequence[i] + movementsSequence[i + 1] + movementsSequence[i + 2] + movementsSequence[i + 3]);
            switch (blockType) {
                case MovementBlock::RFLF:
                    if (prevBlockType == blockType || prevBlockType == MovementBlock::LFLF)
                        performSequence("F#", diagPath);
                    else if (prevBlockType == MovementBlock::LFRF || prevBlockType == MovementBlock::RFRF)
                        performSequence("R#F#", diagPath);
                    else if (prevBlockType == MovementBlock::F)
                        performSequence("R45#F#", diagPath);
                    else
                        performSequence("R#FH#L45#FH#", diagPath, true);
                    i += 3;
                    break;

                case MovementBlock::LFRF:
                    if (prevBlockType == blockType || prevBlockType == MovementBlock::RFRF)
                        performSequence("F#", diagPath);
                    else if (prevBlockType == MovementBlock::RFLF || prevBlockType == MovementBlock::LFLF)
                        performSequence("L#F#", diagPath);
                    else if (prevBlockType == MovementBlock::F)
                        performSequence("L45#F#", diagPath);
                    else
                        performSequence("L#FH#R45#FH#", diagPath, true);
                    i += 3;
                    break;

                case MovementBlock::RFRF:
                    if (prevBlockType == blockType)
                        performSequence("R#FH#R#FH#", diagPath, true);
                    else if (prevBlockType == MovementBlock::RFLF || prevBlockType == MovementBlock::LFLF)
                        performSequence("FH#R#FH#", diagPath, true);
                    else if (prevBlockType == MovementBlock::F)
                        performSequence("R45#FH#R#FH#", diagPath, true);
                    else
                        performSequence("R#FH#R45#FH#", diagPath, true);
                    i += 3;
                    break;

                case MovementBlock::LFLF:
                    if (prevBlockType == blockType)
                        performSequence("L#FH#L#FH#", diagPath, true);
                    else if (prevBlockType == MovementBlock::LFRF || prevBlockType == MovementBlock::RFRF)
                        performSequence("FH#L#FH#", diagPath, true);
                    else if (prevBlockType == MovementBlock::F)
                        performSequence("L45#FH#L#FH#", diagPath, true);
                    else
                        performSequence("L#FH#L45#FH#", diagPath, true);
                    i += 3;
                    break;

                default:
                    executeIndividualMovement(movementsSequence, i, diagPath, prevBlockType, currCell);
                    break;
            }
            prevBlockType = blockType;
        } else {
            executeIndividualMovement(movementsSequence, i, diagPath, prevBlockType, currCell);
        }

        i++;
    }
    correctForEdge(diagPath, prevBlockType);
    return diagPath.str();
}

/**
 * @brief Traverses a single goal cell iteratively.
 *
 * @param mouse Pointer to the MouseLocal instance.
 * @param goalCell The target Cell to traverse to.
 * @param diagonalsAllowed Whether diagonal movements are permitted.
 * @param allExplored Whether all cells should be marked as explored.
 * @param avoidGoalCells Whether to avoid goal cells during traversal.
 * @return true If traversal was successful.
 * @return false If traversal failed.
 */
bool traversePathIteratively(MouseLocal* mouse, Cell& goalCell, bool diagonalsAllowed, bool allExplored, bool avoidGoalCells) {
    vector<Cell*> goalCells;
    goalCells.push_back(&goalCell);
    return traversePathIteratively(mouse, goalCells, diagonalsAllowed, allExplored, avoidGoalCells);
}

/**
 * @brief Traverses multiple goal cells iteratively.
 *
 * @param mouse Pointer to the MouseLocal instance.
 * @param goalCells A vector of target Cells to traverse to.
 * @param diagonalsAllowed Whether diagonal movements are permitted.
 * @param allExplored Whether all cells should be marked as explored.
 * @param avoidGoalCells Whether to avoid goal cells during traversal.
 * @return true If traversal was successful for all goals.
 * @return false If traversal failed for any goal.
 */
bool traversePathIteratively(MouseLocal* mouse, vector<Cell*>& goalCells, bool diagonalsAllowed, bool allExplored, bool avoidGoalCells) {
    Cell currCell(0, 0);          // Initialize with appropriate constructor arguments
    Movement* prevMov = nullptr;  // Assuming Movement is a class with pointer semantics

    if (allExplored) {
        setAllExplored(mouse);
    }

    while (true) {
        currCell = (mouse->getMousePosition());
        currCell.setIsExplored(true);

        if (mouse->isGoalCell(currCell, goalCells)) {
            // If the previous movement was diagonal, handle that.
            if (prevMov != nullptr && prevMov->getIsDiagonal()) {
                turnMouseToNextCell(*(prevMov->getFirstMove()), currCell);
                apiPtr->moveForwardHalf();
            }
            break;
        }
        prevMov = nullptr;

        // Detect walls
        mouse->detectAndSetWalls(*apiPtr);  // Pass API if needed

        // Get path from A* or your best algorithm
        vector<Cell*> cellPath = getBestAlgorithmPath(aStarPtr, goalCells, diagonalsAllowed, avoidGoalCells);

        // If you want to color the path
        if (Constants::MazeConstants::showPath && allExplored && !MouseLocal::isSame(*goalCells[0], (mouse->getCell(0, 0)))) {
            for (const auto& c : cellPath) {
                apiPtr->setColor(c->getX(), c->getY(), Constants::MazeConstants::goalPathColor);
            }
        } else if (Constants::MazeConstants::showPath && allExplored && goalCells.size() == 1) {
            for (const auto& c : cellPath) {
                apiPtr->setColor(c->getX(), c->getY(), Constants::MazeConstants::returnPathColor);
            }
        }

        // Log the algorithm path
        string algPathStr = "";
        for (const auto& c : cellPath) {
            algPathStr += "(" + std::to_string(c->getX()) + ", " + std::to_string(c->getY()) + ") -> ";
        }

        // Convert path to string
        vector<Cell*> cellPathPtrs;
        for (auto& cell : cellPath) {
            cellPathPtrs.push_back(cell);
        }
        string path = AStar::pathToString(*mouse, cellPathPtrs);

        if (cellPath.size() > 9 && allExplored) {
            LOG_INFO("Path: " + algPathStr);
            LOG_INFO("LFR Path: " + path);
        }

        if (allExplored && diagonalsAllowed) {
            path = diagonalizeAndRun(currCell, path);
            LOG_INFO("Diag Path: " + path);
        } else {
            // Execute the movement commands step-by-step
            stringstream ss(path);
            string move;
            while (std::getline(ss, move, '#')) {
                executeIndividualMovement(move, currCell);

                if (!currCell.getIsExplored()) {
                    LOG_DEBUG("[RE-CALC] Cell is unexplored, calculating new path.");
                    break;
                }
                LOG_DEBUG("[RE-USE] Reusing ...");
            }
        }
        break;  // The original Java code has "break; // FIXME" so we do the same
    }
    return true;
}
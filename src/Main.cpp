#include "Main.h"

MouseLocal* mousePtr;
API* apiPtr;
AStar* aStarPtr;
FrontierBased* frontierBasedPtr;

int main() {
    stdio_init_all();
    sleep_ms(5000);  // Wait for the serial console to open
    Motor leftMotor(Constants::RobotConstants::leftMotorPin1, Constants::RobotConstants::leftMotorPin2, Constants::RobotConstants::leftMotorEncoderPin1,
                    Constants::RobotConstants::leftMotorEncoderPin2, Constants::RobotConstants::eventsPerRev, Constants::RobotConstants::maxRPM, true);

    Motor rightMotor(Constants::RobotConstants::rightMotorPin1, Constants::RobotConstants::rightMotorPin2, Constants::RobotConstants::rightMotorEncoderPin1,
                     Constants::RobotConstants::rightMotorEncoderPin2, Constants::RobotConstants::eventsPerRev, Constants::RobotConstants::maxRPM);

    // leftMotor.setPIDVariables(0.0175f, 0, 0.075f);
    // rightMotor.setPIDVariables(0.06200f, 0, 0.075f);
    leftMotor.setPIDVariables(0, 0, 0);
    rightMotor.setPIDVariables(0, 0, 0);

    DrivetrainConfiguration config = [] {
        DrivetrainConfiguration cfg;
        cfg.maxRPM = 300.0f;
        cfg.maxTurnRPM = 200.0f;
        cfg.encoderCountsPerCell = 635 * 190 / 180;  // 180 / (32.5 mm (wheel diameter) * 3.14 (pi)) * 360 (encoder counts per rev). = 634.6609.
        cfg.wallThreshold = 50;                      // mm.
        cfg.distancePID = {0.0f, 0.0f, 0.0f};
        cfg.turnPID = {0.0f, 0.0f, 0.0f};
        cfg.yawErrorThrewshold = 3;
        cfg.distanceErrorThreshold = 10.0f;
        return cfg;
    }();

    Drivetrain drivetrain(config, &leftMotor, &rightMotor);
    MouseLocal mouseLocal;

    API api(&mouseLocal, &drivetrain);

    try {
        sleep_ms(1000);

        // for (int i = 0; i < 20; i++) {
        // 	float kPL = 0.0175f;
        // 	float kIL = 0.0f;
        // 	float kDL = 0.075f;
        // 	;
        // 	float kPR = 0.06200f;
        // 	float kIR = 0.0f;
        // 	float kDR = 0.075f;
        // 	;

        // 	leftMotor.setPIDVariables(kPL, kIL, kDL);
        // 	rightMotor.setPIDVariables(kPR, kIR, kDR);
        // 	printf("PID ValuesL: %f %f %f\n", kPL, kIL, kDL);
        // 	printf("PID ValuesR: %f %f %f\n", kPR, kIR, kDR);
        // 	drivetrain.driveForward();
        // 	absolute_time_t startTime = get_absolute_time();
        // 	while (absolute_time_diff_us(startTime, get_absolute_time()) < 10000000) {
        // 		leftMotor.updateEncoder();
        // 		rightMotor.updateEncoder();
        // 		printf("Left Motor RPM %f\n", leftMotor.getCurrentRPM());
        // 		printf("Right Motor RPM %f\n", rightMotor.getCurrentRPM());
        // 		sleep_ms(500);
        // 	}
        // 	drivetrain.stop();
        // 	sleep_ms(1000);
        // }
        // leftMotor.setVoltage(3.0f, true);
        drivetrain.initIMU();
        // drivetrain.driveForwardDistance(10.0f / 18.0f);

        // rightMotor.setVoltage(5.0f, true);
        for (int i = 1; i <= 10; i++) {
            absolute_time_t now = get_absolute_time();
            while (absolute_time_diff_us(now, get_absolute_time()) < 12.5 * 1e6) {
                // leftMotor.updateEncoder();
                // rightMotor.updateEncoder();
                float elapsedTime = absolute_time_diff_us(now, get_absolute_time()) / 1e6f;
                // rightMotor.updateEncoder();
                // printf("Left Motor RPM %f\n", leftMotor.getCurrentRPM());
                // printf("%f, %f\n", elapsedTime, rightMotor.getCurrentRPM());
                // printf("Right Motor RPM %f\n", rightMotor.getCurrentRPM());
                // printf("LRPM=%f, RRPM=%f, LP=%i, RP=%i\n", leftMotor.getCurrentRPM(), rightMotor.getCurrentRPM(), leftMotor.getCurrentPosition(),
                // rightMotor.getCurrentPosition());
                printf("Front Wall=%b, Left Wall=%b, Right Wall=%b\n", drivetrain.checkFrontWallDistance(), drivetrain.checkLeftWallDistance(), drivetrain.checkRightWallDistance());
                sleep_ms(100);
            }
            sleep_ms(2000);
        }
        // }
        // api.moveForward(1);

        // for (int i = 600; i < 1000; i++) {
        // 	rightMotor.updateEncoder();
        // 	rightMotor.getFeedforwardValue(i, "rightMotor");
        // }
        // drivetrain.driveForward();

        //     leftMotor.updateEncoder();
        //     rightMotor.updateEncoder();
        //     printf("Left Motor RPM %f\n", leftMotor.getCurrentRPM());
        // 	printf("Right Motor RPM %f\n", rightMotor.getCurrentRPM());
        // for (int i = 1; i < 2; i++) {
        // 	api.moveForward(i);
        // 	sleep_ms(2000);
        // }

        /*
        // while (1) {
        // 	absolute_time_t now = get_absolute_time();
        // 	int deltaTime = absolute_time_diff_us(lastUpdateTime, now);
        // 	// printf("Delta Time: %f\n", deltaTime);
        // 	lastUpdateTime = now;
        // 	int currentCount = leftMotor.getEncoder()->getCount();

        // 	// printf("Current Count: %d\n", currentCount);
        // 	int deltaCount = currentCount - oldEncoderCount;
        // 	// printf("Delta Count: %d\n", deltaCount);
        // 	oldEncoderCount = currentCount;

        // 	// Calculate RPM with noise filtering
        // 	// float averageDelta = getAverageDeltaTime(deltaTime);
        // 	double averageDelta = deltaTime / 1000000.0;
        // 	double currentRPM = (deltaCount / 360.0) * (60.0 / averageDelta);

        // 	printf("LRPM=%.6f\n", currentRPM);
        // 	sleep_ms(500);
        // }

        /*
        leftMotor.setThrottle(1);
        rightMotor.setThrottle(1);

        while (1){
                leftMotor.updatePWM();
                int32_t leftRPM = leftMotor.getCurrentRPM();
                printf("LRPM=%.6f\n", leftRPM);
                sleep_ms(1000);
        }
                */
        // int32_t rightRPM = rightMotor.getCurrentRPM();

        // int32_t leftPos = leftMotor.getCurrentPosition();
        // int32_t rightPos = rightMotor.getCurrentPosition();
        // printf("LRPM=%d RRPM=%d LP=%d RP=%d\n", leftRPM, rightRPM, leftPos, rightPos);

    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}
// int main() {
// 	stdio_init_all();

// 	Motor leftMotor(Constants::RobotConstants::leftMotorPin1, Constants::RobotConstants::leftMotorPin2,
// 					Constants::RobotConstants::leftMotorEncoderPin1, Constants::RobotConstants::leftMotorEncoderPin2,
// 					Constants::RobotConstants::eventsPerRev, Constants::RobotConstants::maxRPM, true);

// 	Motor rightMotor(Constants::RobotConstants::rightMotorPin1, Constants::RobotConstants::rightMotorPin2,
// 					 Constants::RobotConstants::rightMotorEncoderPin1, Constants::RobotConstants::rightMotorEncoderPin2,
// 					 Constants::RobotConstants::eventsPerRev, Constants::RobotConstants::maxRPM);

// 	leftMotor.setPIDVariables(Constants::RobotConstants::kP, Constants::RobotConstants::kI, Constants::RobotConstants::kD);
// 	rightMotor.setPIDVariables(Constants::RobotConstants::kP, Constants::RobotConstants::kI, Constants::RobotConstants::kD);

// 	float testThrottles[] = {0.5f, 0.25f, -0.1f};

// 	sleep_ms(5000);
// 	printf("Starting test sequence!\n");

// 	while (true) {
// 		for (size_t i = 0; i < sizeof(testThrottles) / sizeof(testThrottles[0]); i++) {
// 			float throttle = testThrottles[i];
// 			leftMotor.setThrottle(throttle);
// 			rightMotor.setThrottle(throttle);

// 			leftMotor.updatePWM();
// 			rightMotor.updatePWM();

// 			absolute_time_t startTime = get_absolute_time();
// 			while (absolute_time_diff_us(startTime, get_absolute_time()) < 2000000) {
//                 leftMotor.updateEncoder();
//                 rightMotor.updateEncoder();

// 				printf("-------\n");
// 				printf("Left RPM: %f\nRight RPM: %f\nat Throttle: %f\n", leftMotor.getCurrentRPM(), rightMotor.getCurrentRPM(), throttle);
// 				sleep_ms(200);
// 			}
// 			sleep_ms(2000);
// 		}
// 	}
// 	return 0;
// }

// int main() {
// 	// Initialize stdio for serial printing
// 	stdio_init_all();
// 	printf("Motor + Encoder Test: twist wheels by hand to observe count/RPM\n");

// 	// Wait a moment for the serial console to open
// 	sleep_ms(2000);

// 	// Here, use the same pins/arguments you normally do for your Motor constructor.
// 	// e.g., if your real motor is Motor(19, 18, 21, 20, 400.0f, 400.0f)
// 	// (replace them below with your actual motorPin1, motorPin2, etc.)
// 	Motor testMotor(19,		 // motorPin1
// 					18,		 // motorPin2
// 					21,		 // encoderPin1
// 					20,		 // encoderPin2
// 					400.0f,	 // eventsPerRev
// 					400.0f	 // maxRPM
// 	);

// 	// Normally you might call:
// 	// testMotor.setPIDVariables(1.0f, 0.0f, 0.0f);
// 	// testMotor.setThrottle(0.2f);
// 	// but we comment these out for the test, so the motor does NOT drive:
// 	// testMotor.setPIDVariables(...);
// 	// testMotor.setThrottle(...);
// 	// testMotor.start();

// 	while (true) {
// 		// We'll just read the encoder data and print it every 500ms
// 		// This calls Motor::updateEncoder(), which calls your embedded Encoder code
// 		// and updates currentPosition / currentRPM
// 		testMotor.updateEncoder();

// 		// Retrieve the new position and RPM
// 		int currentPos = testMotor.getCurrentPosition();
// 		float currentRpm = testMotor.getCurrentRPM();

// 		// Print them out so we can see how it changes when we twist the wheels
// 		printf("Encoder Count = %d, RPM = %.2f\n", currentPos, currentRpm);

// 		// Sleep for half a second, then loop again
// 		sleep_ms(500);
// 	}

// 	return 0;  // Not really reached
// }

// int main() {
// 	stdio_init_all();
// 	sleep_ms(2000);	 // Wait for the serial console to open
// 	Motor leftMotor(Constants::RobotConstants::leftMotorPin1, Constants::RobotConstants::leftMotorPin2,
// 					Constants::RobotConstants::leftMotorEncoderPin1, Constants::RobotConstants::leftMotorEncoderPin2,
// 					Constants::RobotConstants::eventsPerRev, Constants::RobotConstants::maxRPM, true);

// 	Motor rightMotor(Constants::RobotConstants::rightMotorPin1, Constants::RobotConstants::rightMotorPin2,
// 					 Constants::RobotConstants::rightMotorEncoderPin1, Constants::RobotConstants::rightMotorEncoderPin2,
// 					 Constants::RobotConstants::eventsPerRev, Constants::RobotConstants::maxRPM);

// 	leftMotor.setPIDVariables(Constants::RobotConstants::kP, Constants::RobotConstants::kI, Constants::RobotConstants::kD);
// 	rightMotor.setPIDVariables(Constants::RobotConstants::kP, Constants::RobotConstants::kI, Constants::RobotConstants::kD);

// 	leftMotor.setThrottle(0.3);
// 	rightMotor.setThrottle(0.3);

// 	leftMotor.updatePWM();
// 	rightMotor.updatePWM();

// 	while (true) {
// 		leftMotor.updateEncoder();
// 		rightMotor.updateEncoder();

// 		int32_t leftRPM = leftMotor.getCurrentRPM();
// 		int32_t rightRPM = rightMotor.getCurrentRPM();
// 		int32_t leftPos = leftMotor.getCurrentPosition();
// 		int32_t rightPos = rightMotor.getCurrentPosition();
// 		printf("LRPM=%d RRPM=%d LP=%d RP=%d\n", leftRPM, rightRPM, leftPos, rightPos);
// 		// bool stateA = gpio_get(pinA);
// 		// bool stateB = gpio_get(pinB);
// 		// bool stateA2 = gpio_get(pinA2);
// 		// bool stateB2 = gpio_get(pinB2);
// 		// printf("A=%d B=%d C=%d D=%d\n", stateA, stateB, stateA2, stateB2);
// 		sleep_ms(100);
// 	}
// }

// mousePtr = new MouseLocal();
// apiPtr = new API(mousePtr);
// aStarPtr = new AStar();
// frontierBasedPtr = new FrontierBased();

// vector<Cell*> startCells = vector<Cell*>{&mousePtr->getMousePosition()};
// vector<Cell*> goalCells = mousePtr->getGoalCells();

// // Explore maze using frontier-based search.
// setUp(startCells, goalCells);
// frontierBasedPtr->explore(*mousePtr, *apiPtr, false);
// sleepFor(1000);

// // Travel to start cell using A*.
// setUp(startCells);
// traversePathIteratively(mousePtr, startCells, false, true, false);
// sleepFor(1000);

// // Travel to goal cells using A*.
// setUp(goalCells);
// traversePathIteratively(mousePtr, goalCells, true, true, false);
// sleepFor(1000);

// delete frontierBasedPtr;
// delete aStarPtr;
// delete apiPtr;
// delete mousePtr;
// return 0;
// }

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
    LOG_DEBUG("Path Sequence: " + seq);
    // Executes a command sequence.
    while (getline(ss, token, '#')) {
        if (token.empty())
            continue;
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
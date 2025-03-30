#ifndef MAIN_H
#define MAIN_H

#include <algorithm>
#include <chrono>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <stdio.h>
#include "API.h"
#include "Maze_Traversal/AStar.h"
#include "Maze_Traversal/FrontierBased.h"
#include "Maze_Traversal/Maze/Cell.h"
#include "Maze_Traversal/Maze/MouseLocal.h"
#include "Maze_Traversal/Maze/Movement.h"
#include "Subsystems/Drive/Drivetrain.h"
#include "pico/stdlib.h"

using std::to_string;
using std::unordered_map;
using std::vector;

class Cell;
class MouseLocal;
class API;
class AStar;
class FrontierBased;

extern MouseLocal* mousePtr;
extern API* apiPtr;
extern AStar* aStarPtr;
extern FrontierBased* frontierBasedPtr;

void setUp(const vector<Cell*>& goalCells);
void setUp(const vector<Cell*>& startCell, const vector<Cell*>& goalCells);

void sleepFor(int milliseconds);

void setAllExplored(MouseLocal* mouse);

vector<Cell*> getBestAlgorithmPath(AStar* aStar, vector<Cell*>& goalCells, bool diagonalsAllowed, bool avoidGoalCells);

/**
 * @brief Turns the mouse from its current cell to face the next cell.
 * 
 * @param currentCell The current Cell position of the mouse.
 * @param nextCell The next Cell position to face.
 */
void turnMouseToNextCell(const Cell& currentCell, const Cell& nextCell);

/**
 * @brief Handles diagonal movements by analyzing the path and sending the correct commands to the API.
 * 
 * @param currCell The current Cell position of the mouse.
 * @param path The path as a string of movement commands.
 * @return string The modified path after handling diagonals.
 */
string diagonalizeAndRun(Cell& currCell, const string& path);

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
bool traversePathIteratively(MouseLocal* mouse, Cell& goalCell, bool diagonalsAllowed, bool allExplored, bool avoidGoalCells);

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
bool traversePathIteratively(MouseLocal* mouse, vector<Cell*>& goalCells, bool diagonalsAllowed, bool allExplored, bool avoidGoalCells);

#endif	// MAIN_H
#pragma once
#include <vector>

using namespace std;

#include "TessellatorStructures.h"

class Tessellator {

public:
    ////////////////////////////////////////////////////////////////////////////////
    // AUX FUNCTIONS
    ////////////////////////////////////////////////////////////////////////////////
    void Print2DVector(const vector<vector<int>> &grid);
    void PrintRectangle(const rectangle &rect, const int &gridHeight, const int &gridWidth);
    void PrintRectangleFile(char *filename, const rectangle &rect, const int &gridHeight, const int &gridWidth);

    bool CurrentRectIsOpen(const coord2D &_rect);
    bool CellIsOccupied(const vector<vector<int>> &initialGrid, const coord2D &_pos);
    bool PartialRectangleIsCorrect(const vector<vector<int>> &initialGrid, coord2D _currentRectInit, coord2D _currentRectEnd);
    void MarkPartialRectangleOccupied(vector<vector<int>> &currentGrid, coord2D _currentRectInit, coord2D _currentRectEnd, int value);
    bool IsGridComplete(const vector<vector<int>> &initialGrid);
    bool IsValidSolution(const vector<vector<int>> &initialGrid, const coord2D &_currentRect);
    bool IsValid(const vector<vector<int>> &occupiedGrid, const coord2D &_pos, const coord2D &_rect);
    int CalculateRectangleArea(const coord2D &_rectInit, const coord2D &_rectEnd);
    int CalculateNumBlanks(const vector<vector<int>> &initialGrid);
    bool takeOption(int option, const vector<vector<int>> &marksGrid, const coord2D &_lastPos, const coord2D &_lastRect, coord2D &_nextPos, coord2D &_nextRect,
        int &numBlanksClosed);

    ////////////////////////////////////////////////////////////////////////////////
    // ALGORITHM
    ////////////////////////////////////////////////////////////////////////////////
    void CalculateRectanglesRecursive(vector<vector<int>> &marksGrid, vector<rectangle> &bestSolution, int &bestCost,
        coord2D _currentPos, coord2D _currentRect, vector<rectangle> &_currentSolution, int &_currentCost,
        int level, int &numBlanks);
    int CalculateRectangles(const vector<vector<int>> &initialGrid, vector<rectangle> &solution);

private:

};

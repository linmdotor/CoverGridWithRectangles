#include <conio.h>
#include <vector>
#include <algorithm>
#include <iostream>
using namespace std;


////////////////////////////////////////////////////////////////////////////////
// STRUCTURES
////////////////////////////////////////////////////////////////////////////////
struct coord2D
{
    int x;
    int y;

    coord2D(int _x = 0, int _y = 0) :
            x(_x), y(_y) {}
};

struct rectangle
{
    coord2D corner1; // Upper-Left corner
    coord2D corner2; // Lower-Rigth corner

    rectangle(coord2D _c1 = coord2D(), coord2D _c2 = coord2D()) :
        corner1(_c1), corner2(_c2) {}
};

////////////////////////////////////////////////////////////////////////////////
// AUX FUNCTIONS
////////////////////////////////////////////////////////////////////////////////
void Print2DVector(const vector<vector<int>> &grid)
{
    for (int i = 0; i < grid.size(); i++)
    {
        for (int j = 0; j < grid[i].size(); j++)
        {
            std::cout << grid[i][j] << " ";
        }
        std::cout << endl;
    }
}

void PrintRectangle(const rectangle &rect, const int &gridHeight, const int &gridWidth)
{
    for (int i = 0; i < gridHeight; i++)
    {
        for (int j = 0; j < gridWidth; j++)
        {
            if ((rect.corner1.x <= j && rect.corner1.y <= i) &&
                (rect.corner2.x >= j && rect.corner2.y >= i))
                std::cout << "0 "; // prints '0' as part of the rectangle
            else
                std::cout << "- ";

        }
        std::cout << endl;
    }
}

bool CurrentRectIsOpen(const coord2D &_rect)
{
    return (_rect.x != -1 && _rect.y != -1);
}

bool CellIsOccupied(const vector<vector<int>> &initialGrid, const coord2D &_pos)
{
    return (initialGrid[_pos.y][_pos.x] == 1);
}

bool PartialRectangleIsCorrect(const vector<vector<int>> &initialGrid, coord2D _currentRectInit, coord2D _currentRectEnd)
{
    for (int i = _currentRectInit.x; i <= _currentRectEnd.x; i++)
    {
        for (int j = _currentRectInit.y; j <= _currentRectEnd.y; j++)
        {
            if (CellIsOccupied(initialGrid, coord2D(i, j)))
            {
                return false;
            }
        }
    }
    return true;
}

void MarkPartialRectangleOccupied(vector<vector<int>> &currentGrid, coord2D _currentRectInit, coord2D _currentRectEnd)
{
    //mark the currentRect as occupied
    for (int i = _currentRectInit.y; i <= _currentRectEnd.y; i++)
    {
        for (int j = _currentRectInit.x; j <= _currentRectEnd.x; j++)
        {
            currentGrid[i][j] = 1;
            //Print2DVector(currentGrid);
            //std::cout << "MARKED " << i << ", " << j << endl;
        }
    }
}

bool IsGridComplete(const vector<vector<int>> &initialGrid)
{
    for (int i = 0; i < initialGrid[0].size(); i++)
    {
        for (int j = 0; j < initialGrid.size(); j++)
        {
            if (!CellIsOccupied(initialGrid, coord2D(i, j)))
            {
                return false;
            }
        }
    }
    return true;
}

bool IsValidSolution(const vector<vector<int>> &initialGrid, const coord2D &_currentRect)
{
    return (!CurrentRectIsOpen(_currentRect) && IsGridComplete(initialGrid));
}

int CalculateRectangleArea(const coord2D &_rectInit, const coord2D &_rectEnd)
{
    return ((_rectEnd.x - _rectInit.x + 1) * (_rectEnd.y - _rectInit.y + 1));
}

int CalculateNumBlanks(const vector<vector<int>> &initialGrid)
{
    int numBlanks = 0;
    for (int i = 0; i < initialGrid[0].size(); i++)
    {
        for (int j = 0; j < initialGrid.size(); j++)
        {
            if (!CellIsOccupied(initialGrid, coord2D(i, j)))
            {
                numBlanks++;
            }
        }
    }
    return numBlanks;
}
////////////////////////////////////////////////////////////////////////////////
// ALGORITHM
////////////////////////////////////////////////////////////////////////////////
int CalculateRectanglesRecursive(vector<vector<int>> &initialGrid, vector<rectangle> &bestSolution, int &bestCost,
        coord2D _currentPos, coord2D _currentRect, vector<rectangle> _currentSolution, int _currentCost,
        int level, int &numBlanks)
{
    ////////////////////////////////////////////////////////////////////////////////
    // We are trying to solve the problem with a backtracking algorithm

    // We will go through the whole triangle, and in each cell we will "decide" what recursion take;
    // In principle, we have 4 possibilities:
    //  - Close the current rectangle, and move right
    //  - Close the current rectangle, and move down
    //  - Move right
    //  - Move down

    // We will evaluate all the possibilities, and will take the best of all.
    //  (with min. number of rectangles)
    ////////////////////////////////////////////////////////////////////////////////

    //for (int i = 0; i < level; i++)
    //{
    //    std::cout << "    :";
    //}
    //std::cout << _currentPos.x << ", " << _currentPos.y << endl;

    // IS VALID?
    // If the current cell is out of bounds, this partial solution doesn't work
    if (_currentPos.y >= initialGrid.size() ||
        _currentPos.x >= initialGrid[0].size())
    {
        return 0;
    }
    // If the current cell is occupied, and the currentRect is opened, this partial solution doesn't work
    if (CurrentRectIsOpen(_currentRect) &&
        CellIsOccupied(initialGrid, _currentPos))
    {
        return 0;
    }

    // PREPARE LEVEL ITERATION
    // "Open" a new rectangle if possible when then current is closed (rectangle(-1,-1))
    if (!CurrentRectIsOpen(_currentRect) &&
        !CellIsOccupied(initialGrid, _currentPos))
    {
        _currentRect = _currentPos;
    }

    // IS SOLUTION?
    //Evaluate if we have reached the end (numBlanks == 0)
    if(numBlanks == 0)
    {
        // IS VALID SOLUTION?
        // We have a solution only if the _currentRect has been closed and all the grid is occupied
        if(IsValidSolution(initialGrid, _currentRect))
        {
            std::cout << "TERMINADO RECURSION: " << _currentCost << " rectangles." << endl;
            //for each (rectangle rect in _currentSolution)
            //{
            //    PrintRectangle(rect, initialGrid.size(), initialGrid[0].size());
            //    std::cout << endl;
            //}
            return _currentCost;
        }
        else
        {
            return 0;
        }
    }
    else // BACKTRACKING (k+1)
    {
        // Take one of the recursive possibilities:

        // OPTIONS 1 & 2 - MOVE
        CalculateRectanglesRecursive(initialGrid, bestSolution, bestCost, coord2D(_currentPos.x + 1, _currentPos.y), _currentRect, _currentSolution, _currentCost, level + 1, numBlanks);
        CalculateRectanglesRecursive(initialGrid, bestSolution, bestCost, coord2D(_currentPos.x, _currentPos.y + 1), _currentRect,  _currentSolution, _currentCost, level + 1, numBlanks);

        // OPTIONS 3 & 4 - CLOSE THE CURRENT RECTANGLE (IF CORRECT) AND MOVE
        if (CurrentRectIsOpen(_currentRect) &&
            !CellIsOccupied(initialGrid, _currentPos))
        {

            if (PartialRectangleIsCorrect(initialGrid, _currentRect, _currentPos))
            {
                _currentSolution.push_back(rectangle(_currentRect, _currentPos));
                bestSolution.push_back(rectangle(_currentRect, _currentPos));

                MarkPartialRectangleOccupied(initialGrid, _currentRect, _currentPos);

                _currentCost++;
                bestCost++;
                numBlanks -= CalculateRectangleArea(_currentRect, _currentPos);
                _currentRect = coord2D(-1, -1);

                CalculateRectanglesRecursive(initialGrid, bestSolution, bestCost, coord2D(_currentPos.x + 1, _currentPos.y), _currentRect, _currentSolution, _currentCost, level + 1, numBlanks);
                CalculateRectanglesRecursive(initialGrid, bestSolution, bestCost, coord2D(_currentPos.x, _currentPos.y + 1), _currentRect, _currentSolution, _currentCost, level + 1, numBlanks);
            }
            else
            {
                return 0;
            }
        }

        return _currentCost;
    }
}

int CalculateRectangles(const vector<vector<int>> &initialGrid, vector<rectangle> &solution)
{
    vector<vector<int>> copyGrid = initialGrid;
    int numBlanks = CalculateNumBlanks(initialGrid);
    int numRects = 0;
    CalculateRectanglesRecursive(copyGrid, solution, numRects,
        coord2D(0,0), coord2D(-1,-1), vector<rectangle>(), 0,
        0, numBlanks);
    return numRects;
}

////////////////////////////////////////////////////////////////////////////////
// MAIN
////////////////////////////////////////////////////////////////////////////////
int main()
{
    vector<vector<int>> initialGrid {
        //{ 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,1,1,0,0,0,1,1,0,0,0,1,1,0,0,0 },
        //{ 0,1,0,0,0,0,1,1,0,1,0,1,0,0,1,0 },
        //{ 1,0,1,0,0,0,1,1,0,1,0,0,1,0,1,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,1,0,0,0,0,1,0,0,1,0,1,0,0,1,0 },
        //{ 1,0,1,0,0,0,0,1,0,1,0,0,1,0,1,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }

        //{ 1,0,0,0,0,0 },
        //{ 0,0,1,1,1,0 },
        //{ 0,1,1,0,1,0 },
        //{ 0,1,1,1,1,0 },
        //{ 1,0,1,0,0,0 },

        { 0,0 },
        { 0,1 }
    };

    std::cout << "INITIAL GRID:" << endl;
    Print2DVector(initialGrid);
    std::cout << endl;

    vector<rectangle> solution;
    int numRects = CalculateRectangles(initialGrid, solution);

    std::cout << "RESULT: " << numRects << " rectangles." << endl;
    for each (rectangle rect in solution)
    {
        PrintRectangle(rect, initialGrid.size(), initialGrid[0].size());
        std::cout << endl;
    }

    std::cout << "Press any key to Exit...";
    getch();
    return 0;
}
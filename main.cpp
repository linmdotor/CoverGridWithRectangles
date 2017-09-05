#include <conio.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
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
    std::cout << endl;
}

void PrintRectangleFile(char *filename, const rectangle &rect, const int &gridHeight, const int &gridWidth)
{
    ofstream myfile;
    myfile.open(filename, ios::app);

    for (int i = 0; i < gridHeight; i++)
    {
        for (int j = 0; j < gridWidth; j++)
        {
            if ((rect.corner1.x <= j && rect.corner1.y <= i) &&
                (rect.corner2.x >= j && rect.corner2.y >= i))
                myfile << "0 "; // prints '0' as part of the rectangle
            else
                myfile << "- ";

        }
        myfile << endl;
    }
    myfile << endl;
    myfile.close();
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
    if (!CurrentRectIsOpen(_currentRectInit)) // if closed, suppose it correct
    {
        return true;
    }

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

void MarkPartialRectangleOccupied(vector<vector<int>> &currentGrid, coord2D _currentRectInit, coord2D _currentRectEnd, int value)
{
    //mark the currentRect as occupied
    for (int i = _currentRectInit.y; i <= _currentRectEnd.y; i++)
    {
        for (int j = _currentRectInit.x; j <= _currentRectEnd.x; j++)
        {
            currentGrid[i][j] = value;
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

bool IsValid(const vector<vector<int>> &occupiedGrid, const coord2D &_pos, const coord2D &_rect)
{
    // If the current cell is out of bounds, this partial solution doesn't work
    if (_pos.y >= occupiedGrid.size() ||
        _pos.x >= occupiedGrid[0].size())
    {
        return false;
    }

    // If the currentRect is opened and the current cell is occupied, this partial solution doesn't work
    if (CurrentRectIsOpen(_rect) &&
        CellIsOccupied(occupiedGrid, _pos))
    {
        return false;
    }

    return true;
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

// Returns false in case of NON VALID OPTION for the current state
bool takeOption(int option, const vector<vector<int>> &marksGrid, const coord2D &_lastPos, const coord2D &_lastRect, coord2D &_nextPos, coord2D &_nextRect,
    int &numBlanksClosed)
{
    // IF POS IS OUT OF BOUNDS THERE IS NOT ANY VALID OPTION
    if (_lastPos.y >= marksGrid.size() ||
        _lastPos.x >= marksGrid[0].size())
    {
        return false;
    }

    // IF THE CURRENT PARTIAL RECTANGLE IS NOT VALID THERE IS NOT ANY VALID OPTION
    // TODO: would be better doing this check in each MOVE, only with the line expanded.
    if (!PartialRectangleIsCorrect(marksGrid, _lastRect, _lastPos))
    {
        return false;
    }

    // POSSIBLE OPTIONS (4):
    // (Each one, only when possible)
    //  - OPEN CURRENT RECT
    //  - CLOSE CURRENT RECT
    //  - MOVE RIGHT
    //  - MOVE DOWN
    switch (option)
    {
    case 0: // OPEN CURRENT RECT
        if (CurrentRectIsOpen(_lastRect)) // is open -> option invalid
            return false;

        if (CellIsOccupied(marksGrid, _lastPos)) // lastPos is occupied yet -> option invalid
            return false;

        _nextRect = _lastPos;
        _nextPos = _lastPos;
    break;

    // HACK: Close is the last option, to try to extend the rects as long as possible
    case 4: // CLOSE CURRENT RECT
        if (!CurrentRectIsOpen(_lastRect)) // is closed -> option invalid
            return false;

        if (CellIsOccupied(marksGrid, _lastPos)) // lastPos is occupied yet -> option invalid
            return false;

        numBlanksClosed = CalculateRectangleArea(_lastRect, _lastPos);
        _nextRect = coord2D(-1, -1);
        _nextPos = _lastPos;
    break;

    case 1: // MOVE RIGHT
        if (CurrentRectIsOpen(_lastRect) &&
            CellIsOccupied(marksGrid, _lastPos)) // is open and occupied -> option invalid
            return false;

        _nextRect = _lastRect;
        _nextPos = coord2D(_lastPos.x + 1, _lastPos.y);
    break;

    case 2: // MOVE DOWN
        if (CurrentRectIsOpen(_lastRect) &&
            CellIsOccupied(marksGrid, _lastPos)) // is open and occupied -> option invalid
            return false;

        _nextRect = _lastRect;
        _nextPos = coord2D(_lastPos.x, _lastPos.y + 1);
     break;

    // HACK: This "reset" has no meaning in a backtracking algorithm.
    case 3: // RESET // Para hacerlo funcionar, cambiar el "opt < 4" por "opt < 5"
        if (CurrentRectIsOpen(_lastRect)) // is open -> option invalid
            return false;

        _nextRect = _lastRect;

        //FIND FIRST
        coord2D firstZeroPos = coord2D(-1, -1);
        {
            for (int i = 0; i < marksGrid[0].size() && !CurrentRectIsOpen(firstZeroPos); i++)
            {
                for (int j = 0; j < marksGrid.size() && !CurrentRectIsOpen(firstZeroPos); j++)
                {
                    if (!CellIsOccupied(marksGrid, coord2D(i, j)))
                    {
                        firstZeroPos = coord2D(i, j);
                    }
                }
            }
        }
        _nextPos = firstZeroPos;
        break;
    }

    return true;
}
////////////////////////////////////////////////////////////////////////////////
// ALGORITHM
////////////////////////////////////////////////////////////////////////////////
void CalculateRectanglesRecursive(vector<vector<int>> &marksGrid, vector<rectangle> &bestSolution, int &bestCost,
    coord2D _currentPos, coord2D _currentRect, vector<rectangle> &_currentSolution, int &_currentCost,
    int level, int &numBlanks)
{
    ////////////////////////////////////////////////////////////////////////////////
    // We are trying to solve the problem with a backtracking algorithm

    // We will go through the whole triangle, and in each cell we will "decide" what recursion take;
    // In principle, we have 4 possibilities:
    //  - Open a new rectangle
    //  - Close the current rectangle
    //  - Move right
    //  - Move down

    // We will evaluate all the possibilities, and will take the best of all.
    //  (with min. number of rectangles)
    ////////////////////////////////////////////////////////////////////////////////

    int _opt = 0;
    for (int opt = 0; opt < 5; opt++) //it could be: while((opt<4)&&!success)
    {
        // takeOption will take EACH of the options for our current state.
        // Could be some redundant, that will be removed later

        // Temporary values, to use the same values for each opt
        coord2D _newPos;
        coord2D _newRect;
        int numBlanksClosed = 0;

        bool validOption = takeOption(opt, marksGrid, _currentPos, _currentRect, _newPos, _newRect, numBlanksClosed);

        // validOption -> this option "makes sense"
        // Will be evaluated as final solution, or will call recursive function
        if (validOption)
        {
            // IsFinalSolution
            if (numBlanks == 0)
            {
                if (IsValidSolution(marksGrid, _newRect))
                {
                    // SaveSolution
                    bestSolution = _currentSolution;
                    bestCost = _currentCost;
                }
            }
            else
            {
                // MARKS GRID AND REDUCE BLANKS when a rectangle has been closed
                if (numBlanksClosed > 0)
                {
                    MarkPartialRectangleOccupied(marksGrid, _currentRect, _currentPos, 1);
                    _currentSolution.push_back(rectangle(_currentRect, _currentPos));
                    _currentCost++;
                    numBlanks -= numBlanksClosed;
                    //level++;
                }

                // Recursive call
                CalculateRectanglesRecursive(marksGrid, bestSolution, bestCost,
                    _newPos, _newRect, _currentSolution, _currentCost,
                    level, numBlanks);


                // UNMARKS GRID AND INCREASE BLANKS
                //if (numBlanksClosed > 0)
                //{
                //    MarkPartialRectangleOccupied(marksGrid, _currentRect, _currentPos, 0);
                //    _currentSolution.pop_back();
                //    _currentCost--;
                //    numBlanks += numBlanksClosed;
                //    //level--;
                //}

                //
                // TODO: MAYBE, THE LEVEL CAN BE INCREASED ONLY WHEN A RECTANGLE HAS BEEN CLOSED
                //      TO BE ABLE TO KEEP THE SOLUTIONS IN AN ARRAY ???
                //
            }

        }

    } //for
}

int CalculateRectangles(const vector<vector<int>> &initialGrid, vector<rectangle> &solution)
{
    vector<vector<int>> copyGrid = initialGrid;
    int numBlanks = CalculateNumBlanks(copyGrid);
    int numRects = 0;
    int tempCost = 0;
    CalculateRectanglesRecursive(copyGrid, solution, numRects,
        coord2D(0,0), coord2D(-1,-1), vector<rectangle>(), tempCost,
        0, numBlanks);
    return numRects;
}

////////////////////////////////////////////////////////////////////////////////
// MAIN
////////////////////////////////////////////////////////////////////////////////
int main()
{
    vector<vector<int>> initialGrid {
        
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 },
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }
        

        ////BELIZE
        //{ 0,0,0,0,0,0,0,0,1,1,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,1,1,0,0,0,0,0,0,0,1,1,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,1,1,0 },
        //{ 0,0,0,0,1,1,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,1,1,0,0,0,0,0,0,0 },
        //{ 0,1,1,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,1,0,1,1,0,0,0,0,1,0,0 },
        //{ 0,0,1,0,0,0,0,0,0,0,1,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,1,1,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,1,1,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,1,1,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,1,1,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0,0,0,0,0,0,0,0 }



        //{ 0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0 },
        //{ 0,0,0,0,0,0 },
        //{ 0,0,0,0,1,0 },
        //{ 0,0,0,0,0,0 },
        //{ 1,0,0,0,0,0 },
        //{ 0,0,1,1,1,0 },
        //{ 0,1,1,0,1,0 },
        //{ 0,1,1,1,1,0 },
        //{ 1,0,1,0,0,0 },

        //{ 0,0 },
        //{ 0,1 }

        //{ 0,0,0 },
        //{ 0,1,1 },
        //{ 1,0,0 }
    };

    std::cout << "INITIAL GRID:" << endl;
    Print2DVector(initialGrid);
    std::cout << endl;

    vector<rectangle> solution;
    int numRects = CalculateRectangles(initialGrid, solution);

    std::cout << "RESULT: " << numRects << " rectangles." << endl;

    ofstream myfile;
    myfile.open("output.txt", ios::trunc);
    myfile.close();
    for each (rectangle rect in solution)
    {
        //PrintRectangle(rect, initialGrid.size(), initialGrid[0].size());

        PrintRectangleFile("output.txt", rect, initialGrid.size(), initialGrid[0].size());
    }
    std::cout << "RESULT: " << numRects << " rectangles." << endl;
    std::cout << "Press any key to Exit...";
    getch();
    return 0;
}
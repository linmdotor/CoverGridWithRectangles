#include <conio.h>
#include <vector>
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
            cout << grid[i][j] << " ";
        }
        cout << endl;
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
                cout << "0 "; // prints '0' as part of the rectangle
            else
                cout << "- ";

        }
        cout << endl;
    }
}

////////////////////////////////////////////////////////////////////////////////
// ALGORITHM
////////////////////////////////////////////////////////////////////////////////
int CalculateRectanglesRecursive(const vector<vector<int>> &initialGrid, vector<rectangle> &solution,
        coord2D _currentPos, coord2D _currentRectangle, int _currentNumRect, vector<rectangle> _currentSolution)
{
    // We are trying to solve the problem with a backtracking algorithm

    // We will go through the whole triangle, and in each cell we will "decide" what recursion take;
    // In principle, we have 4 possibilities:
    //  - Close the current rectangle, and move right
    //  - Close the current rectangle, and move down
    //  - Move right
    //  - Move down

    //We will evaluate all the possibilities, and will take the best of all.
    //  (with min. number of rectangles)

    return 0;
}

int CalculateRectangles(const vector<vector<int>> &initialGrid, vector<rectangle> &solution)
{
    return CalculateRectanglesRecursive(initialGrid, solution,
        coord2D(0,0), coord2D(-1,-1), 0, vector<rectangle>());
}

////////////////////////////////////////////////////////////////////////////////
// MAIN
////////////////////////////////////////////////////////////////////////////////
int main()
{
    vector<vector<int>> initialGrid {
        { 1,0,0,0,0,1 },
        { 0,0,0,0,0,1 },
        { 0,0,0,0,0,1 },
        { 0,0,0,1,1,1 },
        { 0,0,0,1,1,1 }
    };

    cout << "INITIAL GRID:" << endl;
    Print2DVector(initialGrid);
    cout << endl;

    vector<rectangle> solution;
    int numRects = CalculateRectangles(initialGrid, solution);

    cout << "RESULT: " << numRects << " rectangles." << endl;
    for each (rectangle rect in solution)
    {
        PrintRectangle(rect, initialGrid.size(), initialGrid[0].size());
        cout << endl;
    }

    cout << "Press any key to Exit...";
    getch();
    return 0;
}
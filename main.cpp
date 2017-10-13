#include <conio.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
using namespace std;

#include "Tessellator.h"
#include "AuxStructures.h"
#include "ACXUtilities.h"

////////////////////////////////////////////////////////////////////////////////
// MAIN
////////////////////////////////////////////////////////////////////////////////
int main(int argc, const char * argv[])
{
    Tessellator tess;
    ACXUtilities acxUtils;

    if (argc != 5)
    {
        std::cout << "ERROR: you must pass 4 parameters (path, ACXfilename, ACXFilenameBACKUP, ACXFilenameNEW)" << endl;
        return -1;
    }

    const char *path = argv[1];
    const char *ACXFilename = argv[2];
    const char *ACXFilenameBACKUP = argv[3];
    const char *ACXFilenameNEW = argv[4];

    std::cout << "Loading " << path << ACXFilename << "..." << endl;
    acxUtils.LoadACX(path, ACXFilename, ACXFilenameBACKUP);

    std::cout << "Parsing to Array..." << endl;
    vector<vector<int>> initialGrid = acxUtils.ParseToArray();

    vector<rectangle> solution;
    std::cout << "Calculating solution..." << endl;
    int numRects = tess.CalculateRectangles(initialGrid, solution);
    std::cout << "RESULT: " << numRects << " NEW rectangles." << endl;

    std::cout << "Adding new StreamedAreas..." << endl;
    acxUtils.CreateNewStreamedAreas(solution);

    std::cout << "Generating tessellation of MeshBarriers navMeshes..." << endl;
    acxUtils.GenerateTessellatedMeshBarriersAndNavMeshes();

    std::cout << "Creating connections..." << endl;
    acxUtils.CreateConnections();

    //std::cout << "Creating pathfinding character..." << endl;
    //acxUtils.CreatePathFindingCharacter();

    std::cout << "Saving result into ACX file..." << endl;
    acxUtils.ExportInventory(path, ACXFilenameNEW);

    std::cout << endl;
    return 0;
}
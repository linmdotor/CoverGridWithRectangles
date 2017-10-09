#pragma once
#include <vector>

using namespace std;

#include "AuxStructures.h"

#define _SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS

#include <ACE_Simulation/ACE_Core.h> // ACE_Inventory

class ACE_Solver;
class ACE_StreamedArea;
class ACE_WayPoint;

class ACXUtilities {

public:
    int LoadACX(const std::string path, const std::string filename);
    vector<vector<int>> ParseToArray();
    void CreateNewStreamedAreas(vector<rectangle> rectangles);
    void CreateConnections();
    void ExportInventory(const std::string path, const std::string filename);

private:
    // This inventory will own and manage all AI-implant data.
    ACE_Solver *_mainSolver;
    ACE_Inventory _inventory;
    ACE_IInventoryItem::ItemArray _streamedAreasArray;
    double _cellSize;
    int _numCellsX, _numCellsY;
    BGT_V4 _initPos;

    ACE_StreamedArea* FindFirstStreamedAreaInPoint(BGT_V4 point, ACE_IInventoryItem::ItemArray streamedAreasArray);
    bool PointIsIntoStreamedArea(BGT_V4 point, ACE_StreamedArea * area);
    bool AreaIsAdjacentTo(ACE_StreamedArea * area1, ACE_StreamedArea * area2);
    void Connect2StreamedAreas(ACE_StreamedArea * area1, ACE_StreamedArea * area2);
    ACE_WayPoint* CreateWayPoint(float posX, float posY, int ID, float radius);
    void CalculateInitPosAndNumCells(BGT_V4 worldSize, BGT_V4 rectanglePosition, double cellSize);
};

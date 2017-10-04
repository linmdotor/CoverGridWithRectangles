#pragma once
#include <vector>

using namespace std;

#include "AuxStructures.h"

#define _SILENCE_STDEXT_HASH_DEPRECATION_WARNINGS

#include <ACE_Simulation/ACE_Core.h> // ACE_Inventory

class ACXUtilities {

public:
    int LoadACX(const std::string path, const std::string filename);
    vector<vector<int>> ParseToArray();

private:
    // This inventory will own and manage all AI-implant data.
    ACE_Inventory _inventory;

};


//int CalculateRectangles(const vector<vector<int>> &initialGrid, vector<rectangle> &solution)
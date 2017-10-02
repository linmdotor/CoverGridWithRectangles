#pragma once

////////////////////////////////////////////////////////////////////////////////
// AUXILIAR STRUCTURES
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
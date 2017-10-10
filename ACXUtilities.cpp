#include "ACXUtilities.h"

#include <math.h>

///////////////////////
// AI-Implant INCLUDES
///////////////////////

#include <stdlib.h>

// AI-implant modules
#include <ACE_Simulation/ACE_Core.h>
#include <ACE_Behaviour/ACE_BehaviourSolver.h>
#include <ACE_ActionSelection/ACE_ActionSelectionSolver.h>
#include <ACE_Physics/ACE_SurfaceSolver.h>
#include <ACE_Physics/ACE_CollisionSolver.h>
#include <ACE_World/ACE_EnvironmentSolver.h>
#include <ACE_World/ACE_TrafficSolver.h>
#include <ACE_World/ACE_WayPoint.h>
#include <ACE_World/ACE_MetaConnection.h>
#include <ACE_World/ACE_MetaConnectionNetwork.h>

// For reading ACX files
#include <ACP_Import/ACP_Import.h>
#include <ACP_Import/ACP_Export.h>

// Set this to 0 to disable the debug server
#define IMPLEMENT_DEBUG_SERVER 0

// For the debug server
#if IMPLEMENT_DEBUG_SERVER
#include <ACP_Import/ACP_DebugServer.h>
#endif


#define round(x) (x<0?std::ceil((x)-0.5):std::floor((x)+0.5))

int ACXUtilities::LoadACX(const std::string path, const std::string filename, const std::string filenameBACKUP)
{
    // Initialize AI-implant components
    ACE_Core::InitializeModule();
    ACE_BehaviourSolver::InitializeModule();
    ACE_ActionSelectionSolver::InitializeModule();
    ACE_SurfaceSolver::InitializeModule();
    ACE_CollisionSolver::InitializeModule();
    ACE_EnvironmentSolver::InitializeModule();
    ACE_TrafficSolver::InitializeModule();

    // Import an ACX file into the inventory.
    // This will create AI-implant solvers, characters, and other
    // world markup objects.
    ACP_Import importer;
    _inventory.GetStreamedAreaManager().SetMasterPath(path.c_str());
    std::string complete_path = path + filename;
    if (!importer.Import(complete_path.c_str(), &_inventory))
    {
        BGT_String message("Error importing ACX file \"");
        message += (filename).c_str();
        message += "\".";
        BGT_LOG_ERROR(0, message);
        return EXIT_FAILURE;
    }

    // Make a backup of the original file, because it could be overwritten later
    ACP_Export exporter;
    std::string export_path = path + filenameBACKUP;
    exporter.Export(export_path.c_str(), &_inventory);

    return EXIT_SUCCESS;
}

vector<vector<int>> ACXUtilities::ParseToArray()
{
    // Find the first StreamedArea and it dimensions.
    // In order to the algorithm works, the grid will always have the same dimensions,
    // and will be formed by squares (height == width).
    ACE_IInventoryItem *solverItem = _inventory.GetRootItem()->GetFirstActiveChildOfType(BGT_OBJECT_TYPE(ACE_Solver));
    if (solverItem == NULL)
    {
        BGT_LOG_ERROR(0, "No mainSolver specified in imported ACX file");
        return vector<vector<int>>();
    }
    _mainSolver = (ACE_Solver *)solverItem;

    _mainSolver->GetDescendants(&_streamedAreasArray, BGT_OBJECT_TYPE(ACE_StreamedArea));

    ACE_IInventoryItem *strItem = _mainSolver->GetFirstActiveChildOfType(BGT_OBJECT_TYPE(ACE_StreamedArea));
    if (_streamedAreasArray.GetSize() == 0)
    {
        BGT_LOG_ERROR(0, "No StreamedArea specified in imported ACX file");
        return vector<vector<int>>();
    }
    ACE_StreamedArea *firstStrArea = (ACE_StreamedArea *)_streamedAreasArray[0];

    _cellSize = round(firstStrArea->GetPoint2().x - firstStrArea->GetPoint1().x);

    // Calculate the num of cells from the worldSize and cellSize
    // And the initialPosition (it isn't the limit of the world)
    BGT_V4 worldSize;
    _mainSolver->GetWorldSize(&worldSize);
    CalculateInitPosAndNumCells(worldSize, firstStrArea->GetPoint1(), _cellSize);

    // Output grid
    vector<vector<int>> resultGrid(_numCellsY, vector<int>(_numCellsX));

    // We are going through each cell (adding cellSize), and loooking if the position
    // corresponds to any StreamedArea
    // In AI.Implant the X axis increses to th right, and Y axis increases to upper.
    double startPosX = _initPos.x + (_cellSize / 2.0);
    double startPosY = _initPos.y - (_cellSize / 2.0);

    for (int i = 0; i < _numCellsY; ++i)
    {
        for (int j = 0; j < _numCellsX; ++j)
        {
            BGT_V4 currentPos = BGT_V4_STATIC_CONSTRUCT(startPosX + (j*_cellSize), startPosY - (i*_cellSize), 0, 0);

            if (FindFirstStreamedAreaInPoint(currentPos, _streamedAreasArray) != NULL)
            {
                resultGrid[i][j] = 1;
            }
            else
            {
                resultGrid[i][j] = 0;
            }
        }
    }

    return resultGrid;
}

void ACXUtilities::CreateNewStreamedAreas(vector<rectangle> rectangles)
{
    // Go through the grid and multiplies the coord2D for cellSize to set the new rectangles.
    for (auto rect = rectangles.begin(); rect != rectangles.end(); ++rect)
    {
        BGT_V4 point1 = BGT_V4_STATIC_CONSTRUCT(_initPos.x + (rect->corner1.x*_cellSize), _initPos.y - (rect->corner1.y*_cellSize), 0, 0);
        BGT_V4 point2 = BGT_V4_STATIC_CONSTRUCT(_initPos.x + (rect->corner2.x*_cellSize) + _cellSize, _initPos.y - (rect->corner2.y*_cellSize) - _cellSize, 0, 0);

        // TODO: cambiar los streamSource, streamTrigger, path y parámetros del StreamArea, para que no se cargue nada adicional {jfmartinezd]
        ACE_StreamedArea *newArea = (ACE_StreamedArea *)_streamedAreasArray[0]->Clone();

        newArea->SetName("StreamedArea_NEW");
        newArea->SetPoint1(point1);
        newArea->SetPoint2(point2);
        // Adds the new StreamArea to the inventory and the array
        _mainSolver->AddChild(newArea);
        _streamedAreasArray.Append(newArea);
    }
}


////// TODO: Otra forma bastante más eficiente sería ir recorriendo todos los Streamed Areas
////// Y por cada uno, ir preguntando si el resto están "pegados" (viendo las coordenadas (x,y) de p1 y p2
////// La lógica de "AreaIsAdjacentTo" sería algo así como comprobar si alguna coordenada de p1
////// es igual a alguna de p2, y si es así podría ser que estuvieran pegados, siempre que las otras
////// coordenadas cumplieran alguna condición.
////// Se podría implementar, pero de momento se descarta la idea por haber encontrado otra solución factible. 
////// Sería algo así:

//////for (int i = 0; i < _streamedAreasArray.GetSize(); ++i)
//////{
//////    for (int j = i + 1; j < _streamedAreasArray.GetSize(); ++j)
//////    {
//////        if (AreaIsAdjacentTo((ACE_StreamedArea *)_streamedAreasArray[i], (ACE_StreamedArea *)_streamedAreasArray[j]))
//////        {
//////            adjacencyLists[i].push_back(j);
//////            adjacencyLists[j].push_back(i);
//////        }
//////    }
//////}
void ACXUtilities::CreateConnections()
{
    // remove all the existant connections and waypoints (to simplify)
    ACE_IInventoryItem::ItemArray wayPoints;
    _mainSolver->GetDescendants(&wayPoints, BGT_OBJECT_TYPE(ACE_WayPoint));
    for (int i = 0; i < wayPoints.GetSize(); ++i)
    {
        _inventory.RemoveItem(wayPoints[i]);
    }

    ACE_IInventoryItem::ItemArray metaconnections;
    _mainSolver->GetDescendants(&metaconnections, BGT_OBJECT_TYPE(ACE_MetaConnection));

    for (int i = 0; i < metaconnections.GetSize(); ++i)
    {
        _inventory.RemoveItem(metaconnections[i]->GetId());
    }

    // Adjacency List (graph) with all StreamedAreas connected.
    // If 1 & 2 are connected, "2" will be in the 1's list, BUT "1" WILL NOT BE IN 2's list
    vector<vector<int>> adjacencyLists;
    for (int i = 0; i < _streamedAreasArray.GetSize(); ++i)
    {
        adjacencyLists.push_back(vector<int>());
    }

    //
    // The algorithm will go through all the logic cells, and check if the current position
    // has changed from one to other StreamedArea. Then, there is a link between the current and
    // the previous areas.
    // We must repeat this logic in horizontal and vertical to find all the connections.
    // We start in the upper-left corner.
    //

    // Starting point (init pos + little offset)
    double startPosX = _initPos.x + (_cellSize / 2.0);
    double startPosY = _initPos.y - (_cellSize / 2.0);

    ACE_StreamedArea *previousArea;
    ACE_StreamedArea *currentArea;

    int totalLinks = 0;

    // FIND HORIZONTAL CONNECTIONS
    for (int i = 0; i < _numCellsY; ++i)
    {
        previousArea = NULL;
        currentArea = NULL;

        for (int j = 0; j < _numCellsX; ++j)
        {
            BGT_V4 currentPos = BGT_V4_STATIC_CONSTRUCT(startPosX + (j*_cellSize), startPosY - (i*_cellSize), 0, 0);

            currentArea = FindFirstStreamedAreaInPoint(currentPos, _streamedAreasArray);

            if (previousArea != NULL && currentArea == NULL)
            {
                std::cout << "ERROR: ALL POSITIONS MUST HAVE A STREAMEDAREA." << endl;
                previousArea = NULL;
            }
            else if (previousArea != NULL && currentArea->GetId() != previousArea->GetId())
            {
                int previousAreaIndx = _streamedAreasArray.Find(previousArea);
                int currentAreaIndx = _streamedAreasArray.Find(currentArea);

                // If it is not inserted yet
                if (std::find(adjacencyLists[previousAreaIndx].begin(), adjacencyLists[previousAreaIndx].end(), currentAreaIndx) == adjacencyLists[previousAreaIndx].end())
                {
                    adjacencyLists[previousAreaIndx].push_back(currentAreaIndx);
                    totalLinks++;
                }
            }

            previousArea = currentArea;
        }
    }

    // FIND VERTICAL CONNECTIONS
    for (int j = 0; j < _numCellsX; ++j)
    {
        previousArea = NULL;
        currentArea = NULL;

        for (int i = 0; i < _numCellsY; ++i)
        {
            BGT_V4 currentPos = BGT_V4_STATIC_CONSTRUCT(startPosX + (j*_cellSize), startPosY - (i*_cellSize), 0, 0);

            currentArea = FindFirstStreamedAreaInPoint(currentPos, _streamedAreasArray);

            if (previousArea != NULL && currentArea == NULL)
            {
                std::cout << "ERROR: ALL POSITIONS MUST HAVE A STREAMEDAREA." << endl;
                previousArea = NULL;
            }
            else if (previousArea != NULL && currentArea->GetId() != previousArea->GetId())
            {
                int previousAreaIndx = _streamedAreasArray.Find(previousArea);
                int currentAreaIndx = _streamedAreasArray.Find(currentArea);

                // If it is not inserted yet
                if (std::find(adjacencyLists[previousAreaIndx].begin(), adjacencyLists[previousAreaIndx].end(), currentAreaIndx) == adjacencyLists[previousAreaIndx].end())
                {
                    adjacencyLists[previousAreaIndx].push_back(currentAreaIndx);
                    totalLinks++;
                }
            }

            previousArea = currentArea;
        }
    }

    // CONNECT ALL THE FOUND CONNECTIONS.
    for (int vector1Idx=0; vector1Idx < adjacencyLists.size(); ++vector1Idx)
    {
        ACE_StreamedArea *area1 = (ACE_StreamedArea *)_streamedAreasArray[vector1Idx];
        for (int vector2Idx = 0; vector2Idx < adjacencyLists[vector1Idx].size(); ++vector2Idx)
        {
            ACE_StreamedArea *area2 = (ACE_StreamedArea *)_streamedAreasArray[adjacencyLists[vector1Idx][vector2Idx]];
            Connect2StreamedAreas(area1, area2);
        }
    }

    std::cout << "TOTAL LINKS: " << totalLinks << endl;
}

void ACXUtilities::ExportInventory(const std::string path, const std::string filename)
{
    //Export the current Inventory to an ACX output file
    ACP_Export exporter;
    std::string export_path = path + filename;
    exporter.Export(export_path.c_str(), &_inventory);
}


ACE_StreamedArea* ACXUtilities::FindFirstStreamedAreaInPoint(BGT_V4 point, ACE_IInventoryItem::ItemArray streamedAreasArray)
{
    // For each StreamedArea
    for (int strAreaIdx = 0; strAreaIdx < streamedAreasArray.GetSize(); ++strAreaIdx)
    {
        // It's within the bounds
        ACE_StreamedArea *strArea = (ACE_StreamedArea *)streamedAreasArray[strAreaIdx];
        if (PointIsIntoStreamedArea(point, strArea))
        {
            return strArea;
        }
    }

    return NULL;
}

bool ACXUtilities::PointIsIntoStreamedArea(BGT_V4 point, ACE_StreamedArea * area)
{
    // There is 4 possibilities that the point will be INTO the area.
    // "1" is the point1 of the area, "2" is the point2 of the area, and the "X" is the point
    //
    //    OPT1           OPT2           OPT3           OPT4
    // 1---------|    ----------2    2---------|    ----------1
    // |         |    |         |    |         |    |         |
    // |    X    |    |    X    |    |    X    |    |    X    |
    // |         |    |         |    |         |    |         |
    // ----------2    1----------    ----------1    2----------
    //
    // We must check the 4 possibilities:

    bool opt1 = point.x >= area->GetPoint1().x &&
                point.x <= area->GetPoint2().x &&
                point.y >= area->GetPoint1().y &&
                point.y <= area->GetPoint2().y;

    bool opt2 = point.x >= area->GetPoint1().x &&
                point.x <= area->GetPoint2().x &&
                point.y <= area->GetPoint1().y &&
                point.y >= area->GetPoint2().y;

    bool opt3 = point.x <= area->GetPoint1().x &&
                point.x >= area->GetPoint2().x &&
                point.y <= area->GetPoint1().y &&
                point.y >= area->GetPoint2().y;

    bool opt4 = point.x <= area->GetPoint1().x &&
                point.x >= area->GetPoint2().x &&
                point.y >= area->GetPoint1().y &&
                point.y <= area->GetPoint2().y;

    // returns TRUE if any of the options
    return opt1 || opt2 || opt3 || opt4;
}

bool ACXUtilities::AreaIsAdjacentTo(ACE_StreamedArea * area1, ACE_StreamedArea * area2)
{
    // 1. SUPONEMOS a2 está ARRIBA de a1
    //
    // 2. SUPONEMOS a2 está DERECHA de a1
    //
    // 3. SUPONEMOS a2 está ABAJO de a1
    //
    // 4. SUPONEMOS a2 está IZDA de a1
    //

    return true;
}

int WayPointCounter = 0;

void ACXUtilities::Connect2StreamedAreas(ACE_StreamedArea * area1, ACE_StreamedArea * area2)
{
    // The areas are connected in the middle of the 2 coincident areas.
    // We will create 2 Waypoints (1 in each area), add it to the MainMetaConnection,
    // and create the MetaConnection.

    BGT_V4 a1_p1 = BGT_V4_STATIC_CONSTRUCT(round(area1->GetPoint1().x), round(area1->GetPoint1().y), 0, 0);
    BGT_V4 a1_p2 = BGT_V4_STATIC_CONSTRUCT(round(area1->GetPoint2().x), round(area1->GetPoint2().y), 0, 0);
    BGT_V4 a2_p1 = BGT_V4_STATIC_CONSTRUCT(round(area2->GetPoint1().x), round(area2->GetPoint1().y), 0, 0);
    BGT_V4 a2_p2 = BGT_V4_STATIC_CONSTRUCT(round(area2->GetPoint2().x), round(area2->GetPoint2().y), 0, 0);

    // TODO: Checkear que al menos tienen un lado en común [jfmartinezd]

    // Calculate the segment of intersection from the 2 rectangles
    // - https://stackoverflow.com/questions/19753134/get-the-points-of-intersection-from-2-rectangles
    // There are several ways to do it.
    // The max&min way forces us to normalize the rectangles (p1.x < p2.x && p1.y < p2.y),
    // but we can't we cannot guarantee it.
    // Instead, we will sort the 4 Xcoord and Ycoord, and will take the 2 of te middle.

    int xs[4] = { a1_p1.x, a1_p2.x, a2_p1.x, a2_p2.x };
    int ys[4] = { a1_p1.y, a1_p2.y, a2_p1.y, a2_p2.y };
    std::sort(xs, xs+4);
    std::sort(ys, ys+4);

    BGT_V4 segment_p1 = BGT_V4_STATIC_CONSTRUCT(xs[1], ys[1], 0, 0);
    BGT_V4 segment_p2 = BGT_V4_STATIC_CONSTRUCT(xs[2], ys[2], 0, 0);

    // Calculate the middlePoint (the center of both coordinates).
    BGT_V4 middlePoint = BGT_V4_STATIC_CONSTRUCT((segment_p1.x + segment_p2.x)/2.0f, (segment_p1.y + segment_p2.y)/2.0f, 0, 0);

    ACE_WayPoint *wPoint1;
    ACE_WayPoint *wPoint2;

    // Create the points, and move the middlePoint depending on the orientation
    if (segment_p1.x == segment_p2.x) // vertical segment
    {
        wPoint1 = CreateWayPoint(middlePoint.x - 500.0f, middlePoint.y, WayPointCounter++, 500);
        wPoint2 = CreateWayPoint(middlePoint.x + 500.0f, middlePoint.y, WayPointCounter++, 500);
    }
    else if (segment_p1.y == segment_p2.y) // horizontal segment
    {
        wPoint1 = CreateWayPoint(middlePoint.x, middlePoint.y - 500.0f, WayPointCounter++, 500);
        wPoint2 = CreateWayPoint(middlePoint.x, middlePoint.y + 500.0f, WayPointCounter++, 500);
    }
    else
    {
        std::cout << "ERROR: X or Y MUST BE THE SAME..." << endl;
    }

    ACE_MetaConnectionNetwork *metaConnectionNet = (ACE_MetaConnectionNetwork *)_mainSolver->GetFirstActiveChildOfType(BGT_OBJECT_TYPE(ACE_MetaConnectionNetwork));
    metaConnectionNet->AddWayPoint(wPoint1);
    metaConnectionNet->AddWayPoint(wPoint2);

    ACE_MetaConnection *metaConn = ACE_MetaConnection::CreateObject();
    metaConn->SetSourceId(wPoint1->GetId());
    metaConn->SetDestinationId(wPoint2->GetId());
    metaConn->SetWidth(BGT_V4_GetDistance2(segment_p1, segment_p2) / 2.0f);
    metaConn->SetType(ACE_MetaConnection::typeMULTI_EDGE_TO_MULTI_EDGE);
    metaConn->SetIsBidirectional(true);

    metaConnectionNet->AddChild(metaConn);
}

ACE_WayPoint* ACXUtilities::CreateWayPoint(float posX, float posY, int ID, float radius)
{
    ACE_WayPoint *wPoint = ACE_WayPoint::CreateObject();
    std::ostringstream wpName;
    wpName << "WayPoint_" << ID;
    wPoint->SetName(wpName.str().c_str());
    wPoint->SetShapeFat(radius);
    wPoint->SetPosition(BGT_V4_STATIC_CONSTRUCT(posX, posY, 0, 0));

    _mainSolver->AddChild(wPoint);

    return wPoint;
}

void ACXUtilities::CalculateInitPosAndNumCells(BGT_V4 worldSize, BGT_V4 rectanglePosition, double cellSize)
{
    // We can't do floor(worldSize.x / cellSize); and floor(worldSize.y / cellSize);
    // because the cells have an offset.
    // We have to calculate the real init and end points (with the offset)
    float minX = round(rectanglePosition.x);
    while ((minX - cellSize) >= -(worldSize.x / 2.0f))
    {
        minX -= cellSize;
    }
    float maxX = round(rectanglePosition.x);
    while ((maxX + cellSize) <= (worldSize.x / 2.0f))
    {
        maxX += cellSize;
    }

    float minY = round(rectanglePosition.y);
    while ((minY - cellSize) >= -(worldSize.y / 2.0f))
    {
        minY -= cellSize;
    }
    float maxY = round(rectanglePosition.y);
    while ((maxY + cellSize) <= (worldSize.y / 2.0f))
    {
        maxY += cellSize;
    }

    _initPos = BGT_V4_STATIC_CONSTRUCT(minX, maxY, 0, 0);

    _numCellsX = floor((maxX - minX) / cellSize);
    _numCellsY = floor((maxY - minY) / cellSize);
}

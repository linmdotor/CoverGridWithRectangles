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

int ACXUtilities::LoadACX(const std::string path, const std::string filename)
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
    std::string complete_path = path + filename + ".acx";
    if (!importer.Import(complete_path.c_str(), &_inventory))
    {
        BGT_String message("Error importing ACX file \"");
        message += (filename + ".acx").c_str();
        message += "\".";
        BGT_LOG_ERROR(0, message);
        return EXIT_FAILURE;
    }

    // Make a backup of the original file, because it will be overwritten later
    ACP_Export exporter;
    std::string export_path = path + filename + "_OLD.acx";
    exporter.Export(export_path.c_str(), &_inventory);

    return EXIT_SUCCESS;
}

vector<vector<int>> ACXUtilities::ParseToArray()
{
    // Busca el primer StreamedArea y saca de él sus dimensiones.
    // Damos por hecho que la cuadrícula siempre tiene las mismas dimensiones, y son
    // cuadrados (height == width), si no, el algoritmo no funciona.
    ACE_IInventoryItem *solverItem = _inventory.GetRootItem()->GetFirstActiveChildOfType(BGT_OBJECT_TYPE(ACE_Solver));
    if (solverItem == NULL)
    {
        BGT_LOG_ERROR(0, "No mainSolver specified in imported ACX file");
        //return EXIT_FAILURE;
        return vector<vector<int>>();
    }
    ACE_Solver *mainSolver = (ACE_Solver *)solverItem;

    ACE_IInventoryItem::ItemArray streamedAreasArray;
    mainSolver->GetChildren(&streamedAreasArray, BGT_OBJECT_TYPE(ACE_StreamedArea));

    ACE_IInventoryItem *strItem = mainSolver->GetFirstActiveChildOfType(BGT_OBJECT_TYPE(ACE_StreamedArea));
    if (streamedAreasArray.GetSize() == 0)
    {
        BGT_LOG_ERROR(0, "No StreamedArea specified in imported ACX file");
        //return EXIT_FAILURE;
        return vector<vector<int>>();
    }
    ACE_StreamedArea *firstStrArea = (ACE_StreamedArea *)streamedAreasArray[0];

    double cellSize = round(firstStrArea->GetPoint2().x - firstStrArea->GetPoint1().x);

    // Saca las dimensiones del mundo, para ver cuántas cuadrículas cabrían
    BGT_V4 wordSize;
    mainSolver->GetWorldSize(&wordSize);
    int numCellsX = floor(wordSize.x / cellSize);
    int numCellsY = floor(wordSize.y / cellSize);

    //Crea una matriz para almacenar el resultado
    vector<vector<int>> resultGrid(numCellsY, vector<int>(numCellsX));

    // Dadas dichas dimensiones vamos recorriendo de cellSize en cellSize, viendo
    // si cada punto se encuentra dentro de algún StreamedArea
    // En AI.Implant las X crecen hacia la derecha y las Y crecen hacia arriba
    // se comienza en el medio de donde estaría la primera casilla (superior izquierda)
    double startPosX = -round(wordSize.x / 2.0) + round(cellSize / 2.0);
    double startPosY = round(wordSize.y / 2.0) - round(cellSize / 2.0);

    for (int i = 0; i < numCellsY; i++)
    {
        for (int j = 0; j < numCellsX; j++)
        {
            BGT_V4 currentPos = BGT_V4_STATIC_CONSTRUCT(startPosX + (j*cellSize), startPosY - (i*cellSize), 0, 0);

            if (FindFirstStreamedAreaInPoint(currentPos, streamedAreasArray) != NULL)
            {
                resultGrid[i][j] = 1;
            }
            else
            {
                resultGrid[i][j] = 0;
            }
        }
    }

    // TODO: Controlar el caso de que crea una fila de más (si cabe... :S) [jfmartinezd]

    return resultGrid;
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
    return (point.x > area->GetPoint1().x && point.x < area->GetPoint2().x &&
        point.y > area->GetPoint1().y && point.y < area->GetPoint2().y);
}
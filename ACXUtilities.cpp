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
    std::string export_path = path + filename + "_BACKUP.acx";
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
    _mainSolver = (ACE_Solver *)solverItem;

    _mainSolver->GetChildren(&_streamedAreasArray, BGT_OBJECT_TYPE(ACE_StreamedArea));

    ACE_IInventoryItem *strItem = _mainSolver->GetFirstActiveChildOfType(BGT_OBJECT_TYPE(ACE_StreamedArea));
    if (_streamedAreasArray.GetSize() == 0)
    {
        BGT_LOG_ERROR(0, "No StreamedArea specified in imported ACX file");
        //return EXIT_FAILURE;
        return vector<vector<int>>();
    }
    ACE_StreamedArea *firstStrArea = (ACE_StreamedArea *)_streamedAreasArray[0];

    double cellSize = round(firstStrArea->GetPoint2().x - firstStrArea->GetPoint1().x);

    // Saca las dimensiones del mundo, para ver cuántas cuadrículas cabrían
    BGT_V4 worldSize;
    _mainSolver->GetWorldSize(&worldSize);
    int numCellsX = floor(worldSize.x / cellSize);
    int numCellsY = floor(worldSize.y / cellSize);

    //Crea una matriz para almacenar el resultado
    vector<vector<int>> resultGrid(numCellsY, vector<int>(numCellsX));

    // Dadas dichas dimensiones vamos recorriendo de cellSize en cellSize, viendo
    // si cada punto se encuentra dentro de algún StreamedArea
    // En AI.Implant las X crecen hacia la derecha y las Y crecen hacia arriba
    // se comienza en el medio de donde estaría la primera casilla (superior izquierda)
    double startPosX = -round(worldSize.x / 2.0) + round(cellSize / 2.0);
    double startPosY = round(worldSize.y / 2.0) - round(cellSize / 2.0);

    for (int i = 0; i < numCellsY; i++)
    {
        for (int j = 0; j < numCellsX; j++)
        {
            BGT_V4 currentPos = BGT_V4_STATIC_CONSTRUCT(startPosX + (j*cellSize), startPosY - (i*cellSize), 0, 0);

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

void ACXUtilities::CreateNewStreamedAreas(vector<rectangle> rectangles, const std::string path, const std::string filename)
{
    // Primero debemos encontrar el punto de inicio en el ACX, desde donde empezar a crear rectángulos,
    // porque no es el -worldSize/2
    // Para eso cogemos un StreamedArea al azar, y vamos restando cellSize hasta salir del "mundo"
    ACE_StreamedArea *firstStrArea = (ACE_StreamedArea *)_streamedAreasArray[0];
    double cellSize = round(firstStrArea->GetPoint2().x - firstStrArea->GetPoint1().x);

    BGT_V4 worldSize;
    _mainSolver->GetWorldSize(&worldSize);

    float initX = round(firstStrArea->GetPoint1().x);
    while ((initX - cellSize) >= -(worldSize.x/2.0f))
    {
        initX -= cellSize;
    }

    float initY = round(firstStrArea->GetPoint1().y);
    while ((initY + cellSize) <= (worldSize.y / 2.0f))
    {
        initY += cellSize;
    }

    // Luego, vamos recorriendo el vector, y vamos multiplicando las coord2D por el cellSize
    // Se deberían crear los rectángulos siguiendo la misma orientación que los actuales
    for (auto rect = rectangles.begin(); rect != rectangles.end(); ++rect)
    {
        BGT_V4 point1 = BGT_V4_STATIC_CONSTRUCT(initX + (rect->corner1.x*cellSize), initY - (rect->corner1.y*cellSize), 0, 0);
        BGT_V4 point2 = BGT_V4_STATIC_CONSTRUCT(initX + (rect->corner2.x*cellSize) + cellSize, initY - (rect->corner2.y*cellSize) - cellSize, 0, 0);

        ACE_StreamedArea *newArea = (ACE_StreamedArea *)firstStrArea->Clone();

        newArea->SetName("StreamedArea_NEW");
        newArea->SetPoint1(point1);
        newArea->SetPoint2(point2);
        _mainSolver->AddChild(newArea);
    }

    //Almacena los cambios en el fichero de salida
    ACP_Export exporter;
    std::string export_path = path + filename + "_NEW.acx";
    exporter.Export(export_path.c_str(), &_inventory);
}


//MÁS ALGORITMO

// Va recorriendo todos los Streamed Areas

// Por cada uno, va preguntando por el resto, si están "pegados" (coinciden en X o Y  ¡CUIDADO CON LAS DIAGONALES!)

// Si están pegados, los conecta, desde la mitad de las 2 superficies que se unen

// Mete esa conexión lógica a una lista, para saber cuáles se han conectado ya, y no repetir A->B B->A
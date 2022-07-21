#include "Command.h"
#include "globals.h"

// The global command object contains user input from the GUI.
Command command;

Command::Command()
{
    debug = false;
    joystick = false;
    keyboard = false;
    ax = 0;
    ay = 0;

    showBody = true;
    showTargets = true;
    showGeometricModel = false;
    showSensedGrid = 1;
    showDijkstraMap = false;
    showTrajectoryTrace = false;
    showPlanTrace = false;
    showSimulationDebugDraw = false;
    showWorldVisibilityGraph = false;
    showLocalVisibilityGraph = false;
    showDropOffPoints = true;
    showWorldObstacles = false;
    showWorldMap = true;
    showExpandedWorldMap = true;
    showRayModel = false;
    showWorldPath = true;

    learn = false;
    predictionType = Unicycle; // Unicycle, Holonomic, None
    forceFieldReflex = true;
    stucknessReflex = true;
    useTimeAbort = true;
    useDynamicPath = true;
    useClosing = true;
    ghostMode = false;
    pathPlanningMethod = MinimalConstruct;
    trajectoryPlanningMethod = STAA;
    heuristic = MinimalConstruct;
    trajectoryType = Arc;
    frequency = 30;
}

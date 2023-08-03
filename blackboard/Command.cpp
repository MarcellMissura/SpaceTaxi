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
    showSensedPolygons = false;
    showSensedGrid = false;
    showDijkstraMap = false;
    showTrajectoryTrace = false;
    showSimulationDebugDraw = false;
    showWorldVisibilityGraph = false;
    showLocalVisibilityGraph = false;
    showDropOffPoints = true;
    showWorldPolygons = true;
    showWorldMap = true;
    showRayModel = false;
    showWorldPath = true;
    showLidar = 0;
    showPoseGraph = 1;
    showLineMap = 1;
    showVisibilityPolygon = false;
    showPolygonMap = 1;
    showPose = 1;

    learn = false;
    predictionType = Unicycle; // Unicycle, Holonomic, None
    emergencyBrakeReflex = true;
    stucknessReflex = false;
    useTimeAbort = true;
    useDynamicPath = true;
    useClosing = true;
    ghostMode = true;
    pathPlanningMethod = MinimalConstruct;
    trajectoryPlanningMethod = PD;
    heuristic = MinimalConstruct;
    trajectoryType = Arc;
    frequency = 10;

    globalLocalization = false;
    keepLineObservations = false;
    keepPoseHistory = false;
    mapUpdateEnabled = true;
    laserPointSmoothing = false;
}

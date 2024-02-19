#include "Command.h"

// The global command object contains user input from the GUI.
Command command;

Command::Command()
{
    joystick = false;
    keyboard = false;

    ax = 0;
    ay = 0;
    v = 0;
    w = 0;

    showBody = true;
    showTargets = true;
    showLocalMap = false;
    showCostmap = false;
    showDijkstraMap = false;
    showWorldVisibilityGraph = false;
    showLocalVisibilityGraph = false;
    showWorldMap = true;
    showPaths = true;
    showLaser = 0;
    showPoseGraph = 1;
    showLineMap = 0;
    showMotionPlan = false;
    showVisibilityPolygon = false;
    showPolygonMap = 1;
    showPose = 1;
    showLabels = false;
    showRuler = false;
    showOdometry = false;
    showSafetyZone = true;
    showSimulationDebugDraw = false;
    showWorldPolygons = true;
    showNavGoals = false;
    showRayModel = false;

    bufferToFile = false;
    laserSpatialFilter = false;
    laserSpeckleRemoval = false;
    slamEnabled = false;
    useOdometry = true;
    mapUpdateEnabled = true;
    keepLineObservations = false;
    selectPose = false;
    selectTarget = false;
    clearMap = false;
    fillMap = false;
    emergencyBrakeReflex = false;
    stucknessReflex = false;
    safetyZoneReflex = true;
    useTimeAbort = true;
    useDynamicPath = true;
    useClosing = true;
    draw = true;
    ghostMode = false;
    learn = false;

    heuristic = MinimalConstruct;
    pathPlanningMethod = MinimalConstruct;
    predictionType = Unicycle; // Unicycle, Holonomic, None
    trajectoryPlanningMethod = STAA;
    trajectoryType = Arc;
    frequency = 10;
}

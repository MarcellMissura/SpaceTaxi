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
    showSimulationDebugDraw = false;
    showWorldVisibilityGraph = false;
    showLocalVisibilityGraph = false;
    showNavGoals = false;
    showWorldPolygons = true;
    showWorldMap = true;
    showRayModel = false;
    showPaths = true;
    showLaser = 0;
    showPoseGraph = 1;
    showLineMap = 1;
    showMotionPlan = false;
    showVisibilityPolygon = false;
    showPolygonMap = 1;
    showPose = 1;
    showLabels = false;
    showRuler = false;
    showOdometry = false;
    showSafetyZone = true;

    bufferToFile = false;
    laserSpatialFilter = false;
    laserTemporalFilter = false;
    laserSpeckleRemoval = false;
    slamEnabled = false;
    useOdometry = true;
    mapUpdateEnabled = true;
    globalLocalization = false;
    keepLineObservations = false;
    keepPoseHistory = false;
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
    ghostMode = false;
    draw = true;
    learn = false;

    heuristic = MinimalConstruct;
    pathPlanningMethod = MinimalConstruct;
    predictionType = Unicycle; // Unicycle, Holonomic, None
    trajectoryPlanningMethod = STAA;
    trajectoryType = Arc;
    frequency = 10;
}

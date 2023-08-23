#include "Command.h"

// The global command object contains user input from the GUI.
Command command;

Command::Command()
{
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

    laserSpatialFilter = false;
    laserTemporalFilter = false;
    laserSpeckleRemoval = true;
    slamEnabled = false;
    useOdomAsPrior = true;
    mapUpdateEnabled = true;
    globalLocalization = false;
    keepLineObservations = false;
    keepPoseHistory = false;
    selectPose = false;
    selectTarget = false;
    emergencyBrakeReflex = false;
    stucknessReflex = false;
    useTimeAbort = true;
    useDynamicPath = true;
    useClosing = true;
    ghostMode = true;
    draw = true;
    learn = false;

    heuristic = MinimalConstruct;
    pathPlanningMethod = MinimalConstruct;
    predictionType = Unicycle; // Unicycle, Holonomic, None
    trajectoryPlanningMethod = STAA;
    trajectoryType = Arc;
    frequency = 10;
}

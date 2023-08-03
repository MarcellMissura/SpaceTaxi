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

    slamEnabled = false;
    mapUpdateEnabled = true;
    selectPose = false;
    selectTarget = false;
    globalLocalization = false;
    keepLineObservations = false;
    keepPoseHistory = false;
    laserSpatialFilter = false;
    laserTemporalFilter = false;
    laserParticleRemoval = true;
    useOdomAsPrior = true;
    draw = true;
    learn = false;
    emergencyBrakeReflex = false;
    stucknessReflex = false;
    useTimeAbort = true;
    useDynamicPath = true;
    useClosing = true;
    ghostMode = true;

    heuristic = MinimalConstruct;
    pathPlanningMethod = MinimalConstruct;
    predictionType = Unicycle; // Unicycle, Holonomic, None
    trajectoryPlanningMethod = STAA;
    trajectoryType = Arc;
    frequency = 10;
}

#ifndef COMMAND_H_
#define COMMAND_H_

// The global command object is a blackboard just like the state object.
// It contains variables that are set by key presses by the user and
// control the robot or components of the gui.
struct Command
{
    bool joystick;
    bool keyboard;

    double ax; // Commanded linear acceleration.
    double ay; // Commanded angular acceleration.
    double v; // Commanded linear velocity.
    double w; // Commanded angular velocity.

    bool showBody;
    bool showTargets;
    bool showLocalMap;
    bool showCostmap;
    bool showDijkstraMap;
    bool showWorldVisibilityGraph;
    bool showLocalVisibilityGraph;
    bool showWorldMap;
    bool showPaths;
    int showLaser;
    int showLineMap;
    bool showMotionPlan;
    bool showVisibilityPolygon;
    int showPoseGraph;
    int showPose;
    int showPolygonMap;
    bool showLabels;
    bool showRuler;
    bool showOdometry;
    bool showSafetyZone;
    bool showNavGoals;
    bool showRayModel;
    bool showSimulationDebugDraw;
    bool showWorldPolygons; // the simulated polygons
    

    bool bufferToFile;
    bool slamEnabled;
    bool mapUpdateEnabled;
    bool useOdometry;
    bool keepLineObservations;
    bool laserSpatialFilter;
    bool laserSpeckleRemoval;
    bool emergencyBrakeReflex;
    bool stucknessReflex;
    bool safetyZoneReflex;
    bool useTimeAbort;
    bool useDynamicPath;
    bool useClosing;
    bool selectPose;
    bool selectTarget;
    bool clearMap;
    bool fillMap;
    bool draw;
    bool learn;
    bool ghostMode;

    int predictionType; // Unicycle, Holonomic, None
    int pathPlanningMethod; // None, A*, LazyT*, Full, Naive, Minimal, Dijkstra
    int trajectoryPlanningMethod; // PD, Arc, Bezier, DWA, STAA, RuleBase
    int heuristic; // mc, graph dijkstra, grid dijkstra, rtr, rtr max, rtr min,
    int trajectoryType; // quadratic, cubic, Arc, B0, Fresnel
    int frequency; // 5Hz, 10Hz, 20Hz, 30Hz

    enum { NoPath, GridAStar, LazyThetaStar, FullConstruct, NaiveConstruct, MinimalConstruct,
           Dijkstra, PD, DWA, Reel, RuleBase, STAA, SpeedControl, Keyboard, Arc, Bezier, B0, Fresnel,
           Euklidean, PathEuklidean, RTR, DOCK_RTR, RTR_MAX, RTR_MIN, PathRTR, GraphDijkstra, GridDijkstra,
           Unicycle, Holonomic, None,
           Static, Dynamic };
    Command();
};

extern Command command;

#endif /* COMMAND_H_ */

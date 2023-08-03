#ifndef COMMAND_H_
#define COMMAND_H_

// The global command object contains user input from the GUI.
struct Command
{
    bool joystick;
    bool keyboard;

    double ax;
    double ay;

    bool showBody;
    bool showTargets;
    bool showSensedPolygons;
    bool showSensedGrid;
    bool showDijkstraMap;
    bool showRayModel;
    bool showTrajectoryTrace;
    bool showSimulationDebugDraw;
    bool showWorldVisibilityGraph;
    bool showLocalVisibilityGraph;
    bool showDropOffPoints;
    bool showWorldPolygons;
    bool showWorldMap;
    bool showWorldPath;
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

    bool slamEnabled;
    bool mapUpdateEnabled;
    bool globalLocalization;
    bool keepLineObservations;
    bool keepPoseHistory;
    bool laserSpatialFilter;
    bool laserTemporalFilter;
    bool laserParticleRemoval;
    bool useOdomAsPrior;
    bool emergencyBrakeReflex;
    bool stucknessReflex;
    bool useTimeAbort;
    bool useDynamicPath;
    bool useClosing;
    bool selectPose;
    bool selectTarget;
    bool learn;
    bool draw;
    bool ghostMode;

    int predictionType; // Unicycle, Holonomic, None
    int pathPlanningMethod; // None, A*, LazyT*, Full, Naive, Minimal, Dijkstra
    int trajectoryPlanningMethod; // PD, DWA, Deep DWA, Aborting A*
    int heuristic; // mc, graph dijkstra, grid dijkstra, rtr, rtr max, rtr min,
    int trajectoryType; // quadratic, cubic, Arc, B0, Fresnel
    int frequency; // 10Hz, 20Hz, 30Hz

    enum { NoPath, GridAStar, LazyThetaStar, FullConstruct, NaiveConstruct, MinimalConstruct,
           Dijkstra, PD, DWA, RuleBase, STAA, Keyboard, Arc, B0, Fresnel,
           Euklidean, PathEuklidean, RTR, RTR_MAX, RTR_MIN, PathRTR, GraphDijkstra, GridDijkstra,
           Unicycle, Holonomic, None,
           Static, Dynamic };
	Command();
};

extern Command command;

#endif /* COMMAND_H_ */

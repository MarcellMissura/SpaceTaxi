#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <QString>
#include <QHash>

struct Config
{
    // framework
    double bufferSize;
    double debugLevel;

    // simulation
    double simSpeedUp;
    double simLinearDamping; // General linear damping applied in the phys sim at all times.
    double simAngularDamping; // General angular damping applied in the phys sim at all times.
    double simFriction; // Friction applies only when bodies touch.
    double simLaserRays; // How many simulated sensor rays.
    double simLaserAngleRange; // The opening angle of the simulated sensor field.
    double simLaserLength; // The maximum length of the simulated laser rays.

    // world and maps
    double navGoalRadius;
    double unicycleAgents;
    double voidRadius;
    double voidDropOffPoints;
    double simpleRadius;
    double simpleDropOffPoints;
    double uRadius;
    double uDropOffPoints;
    double tunnelCorridorWidth;
    double clutterWidth;
    double clutterHeight;
    double clutterObstacles;
    double clutterObstaclesSizeMin;
    double clutterObstaclesSizeMax;
    double clutterDropOffPoints;
    double officeRoomSize; // width and height of one room in meters
    double officeWallThickness; // in percent of one room unit
    double officeDoorWidth; // in percent of one room unit
    double officeRooms; // number of rooms per building
    double wareHouseAisleWidth;

    // agent
    double agentRadius;
    double agentHeight;
    double agentWidth;
    double agentLinearAccelerationLimit;
    double agentLinearVelocityLimitForward;
    double agentLinearVelocityLimitBackward;
    double agentAngularAccelerationLimit;
    double agentAngularVelocityLimit;
    double agentTargetReachedDistance;

    // laser sensor processing
    double laserMaxRange; // Points beyond this distance are ignored.
    double laserSmoothingSpatialPasses; // Number of low pass passes.
    double laserSmoothingMinSegmentSize; // Clusters this small will be removed.
    double laserSegmentDistanceThreshold; // If neighbouring points are further apart than this, they don't belong to the same segment.
    double laserDouglasPeuckerEpsilon; // The DP epsilon for the vis polygon and line extraction.
    double laserPolygonThickness; // The assumed thickness of a polygon when detected by the sensor.
    double laserLineMinLength; // Minimum length of extracted lines.
    double laserLineMaxDistance; // Maximum distance up to which lines are detected.
    double laserLineMinAngle; // Minimum angle of a line with respect to the viewing direction.
    double laserTriangleLegLength; // The diameter of the triangle marker.
    double laserTriangleAngle; // The angle at the tip of the triangle marker.
    double laserTriangleSizeTolerance; // Allowed triangle size deviation in percent.
    double laserTriangleSmoothing; // Low passs filter parameter for triangle smoothing.


    // ray model
    double raysNumber;
    double raysAngleRange;
    double raysLength;

    // geometric model
    double gmPolygonDilation; // By how much are static polygons expanded.
    double gmAgentDilation; // By how much are the dynamic polygons expanded.
    double gmDouglasPeuckerEpsilon; // Douglas Peucker epsilon applied in the context of geometric operations.
    double gmPolygonPruning; // How strong is the pruning of all polygons.

    // grid model
    double gridWidth; // Grid model area definition.
    double gridHeight; // Grid model area definition.
    double gridOffset; // Grid model offset in x direction.
    double gridCellSize; // The size of one cell in the grid model, e.g. 0.05 m.
    double gridClosedCellSize; // The size of one cell in the closed grid (STAA).
    double gridBlurRadius; // The size of the costmap.
    double gridSensedDilationRadius; // By how much is the sensed grid dilated.

    // Path planning
    double predictFactor; // How strongly does the prediction time increase with distance.
    double predictMaxPredictTime; // How much time to predict at most.
    double predictIgnoreHorizon; // From what ETA are obstacles ignored.

    // Emergency break reflex.
    double ebPredictionTime;

    // Bezier controller.
    double bezierDt;
    double bezierStretch;

    // Reel Controller
    double reelCarrotOffset;

    // RuleBase controller
    double rbRayDistanceThreshold;
    double rbTargetDistanceThreshold;
    double rbCarrotDistanceThreshold;

    // ST Aborting A* search
    double staaNotches; // Samples per dimension in the action set.
    double staaExpansionLimit; // Abort after this many expansions.
    double staaDt; // How much time passes between expansions.
    double staaFinishedThreshold; // The goal condition.
    double staaGridCostWeight; // Weighting factor for the grid value.
    double staaProximityCostWeight; // Weighting factor for the proximity to dynamic obstacles.
    double staaPolygonGrowth; // The agent polygon is grown a little bit.
    double staaStucknessWeight; // Tunes the reaction to a stuck state.
    double staaInflationFactor; // The heuristic inflation gives more weight to the target.

    // Dynamic Window Approach
    double DWA_carrotOffset; // How far to set the carrot along the path.
    double DWA_notches;
    double DWA_time;
    double DWA_samples;
    double DWA_gridClearance;
    double DWA_geometricClearance;
    double DWA_carrotDistance; // Weight of the distance to the carrot in the fitness function.
    double DWA_backwardsPenalty;

    // Unicycle PD controller.
    double UPD_carrotOffset; // How far to set the carrot along the path.
    double UPD_Plin;
    double UPD_Dlin;
    double UPD_Prot;
    double UPD_Drot;

    // Line slam.
    double slamMinObservationCount; // How many times must a map line have been seen before it can participate in pairing.
    double slamMergeMaxLineDist; // The line-line-dist must be less than this when merging two lines.
    double slamMergeMinOverlap; // The overlap must be at least this much when merging two lines.
    double slamMaxPoseDiff; // Maximum norm of a pose diff the local snap is allowed to output.
    double slamSeenCornerMinAngle; // There has to be at least this much angle between neighbouring lines for a corner to be detected.
    double slamSelectionBoxSize; // The radius of the selection box for the medium snap.
    double slamPairingMaxLengthDeviation; // An input line cannot be longer than an associated map line by more than this.
    double slamPairingMaxPoseDist; // Pairs cannot have a line-pose-distance larger than this between them when building nearest pairs.
    double slamClusteringAngleEps; // How much are two angles allowed to differ before they are considered the same.
    double slamClusteringOrthoEps; // Epsilon for computing ortho clusters when computing a consensus set.
    double slamClusteringTransformEps; // The epsilon parameter for the DBScan clustering of hypotheses.
    double slamHypMinOverlapPercent; // When computing a hypothesis, two pairs must overlap at least by this much for the hypothesis to be valid.
    double slamPoseGraphNodeDist; // How much pose distance between the last node and a newly created node.
    double slamPoseGraphNeighborhoodSize; // How many nodes starting from the one nearest to the agent are considered to be nearby.
    double slamVisibilityPolygonBound; // To what distance is the reduced visibility polygon bounded.
    double slamSnapAcceptanceThreshold; // How much quality (0 - 1) must a snap have in order to be accepted?


    Config();
    ~Config(){}

    void init();
    void save(QString robotName);
    void load(QString robotName);

    double& operator[](int i);
    double& operator()(int i);
    double& operator[](QString key);
    double& operator()(QString key);

private:

    double sink;

    // Registers a member variable for index based access.
    void registerMember(QString name, double* member, double sliderRange)
    {
        memberNames << name;
        memberOffsets[name] = (quint64)member - (quint64)this;
        sliderRanges[name] = sliderRange;
    }

    QHash<QString, unsigned long int> memberOffsets;

public:
    QList<QString> memberNames; // Contains the names of the members in the right order.
    QHash<QString, double> sliderRanges; // The ranges of all explicitly registered config variables.
};

extern Config config;

#endif /* CONFIGURATION_H_ */

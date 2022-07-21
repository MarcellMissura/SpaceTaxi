#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <QString>
#include <QHash>

struct Config
{
    double rcIterationTime;
    double speedUp;
    double bufferSize;
    double debugLevel;

    // agent
    double agentHeight;
    double agentWidth;
    double agentLinearAccelerationLimit;
    double agentLinearVelocityLimitForward;
    double agentLinearVelocityLimitBackward;
    double agentLinearJerkLimit;
    double agentLinearDamping; // General linear damping applied in the phys sim at all times.
    double agentAngularAccelerationLimit;
    double agentAngularVelocityLimit;
    double agentAngularDamping; // General angular damping applied in the phys sim at all times.
    double agentFriction; // Friction applies only when bodies touch.

    // ray model
    double raysNumber;
    double raysAngleRange;
    double raysLength;

    // geometric model
    double gmPolygonExpansionMargin; // By how much are the dynamic polygons expanded.

    // grid model
    double gridDouglasPeuckerEpsilon; // Polygon contour smoothing strength.
    double gridMinimumSegmentSize; // Ignore contours with pieces less than this.
    double gridBlurRadius;
    double gridWorldDilationRadius;
    double gridSensedDilationRadius;
    double gridWidth; // Grid model area definition.
    double gridHeight; // Grid model area definition.
    double gridOffset; // Grid model offset in x direction.
    double gridCellSize; // The size of one cell in the grid model, e.g. 0.05 m.
    double gridClosedCellSize; // The size of one cell in the closed grid.

    // world
    double worldDropOffRadius;
    double unicycleAgents;

    // maps
    double voidRadius;
    double voidDropOffPoints;
    double simpleRadius;
    double simpleDropOffPoints;
    double uRadius;
    double uDropOffPoints;

    double outdoorWidth;
    double outdoorHeight;
    double outdoorObstaclesNr;
    double outdoorObstaclesMinWidth;
    double outdoorObstaclesMaxWidth;
    double outdoorObstaclesMinHeight;
    double outdoorObstaclesMaxHeight;
    double outdoorCarsNr;

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

    // RuleBase controller
    double rbRayDistanceThreshold;
    double rbTargetDistanceThreshold;
    double rbCarrotDistanceThreshold;

    // ST Aborting A* search
    double staaNotches; // Samples per dimension in the action set.
    double staaExpansionLimit; // Abort after this many expansions.
    double staaTimeLimit; // Abort condition in Milliseconds.
    double staaDt; // How much time passes between expansions.
    double staaFinishedThreshold; // The goal condition.
    double staaGridCostWeight; // Weighting factor for the grid value.
    double staaProximityCostWeight; // Weighting factor for the proximity to dynamic obstacles.
    double staaPolygonGrowth; // The agent polygon is grown a little bit.
    double staaCollisionGridDilation;
    double staaSidePreference; // -1 to 1 preference for the left or the right side.
    double staaPredictionTimeLimit; // How far into the future are we allowed to predict.
    double staaStucknessWeight; // Tunes the reaction to a stuck state.

    // Dynamic Window Approach
    double DWA_carrotOffset; // How far to set the carrot along the path.
    double DWA_notches;
    double DWA_time;
    double DWA_depth;
    double DWA_samples;
    double DWA_fraction;
    double DWA_gridClearance;
    double DWA_geometricClearance;
    double DWA_targetDistance;
    double DWA_targetAngle;
    double DWA_backwardsPenalty;

    // Unicycle PD controller.
    double UPD_carrotOffset; // How far to set the carrot along the path.
    double UPD_Kp_lin;
    double UPD_Kd_lin;
    double UPD_Kp_rot;
    double UPD_Kd_rot;

    // Gui config
    double colorContrast;
    double transparency;
    double scale;
    double sampleFactor; // grid density and such


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
    QHash<QString, double> sliderRanges; // The ranges of all explicitely registered config variables.
};

extern Config config;

#endif /* CONFIGURATION_H_ */

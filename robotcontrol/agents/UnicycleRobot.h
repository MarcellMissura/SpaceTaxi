#ifndef ROBOT_H
#define ROBOT_H

#include "UnicycleObstacle.h"
#include "lib/geometry/GeometricModel.h"
#include "lib/geometry/GridModel.h"
#include "robotcontrol/perception/LaserSensor.h"
#include "robotcontrol/controller/ShortTermAbortingAStar.h"
#include "robotcontrol/controller/UnicycleDWA.h"
#include "robotcontrol/controller/RuleBase.h"
#include "robotcontrol/slam/GeometricMap.h"

class UnicycleRobot : public UnicycleObstacle
{
public:

    StopWatch stopWatch;

    bool inited;

    // Input
    LaserSensor laserSensor;
    Pose2D odomPose;

    // Pose monitoring
    Pose2D initialPose;
    Pose2D initialOdomPose; // These are global.
    Vector<Pose2D> odomHistory;
    Vector<Pose2D> poseHistory;

    // World Map
    GeometricModel worldMap; // The assumed map the robot has built for itself.
    GeometricModel clippedWorldMap; // The world map clipped to the local map.
    GeometricMap geometricMap; // The built map.

    // Local map sensing. Grid, geometric, and ray models.
    Polygon visibilityPolygon; // The awesome visibility polygon.
    GridModel occupanyGrid; // First line grid sensing from the laser points.
    GridModel sensedGrid; // Local occupancy grid and costmap computed from the sensor input.
    GeometricModel localMap;
    Vector<double> rays;
    Vector<Vec2> rayModel;

    // Target stuff.
    bool atTarget;
    Vector<uint> dropOffPointQueue;
    uint targetDropOffId; // The id of the target drop off point.
    Pose2D mainTarget; // The world coordinates of the main target.
    Pose2D intermediateTarget; // The local coordinates of the intermediate target at the boundary of the sensed grid.
    Pose2D carrot; // Nearby target in local coordinates that steers dumb controllers.
    Vec2 joystickCarrot;

    // Path stuffs.
    Vector<Vec2> worldPath;
    Vector<Vec2> dynamicPath;
    Vector<Vec2> staticPath;
    bool worldPathSuccess;
    bool dynamicPathSuccess;
    bool staticPathSuccess;
    bool trajectorySuccess;

    // Reflex controller variables.
    int stuckDetectionCounter;
    int stuckTimer;
    bool isStuck;
    Vec2 stucknessImpulse;
    bool ebActive;
    double ebA;
    Vec2 impactPoint;
    bool inCollision;
    bool collided;

    // Motion controllers.
    UnicycleDWA unicycleDWA;
    ShortTermAbortingAStar shortTermAbortingAStar;
    RuleBase ruleBase;

    // Statistics.
    uint score;
    uint collisions;
    uint stucks;
    uint closes;
    double milage;
    double pathTime;
    double trajectoryTime;

public:

    UnicycleRobot();
    ~UnicycleRobot(){}

    void init();
    void reset();

    void setInput(const LaserSensor& laserInput, const Pose2D& odomInput);
    void setMainTarget(const Pose2D& pose);
    void setInitialPose(const Pose2D& p);
    Vec2 getTxVel(); // the output

    void step();
    void sense();
    void act();
    void learn();

    void saveMap() const;
    void loadMap();

    void draw();
    void draw(QPainter* painter) const;

private:
    void computeRayModel();
    Vec2 pdControlTo(const Pose2D& p) const;
    Vec2 forceReaction(const Vec2 &q) const;

    void drawWorldMap() const;
    void drawWorldPaths() const;
    void drawMainTarget() const;
    void drawLaserPoints() const;
    void drawBody() const;
    void drawOdometry() const;
    void drawPose() const;
    void drawGridModel() const;
    void drawGeometricModel() const;
    void drawRayModel() const;
    void drawMotionPlan() const;
    void drawVisibilityPolygon() const;
    void drawTargets() const;
    void drawVisibilityGraph() const;
};

QDebug operator<<(QDebug dbg, const UnicycleRobot &o);

#endif // ROBOT_H

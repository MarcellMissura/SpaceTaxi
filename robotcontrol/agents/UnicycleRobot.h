#ifndef ROBOT_H
#define ROBOT_H

#include "UnicycleObstacle.h"
#include "lib/geometry/GeometricModel.h"
#include "lib/geometry/GridModel.h"
#include "lib/geometry/Path.h"
#include "robotcontrol/perception/LaserSensor.h"
#include "robotcontrol/perception/OdomSensor.h"
#include "robotcontrol/controller/ShortTermAbortingAStar.h"
#include "robotcontrol/controller/UnicycleDWA.h"
#include "robotcontrol/slam/GeometricMap.h"

class UnicycleRobot : public UnicycleObstacle
{
public:

    StopWatch stopWatch;

    // Input
    LaserSensor laserSensor;
    Pose2D odomPose;

    // Pose monitoring
    Pose2D initialPose;
    Vector<Pose2D> poseHistory;
    OdomSensor odomSensor;

    // World Map
    GeometricMap worldMap; // The built map.
    GeometricModel clippedWorldMap; // The world map clipped to the local map.

    // Local map sensing. Grid, geometric, and ray models.
    Polygon visibilityPolygon; // The awesome visibility polygon.
    GridModel occupanyGrid; // First line grid sensing from the laser points.
    GridModel costmap; // Local occupancy grid and costmap computed from the sensor input.
    GeometricModel localMap;

    // Navigation handles.
    bool atTarget;
    Vector<uint> navGoalQueue;
    uint targetNavGoalId; // The id of the target drop off point.
    Pose2D mainTarget; // The world coordinates of the main target.
    Pose2D intermediateTarget; // The local coordinates of the intermediate target at the boundary of the sensed grid.
    Pose2D carrot; // Nearby target in local coordinates that steers dumb controllers.
    Vec2 joystickCarrot;

    // Path stuffs.
    Path worldPath;
    Path dynamicPath;
    Path staticPath;
    bool worldPathSuccess;
    bool dynamicPathSuccess;
    bool staticPathSuccess;
    bool trajectorySuccess;

    // Motion controllers.
    UnicycleDWA unicycleDWA;
    ShortTermAbortingAStar shortTermAbortingAStar;

    // Statistics.
    uint score;
    uint collisions;
    uint stucks;
    double milage;
    double pathTime;
    double trajectoryTime;

public:

    UnicycleRobot();
    ~UnicycleRobot(){}

    void init();
    void reset();

    void setInput(const LaserSensor& laserInput, const OdomSensor& odomSensor);
    void setMainTarget(const Pose2D& pose);
    void setInitialPose(const Pose2D& p);
    Vec2 getTxVel(); // the output

    void step();
    void sense();
    void act();

    void clearMap(const Polygon& pol);
    void fillMap(const Polygon& pol);
    void saveMap() const;
    void loadMap();

    void draw();
    void draw(QPainter* painter) const;

private:
    Vec2 pdControlTo(const Vec2& p) const;

    void drawWorldMap() const;
    void drawWorldPaths() const;
    void drawMainTarget() const;
    void drawLaserPoints() const;
    void drawBody() const;
    void drawOdometry() const;
    void drawPose() const;
    void drawGridModel() const;
    void drawGeometricModel() const;
    void drawMotionPlan() const;
    void drawVisibilityPolygon() const;
    void drawTargets() const;
    void drawVisibilityGraph() const;
};

QDebug operator<<(QDebug dbg, const UnicycleRobot &o);

#endif // ROBOT_H

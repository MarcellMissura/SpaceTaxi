#ifndef ROBOT_H
#define ROBOT_H

#include "lib/util/Vec2.h"
#include "lib/util/Pose2D.h"
#include "lib/util/StopWatch.h"
#include "perception/RGBDSensor.h"
#include "perception/LaserSensor.h"
#include "lib/geometry/GeometricModel.h"
#include "lib/geometry/DynamicGeometricModel.h"
#include "lib/geometry/GridModel.h"
#include "lib/geometry/VisibilityGraph.h"
#include "agents/UnicycleObstacle.h"
#include "controller/UnicycleDWA.h"
#include "controller/ShortTermAbortingAStar.h"
#include "slam/GeometricMap.h"

class UnicycleRobot : public UnicycleObstacle
{
public:

    // This is a flag that indicates if a new sensor update was received
    bool rgbdUpdated, laserUpdated;

    // Velocity monitoring and control
    Vec2 rxVel;
    Vec2 txVel;

private:

    StopWatch stopWatch;

    // Sensor stuff
    static RGBDSensor rgbdSensor;
    LaserSensor laserSensor;

    // Pose monitoring
    Pose2D initialPose, initialOdomPose, externalLocalizerPose; // These are global.
    Pose2D odomPose, prevOdomPose; // Also global.
    Pose2D odomDiff; // local
    Vector<Pose2D> odomHistory;
    Vector<Pose2D> poseHistory;

    bool atTarget;
    Pose2D mainTarget; // The world coordinates of the main target.
    Pose2D intermediateTarget; // The local coordinates of the intermediate target at the boundary of the sensed grid.
    Pose2D carrot; // Nearby target in local coordinates that steers dumb controllers.

    Vector<Vec2> staticWorldPath;
    Vector<Vec2> dynamicPath;
    bool staticWorldPathSuccess;
    bool dynamicPathSuccess;
    bool trajectorySuccess;

    // Grid and geometric models.
    GridModel mapAndSensedGrid;
    GridModel dilatedMapAndSensedGrid;
    GridModel sensedGrid;
    GridModel mapGrid;
    GeometricModel mapAndSensedPolygons;
    GeometricModel expandedMapAndSensedPolygons;
    DynamicGeometricModel dynamicPolygons;
    DynamicGeometricModel expandedDynamicPolygons;
    Vector<Vec2> rayModel;

    // Path and motion controllers.
    UnicycleDWA unicycleDWA;
    ShortTermAbortingAStar shortTermAbortingAStar;

    static QMutex odomPoseMutex;

public:

    // These need to be public for drawing labels.
    static GeometricModel unifiedGeometricModel;
    static GeometricMap geometricMap;
    Polygon visibilityPolygon;

    UnicycleRobot();
    ~UnicycleRobot() {}

    void init();
    void reset();

    void step();
    void bufferStep();
    void sense();
    void act();

    void manualMapUpdate();
    void saveMap() const;
    void loadMap();

    CameraInfo getCameraInfo() const;
    void setCameraInfo(const CameraInfo& info);
    void setCameraTransform(const Transform3D& T);
    Transform3D getCameraTransform() const;
    const Vector<RGBPixel>& readColorBuffer() const;
    void writeColorBuffer(const Vector<RGBPixel>& colors);
    const Vector<Vec3>& readPointBuffer() const;
    void writePointBuffer(const Vector<Vec3>& points);
    Vector<Vec2> readLaserBuffer() const;
    void writeLaserBuffer(const Vector<Vec2> &points);
    const LaserInfo& getLaserInfo() const;
    void setLaserInfo(const LaserInfo& info);
    void setLaserToBasePose(const Pose2D& p);

    void setTarget(const Pose2D& pose);

    const Vec2 &getTxVel();
    void setRxVel(double v, double w);

    void setInitialPose(const Pose2D& p);
    void setOdomPose(const Pose2D &p);
    Pose2D getOdomPose() const;
    void setExternalLocalizerPose(const Pose2D& p);

    bool isPointInLocalGrid(const Vec2& p) const;
    Vec2u getNodeIndex(const Vec2& p) const;
    double getValueAt(const Vec2& p) const;

    void draw();
    void draw(QPainter* painter) const;

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream &in);

private:

    void computeRayModel();
    Vec2 pdControlTo(const Pose2D& p) const;

    void drawWorldPaths() const;
    void drawMainTarget() const;
    void drawPointCloud() const;
    void drawCameraPlane() const;
    void drawLaserPoints() const;
    void drawBody() const;
    void drawOdometry() const;
    void drawPose() const;
    void drawGridModel() const;
    void drawGeometricModel() const;
    void drawRayModel() const;
    void drawWorldMap();
    void drawVisibilityPolygon() const;
    void drawTargets() const;
    void drawMotionPlan() const;
    void drawVisibilityGraph() const;
};

QDataStream& operator<<(QDataStream& out, const UnicycleRobot &o);
QDataStream& operator>>(QDataStream& in, UnicycleRobot &o);
QDebug operator<<(QDebug dbg, const UnicycleRobot &o);


#endif // ROBOT_H

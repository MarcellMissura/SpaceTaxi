#ifndef LASERSENSOR_H_
#define LASERSENSOR_H_

#include "lib/util/Vector.h"
#include "lib/util/Vec2.h"
#include "lib/util/Pose2D.h"
#include "lib/geometry/Line.h"
#include "lib/geometry/Polygon.h"
#include "lib/util/RingBuffer.h"
#include "robotcontrol/slam/TrackedLine.h"
#include <QPainter>
#include <QMutex>

struct LaserInfo
{
    uint rays; // Number of laser rays.
    double angleMin; // Start angle of the scan (in radians)
    double angleMax; // End angle of the scan (in radians)
    double angleIncrement; // Angular distance between measurements
    double rangeMin; // Minimum range value
    double rangeMax; // Maximum range value
    double timeIncrement; // time between measurements [seconds] - if your scanner
                           // is moving, this will be used in interpolating position
                           // of 3d points. typically scanTime / laserPoints
    double scanTime; // time between scans [seconds]

    LaserInfo()
    {
        rays = 540;
        angleIncrement = 0.00872665;
        angleMax = 2.35619;
        angleMin = -2.35619;
        rangeMax = 29.5;
        rangeMin = 0.01;

        // T-bot
        // rays: 540
        //angle min, max, increment: -2.35619 2.35619 0.00872665
        //range min, max: 0.001 29.5
        //time increment, scan time: 4.62107e-05 0
    }
};

class LaserSensor
{
    uint frameId;
    LaserInfo laserInfo;

    // Pose of the laser scanner in the robot's base frame.
    Pose2D laserToBasePose;

    // Allocated buffer for the point cloud.
    Vector<Vec2> pointBuffer;

    // Mutex for thread safety.
    mutable QMutex mutex;

public:

    LaserSensor();
    ~LaserSensor();

    LaserSensor(const LaserSensor &o);
    LaserSensor& operator=(const LaserSensor &o);

    const LaserInfo& getLaserInfo() const;
    void setLaserInfo(const LaserInfo& info);

    void setLaserToBasePose(const Pose2D& p);
    const Pose2D& getLaserToBasePose() const;

    void writePointBuffer(const Vector<Vec2> &pointBuffer);
    void readPointBuffer(Vector<Vec2>& pointBuffer) const;
    Vector<Vec2> readPointBuffer() const;

    // Feature extraction.
    Vector<Vec2> getNormals() const;
    const Vector<TrackedLine> &extractLines(bool debug=false) const;
    const Polygon &getVisibilityPolygon() const;
    Polygon extractTriangleMarker(int debug=0) const;

    // Smoothing filters.
    void filter();
    void particleRemoval();
    void spatialFilter();

    // Drawing functions.
    void draw(QPainter* painter) const;
    void draw() const;
    void printInfo() const;

    // Serialization.
    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream &in);
};

QDataStream& operator<<(QDataStream& out, const LaserSensor &o);
QDataStream& operator>>(QDataStream& in, LaserSensor &o);
QDebug operator<<(QDebug dbg, const LaserSensor &o);

#endif

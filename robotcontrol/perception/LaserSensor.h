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
        rays = 0;
        angleIncrement = 0.0;
        angleMax = 0.0;
        angleMin = 0.0;
        rangeMax = 0.0;
        rangeMin = 0.0;
        timeIncrement = 0.0;
        scanTime = 0.0;

        // T-bot
        // rays: 540
        //angle min, max, increment: -2.35619 2.35619 0.00872665
        //range min, max: 0.001 29.5
        //time increment, scan time: 4.62107e-05 0
    }

    // Serialization.
    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream &in);
};

class LaserSensor
{
    uint frameId;

    // LaserInfo struct that describes the properties of the sensor.
    LaserInfo laserInfo;

    // Pose of the laser scanner in the robot's base frame.
    Pose2D laserToBasePose;

    // Range buffer and unit vectors.
    Vector<double> rangeBuffer;
    Vector<Vec2> unitVectors;

    // Allocated point buffer for the point cloud.
    Vector<Vec2> pointBuffer;

    Vector<Vector<uint>> pointSegments;
    Vector<Vector<uint>> lineSegments;
    Vector<uint> segment;

    // Mutex for thread safety.
    mutable QMutex mutex;

    // For visualization.
    mutable Vector<Polygon> triangles;

public:

    LaserSensor();
    ~LaserSensor();

    LaserSensor(const LaserSensor &o);
    LaserSensor& operator=(const LaserSensor &o);

    const LaserInfo& getLaserInfo() const;
    void setLaserInfo(const LaserInfo& info);

    void setLaserToBasePose(const Pose2D& p);
    const Pose2D& getLaserToBasePose() const;

    void writeRangeBuffer(const Vector<double> &rangeBuffer);
    void readRangeBuffer(Vector<double>& rangeBuffer) const;
    Vector<double> readRangeBuffer() const;

    void writePointBuffer(const Vector<Vec2> &pointBuffer);
    void readPointBuffer(Vector<Vec2>& pointBuffer) const;
    Vector<Vec2> readPointBuffer() const;

    // Feature extraction.
    Vector<Vec2> extractNormals() const;
    const Vector<TrackedLine> &extractLines(bool debug=false) const;
    const Polygon &extractVisibilityPolygon() const;
    const Vector<Polygon> &extractSensedPolygons() const;
    Polygon extractTriangle(int debug=0) const;

    // Drawing functions.
    void draw(QPainter* painter) const;
    void draw() const;

    // Serialization.
    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream &in);

private:

    // Smoothing filters.
    void speckleRemoval();
    void spatialFilter();

    // Segmentation.
    void segmentation();
    void reduction();
    void douglasPeuckerSub(uint fromIdx, uint toIdx);

    bool isLineOkay(const Line &l, bool debug=false) const;
};

QDataStream& operator<<(QDataStream& out, const LaserSensor &o);
QDataStream& operator>>(QDataStream& in, LaserSensor &o);
QDebug operator<<(QDebug dbg, const LaserSensor &o);
QDataStream& operator<<(QDataStream& out, const LaserInfo &o);
QDataStream& operator>>(QDataStream& in, LaserInfo &o);
QDebug operator<<(QDebug dbg, const LaserInfo &o);

#endif

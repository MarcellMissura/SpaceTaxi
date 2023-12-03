#ifndef ODOMSENSOR_H_
#define ODOMSENSOR_H_

#include "lib/util/Vector.h"
#include "lib/util/Pose2D.h"
#include <QPainter>
#include <QMutex>

class OdomSensor
{
public:
    uint frameId;

    bool inited;
    Vector<Pose2D> odomHistory;
    Pose2D initialOdomPose;
    Pose2D lastOdomPose;
    Pose2D odomPose;

    // Mutex for thread safety.
    mutable QMutex mutex;

public:

    OdomSensor();
    ~OdomSensor();

    OdomSensor(const OdomSensor &o);
    OdomSensor& operator=(const OdomSensor &o);

    void writeOdomPose(const Pose2D &odomPose, bool saveHistory=true);
    Pose2D readOdomPose() const;
    Pose2D readOdomIncrement();

    // Drawing functions.
    void draw(QPainter* painter) const;
    void draw() const;

    // Serialization.
    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream &in);
};

QDataStream& operator<<(QDataStream& out, const OdomSensor &o);
QDataStream& operator>>(QDataStream& in, OdomSensor &o);
QDebug operator<<(QDebug dbg, const OdomSensor &o);

#endif

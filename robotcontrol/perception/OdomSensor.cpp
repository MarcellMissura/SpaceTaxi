#include "OdomSensor.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"

// The OdomSensor class bundles all things related to the odometry sensor.

OdomSensor::OdomSensor()
{
    frameId = 0;
    inited = false;
}

OdomSensor::~OdomSensor()
{

}

// Copy constructor.
OdomSensor::OdomSensor(const OdomSensor &o)
{
    *this = o;
}

// Assignment operator.
// It is needed to copy the internal buffers.
// An assignment deep copies the OdomSensor object.
// The assignment operator needs to exist so that the
// operation can be mutexed.
OdomSensor& OdomSensor::operator=(const OdomSensor &o)
{
    if (this == &o)
        return *this;

    QMutexLocker locker(&mutex);
    QMutexLocker locker2(&o.mutex);

    frameId = o.frameId;
    odomPose = o.odomPose;

    return *this;
}

// Locks the mutex and overwrites the odom data.
void OdomSensor::writeOdomPose(const Pose2D &op, bool saveHistory)
{
    QMutexLocker locker(&mutex);
    frameId++;
    odomPose = op;

    // Init pose tracking.
    // This is not in the init() function because we need to have received an odomPose.
    if (!inited)
    {
        inited = true;
        initialOdomPose = odomPose;
        lastOdomPose = odomPose; // The odom pose is given relative to some other random place.
    }

    // Keeping these only for visualization.
    if (saveHistory)
        odomHistory << odomPose;
}

// Locks the mutex to wait for write operations to finish and returns a copy
// of the odom increment. The increment is counted with respect to the odom
// pose at the last call to this function. This is a thread safe operation.
Pose2D OdomSensor::readOdomIncrement()
{
    QMutexLocker locker(&mutex);
    Pose2D odomIncrement = odomPose - lastOdomPose;
    lastOdomPose = odomPose;
    return odomIncrement;
}

// Locks the mutex to wait for write operations to finish and returns a copy
// of the odom pose. This is a thread safe operation.
Pose2D OdomSensor::readOdomPose() const
{
    QMutexLocker locker(&mutex);
    return odomPose - initialOdomPose;
}

// Draws the odom history on a QPainter in 3 modes.
// 0 - draw nothing
// 1 - draw the laser beams and laser points
// 2 - draw the points and the extracted features
void OdomSensor::draw(QPainter *painter) const
{
    QMutexLocker locker(&mutex);
    for (uint i = 0; i < odomHistory.size(); i++)
        drawUtil.drawNoseCircle(painter, odomHistory[i], drawUtil.penThin, drawUtil.brushIvory, 0.1);
}

// Draws the sensor data.
void OdomSensor::draw() const
{
    QMutexLocker locker(&mutex);
    //GLlib::drawNoseCircle(odomDiff+poseHistory.last(), drawUtil.transparent, 0.6*config.agentRadius);

    // not done yet

}

void OdomSensor::streamOut(QDataStream& out) const
{
    //QMutexLocker locker(&mutex);
    out << odomPose;
}

void OdomSensor::streamIn(QDataStream &in)
{
    //QMutexLocker locker(&mutex);
    in >> odomPose;
}

QDataStream& operator<<(QDataStream& out, const OdomSensor &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, OdomSensor &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const OdomSensor &o)
{
    dbg << o.readOdomPose();
    return dbg;
}

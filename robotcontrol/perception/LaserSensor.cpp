#include "LaserSensor.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"
#include "blackboard/State.h"
#include "lib/geometry/Line.h"
#include "lib/util/ColorUtil.h"
#include "lib/util/GLlib.h"

// The LaserSensor class bundles all things related to the laser sensor we are using
// for perception. The LaserSensor class contains a buffer for the data, which make
// up the points of the laser rays.
// The LaserSensor class offers an interface to read and write this buffer, and functions
// to process the laser data to a visiblity polygon and high-fidelity lines features
// and corners.

LaserSensor::LaserSensor()
{
    frameId = 0;
    init();
}

LaserSensor::~LaserSensor()
{

}

// Copy constructor.
LaserSensor::LaserSensor(const LaserSensor &o)
{
    *this = o;
}

// Assignment operator.
// It is needed to copy the internal buffers.
// An assignment deep copies the LaserSensor object.
// The assignment operator needs to exist so that the
// operation can be mutexed.
LaserSensor& LaserSensor::operator=(const LaserSensor &o)
{
    if (this == &o)
        return *this;

    QMutexLocker locker(&mutex);

    frameId = o.frameId;
    laserInfo = o.laserInfo;
    laserToBasePose = o.laserToBasePose;
    pointBuffer = o.pointBuffer;

    return *this;
}


// Initialization after construction.
void LaserSensor::init()
{
    // Default for the HSR
    laserToBasePose.setPos(0.167,0);
    laserToBasePose.setHeading(0.0);
}

// Returns a descriptor with information about the laser sensor.
const LaserInfo &LaserSensor::getLaserInfo() const
{
    return laserInfo;
}

// Sets a descriptor with information about the laser sensor.
void LaserSensor::setLaserInfo(const LaserInfo &info)
{
    this->laserInfo = info;
}

void LaserSensor::setLaserToBasePose(const Pose2D &p)
{
    this->laserToBasePose = p;
}

const Pose2D &LaserSensor::getLaserToBasePose() const
{
    return laserToBasePose;
}

// Locks the mutex and overwrites the buffer with the data in ob.
void LaserSensor::writePointBuffer(const Vector<Vec2>& ob)
{
    QMutexLocker locker(&mutex);
    frameId++;
    pointBuffer = ob;
    return;
}

// Locks the mutex and overwrites the ob buffer with the data in this object.
// This is a thread safe way of copying the data. Since the laser holds only
// a small amount of data, and you can use your own persitant buffer, this is
// the best way of accessing the laser points.
void LaserSensor::readPointBuffer(Vector<Vec2> &ob) const
{
    QMutexLocker locker(&mutex);
    ob = pointBuffer;
    return;
}

// Locks the mutex to wait for write operations to finish and returns a copy
// of the laser data. This is a thread safe way of accessing the data points,
// but it creates a new copy of the laser point buffer each time you call it.
Vector<Vec2> LaserSensor::readPointBuffer() const
{
    QMutexLocker locker(&mutex);
    return pointBuffer;
}

// Computes and returns the normals of the laser points.
Vector<Vec2> LaserSensor::getNormals() const
{
    QMutexLocker locker(&mutex);
    Vector<Vec2> normals;
    for (uint i = 0; i < pointBuffer.size(); i++)
    {
        int d = 4; // neighbour distance
        const Vec2& v1 = pointBuffer[max((int)i-d, 0)];
        const Vec2& v3 = pointBuffer[min(i+d, pointBuffer.size()-1)];
        normals << (v3-v1).normal();
    }
    return normals;
}

// Extracts high-fidelity line features from the laser sensor based on the visibility
// polygon. The line features are always at least config.laserMinLineLength long.
// Clearly seen corners are marked in the lines in the seenP1 and seenP2 flags.
const Vector<TrackedLine> &LaserSensor::extractLines(bool debug) const
{
    const Polygon& visibilityPolygon = getVisibilityPolygon();

    thread_local Vector<TrackedLine> lineResultBuffer;
    lineResultBuffer.clear();
    ListIterator<Line> it = visibilityPolygon.edgeIterator();
    while (it.hasNext())
    {
        const Line& line0 = it.peekPrev();
        const Line& line2 = it.peekNext();
        const Line& line1 = it.next();

        // Discard sight lines.
        if (line1.isSightLine())
            continue;

        // Discard lines that are too short.
        if (line1.length() < config.laserMinLineLength)
        {
            if (debug)
                qDebug() << "Discarding line" << line1 << "due to shortness." << line1.length();
            continue;
        }

        // Discard far away lines where both end points are too far away.
        if (line1.p1().norm() > config.laserMaxLineDistance && line1.p2().norm() > config.laserMaxLineDistance)
        {
            if (debug)
                qDebug() << "Discarding line" << line1 << "due to distance.";
            continue;
        }

        double parallelMeasureThreshold = 0.993;

        // Discard lines that are too closely aligned with the direction of the sensor ray.
        // These are often phantom lines that don't really exist.
        if (fabs(line1.p1().normalized() * line1.lineVector().normalized()) > parallelMeasureThreshold)
            continue;

        // Mark seen vertices. Seen vertices occur between blocking lines and sight lines, if the
        // sight line continues in the "away" direction, and between neighbouring blocking lines
        // that have a large enough angle to each other.
        bool seenP1 = false;
        bool seenP2 = false;
        if (line0.length() >= config.laserMinLineLength && (fabs(line0.p1().normalized() * line0.lineVector().normalized()) < parallelMeasureThreshold))
        {
            double angle1 = line0.angle(line1);
            if (line0.isBlockingLine() && fabs(angle1) > config.slamSeenCornerMinAngle)
                seenP1 = true;
            if (line0.isSightLine() && angle1 > config.slamSeenCornerMinAngle)
                seenP1 = true;
        }

        if (line2.length() >= config.laserMinLineLength && (fabs(line2.p1().normalized() * line2.lineVector().normalized()) < parallelMeasureThreshold))
        {
            double angle2 = line1.angle(line2);
            if (line2.isBlockingLine() && fabs(angle2) > config.slamSeenCornerMinAngle)
                seenP2 = true;
            if (line2.isSightLine() && angle2 > config.slamSeenCornerMinAngle)
                seenP2 = true;
        }

        // Add a new line.
        lineResultBuffer << TrackedLine(line1, seenP1, seenP2, state.frameId);
    }

    return lineResultBuffer;
}

// Extracts the visibility polygon from the laser sensor.
// The laser points are first broken up into clusters on boundaries with too
// much distance between two neighbouring points. Then, each cluster is Douglas
// Peuckerized on its own. The clusters are then combined to a single polygon
// where edges inside a cluster become blocking lines and edges between clusters
// become sight lines. Also, inverse spikes that are caused by small objects such
// as chair legs are removed (groomed) from the polygon. The returned polygon is
// star shaped and is guaranteed (almost) to be free of self-intersections.
const Polygon& LaserSensor::getVisibilityPolygon() const
{
    QMutexLocker locker(&mutex);

    thread_local Polygon visibilityPolygon;
    visibilityPolygon.clear();

    if (pointBuffer.isEmpty())
        return visibilityPolygon;

    // Pour the buffer into a Polygon and mark segment boundaries as sight lines
    // where the distance between two points is too large.
    visibilityPolygon.appendVertex(Vec2(), Line::SightLine); // This point is only needed for sensors that don't cover the full 360.
    visibilityPolygon.appendVertex(pointBuffer[0], Line::SightLine);
    for (uint i = 1; i < pointBuffer.size(); i++)
    {
        if ((pointBuffer[i-1]-pointBuffer[i]).norm() > config.laserSegmentDistanceThreshold
                || pointBuffer[i].norm() > config.laserLengthBound)
        {
            visibilityPolygon.appendVertex(pointBuffer[i], Line::SightLine);
            //qDebug() << "appended sight vertex" << pointBuffer[i] << visibilityPolygon.getEdges().last();
        }
        else
        {
            visibilityPolygon.appendVertex(pointBuffer[i], Line::BlockingLine);
            //qDebug() << "appended block vertex" << pointBuffer[i] << visibilityPolygon.getEdges().last();
        }
    }
    visibilityPolygon.getEdges().last().setType(Line::SightLine);
    //qDebug() << "last vertex set to sight" << visibilityPolygon.getEdges().last();


    // Unfortunately, just concatenating the laser points sometimes results in a
    // self-intersecting polygon. Smoothing with DP fixes most of the cases, but
    // unfortunately not all, so we will still attempt self-intersection later.
    // In fact, smoothing itself can introduce self-intersections. It would be
    // great to analyze how it happens and apply a local fix. Grooming may remove
    // a self-intersection, but this is not guaranteed. Thus, we are still short
    // of a mechanism that guarantees the absence of self-intersections. A full
    // blown self intersection removal is a costly O(N²) operation and we are
    // currently applying an approximate O(N) local fix with a stride of 3.
    // After grooming, only two self-intersecting cases remain in the bucket dataset,
    // one of which the self-intersection repair can fix and one it cannot. It's
    // probably a bigger issue than just the stride of 3. In the "other" dataset,
    // the local self-intersection repair fixes all seven cases. The issue has
    // become rare, but it still needs to be thoroughly investigated before we
    // can claim that we are guaranteeing no self-intersections.


    // Use the Douglas Peucker algorithm to simplify the segments to polylines.
    visibilityPolygon.simplify(config.laserDouglasPeuckerEpsilon);
    // Prune short edges from the polygon.
    ListIterator<Line> shortIt = visibilityPolygon.edgeIterator();
    while (!shortIt.atEnd())
    {
        if (shortIt.cur().length() < 0.035)
        {
            //qDebug() << "removing short segment" << shortIt << shortIt.cur().length();
            visibilityPolygon.removeEdge(shortIt);
        }
        else
        {
            shortIt.next();
        }
    }

    // Grooming. Remove inverse spikes.
    // Inverse spikes are very sharp inward corners that are caused by a single
    // laser point being way out of place due to sensor noise or a very small
    // object such as a chair leg.
    visibilityPolygon.rewriteEdgeIds();
    ListIterator<Line> spikeIt = visibilityPolygon.edgeIterator();
    while (!spikeIt.atEnd())
    {
        Line& l1 = spikeIt.peekPrev();
        const Line& l2 = spikeIt.cur();

        double f = l1.lineVector()*l2.lineVector()/(l1.length()*l2.length());
        if (f < -0.98)
        {
            if (l2.isSightLine())
                l1.setSightLine();
            visibilityPolygon.removeEdge(spikeIt); // This also steps the iterator.
        }
        else
        {
            spikeIt.next();
        }
    }

//    if (visibilityPolygon.isSelfIntersecting()) // Can be removed when it no longer occurs.
//        qDebug() << state.frameId << "The groomed laser result is self intersecting.";

    visibilityPolygon.repairSelfIntersections(3); // Loop removal with a narrow window.
    visibilityPolygon.rewriteEdgeIds();

    //qDebug() << "vis pol computed:" << visibilityPolygon;

    return visibilityPolygon;
}

// Smoothes the points with a configurable low-pass filter
// and removes small segments.
void LaserSensor::smoothPoints()
{
    QMutexLocker locker(&mutex);

    thread_local Vector<Vec2> pointBufferCpy;
    pointBufferCpy.clear();
    thread_local Vector<Vec2> segment;
    segment.clear();

    // Remove small clusters.
    int firstSegmentBoundaryIdx = 0;
    for (int i = 1; i < pointBuffer.size(); i++)
    {
        if ((pointBuffer[i]-pointBuffer[i-1]).norm() > config.laserSegmentDistanceThreshold)
        {
            if (segment.size() >= config.laserSmoothingMinSegmentSize)
                pointBufferCpy << segment;
            segment.clear();

            if (firstSegmentBoundaryIdx == 0)
            {
                firstSegmentBoundaryIdx = i;
                pointBufferCpy.clear();
            }
        }

        segment << pointBuffer[i];
    }
    for (int i = 0; i < firstSegmentBoundaryIdx; i++)
        segment << pointBuffer[i];
    pointBufferCpy << segment;

    pointBuffer = pointBufferCpy;

    // Multi pass low-pass filter.
    for (uint n = 0; n < config.laserSmoothingPasses; n++) // multiple passes!
    {
        for (int i = 0; i < pointBuffer.size(); i++)
        {
            uint j = mod(i-1, pointBuffer.size());
            uint k = (i+1) % pointBuffer.size();

            // Do not interpolate over segment boundaries.
            if ((pointBuffer[i]-pointBuffer[j]).norm() > config.laserSegmentDistanceThreshold
                    || (pointBuffer[i]-pointBuffer[k]).norm() > config.laserSegmentDistanceThreshold)
                continue;

            pointBufferCpy[i] = 0.5 * (pointBuffer[i] + (pointBuffer[j] + 0.5*(pointBuffer[k]-pointBuffer[j])));
        }

        pointBuffer = pointBufferCpy;
    }

    return;
}

// Draws the laser points on a QPainter in 3 modes.
// 0 - draw nothing
// 1 - draw the laser beams and laser points
// 2 - draw the extracted lines and beam and points
void LaserSensor::draw(QPainter *painter) const
{
    if (command.showLidar == 0)
        return;

    if (pointBuffer.isEmpty())
        return;

    // Raw laser points segmentwise connected.
    if (command.showLidar > 0)
    {
        // Segment the point buffer into contiguous sequences by splitting where the
        // distance between two points is too large.
        mutex.lock();
        Vector<Vector<Vec2> > segments;
        Vector<Vec2> segment;
        segment << pointBuffer[0];
        for (uint i = 1; i < pointBuffer.size(); i++)
        {
            if ((pointBuffer[i-1]-pointBuffer[i]).norm() > config.laserSegmentDistanceThreshold)
            {
                segments << segment;
                segment.clear();
            }
            segment << pointBuffer[i];
        }
        segments << segment;
        mutex.unlock();

        painter->save();
        for (uint i = 0; i < segments.size(); i++)
        {
            QColor c = drawUtil.randomColor(i);

            // Draw the segment connecting lines.
            QPen pen;
            pen.setCosmetic(true);
            pen.setWidth(2);
            pen.setColor(c);
            painter->setPen(pen);
            painter->setBrush(QBrush(c));
            for (uint j = 1; j < segments[i].size(); j++)
                painter->drawLine(segments[i][j-1], segments[i][j]);

            // Draw the ray in faint color.
            painter->setPen(drawUtil.penRedThin);
            painter->setBrush(drawUtil.brushRed);
            painter->setOpacity(0.1);
            for (uint j = 0; j < segments[i].size(); j++)
                painter->drawLine(QPointF(), segments[i][j]);
            painter->setOpacity(1.0);

            // Draw the end points.
            painter->setBrush(QBrush(c));
            for (uint j = 0; j < segments[i].size(); j++)
                painter->drawEllipse(segments[i][j], 0.03, 0.03);
        }
        painter->restore();
    }

    // Line features as extracted for line mapping.
    if (command.showLidar == 2)
    {
        painter->save();
        QPen penThick;
        penThick.setCosmetic(true);
        penThick.setWidth(8);
        penThick.setColor(drawUtil.blue);
        painter->setPen(penThick);
        painter->setBrush(drawUtil.brushBlue);
        Vector<TrackedLine> lines = extractLines();
        for (uint i = 0; i < lines.size(); i++)
        {
            painter->drawLine(lines[i].p1(), lines[i].p2());
            if (lines[i].seenP1)
                painter->drawEllipse(lines[i].p1(), 0.05, 0.05);
            if (lines[i].seenP2)
                painter->drawEllipse(lines[i].p2(), 0.05, 0.05);
        }
        painter->restore();
    }
}

// Draws the sensor data.
void LaserSensor::draw() const
{
    if (command.showLidar == 0)
        return;

    if (pointBuffer.isEmpty())
        return;

    // Raw laser points segmentwise connected.
    if (command.showLidar == 1 || command.showLidar == 3)
    {
        // Segment the point buffer into contiguous sequences by splitting where the
        // distance between two points is too large.
        mutex.lock();
        Vector<Vector<Vec2> > segments;
        Vector<Vec2> segment;
        segment << pointBuffer[0];
        for (uint i = 1; i < pointBuffer.size(); i++)
        {
            if ((pointBuffer[i-1]-pointBuffer[i]).norm() > config.laserSegmentDistanceThreshold)
            {
                segments << segment;
                segment.clear();
            }
            segment << pointBuffer[i];
        }
        segments << segment;
        mutex.unlock();


        // Connected laser points.
        glPointSize(5);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);
        for (uint i = 0; i < segments.size(); i++)
        {
            QColor c = drawUtil.randomColor(i);
            glColor3f(c.redF(), c.greenF(), c.blueF());
            for (uint j = 0; j < segments[i].size(); j++)
                glVertex2dv(segments[i][j]);
        }
        glEnd();
        glColor3f(1.0, 0.0, 0.0);
        glLineWidth(1);
        glBegin(GL_LINES);
        for (uint i = 0; i < segments.size(); i++)
        {
            QColor c = drawUtil.randomColor(i);
            glColor3f(c.redF(), c.greenF(), c.blueF());
            for (uint j = 1; j < segments[i].size(); j++)
            {
                glVertex2dv(segments[i][j-1]);
                glVertex2dv(segments[i][j]);
            }
        }
        glEnd();
    }

    // Line features as extracted for line mapping.
    if (command.showLidar == 2 || command.showLidar == 3)
    {
        glPushMatrix();
        glTranslated(0,0,0.01);
        glColor3f(0.0, 0.0, 1.0);
        glLineWidth(5);
        Vector<TrackedLine> lines = extractLines();
        for (uint i = 0; i < lines.size(); i++)
        {
            GLlib::drawLine(lines[i].p1(), lines[i].p2(), config.laserDouglasPeuckerEpsilon);
            if (lines[i].seenP1)
                GLlib::drawFilledCircle(lines[i].p1(), drawUtil.brushBlue.color(), 0.05);
            if (lines[i].seenP2)
                GLlib::drawFilledCircle(lines[i].p2(), drawUtil.brushBlue.color(), 0.05);
        }
        glPopMatrix();
    }
}

void LaserSensor::printInfo() const
{
    QMutexLocker locker(&mutex);
    for (uint i = 0; i < pointBuffer.size(); i++)
    {
        qDebug() << i << pointBuffer[i] << laserInfo.angleMin+i*laserInfo.angleIncrement << pointBuffer[i].angle()
                 << fabs(laserInfo.angleMin+i*laserInfo.angleIncrement - pointBuffer[i].angle());
    }
}

void LaserSensor::streamOut(QDataStream& out) const
{
    QMutexLocker locker(&mutex);
    out << frameId;
    out << pointBuffer;
}

void LaserSensor::streamIn(QDataStream &in)
{
    QMutexLocker locker(&mutex);
    in >> frameId;
    in >> pointBuffer;
}

QDataStream& operator<<(QDataStream& out, const LaserSensor &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, LaserSensor &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const LaserSensor &o)
{
    dbg << o.readPointBuffer();
    return dbg;
}

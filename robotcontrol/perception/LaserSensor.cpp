#include "LaserSensor.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"
#include "blackboard/State.h"
#include "lib/geometry/Line.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"
#include "lib/util/Statistics.h"

// The LaserSensor class bundles all things related to the laser sensor we are using
// for perception. The LaserSensor class contains a pointBuffer where the laser points
// are stored as Vec2. The LaserSensor class offers an interface to read and write this
// buffer in a thread safe manner, and functions to process the laser data to a visiblity
// polygon, high-fidelity line features and corners, and triangle markers.

LaserSensor::LaserSensor()
{
    frameId = 0;
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
    QMutexLocker locker2(&o.mutex);

    frameId = o.frameId;
    laserInfo = o.laserInfo;
    laserToBasePose = o.laserToBasePose;
    rangeBuffer = o.rangeBuffer;
    rangeHistory = o.rangeHistory;
    unitVectors = o.unitVectors;
    pointBuffer = o.pointBuffer;

    return *this;
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

    unitVectors.clear();
    Vec2 base(1,0);
    for (int i = 0; i < info.rays; i++)
        unitVectors << base.rotated(laserInfo.angleMin + i*laserInfo.angleIncrement);
}

// Sets the transformation between the base link and the laser sensor.
void LaserSensor::setLaserToBasePose(const Pose2D &p)
{
    this->laserToBasePose = p;
}

// Retrieves the transformation between the base link and the laser sensor.
const Pose2D &LaserSensor::getLaserToBasePose() const
{
    return laserToBasePose;
}

// Locks the mutex and overwrites the point buffer with new points
// computed from the ranges. This will only work if the LaserInfo
// has been set correctly.
void LaserSensor::writeRangeBuffer(const Vector<double> &rb)
{
    QMutexLocker locker(&mutex);
    frameId++;

    this->rangeBuffer = rb;
    if (command.laserTemporalFilter)
    {
        rangeHistory.setMaxSize(config.laserSmoothingMedianFilterSize);
        rangeHistory.push(rangeBuffer);
    }

    pointBuffer.clear();
    for (uint i = 0; i < rangeBuffer.size(); i++)
        if (!isnull(rangeBuffer[i]) && !isnan(rangeBuffer[i])) // Discard nans and zeros.
            pointBuffer << unitVectors[i] * rangeBuffer[i];

//    qDebug() << rangeBuffer;
//    qDebug() << pointBuffer;
}

// Locks the mutex and overwrites the rb buffer with the data in this object.
// This is a thread safe way of copying the ranges. Since you can use your own
// persitant buffer, this is the best way of accessing the laser ranges.
void LaserSensor::readRangeBuffer(Vector<double> &rb) const
{
    QMutexLocker locker(&mutex);
    rb = rangeBuffer;
    return;
}

// Locks the mutex to wait for write operations to finish and returns a copy
// of the laser ranges. This is a thread safe way of accessing the ranges,
// but it creates a new copy of the range buffer each time you call it.
Vector<double> LaserSensor::readRangeBuffer() const
{
    QMutexLocker locker(&mutex);
    return rangeBuffer;
}


// Locks the mutex and overwrites the point buffer with the data in pb.
void LaserSensor::writePointBuffer(const Vector<Vec2>& pb)
{
    QMutexLocker locker(&mutex);
    frameId++;
    pointBuffer = pb;
    return;
}

// Locks the mutex and overwrites the pb buffer with the data in this object.
// This is a thread safe way of accessing the points. Since you can use your
// own persitant buffer, this is the best way of accessing the laser points.
void LaserSensor::readPointBuffer(Vector<Vec2> &pb) const
{
    QMutexLocker locker(&mutex);
    pb = pointBuffer;
    return;
}

// Locks the mutex to wait for write operations to finish and returns a copy
// of the laser points. This is a thread safe way of accessing the laser points,
// but it creates a new copy of the point buffer each time you call it.
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

        // Discard lines that are too short, too far away, or have too little angle to the ray direction.
        if (!isLineOkay(line1))
            continue;

        // Mark seen vertices. Seen vertices occur between blocking lines and sight lines, if the
        // sight line continues in the "away" direction, and between neighbouring blocking lines
        // that have a large enough angle to each other.
        bool seenP1 = false;
        bool seenP2 = false;
        if (isLineOkay(line0))
        {
            double angle1 = line0.angle(line1);
            if (line0.isBlockingLine() && fabs(angle1) > config.slamSeenCornerMinAngle)
                seenP1 = true;
            if (line0.isSightLine() && angle1 > config.slamSeenCornerMinAngle)
                seenP1 = true;
        }

        if (isLineOkay(line2)) // This leads to a segfault in frame 436
        {
            double angle2 = line1.angle(line2);
            if (line2.isBlockingLine() && fabs(angle2) > config.slamSeenCornerMinAngle)
                seenP2 = true;
            if (line2.isSightLine() && angle2 > config.slamSeenCornerMinAngle)
                seenP2 = true;
        }

        // Add the detected line to the buffer.
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
        if ((pointBuffer[i-1]-pointBuffer[i]).norm() > config.laserSegmentDistanceThreshold)
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
            visibilityPolygon.removeEdge(shortIt);
        else
            shortIt.next();
    }

//    if (visibilityPolygon.isSelfIntersecting()) // Can be removed when it no longer occurs.
//        qDebug() << state.frameId << "The groomed laser result is self intersecting.";

    visibilityPolygon.repairSelfIntersections(3); // Loop removal with a narrow window.
    visibilityPolygon.rewriteEdgeIds();

    //qDebug() << "vis pol computed:" << visibilityPolygon;

    return visibilityPolygon;
}

// Returns the detected triangle dock frame.
Polygon LaserSensor::extractTriangleMarker(int debug) const
{
    // The way this algorithm works is that for every point in the point buffer, we assume the point to
    // be the tip of the triangle and try to find triangle candidates in the neighborhood (stride) around
    // the point by validating whether the tip, a point to the left of the tip and a point to the right of
    // the tip make a triangle of the right size, shape, and orientation. Every candidate that matches the
    // search criteria is then voted by all points in the stride. The points in the stride that are close
    // enough to one of the legs of the triangle candidate votes for the triangle. Triangle hypotheses are
    // then clustered, i.e., similar triangles are grouped together, and the cluster with the most total
    // votes is selected. In the end, the algorithm returns a triangle and its pose computed as the weighted
    // average of the most voted cluster.

    QMutexLocker locker(&mutex);

    thread_local Vector<Polygon> triangles;
    triangles.clear();
    thread_local Vector<Pose2D> poses;
    poses.clear();
    thread_local Vector<uint> votes;
    votes.clear();

    // Determine the expected scalar product of the normalized triangle legs.
    // This scalar product will be used for a quick shape check.
    double fs = fsin(0.5*config.laserTriangleAngle);
    double expectedScalar = 1.0-2.0*fs*fs;

    // Further parameters that determine the selection of triangle candidates.
    double legLength = config.laserTriangleLegLength; // The expected length of a triangle leg.
    double maxLegDifference = 0.15; // The max deviation of the two leg lengths in percent.
    double maxExpectedScalarDiff = 0.15; // Max deviation from the expected scalar product of the legs.
    double maxOrthogonalDist = 0.008; // Max deviation of a voting point to the triangle leg in meters.

    // For every laser point in the point buffer...
    for (uint i = 0; i < pointBuffer.size(); i++)
    {
        Vec2 a = pointBuffer[i]; // Assume the point is point a, the tip of the triangle.

        // Compute the size of the neighborhood (stride) that we scan left and right of the point in
        // order to discover triangle candidates. The stride is computed based on the distance to point i.
        // How many laser rays left and right of the point do we have to check to cover the expected leg
        // length of the triangle?
        uint stride = fatan2(legLength, a.norm())/laserInfo.angleIncrement;

        // Skip pointlessly small strides.
        if (stride < 3)
            continue;

        // This check makes it easy to deal with the boundaries of the sensor range.
        if (i <= stride || i >= pointBuffer.size()-stride)
            continue;

        if (debug > 1)
            qDebug() << i << "Scanning point" << a;

        // Scan for triangle candidates in the stride.
        for (uint j = 1; j <= stride; j++)
        {
            // Points b and c are the legs of the triangle with respect to point a.
            Vec2 b = pointBuffer[i-j]; // b is to the right of the tip.
            Vec2 c = pointBuffer[i+j]; // c is to the left of the tip.

            // Check for shape, size, and orientation.
            double n1 = (b-a).norm();
            double n2 = (c-a).norm();
            bool scalarProductRight = fabs(((b-a)/n1)*((c-a)/n1) - expectedScalar) < maxExpectedScalarDiff;
            bool reasonableSize = n1 < legLength && n1 > 0.4*legLength && n2 < legLength && n2 > 0.4*legLength && fabs(n1/n2-1.0) < maxLegDifference;
            bool frontFacing = -c.det(b-c) * (a-c).det(b-c) > 0;
            if (debug > 1)
                qDebug() << "  " << j << "|"
                         << scalarProductRight << frontFacing << reasonableSize
                         << "| scalar:" << fabs(((b-a)/n1)*((c-a)/n1) - expectedScalar)
                         << "| size:" << n1 << n2 << fabs((n1/n2)-1.0);

            // If all criteria are right, we have a triangle candidate.
            if (scalarProductRight && reasonableSize && frontFacing)
            {
                // Count the supporting points along the legs of the triangle candidate.
                uint voteCountLeft = 0;
                uint voteCountRight = 0;
                Line leg1(a, b);
                Line leg2(a, c);
                for (uint k = 1; k <= stride; k++)
                {
                    // skip self votes
                    if (k == j)
                        continue;

                    if (fabs(leg1.orthogonalDistance(pointBuffer[i-k])) < maxOrthogonalDist)
                        voteCountRight++;
                    if (fabs(leg2.orthogonalDistance(pointBuffer[i+k])) < maxOrthogonalDist)
                        voteCountLeft++;
                    if (debug > 2)
                        qDebug() << "      " << k << "|" << voteCountRight << voteCountLeft << "|" << fabs(leg1.orthogonalDistance(pointBuffer[i+k])) << fabs(leg2.orthogonalDistance(pointBuffer[i-k]));
                }

                // Discard candidates with too few votes either left or right.
                if (voteCountLeft <= max((uint)3, stride/3) || voteCountRight <= max((uint)3, stride/3))
                    continue;

                // Construct the triangle and its pose.
                Polygon trig;
                trig << a << b << c;
                Pose2D pose;
                pose.setPos(a);
                pose.setOrientation((b-c).angle()+PI2);
                triangles << trig; // Only for visualization.
                poses << pose;
                votes << voteCountLeft+voteCountRight;

                if (debug > 1)
                    qDebug() << "   triangle:" << a << b << c << "pose:" << pose << "votes:" << voteCountLeft << voteCountRight;
            }
        }
    }

    if (poses.isEmpty())
        return Polygon();

    // Now we have a set of triangle and pose hypotheses along with their votes.

    // Cluster the hypotheses by their pose.
    Vector<Vector<uint> > clusterIdx = Statistics::cluster(poses, 0.2);


    // Determine the index and the votes of the largest cluster.
    uint largestClusterVotes = 0;
    uint largestClusterIdx = 0;
    for (uint i = 0; i < clusterIdx.size(); i++)
    {
        uint clusterVotes = 0;
        for (uint j = 0; j < clusterIdx[i].size(); j++)
            clusterVotes += votes[clusterIdx[i][j]];
        if (clusterVotes > largestClusterVotes)
        {
            largestClusterIdx = i;
            largestClusterVotes = clusterVotes;
        }
    }

    if (debug > 0)
    {
        qDebug() << "Clusters:";
        for (uint i = 0; i < clusterIdx.size(); i++)
        {
            uint clusterVotes = 0;
            for (uint j = 0; j < clusterIdx[i].size(); j++)
                clusterVotes += votes[clusterIdx[i][j]];
            qDebug() << i << "Cluster votes:" << clusterVotes;
            for (uint j = 0; j < clusterIdx[i].size(); j++)
                qDebug() << "  " << j << "votes:" <<  votes[clusterIdx[i][j]] << "pose:" << poses[clusterIdx[i][j]];
        }
    }

    // Compute the weighted average pose of the largest cluster. That's our triangle.
    double totalWeight = 0;
    Pose2D avgPose;
    for (uint j = 0; j < clusterIdx[largestClusterIdx].size(); j++)
    {
        Pose2D tr = poses[clusterIdx[largestClusterIdx][j]];
        double weight = votes[clusterIdx[largestClusterIdx][j]];
        avgPose.x += weight*tr.x;
        avgPose.y += weight*tr.y;
        avgPose.z += weight*tr.z;
        totalWeight += weight;
    }
    avgPose /= totalWeight;

    Polygon avgTriangle;
    avgTriangle << Vec2()
                << Vec2(legLength, 0).rotated(0.5*config.laserTriangleAngle)
                << Vec2(legLength, 0).rotated(-0.5*config.laserTriangleAngle);
    avgTriangle.setPose(avgPose);
    return avgTriangle;
}

// Main control for all filters (temporal, spatial, and speckle removal).
// The filters can be enabled and disable in command and tuned with config parameters.
void LaserSensor::filter()
{
    if (command.laserTemporalFilter)
        temporalFilter();
    if (command.laserSpeckleRemoval)
        speckleRemoval();
    if (command.laserSpatialFilter)
        spatialFilter();
}

// Removes small clusters of points as determined by the parameters:
// config.laserSegmentDistanceThreshold - maximum distance between points in a segment
// config.laserSmoothingMinSegmentSize - minimum number of points in a cluster (otherwise removed).
// It removes points from the pointBuffer.
void LaserSensor::speckleRemoval()
{
    QMutexLocker locker(&mutex);

    thread_local Vector<Vec2> pointBufferCpy;
    pointBufferCpy.clear();
    thread_local Vector<Vec2> segment;
    segment.clear();

    // Remove small clusters.
    for (int i = 1; i < pointBuffer.size(); i++)
    {
        if ((pointBuffer[i]-pointBuffer[i-1]).norm() > config.laserSegmentDistanceThreshold)
        {
            if (segment.size() >= config.laserSmoothingMinSegmentSize)
                pointBufferCpy << segment;
            segment.clear();
        }

        segment << pointBuffer[i];
    }
    if (segment.size() >= config.laserSmoothingMinSegmentSize)
        pointBufferCpy << segment;

    pointBuffer = pointBufferCpy;
}

// Smoothes the points with a configurable spatial low-pass filter.
// It replaces the points in the point buffer.
void LaserSensor::spatialFilter()
{
    QMutexLocker locker(&mutex);

    thread_local Vector<Vec2> pointBufferCpy;
    pointBufferCpy = pointBuffer;

    // Multi pass low-pass filter.
    for (uint n = 0; n < config.laserSmoothingSpatialPasses; n++) // multiple passes!
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

// Smoothes the ranges with a configurable median filter over historical data
// of each sensor ray. This filter needs to be applied first, because it replaces
// the data in the range buffer and the point buffer.
void LaserSensor::temporalFilter()
{
    QMutexLocker locker(&mutex);

    // Replace the range buffer with the Median of the historical data.
    Vector<double> psv;
    for (uint i = 0; i < laserInfo.rays; i++)
    {
        //qDebug() << "filtering point" << i;
        psv.clear();
        for (uint j = 0; j < rangeHistory.size(); j++)
        {
            double v = rangeHistory[j][i];
            if (!isnull(v) && !isnan(v)) // Discard nans and zeros.
                psv << v;
            //qDebug() << "  adding point" << j << ps.point << ps.length;
        }
        psv.sort();
        uint psvidx = psv.size()/2;
        rangeBuffer[i] = psv.isEmpty() ? 0 : psv[psvidx];
        //qDebug() << "median:" << psv.size() << psvidx << psv[psvidx].point;
    }

    // Rewrite the point buffer.
    pointBuffer.clear();
    for (uint i = 0; i < rangeBuffer.size(); i++)
        if (!isnull(rangeBuffer[i]) && !isnan(rangeBuffer[i])) // Discard nans and zeros.
            pointBuffer << unitVectors[i] * rangeBuffer[i];
}


// Returns true if a line is reliable, i.e. is long enough, close enough, and has a good angle.
bool LaserSensor::isLineOkay(const Line &l, bool debug) const
{
    // Discard lines that are too short.
    if (l.length() < config.laserLineMinLength)
    {
        if (debug)
            qDebug() << "Discarding line" << l << "due to shortness." << l.length();
        return false;
    }

    // Discard far away lines where both end points are too far away.
    if (l.p1().norm() > config.laserLineMaxDistance && l.p2().norm() > config.laserLineMaxDistance)
    {
        if (debug)
            qDebug() << "Discarding line" << l << "due to distance.";
        return false;
    }

    // Discard lines that are too closely aligned with the direction of the sensor ray.
    // These are often phantom lines that don't really exist.
    if (fabs((l.center()*l.lineVector()) / (l.center().norm()*l.length())) > fcos(config.laserLineMinAngle))
    {
        if (debug)
            qDebug() << "Discarding line" << l << "due to angle." << acos(fabs((l.center()*l.lineVector()) / (l.center().norm()*l.length())));
        return false;
    }

    return true;
}

// Draws the laser points on a QPainter in 3 modes.
// 0 - draw nothing
// 1 - draw the laser beams and laser points
// 2 - draw the points and the extracted features
void LaserSensor::draw(QPainter *painter) const
{
    if (command.showLaser == 0)
        return;

    if (pointBuffer.isEmpty())
        return;

    // Raw laser points segmentwise connected.
    if (command.showLaser > 0)
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
            if (segment.isEmpty() || segment.last() != pointBuffer[i])
                segment << pointBuffer[i];
        }
        segments << segment;
        mutex.unlock();

        uint counter = 0;
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

            // Draw the points.
            painter->setBrush(QBrush(c));
            for (uint j = 0; j < segments[i].size(); j++)
                painter->drawEllipse(segments[i][j], 0.008, 0.008);

            // Draw the labels.
            if (command.showLabels)
            {
                for (uint j = 0; j < segments[i].size(); j++)
                {
                    painter->save();
                    painter->translate(segments[i][j]);
                    painter->scale(0.0005, -0.0005);
                    painter->setOpacity(0.8);
                    painter->drawText(QPointF(), QString::number(counter++));
                    painter->restore();
                }
            }
        }
        painter->restore();
    }

    if (command.showLaser == 2)
    {
        // Line features as extracted for line mapping.
        QPen penThick;
        penThick.setCosmetic(true);
        penThick.setWidth(8);
        penThick.setColor(drawUtil.blue);
        Vector<TrackedLine> lines = extractLines();
        for (uint i = 0; i < lines.size(); i++)
            lines[i].draw(painter, penThick);

        // Line labels in blue.
        if (command.showLabels)
            for (uint i = 0; i < lines.size(); i++)
                lines[i].drawLabel(painter, drawUtil.penBlue, 1.0);

        // Docking triangle feature.
        Polygon triangle = extractTriangleMarker();
        if (!triangle.isEmpty())
        {
            triangle.draw(painter, drawUtil.penThick, drawUtil.brushYellow);
            drawUtil.drawFrame(painter, triangle.pose());
        }
    }
}

// Draws the sensor data.
void LaserSensor::draw() const
{
    if (command.showLaser == 0)
        return;

    if (pointBuffer.isEmpty())
        return;

    // Raw laser points segmentwise connected.
    if (command.showLaser == 1 || command.showLaser == 3)
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
    if (command.showLaser == 2 || command.showLaser == 3)
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

void LaserSensor::streamOut(QDataStream& out) const
{
    QMutexLocker locker(&mutex);
    out << frameId;
    out << laserInfo;
    out << laserToBasePose;
    out << rangeBuffer;
}

void LaserSensor::streamIn(QDataStream &in)
{
    QMutexLocker locker(&mutex);
    in >> frameId;
    in >> laserInfo;
    in >> laserToBasePose;
    in >> rangeBuffer;
    setLaserInfo(laserInfo);
    pointBuffer.clear();
    for (uint i = 0; i < rangeBuffer.size(); i++)
        if (!isnull(rangeBuffer[i]) && !isnan(rangeBuffer[i])) // Discard nans and zeros.
            pointBuffer << unitVectors[i] * rangeBuffer[i];
}

void LaserInfo::streamOut(QDataStream& out) const
{
    out << rays;
    out << angleMin;
    out << angleMax;
    out << angleIncrement;
    out << rangeMin;
    out << rangeMax;
    out << timeIncrement;
    out << scanTime;
}

void LaserInfo::streamIn(QDataStream &in)
{
    in >> rays;
    in >> angleMin;
    in >> angleMax;
    in >> angleIncrement;
    in >> rangeMin;
    in >> rangeMax;
    in >> timeIncrement;
    in >> scanTime;
}

QDataStream& operator<<(QDataStream& out, const LaserInfo &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, LaserInfo &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const LaserInfo &o)
{
    dbg << "LaserInfo:";
    dbg << "rays:" << o.rays;
    dbg << "angle min, max, increment:" << o.angleMin << o.angleMax << o.angleIncrement;
    dbg << "range min, max:" << o.rangeMin << o.rangeMax;
    dbg << "time increment, scan time:" << o.timeIncrement << o.scanTime;
    return dbg;
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
    dbg << o.readRangeBuffer();
    dbg << o.readPointBuffer();
    return dbg;
}

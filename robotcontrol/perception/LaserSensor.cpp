#include "LaserSensor.h"
#include "board/Config.h"
#include "board/Command.h"
#include "board/State.h"
#include "lib/geometry/Line.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"
#include "lib/util/Statistics.h"

// The LaserSensor class bundles all things related to the laser sensor we are using
// for perception. The LaserSensor class contains a rangeBuffer where the lengths of
// all laser rays are stored. The rangeBuffer is processed to a Vec2 pointBuffer using
// the LaserInfo that describes the max range and the angular resolution of the physical
// sensor. The pointBuffer contains the points that are actually seen, i.e. are non zero,
// non nan, and well below the maximum range. The point buffer is then filtered using
// spatial filtering and speckle removal. The remaining points are segmented by breaking
// them up into clusters where two neighboring points are distant to each other. The
// clusters are further processed by the Douglas Peucker algorithm resulting in line-like
// parts that may or may not be connected at their ends. The parts can be simplified by
// discarding all the in between points. Finally, the parts are processed to sensed
// polygons and to the visibility polygon.

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

// Locks the mutex and overwrites the range buffer with the new ranges in rb.
// Then it immediately computes the point buffer from the ranges and applies
// filtering. This will only work if the LaserInfo has been set correctly.
// This is the primary method to feed the laser sensor with data.
void LaserSensor::writeRangeBuffer(const Vector<double> &rb)
{
    QMutexLocker locker(&mutex);
    frameId++;

    this->rangeBuffer = rb;

    // Convert to point buffer.
    pointBuffer.clear();
    for (uint i = 0; i < rangeBuffer.size(); i++)
        if (!isnull(rangeBuffer[i]) && !isnan(rangeBuffer[i]) && rangeBuffer[i] < config.laserMaxRange) // Discard nans, zeros, and far away points.
            pointBuffer << unitVectors[i] * rangeBuffer[i];

    // Spatial filter.
    if (command.laserSpatialFilter)
        spatialFilter();

    // First line laser data filtering. Spatial filter and speckle removal.
    if (command.laserSpeckleRemoval)
        speckleRemoval();

    segmentation();
    reduction();

    //qDebug() << "points:" << pointSegments;
    //qDebug() << "lines:" << lineSegments;
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
// Using this function is discouraged as it is better to use writeRangeBuffer()
// to write the raw ranges into the sensor where filtering can be applied.
void LaserSensor::writePointBuffer(const Vector<Vec2>& pb)
{
    QMutexLocker locker(&mutex);
    frameId++;
    pointBuffer = pb;

    // Spatial filter.
    if (command.laserSpatialFilter)
        spatialFilter();

    // First line laser data filtering. Spatial filter and speckle removal.
    if (command.laserSpeckleRemoval)
        speckleRemoval();

    segmentation();
    reduction();

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
Vector<Vec2> LaserSensor::extractNormals() const
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
    const Polygon& visibilityPolygon = extractVisibilityPolygon();

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
        // that have a sufficiently large angle to each other.
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

        if (line2.length() >= config.laserLineMinLength && (fabs(line2.p1().normalized() * line2.lineVector().normalized()) < 0.98))
        //if (isLineOkay(line2)) // This leads to a segfault in frame 436
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
const Polygon& LaserSensor::extractVisibilityPolygon() const
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
Polygon LaserSensor::extractTriangle(int debug) const
{
    // The docking frame detection is based on a sample consensus approach.
    // For every point in the point buffer, we assume the point to
    // be the tip of the triangle and try to find triangle candidates in the neighborhood (stride) around
    // the point by validating whether the tip, a point to the left of the tip and a point to the right of
    // the tip make a triangle of the right size, shape, and orientation. Every candidate that matches the
    // search criteria is then voted by the points along the legs of the triangle. Triangle hypotheses are
    // clustered, i.e., triangles with similar poses are grouped together, and the cluster nearest to the
    // observer is selected. The pose of the observed triangle is computed as the weighted average of the
    // nearest cluster. Then, a low pass filter is applied to smooth the most recent triangle
    // observation against the one before. In the end, the algorithm returns a triangle polygon with its
    // pose set to the smoothed pose of the nearest cluster.

    QMutexLocker locker(&mutex);

    StopWatch sw;
    sw.start();

    //thread_local Vector<Polygon> triangles;
    triangles.clear(); // visualization
    thread_local Vector<Pose2D> poses;
    poses.clear();
    thread_local Vector<uint> votes;
    votes.clear();

    // The primary parameters of the procedure are the leg length of the triangular docking frame,
    // the angle between the legs of the triangle, and the strength of the low-pass smoothing.
    double legLength = config.laserTriangleLegLength; // The expected length of a triangle leg in meters.
    double triangleAngle = config.laserTriangleAngle; // The expected angle between the legs of the triangle in rad.
    double smoothing = config.laserTriangleSmoothing; // Low passs filter parameter for triangle smoothing.

    // Secondary parameters that determine the selection of triangle candidates.
    // These parameters only rarely need to be tuned and require a deep understanding of the code.
    double angleTolerance = 0.07; // Max deviation from the expected scalar product of the legs.
    double sizeTolerance = config.laserTriangleSizeTolerance; // Allowed triangle size deviation in percent.
    double symmetryTolerance = 0.05; // Allowed symmetry deviation of a triangle in percent.
    double maxOrthogonalDist = 0.008; // Max deviation of a voting point to the triangle leg in meters.
    int minPoints = 3; // Minimum points needed to form a stride and to confirm a triangle.

    // The triangle angle is processed to the scalar product of the two normalized triangle legs.
    double fs = fsin(0.5*triangleAngle);
    double expectedScalar = 1.0-2.0*fs*fs; // Angle processed to scalar product.

    uint observed = 10000;

    // For every laser point in the point buffer...
    for (int i = 0; i < pointBuffer.size(); i++)
    {
        Vec2 a = pointBuffer[i]; // Assume the point is point a, the tip of the triangle.

        // Compute the size of the neighborhood (the stride) that we scan left and right of the point i in
        // order to discover triangle candidates. The stride is computed based on the distance to point i.
        // How many laser rays left and right of the point i do we have to check to cover the expected leg
        // length of the triangle?
        int stride = 2.0*fatan2(0.5*legLength, a.norm())/laserInfo.angleIncrement; // overestimated

        // Skip pointlessly small strides.
        if (stride <= minPoints)
            continue;

        // This check makes it easy to deal with the boundaries of the sensor range.
        if (i <= stride || i >= pointBuffer.size()-stride)
            continue;

        if (debug > 1 && i == observed)
            qDebug() << i << "Scanning point" << a << "stride:" << stride;

        // Collect corner candidates that have about the right distance from the tip a.
        Vector<int> bIdx;
        Vector<int> cIdx;
        for (int j = minPoints; j <= stride; j += minPoints)
        {
            Vec2 b = pointBuffer[i-j]; // b is to the right of the tip
            Vec2 c = pointBuffer[i+j]; // c is to the left of the tip

            double n1 = (b-a).norm();
            double n2 = (c-a).norm();

//            if (debug > 1 && observed == i)
//            {
//                qDebug() << "  Testing b" << i-j << b << "n:" << n1 << (n1 < (1.0+sizeTolerance)*legLength && n1 > (1.0-sizeTolerance)*legLength);
//                qDebug() << "  Testing c" << i+j << c << "n:" << n2 << (n2 < (1.0+sizeTolerance)*legLength && n2 > (1.0-sizeTolerance)*legLength);
//            }

            if (n1 < (1.0+sizeTolerance)*legLength && n1 > (1.0-sizeTolerance)*legLength)
                bIdx << i-j;
            if (n2 < (1.0+sizeTolerance)*legLength && n2 > (1.0-sizeTolerance)*legLength)
                cIdx << i+j;
        }

        if (debug > 1 && observed == i)
        {
            qDebug() << "Right corner candidates b:" << bIdx;
            qDebug() << "Left corner candidates c:" << cIdx;
        }

        // Test all combos of left and right corner points for angle and orientation.
        for (int kb = 0; kb < bIdx.size(); kb++)
        {
            for (int kc = 0; kc < cIdx.size(); kc++)
            {
                Vec2 b = pointBuffer[bIdx[kb]]; // b is to the right of the tip.
                Vec2 c = pointBuffer[cIdx[kc]]; // c is to the left of the tip.
                double n1 = (b-a).norm();
                double n2 = (c-a).norm();

                bool frontFacing = -c.det(b-c) * (a-c).det(b-c) > 0;
                bool scalarProductCorrect = fabs(((b-a)/n1 * (c-a)/n2) - expectedScalar) < angleTolerance;
                bool reasonableSize = n1 < (1.0+sizeTolerance)*legLength && n1 > (1.0-sizeTolerance)*legLength
                                   && n2 < (1.0+sizeTolerance)*legLength && n2 > (1.0-sizeTolerance)*legLength
                                   && fabs(n1/n2 - 1.0) < symmetryTolerance;

                if (debug > 2 && i == observed)
                    qDebug() << "  " << bIdx[kb] << cIdx[kc] << "|"
                             << frontFacing << scalarProductCorrect << reasonableSize
                             << "| scalar:" << ((b-a)/n1)*((c-a)/n2) << "expected:" << expectedScalar << "max diff:" << angleTolerance
                             << "| size:" << n1 << n2 << "asymmetry:" << fabs((n1/n2)-1.0) << "max:" << symmetryTolerance;

                // If all criteria are right, we have a decent triangle candidate.
                // Now count the supporting points along the legs to perform a thorough check.
                if (scalarProductCorrect && reasonableSize && frontFacing)
                {
                    Line rightLeg(a, b);
                    Line leftLeg(a, c);

                    uint voteCountLeft = 0;
                    uint voteCountRight = 0;
                    uint totalCountLeft = 0;
                    uint totalCountRight = 0;

                    for (int m = i+minPoints; m <= cIdx.last(); m+=minPoints)
                    {
                        totalCountLeft++;
                        if (fabs(leftLeg.orthogonalDistance(pointBuffer[m])) < maxOrthogonalDist)
                            voteCountLeft++;
                        if (debug > 3 && i == observed)
                            qDebug() << "      " << m << " left |" << voteCountLeft << voteCountRight << "|" << leftLeg.orthogonalDistance(pointBuffer[m]);
                    }

                    for (int m = i-minPoints; m >= bIdx.last(); m-=minPoints)
                    {
                        totalCountRight++;
                        if (fabs(rightLeg.orthogonalDistance(pointBuffer[m])) < maxOrthogonalDist)
                            voteCountRight++;
                        if (debug > 3 && i == observed)
                            qDebug() << "      " << m << "right |" << voteCountLeft << voteCountRight << "|" << rightLeg.orthogonalDistance(pointBuffer[m]);
                    }

                    // Discard candidates with too few votes either left or right.
                    if (voteCountLeft <= max((uint)minPoints, totalCountLeft/2) || voteCountRight <= max((uint)minPoints, totalCountRight/2))
                        continue;

                    // Construct the triangle and its pose.
                    Polygon trig;
                    trig << a << b << c;
                    triangles << trig; // Only for visualization.
                    Pose2D pose;
                    pose.setPos(a);
                    pose.setOrientation((b-c).angle()+PI2);
                    poses << pose; // The poses are what really matters, they will be clustered to find the consensus set.
                    votes << voteCountLeft+voteCountRight; // The votes will become weights when computing the average of clusters.

                    if (debug > 1 && i == observed)
                        qDebug() << "   triangle:" << a << b << c << "pose:" << pose << "votes:" << voteCountLeft << voteCountRight;
                }
            }
        }
    }

    double time = sw.elapsedTimeMs();
    if (debug > 0)
        qDebug() << "Detection time:" << time << "ms";
    sw.start();

    // Didn't find anything at all? Return an empty polygon.
    if (poses.isEmpty())
        return Polygon();

    // Now we have a set of triangle and pose hypotheses along with their votes.

    // Cluster the hypotheses by their pose.
    Vector<Vector<uint> > clusterIdx = Statistics::cluster(poses, 0.2);

    // Compute the weighted average pose for every cluster.
    Vector<Pose2D> clusterPoses;
    for (uint i = 0; i < clusterIdx.size(); i++)
    {
        double totalWeight = 0;
        Pose2D avgPose;
        for (uint j = 0; j < clusterIdx[i].size(); j++)
        {
            Pose2D tr = poses[clusterIdx[i][j]];
            double weight = votes[clusterIdx[i][j]];
            avgPose.x = totalWeight*avgPose.x + weight*(avgPose.x + (tr.x - avgPose.x));
            avgPose.y = totalWeight*avgPose.y + weight*(avgPose.y + (tr.y - avgPose.y));
            avgPose.z = totalWeight*avgPose.z + weight*(avgPose.z + ffpicut(tr.z - avgPose.z));
            totalWeight += weight;
            avgPose /= totalWeight;
        }
        clusterPoses << avgPose;
    }

    // Determine the index of the closest cluster.
    uint closestClusterIdx = 0;
    double closestClusterDist = clusterPoses[0].pos().norm();
    for (uint i = 1; i < clusterIdx.size(); i++)
    {
        if (clusterPoses[i].pos().norm() < closestClusterDist)
        {
            closestClusterIdx = i;
            closestClusterDist = clusterPoses[i].pos().norm();
        }
    }

    if (debug > 1)
    {
        qDebug() << "Clusters:";
        for (uint i = 0; i < clusterIdx.size(); i++)
        {
            uint clusterVotes = 0;
            for (uint j = 0; j < clusterIdx[i].size(); j++)
                clusterVotes += votes[clusterIdx[i][j]];
            qDebug() << "  Cluster" << i << "size:" << clusterIdx[i].size() << "votes:" << clusterVotes << "pose:" << clusterPoses[i] << "dist:"  << clusterPoses[i].pos().norm();
            if (debug > 3)
                for (uint j = 0; j < clusterIdx[i].size(); j++)
                    qDebug() << "    " << j << "votes:" <<  votes[clusterIdx[i][j]] << "pose:" << poses[clusterIdx[i][j]];
        }
    }


    thread_local Pose2D dockingFrame;
    if (dockingFrame.isNull())
        dockingFrame = clusterPoses[closestClusterIdx];
    else
        dockingFrame.summed(clusterPoses[closestClusterIdx].diff(dockingFrame) * (1.0-smoothing));

    Polygon avgTriangle;
    avgTriangle << Vec2()
                << Vec2(legLength, 0).rotated(0.5*config.laserTriangleAngle)
                << Vec2(legLength, 0).rotated(-0.5*config.laserTriangleAngle);
    avgTriangle.setPose(dockingFrame);

    time = sw.elapsedTimeMs();
    if (debug > 0)
        qDebug() << "Cluster time:" << time << "ms";
    sw.start();

    return avgTriangle;
}

// Returns a Vector of Polygons that represent the obstacles as seen by the end points
// of the laser rays.
const Vector<Polygon>& LaserSensor::extractSensedPolygons() const
{
    QMutexLocker locker(&mutex);

    thread_local Vector<Polygon> sensedPolygons;
    sensedPolygons.clear();

    if (lineSegments.isEmpty())
        return sensedPolygons;

    double offset = config.laserPolygonThickness;
    for (uint i = 0; i < lineSegments.size(); i++)
    {
        Polygon pol;
        for (uint j = 0; j < lineSegments[i].size(); j++)
            pol << pointBuffer[lineSegments[i][j]];
        for (int j = lineSegments[i].size()-1; j >= 0; j--)
            pol << pointBuffer[lineSegments[i][j]] + pointBuffer[lineSegments[i][j]].normalized()*offset;
        sensedPolygons << pol;
    }

    return sensedPolygons;
}

// Segment the point buffer into contiguous sequences by splitting where the
// distance between two points is too large. The number of points remains the
// same, it's just that the point set is split up into smaller sets. The point
// buffer remains unmodified and the result is written into the pointSegments.
void LaserSensor::segmentation()
{
    pointSegments.clear();
    segment.clear();
    segment << 0;
    for (uint i = 1; i < pointBuffer.size(); i++)
    {
        if ((pointBuffer[i-1]-pointBuffer[i]).norm() > config.laserSegmentDistanceThreshold)
        {
            if (segment.size() >= config.laserSmoothingMinSegmentSize)
                pointSegments << segment;
            segment.clear();
        }
        if (segment.isEmpty() || segment.last() != i)
            segment << i;
    }
    if (segment.size() >= config.laserSmoothingMinSegmentSize)
        pointSegments << segment;
}

// Simplifies the point buffer by replacing sequences of points by lines with the
// Douglas Peucker (DP) algorithm.
// https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
// The point buffer remains unmodified, but the lineSegments structure is rewritten
// with lines that approximate the current point buffer.
void LaserSensor::reduction()
{
    lineSegments.clear();
    for (uint i = 0; i < pointSegments.size(); i++)
    {
        segment.clear();
        douglasPeuckerSub(pointSegments[i].first(), pointSegments[i].last()); // writes into segment
        lineSegments << segment;
    }
    return;
}

// Recursive subroutine that computes the Douglas Peucker algorithm
// on a set of points accessible by the iterators fromIt and toIt.
void LaserSensor::douglasPeuckerSub(uint fromIdx, uint toIdx)
{
    //qDebug() << " dp sub called from:" << fromIdx << "to:" << toIdx;

    // Construct a line from the first point in the set to the last.
    Line line(pointBuffer[fromIdx], pointBuffer[toIdx]);

    // Find the point with the maximum distance to the line.
    uint kmax = fromIdx;
    double dmax = 0;
    for (uint k = fromIdx+1; k < toIdx; k++)
    {
        double d = fabs(line.orthogonalDistance(pointBuffer[k]));
        if (d > dmax)
        {
            dmax = d;
            kmax = k;
        }
    }

    // If max distance is greater than epsilon, split the sequence
    // at the maximum distance point and simplify each half recursively.
    if (dmax > config.laserDouglasPeuckerEpsilon)
    {
        douglasPeuckerSub(fromIdx, kmax);
        douglasPeuckerSub(kmax, toIdx);
    }

    // Otherwise replace all points with the first and the last and end the recursion.
    else
    {
        if (segment.isEmpty() || segment.last() != fromIdx)
            segment << fromIdx;
        if (segment.last() != toIdx)
            segment << toIdx;
        //qDebug() << " dmax:" << dmax << maxIt.cur();
    }

    return;
}

// Removes small clusters of points from the point buffer as determined by the parameters:
// config.laserSegmentDistanceThreshold - maximum distance between points in a segment
// config.laserSmoothingMinSegmentSize - minimum number of points in a cluster (otherwise removed).
// The filter removes points from the pointBuffer and does not modify the rangeBuffer.
void LaserSensor::speckleRemoval()
{
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

// Smoothes the points in the point buffer with a configurable spatial low-pass filter.
// It replaces the points in the point buffer with a smoothed version computed from the neighbors.
void LaserSensor::spatialFilter()
{
    thread_local Vector<Vec2> pointBufferCpy;
    pointBufferCpy = pointBuffer;

    // Multi pass low-pass filter over the point buffer.
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
    if (min(fabs(l.p1().normalized() * l.lineVector().normalized()), fabs(l.p2().normalized() * l.lineVector().normalized())) > fcos(config.laserLineMinAngle))
    //if (fabs((l.center()*l.lineVector()) / (l.center().norm()*l.length())) > fcos(config.laserLineMinAngle))
    {
        if (debug)
            qDebug() << "Discarding line" << l << "due to angle." << fabs(l.p1().normalized() * l.lineVector().normalized()) << fabs(l.p2().normalized() * l.lineVector().normalized()) << fcos(config.laserLineMinAngle);
        return false;
    }

    return true;
}

// Draws the laser points on a QPainter in 3 modes.
// 0 - draw nothing
// 1 - draw the raw laser beams and laser points with labels
// 2 - draw the laser points segmentwise connected
// 3 - draw the extracted features
void LaserSensor::draw(QPainter *painter) const
{
    if (command.showLaser == 0)
        return;

    if (pointBuffer.isEmpty())
        return;

    // Raw laser points with labels.
    if (command.showLaser == 1)
    {
        mutex.lock();

        // Draw the raw laser rays with end points.
        for (uint i = 0; i < pointBuffer.size(); i++)
        {
            // Draw the ray in faint color.
            painter->setPen(drawUtil.penRedThin);
            painter->setOpacity(0.1);
            painter->drawLine(QPointF(), pointBuffer[i]);

            // Draw the end point.
            painter->setOpacity(1.0);
            painter->setBrush(drawUtil.brushRed);
            painter->drawEllipse(pointBuffer[i], 0.008, 0.008);
        }

        // Draw the label.
        if (command.showLabels)
        {
            painter->setPen(drawUtil.penThin);
            painter->setOpacity(1.0);
            for (uint i = 0; i < pointBuffer.size(); i++)
            {
                painter->save();
                painter->translate(pointBuffer[i]);
                painter->scale(0.00025, -0.00025);
                painter->drawText(QPointF(), QString::number(i));
                painter->restore();
            }
        }

        mutex.unlock();
    }

    // Raw laser points segmentwise connected.
    if (command.showLaser == 2)
    {
        mutex.lock();

        // Draw the segments of the point buffer.
        painter->save();
        for (uint i = 0; i < pointSegments.size(); i++)
        {
            QColor c = drawUtil.randomColor(i);

            // Draw the segment with connected lines.
            QPen pen;
            pen.setCosmetic(true);
            pen.setWidth(2);
            pen.setColor(c);
            painter->setPen(pen);
            painter->setBrush(QBrush(c));
            for (uint j = 1; j < pointSegments[i].size(); j++)
                painter->drawLine(pointBuffer[pointSegments[i][j-1]], pointBuffer[pointSegments[i][j]]);
            for (uint j = 0; j < pointSegments[i].size(); j++)
                painter->drawEllipse(pointBuffer[pointSegments[i][j]], 0.008, 0.008);

        }
        painter->restore();

        // Draw the simplified segments.
        painter->save();
        for (uint i = 0; i < lineSegments.size(); i++)
        {
            QColor c = drawUtil.randomColor(i+1);

            // Draw the segment with connected lines.
            QPen pen;
            pen.setCosmetic(true);
            pen.setWidth(3);
            pen.setColor(c);
            painter->setPen(pen);
            painter->setBrush(QBrush(c));
            for (uint j = 1; j < lineSegments[i].size(); j++)
            {
                painter->drawLine(pointBuffer[lineSegments[i][j-1]], pointBuffer[lineSegments[i][j]]);
                if (command.showLabels)
                {
                    Line line(pointBuffer[lineSegments[i][j-1]], pointBuffer[lineSegments[i][j]]);
                    line.drawLabel(painter);
                }
            }
        }
        painter->restore();

        mutex.unlock();
    }

    // Draw the extracted features.
    if (command.showLaser == 3)
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

        // Docking triangle frame.
        Polygon triangle = extractTriangle();
        for (uint i = 0; i < triangles.size(); i++)
            triangles[i].draw(painter, drawUtil.pen, drawUtil.brushYellow, 0.1);
        if (!triangle.isEmpty())
        {
            triangle.draw(painter, drawUtil.penThick, drawUtil.brushYellow);
            drawUtil.drawFrame(painter, triangle.pose());
        }

        // Sensed polygons.
        Vector<Polygon> dp = extractSensedPolygons();
        for (uint i = 0; i < dp.size(); i++)
            dp[i].draw(painter, drawUtil.pen, drawUtil.brushRed, 0.5);
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

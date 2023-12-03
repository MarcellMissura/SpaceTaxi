#include "SampleGrid.h"
#include "blackboard/Config.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"
#include <QGLViewer/qglviewer.h>

Vec3 Sample::up = Vec3(0,0,1);

SampleGrid::SampleGrid()
{
    debug = 0;
    upVector.z = 1;
    floorPlane.n = upVector;
}

// Computes a set of low resolution samples in image coordinates.
// It needs to know the width and height of the image that will be
// used as argument for the update() function. The number of
// samples is a configuration parameter.
void SampleGrid::init(uint width, uint height)
{
    samples.clear();
    for (int k = 0; k < config.samplesY; k++)
    {
        Vector<Sample> V;
        for (int l = 0; l < config.samplesX; l++)
        {
            int i = l*(width-1)/(config.samplesX-1);
            int j = height-1-k*(height-1)/(config.samplesY-1);
            Sample sample;
            sample.gridIdx = Vec2u(l,k);
            sample.imagePx = Vec2u(i,j);
            sample.bufferIdx = i+j*width;
            //qDebug() << sample.gridIdx << sample.imagePx << sample.bufferIdx;
            V << sample;
        }
        samples << V;
    }
}

// Copies the data from state.robot.readPointBuffer() and computes the normals of the low res samples.
void SampleGrid::update(const Vector<Vec3>& pointBuffer)
{
//    //qDebug() << "update start:" << floorAvg.n << floorAvg.p;
//    if (upVector*floorPlane.n > 0.5)
//    {
//        setUpVector(floorPlane.n);
//    }
//    else
//    {
//        qDebug() << "Out of synch" << floorPlane.n << upVector << upVector*floorPlane.n;
//    }

    // Copy the samples from the pointBuffer and reset all in flags.
    for (int i = 0; i < samples.size(); i++)
    {
        for (int j = 0; j < samples[i].size(); j++)
        {
            samples[i][j].p = pointBuffer[samples[i][j].bufferIdx];
            samples[i][j].in = !samples[i][j].p.isNull();
        }
    }

    // Compute all normals.
    Vec3 normal;
    for (int i = 0; i < samples.size(); i++)
    {
        for (int j = 0; j < samples[i].size(); j++)
        {
            if (!samples[i][j].in)
                continue;

            int upIdx = i+1;
            if (i == samples.size()-1 || !samples[upIdx][j].in)
                upIdx = i;

            int downIdx = i-1;
            if (i == 0 || !samples[downIdx][j].in)
                downIdx = i;

            int rightIdx = j+1;
            if (j == samples[i].size()-1 || !samples[i][rightIdx].in)
                rightIdx = j;

            int leftIdx = j-1;
            if (j == 0 || !samples[i][leftIdx].in)
                leftIdx = j;

            if (upIdx == downIdx || leftIdx == rightIdx)
                continue;

            Sample& up = samples[upIdx][j];
            Sample& down = samples[downIdx][j];
            Sample& right = samples[i][rightIdx];
            Sample& left = samples[i][leftIdx];
            normal = -((up.p-down.p)^(right.p-left.p));
            normal.normalize();
            samples[i][j].n = normal;

            //qDebug() << i << j << samples[i][j].in << samples[i][j];
        }
    }
}

// Sets the up vector to use for pruning and sorting.
void SampleGrid::setUpVector(const Vec3 &up)
{
    upVector = up;
    upVector.normalize();
    Sample::up = upVector;
}

// Returns the up vector.
Vec3 SampleGrid::getUpVector() const
{
    return upVector;
}

// 1 (Best if sorting works)
// Prune the samples
// Sort the pruned samples vertically
// Flood fill in sorted order
// Merge down to get floor plane
Sample SampleGrid::fitPlaneFlood()
{
    if (debug > 0)
        qDebug() << "SampleGrid::fitPlaneFlood(): up:" << upVector;

    prune();

    if (prunedSamples.size() < 2)
        return floorPlane;

    // Sort by z coordinate.
    prunedSamples.sort();

    // Reset things.
    planes.clear();
    planeAvg.clear();
    floorSegment.clear();
    floorPlane = prunedSamples[1]; // Always accept the first cluster.

    // Start a flood fill at every point in the sorted pruned set.
    for (int i = 2; i < prunedSamples.size()-1; i++)
    {
        if (debug > 0)
            qDebug() << "Trying" << prunedSamples[i].p << upVector*prunedSamples[i].p << "in:" << isIn(prunedSamples[i].gridIdx);

        if (!isIn(prunedSamples[i].gridIdx))
            continue;

        // Start a flood fill at this sample and collect similar samples in the neighbourhood.
        // This procedure will push similar points into the planeCluster.
        planeCluster.clear();
        floodFill(prunedSamples[i].gridIdx);

        // Ignore clusters that consist only of one point.
        if (planeCluster.size() == 1)
            continue;

        // Compute the average estimate of the cluster.
        Sample avg;
        avg.n.z = 0;
        for (int j = 0; j < planeCluster.size(); j++)
            avg += planeCluster[j];
        avg /= planeCluster.size();

        planeAvg << avg;
        planes << planeCluster;

        if (debug > 0)
            qDebug() << "New cluster:" << planeCluster.size() << "(" << floorSegment.size() << ")" << avg << "dist:" << floorPlane.distance(avg);

        // Merge the cluster with the floor plane if the distance is close.
        if (floorPlane.distance(avg) < config.mergeThreshold)
        {
            // Merge the new cluster into the floor.
            floorPlane.p = (floorPlane.p*floorSegment.size()+avg.p*planeCluster.size())/(floorSegment.size()+planeCluster.size());
            floorPlane.n = (floorPlane.n*floorSegment.size()+avg.n*planeCluster.size())/(floorSegment.size()+planeCluster.size());
            floorPlane.n.normalize();
            floorSegment << planeCluster;

            if (debug > 0)
                qDebug() << "Merged with floor. New rep:" << floorPlane;
        }

        // If that didn't work, a huge cluster can replace a very small one.
        if (floorSegment.size()*20 < planeCluster.size())
        {
            floorPlane = avg;
            floorSegment.clear();
            floorSegment << planeCluster;

            if (debug > 0)
                qDebug() << "Replaced floor." << floorSegment.size() << "avg:" << floorPlane;
        }
    }

    // Fit a plane to the points in the floor segment.
    if (floorSegment.size() > 2)
    {
        ols.reset();
        for (int i = 0; i < floorSegment.size(); i++)
            ols.addDataPoint(floorSegment[i].p);
        ols.init();
        floorPlane.n = ols.getNormal();
        floorPlane.p.z = ols.evaluateAt(floorPlane.p);
    }

    return floorPlane;
}


// 2 (Best if sorting makes no difference)
// Prune the samples
// Flood fill in grid order
// Sort segments (less sorting)
// Merge down
Sample SampleGrid::fitPlaneFlood2()
{
    if (debug > 0)
        qDebug() << "SampleGrid::fitPlaneFlood2():";

    prune();

    planes.clear();
    planeAvg.clear();

    for (int i = 0; i < prunedSamples.size(); i++)
    {
        // Start a flood fill at this sample and collect similar samples in the neighbourhood.
        // This procedure will push similar points into the planeCluster.
        planeCluster.clear();
        floodFill(prunedSamples[i].gridIdx);

        // Ignore clusters that consist only of one point.
        //if (planeCluster.size() == 1)
        //    continue;

        // Compute the average estimate of the cluster.
        Sample avg;
        avg.n.z = 0;
        for (int j = 0; j < planeCluster.size(); j++)
            avg += planeCluster[j];
        avg /= planeCluster.size();

        if (debug > 0)
            qDebug() << "New cluster" << i << "points:" << planeCluster.size() << "avg:" << avg;

        // Allocate a new cluster.
        avg.clusterId = planeAvg.size();
        planeAvg << avg;
        planes << planeCluster;
    }

    // Sort the plane segments.
    planeAvg.sort();

    // Merge down
    for (int i = 0; i < planeAvg.size(); i++)
    {
        Sample& avg = planeAvg[i];

        // Always accept the first cluster.
        if (i == 0)
        {
            floorPlane = planeAvg[i];
            floorSegment = planes[avg.clusterId];
            if (debug > 0)
                qDebug() << "New cluster:" << planeCluster.size() << avg;
            continue;
        }

        // Merge the cluster with the floor plane if the distance is close.
        if (floorPlane.distance(avg) < config.mergeThreshold)
        {
            // Merge the new cluster into the floor.
            floorPlane.p = (floorPlane.p*floorSegment.size()+avg.p*planes[avg.clusterId].size())/(floorSegment.size()+planes[avg.clusterId].size());
            floorPlane.n = (floorPlane.n*floorSegment.size()+avg.n*planes[avg.clusterId].size())/(floorSegment.size()+planes[avg.clusterId].size());
            floorPlane.n.normalize();
            floorSegment << planes[avg.clusterId];
            //qDebug() << "Merged with floor. New rep:" << floorSegmentRep.p << floorSegmentRep.normal << "new dist:" << floorSegmentRep.distance(avg);
        }

        // If that didn't work, a huge cluster can replace a very small one.
        if (floorSegment.size()*20 < planes[avg.clusterId].size())
        {
            floorPlane = avg;
            floorSegment.clear();
            floorSegment << planes[avg.clusterId];
        }
    }

    // Fit a plane to the points in the floor segment.
    if (floorSegment.size() > 2)
    {
        ols.reset();
        for (int i = 0; i < floorSegment.size(); i++)
            ols.addDataPoint(floorSegment[i].p);
        ols.init();
        floorPlane.n = ols.getNormal();
        floorPlane.p.z = ols.evaluateAt(floorPlane.p);
    }

    return floorPlane;
}


// 3
// Flood fill in grid order
// Merge with best (N^2)
void SampleGrid::findAllPlanes()
{
    if (debug > 0)
        qDebug() << "SampleGrid::findAllPlanes():";

    //prune();

    planes.clear();
    planeAvg.clear();

    for (int i = 0; i < samples.size(); i++)
    {
        for (int j = 0; j < samples[i].size(); j++)
        {
            if (debug > 0)
                qDebug() << "Trying" << i << j << samples[i][j] << "in:" << samples[i][j].in;

            if (!samples[i][j].in)
                continue;

            // Start a flood fill at this sample and collect similar samples in the neighbourhood.
            // This procedure will push similar points into the planeCluster.
            planeCluster.clear();
            floodFill(samples[i][j].gridIdx);

            // Ignore clusters that consist only of one point.
            if (planeCluster.size() < config.minimumSegmentSize)
                continue;

            if (debug > 0)
                qDebug() << "plane cluster:" << planeCluster;

            // Compute the average estimate of the cluster.
            Sample avg;
            avg.n.z = 0;
            for (int j = 0; j < planeCluster.size(); j++)
                avg += planeCluster[j];
            avg /= planeCluster.size();

            if (debug > 0)
                qDebug() << "New cluster" << i << j << "points:" << planeCluster.size() << "avg:" << avg;

            // See if you can merge this cluster with any of the clusters so far.
            double minDist = 1000;
            int minIdx = 0;
            for (int k = 0; k < planeAvg.size(); k++)
            {
                double d = planeAvg[k].distance(avg);
                if (d < minDist)
                {
                    minDist = d;
                    minIdx = k;
                }
            }

            if (minDist < config.mergeThreshold)
            {
                // Merge step.
                planeAvg[minIdx].p = (planeAvg[minIdx].p*planes[minIdx].size()+avg.p*planeCluster.size())/(planes[minIdx].size()+planeCluster.size());
                planeAvg[minIdx].n = (planeAvg[minIdx].n*planes[minIdx].size()+avg.n*planeCluster.size())/(planes[minIdx].size()+planeCluster.size());
                planeAvg[minIdx].n.normalize();
                planes[minIdx] << planeCluster;
                if (debug > 0)
                    qDebug() << "Cluster merged with:" << minIdx << planes[minIdx].size() << "avg:" << planeAvg[minIdx];

            }

            // Allocate a new cluster.
            else
            {
                avg.clusterId = planeAvg.size();
                planeAvg << avg;
                planes << planeCluster;
            }
        }
    }
}

// Collects neighbouring samples into the planeCluster vector based on their distance function .
// This is a simple recursive four-neighbour implementation.
void SampleGrid::floodFill(const Vec2u &parentIdx)
{
    Sample& parent = samples[parentIdx.y][parentIdx.x];
    if (!parent.in)
        return;

    parent.in = false;
    planeCluster << parent;

    if (debug > 1)
        qDebug() << "   pushed" << parent.gridIdx << parent;

    if (parent.gridIdx.x > 0)
    {
        Vec2u childIdx = parent.gridIdx - Vec2u(1,0);
        Sample& child = samples[childIdx.y][childIdx.x];
        if (debug > 1)
            qDebug() << "   dist:" << childIdx << parent.distance(child) << "parent:" << parent << "child:" << child;
        if (parent.distance(child) < config.floodThreshold)
            floodFill(childIdx);
    }
    if (parent.gridIdx.x < config.samplesX-1)
    {
        Vec2u childIdx = parent.gridIdx + Vec2u(1,0);
        Sample& child = samples[childIdx.y][childIdx.x];
        if (debug > 1)
            qDebug() << "   dist:" << childIdx << parent.distance(child) << "parent:" << parent << "child:" << child;
        if (parent.distance(child) < config.floodThreshold)
            floodFill(childIdx);
    }
    if (parent.gridIdx.y > 0)
    {
        Vec2u childIdx = parent.gridIdx - Vec2u(0,1);
        Sample& child = samples[childIdx.y][childIdx.x];
        if (debug > 1)
            qDebug() << "   dist:" << childIdx << parent.distance(child) << "parent:" << parent << "child:" << child;
        if (parent.distance(child) < config.floodThreshold)
            floodFill(childIdx);
    }
    if (parent.gridIdx.y < config.samplesY-1)
    {
        Vec2u childIdx = parent.gridIdx + Vec2u(0,1);
        Sample& child = samples[childIdx.y][childIdx.x];
        if (debug > 1)
            qDebug() << "   dist:" << childIdx << parent.distance(child) << "parent:" << parent << "child:" << child;
        if (parent.distance(child) < config.floodThreshold)
            floodFill(childIdx);
    }
}

// Produces the pruned set.
void SampleGrid::prune()
{
    prunedSamples.clear();
    for (int i = 0; i < samples.size(); i++)
    {
        for (int j = 0; j < samples[i].size(); j++)
        {
            if (!samples[i][j].in)
                continue;

            samples[i][j].angle = samples[i][j].n*upVector; // A scalar product-based upright check.
            if (samples[i][j].angle > config.pruneThreshold)
            {
                prunedSamples << samples[i][j];
            }
            else
            {
                samples[i][j].in = false;
            }
        }
    }
}

// Returns true if the sample identified by gridIdx is "in".
bool SampleGrid::isIn(const Vec2u &gridIdx) const
{
    return samples[gridIdx.y][gridIdx.x].in;
}

// Draws the locations of the pruned samples.
void SampleGrid::drawSamples(QPainter *painter) const
{
    int circleSize = 6;

    painter->save();

    // Draw the entire sample set.
    if (true)
    {
        for (int i = 0; i < samples.size(); i++)
        {
            for (int j = 0; j < samples[i].size(); j++)
            {
                if (samples[i][j].p.isNull())
                    painter->setPen(drawUtil.penThin);
                else if (samples[i][j].in)
                    painter->setPen(drawUtil.penGreenThin);
                else
                    painter->setPen(drawUtil.penRedThin);
                painter->drawEllipse(samples[i][j].imagePx.x-circleSize/2, samples[i][j].imagePx.y-circleSize/2, circleSize, circleSize);
            }
        }
    }

    // Draw the pruned sample set in red.
    if (true)
    {
        painter->setPen(drawUtil.penRedThin);
        painter->setBrush(drawUtil.brushRed);
        for (int i = 0; i < prunedSamples.size(); i++)
        {
            if (floorSegment.contains(prunedSamples[i]))
                continue;
            painter->drawEllipse(prunedSamples[i].imagePx.x-circleSize/2, prunedSamples[i].imagePx.y-circleSize/2, circleSize, circleSize);
        }
    }

    // Draw the floor segment samples in green.
    if (true)
    {
        painter->setPen(drawUtil.penGreenThin);
        painter->setBrush(drawUtil.brushGreen);
        for (int i = 0; i < floorSegment.size()-1; i++)
            painter->drawEllipse(floorSegment[i].imagePx.x-circleSize/2, floorSegment[i].imagePx.y-circleSize/2, circleSize, circleSize);
    }

    // Draw the path of expansion.
    if (false)
    {
        painter->setPen(drawUtil.penGreenThin);
        painter->setBrush(drawUtil.brushGreen);
        for (int i = 0; i < floorSegment.size()-1; i++)
            painter->drawLine(floorSegment[i].imagePx.x, floorSegment[i].imagePx.y, floorSegment[i+1].imagePx.x, floorSegment[i+1].imagePx.y);
    }

    // Draw the neighbour lines to highlight the floor.
    if (false)
    {
        painter->setPen(drawUtil.penGreenThin);
        painter->setBrush(drawUtil.brushGreen);
        for (int i = 0; i < floorSegment.size()-1; i++)
        {
            const Vec2u& gridIdx = floorSegment[i].gridIdx;
            //qDebug() << gridIdx << imagePx << samples[gridIdx.y][gridIdx.x+1].imagePx;
            if (gridIdx.x < samples.size()-1 && samples[gridIdx.y][gridIdx.x+1].in)
                painter->drawLine(floorSegment[i].imagePx.x,
                                  floorSegment[i].imagePx.y,
                                  samples[gridIdx.y][gridIdx.x+1].imagePx.x,
                                  samples[gridIdx.y][gridIdx.x+1].imagePx.y);
            if (gridIdx.y < samples[0].size()-1 && samples[gridIdx.y+1][gridIdx.x].in)
                painter->drawLine(floorSegment[i].imagePx.x,
                                  floorSegment[i].imagePx.y,
                                  samples[gridIdx.y+1][gridIdx.x].imagePx.x,
                                  samples[gridIdx.y+1][gridIdx.x].imagePx.y);
        }
    }

    // Draw the sample grid coordinates.
    if (true)
    {
        QFont font = painter->font();
        font.setPointSize(8);
        painter->setFont(font);

        painter->setPen(drawUtil.penWhiteThin);
        for (int i = 0; i < samples.size(); i++)
        {
            const Sample& sam = samples[i][1];
            painter->save();
            painter->translate(QPoint(sam.imagePx.x, sam.imagePx.y));
            painter->drawText(QPoint(QPoint(-5,5)), QString::number(sam.gridIdx.y));
            painter->restore();
        }
        for (int j = 0; j < samples[1].size(); j++)
        {
            const Sample& sam = samples[1][j];
            painter->save();
            painter->translate(QPoint(sam.imagePx.x, sam.imagePx.y));
            painter->drawText(QPoint(QPoint(-5,5)), QString::number(sam.gridIdx.x));
            painter->restore();
        }
    }

    painter->restore();
}

// Draws the undiscarded normals in an OpenGL context.
// It also draws the convex hull of the points that were considered to be floor.
void SampleGrid::drawSamples() const
{
    glLineWidth(2);

    // Draw the normal vectors of all samples.
    if (false)
    {
        for (int i = 0; i < samples.size(); i++)
        {
            for (int j = 0; j < samples[i].size(); j++)
            {
                if (samples[i][j].p.isNull())
                    continue;

                if (samples[i][j].in)
                    glColor3f(0.0, 1.0, 0.0);
                else
                    glColor3f(1.0, 0.0, 0.0);

                QGLViewer::drawArrow(qglviewer::Vec(samples[i][j].p), qglviewer::Vec(samples[i][j].p+samples[i][j].n.normalized(0.07)), 0.004);
            }
        }
    }

    // Draw the normal vectors of the pruned samples.
    if (true)
    {
        glColor3f(1.0, 0.0, 0.0);
        for (int i = 0; i < prunedSamples.size(); i++)
            if (!floorSegment.contains(prunedSamples[i]))
                QGLViewer::drawArrow(qglviewer::Vec(prunedSamples[i].p), qglviewer::Vec(prunedSamples[i].p+prunedSamples[i].n.normalized(0.07)), 0.004);
    }

    // Draw the normals of just the floor segment.
    if (true)
    {
        glColor3f(0.0, 1.0, 0.0);
        for (int i = 0; i < floorSegment.size()-1; i++)
            QGLViewer::drawArrow(qglviewer::Vec(floorSegment[i].p), qglviewer::Vec(floorSegment[i].p+floorSegment[i].n.normalized(0.07)), 0.004);
    }

    // Draw the neighbour lines to highlight the floor.
    if (false)
    {
        glLineWidth(2);
        glColor3f(0.0, 1.0, 0.0);
        for (int i = 0; i < floorSegment.size()-1; i++)
        {
            const Vec2u& gridIdx = floorSegment[i].gridIdx;
            if (gridIdx.x < samples.size()-1 && samples[gridIdx.y][gridIdx.x+1].in)
            {
                glBegin(GL_LINES);
                glVertex3dv(floorSegment[i].p);
                glVertex3dv(samples[gridIdx.y][gridIdx.x+1].p);
                glEnd();
            }
            if (gridIdx.y < samples[0].size()-1 && samples[gridIdx.y+1][gridIdx.x].in)
            {
                glBegin(GL_LINES);
                glVertex3dv(floorSegment[i].p);
                glVertex3dv(samples[gridIdx.y+1][gridIdx.x].p);
                glEnd();
            }
        }
    }

    // Draw the path of expansion.
    if (false)
    {
        glLineWidth(1);
        glColor3f(0.0, 1.0, 0.0);
        for (int i = 0; i < floorSegment.size()-1; i++)
        {
            glBegin(GL_LINES);
            glVertex3dv(floorSegment[i].p);
            glVertex3dv(floorSegment[i+1].p);
            glEnd();
        }
    }

    // Compute the convex hull of the floor segment.
    if (true)
    {
        Polygon pol;
        for (int i = 0; i < floorSegment.size(); i++)
            pol << floorSegment[i].p;
        Polygon ch = pol.convexHull();
        glLineWidth(6);
        glColor3f(0, 0, 1.0);
        glBegin(GL_LINE_LOOP);
        ListIterator<Line> it = ch.edgeIterator();
        while (it.hasNext())
        {
            Vec2 v = it.next().p1();
            glVertex3f(v.x, v.y, floorPlane.evaluateAt(v));
        }
        glEnd();
    }

    // Draw all found planes.
    if (false)
    {
        for (int i = 0; i < planes.size(); i++)
        {
            // Assign a color.
            QColor c = drawUtil.getHeightMapColor(i, 0, planes.size());
            glColor3f(c.redF(), c.greenF(), c.blueF());

            // All contained normals.
            for (int j = 0; j < planes[i].size(); j++)
                QGLViewer::drawArrow(qglviewer::Vec(planes[i][j].p), qglviewer::Vec(planes[i][j].p+planes[i][j].n.normalized(0.07)), 0.004);

            // The convex hull of the plane segment.
            Polygon pol;
            for (int j = 0; j < planes[i].size(); j++)
                pol << planes[i][j].p;
            Polygon ch = pol.convexHull();
            glLineWidth(6);
            glBegin(GL_LINE_LOOP);
            ListIterator<Line> it = ch.edgeIterator();
            while (it.hasNext())
            {
                Vec2 v = it.next().p1();
                glVertex3f(v.x, v.y, planeAvg[i].evaluateAt(v));
            }
            glEnd();

            // The avg normal of the plane segment.
            QGLViewer::drawArrow(qglviewer::Vec(planeAvg[i].p), qglviewer::Vec(planeAvg[i].p+planeAvg[i].n.normalized(0.2)), 0.01);
        }
    }

    // The up vector.
    if (false)
    {
        glPushMatrix();
        glTranslated(0.5, 0, 0.0);
        glColor3f(1.0, 0.0, 0.0);
        QGLViewer::drawArrow(qglviewer::Vec(0,0,0), qglviewer::Vec(upVector.normalized(0.3)), 0.01);
        glPopMatrix();
    }
}

void SampleGrid::drawAllPlanes() const
{
    // Draw the normal vectors of all samples.
    if (true)
    {
        for (int i = 0; i < samples.size(); i++)
        {
            for (int j = 0; j < samples[i].size(); j++)
            {
                if (samples[i][j].p.isNull())
                    continue;

                if (samples[i][j].in)
                    glColor3f(0.0, 1.0, 0.0);
                else
                    glColor3f(1.0, 0.0, 0.0);

                QGLViewer::drawArrow(qglviewer::Vec(samples[i][j].p), qglviewer::Vec(samples[i][j].p+samples[i][j].n.normalized(0.07)), 0.004);
            }
        }
    }

    // Normals that have not been assigned to a segment.
    if (false)
    {
        for (int i = 0; i < samples.size(); i++)
        {
            for (int j = 0; j < samples[i].size(); j++)
            {
                if (samples[i][j].p.isNull())
                    continue;

                if (samples[i][j].clusterId >= 0)
                    continue;

                glColor3f(1.0, 0.0, 0.0);
                QGLViewer::drawArrow(qglviewer::Vec(samples[i][j].p), qglviewer::Vec(samples[i][j].p+samples[i][j].n.normalized(0.07)), 0.004);
            }
        }
    }

    // Draw just the floor segment.
    if (false)
    {
        // All normals.
        glColor3f(0.0, 1.0, 0.0);
        for (int i = 0; i < floorSegment.size()-1; i++)
            QGLViewer::drawArrow(qglviewer::Vec(floorSegment[i].p), qglviewer::Vec(floorSegment[i].p+floorSegment[i].n.normalized(0.07)), 0.004);

        // The convex hull.
        Polygon pol;
        for (int i = 0; i < floorSegment.size(); i++)
            pol << floorSegment[i].p;
        Polygon ch = pol.convexHull();
        glLineWidth(6);
        glColor3f(0, 1.0, 0);
        glBegin(GL_LINE_LOOP);
        ListIterator<Line> it = ch.edgeIterator();
        while (it.hasNext())
        {
            Vec2 v = it.next().p1();
            glVertex3f(v.x, v.y, floorPlane.evaluateAt(v));
        }
        glEnd();

        // The avg normal of the floor segment.
        QGLViewer::drawArrow(qglviewer::Vec(floorPlane.p), qglviewer::Vec(floorPlane.p+floorPlane.n.normalized(0.2)), 0.01);

    }
}

int SampleGrid::getDebug() const
{
    return debug;
}

void SampleGrid::setDebug(int value)
{
    debug = value;
}

QDebug operator<<(QDebug dbg, const SampleGrid &o)
{
    //dbg << "r:" << o.roll << "p:" << o.pitch << "y:" << o.yaw << "x:" << o.x << "y:" << o.y << "z:" << o.z;
    return dbg;
}

QDebug operator<<(QDebug dbg, const Sample &o)
{
    dbg << "p:" << o.p << "n:" << o.n;
    return dbg;
}


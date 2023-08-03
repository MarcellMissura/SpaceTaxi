#include "GeometricMap.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"
#include "blackboard/State.h"
#include "lib/util/ColorUtil.h"
#include "lib/util/GLlib.h"
#include "lib/util/Statistics.h"
#include "lib/util/StopWatch.h"
#include "robotcontrol/slam/GraphConstraint.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

// This is a geometric map class where the map is made of polygons, lines, and a pose graph.
// The pose graph serves as the spine of the entire map and its main purpose is to
// aid the loop closing. Whenever the closest node of the graph is farther away from
// the robot than a certain threshold, a new node is added to the graph creating a
// dangling path while exploring. When a loop closing situation is detected, a graph
// optimization algorithm distributes the loop closing error over the entire dangling
// path back to the root of the path and the loop is closed in the graph by connecting
// the leaf node with the root node. The lines and polygons that make up the map are
// attached to the pose graph nodes and selecting the closest node helps with gathering
// the currently relevant part of the map.
// The lines are landmarks only used for localization. Lines are being added, updated
// and erased from the map as the robot moves along.
// The polygons are the skin of the map. They are needed to demark free space and
// blocked space without any gaps so that path planning and motion planning can work.
// There is at least one large freespace polygon surrounding the map and multiple blocked
// space polygons are inside of it demarking free and occupied space with a two level
// hierarchy. Most of the functions on the GeometricMap class are private and are used
// for the internal workings of the map maintenance. The main public interface of the
// GeometricMap is the slam() function through which it receives the input needed to
// perform the simultaneous localization and mapping.

GeometricMap::GeometricMap()
{
    globalSnapQuality = 0;
    closestPoseGraphNode = 0;
    closestPoseGraphNodeBefore = 0;
}

// Clears the LineMap to a blank state.
void GeometricMap::clear()
{
    inputPose.setZero();
    localSnappedPose.setZero();
    inputLines.clear();
    mapLines.clear();
    confirmedLinePairs.clear();
    unconfirmedInputLines.clear();
    closestPoseGraphNode = 0;
    closestPoseGraphNodeBefore = 0;
    poseGraphNodes.clear();
}

// Returns true when there are no lines in the map.
bool GeometricMap::isEmpty() const
{
    return mapLines.isEmpty();
}

// Adds a line to the map, if the line is long enough.
void GeometricMap::addLine(const TrackedLine &o)
{
    if (o.length() < config.laserMinLineLength)
        return;

    uint id = 0;
    if (!mapLines.isEmpty())
        id = mapLines.last().id+1;
    mapLines << o;
    mapLines.last().id = id;
}

// The main slam function that drives the localization and map building. It returns a Pose2D that is
// localized in the map and also updates the map using the inputLines and the visibility polygon.
// The inputPose should be a good guess to the current pose in the map. The inputLines and the visibility
// polygon are expected in local coordinates relative to inputPose. In this process, the pose graph is grown,
// the line map is updated by adding new lines and moving already known map lines towards new observations,
// and the polygons are grown as the robot drives into unexplored areas. Lines and polygons are erased with
// the help of the visibility polygon, if they no longer exist.
Pose2D GeometricMap::slam(const Pose2D &inputPose, const Vector<TrackedLine> &inputLines, const Polygon &visPol)
{
    this->inputPose = inputPose;
    this->inputLines = inputLines;

    // If the map is empty, initialize the map with the currently observed lines.
    if (isEmpty())
    {
        for (uint i = 0; i < inputLines.length(); i++)
            addLine(inputLines[i]+inputPose);
        ListIterator<TrackedLine> mapLineIterator = mapLines.begin();
        while (mapLineIterator.hasNext())
            activeMapLines << &mapLineIterator.next();
        addPoseGraphNode(inputPose, activeMapLines);
        updatePolygonMap(inputPose, visPol);
        return inputPose;
    }

    if (state.frameId == 7078)
        state.stop = true;

    // Procedure:
    // Gather active map lines
    // snap -> medium pose, local pose, confirmed lines, unconfirmed lines, observers
    // update map lines -> seen lines
    // unify active map lines and seen lines
    // update pose graph
    // update polygons
    // loop close -> observer lines
    // merge, expire, erase


    // Gather the map lines near the input pose from the pose graph.
    // These active map lines are used for snapping, merging, expiring.
    // The "active" flag of the map lines are used for global snapping,
    // so that the non-active lines can be selected faster.
    gatherActiveMapLines(inputPose);

    // Use the global snap function to try and localize in the entire map.
    // This is needed to initially find our pose in the map when the robot is switched on.
    if (command.globalLocalization)
        globalSnappedPose = snapGlobal(this->inputLines, config.debugLevel > 10);

    // The medium snap is used as an indicator for loop closing. The snapMedium() function
    // also produces a mediumSnapQuality indicator, confirmedLinePairs, and observer poses.
    mediumSnappedPose = snapMedium(this->inputLines, inputPose, config.debugLevel > 10);

    // Use the local snap function to compute a snappedPose that aligns the input lines with the map
    // lines as good as it can. After this function, the confirmedLinePairs and unconfirmedInputLines
    // will be filled with data as needed for updating the map.
    localSnappedPose = snapLocal(this->inputLines, inputPose, config.debugLevel > 10);

    // Detect bad snaps and return in order to avoid messing up the map.
    if (localSnappedPose == inputPose)
    {
        qDebug() << state.frameId << "Bad snap detected!";
        //qDebug() << "input pose:" << inputPose;
        //qDebug() << "input lines:" << inputLines;
        //localSnappedPose = snapLocal(this->inputLines, inputPose, true);

        if (state.frameId > 4550 && state.frameId < state.frameId < 4600)
            return savedSnappedPose; // hack for the bad frames in the dataset

        return localSnappedPose;
    }
    savedSnappedPose = localSnappedPose;

    // Return if the map update is manually disabled.
    if (!command.mapUpdateEnabled)
        return localSnappedPose;

    // Update the line map by adding newly observed lines and updating observed lines.
    // A Vector of seen lines is returned that have been observed by the sensor in this iteration.
    LinkedList<TrackedLine*> seenLines = updateMapLines(config.debugLevel > 10);
    activeMapLines.unify(seenLines);

    // Update the pose graph.
    updatePoseGraph(localSnappedPose, seenLines, config.debugLevel > 10);

    // Update the polygon map.
    updatePolygonMap(localSnappedPose, visPol, config.debugLevel > 0);

    // Loop closing.
    // Loop closing is detected when the medium snap successfully managed to snap
    // near the local snap without using the active lines.
    if (!mediumSnappedPose.isNull() && mediumSnapQuality > 0.95)
    {
        qDebug() << state.frameId << "Loop closing event detected. quality:" << mediumSnapQuality << "norm:" << (mediumSnappedPose-localSnappedPose).norm()
                 << "medium pose:" << mediumSnappedPose << "local pose:" << localSnappedPose;

        // Perform the loop-closing optimization.
        closeLoop(localSnappedPose, mediumSnappedPose);

        // Correct the last pose of the robot.
        localSnappedPose = mediumSnappedPose;
        this->inputPose = mediumSnappedPose;

        // Gather more active lines.
        activeMapLines.unify(closestPoseGraphNode->gatherNearbyLines(config.slamPoseGraphNearbyNodes));
    }

    // Map line maintenance. Merge, expire and erase.
    mergeMapLines(config.debugLevel > 10);
    expireMapLines(config.debugLevel > 10);
    eraseMapLines(localSnappedPose, visPol, config.debugLevel > 10);

    return localSnappedPose;
}

// Prints the amount of memory used by the map in kilobytes.
void GeometricMap::printMemoryUsage() const
{
    qDebug() << "Map lines:" << (double(sizeof(TrackedLine) * mapLines.size()) / 1024) << "kb" << mapLines.size();
    qDebug() << "Graph Nodes:" << (double(sizeof(PoseGraphNode) * poseGraphNodes.size()) / 1024) << "kb" << poseGraphNodes.size();
    ListIterator<TrackedLine> it = mapLines.begin();
    uint observerCount = 0;
    while (it.hasNext())
        observerCount += it.next().observerNodes.size();
    qDebug() << "Observer connections" << (double(sizeof(PoseGraphNode*) * observerCount) / 1024) << "kb" << observerCount;
    ListIterator<PoseGraphNode> it2 = poseGraphNodes.begin();
    uint seenLineCount = 0;
    while (it2.hasNext())
        seenLineCount += it2.next().seenMapLines.size();
    qDebug() << "Seen line connections" << (double(sizeof(TrackedLine*) * seenLineCount) / 1024) << "kb" << seenLineCount;
    qDebug() << "Polygon vertices:" << sizeof(double)*polygonMap.getVertexCount() / 1024;
}

//
void GeometricMap::exportMap() const
{
    qDebug() << "Map lines:";
    qDebug() << mapLines;
    qDebug() << "Map Polygons:";
    qDebug() << polygonMap;
    qDebug() << "Graph Nodes:";
    qDebug() << poseGraphNodes;
}

// The set of active map lines is gathered from the graph nodes near the agent.
// When building this set, we also mark map lines as active or inactive for easier
// processing later.
void GeometricMap::gatherActiveMapLines(const Pose2D& pose)
{
    ListIterator<TrackedLine*> it = activeMapLines.begin();
    while (it.hasNext())
        it.next()->active = false;
    activeMapLines = getClosestNode(pose)->gatherNearbyLines(config.slamPoseGraphNearbyNodes);
    it = activeMapLines.begin();
    while (it.hasNext())
        it.next()->active = true;
}

// Gathers pointers to the map lines that have been seen recently.
LinkedList<TrackedLine*> GeometricMap::gatherRecentMapLines() const
{
    LinkedList<TrackedLine*> recentMapLines; // A set of recently seen map lines.
    ListIterator<TrackedLine> it = mapLines.begin();
    while (it.hasNext())
    {
        TrackedLine* l = &it.next();
        //qDebug() << state.frameId << *l << (state.frameId - l->lastSeen);
        if (state.frameId - l->lastSeen < 100)
            recentMapLines << l;
    }
    return recentMapLines;
}

// Returns pointers to the map lines that collide with the given box.
LinkedList<TrackedLine *> GeometricMap::boxSelectMapLines(const Box &box) const
{
    LinkedList<TrackedLine*> intersectingMapLines;
    ListIterator<TrackedLine> it = mapLines.begin();
    while (it.hasNext())
    {
        TrackedLine* l = &it.next();
        if (box.intersects(*l))
            intersectingMapLines << l;
    }
    return intersectingMapLines;
}

// It returns a Pose2D that is localized in the map, given an inputPose to start the snap from.
// The inputPose should be a good initial guess. The last pose is always a good initial guess.
// The inputLines are expected in local coordinates relative to the inputPose, as sensed by the
// laser sensor. The returned Pose2D is a corrected pose in world coordinates that replaces the
// inputPose. After this function, the confirmedLinePairs and the unconfirmedInputLines will
// be filled with data.
Pose2D GeometricMap::snapLocal(Vector<TrackedLine> &inputLines, const Pose2D &inputPose, bool debug)
{
    // Local snapping happens like so:
    // 1. Compute strict pairs between input lines and map lines.
    // 2. Compute the rotation consensus set and discard the outliers.
    // 3. Settle on the average rotation of the consenus set.
    // 4. Compute translation hypotheses from pairs of the remaining pairs.
    // 5. If only parallel pairs are present.
    // 6.    Compute the translation consensus set to discard outliers.
    // 7.    Settle on the average of the translation consensus set (requires odometry).
    // 8. If fully qualified pairs are present.
    // 9.    Compute the hypothesis consensus set and discard outliers.
    // 10.   Settle on the average translation of the hypothesis consensus set.
    // 11. Determine the confirmed line pairs and the unconfirmed input lines.

    // Explanation:
    // 1. We pair input lines with map lines using a tight threshold on the line-pose-distance between
    // the input line and the map line. For all input lines that can be paired to at least one map line
    // within the line-pose-distance threshold, we only keep the closest one. As a result, we obtain a
    // few well filtered line pairs. TODO: It is still worth trying to keep more than just the nearest map line.
    // 2. Then, the line pairs are pruned to a consensus set with respect to rotation. In the local snap
    // setting, where all input lines are close to their paired map line, we can assume a unimodal
    // distribution of the rotation with perhaps a few outliers. The consensus set essentially discards
    // these outliers.
    // 3. Now we can settle on the average rotation suggested by the line pairs remaining in the rotation
    // consensus set. This set includes all the parallel lines the robot is seeing that are very useful
    // for finding the correct rotation. We rotate the input pose to the determined rotation and use the
    // corrected input pose for all subsequent computations.
    // 4. Then, we build pairs of the line pairs that remained in the rotation consensus set in order to
    // acquire a set of hypotheses of what the translation of the entire scan could be. Since one line pair
    // only gives information about the rotation and the orthogonal translation with respect to the map line,
    // but leaves uncertainty about the translation along the map line, two pairs are needed to determine all
    // parameters of the transformation (x,y,theta). In the local snap setting we already settled on the
    // rotation theta, so we only compute the translation (x,y). We discard pairs of line pairs (hypotheses)
    // that don't manage to overlap their input lines with the respective map lines by at least 75% after the
    // translation.
    // 5. We call a pair of line pairs (a hypothesis) parallel when the map lines of both involved pairs are
    // (almost) parallel. When all input lines are parallel, then all hypotheses turn out to be parallel as well.
    // This is a special case where it is impossible to determine the correct translation. We can match the input
    // lines with their map lines by translating orthogonally to the map lines, but we are left with uncertainty
    // in the direction along the map lines that can only be taken from odometry. Odometry is not very precise,
    // but it is better than nothing.
    // 6. When all hypotheses are parallel, we compute the translation consensus set from them and rule out
    // outliers with respect to the orthogonal distance between the input lines and the respective map lines.
    // In the local snap setting, we are expecing a unimodal distribution.
    // 7. From the remaining translation consensus set, we compute the average translation and correct the input
    // pose by translating it by that amount. The odometry translation is already contained in the input pose.
    // 8. If fully qualified hypotheses are present, i.e. not all hypotheses are parallel, we can determine the
    // full translation for a snap. In this case, we disregard all parallel hypotheses and focus only on the fully
    // qualified ones.
    // 9. From the fully qualified hypotheses, we compute the hypothesis consensus set and discard outliers.
    // In the local snap setting, we are expecing a unimodal distribution so we don't need to perform cluster analysis.
    // 10. We compute the average translation of the fully qualified hypotheses and correct the input pose with it.
    // 11. After figuring out the local snap, we build the confirmed line pairs and the unconfirmed input lines data
    // structures for further processing by the updateMapLines() function, which essentially updates the line map.

    localSnappedPose = inputPose;
    if (isEmpty())
        return localSnappedPose;

    if (debug)
        qDebug() << state.frameId << "Local snapping pose" << inputPose;

    // 1. Build the closest pairs between input lines and map lines.
    // The line pairs come with weights that are computed based on the lengths of the involved lines.
    // The inputPose is written into all linePairs.
    Vector<LinePair> linePairs = computeNearestLinePairs(inputLines, inputPose);
    if (linePairs.isEmpty())
    {
        qDebug() << state.frameId << "No local pairs found!";
        return localSnappedPose;
    }

    if (debug)
    {
        qDebug() << state.frameId << "All local line pairs:" << linePairs.size();
        qDebug() << linePairs;
    }

    // 2. Extract the largest consensus set from the line pairs with respect to rotation.
    // This is to remove outliers. In the local snap setting where all input lines are
    // close to their paired map line, we can assume a unimodal distribution of the
    // rotation with perhaps a few outliers.
    linePairs = rotationConsensus(linePairs, config.slamClusteringAngleEps, debug);

    if (debug)
    {
        qDebug() << state.frameId << "Rotation angle sample consensus:";
        qDebug() << linePairs;
    }

    // 3. Compute the weighted average rotation of the consensus set and let that be the rotation of our snap transform.
    double avgRotation = 0;
    double totalWeight = 0;
    for (uint i = 0; i < linePairs.size(); i++)
    {
        avgRotation += linePairs[i].weight()*linePairs[i].angleDiff();
        totalWeight += linePairs[i].weight();
    }
    if (totalWeight > 0)
        avgRotation /= totalWeight;
    localSnappedPose.turn(avgRotation);

    // Update the rotation of the line pairs in the consensus set.
    for (uint i = 0; i < linePairs.size(); i++)
        linePairs[i].inputPose = localSnappedPose;



    // 4. Build pairs of line pairs to acquire a set of hypotheses of what the transformation could be.
    // The hypotheses are split into two sets: proper hypotheses and parallel hypotheses.
    // Sometimes, only parallel hypotheses are present, e.g. when driving along a corridor, and
    // localization is only possible with the help of odometry.
    Vector<TransformationHypothesis> properHypothesisSet; // fully qualified
    Vector<TransformationHypothesis> parallelHypothesisSet;
    TransformationHypothesis th;
    for (uint i = 0; i < linePairs.size(); i++)
    {
        for (uint j = i+1; j < linePairs.size(); j++)
        {
            LinePair& lpi = linePairs[i];
            LinePair& lpj = linePairs[j];

            // Compute a translation hypothesis from line pairs i and j.
            if (th.computeTranslation(lpi, lpj, debug)) // returns false if the pair is invalid
            {
                // In a local snap, the transformation cannot be very large.
                if (th.tr().norm() > config.slamMaxPoseDiff)
                {
                    if (debug)
                        qDebug() << "Discarding hyp" << lpi.inputLine->id << lpi.mapLine->id << "and" << lpj.inputLine->id << lpj.mapLine->id << "because of pose diff." << th.tr().norm();
                    continue;
                }

                // Valid hypothesis found.
                if (th.isParallel)
                {
                    th.id = parallelHypothesisSet.size()+100;
                    th.linePair1 = &lpi;
                    th.linePair2 = &lpj;
                    parallelHypothesisSet << th;
                }
                else
                {
                    th.id = properHypothesisSet.size();
                    th.linePair1 = &lpi;
                    th.linePair2 = &lpj;
                    properHypothesisSet << th;
                }

                if (debug)
                {
                    LinePair lpii = lpi;
                    LinePair lpjj = lpj;
                    lpii.inputPose = th.tr()+localSnappedPose;
                    lpjj.inputPose = th.tr()+localSnappedPose;
                    qDebug() << th.id << "Pairs" << lpi.inputLine->id << lpi.mapLine->id << "and" << lpj.inputLine->id << lpj.mapLine->id
                             << "suggest" << th.tr() << "norm:" << th.tr().norm() << "weight:" << th.weight << th.linePair1->mapLine->totalWeight << th.linePair2->mapLine->totalWeight
                             //<< "angDiff:" << fabs(pihalfcut(lpi.mapLine->ang - lpj.mapLine->ang))
                             //<< "overlap:" << lpii.percentualOverlap() << lpjj.percentualOverlap()
                             << "lld:" << lpii.lineLineDist() << lpjj.lineLineDist() << "parallel:" << th.isParallel;
                }
            }
        }
    }

    if (properHypothesisSet.isEmpty() && parallelHypothesisSet.isEmpty())
    {
        qDebug() << state.frameId << "No local hypotheses found!";
        return inputPose; // hack to solve an artifical problem with a data set.
        //return localSnappedPose;
    }


    // 5. If only parallel pairs are present.
    if (properHypothesisSet.isEmpty() && !parallelHypothesisSet.isEmpty())
    {
        // Parallel snap case.

        // 6. Compute the translation consensus set to discard outliers.
        // Extract the largest consensus set from the line pairs with respect to translation.
        linePairs = translationConsensus(linePairs, config.slamClusteringOrthoEps);

        if (false && debug)
        {
            qDebug() << state.frameId << "Translation consensus:";
            qDebug() << linePairs;
        }

        // 7. Settle on the average of the translation consensus set (requires odometry).
        // Compute the weighted average translation of the remaining line pairs.
        // The weighted average computes robustly even if all lines are parallel.
        // However, one degree of freedom remains unknown. Odometry is needed in such cases.
        Vec2 avgTranslation;
        totalWeight = 0;
        for (uint i = 0; i < linePairs.size(); i++)
        {
            linePairs[i].inputPose = localSnappedPose;
            linePairs[i].tr().pos();
            avgTranslation += -linePairs[i].weight()*linePairs[i].ortho() * linePairs[i].mapLine->normal();
            totalWeight += linePairs[i].weight();
        }
        if (totalWeight > 0)
            avgTranslation /= totalWeight;
        localSnappedPose.translate(avgTranslation);

        if (debug)
            qDebug() << "Avg transform:" << localSnappedPose << "local:" << localSnappedPose - inputPose << "rot:" << avgRotation << "trans:" << avgTranslation;


        // 11. Build a set of confirmed line pairs.
        confirmedLinePairs.clear();
        for (uint i = 0; i < linePairs.size(); i++)
        {
            confirmedLinePairs << linePairs[i];
            confirmedLinePairs.last().inputPose = localSnappedPose;
            confirmedLinePairs.last().inputLine->active = true;
        }

        if (false && debug)
            qDebug() << state.frameId << "Confirmed line pairs:" << confirmedLinePairs;
    }

    // 8. If fully qualified pairs are present.
    else
    {
        // 9. Compute the hypothesis consensus set and discard outliers.
        // Compute the consensus of the set of "proper" hypotheses.
        Vector<TransformationHypothesis> hypothesisConsensusSet = hypConsensus(properHypothesisSet, config.slamClusteringTransformEps);
        if (false && debug)
        {
            qDebug() << state.frameId << "Hypothesis consensus set:";
            qDebug() << hypothesisConsensusSet;
        }

        // 10. Settle on the average translation of the hypothesis consensus set.
        // Compute the weighted average transformation of the consensus set.
        double totalWeight = 0;
        Pose2D avgTransform;
        for (uint i = 0; i < hypothesisConsensusSet.size(); i++)
        {
            TransformationHypothesis& t = hypothesisConsensusSet[i];
            Pose2D tr = t.tr();
            avgTransform.x += t.weight*tr.x;
            avgTransform.y += t.weight*tr.y;
            avgTransform.z += t.weight*tr.z; // should be 0
            totalWeight += t.weight;
        }
        avgTransform /= totalWeight;
        localSnappedPose = avgTransform+localSnappedPose;

        if (debug)
            qDebug() << "Weighted avg transform:" << avgTransform << localSnappedPose;


        // 11. Build a set of confirmed line pairs that appear in the consensus set.
        confirmedLinePairs.clear();
        for (uint i = 0; i < hypothesisConsensusSet.size(); i++)
        {
            if (!hypothesisConsensusSet[i].linePair1->confirmed)
            {
                confirmedLinePairs << *hypothesisConsensusSet[i].linePair1;
                confirmedLinePairs.last().inputPose = localSnappedPose;
                confirmedLinePairs.last().inputLine->active = true;
            }
            if (!hypothesisConsensusSet[i].linePair2->confirmed)
            {
                confirmedLinePairs << *hypothesisConsensusSet[i].linePair2;
                confirmedLinePairs.last().inputPose = localSnappedPose;
                confirmedLinePairs.last().inputLine->active = true;
            }
            hypothesisConsensusSet[i].linePair1->confirmed = true;
            hypothesisConsensusSet[i].linePair2->confirmed = true;
        }
        if (false && debug)
            qDebug() << state.frameId << "Confirmed line pairs:" << confirmedLinePairs;
    }


    // 4. Build the set of unconfirmed input lines.
    unconfirmedInputLines.clear();
    for (uint i = 0; i < inputLines.size(); i++)
    if (!inputLines[i].active)
        unconfirmedInputLines << inputLines[i] + localSnappedPose;

    if (false && debug)
        qDebug() << state.frameId << "Unconfirmed input lines:" << unconfirmedInputLines;


    if (debug)
        qDebug() << "Returning snapped pose:" << localSnappedPose  << "diff:" << localSnappedPose-inputPose << (localSnappedPose-inputPose).norm();

    return localSnappedPose;
}

// Attempts global localization in the map and returns a Pose2D that is localized in the map.
// Global localization fails when all input lines are parallel, e.g. when driving along a straight
// corridor without any features. The function returns a zero pose when the global localization fails.
// The inputLines are expected in local coordinates. The returned Pose2D is a pose in world coordintes.
Pose2D GeometricMap::snapGlobal(Vector<TrackedLine> &inputLines, bool debug)
{
    // Global snapping happens like so:
    // 1. We pair input lines with map lines. The pairs are made from all combinations of input lines
    // and map lines. Only few pairs can be discarded based on length considerations.
    // 2. We build pairs of line pairs to acquire a set of hypotheses of what the transformation of
    // the entire scan could be. Since one input - map line pair only gives information about the rotation
    // and then the orthogonal distance between the lines, but leaves uncertainty along the map line, two
    // pairs are needed to determine all parameters of the transformation (x,y,theta).
    // 3. We compute a final transformation from the consensus set of the computed hypotheses.
    // 4. We build a set of confirmed line pairs from the input and map lines that could be matched.
    // 5. We assess the quality of the global snap by the confirmed input length over the total input length.

    globalSnappedPose = Pose2D();
    if (isEmpty())
        return globalSnappedPose;

    if (debug)
        qDebug() << state.frameId << "Global snapping.";

    // 1. Build pairs between input lines and map lines.
    // The pairs are made from all combinations of input lines and map lines.
    // Only few pairs can be discarded based on length considerations.
    // The line pairs come with weights that are computed based on the lengths of the involved lines.
    // The inputPose of all linePairs remains zero.
    Vector<LinePair> linePairs = computeAllLinePairs(inputLines);
    if (linePairs.isEmpty())
    {
        if (debug)
            qDebug() << state.frameId << "No global pairs found!";
        return globalSnappedPose;
    }

    if (debug)
    {
        //qDebug() << state.frameId << "All global line pairs:" << linePairs.size();
        //qDebug() << linePairs;
    }


    // 2. Build pairs of line pairs to acquire a set of hypotheses of what the transformation
    // of the globalSnapPose could be. In this process, the hypotheses are split into two sets:
    // parallel hypotheses and proper hypotheses. For global snapping, all parallel hypotheses
    // are discarded.
    Vector<TransformationHypothesis> properHypothesisSet;
    TransformationHypothesis th;
    for (uint i = 0; i < linePairs.size(); i++)
    {
        for (uint j = i+1; j < linePairs.size(); j++)
        {
            LinePair& lpi = linePairs[i];
            LinePair& lpj = linePairs[j];

            // Compute the transformation hypothesis from line pairs i and j.
            if (th.computeTransform(lpi, lpj)) // returns false if the pair is invalid
            {
                if (!th.isParallel)
                {
                    // Valid hypothesis found.
                    th.id = properHypothesisSet.size();
                    th.linePair1 = &lpi;
                    th.linePair2 = &lpj;
                    properHypothesisSet << th;

                    if (false && debug)
                    {
                        LinePair lpii = lpi;
                        LinePair lpjj = lpj;
                        lpii.inputPose = th.tr();
                        lpjj.inputPose = th.tr();
                        qDebug() << th.id << "Pairs" << lpi.inputLine->id << lpi.mapLine->id << "and" << lpj.inputLine->id << lpj.mapLine->id
                                 << "suggest" << th.tr() << "norm:" << th.tr().norm() << "weight:" << th.weight
                                 //<< "angDiff:" << fabs(pihalfcut(lpi.mapLine->ang - lpj.mapLine->ang))
                                 //<< "overlap:" << lpii.percentualOverlap() << lpjj.percentualOverlap()
                                 << "lld:" << lpii.lineLineDist() << lpjj.lineLineDist() << "parallel:" << th.isParallel;
                    }
                }
            }
        }
    }

    if (properHypothesisSet.isEmpty())
    {
        // This happens quite a lot when all input lines are parallel.
        // No global snapping is possible.

        if (debug)
            qDebug() << state.frameId << "No proper global hypothesis set found!";
        return globalSnappedPose; // Should be zero.
    }

    // 3. Extract the largest consensus set from the hypothesis set.
    // This is a great way to discard outliers, i.e. small sets of single or few transform hypotheses
    // that don't result in a good match.
    Vector<TransformationHypothesis> hypothesisConsensusSet = hypConsensus(properHypothesisSet, config.slamClusteringTransformEps);
    if (debug)
    {
        qDebug() << state.frameId << "Hypothesis consensus set:";
        qDebug() << hypothesisConsensusSet;
    }

    // Special case: if no hypothesis received any votes, the global snapping failed.
    bool allZeroVotes = true;
    for (uint i = 0; i < hypothesisConsensusSet.size(); i++)
    {
        if (hypothesisConsensusSet[i].votes > 0)
        {
            allZeroVotes = false;
            break;
        }
    }
    if (allZeroVotes)
    {
        if (debug)
            qDebug() << state.frameId << "No hypothesis received any votes. Global snap failed.";
        return globalSnappedPose;
    }

    // The consensus set can still contain multiple clusters of hypotheses, i.e. it is multi-modal.
    // Perform a cluster analysis and pick the largest one with the most votes.
    Vector<Vector<TransformationHypothesis> > hypsClusters = hypClusters(hypothesisConsensusSet, config.slamClusteringTransformEps);
    if (hypsClusters.size() > 1)
    {
        //qDebug() << state.frameId << "Multiple hyp clusters found." << hypsClusters;

        // Count the votes in each cluster.
        Vector<uint> totalVotesInClusters;
        for (uint i = 0; i < hypsClusters.size(); i++)
        {
            uint totalVotesInCluster = 0;
            for (uint j = 0; j < hypsClusters[i].size(); j++)
                totalVotesInCluster += hypsClusters[i][j].votes;
            totalVotesInClusters << totalVotesInCluster;
        }

        // See where the most votes are.
        uint maxVotesIndex = 0;
        uint maxVotes = 0;
        for (uint i = 0; i < totalVotesInClusters.size(); i++)
        {
            if (totalVotesInClusters[i] > maxVotes)
            {
                maxVotesIndex = i;
                maxVotes = totalVotesInClusters[i];
            }
        }

        // Test if we can find one unique cluster with the most votes.
        bool unimodal = true;
        for (uint i = 0; i < totalVotesInClusters.size(); i++)
        {
            if (totalVotesInClusters[i] >= maxVotes && i != maxVotesIndex)
            {
                unimodal = false;
                break;
            }
        }

        if (unimodal)
        {
            // One cluster is clearly the winner. That means it managed to match the most lines and
            // is certainly the best choice.
            hypothesisConsensusSet = hypsClusters[maxVotesIndex];
            //qDebug() << "Cluster votes:" << totalVotesInClusters << "max idx:" << maxVotesIndex << "unimodal:" << unimodal;
            if (debug)
                qDebug() << state.frameId << "Consensus ambiguity successfully resolved. votes:" << &totalVotesInClusters;
        }
        else
        {
            // This is problematic. There are multiple equally bad hypotheses. In most cases, this
            // happens when no good match could be made, only a bunch of partial matches. We consider
            // this a failed match.
            if (debug)
                qDebug() << state.frameId << "Consensus ambiguity could not be resolved. votes:" << &totalVotesInClusters;
            return globalSnappedPose;
        }
    }


    // 3. Compute the weighted average transformation of the consensus set and let that be our snap transform.
    double totalWeight = 0;
    Pose2D avgTransform;
    for (uint i = 0; i < hypothesisConsensusSet.size(); i++)
    {
        TransformationHypothesis& t = hypothesisConsensusSet[i];
        Pose2D tr = t.tr();
        avgTransform.x += t.weight*tr.x;
        avgTransform.y += t.weight*tr.y;
        avgTransform.z += t.weight*tr.z;
        totalWeight += t.weight;
    }
    avgTransform /= totalWeight;
    globalSnappedPose = avgTransform;

    if (debug)
        qDebug() << state.frameId << "Weighted avg transform:" << avgTransform;


    // 4. Build a set of confirmed line pairs that appear in the consensus set.
    confirmedLinePairs.clear();
    for (uint i = 0; i < hypothesisConsensusSet.size(); i++)
    {
        if (!hypothesisConsensusSet[i].linePair1->confirmed)
        {
            confirmedLinePairs << *hypothesisConsensusSet[i].linePair1;
            confirmedLinePairs.last().inputPose = globalSnappedPose;
        }
        if (!hypothesisConsensusSet[i].linePair2->confirmed)
        {
            confirmedLinePairs << *hypothesisConsensusSet[i].linePair2;
            confirmedLinePairs.last().inputPose = globalSnappedPose;
        }
        hypothesisConsensusSet[i].linePair1->confirmed = true;
        hypothesisConsensusSet[i].linePair2->confirmed = true;
    }
    if (debug)
    {
        qDebug() << state.frameId << "Input lines:" << inputLines;
        qDebug() << state.frameId << "Confirmed global line pairs:" << confirmedLinePairs;
    }


    // 5. Evaluate the quality of the global snap.
    // The snap quality is expressed as successfully overlapped input length divided by the total input length.
    // The snap quality approaches 1 if most input lines were part of a hypothesis in the consensus set.
    double totalInputLength = 0;
    for (uint i = 0; i < inputLines.size(); i++)
        totalInputLength += inputLines[i].length();
    double confirmedLength = 0;
    for (uint i = 0; i < confirmedLinePairs.size(); i++)
        confirmedLength += confirmedLinePairs[i].overlap();
    globalSnapQuality = confirmedLength/totalInputLength;

    if (debug)
        qDebug() << state.frameId << "Global snap quality:" << globalSnapQuality
                 << "total length:" << totalInputLength
                 << "confirmed length:" << confirmedLength;

    // We only consider highly reliable global snaps with at least 75% confirmed overlap.
    if (globalSnapQuality < 0.75)
    {
        // Global snap failed.
        globalSnappedPose = Pose2D();
        return globalSnappedPose;
    }

    if (debug)
        qDebug() << "Returning global snap:" << globalSnappedPose << "quality:" << globalSnapQuality;

    return globalSnappedPose;
}

// Attempts medium range localization in the map and returns a Pose2D that is localized in the map.
// Medium range localization fails when all input lines are parallel, e.g. when driving along a straight
// corridor without any features. The function returns a zero pose when the localization fails.
// The inputLines are expected in local coordinates. The returned Pose2D is a pose in world coordintes.
Pose2D GeometricMap::snapMedium(Vector<TrackedLine> &inputLines, const Pose2D &inputPose, bool debug)
{
    // Medium snapping happens exactly the same as the global snap, except we preselect map lines
    // with a box constraint.
    // 1. We pair input lines with map lines. The pairs are made from all combinations of input lines
    // and map lines. Only few pairs can be discarded based on length considerations.
    // 2. We build pairs of line pairs to acquire a set of hypotheses of what the transformation of
    // the entire scan could be. Since one input - map line pair only gives information about the rotation
    // and then the orthogonal distance between the lines, but leaves uncertainty along the map line, two
    // pairs are needed to determine all parameters of the transformation (x,y,theta).
    // 3. We compute a final transformation from the consensus set of the computed hypotheses.
    // 4. We build a set of confirmed line pairs from the input and map lines that could be matched.
    // 5. We assess the quality of the snap by the confirmed input length over the total input length.

    mediumSnappedPose = Pose2D();
    if (isEmpty())
        return mediumSnappedPose;

    if (debug)
        qDebug() << state.frameId << "Medium snapping.";

    // 1. Build pairs between input lines and map lines.
    // The pairs are made from all combinations of input lines and map lines.
    // Only few pairs can be discarded based on length considerations.
    // The line pairs come with weights that are computed based on the lengths of the involved lines.
    // The inputPose of all linePairs remains zero.
    Vector<LinePair> linePairs = computeBoxLinePairs(inputLines, inputPose);
    if (linePairs.isEmpty())
    {
        if (debug)
            qDebug() << state.frameId << "No medium pairs found!";
        return mediumSnappedPose;
    }

    if (false && debug)
    {
        qDebug() << state.frameId << "All medium line pairs:" << linePairs.size();
        qDebug() << linePairs;
    }


    // 2. Build pairs of line pairs to acquire a set of hypotheses of what the transformation
    // of the mediumSnapPose could be. In this process, the hypotheses are split into two sets:
    // parallel hypotheses and proper hypotheses. For medium snapping, all parallel hypotheses
    // are discarded.
    Vector<TransformationHypothesis> properHypothesisSet;
    TransformationHypothesis th;
    for (uint i = 0; i < linePairs.size(); i++)
    {
        for (uint j = i+1; j < linePairs.size(); j++)
        {
            LinePair& lpi = linePairs[i];
            LinePair& lpj = linePairs[j];

            // Compute the transformation hypothesis from line pairs i and j.
            if (th.computeTransform(lpi, lpj)) // returns false if the pair is invalid
            {
                if (!th.isParallel)
                {
                    // Valid hypothesis found.
                    th.id = properHypothesisSet.size();
                    th.linePair1 = &lpi;
                    th.linePair2 = &lpj;
                    properHypothesisSet << th;

                    if (debug)
                    {
                        LinePair lpii = lpi;
                        LinePair lpjj = lpj;
                        lpii.inputPose = th.tr();
                        lpjj.inputPose = th.tr();
                        qDebug() << th.id << "Pairs" << lpi.inputLine->id << lpi.mapLine->id << "and" << lpj.inputLine->id << lpj.mapLine->id
                                 << "suggest" << th.tr() << "norm:" << th.tr().norm() << "weight:" << th.weight
                                 //<< "angDiff:" << fabs(pihalfcut(lpi.mapLine->ang - lpj.mapLine->ang))
                                 //<< "overlap:" << lpii.percentualOverlap() << lpjj.percentualOverlap()
                                 << "lld:" << lpii.lineLineDist() << lpjj.lineLineDist() << "parallel:" << th.isParallel;
                    }
                }
            }
        }
    }

    if (properHypothesisSet.isEmpty())
    {
        // This happens quite a lot when all input lines are parallel.
        // No medium snapping is possible.

        if (debug)
            qDebug() << state.frameId << "No proper medium hypothesis set found!";
        return mediumSnappedPose; // Should be zero.
    }

    // 3. Extract the largest consensus set from the hypothesis set.
    // This is a great way to discard outliers, i.e. small sets of single or few transform hypotheses
    // that don't result in a good match.
    Vector<TransformationHypothesis> hypothesisConsensusSet = hypConsensus(properHypothesisSet, config.slamClusteringTransformEps);
    if (debug)
    {
        qDebug() << state.frameId << "Hypothesis consensus set:";
        qDebug() << hypothesisConsensusSet;
    }

    // Special case: if no hypothesis received any votes, the medium snapping failed.
    bool allZeroVotes = true;
    for (uint i = 0; i < hypothesisConsensusSet.size(); i++)
    {
        if (hypothesisConsensusSet[i].votes > 0)
        {
            allZeroVotes = false;
            break;
        }
    }
    if (allZeroVotes)
    {
        if (debug)
            qDebug() << state.frameId << "No hypothesis received any votes. Medium snap failed.";
        return mediumSnappedPose;
    }

    // The consensus set can still contain multiple clusters of hypotheses, i.e. it is multi-modal.
    // Perform a cluster analysis and pick the largest one with the most votes.
    Vector<Vector<TransformationHypothesis> > hypsClusters = hypClusters(hypothesisConsensusSet, config.slamClusteringTransformEps);
    if (hypsClusters.size() > 1)
    {
        if (false && debug)
            qDebug() << state.frameId << "Multiple hyp clusters found." << hypsClusters;

        // Count the votes in each cluster.
        Vector<uint> totalVotesInClusters;
        for (uint i = 0; i < hypsClusters.size(); i++)
        {
            uint totalVotesInCluster = 0;
            for (uint j = 0; j < hypsClusters[i].size(); j++)
                totalVotesInCluster += hypsClusters[i][j].votes;
            totalVotesInClusters << totalVotesInCluster;
        }

        // See where the most votes are.
        uint maxVotesIndex = 0;
        uint maxVotes = 0;
        for (uint i = 0; i < totalVotesInClusters.size(); i++)
        {
            if (totalVotesInClusters[i] > maxVotes)
            {
                maxVotesIndex = i;
                maxVotes = totalVotesInClusters[i];
            }
        }

        // Test if we can find one unique cluster with the most votes.
        bool unimodal = true;
        for (uint i = 0; i < totalVotesInClusters.size(); i++)
        {
            if (totalVotesInClusters[i] >= maxVotes && i != maxVotesIndex)
            {
                unimodal = false;
                break;
            }
        }

        if (unimodal)
        {
            // One cluster is clearly the winner. That means it managed to match the most lines and
            // is certainly the best choice.
            hypothesisConsensusSet = hypsClusters[maxVotesIndex];
            //qDebug() << "Cluster votes:" << totalVotesInClusters << "max idx:" << maxVotesIndex << "unimodal:" << unimodal;
            if (debug)
                qDebug() << state.frameId << "Consensus ambiguity successfully resolved. votes:" << &totalVotesInClusters;
        }
        else
        {
            // This is problematic. There are multiple equally bad hypotheses. In most cases, this
            // happens when no good match could be made, only a bunch of partial matches. We consider
            // this a failed match.
            if (debug)
                qDebug() << state.frameId << "Consensus ambiguity could not be resolved. votes:" << &totalVotesInClusters;
            return mediumSnappedPose;
        }
    }


    // 3. Compute the weighted average transformation of the consensus set and let that be our snap transform.
    double totalWeight = 0;
    Pose2D avgTransform;
    for (uint i = 0; i < hypothesisConsensusSet.size(); i++)
    {
        TransformationHypothesis& t = hypothesisConsensusSet[i];
        Pose2D tr = t.tr();
        avgTransform.x += t.weight*tr.x;
        avgTransform.y += t.weight*tr.y;
        avgTransform.z += t.weight*tr.z;
        totalWeight += t.weight;
    }
    avgTransform /= totalWeight;
    mediumSnappedPose = avgTransform;

    if (debug)
        qDebug() << state.frameId << "Weighted avg transform:" << avgTransform;


    // 4. Build a set of confirmed line pairs that appear in the consensus set.
    confirmedLinePairs.clear();
    for (uint i = 0; i < hypothesisConsensusSet.size(); i++)
    {
        if (!hypothesisConsensusSet[i].linePair1->confirmed)
        {
            confirmedLinePairs << *hypothesisConsensusSet[i].linePair1;
            confirmedLinePairs.last().inputPose = mediumSnappedPose;
        }
        if (!hypothesisConsensusSet[i].linePair2->confirmed)
        {
            confirmedLinePairs << *hypothesisConsensusSet[i].linePair2;
            confirmedLinePairs.last().inputPose = mediumSnappedPose;
        }
        hypothesisConsensusSet[i].linePair1->confirmed = true;
        hypothesisConsensusSet[i].linePair2->confirmed = true;
    }
    if (debug)
    {
        qDebug() << state.frameId << "Input lines:" << inputLines;
        qDebug() << state.frameId << "Confirmed medium line pairs:" << confirmedLinePairs;
    }


    // 5. Evaluate the quality of the medium snap.
    // The snap quality as expressed as successfully overlapped input length divided by the total input length.
    // The snap quality approaches 1 if most input lines were part of a hypothesis in the consensus set.
    double totalInputLength = 0;
    for (uint i = 0; i < inputLines.size(); i++)
        totalInputLength += inputLines[i].length();
    double confirmedLength = 0;
    for (uint i = 0; i < confirmedLinePairs.size(); i++)
        confirmedLength += confirmedLinePairs[i].overlap();
    mediumSnapQuality = confirmedLength/totalInputLength;

    if (debug)
        qDebug() << state.frameId << "Medium snap quality total length:" << totalInputLength
                 << "confirmed length:" << confirmedLength << mediumSnapQuality;

    // We only consider highly reliable global snaps with at least 75% confirmed overlap.
    if (mediumSnapQuality < 0.75)
    {
        // Snap failed.
        mediumSnappedPose = Pose2D();
        return mediumSnappedPose;
    }

    // Gather the observers from the confirmed line pairs that contributed to the
    // global snap. We need this to identify the closest pose graph node in a loop
    // closing situation, and also to gather the map lines that need merging after
    // closing a loop.
    observers.clear();
    for (uint i = 0; i < confirmedLinePairs.size(); i++)
        observers.unify(confirmedLinePairs[i].mapLine->observerNodes);
    //qDebug() << "Observers:" << observers;

    if (debug)
        qDebug() << "Returning medium snap:" << mediumSnappedPose << "quality:" << mediumSnapQuality;

    return mediumSnappedPose;
}

// Build pairs between input lines and map lines in a way that for every input line, if applicable,
// the nearest map line is found according to the line-pose-distance with respect to the inputPose.
// Pairs are still discarded based on line length and line-pose-distance thresholds so that some input
// lines may end up without a map line pair. The input lines are expected in local coordinates relative
// to the inputPose. The computed pairs are returned as a Vector of LinePairs that have the inputPose
// written into them.
Vector<LinePair> GeometricMap::computeNearestLinePairs(Vector<TrackedLine> &inputLines, const Pose2D &inputPose, bool debug)
{
    // Using an inputPose and thresholds in the transformation between input and map line pairs,
    // we are actually investing a prior and constrain this function to be a "local" pair
    // building function, where the actual pose of the robot where the pairs are seen from
    // can only deviate from the inputPose by a bounded amount defined by the bounds applied
    // to the line-pose-distance. We are picking pairs only from the active map lines in the
    // close neighbourhood of the agent in order to avoid having to cycle through all lines
    // in the map.

    Vector<LinePair> linePairs;

    // For all combination of active map lines and input lines...
    for (uint i = 0; i < inputLines.size(); i++)
    {
        TrackedLine& inputLine = inputLines[i];

        double nearestMapLineCost = 0;
        TrackedLine* nearestMapLine = 0;

        ListIterator<TrackedLine*> mapLineIterator = activeMapLines.begin(); // Using the active map lines only.
        while (mapLineIterator.hasNext())
        {
            TrackedLine* mapLine = mapLineIterator.next();

            // Length difference threshold check. Input lines cannot be significantly longer than map lines.
            // This is a generic constraint that can always be applied, no matter if local or global pairing.
            double lengthDiff = inputLine.length()-mapLine->length();
            if (lengthDiff > config.slamPairingMaxLengthDeviation)
            {
                if (debug)
                    qDebug() << "Discarding pair" << inputLine.id << mapLine->id << "due to length deviation" << lengthDiff << "lengths:" << inputLine.length() << mapLine->length();
                continue;
            }
/*
            // Cost check. If the line-pose-distance is above a certain threshold, the lines cannot be pairs.
            double cost = mapLine->linePoseDist(inputLine, inputPose);
            if (cost > config.slamPairingMaxPoseDist)
            {
                if (debug)
                    qDebug() << "Discarding pair" << inputLine.id << mapLine->id << "due to lpd cost" << cost;
                continue;
            }

            // Okay, input line i and the map line can be paired.
            LinePair lp;
            lp.id = linePairs.size();
            lp.inputPose = inputPose; // The assumed pose the input lines are seen in.
            lp.inputLine = &inputLine; // A pointer to the input line in local coordinates relative to inputPose.
            lp.mapLine = mapLine; // A pointer to the map line in world coordinates.
            linePairs << lp;
*/

            // Line-pose-dist.
            double cost = mapLine->linePoseDist(inputLine, inputPose);
            if (nearestMapLineCost == 0 || cost < nearestMapLineCost)
            {
                nearestMapLineCost = cost;
                nearestMapLine = mapLine;
            }
        }

        // Cost check. If the line-pose-distance is above a certain threshold, the lines cannot be pairs.
        if (nearestMapLineCost == 0 || nearestMapLineCost > config.slamPairingMaxPoseDist)
        {
            if (debug)
            {
                qDebug() << "No valid nearest pair found to input line" << inputLine.id << ".";
                if (nearestMapLine != 0)
                    qDebug() << " Nearest map line" << nearestMapLine->id << "cost:" << nearestMapLineCost;
            }
        }
        else
        {
            // Okay, input line i and the nearest map line can be paired.
            LinePair lp;
            lp.id = linePairs.size();
            lp.inputPose = inputPose; // The assumed pose the input lines are seen in.
            lp.inputLine = &inputLine; // A pointer to the input line in local coordinates relative to inputPose.
            lp.mapLine = nearestMapLine; // A pointer to the map line in world coordinates.
            linePairs << lp;
        }
    }
    return linePairs;
}

// Build all possible pairs between input lines and map lines in a way that only one sanity
// check is applied: an input line is not allowed to be much longer than a map line.
// We never want to global snap in the active snap, so active lines are also ignored.
// The input lines can be in any coordinate frame, it does not matter.
// The inputPose of the returned line pairs remains zero.
Vector<LinePair> GeometricMap::computeAllLinePairs(Vector<TrackedLine> &inputLines, bool debug)
{
    Vector<LinePair> linePairs;

    // For all combination of map lines and input lines...
    for (uint i = 0; i < inputLines.size(); i++)
    {
        TrackedLine& inputLine = inputLines[i];

        ListIterator<TrackedLine> mapLineIterator = mapLines.begin();
        while (mapLineIterator.hasNext()) // There could be many map lines.
        {
            TrackedLine& mapLine = mapLineIterator.next();

            // Length difference threshold check. Input lines cannot be significantly longer, than map lines.
            // This is a generic constraint that can always be applied, no matter if local or global pairing.
            double lengthDiff = inputLine.length()-mapLine.length();
            if (lengthDiff > config.slamPairingMaxLengthDeviation)
            {
                if (debug)
                    qDebug() << "Discarding pair" << inputLine.id << mapLine.id << "due to length deviation" << lengthDiff;
                continue;
            }

            // Okay, input line i and map line j can be paired.
            LinePair lp;
            lp.id = linePairs.size();
            lp.inputLine = &inputLine; // A poiner to the input line in local coordinates relative to inputPose.
            lp.mapLine = &mapLine; // A pointer to the map line in world coordinates.
            linePairs << lp;
        }
    }

    return linePairs;
}

// Build all possible pairs between input lines and map lines that touch the selection box.
// One sanity check is applied: an input line is not allowed to be much longer than a map line.
// We never want to global snap in the active snap, so active lines are also ignored.
// The input lines can be in any coordinate frame, it does not matter.
// The inputPose of the returned line pairs remains zero.
Vector<LinePair> GeometricMap::computeBoxLinePairs(Vector<TrackedLine> &inputLines, const Pose2D &inputPose, bool debug)
{
    Vector<LinePair> linePairs;

    Box selectionBox(inputPose.pos(), config.slamSelectionBoxSize, -config.slamSelectionBoxSize, -config.slamSelectionBoxSize, config.slamSelectionBoxSize);
    LinkedList<TrackedLine *> selectedMapLines = boxSelectMapLines(selectionBox);

    // For all combination of map lines and input lines...
    for (uint i = 0; i < inputLines.size(); i++)
    {
        TrackedLine& inputLine = inputLines[i];

        ListIterator<TrackedLine*> mapLineIterator = selectedMapLines.begin();
        while (mapLineIterator.hasNext()) // There could be many map lines.
        {
            TrackedLine* mapLine = mapLineIterator.next();

            if (mapLine->isActive())
            {
                if (debug)
                    qDebug() << "Discarding pair" << inputLine.id << mapLine->id << "due to the map line being active.";
                continue;
            }

            // Length difference threshold check. Input lines cannot be significantly longer, than map lines.
            // This is a generic constraint that can always be applied, no matter if local or global pairing.
            double lengthDiff = inputLine.length()-mapLine->length();
            if (lengthDiff > config.slamPairingMaxLengthDeviation)
            {
                if (debug)
                    qDebug() << "Discarding pair" << inputLine.id << mapLine->id << "due to length deviation" << lengthDiff;
                continue;
            }

            // Okay, input line i and map line j can be paired.
            LinePair lp;
            lp.id = linePairs.size();
            lp.inputLine = &inputLine; // A poiner to the input line in local coordinates relative to inputPose.
            lp.mapLine = mapLine; // A pointer to the map line in world coordinates.
            linePairs << lp;
        }
    }

    return linePairs;
}

// Updates the map lines by updating confirmed line pairs with their associated observation
// and adding unconfirmed observations as new lines. Before calling this function, propsected
// confirmedLinePairs and unconfirmedInputLines must have been computed. The function returns
// a Vector of pointers to the map lines that have been seen by this update.
LinkedList<TrackedLine*> GeometricMap::updateMapLines(bool debug)
{
    LinkedList<TrackedLine*> seenLines;

    // Every confirmed map line (one that has at least one paired input line) is moved towards
    // its associated line observation. Right now it is such that if the mapLine does not accept
    // the observation, the observation is discarded.
    for (uint i = 0; i < confirmedLinePairs.size(); i++)
    {
        bool ret = confirmedLinePairs[i].mapLine->addLineObservation(confirmedLinePairs[i].prospectedLine(), 1.0);
        seenLines.unify(confirmedLinePairs[i].mapLine);
        if (ret)
        {
            if (debug)
                qDebug() << "Line" << confirmedLinePairs[i].mapLine->id << "updated with" << confirmedLinePairs[i].inputLine->id;
        }
    }

    // Add the unconfirmed input lines to the map, if they are long enough.
    for (uint i = 0; i < unconfirmedInputLines.size(); i++)
    {
        addLine(unconfirmedInputLines[i]);
        seenLines << &mapLines.last();
        if (debug)
            qDebug() << "Unconfirmed line" << unconfirmedInputLines[i].id << mapLines.last() << "added to the map.";
    }

    // Debug output all lines.
    if (false && debug)
    {
        qDebug() << state.frameId << "All map lines:";
        ListIterator<TrackedLine> mapLineIterator = mapLines.begin();
        while (mapLineIterator.hasNext())
            qDebug() << mapLineIterator.next();
        qDebug() << activeMapLines;
    }

    return seenLines;
}

// Merges pairs of map lines if they are close according to the line-line-distance metric
// and joins them to a weighted average.
void GeometricMap::mergeMapLines(bool debug)
{
    if (debug)
        qDebug() << state.frameId << "Merging in active lines" << activeMapLines;

    // Merge pairs of map lines if they are close to each other according to the
    // line-line-distance metric and join them to a weighted average.
    ListIterator<TrackedLine*> activeMapLineIterator = activeMapLines.begin();
    while (activeMapLineIterator.hasNext())
    {
        TrackedLine* mapLine1 = activeMapLineIterator.cur();

        ListIterator<TrackedLine*> it2 = activeMapLineIterator.nextIt();
        while (it2.hasNext() && !it2.atBegin())
        {
            TrackedLine* mapLine2 = it2.cur();

            // Skip pairs where neither map line has been touched in this frame.
            // This keeps the computation time in check, even if it is an O(N²) algorithm.
            // Right now, this is in the way of loop closing. It would be better to use the
            // active map line set for merging.
//            if (mapLine1->lastSeen < state.frameId && mapLine2->lastSeen < state.frameId)
//            {
//                it2.next();
//                continue;
//            }

            if (false && debug)
                qDebug() << state.frameId << "Merging" << mapLine1->id << "and" << mapLine2->id << "lld:" << mapLine1->lineLineDist(*mapLine2, debug);
            if (mapLine1->addLineObservation(*mapLine2, mapLine2->totalWeight))
            {
                if (debug)
                    qDebug() << state.frameId << "Merged map lines" << mapLine1->id << "and" << mapLine2->id;
                removeLineFromPoseGraph(mapLine2);
                mapLines.removeOne(*mapLine2);
                activeMapLines.remove(it2); // This also steps the iterator forward.
            }
            else
            {
                it2.next();
            }
        }

        activeMapLineIterator.next();
    }
}

// Removes uncertain lines from the map that haven't been confirmed with enough observations.
void GeometricMap::expireMapLines(bool debug)
{
    // Expire uncertain lines from the map that haven't been confirmed enough times.
    // Only lines in the active line set can expire.
    ListIterator<TrackedLine*> expiredLinesIterator = activeMapLines.begin();
    while (expiredLinesIterator.hasNext())
    {
        TrackedLine* ml = expiredLinesIterator.cur();

        if (ml->observationCount < config.slamMinObservationCount && (state.frameId - ml->lastSeen) > config.slamMinObservationCount)
        {
            if (debug)
                qDebug() << state.frameId << "Expired uncertain map line" << ml << expiredLinesIterator.hasNext() << expiredLinesIterator.atEnd();
            removeLineFromPoseGraph(ml);
            activeMapLines.remove(expiredLinesIterator); // This also steps the iterator forward.
            mapLines.remove(ml);
        }
        else
        {
            expiredLinesIterator.next();
        }
    }
}

// Erases map lines that no longer exist. The visibility polygon is used to clear the map.
void GeometricMap::eraseMapLines(const Pose2D& pose, const Polygon &visPol, bool debug)
{
    // Erase map lines that no longer exist.
    // A negatively inflated and range-bounded visibility polygon is used to erase lines.
    // Without the negative inflation, the map lines would randomly intersect with the boundary
    // of the visibility polygon. For some reason I don't understand, offsetting with the Clipper
    // library works best on a polygon in local coordinates. Multiple polygons can be the
    // result of the negative offsetting. Only the lines in the active line set can be erased,
    // otherwise we would erase old but correct map lines shortly before loop closing situations.

    // Generate the bounded and offseted visiblity polygon.
    Polygon boundedVisPol = visPol;
    ListIterator<Line> ei = boundedVisPol.edgeIterator();
    while (ei.hasNext())
    {
        Line& edge = ei.next();
        Vec2& p1 = edge.p1();
        Vec2& p2 = edge.p2();
        if (p1.length() > config.slamVisibilityPolygonMaxDistance)
            p1.normalize(config.slamVisibilityPolygonMaxDistance);
        if (p2.length() > config.slamVisibilityPolygonMaxDistance)
            p2.normalize(config.slamVisibilityPolygonMaxDistance);
    }
    Polygon visibilityPolygon = boundedVisPol + pose;
    Vector<Polygon> reducedVisibilityPolygons = boundedVisPol.offseted(-config.slamVisibilityPolygonShrinking, config.laserDouglasPeuckerEpsilon);
    reducedVisibilityPolygons += pose;

    // Test every mapLine in the active map lines set if it intersects with the reduced visibility polygon.
    Vector<TrackedLine> lineBuffer;
    ListIterator<TrackedLine*> it = activeMapLines.begin();
    while (it.hasNext())
    {
        const TrackedLine& mapLine = *it.cur();

        bool intersects = false;
        for (uint j = 0; j < reducedVisibilityPolygons.size(); j++)
        {
            if (reducedVisibilityPolygons[j].intersects(mapLine.p1())
                    || reducedVisibilityPolygons[j].intersects(mapLine.p2())
                    || reducedVisibilityPolygons[j].intersects(mapLine))
            {
                if (debug)
                    qDebug() << state.frameId << "Erasing map line"
                             << mapLine << "intersects with the visibility polygon.";

                // If the line does intersect with the reduced visibility polygon, use the full visibility
                // polygon to clip the line and erase the part that is inside the visbility polygon.
                // A line can be split into multiple pieces in this process. Remove the line from the map
                // but keep the remaining pieces.
                Vector<Line> clippedLines = visibilityPolygon.clipLine(mapLine);
                for (uint k = 0; k < clippedLines.size(); k++)
                {
                    if (clippedLines[k].length() > config.laserMinLineLength)
                    {
                        lineBuffer << TrackedLine(clippedLines[k], state.frameId);
                        lineBuffer.last().firstSeen = mapLine.firstSeen;
                        lineBuffer.last().lastSeen = mapLine.lastSeen;
                        lineBuffer.last().observationCount = mapLine.observationCount;
                        lineBuffer.last().observerNodes = mapLine.observerNodes;
                    }
                }

                mapLines.removeOne(*it.cur());
                removeLineFromPoseGraph(it.cur());
                activeMapLines.remove(it); // Steps the iterator forward.

                intersects = true;
                break;
            }
        }

        if (!intersects)
            it.next();
    }

    for (uint i = 0; i < lineBuffer.size(); i++)
    {
        addLine(lineBuffer[i]);
        ListIterator<PoseGraphNode*> it = lineBuffer[i].observerNodes.begin();
        while (it.hasNext())
            it.next()->seenMapLines << &mapLines.last();

        if (debug)
            qDebug() << "Clipped line" << lineBuffer[i].id << mapLines.last() << "added to the map.";
    }

    return;
}

// Returns linePairs that occur in the largest consensus set with respect to the angle between
// the lines in a pair and the neighbourhood threshold eps.
Vector<LinePair> GeometricMap::rotationConsensus(const Vector<LinePair> &linePairs, double eps, bool debug)
{
    // Extract the angle diffs from the given line pairs.
    static Vector<double> angleDiffs;
    angleDiffs.clear();
    for (uint i = 0; i < linePairs.size(); i++)
        angleDiffs << linePairs[i].angleDiff();

    if (debug)
    {
        // Compute an epsilon voting over the angleDiffs.
        // This is just for the debug output.
        Vector<uint> rotationAngleVotes = Statistics::voting(angleDiffs, eps);
        qDebug() << "Rotation angle votes:";
        for (uint i = 0; i < rotationAngleVotes.size(); i++)
        {
            const LinePair& lp = linePairs[i];
            qDebug() << rotationAngleVotes[i] << "  " << lp.inputLine->id << lp.mapLine->id
                     << "weight:" << lp.weight()
                     << "angle:" << lp.angleDiff()
                     << "ortho:" << lp.ortho();
        }
    }

    // Compute the consesus set of the angle diffs.
    const Vector<uint>& consensusIdx = Statistics::consensusSet(angleDiffs, eps);
    Vector<LinePair> consensusSet;
    for (uint i = 0; i < consensusIdx.size(); i++)
        consensusSet << linePairs[consensusIdx[i]];

    return consensusSet;
}

// Returns linePairs that occur in the largest consensus set with respect to the ortho between
// the lines in a pair and the neighbourhood threshold eps.
Vector<LinePair> GeometricMap::translationConsensus(const Vector<LinePair>& linePairs, double eps, bool debug)
{
    // Extract the orthos from the line pairs.
    static Vector<double> orthos;
    orthos.clear();
    for (uint i = 0; i < linePairs.size(); i++)
        orthos << linePairs[i].ortho();

    if (debug)
    {
        // Compute an epsilon voting over the orthos.
        // This is just for the debug output.
        Vector<uint> orthoVotes = Statistics::voting(orthos, eps);
        qDebug() << "Ortho votes:";
        for (uint i = 0; i < orthoVotes.size(); i++)
        {
            const LinePair& lp = linePairs[i];
            qDebug() << orthoVotes[i] << "  " << lp.inputLine->id << lp.mapLine->id
                     << "weight:" << lp.weight()
                     << "angle:" << lp.angleDiff()
                     << "ortho:" << lp.ortho();
        }
    }

    // Compute the consesus set of the orthos.
    const Vector<uint>& consensusIdx = Statistics::consensusSet(orthos, eps);
    Vector<LinePair> consensusSet;
    for (uint i = 0; i < consensusIdx.size(); i++)
        consensusSet << linePairs[consensusIdx[i]];

    return consensusSet;
}

// Returns the transformation hypotheses that are in consens with each other with respect to the dist
// between their transformations and the neighbourhood threshold eps. In other words, outliers are
// discarded that are far away from the agreeing majority of the hypotheses set.
Vector<TransformationHypothesis> GeometricMap::hypConsensus(Vector<TransformationHypothesis>& hyps, double eps, bool debug)
{
    // The consensus set is computed with a voting algorithm. For every hypothesis in the set,
    // we count all voters that are less than eps distance away. Finally, we determine the hyp(s)
    // that have the most voters and gather their voters to form the consensus set.
    // We apply a restriction that each input line is only allowed to vote once for a hypothesis
    // in order to avoid overvoting when many map lines are close to each other and the same
    // input line is assigned to different map lines in the same place.
    uint mostVotes = 0;
    static Vector<uint> mostVotesIdx;
    mostVotesIdx.clear();

    // Reset all votes to zero.
    for (uint i = 0; i < hyps.size(); i++)
        hyps[i].votes = 0;

    for (uint i = 0; i < hyps.size(); i++)
    {
        // Reset the input line closing. We (ab)use the active flag of the TrackedLine
        // for marking the input lines that have already voted.
        for (uint k = 0; k < hyps.size(); k++)
        {
            hyps[k].linePair1->inputLine->active = false;
            hyps[k].linePair2->inputLine->active = false;
        }
        hyps[i].linePair1->inputLine->active = true;
        hyps[i].linePair2->inputLine->active = true;

        for (uint j = 0; j < hyps.size(); j++)
        {
            if (i == j)
                continue;

            if (hyps[i].dist(hyps[j]) < eps)
            {
                if (false && debug)
                    qDebug() << "hyps" << i << j << "are close. dist:" << hyps[i].dist(hyps[j]);

                if (!hyps[j].linePair1->inputLine->active)
                {
                    hyps[i].votes++;
                    hyps[j].linePair1->inputLine->active = true;
                    if (false && debug)
                        qDebug() << "  input line" << hyps[j].linePair1->inputLine->id << "voted";
                }

                if (!hyps[j].linePair2->inputLine->active)
                {
                    hyps[i].votes++;
                    hyps[j].linePair2->inputLine->active = true;
                    if (false && debug)
                        qDebug() << "  input line" << hyps[j].linePair2->inputLine->id << "voted";
                }
            }
        }

        if (hyps[i].votes > mostVotes)
        {
            mostVotes = hyps[i].votes;
            mostVotesIdx.clear();
        }

        if (hyps[i].votes >= mostVotes)
            mostVotesIdx << i;
    }

    if (debug)
    {
        qDebug() << "Hyp consensus:";
        for (uint i = 0; i < hyps.size(); i++)
            qDebug() << i << hyps[i].votes << "hyp:" << hyps[i];
        qDebug() << "Most popular:" << &mostVotesIdx;
    }

    // 2. Collect the indices of all the voters.

    uint ms = mostVotesIdx.size();
    char closed[hyps.size()];
    memset(closed, 0, hyps.size());
    for (uint k = 0; k < ms; k++)
        closed[mostVotesIdx[k]] = 1;
    for (uint k = 0; k < ms; k++)
    {
        uint i = mostVotesIdx[k];
        for (uint j = 0; j < hyps.size(); j++)
        {
            if (closed[j] == 1)
                continue;

            if (hyps[i].dist(hyps[j]) < eps)
            {
                mostVotesIdx << j;
                closed[j] = 1;
            }
        }
    }

    static Vector<TransformationHypothesis> consensusSet;
    consensusSet.clear();
    for (uint i = 0; i < mostVotesIdx.size(); i++)
        consensusSet << hyps[mostVotesIdx[i]];

    return consensusSet;
}

// Clusters the vector of TransformationHypothesis with the DBScan algorithm using eps as
// the distance threshold. A vector of vectors is returned, one vector for each cluster
// containing the *index* of the element in the list belonging to this cluster.
const Vector<Vector<TransformationHypothesis> >& GeometricMap::hypClusters(const Vector<TransformationHypothesis>& hyps, double eps, bool debug)
{
    bool clustered[hyps.size()];
    for (uint i = 0; i < hyps.size(); i++)
        clustered[i] = false;

    static Vector<uint> N; // neighbourhood queue
    static Vector<TransformationHypothesis> cluster; // current cluster in the making
    static Vector<Vector<TransformationHypothesis> > clusters; // resulting clusters
    N.clear();
    cluster.clear();
    clusters.clear();
    for (uint i = 0; i < hyps.size(); i++)
    {
        if (clustered[i])
            continue;

        // Begin a cluster with item i.
        cluster.clear();
        cluster << hyps[i];
        clustered[i] = true;
        if (debug)
            qDebug() << "Begin cluster" << clusters.size() << "with" << i << hyps[i];

        // Queue all item i's eps neighbours (except itself) into the neighbourhood queue.
        N.clear();
        for (uint j = i+1; j < hyps.size(); j++)
        {
            if (!clustered[j] && hyps[j].dist(hyps[i]) < eps)
            {
                N << j;
                if (debug)
                    qDebug() << " Queueing direct neighbour" << j << hyps[j] << "dist:" << hyps[j].dist(hyps[i]);
            }
            else if (debug)
                qDebug() << " Skipping direct neighbour" << j << hyps[j] << "dist:" << hyps[j].dist(hyps[i]);
        }

        for (uint k = 0; k < N.size(); k++)
        {
            if (clustered[N[k]])
                continue;

            clustered[N[k]] = true;
            cluster << hyps[N[k]];
            if (debug)
                qDebug() << "  Adding neighbour" << N[k] << hyps[N[k]] << "dist:" << hyps[N[k]].dist(hyps[i]);
            for (uint j = 0; j < hyps.size(); j++)
            {
                if (!clustered[j] && hyps[j].dist(hyps[N[k]]) < eps)
                {
                    N << j;
                    if (debug)
                        qDebug() << "   Neighbor" << j << hyps[j] << "is reachable from" << N[k] << hyps[N[k]] << "dist:" << hyps[j].dist(hyps[N[k]]);
                }
                else if (debug)
                    qDebug() << "  Skipping neighbor" << j << hyps[j] << "dist:" << hyps[j].dist(hyps[N[k]]);
            }
        }

        clusters << cluster;
    }

    return clusters;
}



// Updates the pose graph by adding a new node at the provided pose if needed and connecting
// the seen map lines with the graph. The set of seen map lines as currently seen by the sensor
// is expected to be unified and in global coordinates.
void GeometricMap::updatePoseGraph(const Pose2D& pose, const LinkedList<TrackedLine *> &seenMapLines, bool debug)
{
    // Update the pose graph.
    // The pose graph is a set of nodes (PoseGraphNodes) that demark specific poses (x,y,theta)
    // and hold pointers to the map lines that were seen from those poses. The pose graph is grown
    // by adding a new node whenever the closest node is farther away than a tunable threshold.
    // Whenever a new node is added, it is linked with the currently seen map lines and the closest
    // node in the graph. Newly observed lines are connected with the nearest graph node whenever
    // they are discovered.

    // First, determine the closest node.
    closestPoseGraphNodeBefore = closestPoseGraphNode;
    closestPoseGraphNode = getClosestNode(pose);
    if (debug && !closestPoseGraphNode->isLeaf())
        qDebug() << state.frameId << "The closest node is not a leaf node.";
    if (debug && closestPoseGraphNode->isJunction())
        qDebug() << state.frameId << "The closest node is a junction.";
    if (debug)
        qDebug() << state.frameId << "Closest node is:" << closestPoseGraphNode << "dist:" << closestPoseGraphNode->dist(pose);

    // Detect small loops.
    if (closestPoseGraphNode != 0 && closestPoseGraphNodeBefore != 0
            && abs((int)closestPoseGraphNode->id - (int)closestPoseGraphNodeBefore->id) > 1
            && !closestPoseGraphNode->neighbours.contains(closestPoseGraphNodeBefore))
    {
        if (debug)
            qDebug() << "Small loop detected between nodes" << closestPoseGraphNode->id << "and" << closestPoseGraphNodeBefore->id;
        closestPoseGraphNode->neighbours << closestPoseGraphNodeBefore;
        closestPoseGraphNodeBefore->neighbours << closestPoseGraphNode;
    }


    // Distance-based pose graph node creation.
    // Add a node to the poseGraph (sometimes) and connect it with all currently seen lines.
    if (poseGraphNodes.isEmpty() || closestPoseGraphNode->dist(pose) > config.slamPoseGraphNodeDist)
    {
        addPoseGraphNode(pose, seenMapLines);
        closestPoseGraphNodeBefore = closestPoseGraphNode;
        closestPoseGraphNode = &poseGraphNodes.last();
        if (debug)
            qDebug() << state.frameId << "New distance Node added" << poseGraphNodes.last();
    }
    else
    {
        // Update the node - map line connections in both directions
        closestPoseGraphNode->seenMapLines.unify(seenMapLines);
        ListIterator<TrackedLine*> it = seenMapLines.begin();
        while (it.hasNext())
            it.next()->observerNodes.unify(closestPoseGraphNode);
    }

    if (debug)
        qDebug() << state.frameId << "Graph nodes:" << poseGraphNodes;
}

// Updates the polygon map.
void GeometricMap::updatePolygonMap(const Pose2D& pose, const Polygon& visPol, bool debug)
{
    // We compute a range-bounded and negatively inflated (offseted) polygon from the visibility
    // polygon. The reduced visibility polygon is used for erasing lines and for computing the
    // polygonal map through union operations. Multiple polygons can be the result of the negative
    // offsetting. However, we limit ourselves to one polygon right in front of the robot. We unify
    // this polygon with the observed polygon of the closest pose graph node. This way, the union
    // operation is relatively cheap, because it is always performed between two small polygons.
    // Also, the growth of the union result is not a big deal because of the offsetting. Whenever a
    // new node becomes the closest one, we unify the observed polygon of the closest
    // node so far with the map polygons. The grown observed polygon of a node can contain holes.
    // Therefore, we need to attach a hierarchy of polygons to a graph node and also the whole
    // polygonal map needs to be a hierarchy.

    // Generate the bounded visibility polygon. This is to throw away unreliable parts of the
    // visibility polygon that are outside of the reliable sensor range.
    Polygon boundedVisPol = visPol;
    ListIterator<Line> ei = boundedVisPol.edgeIterator();
    while (ei.hasNext())
    {
        Line& edge = ei.next();
        Vec2& p1 = edge.p1();
        Vec2& p2 = edge.p2();
        if (p1.length() > config.slamVisibilityPolygonMaxDistance)
            p1.normalize(config.slamVisibilityPolygonMaxDistance);
        if (p2.length() > config.slamVisibilityPolygonMaxDistance)
            p2.normalize(config.slamVisibilityPolygonMaxDistance);
    }

    // Use a negative offset to reduce the size of the bounded visibility polygon.
    // This operation implicitely inflates the obstacles and mitigates the growing union problem.
    // The offsetting operation can result in multiple polygons.
    const Vector<Polygon>& reducedVisibilityPolygons = boundedVisPol.offseted(-config.slamVisibilityPolygonShrinking, config.laserDouglasPeuckerEpsilon);

    // Select the "root" offseted visibility polygon right in front of the robot that would
    // contain the robot in the origin if it weren't for the offset. We could still try just
    // adding the whole pack, what's the big deal?
    for (uint i = 0; i < reducedVisibilityPolygons.size(); i++)
    {
        // Extend the observed polygon of the closest node by uniting it with the currently seen
        // reduced visibility polygon.
        Polygon ppol = reducedVisibilityPolygons[i];
        ppol += pose;
        ppol.transform();
        if (closestPoseGraphNode != 0)
            closestPoseGraphNode->observedNeighborhood.unite(ppol);
        else
            polygonMap.unite(ppol);
    }

    // If the closest pose graph node just changed, merge the observed neighborhood of the last
    // closest node with the polygon map.
    if (closestPoseGraphNode != closestPoseGraphNodeBefore && closestPoseGraphNodeBefore != 0)
    {
        polygonMap.unite(closestPoseGraphNodeBefore->observedNeighborhood);
        polygonMap.simplify(2*config.laserDouglasPeuckerEpsilon);
    }
}

// Adds a new node to the graph and connects the currently seen lines with the new node.
void GeometricMap::addPoseGraphNode(const Pose2D &pose, const LinkedList<TrackedLine *> &seenMapLines)
{
    // Append a new node to the Vector.
    PoseGraphNode newNode;
    if (!poseGraphNodes.isEmpty())
        newNode.id = poseGraphNodes.last().id+1;
    newNode.pose = pose;
    newNode.lastPose = pose;
    newNode.seenMapLines = seenMapLines;
    poseGraphNodes.push(newNode);
    if (closestPoseGraphNode != 0)
    {
        poseGraphNodes.last().neighbours << closestPoseGraphNode;
        closestPoseGraphNode->neighbours << &poseGraphNodes.last();
    }

    // Connect all seen lines with a pointer to the new graph node.
    ListIterator<TrackedLine*> it = seenMapLines.begin();
    while (it.hasNext())
        it.next()->observerNodes << &poseGraphNodes.last();
}

// Removes a node from the graph.
void GeometricMap::removePoseGraphNode(PoseGraphNode *pgn)
{
    //qDebug() << state.frameId << "Removing non-critical graph node" << pgn.id;
    //qDebug() << "seen lines:" << pgn.seenMapLines;
    ListIterator<TrackedLine*> linesIt = pgn->seenMapLines.begin();
    while (linesIt.hasNext())
        linesIt.next()->observerNodes.removeOne(pgn);
    ListIterator<PoseGraphNode*> neighborsIt = pgn->neighbours.begin();
    while (neighborsIt.hasNext())
    {
        neighborsIt.cur()->neighbours.removeOne(pgn);
        neighborsIt.cur()->neighbours.unify(pgn->neighbours);
        neighborsIt.cur()->neighbours.removeOne(neighborsIt.cur());
        neighborsIt.next();
    }
    poseGraphNodes.remove(pgn);
}

// Removes all pointers to line o from the pose graph.
void GeometricMap::removeLineFromPoseGraph(TrackedLine* o)
{
    ListIterator<PoseGraphNode> it = poseGraphNodes.begin();
    while (it.hasNext())
        it.next().seenMapLines.removeOne(o);
}

// Gathers pointers to the map lines that have been seen by nodes close to the pose.
LinkedList<TrackedLine*> GeometricMap::gatherNearbyMapLines(const Pose2D& pose) const
{
    return getClosestNode(pose)->gatherNearbyLines(config.slamPoseGraphNearbyNodes);
}

// Gathers pointers to the nodes close to the pose.
LinkedList<PoseGraphNode *> GeometricMap::gatherNearbyNodes(const Pose2D &pose) const
{
    return getClosestNode(pose)->gatherNeighborhood(config.slamPoseGraphNearbyNodes);
}

// Returns the PoseGraphNode that minimizes the Pose2D dist to pose p.
PoseGraphNode* GeometricMap::getClosestNode(const Pose2D &p, bool debug) const
{
    if (debug)
        qDebug() << state.frameId << "Looking for closest node to pose" << p;

    PoseGraphNode* closestNode = &poseGraphNodes.first();
    double dist = poseGraphNodes.first().dist(p);
    ListIterator<PoseGraphNode> it = poseGraphNodes.begin();
    while (it.hasNext())
    {
        double ddist = it.cur().dist(p);
        if (false && debug)
            qDebug() << "  Checking node" << it.cur().id << it.cur().pose << "dist:" << ddist;
        if (ddist < dist)
        {
            closestNode = &it.cur();
            dist = ddist;
            if (debug)
                qDebug() << "  Closest node is now" << closestNode->id << closestNode->pose << "dist:" << dist;
        }
        it.next();
    }
    return closestNode;
}

// Performs a loop closing optimization where the local snapped pose becomes
// the medium snapped pose. Both poses are given in world coordinates.
void GeometricMap::closeLoop(const Pose2D &localSnap, const Pose2D &mediumSnap)
{
    // The graph optimization happens between a leaf node and a root node.
    // The leaf node is always the node closest to the robot in the moment of the loop detection.
    // The root node is the closest to the medium snap selected from the observers of the medium snap.
    // What the optimization does is that it computes the offset the leaf node needs to be moved by
    // in order for the local snapped pose to become the medium snapped pose. This offset is then
    // distributed over the graph nodes that make up the path from the leaf node to the root node.

    // Determine the closest node to the medium snap among the observer nodes.
    // The observer nodes are gathered from the confirmed line pairs in the snapMedium() function.
    //qDebug() << "Observers:" << observers;
    uint closestIdx = 0;
    double closestDist = observers[0]->dist(mediumSnap);
    for (uint i = 1; i < observers.size(); i++)
    {
        double dist = observers[i]->dist(mediumSnap);
        if (dist < closestDist)
        {
            closestIdx = i;
            closestDist = dist;
        }
    }

    // The three things needed for the loop closing.
    PoseGraphNode* rootNode = observers[closestIdx];
    PoseGraphNode* leafNode = closestPoseGraphNode;
    Pose2D offset = ((mediumSnap - localSnap) + leafNode->pose) - rootNode->pose;

    qDebug() << state.frameId << "PoseGraph::closeLoop()";
    qDebug() << "  root node:" << rootNode->id;
    qDebug() << "  leaf node:" << leafNode->id;
    qDebug() << "  Offset:" << offset << "norm:" << (mediumSnap-localSnap).norm();


    // Optimize the graph between the root node and the leaf node to account for the loop closing offset.
    // This applies a small pose offset for every node along the way from the root to leaf so that in
    // total, the leaf node moves by the loop closing offset.
    optimizeGraph(rootNode, leafNode, offset);


    // Connect the root node with the leaf node.
    leafNode->neighbours.unify(rootNode);
    rootNode->neighbours.unify(leafNode);

    // Update the map lines according to the offset of the associated graph nodes.
    // For now, this iterates over all map lines, but perhaps this could be optimized later, e.g.
    // by gathering pointers to the relevant lines when determining the path from leaf to node.
    ListIterator<TrackedLine> mapLinesIt = mapLines.begin();
    while (mapLinesIt.hasNext())
    {
        TrackedLine& mapLine = mapLinesIt.next();
        Line l = (mapLine-mapLine.observerNodes.first()->lastPose)+mapLine.observerNodes.first()->pose;
        TrackedLine tl(l, true, true, state.frameId);
        ListIterator<PoseGraphNode*> observerIterator = mapLine.observerNodes.begin();
        while (observerIterator.hasNext())
        {
            PoseGraphNode* pgn = observerIterator.next();
            tl.addLineObservation((mapLine-pgn->lastPose)+pgn->pose); // Can pose transformations be chained into one operation?
        }
        mapLine.setTo(tl);
    }


    // Update the observed neighborhood geometric models according to the offset of the associated graph nodes.
    // For now this iterates over all graph nodes, but this could be optimized later.
    ListIterator<PoseGraphNode> nodeIterator = poseGraphNodes.begin();
    while (nodeIterator.hasNext())
    {
        PoseGraphNode& pgn = nodeIterator.next();
        pgn.observedNeighborhood -= pgn.lastPose;
        pgn.observedNeighborhood += pgn.pose;
    }

    // Recompute the polygonal map by computing the union over the observed models of all nodes.
    // TODO Is this faster when we gather all involved polygons first and compute the union only once?
    polygonMap.clear();
    nodeIterator = poseGraphNodes.begin();
    while (nodeIterator.hasNext())
        polygonMap.unite(nodeIterator.next().observedNeighborhood);


    // Update the lastPose to the current pose for the next optimization.
    // For now this iterates over all graph nodes, but this could be optimized later.
    nodeIterator = poseGraphNodes.begin();
    while (nodeIterator.hasNext())
    {
        PoseGraphNode& pgn = nodeIterator.next();
        pgn.lastPose = pgn.pose;
    }
}

// Optimizes the graph between two nodes by iteratively linearizing
// the least-squares problem and solving the corresponding linear system.
// This function will update the graph poses of the graph nodes.
void GeometricMap::optimizeGraph(PoseGraphNode* rootNode, PoseGraphNode* leafNode, const Pose2D& offset)
{
    // The graph optimization procedure is carried out according to:
    // http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf

    StopWatch sw;

    // Renumber the pose graph nodes as long as the id is important.
    // TODO This complication should be eliminated.
    uint id = 0;
    ListIterator<PoseGraphNode> nodeIterator = poseGraphNodes.begin();
    while (nodeIterator.hasNext())
        nodeIterator.next().id = id++;

    // TODO
    // This is a little cheat using the iterator directly on the LinkedList of the graph nodes.
    // This will have to be replaced by a path from root to leaf computed using the neighbors of the nodes.
    ListIterator<PoseGraphNode> node1 = poseGraphNodes.iteratorAt(rootNode);
    ListIterator<PoseGraphNode> node2 = poseGraphNodes.iteratorAt(leafNode);
    qDebug() << "Optimizing graph from node" << node1.cur().id << "to node" << node2.cur().id;

    // Omega is a constant information matrix used for all nodes.
    Eigen::Matrix3d sigma = Eigen::Matrix3d::Zero();
    sigma(0, 0) = 0.15 * 0.15; // std_x = 0.15 m
    sigma(1, 1) = 0.15 * 0.15; // std_y = 0.15 m
    sigma(2, 2) = 0.2 * 0.2;   // std_theta = 0.2 rad
    Eigen::Matrix3d omega = sigma.inverse();

    // Generate the graph constraints.
    // TODO We could get rid of the graph constraints and hide the whole optimization in a unit.
    LinkedList<GraphConstraint> graphConstraints;
    graphConstraints.clear();
    ListIterator<PoseGraphNode> it = node1;
    while (it.hasNext() && it != node2.nextIt())
    {
        GraphConstraint newEdge;
        newEdge.addOdomConstraint(it.peekPrev(), it.cur());
        graphConstraints.push_back(newEdge);
        it.next();
    }

    // The special loop closing constraint.
    GraphConstraint newEdge;
    newEdge.addLoopClosingConstraint(node1.cur(), node2.cur(), offset);
    graphConstraints.push_back(newEdge);

    // Create the linear system.
    uint noOfNodes = 0;
    it = node1;
    while (it.hasNext() && it != node2.nextIt())
    {
        noOfNodes++;
        it.next();
    }
    uint dim = 3 * noOfNodes;

    Eigen::VectorXd x(dim);
    Eigen::VectorXd b(dim);
    Eigen::VectorXd dx(dim);
    Eigen::MatrixXd H(dim,dim);
    Eigen::SparseMatrix<double> HSparse;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;

    // We set the initial guess for ls.x to the current pose of the graph nodes.
    it = node1;
    uint i = 0;
    while (it.hasNext() && it != node2.nextIt())
    {
        const Pose2D& pose = it.cur().pose;
        x(3*i) = pose.x;
        x(3*i+1) = pose.y;
        x(3*i+2) = pose.z;

        //qDebug() << "building ls.x i" << i << "graph node" << it.cur().id << "ls.x:" << ls.x(3*i) << ls.x(3*i+1) << ls.x(3*i+2);

        it.next();
        i++;
    }

    uint numOfIter = 0;
    bool converged = false;
    while (!converged && numOfIter < 10)
    {
        // Linearizing the least-squares problem.
        H.setZero(dim, dim);
        b.setZero(dim);
        ListIterator<GraphConstraint> edgeIter = graphConstraints.begin();
        while (edgeIter.hasNext())
        {
            GraphConstraint& edge = edgeIter.next();

            // We only optimize the graph between two loop closing nodes.
            if (edge.i >= node1.cur().id)
            {
                edge.linearizeConstraint();
                uint i = edge.i-node1.cur().id;
                uint j = edge.j-node1.cur().id;

                //qDebug() << "H update i j" << i << j << "edge" << edge.i << edge.j;

                H.block<3, 3>(3*i, 3*i) += edge.getAij().transpose() * omega * edge.getAij();
                H.block<3, 3>(3*i, 3*j) += edge.getAij().transpose() * omega * edge.getBij();
                H.block<3, 3>(3*j, 3*i) += edge.getBij().transpose() * omega * edge.getAij();
                H.block<3, 3>(3*j, 3*j) += edge.getBij().transpose() * omega * edge.getBij();

                b.block<3, 1>(3*i, 0) += edge.getAij().transpose() * omega * edge.getErrorVector();
                b.block<3, 1>(3*j, 0) += edge.getBij().transpose() * omega * edge.getErrorVector();
            }
            else
            {
                qDebug() << "edge" << edge.i << "does not participate.";
            }
        }

        // Keep the first node fixed.
        H.block<3, 3>(0, 0) += Eigen::Matrix3d::Identity();

        // Sparse solver.
        HSparse = H.sparseView();
        solver.compute(HSparse);
        if (solver.info() != Eigen::Success)
        {
            qDebug() << "Decomposition failed at Iteration#: " << numOfIter;
            return;
        }

        dx = solver.solve(b);
        if (solver.info() != Eigen::Success)
        {
            qDebug() << "Solving failed at Iteration#: " << numOfIter;
            return;
        }

        // Update the parameters.
        x += dx;

        // Update the graph poses
        // TODO This could be done only once in the end.
        it = node1;
        i = 0;
        while (it.hasNext() && it != node2.nextIt())
        {
            Pose2D& pose = it.cur().pose;
            pose.x = x(3*i);
            pose.y = x(3*i+1);
            pose.z = x(3*i+2);

            //qDebug() << "updateGraphPoses i" << i << "node" << it.cur().id << "pose" << it.cur().pose;

            it.next();
            i++;
        }

        // Update the constraints
        edgeIter = graphConstraints.begin();
        while (edgeIter.hasNext())
        {
            GraphConstraint& edge = edgeIter.next();
            edge.updateConstraint(*edge.ni, *edge.nj);
            //qDebug() << "updateGraphConstraints i j" << edge.i << edge.j << "nodes" << edge.ni->id << edge.nj->id;
        }

        numOfIter++;

        if (dx.norm() < 0.001)
        {
            double optTime = sw.elapsedTimeMs();
            qDebug() << "Optimization Successful! -- Total time:" << optTime << "(ms) -- No of Iterations:"
                     << numOfIter << "-- Delta X Norm:" << dx.norm() << "\n";
            converged = true;
        }
    }

    if (!converged)
    {
        double optTime = sw.elapsedTimeMs();
        qDebug() << "Optimization NOT Successful! -- Total time:" << optTime << "(ms) -- No of Iterations:"
                 << numOfIter << "-- Delta X Norm:" << dx.norm() << "\n";
    }

    return;
}

// Draws the LineMap on a QPainter.
void GeometricMap::draw(QPainter *painter, const QPen &pen, const QBrush &brush, double opacity) const
{
    painter->save();
    painter->setPen(pen);
    painter->setBrush(brush);
    painter->setOpacity(opacity);
    ListIterator<TrackedLine> it = mapLines.begin();
    while (it.hasNext())
        it.next().draw(painter);
    painter->restore();
}

// Draws the GeometricMap in OpenGL context.
void GeometricMap::draw() const
{
    if (command.showLineMap == 0)
        return;

    if (command.showPolygonMap > 0)
        drawPolygonMap();

    // Line matching view.
    if (command.showLineMap == 1)
        drawLineMatching();

    // Bare line map view.
    if (command.showLineMap == 2)
        drawMapLines();

    // Line observation accumulation view.
    if (command.showLineMap == 3)
        drawLineAccumulation();

    if (command.showPoseGraph > 0)
        drawPoseGraph();

    if (globalSnapQuality >= 0.75)
        drawGlobalSnap();
}

void GeometricMap::drawLineMatching() const
{
    // First all map lines in light grey.
    glLineWidth(5);
    glColor3f(0.9,0.9,0.9); // light grey
    ListIterator<TrackedLine> it = mapLines.begin();
    while (it.hasNext())
        it.next().draw();

    glTranslated(0,0,0.001);

    // The selection box.
    Box selectionBox(inputPose.pos(), config.slamSelectionBoxSize, -config.slamSelectionBoxSize, -config.slamSelectionBoxSize, config.slamSelectionBoxSize);
    GLlib::setColor(drawUtil.lightGrey);
    glLineWidth(1);
    //selectionBox.draw();

    // The selection box line set in grey.
    glLineWidth(5);
    glColor3f(0.7,0.7,0.7); // grey
    it = mapLines.begin();
    while (it.hasNext())
    {
        if (selectionBox.intersects(it.cur()))
            it.cur().draw();
        it.next();
    }

    glTranslated(0,0,0.001);

    // The active map line set in black.
    glLineWidth(5);
    glColor3f(0.3,0.3,0.3); // black
    ListIterator<TrackedLine*> it2 = activeMapLines.begin();
    while (it2.hasNext())
        it2.next()->draw();

    glTranslated(0,0,0.001);

    // The input pose.
    GLlib::drawNoseCircle(inputPose, drawUtil.black, 0.5*config.agentRadius);

    // The input lines in blue relative to inputPose.
    glLineWidth(5);
    glColor3f(0,0,0.8); // blue
    glPushMatrix();
    glMultMatrixd(inputPose.getMatrix());
    for (uint i = 0; i < inputLines.size(); i++)
        inputLines[i].draw();
    glPopMatrix();

    glTranslated(0,0,0.001);

    // Prospected input lines in red.
    glPushMatrix();
    glMultMatrixd(localSnappedPose.getMatrix());
    glLineWidth(5);
    glColor3f(1,0,0); // red
    for (uint j = 0; j < inputLines.size(); j++)
        inputLines[j].draw();
    glPopMatrix();
}

void GeometricMap::drawMapLines() const
{
    // All map lines in black.
    glColor3f(0,0,0); // black
    glLineWidth(5);
    ListIterator<TrackedLine> it = mapLines.begin();
    while (it.hasNext())
    {
        const TrackedLine& l = it.next();
        l.draw();
        if (l.seenP1)
            GLlib::drawFilledCircle(l.p1(), drawUtil.brush.color(), 0.03);
        if (l.seenP2)
            GLlib::drawFilledCircle(l.p2(), drawUtil.brush.color(), 0.03);
    }
}

void GeometricMap::drawLineAccumulation() const
{
    // The accumulated lines in thin black.
    glLineWidth(1);
    glColor3f(0,0,0); // black
    ListIterator<TrackedLine> it = mapLines.begin();
    while (it.hasNext())
    {
        const TrackedLine& l = it.next();
        for (uint j = 0; j < l.lineObservations.size(); j++)
            l.lineObservations[j].draw();
    }

    glTranslatef(0,0,0.001);

    // All map lines in thick blue.
    glLineWidth(7);
    //glColor3i(127,127,255); // blue
    glColor3f(0.5,0.5,1); // blue
    it = mapLines.begin();
    while (it.hasNext())
        it.next().draw();

    glTranslatef(0,0,0.001);

    // The seen vertices with black dots.
    glColor3f(0,0,0); // black
    it = mapLines.begin();
    while (it.hasNext())
    {
        const TrackedLine& l = it.next();
        for (uint j = 0; j < l.lineObservations.size(); j++)
        {
            const TrackedLine& line = l.lineObservations[j];
            if (line.seenP1)
                GLlib::drawFilledCircle(line.p1(), drawUtil.brush.color(), 0.0075);
            if (line.seenP2)
                GLlib::drawFilledCircle(line.p2(), drawUtil.brush.color(), 0.0075);
        }
    }

    glTranslatef(0,0,0.001);

    // The map line seen vertices.
    it = mapLines.begin();
    while (it.hasNext())
    {
        const TrackedLine& l = it.next();
        if (l.seenP1)
            GLlib::drawFilledCircle(l.p1(), QColor(255,127,255), 0.03);
        if (l.seenP2)
            GLlib::drawFilledCircle(l.p2(), QColor(255,127,255), 0.03);
    }
}

void GeometricMap::drawGlobalSnap() const
{
    glPushMatrix();
    glTranslated(0, 0, 0.01);
    GLlib::drawNoseCircle(globalSnappedPose, Qt::red, 0.5*config.agentRadius);
    glPopMatrix();
}

void GeometricMap::drawPoseGraph() const
{
    if (command.showPoseGraph == 0)
        return;

    QColor activeColor(qRgba(0, 255, 0, 255));
    QColor inactiveColor(qRgba(0, 255, 0, 200));
    inactiveColor.setRgba(qRgba(0, 255, 0, 200));

    // The seen line connections of all nodes.
    if (command.showPoseGraph >= 3)
    {
        glLineWidth(1);
        GLlib::setColor(drawUtil.lightGrey);
        ListIterator<PoseGraphNode> it = poseGraphNodes.begin();
        while (it.hasNext())
            it.next().drawSeenLineConnections();

        glTranslated(0, 0, 0.001);
    }

    // The seen line connections of the nearby nodes.
    if (command.showPoseGraph >= 2 && closestPoseGraphNode != 0)
    {
        LinkedList<PoseGraphNode*> nearbyNodes = closestPoseGraphNode->gatherNeighborhood(config.slamPoseGraphNearbyNodes);
        ListIterator<PoseGraphNode*> it = nearbyNodes.begin();
        glLineWidth(1);
        GLlib::setColor(drawUtil.lightGrey);
        while (it.hasNext())
            it.next()->drawSeenLineConnections();

        glTranslated(0, 0, 0.001);
    }

    // All nodes.
    ListIterator<PoseGraphNode> it = poseGraphNodes.begin();
    while (it.hasNext())
    {
        glLineWidth(1);
        GLlib::setColor(drawUtil.grey);
        ListIterator<PoseGraphNode*> nit = it.cur().neighbours.begin();
        while (nit.hasNext())
            Line(it.cur().pose.pos(), nit.next()->pose.pos()).draw();
        it.next().draw(inactiveColor, 0.5*config.agentRadius);
    }

    glTranslated(0, 0, 0.001);

    if (closestPoseGraphNode != 0)
    {
        // The neighborhood of the closest node.
        closestPoseGraphNode->drawNeighborhood(config.slamPoseGraphNearbyNodes, activeColor, 0.6*config.agentRadius);

        // Mark the closest node with a red circle.
        GLlib::drawCircle(closestPoseGraphNode->pose, drawUtil.red, 0.8*config.agentRadius, 0.02);
    }
}

// Draws the polygon map.
void GeometricMap::drawPolygonMap() const
{
    if (command.showPolygonMap == 0)
        return;

    // 0 - nothing
    // 1 - map polygon
    // 2 - observed neighborhood
    // 3 - both

    if (command.showPolygonMap == 1 || command.showPolygonMap == 3)
    {
        polygonMap.draw(drawUtil.pen, drawUtil.lightGreen, 0.3);
        glTranslated(0, 0, 0.002);
    }

    if (closestPoseGraphNode != 0 && command.showPolygonMap > 1)
    {
        closestPoseGraphNode->observedNeighborhood.draw(drawUtil.pen, drawUtil.lightGreen, 0.8);
        glTranslated(0, 0, 0.002);
    }
}

// Writes the LineMap into a data stream.
void GeometricMap::streamOut(QDataStream &out) const
{
    out << mapLines;
    out << poseGraphNodes;
}

// Reads the LineMap from a data stream.
void GeometricMap::streamIn(QDataStream &in)
{
    in >> mapLines;
    in >> poseGraphNodes;
}

QDataStream& operator<<(QDataStream& out, const GeometricMap &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, GeometricMap &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const GeometricMap &o)
{
    //dbg << o.getLines();
    return dbg;
}

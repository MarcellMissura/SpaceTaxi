#include "GeometricMap.h"
#include "GraphConstraint.h"
#include "HypothesisSet.h"
#include "board/Config.h"
#include "board/Command.h"
#include "board/State.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"
#include "lib/util/Statistics.h"
#include "lib/util/StopWatch.h"
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
// the currently relevant lines of the map. The lines are landmarks only used for
// localization. Lines are being added, updated and erased from the map as the robot moves
// along. The polygons are the skin of the map. They are needed to demark free space and
// blocked space without any gaps so that path planning and motion planning can take place.
// There is at least one large freespace polygon surrounding the map and multiple blocked
// space polygons are inside of it demarking free and occupied space with a two level
// hierarchy. Most of the functions on the GeometricMap class are private and are used
// for the internal workings of the map maintenance. The main public interface of the
// GeometricMap is the slam(inputPose, inputLines) function through which it receives an
// initial guess of the pose and the lines seen by the sensor. The slam function drives
// the simultaneous localization and mapping by incrementally building the map and returning
// a localized pose estimate.

GeometricMap::GeometricMap()
{
    closestPoseGraphNode = 0;
    closestPoseGraphNodeBefore = 0;
    explorationMode = true;
    isLocalized = false;
    badSnapCounter = 0;
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
    explorationMode = true;
    isLocalized = false;
    badSnapCounter = 0;
}

// Returns true when there are no lines in the map.
bool GeometricMap::isEmpty() const
{
    return mapLines.isEmpty();
}

// Adds a line to the map, if the line is long enough.
void GeometricMap::addLine(const TrackedLine &o)
{
    if (o.length() < config.laserLineMinLength)
        return;

    uint id = 0;
    if (!mapLines.isEmpty())
        id = mapLines.last().id+1;
    mapLines << o;
    mapLines.last().id = id;
}

// The main slam function that drives the localization and map building. It returns a Pose2D that is
// localized in the map and also updates the map using the inputLines, the visibility polygon, and the
// sensed polygons. The inputPose should be a good guess to the current pose in the map. The inputLines,
// the visibility polygon and the sensed polygons are expected in local coordinates relative to inputPose.
// In this process, the pose graph is grown, the line map is updated by adding new lines and moving already
// known map lines towards new observations, and the map polygons are grown as the robot drives into
// unexplored areas. Lines and polygons are erased with the help of the visibility polygon.
Pose2D GeometricMap::slam(const Pose2D &inputPose, const Vector<TrackedLine> &inputLines, const Polygon &visibilityPolygon, const LinkedList<Polygon> sensedPolygons)
{
    this->inputPose = inputPose;
    this->inputLines = inputLines;

    // Process the visibility polygon first so that it only has to be done once.
    // We compute a range-bounded and negatively inflated (offseted) polygon from the visibility
    // polygon. The size bounding throws away unreliable parts of the visibility polygon that are
    // outside of the reliable sensor range. The negative offsetting operation implicitly inflates
    // the obstacles and mitigates the growing union problem. The offsetting operation can result
    // in multiple polygons. The reduced visibility polygon(s) are used for erasing lines and for
    // computing the polygonal map through union operations.

    Polygon boundedVisPol = visibilityPolygon;
    ListIterator<Line> ei = boundedVisPol.edgeIterator();
    while (ei.hasNext())
    {
        Line& edge = ei.next();
        Vec2& p1 = edge.p1();
        Vec2& p2 = edge.p2();
        if (p1.length() > config.slamVisibilityPolygonBound)
            p1.normalize(config.slamVisibilityPolygonBound);
        if (p2.length() > config.slamVisibilityPolygonBound)
            p2.normalize(config.slamVisibilityPolygonBound);
    }
    const Vector<Polygon>& reducedVisibilityPolygons = boundedVisPol.offseted(-config.gmPolygonDilation);

    // For measuring localization time and map update time.
    StopWatch stopWatch;
    stopWatch.start();

    // Procedure:
    // If the map is empty, initialize new map, graph, and line structures.
    // If the agent is not localized, attempt global localization.
    // Otherwise start snapping as follows.
    // Determine the closest node
    // Gather active map lines
    // snap -> medium pose, local pose, confirmed lines, unconfirmed lines, observers
    // update map lines -> seen lines
    // unify active map lines and seen lines
    // update pose graph
    // update polygon map
    // loop close -> observer lines
    // merge, expire, erase

    // If the map is empty, initialize the map with the currently observed lines,
    // polygons, and the first pose graph node. Set the state to exploring and localized.
    if (isEmpty())
    {
        for (uint i = 0; i < inputLines.length(); i++)
            addLine(inputLines[i]+inputPose);
        ListIterator<TrackedLine> mapLineIterator = mapLines.begin();
        while (mapLineIterator.hasNext())
            activeMapLines << &mapLineIterator.next();
        addPoseGraphNode(inputPose, activeMapLines);
        closestPoseGraphNode = &poseGraphNodes.last();
        closestPoseGraphNodeBefore = 0;
        explorationMode = true;
        isLocalized = true;
        updatePolygonMap(inputPose, reducedVisibilityPolygons, sensedPolygons);
        return inputPose;
    }

    poseHypotheses.clear();

    // Detect weak input.
    bool weakInput = false;
    double totalInputLength = 0;
    for (uint i = 0; i < inputLines.size(); i++)
        totalInputLength += inputLines[i].length();
    if (inputLines.size() < 5 || totalInputLength < 5.0) // hidden parameters
        weakInput = true;

    // If the robot is not localized, for example after loading a map or having lost tracking...
    if (!isLocalized)
    {
        // Use the global snap function to try and localize in the entire map.
        // This is needed to initially find our pose in the map when the map is loaded
        // or to find ourselves in the map after we lost tracking.
        globalSnappedPose = snapGlobal(this->inputLines, config.debugLevel > 10);

        // Sanity check.
        if (globalSnappedPose.confidence > config.slamSnapAcceptanceThreshold)
        {
            qDebug() << state.frameId << "Global localization succeeded. quality:" << globalSnappedPose.confidence << "global pose:" << globalSnappedPose;
            isLocalized = true;
            closestPoseGraphNode = getClosestNode(globalSnappedPose);
            closestPoseGraphNodeBefore = 0;
            localSnappedPose = globalSnappedPose; // for drawing
            badSnapCounter = 0;
            return globalSnappedPose; // no map update, but localized pose
        }
        else
        {
            qDebug() << state.frameId << "Global localization failed. quality:" << globalSnappedPose.confidence << "global pose:" << globalSnappedPose;
        }

        return inputPose; // no map update, no localization
    }


    // Determine the closest graph node.
    // This task is not as trivial as selecting the Euklidean closest node from the graph,
    // because then errors can occur in near loop closing situations. Imagine the robot has driven
    // a large loop and is returning to a known portion of the map with a large error. Loop detection
    // is based on snapping to lines inside the selection box around the robot EXCLUDING the recent
    // lines in the graph neighborhood. If we were to accept an older node in the graph as the closest
    // one before the loop has been closed, then the old lines would become the graph neighborhood of
    // the robot, which are not allowed to be used for the medium snap, and the loop detection would
    // fail. But then again, there are small loops where the robot drives around a room and returns to
    // a recent part of the graph where we do want to accept the closest node and create a link to it,
    // otherwise the robot would create new lines where we already have lines. Therefore, the closest
    // node detection algorithm is as follows. Determine the geometrically closest node candidate. If
    // the candidate is seen line-connected with the current closest node, then we accept the new
    // closest node and close a small loop. However, if the candidate is not connected with the current
    // closest node in any way, then we do not accept the candidate and keep the current closest node.
    // Loop closing may occur if the medium snap snaps in the latter case. In the former case, the
    // medium snap should not snap.
    PoseGraphNode* closestNodeCandidate = getClosestNode(inputPose);
    if (closestNodeCandidate != closestPoseGraphNode)
    {
        if (closestPoseGraphNode == 0)
            qDebug() << "It's going to crash because of missing closestPoseGraphNode" << closestPoseGraphNode;

        // Shares a seen line with the neighborhood of the closest node.
//        if (closestNodeCandidate->getSeenLines().intersects(closestPoseGraphNode->getSeenLines()))
        if (closestNodeCandidate->getSeenLines().intersects(closestPoseGraphNode->gatherNearbyLines())) // larger neighborhood
        {
            // Detect and close small loops.
            // Small loops occur when the robot drives a small loop, e.g. inside a room, and returns to
            // a recent pose graph node. We can close these small loops without optimization by simply
            // connecting the last closest node with the new closest node the robot returned to. Creating
            // this link allows to match with recent lines that would otherwise not be in the neighborhood.

            // If the nodes are not connected yet, connect them now. This closes a small loop.
            if (!closestPoseGraphNode->neighbours.contains(closestNodeCandidate))
            {
                closestPoseGraphNode->neighbours << closestNodeCandidate;
                closestNodeCandidate->neighbours << closestPoseGraphNode;
                if (config.debugLevel > 0)
                    qDebug() << "Small loop detected between nodes" << closestPoseGraphNode->id << "and" << closestNodeCandidate->id;
            }

            // Accept the new closest node.
            closestPoseGraphNodeBefore = closestPoseGraphNode;
            closestPoseGraphNode = closestNodeCandidate;
            if (config.debugLevel > 0)
                qDebug() << state.frameId << "New closest node:" << closestPoseGraphNode << "dist:" << closestPoseGraphNode->dist(inputPose);

            // Snap out of exploration mode when the closest node is not a leaf node.
            if (!closestPoseGraphNode->isLeaf())
            {
                if (config.debugLevel > 0 && explorationMode)
                    qDebug() << "The closest node is not a leaf node. Exploration mode off.";
                explorationMode = false;
            }
        }
    }

    // Gather the map lines near the input pose from the pose graph.
    // These active map lines are used for snapping, merging, expiring.
    // The "active" flag of the map lines are used for medium snapping,
    // so that the non-active lines can be selected faster.
    gatherActiveMapLines(closestPoseGraphNode);

    // The medium snap is used as an indicator for loop closing. The snapMedium() function
    // also produces a mediumSnapQuality indicator, confirmedLinePairs, and observer nodes.
    if (explorationMode)
        mediumSnappedPose = snapMedium(this->inputLines, inputPose, config.debugLevel > 10);

    // Use the local snap function to compute a snappedPose that aligns the input lines with the map
    // lines as good as it can. After this function, the confirmedLinePairs and unconfirmedInputLines
    // will be filled with data as needed for updating the map.
    localSnappedPose = snapLocal(this->inputLines, inputPose, config.debugLevel > 10);

    // Detect bad snaps and return the input pose without modifying the map.
    // Too many bad snaps lead to loss of localization.
    if (localSnappedPose.isNan())
    {
        qDebug() << state.frameId << "Bad snap detected! counter:" << badSnapCounter << "weak input:" << weakInput;

        if (!weakInput)
            badSnapCounter++;
        if (badSnapCounter > 10)
        {
            isLocalized = false; // lost our pose tracking.
            qDebug() << state.frameId << "Pose tracking lost. Relocalizing.";
        }

        return inputPose;
    }
    else
    {
        badSnapCounter = 0;
    }

    state.localizationTime = stopWatch.elapsedTimeMs();
    stopWatch.start();

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
    updatePolygonMap(localSnappedPose, reducedVisibilityPolygons, sensedPolygons, config.debugLevel > 10);

    // Loop closing.
    // Loop closing is detected when the medium snap successfully managed to snap without using the active lines.
    // A few sanity checks are the last defense against misdetections that mess up the entire map.
    Pose2D offset = mediumSnappedPose - localSnappedPose;
    if (mediumSnappedPose.confidence > 0.9 && fabs(offset.z) < PI2)
    {
        qDebug() << state.frameId << "Loop closing event detected. quality:" << mediumSnappedPose.confidence << "norm:" << (mediumSnappedPose-localSnappedPose).norm()
                 << "medium pose:" << mediumSnappedPose << "local pose:" << localSnappedPose;

        // Perform the loop-closing optimization.
        closeLoop(localSnappedPose, mediumSnappedPose);

        // Correct the last pose of the robot.
        localSnappedPose = mediumSnappedPose;
        this->inputPose = mediumSnappedPose;

        // Gather more active lines.
        activeMapLines.unify(closestPoseGraphNode->gatherNearbyLines(config.slamPoseGraphNeighborhoodSize));

        // A successfully closed loop turns the exploration mode off.
        mediumSnappedPose.setNull();
        explorationMode = false;
    }

    // Map line maintenance. Merge, expire and erase.
    mergeMapLines(config.debugLevel > 10);
    expireMapLines(config.debugLevel > 10);
    eraseMapLines(localSnappedPose, visibilityPolygon, reducedVisibilityPolygons, config.debugLevel > 10);

    state.mapUpdateTime = stopWatch.elapsedTimeMs();
    state.mapRam = memoryUsage();
    //printMemoryUsage();

    return localSnappedPose;
}

// Manually localizes the slam system.
void GeometricMap::localizeAt(const Pose2D& p)
{
    isLocalized = true;
    closestPoseGraphNode = getClosestNode(p);
}

// Clears a polygonal region from the map as described by pol.
void GeometricMap::clearPolygon(const Polygon &pol)
{
    ListIterator<PoseGraphNode> nodesIt = poseGraphNodes.begin();
    while (nodesIt.hasNext())
    {
        nodesIt.cur().observedNeighborhood.clipPolygons(pol);
        nodesIt.cur().observedNeighborhood.pruneOut(config.gmPolygonPruning);
        nodesIt.next();
    }

    polygonMap.clipPolygons(pol);
    polygonMap.dilate(config.gmPolygonDilation);
    polygonMap.pruneOut(config.gmPolygonPruning);
    polygonMap.simplify(config.gmDouglasPeuckerEpsilon);
    polygonMap.renumber();
    polygonMap.resetSearch();
}

// Fills a polygonal region in the map as described by pol.
void GeometricMap::fillPolygon(const Polygon &pol)
{
    ListIterator<PoseGraphNode> nodesIt = poseGraphNodes.begin();
    while (nodesIt.hasNext())
        nodesIt.next().observedNeighborhood.unite(pol);

    polygonMap.unite(pol);
    polygonMap.dilate(config.gmPolygonDilation);
    polygonMap.pruneOut(config.gmPolygonPruning);
    polygonMap.simplify(config.gmDouglasPeuckerEpsilon);
    polygonMap.renumber();
    polygonMap.resetSearch();
}

// Returns the polygons that make up the polygon map as a GeometricModel.
const GeometricModel &GeometricMap::getGeometricModel() const
{
    return polygonMap;
}

// Prints the amount of memory used by the map in kilobytes.
void GeometricMap::printMemoryUsage() const
{
    double toKB = 1.0 / 1024.0;

    qDebug() << state.frameId << "Memory usage:";
    qDebug() << "One map line: tl:" << sizeof(TrackedLine) << "l:" << sizeof(Line) << "vec of tl:" << sizeof(Vector<TrackedLine>) << "ll of nodes*:" << sizeof(LinkedList<PoseGraphNode*>);
    qDebug() << "One graph node: pn:" << sizeof(PoseGraphNode) << "ll of tl*:" << sizeof(LinkedList<TrackedLine*>) << "ll of node*:" << sizeof(LinkedList<PoseGraphNode*>);

    double mapLinesKB = double(sizeof(TrackedLine) * mapLines.size()) * toKB;
    double graphNodesKB = double(sizeof(PoseGraphNode) * poseGraphNodes.size()) * toKB;
    qDebug() << "Map lines:" << mapLinesKB << "kb lines:" << mapLines.size();
    qDebug() << "Graph Nodes:" << graphNodesKB << "kb nodes:" << poseGraphNodes.size();

    ListIterator<TrackedLine> it = mapLines.begin();
    uint observerCount = 0;
    while (it.hasNext())
        observerCount += it.next().observerNodes.size();
    double observerConnectionsKB = double(sizeof(PoseGraphNode*) * observerCount) * toKB;
    qDebug() << "Observer connections" << observerConnectionsKB << "kb observers:" << observerCount;

    ListIterator<PoseGraphNode> it2 = poseGraphNodes.begin();
    uint seenLineCount = 0;
    while (it2.hasNext())
        seenLineCount += it2.next().seenMapLines.size();
    double seenLineConnectionsKB = double(sizeof(TrackedLine*) * seenLineCount) * toKB;
    qDebug() << "Seen line connections" << seenLineConnectionsKB << "kb seen lines:" << seenLineCount;

    ListIterator<PoseGraphNode> it3 = poseGraphNodes.begin();
    uint neighborCount = 0;
    while (it3.hasNext())
        neighborCount += it3.next().neighbours.size();
    double neighborConnectionsKB = double(sizeof(PoseGraphNode*) * neighborCount) * toKB;
    qDebug() << "Neighbor connections" << neighborConnectionsKB << "kb neighbors" << neighborCount;

    double polygonVerticesKB = double(2*sizeof(double) * polygonMap.getVertexCount()) * toKB;
    qDebug() << "Polygon vertices:" << polygonVerticesKB << "kb vertices:" << polygonMap.getVertexCount();

    double totalKB = mapLinesKB + graphNodesKB + observerConnectionsKB + seenLineConnectionsKB + neighborConnectionsKB + polygonVerticesKB;
    qDebug() << "total:" << totalKB << "kb";
}

// Prints the amount of memory used by the map in kilobytes.
double GeometricMap::memoryUsage() const
{
    double toMB = 1.0 / (1024.0*1024.0);
    double mapLinesMB = double(sizeof(TrackedLine) * mapLines.size()) * toMB;
    double graphNodesMB = double(sizeof(PoseGraphNode) * poseGraphNodes.size()) * toMB;

    ListIterator<TrackedLine> it = mapLines.begin();
    uint observerCount = 0;
    while (it.hasNext())
        observerCount += it.next().observerNodes.size();
    double observerConnectionsMB = double(sizeof(PoseGraphNode*) * observerCount) * toMB;

    ListIterator<PoseGraphNode> it2 = poseGraphNodes.begin();
    uint seenLineCount = 0;
    while (it2.hasNext())
        seenLineCount += it2.next().seenMapLines.size();
    double seenLineConnectionsMB = double(sizeof(TrackedLine*) * seenLineCount) * toMB;

    ListIterator<PoseGraphNode> it3 = poseGraphNodes.begin();
    uint neighborCount = 0;
    while (it3.hasNext())
        neighborCount += it3.next().neighbours.size();
    double neighborConnectionsMB = double(sizeof(PoseGraphNode*) * neighborCount) * toMB;

    double polygonVerticesMB = double(2*sizeof(double) * polygonMap.getVertexCount()) * toMB;

    return mapLinesMB + graphNodesMB + observerConnectionsMB + seenLineConnectionsMB + neighborConnectionsMB + polygonVerticesMB;
}

// Searches the map for the shortest path from "from" to "to". In order to guarantee
// a path even in situations where "from" or "to" are occluded, both from and to are
// moved into free space. The starting point and the target of the search are provided
// as an argument to this function. The returned boolean indicates whether the path
// was successfully found. A path search can be unsuccessful when there really is no way from
// start to target in the map. After a successful search, you can retrieve the
// found path using the getPath() function.
bool GeometricMap::computeStaticPath(const Vec2 &from, const Vec2 &to, int debug)
{
    return polygonMap.computeStaticPath(from, to, debug);
}

// Returns the last computed path.
const Path &GeometricMap::getPath() const
{
    return polygonMap.getPath();
}

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
void GeometricMap::gatherActiveMapLines(PoseGraphNode* closestNode)
{
    ListIterator<TrackedLine*> it = activeMapLines.begin();
    while (it.hasNext())
        it.next()->active = false;
    activeMapLines = closestNode->gatherNearbyLines(config.slamPoseGraphNeighborhoodSize);
    it = activeMapLines.begin();
    while (it.hasNext())
        it.next()->active = true;
}

// Returns pointers to the map lines that collide with the given box.
LinkedList<TrackedLine *> GeometricMap::boxSelectMapLines(const Box &box) const
{
    LinkedList<TrackedLine*> intersectingMapLines;
    ListIterator<TrackedLine> it = mapLines.begin(); // There could be many map lines.
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
    // 1. Compute strict nearest pairs between input lines and map lines.
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
    Vector<LinePair> linePairs = computeNearestLinePairs(inputLines, inputPose, debug);
    if (linePairs.isEmpty())
    {
        qDebug() << state.frameId << "No local pairs found!";
        localSnappedPose.setNan(); // Localization failed.
        return localSnappedPose;
    }

    if (debug)
    {
        qDebug() << state.frameId << "All local line pairs:" << linePairs.size();
        qDebug() << &linePairs;
    }

    // 2. Extract the largest consensus set from the line pairs with respect to rotation.
    // This is to remove outliers. In the local snap setting where all input lines are
    // close to their paired map line, we can assume a unimodal distribution of the
    // rotation with perhaps a few outliers.
    linePairs = rotationConsensus(linePairs, config.slamClusteringAngleEps);

    if (debug)
    {
        qDebug() << state.frameId << "Rotation angle sample consensus:";
        qDebug() << &linePairs;
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

    if (debug)
        qDebug() << "Avg rotation:" << avgRotation;

    // Update the rotation of the line pairs in the consensus set.
    localSnappedPose.turn(avgRotation);
    for (uint i = 0; i < linePairs.size(); i++)
        linePairs[i].inputPose = localSnappedPose;


    // 4. Build pairs of line pairs to acquire a set of hypotheses of what the transformation could be.
    // The hypotheses are split into two sets: proper hypotheses and parallel hypotheses.
    // Sometimes, only parallel hypotheses are present, e.g. when driving along a corridor, and
    // localization is only possible with the help of odometry.
    HypothesisSet properHypothesisSet; // fully qualified
    HypothesisSet parallelHypothesisSet;
    TransformationHypothesis th;
    for (uint i = 0; i < linePairs.size(); i++)
    {
        for (uint j = i+1; j < linePairs.size(); j++)
        {
            LinePair& lpi = linePairs[i];
            LinePair& lpj = linePairs[j];

            // Compute a translation hypothesis from line pairs i and j.
            if (th.computeTranslation(lpi, lpj, debug)) // returns false if the pair of pairs is invalid
            {
                // In a local snap, the transformation cannot be very large.
                // Discard the hypothesis if it suggests a too large of a pose jump.
                // This constraint can only be applied when local snapping.
                if (th.tr().norm() > config.slamMaxPoseDiff)
                {
                    if (debug)
                        qDebug() << "Discarding hyp" << lpi.inputLine->id << lpi.mapLine->id << "and" << lpj.inputLine->id << lpj.mapLine->id << "because of the pose diff limit." << th.tr().norm();
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
        localSnappedPose.setNan(); // Localization failed.
        return localSnappedPose;
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
        properHypothesisSet.discardOutliers(config.slamClusteringTransformEps);
        if (debug)
        {
            qDebug() << state.frameId << "Hypothesis consensus set:";
            qDebug() << &properHypothesisSet;
        }

        // 10. Settle on the average translation of the hypothesis consensus set.
        // Compute the weighted average transformation of the consensus set.
        //Pose2D avgTransform = computeAvgTransform(properHypothesisSet);
        localSnappedPose = properHypothesisSet.avgTransform()+localSnappedPose;

        if (debug)
            qDebug() << "Weighted avg transform:" << properHypothesisSet.avgTransform() << localSnappedPose;

        // 11. Build a set of confirmed line pairs that appear in the consensus set.
        confirmedLinePairs = properHypothesisSet.getConfirmedLinePairs();
        if (debug)
            qDebug() << state.frameId << "Confirmed line pairs:" << &confirmedLinePairs;
    }


    // 4. Build the set of unconfirmed input lines.
    unconfirmedInputLines.clear();
    for (uint i = 0; i < inputLines.size(); i++)
    if (!inputLines[i].active)
        unconfirmedInputLines << inputLines[i] + localSnappedPose;

    if (debug)
        qDebug() << state.frameId << "Unconfirmed input lines:" << &unconfirmedInputLines;

    if (debug)
        qDebug() << "Returning snapped pose:" << localSnappedPose  << "diff:" << localSnappedPose-inputPose << (localSnappedPose-inputPose).norm();

    return localSnappedPose;
}

// Attempts global localization in the map. The highest requirement on the global localization function
// is that it should never be wrong about having found a unique pose in the map. It is okay if the function
// returns a low globalSnapQuality and a faulty pose as long as if it does return a pose with high certainty,
// it is always right. Global localization fails when all input lines are parallel, e.g. when driving along
// a straight corridor without any features. The function writes zero globalSnapQuality when it fails due to
// a weak input, insufficient certainty of localization or ambiguous poses in the map.
// The inputLines are expected in local coordinates. The returned Pose2D is a pose in world coordintes.
Pose2D GeometricMap::snapGlobal(Vector<TrackedLine> &inputLines, bool debug)
{
    // Global snapping happens like so:
    // 1. Pair all input lines with all map lines. Only few pairs can be discarded based on length considerations.
    // 2. Build pairs of line pairs to acquire a set of hypotheses of what the transformation of
    // the entire scan could be. Since one input - map line pair only gives information about the rotation
    // and then the orthogonal distance between the lines, but leaves uncertainty along the map line, two
    // non-parallel pairs are needed to determine all parameters of the transformation (x,y,theta).
    // 3. Given the large set of hypotheses determined from all input - map line pairs of pairs combinations, a
    // clustering algorithm is used to group similar hypotheses together. The clusters then need to be split into
    // conflict free subclusters where one input line can only be assigned to one map line. Then, the clusters are
    // pruned so that only the best evaluated cluster remains in one place and the others are removed. Also, too
    // small clusters and clusters without a quality threshold are erased as they are too uncertain.
    // 4. Compute a final transformation from the consensus set of the computed hypotheses and assess the quality
    // of the global snap by the confirmed input length over the total input length.

    globalSnappedPose.setNull();

    if (debug)
        qDebug() << state.frameId << "Global snapping.";

    // Discard weak input cases.
    double totalInputLength = 0;
    for (uint i = 0; i < inputLines.size(); i++)
        totalInputLength += inputLines[i].length();
    if (inputLines.size() < 5 || totalInputLength < 5.0)
    {
        //if (debug)
            qDebug() << state.frameId << "Global localization failed due to weak input.";
        return globalSnappedPose;
    }

    // 1. Build pairs between input lines and map lines.
    // The pairs are made from all combinations of input lines and map lines.
    // Only few pairs can be discarded based on length considerations.
    // The line pairs come with weights that are computed based on the lengths of the involved lines.
    // The inputPose of all linePairs remains zero.
    Vector<LinePair> linePairs = computeAllLinePairs(inputLines);
    if (linePairs.isEmpty())
    {
        //if (debug)
            qDebug() << state.frameId << "Global localization failed. No line pairs found!";
        return globalSnappedPose;
    }

    if (false && debug)
    {
        qDebug() << state.frameId << "All global line pairs:" << linePairs.size();
        qDebug() << linePairs;
    }


    // 2. Build pairs of line pairs to acquire a set of hypotheses of what the transformation
    // of the globalSnapPose could be. In this process, the hypotheses are split into two sets:
    // parallel hypotheses and proper hypotheses. For global snapping, only proper hypotheses
    // are used.
    HypothesisSet properHypothesisSet;
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
                if (!th.isParallel) // Parallel hyps are excluded from global localization.
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
        // No global snapping is possible.

        //if (debug)
            qDebug() << state.frameId << "Global localization failed. No proper hypothesis set found!";
        return globalSnappedPose; // Should be zero.
    }

    // 3. Given the large set of hypotheses determined from all input - map line combinations, we use a
    // clustering algorithm to group similar hypotheses together and to select the largest cluster: the
    // consensus set. Special care needs to be taken to eliminate conflicting hypotheses stemming from
    // multiple lines in the same place resulting from drift and sensor noise. Two hypotheses are
    // conflicting when the same input line is assigned to different map lines.

    // Cluster the proper hypothesis set.
    Vector<HypothesisSet> hypsClusters = properHypothesisSet.cluster(config.slamClusteringTransformEps);
    if (debug)
        qDebug() << state.frameId << "Hyp clusters:" << &hypsClusters;

    // Remove conflicted hyps from the clusters by splitting the clusters into conflict-free subclusters.
    // Two hypotheses are in conflict when the same input line is assigned to different map lines.
    Vector<HypothesisSet> hypsClustersBuffer;
    for (uint m = 0; m < hypsClusters.size(); m++)
        hypsClustersBuffer << hypsClusters[m].splitConflictFreeSubClusters();

    // Compute the snap qualities.
    for (uint i = 0; i < hypsClustersBuffer.size(); i++)
        hypsClustersBuffer[i].snapQuality(inputLines);

    // Remove too small and low quality clusters.
    hypsClusters.clear();
    for (uint i = 0; i < hypsClustersBuffer.size(); i++)
        if (hypsClustersBuffer[i].size() >= 5 && hypsClustersBuffer[i].snapQuality(inputLines) > 0.5)
            hypsClusters << hypsClustersBuffer[i];

    // No more hyps left? Global snap failed.
    if (hypsClusters.isEmpty())
    {
        //if (debug)
            qDebug() << state.frameId << "Global localization failed. No good hypotheses left.";
        return globalSnappedPose;
    }

    if (false && debug)
        qDebug() << state.frameId << "Split Hyp clusters:" << &hypsClusters;

    // Sort by quality. The best clusters are sorted to the begining.
    hypsClusters.sort(-1);
    if (false && debug)
        qDebug() << state.frameId << "Sorted split clusters:" << &hypsClusters;

    // Discard all but the clusters of best quality.
    if (hypsClusters.size() > 5)
        hypsClusters.resize(5);
    if (false && debug)
        qDebug() << state.frameId << "Sorted, quality-pruned clusters:" << hypsClusters;


    // Unify clusters by their inter cluster distance by replacing two that are close to
    // each other by the one with the better snap quality.
    hypsClustersBuffer.clear();
    for (int i = 0; i < hypsClusters.size(); i++)
    {
        for (int j = hypsClusters.size()-1; j > i; j--)
        {
            Pose2D pi = hypsClusters[i].avgTransform();
            Pose2D pj = hypsClusters[j].avgTransform();
            double qi = hypsClusters[i].snapQuality(inputLines);
            double qj = hypsClusters[j].snapQuality(inputLines);
            //qDebug() << i << j << (pi.dist(pj) < config.slamClusteringTransformEps) << qi << qj;
            if (pi.dist(pj) < config.slamClusteringTransformEps)
            {
                if (qi < qj)
                    hypsClusters[i] = hypsClusters[j];
                hypsClusters.remove(j);
            }
        }
    }

    if (debug)
        qDebug() << state.frameId << "Sorted, quality-pruned, and unified clusters:" << &hypsClusters;

    // Do we still have multiple clusters with a decent snap quality?
    if (hypsClusters.size() > 1)
    {
        // Yes? Then we have ambiguous hypotheses and cannot properly localize.
        qDebug() << state.frameId << "Global localization failed. Consensus ambiguity could not be resolved.";
        if (debug)
            qDebug() << &hypsClusters;

        for (uint i = 0; i < hypsClusters.size(); i++)
            poseHypotheses << hypsClusters[i].avgTransform();

        return globalSnappedPose;
    }

    // One distinct cluster is clearly the winner. This cluster is the consensus set. We can be fairly sure
    // that it is the correct unique pose of the robot in the map. Uncertainty remains only when a wrong pose
    // produces a really good snap and the correct pose does not. How would that possibly even happen?
    if (debug)
        qDebug() << state.frameId << "Consensus set successfully extracted.";

    // 4. Compute the weighted average transformation of the consensus set and let that be our snap transform.
    // Evaluate the quality of the global snap. The snap quality is expressed as successfully overlapped input
    // length divided by the total input length. The snap quality approaches 1 if most input lines were part
    // of a hypothesis in the consensus set.
    globalSnappedPose = hypsClusters[0].avgTransform();
    globalSnappedPose.confidence = hypsClusters[0].snapQuality(inputLines);

    // Print out the input lines and the consensus set.
    if (debug)
    {
        qDebug() << "input lines:" << &inputLines;
        qDebug() << hypsClusters[0];
    }

    //if (debug)
        qDebug() << state.frameId << "Returning global snap:" << globalSnappedPose << "quality:" << globalSnappedPose.confidence
                 << "cluster size:" << hypsClusters[0].size()
                 << "input size:" << inputLines.size() << "input length:" << totalInputLength;

    return globalSnappedPose;
}

// Attempts medium range localization inside the selection box and returns a hopefully localized Pose2D.
// Medium range localization fails when all input lines are parallel, e.g. when driving along a straight
// corridor without any features. The function returns a zero pose when the localization fails. The
// function also writes the mediumSnapQuality variable that indicates the quality of the snap. The quality
// is zero when the snap fails and it's 1 when the snap is perfect, i.e. all input lines have been paired
// and fully covered. The inputLines are expected in local coordinates relative to inputPose. The returned
// Pose2D is a pose in world coordintes.
Pose2D GeometricMap::snapMedium(Vector<TrackedLine> &inputLines, const Pose2D &inputPose, bool debug)
{
    // Medium snapping happens exactly the same as the global snap, except we preselect map lines
    // with a box constraint.
    // 1. Pair input lines with map lines. The pairs are made from all combinations of input lines
    // and map lines. Only a few pairs can be discarded based on length considerations.
    // 2. Build pairs of line pairs to acquire a set of hypotheses of what the transformation of
    // the entire scan could be. Since one input - map line pair only gives information about the rotation
    // and then the orthogonal distance between the lines, but leaves uncertainty along the map line, two
    // pairs are needed to determine all parameters of the transformation (x,y,theta).
    // 3. Compute a final transformation from the consensus set of the computed hypotheses.
    // 4. Build a set of confirmed line pairs from the input and map lines that could be matched.
    // 5. Assess the quality of the snap by the confirmed input length over the total input length.

    mediumSnappedPose.setNull();

    if (debug)
        qDebug() << state.frameId << "Medium snapping.";


    // Discard weak input cases.
    double totalInputLength = 0;
    for (uint i = 0; i < inputLines.size(); i++)
        totalInputLength += inputLines[i].length();
    if (inputLines.size() < 5 || totalInputLength < 5.0)
    {
        if (debug)
            qDebug() << state.frameId << "Loop closing detection failed due to weak input.";
        return mediumSnappedPose;
    }

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
    HypothesisSet properHypothesisSet;
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
                if (!th.isParallel) // When medium snapping, parallel pairs are excluded.
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
        // This usually happens when all input lines are parallel.
        // No medium snapping is possible.

        if (debug)
            qDebug() << state.frameId << "No proper medium hypothesis set found!";
        return mediumSnappedPose; // Should be zero.
    }

    // 3. Given the large set of hypotheses determined from all input - map line combinations, we use a
    // clustering algorithm to group similar hypotheses together and to select the largest cluster: the
    // consensus set. Special care needs to be taken to eliminate conflicting hypotheses stemming from
    // multiple lines in the same place resulting from drift and sensor noise. Two hypotheses are
    // conflicting when the same input line is assigned to different map lines.

    // Cluster the proper hypothesis set.
    Vector<HypothesisSet> hypsClusters = properHypothesisSet.cluster(config.slamClusteringTransformEps);
    if (debug)
        qDebug() << state.frameId << "Hyp clusters:" << &hypsClusters;

    // Remove conflicted hyps from the clusters by splitting the clusters into conflict-free subclusters.
    // Two hypotheses are in conflict when the same input line is assigned to different map lines.
    Vector<HypothesisSet> hypsClustersBuffer;
    for (uint m = 0; m < hypsClusters.size(); m++)
        hypsClustersBuffer << hypsClusters[m].splitConflictFreeSubClusters();

    // Compute the snap qualities.
    for (uint i = 0; i < hypsClustersBuffer.size(); i++)
        hypsClustersBuffer[i].snapQuality(inputLines);

    // Remove too small and low quality clusters.
    hypsClusters.clear();
    for (uint i = 0; i < hypsClustersBuffer.size(); i++)
        if (hypsClustersBuffer[i].size() >= 5 && hypsClustersBuffer[i].snapQuality(inputLines) > 0.5)
            hypsClusters << hypsClustersBuffer[i];

    // No more hyps left? Medium snap failed.
    if (hypsClusters.isEmpty())
    {
        if (debug)
            qDebug() << state.frameId << "Medium snap failed. No good hypotheses left.";
        return mediumSnappedPose;
    }

    if (false && debug)
        qDebug() << state.frameId << "Split Hyp clusters:" << &hypsClusters;

    // Sort by quality. The best clusters are sorted to the begining.
    hypsClusters.sort(-1);
    if (false && debug)
        qDebug() << state.frameId << "Sorted split clusters:" << &hypsClusters;

    // Discard all but the clusters of best quality.
    if (hypsClusters.size() > 5)
        hypsClusters.resize(5);
    if (false && debug)
        qDebug() << state.frameId << "Sorted, quality-pruned clusters:" << hypsClusters;


    // Unify clusters by their inter cluster distance by replacing two that are close to
    // each other by the one with the better snap quality.
    hypsClustersBuffer.clear();
    for (int i = 0; i < hypsClusters.size(); i++)
    {
        for (int j = hypsClusters.size()-1; j > i; j--)
        {
            Pose2D pi = hypsClusters[i].avgTransform();
            Pose2D pj = hypsClusters[j].avgTransform();
            double qi = hypsClusters[i].snapQuality(inputLines);
            double qj = hypsClusters[j].snapQuality(inputLines);
            //qDebug() << i << j << (pi.dist(pj) < config.slamClusteringTransformEps) << qi << qj;
            if (pi.dist(pj) < config.slamClusteringTransformEps)
            {
                if (qi < qj)
                    hypsClusters[i] = hypsClusters[j];
                hypsClusters.remove(j);
            }
        }
    }

    if (debug)
        qDebug() << state.frameId << "Sorted, quality-pruned, and unified clusters:" << &hypsClusters;

    // Do we still have multiple clusters with a good snap quality?
    if (hypsClusters.size() > 1)
    {
        // Yes? Then we have ambiguous hypotheses and cannot properly localize.
        if (debug)
        {
            qDebug() << state.frameId << "Medium snap failed. Consensus ambiguity could not be resolved.";
            qDebug() << &hypsClusters;
        }

        for (uint i = 0; i < hypsClusters.size(); i++)
            poseHypotheses << hypsClusters[i].avgTransform();

        return mediumSnappedPose;
    }

    // One cluster is clearly the winner. That means it managed to match the most lines and
    // is probably the best choice.
    if (debug)
        qDebug() << state.frameId << "Consensus set successfully extracted.";

    // 4. Compute the weighted average transformation of the consensus set and let that be our snap transform.
    // 6. Evaluate the quality of the snap.
    // The snap quality is expressed as successfully overlapped input length divided by the total input length.
    // The snap quality approaches 1 if most input lines were part of a hypothesis in the consensus set.
    mediumSnappedPose = hypsClusters[0].avgTransform();
    mediumSnappedPose.confidence = hypsClusters[0].snapQuality(inputLines);

    // Print out the input lines and the consensus set.
    if (false && debug)
    {
        qDebug() << "input lines:" << &inputLines;
        qDebug() << hypsClusters[0];
    }

    //if (debug)
        qDebug() << state.frameId << "Returning medium snap:" << mediumSnappedPose << "quality:" << mediumSnappedPose.confidence
                 << "cluster size:" << hypsClusters[0].size()
                 << "input size:" << inputLines.size() << "input length:" << totalInputLength;
        poseHypotheses << mediumSnappedPose;


    // 4. Build a set of confirmed line pairs that appear in the consensus set.
    Vector<TransformationHypothesis> hyps = hypsClusters[0].getHyps();
    confirmedLinePairs.clear();
    for (uint i = 0; i < hyps.size(); i++)
    {
        TransformationHypothesis& t = hyps[i];

        if (!t.linePair1->confirmed)
        {
            confirmedLinePairs << *t.linePair1;
            confirmedLinePairs.last().inputPose = mediumSnappedPose;
        }
        if (!t.linePair2->confirmed)
        {
            confirmedLinePairs << *t.linePair2;
            confirmedLinePairs.last().inputPose = mediumSnappedPose;
        }
        t.linePair1->confirmed = true;
        t.linePair2->confirmed = true;
    }
    if (debug)
    {
        qDebug() << state.frameId << "Input lines:" << &inputLines;
        qDebug() << state.frameId << "Confirmed medium line pairs:" << &confirmedLinePairs;
    }

    // Gather the observer nodes from the confirmed line pairs that contributed to the
    // medium snap. We need these nodes to identify the closest pose graph node in a loop
    // closing situation, and also to gather the map lines that need merging after
    // closing the loop.
    mediumSnapObservers.clear();
    for (uint i = 0; i < confirmedLinePairs.size(); i++)
        mediumSnapObservers.unify(confirmedLinePairs[i].mapLine->observerNodes);
    //qDebug() << "Observers:" << observers;

    if (debug)
        qDebug() << "Returning medium snap:" << mediumSnappedPose << "quality:" << mediumSnappedPose.confidence;

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
    // building function, where the actual pose of the robot the pairs are seen from can only
    // deviate from the inputPose by a limited amount defined by the bounds applied to the
    // line-pose-distance. We are picking pairs only from the active map lines in the close
    // neighbourhood of the agent in order to avoid having to cycle through all lines in the map.

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
            // Input lines can only be longer than map lines by a small amount that occurs when the robot is
            // exploring a new area and it extends a partially seen line with new information. The amount of
            // extension depends strongly on the velocity of the robot and on the frame rate. Map lines,
            // however, can be significantly longer than input lines, for example in situations where only a
            // small portion of a long wall is observed through an open door.
            double lengthDiff = inputLine.length()-mapLine->length();
            if (lengthDiff > config.slamPairingMaxLengthDeviation)
            {
                if (debug)
                    qDebug() << "Discarding pair" << inputLine.id << mapLine->id << "due to length deviation" << lengthDiff << "lengths:" << inputLine.length() << mapLine->length();
                continue;
            }

            // Line-pose-dist.
            double cost = mapLine->linePoseDist(inputLine, inputPose);
            if (nearestMapLineCost == 0 || cost < nearestMapLineCost)
            {
                nearestMapLineCost = cost;
                nearestMapLine = mapLine;
                if (debug)
                {
                    qDebug() << "New nearest pair" << inputLine.id << mapLine->id  << "cost:" << nearestMapLineCost;
                    mapLine->linePoseDist(inputLine, inputPose, debug);
                }
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
// We never want to global snap in the active snap, so active lines are ignored.
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
        while (mapLineIterator.hasNext())
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

            if (debug)
                qDebug() << state.frameId << "Merging" << mapLine1->id << "and" << mapLine2->id << "lld:" << mapLine1->lineLineDist(*mapLine2);
            if (mapLine1->addLineObservation(*mapLine2, mapLine2->totalWeight, debug))
            {
                if (debug)
                    qDebug() << state.frameId << "Merged map lines" << mapLine1->id << "and" << mapLine2->id;
                mapLine1->observerNodes.unify(mapLine2->observerNodes);
                ListIterator<PoseGraphNode*> it = mapLine2->observerNodes.begin();
                while (it.hasNext())
                    it.next()->seenMapLines.removeOne(mapLine2);
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
void GeometricMap::eraseMapLines(const Pose2D& pose, const Polygon& visibilityPolygon, const Vector<Polygon>& reducedVisibilityPolygons, bool debug)
{
    // Erase map lines that no longer exist.
    // A negatively inflated and range-bounded visibility polygon is used to erase lines.
    // Without the negative inflation, the map lines would randomly intersect with the boundary
    // of the visibility polygon. For some reason I don't understand, offsetting with the Clipper
    // library works best on a polygon in local coordinates. Multiple polygons can be the
    // result of the negative offsetting. Only the lines in the active line set can be erased,
    // otherwise we would erase old but correct map lines shortly before loop closing situations.

    Vector<Polygon> reducedVisibilityPolygonsTr = reducedVisibilityPolygons + pose;

    // Test every mapLine in the active map lines set if it intersects with the reduced visibility polygon.
    Vector<TrackedLine> lineBuffer;
    ListIterator<TrackedLine*> it = activeMapLines.begin();
    while (it.hasNext())
    {
        const TrackedLine& mapLine = *it.cur();

        bool intersects = false;
        for (uint j = 0; j < reducedVisibilityPolygonsTr.size(); j++)
        {
            if (reducedVisibilityPolygonsTr[j].intersects(mapLine.p1())
                    || reducedVisibilityPolygonsTr[j].intersects(mapLine.p2())
                    || reducedVisibilityPolygonsTr[j].intersects(mapLine))
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
                    if (clippedLines[k].length() > config.laserLineMinLength)
                    {
                        lineBuffer << TrackedLine(clippedLines[k], state.frameId);
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

// Clusters the vector of TransformationHypothesis with the DBScan algorithm using eps as
// the distance threshold. A Vector of Vectors is returned, one Vector for each cluster.
const Vector<Vector<TransformationHypothesis> >& GeometricMap::hypClusters(const Vector<TransformationHypothesis>& hyps, double eps, bool debug)
{
    bool clustered[hyps.size()];
    for (uint i = 0; i < hyps.size(); i++)
        clustered[i] = false;

    static Vector<uint> N; // neighborhood queue
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

        // Queue all item i's eps neighbors (except itself) into the neighborhood queue.
        N.clear();
        for (uint j = i+1; j < hyps.size(); j++)
        {
            if (!clustered[j] && hyps[j].dist(hyps[i]) < eps)
            {
                N << j;
                if (debug)
                    qDebug() << " Queueing direct neighbor" << j << hyps[j] << "dist:" << hyps[j].dist(hyps[i]);
            }
            else if (debug)
                qDebug() << " Skipping direct neighbor" << j << hyps[j] << "dist:" << hyps[j].dist(hyps[i]);
        }

        // Work through the neighborhood queue and keep queueing all epsilon-reachable neighbors of neighbors.
        for (uint k = 0; k < N.size(); k++)
        {
            if (clustered[N[k]])
                continue;

            clustered[N[k]] = true;
            cluster << hyps[N[k]];
            if (debug)
                qDebug() << "  Adding neighbor" << N[k] << hyps[N[k]] << "dist:" << hyps[N[k]].dist(hyps[i]);
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

    // Distance-based pose graph node creation.
    if (poseGraphNodes.isEmpty() || closestPoseGraphNode->dist(pose) > config.slamPoseGraphNodeDist)
    {
        // Add a new node to the poseGraph when the closest node is out of reach and connect it with all
        // currently seen lines. This also sets the robot into exploration mode.

        addPoseGraphNode(pose, seenMapLines);
        closestPoseGraphNodeBefore = closestPoseGraphNode;
        closestPoseGraphNode = &poseGraphNodes.last();
        explorationMode = true;
        if (debug)
            qDebug() << state.frameId << "New pose graph node added" << poseGraphNodes.last();
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
void GeometricMap::updatePolygonMap(const Pose2D& pose, const Vector<Polygon>& reducedVisibilityPolygons, const LinkedList<Polygon> sensedPolygons, bool debug)
{
    // The update procedure of the polygon map is somewhat complicated. Because of loop closing, the world
    // map is actually maintained as a union of many smaller maps (GeometricModels) attached to the pose
    // graph nodes. We call a local map of a pose graph node the observed neighborhood of a node. Therefore,
    // an update operation is performed only on the local map of the closest pose graph node in every iteration
    // and the world map is updated as a union of the world map so far and the neighborhood of the closest
    // node. In the event of loop closing, the entire world map is recomputed as the union of all neighborhoods.
    // The update of an observed neighborhood bears further complications. The update is performed using the
    // reduced visibility polygon(s) and the sensed polygons as input. The reduction of the visibility
    // polygon happens by the same amount as the dilation of sensed polygons for the same reason: expansion
    // for path planning. Path planning needs to regard blocked space with a certain amount of dilation so
    // that the robot can be regarded as a point. The union of the reduced visibility polygons produces the
    // root polygon(s) we are actually interested in as a map. The union of many visibility
    // polygons can produce CW holes in the otherwise CCW root polygons. These holes represent obstacles that
    // stand in the map. The observed dilated polygons are in close relation to the visibility polygon as at
    // the time of the observation, the observed dilated polygons mark the boundary of the visibility polygon.
    // In the meanest environments, these dilated polygons at the boundary of the map stem from clutter that
    // can only be 9unreliably observed and so it often happens that missed observations result in too large
    // visibility polygons that falsly expand the boundary of the map with cluttered regions ending up being
    // regarded as free space. Therefore, we keep the observed dilated polygons and clip the edges of the
    // root polygons with them. This way, unfortunately, clutter cannot be erased by observation when it does
    // disappear in the real world, but this works well enough for now. An even more complicated algorithm is
    // needed to fully automate the updates.

    // As the world map is only ever used for path planning, there is no need for other than reduced root
    // polygons and dilated polygons. Collision checking with actual polygons are handeled in the local map.

    closestPoseGraphNode->observedNeighborhood.observationUpdate(reducedVisibilityPolygons+pose, sensedPolygons+pose);
    LinkedList<PoseGraphNode *> nearbyNeighborhoods = closestPoseGraphNode->gatherNeighborhood(config.slamPoseGraphNeighborhoodSize);
    ListIterator<PoseGraphNode*> it = nearbyNeighborhoods.begin();
    while (it.hasNext())
    {
        it.cur()->observedNeighborhood.clipPolygons(reducedVisibilityPolygons + pose);
        it.cur()->observedNeighborhood.pruneOut(config.gmPolygonPruning);
        it.next();
    }

    polygonMap.unite(closestPoseGraphNode->observedNeighborhood);
    polygonMap.pruneOut(config.gmPolygonPruning);
    polygonMap.clipPolygons(reducedVisibilityPolygons + pose);
    polygonMap.pruneOut(config.gmPolygonPruning);
    polygonMap.dilate(config.gmPolygonDilation);
    polygonMap.pruneOut(config.gmPolygonPruning);
    polygonMap.simplify(config.gmDouglasPeuckerEpsilon);
    polygonMap.renumber();
    polygonMap.resetSearch();
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
    return getClosestNode(pose)->gatherNearbyLines(config.slamPoseGraphNeighborhoodSize);
}

// Gathers pointers to the nodes close to the pose.
LinkedList<PoseGraphNode *> GeometricMap::gatherNearbyNodes(const Pose2D &pose) const
{
    return getClosestNode(pose)->gatherNeighborhood(config.slamPoseGraphNeighborhoodSize);
}

// Returns the Euklidean closest PoseGraphNode to pose p.
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
    double closestDist = mediumSnapObservers[0]->dist(mediumSnap);
    for (uint i = 1; i < mediumSnapObservers.size(); i++)
    {
        double dist = mediumSnapObservers[i]->dist(mediumSnap);
        if (dist < closestDist)
        {
            closestIdx = i;
            closestDist = dist;
        }
    }

    // The three things needed for the loop closing: root node, leaf node, and offset.
    PoseGraphNode* rootNode = mediumSnapObservers[closestIdx];
    PoseGraphNode* leafNode = closestPoseGraphNode;
    Pose2D offset = ((mediumSnap - localSnap) + leafNode->pose) - rootNode->pose;

    qDebug() << state.frameId << "PoseGraph::closeLoop()";
    qDebug() << "  root node:" << rootNode->id;
    qDebug() << "  leaf node:" << leafNode->id;
    qDebug() << "  offset:" << (mediumSnap - localSnap) << offset << "norm:" << (mediumSnap-localSnap).norm();


    // Optimize the graph between the root node and the leaf node to account for the loop closing offset.
    // This applies a small pose offset for every node along the way from the root to leaf so that in
    // total, the leaf node moves by the loop closing offset.
    optimizeGraph(rootNode, leafNode, offset);


    StopWatch sw;
    sw.start();

    // Connect the root node with the leaf node.
    leafNode->neighbours.unify(rootNode);
    rootNode->neighbours.unify(leafNode);

    // Line map update.
    // Update the map lines according to the offset of the associated graph nodes.
    // For now, this iterates over all map lines, but perhaps this could be optimized later, e.g.
    // by gathering pointers to the relevant lines when determining the path from root to leaf.
    ListIterator<TrackedLine> mapLinesIt = mapLines.begin();
    while (mapLinesIt.hasNext())
    {
        TrackedLine& mapLine = mapLinesIt.next();
        Line l = (mapLine-mapLine.observerNodes.first()->lastPose) + mapLine.observerNodes.first()->pose;
        TrackedLine tl(l, true, true, state.frameId);
        ListIterator<PoseGraphNode*> observerIterator = mapLine.observerNodes.begin();
        while (observerIterator.hasNext())
        {
            PoseGraphNode* pgn = observerIterator.next();
            tl.addLineObservation((mapLine-pgn->lastPose)+pgn->pose); // Can pose transformations be chained into one operation?
        }
        mapLine.setTo(tl);
    }

    double tt = sw.elapsedTimeMs();
    qDebug() << "loop close line map update:" << tt;
    sw.start();


    // Polygon map update.
    // Update the observed neighborhood geometric models according to the offset of the associated graph nodes.
    // For now this iterates over all graph nodes, but this could be optimized later.
    ListIterator<PoseGraphNode> nodeIterator = poseGraphNodes.begin();
    while (nodeIterator.hasNext())
    {
        PoseGraphNode& pgn = nodeIterator.next();
        pgn.observedNeighborhood -= pgn.lastPose; // should be possible to collate
        pgn.observedNeighborhood += pgn.pose;
    }

    // Recompute the polygonal map by computing the union over the observed neighborhoods of all nodes.
    Vector<Polygon> roots;
    Vector<Polygon> pols;
    nodeIterator = poseGraphNodes.begin();
    while (nodeIterator.hasNext())
    {
        //polygonMap.unite(nodeIterator.next().observedNeighborhood);
        roots << nodeIterator.cur().observedNeighborhood.getRootPolygons();
        pols << nodeIterator.cur().observedNeighborhood.getPolygons();
        nodeIterator.next();
    }
    polygonMap.clear();
    polygonMap.unite(roots, pols);
    polygonMap.dilate(config.gmPolygonDilation);
    polygonMap.pruneOut(config.gmPolygonPruning);
    polygonMap.simplify(config.gmDouglasPeuckerEpsilon);
    polygonMap.renumber();
    polygonMap.resetSearch();

    tt = sw.elapsedTimeMs();
    qDebug() << "loop close polygon update:" << tt;


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
// This function will update the poses of the graph nodes.
void GeometricMap::optimizeGraph(PoseGraphNode* rootNode, PoseGraphNode* leafNode, const Pose2D& offset)
{
    // The graph optimization procedure is carried out according to:
    // http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf

    StopWatch sw;

    // TODO
    // This is a little cheat using the iterator directly on the LinkedList of the graph nodes.
    // This will have to be replaced by a path from root to leaf computed using the neighbors of the nodes.
    ListIterator<PoseGraphNode> rootIterator = poseGraphNodes.iteratorAt(rootNode);
    ListIterator<PoseGraphNode> leafIterator = poseGraphNodes.iteratorAt(leafNode);
    qDebug() << "Optimizing graph from node" << rootIterator.cur().id << "to node" << leafIterator.cur().id;

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
    ListIterator<PoseGraphNode> it = rootIterator;
    while (it != leafIterator)
    {
        //qDebug() << "Constraint from" << it.cur().id << "to:" << it.peekNext().id;
        GraphConstraint newEdge;
        newEdge.addOdomConstraint(it.cur(), it.peekNext());
        graphConstraints.push_back(newEdge);
        it.next();
    }

    // The special loop closing constraint.
    GraphConstraint newEdge;
    newEdge.addLoopClosingConstraint(*rootNode, *leafNode, offset);
    graphConstraints.push_back(newEdge);



    // Create the linear system.
    uint dim = 3 * graphConstraints.size();

    Eigen::VectorXd x(dim);
    Eigen::VectorXd b(dim);
    Eigen::VectorXd dx(dim);
    Eigen::MatrixXd H(dim,dim);
    Eigen::SparseMatrix<double> HSparse;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;

    // We set the initial guess for x to the current pose of the graph nodes.
    it = rootIterator;
    uint i = 0;
    while (it != leafIterator)
    {
        const Pose2D& pose = it.cur().pose;
        x(3*i) = pose.x;
        x(3*i+1) = pose.y;
        x(3*i+2) = pose.z;
        //qDebug() << "building x i" << i << "graph node" << it.cur().id << "x:" << x(3*i) << x(3*i+1) << x(3*i+2);
        it.next();
        i++;
    }
    x(3*i) = it.cur().pose.x;
    x(3*i+1) = it.cur().pose.y;
    x(3*i+2) = it.cur().pose.z;

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

            edge.linearizeConstraint();
            uint i = edge.i-rootIterator.cur().id;
            uint j = edge.j-rootIterator.cur().id;

            //qDebug() << "H update i j" << i << j << "edge" << edge.i << edge.j;

            H.block<3, 3>(3*i, 3*i) += edge.getAij().transpose() * omega * edge.getAij();
            H.block<3, 3>(3*i, 3*j) += edge.getAij().transpose() * omega * edge.getBij();
            H.block<3, 3>(3*j, 3*i) += edge.getBij().transpose() * omega * edge.getAij();
            H.block<3, 3>(3*j, 3*j) += edge.getBij().transpose() * omega * edge.getBij();

            b.block<3, 1>(3*i, 0) += edge.getAij().transpose() * omega * edge.getErrorVector();
            b.block<3, 1>(3*j, 0) += edge.getBij().transpose() * omega * edge.getErrorVector();
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
        it = rootIterator;
        i = 0;
        while (it != leafIterator)
        {
            Pose2D& pose = it.cur().pose;
            pose.x = x(3*i);
            pose.y = x(3*i+1);
            pose.z = x(3*i+2);

            //qDebug() << "updateGraphPoses i" << i << "node" << it.cur().id << "dx:" << dx(3*i)<< dx(3*i+1)<< dx(3*i+2);

            it.next();
            i++;
        }
        Pose2D& pose = it.cur().pose;
        pose.x = x(3*i);
        pose.y = x(3*i+1);
        pose.z = x(3*i+2);

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
void GeometricMap::draw(QPainter *painter) const
{
    // Draw the polygon map.
    if (command.showPolygonMap > 0)
    {
        // 0 - nothing
        // 1 - map polygon
        // 2 - observed neighborhood
        // 3 - both
        // 4 - polygons and the nearby observed neighborhoods

        if (command.showPolygonMap == 1)
            polygonMap.draw(painter, drawUtil.pen, drawUtil.brushGray, drawUtil.brushLightGray);

        if (command.showPolygonMap == 2 && closestPoseGraphNode != 0)
            closestPoseGraphNode->observedNeighborhood.draw(painter, drawUtil.pen, drawUtil.brushGray, drawUtil.brushLightGray);

        if (command.showPolygonMap == 3)
        {
            polygonMap.draw(painter, drawUtil.pen, drawUtil.brushGray, drawUtil.brushLightGray);
            if (closestPoseGraphNode != 0)
                closestPoseGraphNode->observedNeighborhood.draw(painter, drawUtil.pen, drawUtil.brushGray, drawUtil.brushLightGray);
        }

        if (command.showPolygonMap == 4)
        {
            if (closestPoseGraphNode != 0)
            {
                LinkedList<PoseGraphNode *> nn = closestPoseGraphNode->gatherNeighborhood(config.slamPoseGraphNeighborhoodSize);
                ListIterator<PoseGraphNode*> nnit = nn.begin();
                while (nnit.hasNext())
                    nnit.next()->observedNeighborhood.draw(painter, drawUtil.pen, drawUtil.brushGray, drawUtil.brushLightGray);
            }
        }

        if (command.showPolygonMap == 5)
        {
            polygonMap.draw(painter, drawUtil.pen, drawUtil.brushGray, drawUtil.brushLightGray);
            if (closestPoseGraphNode != 0)
            {
                LinkedList<PoseGraphNode *> nn = closestPoseGraphNode->gatherNeighborhood(config.slamPoseGraphNeighborhoodSize);
                ListIterator<PoseGraphNode*> nnit = nn.begin();
                while (nnit.hasNext())
                    nnit.next()->observedNeighborhood.draw(painter, drawUtil.pen, drawUtil.brushGray, drawUtil.brushLightGray);
            }
        }
    }

    // Draw the line map.
    if (command.showLineMap > 0)
    {
        // 0 - nothing
        // 1 - line matching view
        // 2 - line map view
        // 3 - line observation accumulation view

        // Line matching view.
        if (command.showLineMap == 1)
        {
            // First all map lines in light gray.
            ListIterator<TrackedLine> it = mapLines.begin();
            while (it.hasNext())
                it.next().draw(painter, drawUtil.penLightGrayThin);

            // Loop closing detection with the selection box and the selected line set.
            if (explorationMode)
            {
                // The selection box.
                Box selectionBox(inputPose.pos(), config.slamSelectionBoxSize, -config.slamSelectionBoxSize, -config.slamSelectionBoxSize, config.slamSelectionBoxSize);
                selectionBox.draw(painter, drawUtil.penLightGrayThin);

                // The selected line set in gray.
                it = mapLines.begin();
                while (it.hasNext())
                {
                    if (selectionBox.intersects(it.cur()))
                        it.cur().draw(painter, drawUtil.penGray);
                    it.next();
                }

                // The selected line set labels in gray.
                if (command.showLabels)
                {
                    it = mapLines.begin();
                    while (it.hasNext())
                    {
                        if (selectionBox.intersects(it.cur()))
                            it.cur().drawLabel(painter, drawUtil.penGray);
                        it.next();
                    }
                }
            }

            // The active map line set in black.
            ListIterator<TrackedLine*> it2 = activeMapLines.begin();
            while (it2.hasNext())
                it2.next()->draw(painter, drawUtil.pen);

            // The input lines in blue relative to inputPose.
            painter->save();
            painter->setTransform(inputPose, true);
            for (uint i = 0; i < inputLines.size(); i++)
                inputLines[i].draw(painter, drawUtil.penBlueThick);
            painter->restore();

            // Prospected input lines in red relative to local snapped pose.
            painter->save();
            painter->setTransform(localSnappedPose, true);
            for (uint j = 0; j < inputLines.size(); j++)
                inputLines[j].draw(painter, drawUtil.penRedThick);
            painter->restore();

            // The input pose in blue.
            drawUtil.drawNoseCircle(painter, inputPose, drawUtil.penBlue, drawUtil.brushBlue, 0.3*config.agentRadius);

            // The local snapped pose in red.
            drawUtil.drawNoseCircle(painter, localSnappedPose, drawUtil.penRed, drawUtil.brushRed, 0.3*config.agentRadius);

            // Line matching labels.
            if (command.showLabels)
            {
                // All map line labels in light gray.
                ListIterator<TrackedLine> it = mapLines.begin();
                while (it.hasNext())
                    it.next().drawLabel(painter, drawUtil.penLightGray);

                // The active map line set labels in black.
                ListIterator<TrackedLine*> it2 = activeMapLines.begin();
                while (it2.hasNext())
                    it2.next()->drawLabel(painter, drawUtil.pen);

                // Input line labels in blue.
                painter->save();
                painter->setTransform(inputPose, true);
                for (uint i = 0; i < inputLines.size(); i++)
                    inputLines[i].drawLabel(painter, drawUtil.penBlue, 1.0, -inputPose.heading());
                painter->restore();
            }
        }

        // Bare line map view.
        if (command.showLineMap == 2)
        {
            // All map lines in black.
            ListIterator<TrackedLine> it = mapLines.begin();
            while (it.hasNext())
                it.next().draw(painter, drawUtil.penThick);

            // Bare line map labels.
            if (command.showLabels)
            {
                ListIterator<TrackedLine> it = mapLines.begin();
                while (it.hasNext())
                    it.next().drawLabel(painter, drawUtil.pen, 0.8);
            }
        }

        // Line observation accumulation view.
        if (command.showLineMap == 3)
        {
            // The accumulated lines in thin black.
            ListIterator<TrackedLine> it = mapLines.begin();
            while (it.hasNext())
            {
                const TrackedLine& l = it.next();
                for (uint j = 0; j < l.lineObservations.size(); j++)
                    l.lineObservations[j].draw(painter, drawUtil.penThin);
            }

            // All map lines in thick blue.
            it = mapLines.begin();
            while (it.hasNext())
                it.next().draw(painter, drawUtil.penBlueThick);
        }
    }

    // Draw the pose graph.
    if (command.showPoseGraph > 0)
    {
        // 0 - none
        // 1 - the nodes and the closest neighborhood
        // 2 - neighborhood seen line connections
        // 3 - all seen line connections

        QColor activeColor(qRgba(0, 255, 0, 255));
        QColor inactiveColor(qRgba(0, 255, 0, 200));
        inactiveColor.setRgba(qRgba(0, 255, 0, 200));

        // The seen line connections of all nodes.
        if (command.showPoseGraph >= 3)
        {
            ListIterator<PoseGraphNode> it = poseGraphNodes.begin();
            while (it.hasNext())
                it.next().drawSeenLineConnections(painter, drawUtil.penLightGrayThin);
        }

        // The seen line connections of the nearby nodes.
        if (command.showPoseGraph >= 2 && closestPoseGraphNode != 0)
        {
            LinkedList<PoseGraphNode*> nearbyNodes = closestPoseGraphNode->gatherNeighborhood(config.slamPoseGraphNeighborhoodSize);
            ListIterator<PoseGraphNode*> it = nearbyNodes.begin();
            while (it.hasNext())
                it.next()->drawSeenLineConnections(painter, drawUtil.penLightGrayThin);
        }

        // All nodes.
        ListIterator<PoseGraphNode> it = poseGraphNodes.begin();
        while (it.hasNext())
        {
            ListIterator<PoseGraphNode*> nit = it.cur().neighbours.begin();
            while (nit.hasNext())
                Line(it.cur().pose.pos(), nit.next()->pose.pos()).draw(painter, drawUtil.penGray);
            it.next().draw(painter, drawUtil.penThin, drawUtil.brushGreen, 0.5*config.agentRadius);
        }

        // The neighborhood of the closest node.
        if (closestPoseGraphNode != 0)
        {
            closestPoseGraphNode->drawNeighborhood(painter, config.slamPoseGraphNeighborhoodSize, drawUtil.pen, drawUtil.brushGreen, 0.6*config.agentRadius);

            // Mark the closest node with a red circle.
            painter->save();
            painter->translate(closestPoseGraphNode->pose.pos());
            painter->setPen(drawUtil.penRedThick);
            painter->setBrush(Qt::NoBrush);
            painter->drawEllipse(QPointF(), 0.8*config.agentRadius, 0.8*config.agentRadius);
            painter->restore();
        }

        // Pose graph node labels.
        if (command.showLabels)
        {
            ListIterator<PoseGraphNode> it = poseGraphNodes.begin();
            while (it.hasNext())
                it.next().drawLabel(painter, drawUtil.pen, 0.8);
        }
    }

    // The pose hypotheses.
    for (uint i = 0; i < poseHypotheses.size(); i++)
        drawUtil.drawNoseCircle(painter, poseHypotheses[i], drawUtil.pen, drawUtil.brushRed, 0.15);
}

// Draws the visibility graph in an OpenGL context.
void GeometricMap::drawVisibilityGraph() const
{
    polygonMap.drawVisibilityGraph();
}

// Draws the visibility graph on a QPainter.
void GeometricMap::drawVisibilityGraph(QPainter *painter) const
{
    polygonMap.drawVisibilityGraph(painter);
}

// Draws the GeometricMap in OpenGL context.
void GeometricMap::draw() const
{
    if (command.showLineMap == 0)
        return;

    if (command.showPolygonMap > 0)
    {
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

    // Line matching view.
    if (command.showLineMap == 1)
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
        GLlib::setColor(drawUtil.lightGray);
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

    // Bare line map view.
    if (command.showLineMap == 2)
    {
        // All map lines in black.
        glColor3f(0,0,0); // black
        glLineWidth(5);
        ListIterator<TrackedLine> it = mapLines.begin();
        while (it.hasNext())
            it.next().draw();
    }

    // Line observation accumulation view.
    if (command.showLineMap == 3)
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

    if (command.showPoseGraph > 0)
    {
        QColor activeColor(qRgba(0, 255, 0, 255));
        QColor inactiveColor(qRgba(0, 255, 0, 200));
        inactiveColor.setRgba(qRgba(0, 255, 0, 200));

        // The seen line connections of all nodes.
        if (command.showPoseGraph >= 3)
        {
            glLineWidth(1);
            GLlib::setColor(drawUtil.lightGray);
            ListIterator<PoseGraphNode> it = poseGraphNodes.begin();
            while (it.hasNext())
                it.next().drawSeenLineConnections();

            glTranslated(0, 0, 0.001);
        }

        // The seen line connections of the nearby nodes.
        if (command.showPoseGraph >= 2 && closestPoseGraphNode != 0)
        {
            LinkedList<PoseGraphNode*> nearbyNodes = closestPoseGraphNode->gatherNeighborhood(config.slamPoseGraphNeighborhoodSize);
            ListIterator<PoseGraphNode*> it = nearbyNodes.begin();
            glLineWidth(1);
            GLlib::setColor(drawUtil.lightGray);
            while (it.hasNext())
                it.next()->drawSeenLineConnections();

            glTranslated(0, 0, 0.001);
        }

        // All nodes.
        ListIterator<PoseGraphNode> it = poseGraphNodes.begin();
        while (it.hasNext())
        {
            glLineWidth(1);
            GLlib::setColor(drawUtil.gray);
            ListIterator<PoseGraphNode*> nit = it.cur().neighbours.begin();
            while (nit.hasNext())
                Line(it.cur().pose.pos(), nit.next()->pose.pos()).draw();
            it.next().draw(inactiveColor, 0.5*config.agentRadius);
        }

        glTranslated(0, 0, 0.001);

        if (closestPoseGraphNode != 0)
        {
            // The neighborhood of the closest node.
            closestPoseGraphNode->drawNeighborhood(config.slamPoseGraphNeighborhoodSize, activeColor, 0.6*config.agentRadius);

            // Mark the closest node with a red circle.
            GLlib::drawCircle(closestPoseGraphNode->pose, drawUtil.red, 0.8*config.agentRadius, 0.02);
        }
    }

    if (globalSnappedPose.confidence >= 0.75)
    {
        glPushMatrix();
        glTranslated(0, 0, 0.01);
        GLlib::drawNoseCircle(globalSnappedPose, Qt::red, 0.5*config.agentRadius);
        glPopMatrix();
    }
}

// Writes the LineMap into a data stream.
void GeometricMap::streamOut(QDataStream &out) const
{
    // Renumber the map lines.
    uint id = 1;
    ListIterator<TrackedLine> lineIt = mapLines.begin();
    while (lineIt.hasNext())
        lineIt.next().id = id++;

    // Renumber the pose graph nodes.
    id = 1;
    ListIterator<PoseGraphNode> nodesIt = poseGraphNodes.begin();
    while (nodesIt.hasNext())
        nodesIt.next().id = id++;

    // Save the map lines and the pose graph nodes first.
    out << mapLines;
    out << poseGraphNodes;

    // Save the observer node assigments of the map lines.
    // We save the ids of the observer nodes that we convert
    // back to pointers when the file is loaded.
    lineIt = mapLines.begin();
    while (lineIt.hasNext())
    {
        const TrackedLine& tl = lineIt.next();

        Vector<uint> ids;
        ListIterator<PoseGraphNode*> it = tl.observerNodes.begin();
        while (it.hasNext())
            ids << it.next()->id;
        out << ids;
        //qDebug() << "line" << tl.id << "nodes:" << ids;
    }

    // Save the neighbor and seen line assignments of the pose graph nodes.
    // We save the ids of the neighbors and the ids of the seen lines and recreate
    // pointers to objects from them when the file is loaded.
    nodesIt = poseGraphNodes.begin();
    while (nodesIt.hasNext())
    {
        const PoseGraphNode& pn = nodesIt.next();

        Vector<uint> ids;
        ListIterator<PoseGraphNode*> it = pn.neighbours.begin();
        while (it.hasNext())
            ids << it.next()->id;
        out << ids;

        //qDebug() << "node" << pn.id << "neighbors:" << ids;

        ids.clear();
        ListIterator<TrackedLine*> it2 = pn.seenMapLines.begin();
        while (it2.hasNext())
            ids << it2.next()->getId();
        out << ids;

        //qDebug() << "node" << pn.id << "lines:" << ids;
    }

    // Save the polygon map.
    out << polygonMap;
}

// Reads the LineMap from a data stream.
void GeometricMap::streamIn(QDataStream &in)
{
    in >> mapLines;
    in >> poseGraphNodes;

    // Load the observer node assignments of the map lines.
    ListIterator<TrackedLine> lineIt = mapLines.begin();
    while (lineIt.hasNext())
    {
        TrackedLine& tl = lineIt.next();

        Vector<uint> ids;
        in >> ids;
        //qDebug() << "line" << tl.id << "nodes:" << ids;
        for (uint i = 0; i < ids.size(); i++)
        {
            ListIterator<PoseGraphNode> nodeIt = poseGraphNodes.begin();
            while (nodeIt.hasNext())
            {
                if (nodeIt.cur().id == ids[i])
                {
                    tl.observerNodes << &nodeIt.cur();
                    break;
                }
                nodeIt.next();
            }
        }
    }

    // Load the neighbor and the seen lines assignments of the pose graph nodes.
    ListIterator<PoseGraphNode> nodeIt = poseGraphNodes.begin();
    while (nodeIt.hasNext())
    {
        PoseGraphNode& pn = nodeIt.next();

        Vector<uint> ids;
        in >> ids;
        //qDebug() << "node" << pn.id << "neighbors:" << ids;
        for (uint i = 0; i < ids.size(); i++)
        {
            ListIterator<PoseGraphNode> nodeIt2 = poseGraphNodes.begin();
            while (nodeIt2.hasNext())
            {
                if (nodeIt2.cur().id == ids[i])
                {
                    pn.neighbours << &nodeIt2.cur();
                    break;
                }
                nodeIt2.next();
            }
        }

        ids.clear();
        in >> ids;
        //qDebug() << "node" << pn.id << "seen lines:" << ids;
        for (uint i = 0; i < ids.size(); i++)
        {
            ListIterator<TrackedLine> lineIt2 = mapLines.begin();
            while (lineIt2.hasNext())
            {
                if (lineIt2.cur().id == ids[i])
                {
                    pn.seenMapLines << &lineIt2.cur();
                    break;
                }
                lineIt2.next();
            }
        }
    }

    in >> polygonMap;
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

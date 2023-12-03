#ifndef GEOMETRICMAP_H_
#define GEOMETRICMAP_H_

#include "lib/globals.h"
#include "lib/util/Vector.h"
#include "lib/util/Pose2D.h"
#include "lib/geometry/Polygon.h"
#include "TrackedLine.h"
#include "LinePair.h"
#include "PoseGraphNode.h"
#include "TransformationHypothesis.h"

class GeometricMap
{
    Pose2D inputPose; // The assumed pose in the map given to us last (odometry input).
    Vector<TrackedLine> inputLines; // The lines observed in the inputPose (laser input).

    LinkedList<TrackedLine> mapLines; // These lines make up the actual line map.
    LinkedList<PoseGraphNode> poseGraphNodes; // The nodes of the pose graph.
    GeometricModel polygonMap; // The polygons of the polygon map.

    bool isLocalized; // Is the agent localized in the map?
    bool explorationMode; // Is the agent exploring new territory? Loop closing can only happen in exploration mode.
    int badSnapCounter; // Counts bad snaps that lead to loss of localization.

    Pose2D localSnappedPose; // The corrected pose in the map (output).
    Pose2D globalSnappedPose; // The result of the global snap. (output)
    Pose2D mediumSnappedPose; // The result of the medium snap. (output)

    Vector<Pose2D> poseHypotheses; // Only for visualization.

    Vector<LinePair> confirmedLinePairs; // The confirmed line pairs that were used for the last snap.
    Vector<TrackedLine> unconfirmedInputLines; // The input lines that were not paired in the end.
    Vector<PoseGraphNode*> mediumSnapObservers; // The pose graph nodes observing a medium snap.
    LinkedList<TrackedLine*> activeMapLines; // A set of map lines visibile right now.
    PoseGraphNode* closestPoseGraphNode; // The currently closest graph node.
    PoseGraphNode* closestPoseGraphNodeBefore;

public:

    GeometricMap();
    ~GeometricMap(){}

    void clear();
    bool isEmpty() const;

    Pose2D slam(const Pose2D &inputPose, const Vector<TrackedLine> &inputLines, const Polygon& visibilityPolygon, const LinkedList<Polygon> sensedPolygons);

    void localizeAt(const Pose2D& p);

    void clearPolygon(const Polygon& pol);
    void fillPolygon(const Polygon& pol);

    const GeometricModel& getGeometricModel() const;
    void exportMap() const;
    void printMemoryUsage() const;
    double memoryUsage() const;

    bool computeStaticPath(const Vec2 &from, const Vec2 &to, int debug=0);
    const Path &getPath() const;

    // Drawing methods.
    void draw() const;
    void draw(QPainter* painter) const;
    void drawVisibilityGraph() const;
    void drawVisibilityGraph(QPainter* painter) const;

    void streamOut(QDataStream &out) const;
    void streamIn(QDataStream &in);

private:

    // Snap functions.
    Pose2D snapLocal(Vector<TrackedLine>& inputLines, const Pose2D &inputPose, bool debug=false);
    Pose2D snapGlobal(Vector<TrackedLine> &inputLines, bool debug=false);
    Pose2D snapMedium(Vector<TrackedLine> &inputLines, const Pose2D &inputPose, bool debug=false);
    Pose2D cls(Vector<LinePair>& linePairs, bool debug=false);

    // Pair computations.
    Vector<LinePair> computeNearestLinePairs(Vector<TrackedLine> &inputLines, const Pose2D &inputPose, bool debug=false);
    Vector<LinePair> computeAllLinePairs(Vector<TrackedLine> &inputLines, bool debug=false);
    Vector<LinePair> computeBoxLinePairs(Vector<TrackedLine> &inputLines, const Pose2D &inputPose, bool debug=false);

    // Map lines maintenance.
    LinkedList<TrackedLine*> updateMapLines(bool debug=false);
    void eraseMapLines(const Pose2D &inputPose, const Polygon& visibilityPolygon, const Vector<Polygon> &reducedVisibilityPolygons, bool debug=false);
    void expireMapLines(bool debug=false);
    void mergeMapLines(bool debug=false);
    void addLine(const TrackedLine& l);

    // Active line set.
    void gatherActiveMapLines(PoseGraphNode *closestNode);
    LinkedList<TrackedLine*> boxSelectMapLines(const Box& box) const;
    LinkedList<TrackedLine *> gatherNearbyMapLines(const Pose2D &pose) const;

    // Consensus analysis functions.
    const Vector<Vector<TransformationHypothesis> > &hypClusters(const Vector<TransformationHypothesis>& hyps, double eps, bool debug=false);
    Vector<LinePair> rotationConsensus(const Vector<LinePair>& linePairs, double eps, bool debug=false);
    Vector<LinePair> translationConsensus(const Vector<LinePair>& linePairs, double eps, bool debug=false);

    // Pose Graph functions.
    void updatePoseGraph(const Pose2D& pose, const LinkedList<TrackedLine *> &seenMapLines, bool debug=false);
    void addPoseGraphNode(const Pose2D &pose, const LinkedList<TrackedLine *> &seenMapLines);
    void removePoseGraphNode(PoseGraphNode *pgn);
    void removeLineFromPoseGraph(TrackedLine* o);
    PoseGraphNode* getClosestNode(const Pose2D& p, bool debug=false) const;
    LinkedList<PoseGraphNode*> gatherNearbyNodes(const Pose2D &pose) const;

    // Polygon map functions.
    void updatePolygonMap(const Pose2D& pose, const Vector<Polygon> &reducedVisibilityPolygons, const LinkedList<Polygon> sensedPolygons, bool debug=false);

    // Loop closing functions.
    void closeLoop(const Pose2D& localSnap, const Pose2D &mediumSnap);
    void optimizeGraph(PoseGraphNode *rootNode, PoseGraphNode *leafNode, const Pose2D &offset);
};

QDebug operator<<(QDebug dbg, const GeometricMap &w);
QDataStream& operator<<(QDataStream& out, const GeometricMap &o);
QDataStream& operator>>(QDataStream& in, GeometricMap &o);

#endif

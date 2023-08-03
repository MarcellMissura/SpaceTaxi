#ifndef GEOMETRICMAP_H_
#define GEOMETRICMAP_H_

#include "globals.h"
#include "lib/util/Vector.h"
#include "lib/util/Pose2D.h"
#include "lib/geometry/Polygon.h"
#include "TrackedLine.h"
#include "LinePair.h"
#include "PoseGraphNode.h"
#include "TransformationHypothesis.h"

class GeometricMap
{

public:

    // These are public so that the labels can be drawn in the OpenGL widget.

    Pose2D inputPose; // The assumed pose in the map given to us last (input).
    Vector<TrackedLine> inputLines; // The lines observed in the inputPose (input).

    LinkedList<TrackedLine> mapLines; // These lines make up the actual line map.
    LinkedList<PoseGraphNode> poseGraphNodes; // The nodes of the pose graph.
    GeometricModel polygonMap; // The polygons of the polygon map.

private:

    Pose2D localSnappedPose; // The corrected pose in the map (output).
    Pose2D globalSnappedPose; // The result of the global snap. (output)
    Pose2D mediumSnappedPose; // The result of the medium snap. (output)
    Pose2D savedSnappedPose; // hack for the bad frames in the dataset
    double globalSnapQuality; // Quality indicator for the global snap.
    double mediumSnapQuality; // Quality indicator for the medium snap.

    Vector<LinePair> confirmedLinePairs; // The confirmed line pairs that were used for the last snap.
    Vector<TrackedLine> unconfirmedInputLines; // The input lines that were not paired in the end.
    Vector<PoseGraphNode*> observers; // The pose graph nodes observing a snap.
    LinkedList<TrackedLine*> activeMapLines; // A set of map lines visibile right now.
    PoseGraphNode* closestPoseGraphNode; // The currently closest graph node.
    PoseGraphNode* closestPoseGraphNodeBefore;

public:

    GeometricMap();
    ~GeometricMap(){}

    void clear();
    bool isEmpty() const;

    Pose2D slam(const Pose2D &inputPose, const Vector<TrackedLine> &inputLines, const Polygon& visPol);

    void printMemoryUsage() const;
    void exportMap() const;

    // Drawing methods.
    void draw() const;
    void draw(QPainter* painter, const QPen &pen, const QBrush &brush, double opacity=0.5) const;

    void streamOut(QDataStream &out) const;
    void streamIn(QDataStream &in);

private:

    // Snap functions.
    Pose2D snapLocal(Vector<TrackedLine>& inputLines, const Pose2D &inputPose, bool debug=false);
    Pose2D snapGlobal(Vector<TrackedLine> &inputLines, bool debug=false);
    Pose2D snapMedium(Vector<TrackedLine> &inputLines, const Pose2D &inputPose, bool debug=false);

    // Pair computations.
    Vector<LinePair> computeNearestLinePairs(Vector<TrackedLine> &inputLines, const Pose2D &inputPose, bool debug=false);
    Vector<LinePair> computeAllLinePairs(Vector<TrackedLine> &inputLines, bool debug=false);
    Vector<LinePair> computeBoxLinePairs(Vector<TrackedLine> &inputLines, const Pose2D &inputPose, bool debug=false);

    // Map lines maintenance.
    LinkedList<TrackedLine*> updateMapLines(bool debug=false);
    void eraseMapLines(const Pose2D &inputPose, const Polygon &visPol, bool debug=false);
    void expireMapLines(bool debug=false);
    void mergeMapLines(bool debug=false);
    void addLine(const TrackedLine& l);

    // Active line set.
    void gatherActiveMapLines(const Pose2D& pose);
    LinkedList<TrackedLine*> gatherRecentMapLines() const;
    LinkedList<TrackedLine*> boxSelectMapLines(const Box& box) const;
    LinkedList<TrackedLine *> gatherNearbyMapLines(const Pose2D &pose) const;

    // Consensus analysis functions.
    const Vector<Vector<TransformationHypothesis> > &hypClusters(const Vector<TransformationHypothesis>& hyps, double eps, bool debug=false);
    Vector<TransformationHypothesis> hypConsensus(Vector<TransformationHypothesis> &hyps, double eps, bool debug=false);
    Vector<LinePair> rotationConsensus(const Vector<LinePair>& linePairs, double eps, bool debug=false);
    Vector<LinePair> translationConsensus(const Vector<LinePair>& linePairs, double eps, bool debug=false);

    // Pose Graph-related functions.
    void updatePoseGraph(const Pose2D& pose, const LinkedList<TrackedLine *> &seenMapLines, bool debug=false);
    void addPoseGraphNode(const Pose2D &pose, const LinkedList<TrackedLine *> &seenMapLines);
    void removePoseGraphNode(PoseGraphNode *pgn);
    void removeLineFromPoseGraph(TrackedLine* o);
    PoseGraphNode* getClosestNode(const Pose2D& p, bool debug=false) const;
    LinkedList<PoseGraphNode*> gatherNearbyNodes(const Pose2D &pose) const;

    // Polygon map functions.
    void updatePolygonMap(const Pose2D& pose, const Polygon &visPol, bool debug=false);

    // Loop closing functions.
    void closeLoop(const Pose2D& localSnap, const Pose2D &mediumSnap);
    void optimizeGraph(PoseGraphNode *rootNode, PoseGraphNode *leafNode, const Pose2D &offset);

    // Drawing functions.
    void drawGlobalSnap() const;
    void drawLineMatching() const;
    void drawMapLines() const;
    void drawLineAccumulation() const;
    void drawPoseGraph() const;
    void drawPolygonMap() const;
};

QDebug operator<<(QDebug dbg, const GeometricMap &w);
QDataStream& operator<<(QDataStream& out, const GeometricMap &o);
QDataStream& operator>>(QDataStream& in, GeometricMap &o);

#endif

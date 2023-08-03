#ifndef POSEGRAPHNODE_H_
#define POSEGRAPHNODE_H_

#include "lib/geometry/GeometricModel.h"
#include "lib/geometry/Polygon.h"
#include "lib/util/Pose2D.h"
#include "TrackedLine.h"

class PoseGraphNode
{
public:

    uint id;
    Pose2D pose;
    Pose2D lastPose;
    GeometricModel observedNeighborhood; // A union of the visibility polygons that can be attributed to this node.
    LinkedList<TrackedLine*> seenMapLines; // Lines that were seen by this graph node.
    LinkedList<PoseGraphNode*> neighbours; // Neighboring nodes.

    PoseGraphNode();
    PoseGraphNode(const Pose2D& p);
    ~PoseGraphNode(){}

    double dist(const PoseGraphNode& pgn) const;
    double dist(const Pose2D& p) const;

    void setPose(const Pose2D& p);

    bool isJunction() const;
    bool isLeaf() const;

    const LinkedList<TrackedLine*>& getSeenLines() const;

    LinkedList<PoseGraphNode*> gatherNeighborhood(uint depth=1);
    LinkedList<TrackedLine*> gatherNearbyLines(uint depth=1);

    void draw(QPainter* painter, const QPen& pen=QPen(), const QBrush& brush=QBrush(), double radius=0.1) const;
    void drawLabel(QPainter* painter, const QPen& pen=QPen(), double opacity=1.0) const;
    void draw(QColor color = QColor("yellow"), double radius=0.1) const;
    void drawSeenLineConnections(QPainter* painter, const QPen& pen=QPen()) const;
    void drawSeenLineConnections() const;
    void drawNeighborhood(QPainter* painter, uint depth=1, const QPen& pen=QPen(), const QBrush& brush=QBrush(), double radius=0.1) const;
    void drawNeighborhood(uint depth=1, QColor color = QColor("yellow"), double radius=0.1) const;

    void streamOut(QDataStream &out) const;
    void streamIn(QDataStream &in);
};

QDebug operator<<(QDebug dbg, const PoseGraphNode &n);
QDebug operator<<(QDebug dbg, const PoseGraphNode* n);
QDataStream& operator<<(QDataStream& out, const PoseGraphNode &o);
QDataStream& operator>>(QDataStream& in, PoseGraphNode &o);

#endif

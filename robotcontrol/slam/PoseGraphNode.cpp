#include "PoseGraphNode.h"
#include "lib/util/GLlib.h"
#include "lib/util/ColorUtil.h"

PoseGraphNode::PoseGraphNode()
{
    id = 0;
}

PoseGraphNode::PoseGraphNode(const Pose2D &p)
{
    id = 0;
    pose = p;
}

// Returns the distance between the PoseGraphNode pgn and this node.
// The distance between two nodes is measured by the Euklidean distance in xy space.
double PoseGraphNode::dist(const PoseGraphNode &pgn) const
{
    return pose.distxy(pgn.pose);
}

// Returns the distance between the Pose2D p and this node.
// The distance is measured by the Euklidean distance in xy space.
double PoseGraphNode::dist(const Pose2D &p) const
{
    return pose.distxy(p);
}

// Sets the pose of the graph node.
void PoseGraphNode::setPose(const Pose2D &p)
{
    pose = p;
}

// Returns true if this node is a junction, i.e. it has more than two neighbors.
bool PoseGraphNode::isJunction() const
{
    return (neighbours.size() > 2);
}

// Returns true if this node is a leaf node, i.e. it has only one neighbor.
// A leaf node is at the end of a dangling path that hasn't been loop closed yet.
bool PoseGraphNode::isLeaf() const
{
    return (neighbours.size() < 2);
}

// Returns pointers to the graph nodes that can be reached within a range=depth of hops from this node.
LinkedList<PoseGraphNode *> PoseGraphNode::gatherNeighborhood(uint depth)
{
    LinkedList<PoseGraphNode *> nh;
    nh << this;

    if (depth > 0)
    {
        ListIterator<PoseGraphNode*> it = neighbours.begin();
        while (it.hasNext())
            nh.unify(it.next()->gatherNeighborhood(depth-1));
    }

    return nh;
}

// Returns pointers to all map lines as seen by the neighborhood of range=depth around this node.
LinkedList<TrackedLine *> PoseGraphNode::gatherNearbyLines(uint depth)
{
    LinkedList<TrackedLine *> nl;
    LinkedList<PoseGraphNode *> nn = gatherNeighborhood(depth);
    ListIterator<PoseGraphNode*> it = nn.begin();
    while (it.hasNext())
        nl.unify(it.next()->seenMapLines);
    return nl;
}

// Writes the LineMap into a data stream.
void PoseGraphNode::streamOut(QDataStream &out) const
{
    out << id;
    out << pose;
    out << lastPose;
    //out << seenMapLines;
}

// Reads the LineMap from a data stream.
void PoseGraphNode::streamIn(QDataStream &in)
{
    in >> id;
    in >> pose;
    in >> lastPose;
    //in >> seenMapLines;
}

void PoseGraphNode::draw(QColor color, double radius) const
{
    GLlib::drawNoseCircle(pose, color, radius);
}

void PoseGraphNode::drawSeenLineConnections() const
{
    ListIterator<TrackedLine*> it = seenMapLines.begin();
    while (it.hasNext())
        Line(pose.pos(), it.next()->center()).draw();
}

void PoseGraphNode::drawNeighborhood(uint depth, QColor color, double radius) const
{
    if (depth > 0)
    {
        glLineWidth(2);
        GLlib::setColor(drawUtil.black);
        ListIterator<PoseGraphNode*> it = neighbours.begin();
        while (it.hasNext())
        {
            Line(pose.pos(), it.next()->pose.pos()).draw();
            it.cur()->drawNeighborhood(depth-1, color, radius);
        }
    }

    draw(color, radius);
}

QDebug operator<<(QDebug dbg, const PoseGraphNode &n)
{
    Vector<uint> lids;
    ListIterator<TrackedLine*> lit = n.seenMapLines.begin();
    while (lit.hasNext())
        lids << lit.next()->id;
    Vector<uint> nids;
    ListIterator<PoseGraphNode*> nit = n.neighbours.begin();
    while (nit.hasNext())
        nids << nit.next()->id;
    dbg << "id:" << n.id << "pose:" << n.pose << "lines seen:" << &lids << "neighbors:" << &nids;
    return dbg;
}

QDebug operator<<(QDebug dbg, const PoseGraphNode* n)
{
    Vector<uint> lids;
    ListIterator<TrackedLine*> lit = n->seenMapLines.begin();
    while (lit.hasNext())
        lids << lit.next()->id;
    Vector<uint> nids;
    ListIterator<PoseGraphNode*> nit = n->neighbours.begin();
    while (nit.hasNext())
        nids << nit.next()->id;
    dbg << "id:" << n->id << "pose:" << n->pose << "lines seen:" << &lids << "neighbors:" << &nids;
    return dbg;
}

QDataStream& operator<<(QDataStream& out, const PoseGraphNode &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, PoseGraphNode &o)
{
    o.streamIn(in);
    return in;
}

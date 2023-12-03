#include "PoseGraphNode.h"
#include "lib/util/GLlib.h"
#include "lib/util/DrawUtil.h"

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

// Returns the distance between the Pose2D p and the pose of this node.
double PoseGraphNode::poseDist(const Pose2D &p) const
{
    return pose.dist(p);
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

// Returns a reference to the map lines seen from this node.
const LinkedList<TrackedLine *> &PoseGraphNode::getSeenLines() const
{
    return seenMapLines;
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

// Draws the pose graph node as a nose circle on QPainter.
void PoseGraphNode::draw(QPainter* painter, const QPen& pen, const QBrush& brush, double radius) const
{
    drawUtil.drawNoseCircle(painter, pose, pen, brush, radius);
}

// Draws the id of the pose graph node as a label on QPainter.
void PoseGraphNode::drawLabel(QPainter *painter, const QPen &pen, double opacity) const
{
    painter->save();
    QFont font;
    font.setFamily("Arial");
    font.setPointSize(1);
    painter->setFont(font);
    painter->setPen(pen);
    painter->setOpacity(opacity);
    Vec2 c = pose.pos();
    painter->translate(c.x + 0.03, c.y + 0.04);
    painter->scale(0.1, -0.1);
    painter->drawText(QPointF(), QString::number(id));
    painter->restore();
}

// Draws the pose graph node as a nose circle in OpenGL.
void PoseGraphNode::draw(QColor color, double radius) const
{
    GLlib::drawNoseCircle(pose, color, radius);
}

// Draws the seen line connections of this node on QPainter.
void PoseGraphNode::drawSeenLineConnections(QPainter* painter, const QPen& pen) const
{
    ListIterator<TrackedLine*> it = seenMapLines.begin();
    while (it.hasNext())
        Line(pose.pos(), it.next()->center()).draw(painter, pen);
}

// Draws the seen line connections of this node in OpenGL.
void PoseGraphNode::drawSeenLineConnections() const
{
    ListIterator<TrackedLine*> it = seenMapLines.begin();
    while (it.hasNext())
        Line(pose.pos(), it.next()->center()).draw();
}

// Draws the neighborhood of this node on QPainter. The neighborhood are the
// nodes that are reachable from this node through the graph up to a distance of "depth".
void PoseGraphNode::drawNeighborhood(QPainter* painter, uint depth, const QPen& pen, const QBrush& brush, double radius) const
{
    if (depth > 0)
    {
        ListIterator<PoseGraphNode*> it = neighbours.begin();
        while (it.hasNext())
        {
            Line(pose.pos(), it.next()->pose.pos()).draw();
            it.cur()->drawNeighborhood(painter, depth-1, pen, brush, radius);
        }
    }

    draw(painter, pen, brush, radius);
}

// Draws the neighborhood of this node in OpenGL. The neighborhood are the
// nodes that are reachable from this node through the graph up to a distance of "depth".
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

// Writes the LineMap into a data stream.
void PoseGraphNode::streamOut(QDataStream &out) const
{
    out << id;
    out << pose;
    out << lastPose;
    out << observedNeighborhood;

    // There are pointers and cannot be saved to file.
    // They have to be dealt with elsewhere.
    //out << seenMapLines;
    //out << neighbours;
}

// Reads the LineMap from a data stream.
void PoseGraphNode::streamIn(QDataStream &in)
{
    in >> id;
    in >> pose;
    in >> lastPose;
    in >> observedNeighborhood;

    //in >> seenMapLines;
    //in >> neighbours;
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

QDebug operator<<(QDebug dbg, const PoseGraphNode &n)
{
    dbg << "id:" << n.id << "pose:" << n.pose << "lines seen:" << n.seenMapLines.size() << "neighbors:" << n.neighbours.size();
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

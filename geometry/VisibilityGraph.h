#ifndef VISIBILITYGRAPH_H_
#define VISIBILITYGRAPH_H_
#include "util/Vec2.h"
#include "util/Vector.h"
#include "util/LinkedList.h"
#include "util/PriorityQueue.h"
#include "util/AdjacencyMatrix.h"
#include "geometry/Box.h"
#include "geometry/Polygon.h"
#include "agents/Obstacle.h"
#include <QPainter>
#include <QDebug>
class GeometricModel;

struct Node : public Vec2
{
    uint id;
    int obstId; // Which obstacle the node belongs to.
    int lineId1, lineId2; // Ids of the line segments joining at this vertex.
    Vec2 v1,v2; // The in and out edges of the polygon corner used for the tangetial test.

    // A* stuff.
    double h;
    double g;
    double f;
    bool closed;
    Node* parent; // The parent pointer is the previous Node in the path.
    Node* successor; // The successor is the next Node in the found path.

    Node() : Vec2()
    {
        id=0;
        pidx = 0;
        obstId=-1;
        lineId1=-1;
        lineId2=-1;
        h=0;
        g=0;
        f=0;
        parent=0;
        successor=0;
        closed=false;
    }
    Node(Vec2 wp) : Vec2(wp)
    {
        id=0;
        pidx = 0;
        obstId=-1;
        lineId1=-1;
        lineId2=-1;
        h=0;
        g=0;
        f=0;
        parent=0;
        successor=0;
        closed=false;
    }

    bool isClosed() const;

    // Determines if this Node lies on the same polygon edge together with q.
    bool isAPolygonEdge(const Node& q) const
    {
        return (lineId1 != -1 && q.lineId1 != -1 && (q.lineId1 == lineId2 || q.lineId2 == lineId1));
    }

    // Decides if this Node (p) can be tangentially connected with Node q.
    bool isTangentialTo(Node *q) const;


    // Decides if this Node (p) is a corner of an obstacle that lies on the right of the
    // line from Node q to p.
    bool isRightOf(const Node &q) const
    {
        if (obstId == -1)
            return false;

        // Remember that v1 is the vector from this vertex to the next vertex in the polygon,
        // v2 is the vector from this vertex to the previous vertex, and v3 is the vector
        // from q to this vertex.
        Vec2 v3 = *this-q;
        return v1.det(v3) >= -EPSILON && v2.det(v3) >= -EPSILON;
    }

    // Decides if this Node (p) is a corner of an obstacle that lies on the left of the
    // line from Node q to p.
    bool isLeftOf(const Node& q) const
    {
        if (obstId == -1)
            return false;

        // Remember that v1 is the vector from this vertex to the next vertex in the polygon,
        // v2 is the vector from this vertex to the previous vertex, and v3 is the vector
        // from q to this vertex.
        Vec2 v3 = *this-q;
        return v1.det(v3) <= EPSILON && v2.det(v3) <= EPSILON;
    }

    // Decides if this Node (p) is in line with node q. Node p is in line with node q if the
    // line from p to q is colinear with one of the polygon edges adjacent to p (v1 and v2).
    bool isInLineWith(const Node& q) const
    {
        if (obstId == -1)
            return false;

        // Remember that v1 is the vector from this vertex to the next vertex in the polygon,
        // v2 is the vector from this vertex to the previous vertex, and v3 is the vector
        // from q to this vertex.
        Vec2 v3 = *this-q;
        return fabs(v1.det(v3)) <= EPSILON || fabs(v2.det(v3)) <= EPSILON;
    }

    // Stuff needed for priority queueing.
    uint pidx;
    bool cmp(const Node* n2) const {return (f < n2->f);}
    uint getPidx() const {return pidx;}
    void setPidx(uint k) {pidx = k;}
};

class VisibilityGraph
{
public:

    int debug;
    const GeometricModel* gm;
    Vec2 start;
    Vec2 target; // The target where the taxi should go.
    LinkedList<Node> nodes; // These are the nodes of the visibility graph.
    AdjacencyMatrix adjacencyMatrix; // Adjacency matrix of the graph.
    Node* startNode;
    Node* targetNode;
    QHash<int, bool> polygonClosed; // A closed list for the polygons that have been added to the graph.
    Box boundingBox; // A bounding box that confines the search to remain within.
    
    PriorityQueue<Node*> q; // A priority q for A*.
    Vector<Node> path; // The result of a path search.

    uint lineIntersectionTests;
    uint searchIterations;
    uint tangentialTests;
    uint sameSideTests;

public:

    VisibilityGraph();
    ~VisibilityGraph(){}

    void reset();

    void setStart(const Vec2& p);
    void setTarget(const Vec2& p);
    void setDebug(int d);
    void setGeometricModel(const GeometricModel &gm);
    void setBounds(double t, double l, double b, double r);
    const Box& getBounds() const;

    bool minimalConstruct(const Vec2& start);
    const Vector<Node> &getPath() const;

    void draw(QPainter* painter) const;
    void drawPath(QPainter* painter) const;

private:
    void initMinimalConstruct();
    void connectPolygonFully(const Obstacle &obst);
    void findParent(Node* orphan);
};

QDebug operator<<(QDebug dbg, const VisibilityGraph &w);
QDebug operator<<(QDebug dbg, const Node &o);

#endif

#ifndef VISIBILITYGRAPH_H_
#define VISIBILITYGRAPH_H_
#include "lib/util/Vec2.h"
#include "lib/util/Vector.h"
#include "lib/util/LinkedList.h"
#include "lib/util/PriorityQueue.h"
#include "lib/util/AdjacencyMatrix.h"
#include "lib/geometry/Box.h"
#include "lib/geometry/Polygon.h"
#include "lib/geometry/Path.h"
class GeometricModel;
#include <QPainter>
#include <QDebug>

struct Node : public Vec2
{
    uint id;
    int polyId; // Which polygon the node (corner) belongs to.
    int edgeId1, edgeId2; // Ids of the line segments joining at this vertex.
    Vec2 v1,v2; // The in and out edges of the polygon corner used for the tangetial testing.

    // A* stuff.
    double h;
    double g;
    double f;
    bool closed;
    Node* parent; // The parent pointer is the previous Node in the path.
    Node* successor; // The successor is the next Node in the path.

    Node() : Vec2()
    {
        id=0;
        pidx = 0;
        polyId=-1;
        edgeId1=-1;
        edgeId2=-1;
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
        polyId=-1;
        edgeId1=-1;
        edgeId2=-1;
        h=0;
        g=0;
        f=0;
        parent=0;
        successor=0;
        closed=false;
    }

    // Determines if this Node lies on the same polygon edge together with q.
    bool isAPolygonEdge(const Node& q) const
    {
        return (edgeId1 != -1 && q.edgeId1 != -1 && (q.edgeId1 == edgeId2 || q.edgeId2 == edgeId1));
    }

    // Decides if this Node (p) can be tangentially connected with Node q.
    bool isTangentialTo(Node *q, bool debug=false) const
    {
        // Polygon edges are always considered to be tangential.
        // This check is the reason why the geometric model needs to be renumbered before use.
        if (edgeId1 == q->edgeId2 || edgeId2 == q->edgeId1)
        {
            if (debug)
                qDebug() << "       " << id << "and" << q->id << "are tangential due to edge id." << edgeId1 << q->edgeId2 << edgeId2 << q->edgeId1;
            return true;
        }

        // Otherwise the actual tangential test needs to be computed.
        // Remember, v1 is the vector from this vertex to the next vertex in the polygon,
        // v2 is the vector from this vertex to the previous vertex, and v3 is the vector
        // from this vertex to the point q. An edge from this to q is tangential when v1
        // and v3 lie on the same side of v3 on both ends.
        Vec2 v3 = *q-*this;
        double detv3pv1 = v3.x*v1.y-v3.y*v1.x;
        double detv3pv2 = v3.x*v2.y-v3.y*v2.x;
        double detv3qv1 = v3.x*q->v1.y-v3.y*q->v1.x;
        double detv3qv2 = v3.x*q->v2.y-v3.y*q->v2.x;
        if (debug)
            qDebug() << "      Tangential check" << id << q->id << "vdet:" << detv3pv1 << detv3pv2 << "qdet" << detv3qv1 << detv3qv2;
        bool res = ( (polyId == -1 || detv3pv1 > -EPSILON && detv3pv2 > -EPSILON || detv3pv1 < EPSILON && detv3pv2 < EPSILON)
                     &&
                     (q->polyId == -1 || detv3qv1 > -EPSILON && detv3qv2 > -EPSILON || detv3qv1 < EPSILON && detv3qv2 < EPSILON)
                   );
        return res;
    }

    // Decides if this Node (q) is osculating a polygon together with it's predecessor p and successor r.
    bool isOsculating(Node *p, Node *r)
    {
        Vec2 pq = *this-*p;
        Vec2 qr = *r-*this;
        double detpqv3 = pq.x*v1.y-pq.y*v1.x;
        double detpqqr = pq.x*qr.y-pq.y*qr.x;
        double detqrv3 = qr.x*v1.y-qr.y*v1.x;
        double detqrv4 = qr.x*v2.y-qr.y*v2.x;
        return (detpqqr*detpqv3 >= 0 && detqrv3*detpqv3 >= 0 && detqrv4*detpqv3 >= 0);
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

    const GeometricModel* gm;
    LinkedList<Node> nodes; // These are the nodes of the visibility graph.
    AdjacencyMatrix adjacencyMatrix; // Adjacency matrix of the graph.
    Node* startNode;
    Node* targetNode;
    QHash<int, bool> polygonClosed; // A closed list for the polygons that have been added to the graph.
    Box boundingBox; // A bounding box that confines the search to remain within.

    PriorityQueue<Node*> q; // A priority q for A*.
    Path path; // The result of a path search.

    uint lineIntersectionTests;
    uint tangentialTests;

public:

    VisibilityGraph();
    ~VisibilityGraph(){}

    VisibilityGraph(const VisibilityGraph& o);
    VisibilityGraph& operator=(const VisibilityGraph& v);

    void reset();

    void setGeometricModel(const GeometricModel *gm);
    void setBounds(double t, double l, double b, double r);
    void setBounds(const Box& bb);
    const Box& getBounds() const;

    bool computePath(const Vec2 &from, const Vec2 &to, int debug=0);
    const Path &getPath() const;

    void draw(QPainter* painter) const;
    void draw() const;
    void drawPath(QPainter* painter) const;
    void drawPath() const;

private:
    bool minimalConstruct(const Vec2& start, const Vec2& target, int debug=0);
    void startNodeChanged(int debug=0);
    void targetNodeChanged();
    void connectPolygonFully(const Polygon &pol, int debug=0);
    void findParent(Node* orphan, int debug=0);

};

QDebug operator<<(QDebug dbg, const VisibilityGraph &w);
QDebug operator<<(QDebug dbg, const Node &o);

#endif

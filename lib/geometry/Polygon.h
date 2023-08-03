#ifndef POLYGON_H_
#define POLYGON_H_
#include <QPainter>
#include "Line.h"
#include "lib/util/Vec2.h"
#include "lib/util/Vec3.h"
#include "lib/util/Pose2D.h"
#include "lib/util/LinkedList.h"
#include "lib/util/Vector.h"
#include "lib/geometry/Box.h"
#include "lib/pml/hpm2D.h"
#include "lib/pml/unicycle.h"

struct IntersectionPoint
{
    uint id = 0;
    uint size = 0;
    ListIterator<Line> edgeIterator; // Edge iterator of the intersected edge.
    Line edge; // The intersected edge.
    Vec2 ip; // The actual intersection point.
    bool tangential = false;

    bool isPrevOf(const IntersectionPoint& ap)
    {
//        qDebug() << "isprevof" << edge.getId() << ap.edge.getId() << size
//                 << (edge.getId() < ap.edge.getId() && ap.edge.getId() - edge.getId() <= size/2)
//                 << (edge.getId() > ap.edge.getId() && edge.getId() - ap.edge.getId() > size/2);

       return ((edge.getId() < ap.edge.getId() && ap.edge.getId() - edge.getId() <= size/2)
               ||
              (edge.getId() > ap.edge.getId() && edge.getId() - ap.edge.getId() > size/2));
    }

    operator Vec2() const {return ip;}
};

class Polygon
{
public:

    // These variables contain the transformation of the polygon,
    // which is first a translation by (x,y) and then a rotation by theta.
    double x,y,theta;

    // Public access is only needed for state binding.
    // Move back to private when that's no longer needed.

private:
    int id; // A unique id used for various purposes.
    static int idCounter;
    mutable int convexityFlag; // -1: nonconvex, 0: unknown, 1: convex
    mutable int windingFlag; // -1: cw, 0: unknown, 1: ccw
    QColor color;

    LinkedList<Line> edges; // The edges of the polygon and with that also the vertices in p1.

protected:
    mutable Box aabb; // Precomputed bounding box.
    mutable bool boundingBoxValid;

public:

    Polygon();
    Polygon(double x, double y, double w, double h);
    Polygon(const Vec2& v0, const Vec2& v1, const Vec2& v2);
    Polygon(const Vec2& v0, const Vec2& v1, const Vec2& v2, const Vec2& v3);
    ~Polygon(){}

    int getId() const;
    void setId(int id);
    static void resetId();
    void rewriteEdgeIds();
    void setColor(const QColor &col);

    void clear();

    uint size() const;
    bool isEmpty() const;

    Pose2D pose() const;
    void setPose(const Pose2D& pose);
    void setPose(double x, double y, double theta);
    void setPose(const Vec2& p, double theta);
    Vec2 pos() const;
    void setPos(const Vec2& p);
    void setPos(double x, double y);
    double orientation() const;
    void setOrientation(double a);
    void translate(double dx, double dy);
    void translate(const Vec2& d);
    void rotate(double a);
    void turn(double a);
    bool isTransformed() const;
    void transform();
    void untransform();

    void reverseOrder();
    void ensureCCW();
    void ensureCW();
    bool isCCW() const;
    bool isCW() const;
    void setCCW();
    void setCW();

    bool isConvex() const;
    void setConvex();

    bool isSelfIntersecting() const;
    bool repairSelfIntersections(int range=5);

    void setUnitTriangle();
    void setUnitSquare();
    void setUnitHexagon();
    void setUnitOctogon();
    void setUnitNogon(uint n);

    ListIterator<Line> edgeIterator() const;
    const LinkedList<Line>& getEdges() const;
    LinkedList<Vec2> getVertices() const;
    LinkedList<Vec2> getTransformedVertices() const;
    LinkedList<Line> getTransformedEdges() const;

    void setVertices(const LinkedList<Vec2>&v);
    void appendVertex(const Vec2 &p, int type = Line::BlockingLine);
    void appendVertex(double x, double y, int type = Line::BlockingLine);
    void removeVertex(const Vec2& p);
    void setEdges(const LinkedList<Line>&v);
    void appendEdge(const Line &l);
    Polygon& operator<<(const Vec2 &p);
    Polygon& operator<<(const LinkedList<Vec2> &lp);
    Polygon& operator<<(const Line &l);
    Polygon& operator<<(const LinkedList<Line> &lp);
    bool operator==(const Polygon& other) const;

    const Box& boundingBox() const;
    Vec2 centroid() const;
    double diameter() const;
    double area() const;
    double distance(const Vec2& p) const;
    IntersectionPoint closestPoint(const Vec2& p) const;
    IntersectionPoint localClosestPoint(const Vec2& p, ListIterator<Line> inIt) const;
    Line closestEdge(const Vec2 &p) const;
    Vec2 closestNormal(const Vec2& p) const;
    void closestPointNormal(const Vec2& p, Vec2& point, Vec2& normal) const;
    Polygon convexHull() const;
    Vector<Polygon> triangulation(bool debug=false) const;

    void removeVertex(ListIterator<Line> &it);
    void removeEdge(ListIterator<Line> &it);
    void insertVertex(ListIterator<Line> &it, const Vec2& v);

    void grow(double delta);
    void scale(double sx, double sy);
    void scale(double s);
    void simplify(double epsilon=0.05);
    void prune();

    Vector<Line> clipLine(const Line& inputLine) const;
    void clip(const Polygon& clp);
    void clipConvex(const Polygon& clipPolygon, bool debug=false);
    void clipBox(const Box& box, bool debug=false);
    const Vector<Polygon>& clipped(const Polygon& clp) const;
    const Vector<Polygon>& united(const Polygon& clp) const;
    const Vector<Polygon>& united(const Vector<Polygon>& clp) const;
    const Vector<Polygon>& offseted(double delta, double eps=0) const;
    static const LinkedList<Polygon>& unify(const Vector<Polygon> &polygons);
    static const LinkedList<Polygon>& unify(const LinkedList<Polygon> &polygons);
    static const LinkedList<Polygon>& offset(const LinkedList<Polygon> &polygons, double delta, double eps=0);

    double intersects(const Hpm2D &inputKf) const;
    double intersects(const Unicycle &inputUni, bool debug=false) const;
    bool intersects(const Polygon &pol) const;
    bool intersects(const Line& l, bool intersectSightLines=false, bool debug=false) const;
    bool intersects(const Vec2& v, bool boundaryIntersect=true, bool debug=false) const;
    bool intersects(const Vec2& v, double radius) const;
    IntersectionPoint intersection(const Line &inputLine) const;
    IntersectionPoint rayIntersection(const Line& ray) const;
    IntersectionPoint pathIntersection(const Vector<Vec2>& path) const;

    void draw(QPainter* painter, const QPen &pen, const QBrush &brush, double opacity=1.0) const;
    void drawLabel(QPainter* painter) const;
    void draw(const QPen &pen, const QBrush &color, double alpha=1.0) const;

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream& in);

private:
    void douglasPeuckerSub(double epsilon, ListIterator<Line> fromIt, ListIterator<Line> toIt, LinkedList<Line> &result) const;
};

extern Polygon operator+(const Polygon& l, const Pose2D& p);
extern Polygon operator-(const Polygon& l, const Pose2D& p);
extern void operator+=(Polygon& l, const Pose2D& p);
extern void operator-=(Polygon& l, const Pose2D& p);
extern Vector<Polygon> operator+(const Vector<Polygon>& v, const Pose2D& p);
extern Vector<Polygon> operator-(const Vector<Polygon>& v, const Pose2D& p);
extern void operator+=(Vector<Polygon>& v, const Pose2D& p);
extern void operator-=(Vector<Polygon>& v, const Pose2D& p);

QDebug operator<<(QDebug dbg, const Polygon &o);
QDebug operator<<(QDebug dbg, const Polygon* o);
QDataStream& operator<<(QDataStream& out, const Polygon &o);
QDataStream& operator>>(QDataStream& in, Polygon &o);


#endif // POLYGON_H_

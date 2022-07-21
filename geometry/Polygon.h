#ifndef POLYGON_H_
#define POLYGON_H_
#include <QPainter>
#include "Line.h"
#include "util/Vec2.h"
#include "util/Vec3.h"
#include "util/Pose2D.h"
#include "util/LinkedList.h"
#include "util/Vector.h"
#include "geometry/Box.h"
#include "pml/hpm2D.h"
#include "pml/unicycle.h"

class Polygon
{
public:

    // These variables contain the transformation of the polygon,
    // which is first a translation by (x,y) and then a rotation by theta.
    // Whenever the transformation is changed, transform() will have to be
    // called to upate the vertices. However, you can combine several
    // transformations before calling transform() and this way the vertices
    // need to be updated only once.
    double x,y,theta;

protected:

    // The corners of the polygon.
    LinkedList<Vec2> vertices;

    int id; // A unique id used for various purposes.
    static int idCounter;

    mutable char convexityFlag;
    mutable bool boundingBoxValid;
    mutable Box aabb;
    mutable bool edgesAreComputed;
    mutable LinkedList<Line> edges;

    QColor color;

public:

    Polygon();
    Polygon(double x, double y, double w, double h);
    Polygon(const Vec2& v0, const Vec2& v1, const Vec2& v2);
    ~Polygon(){}

    void setColor(const QColor &col);

    int getId() const;
    void setId(int id);
    static void resetId();

    void set(double w, double h);

    void clear();
    int size() const;
    QPolygonF polygon() const;

    Pose2D pose() const;
    void setPose(const Pose2D& pose);
    void setPose(double x, double y, double theta);
    Vec2 pos() const;
    void setPos(const Vec2& p);
    void setPos(double x, double y);
    double rotation() const;
    void setRotation(double a);
    void translate(double dx, double dy);
    void translate(const Vec2& d);
    void rotate(double a);

    bool isTransformed() const;
    void transform();
    void untransform();

    void operator-=(const Pose2D &o);
    void operator+=(const Pose2D &o);

    void grow(double delta);
    void scale(double sx, double sy);

    Vector<Polygon> clipIntersect(const Polygon &cp);

    void reverseOrder();

    Vec2 centroid() const;
    const LinkedList<Line>& getEdges() const;
    const LinkedList<Vec2>& getVertices() const;
    LinkedList<Vec2> getTransformedVertices() const;
    void setVertices(const LinkedList<Vec2>&v);
    ListIterator<Vec2> vertexIterator() const;
    ListIterator<Line> edgeIterator() const;
    const Box& boundingBox() const;

    Vector<Polygon> triangulate() const;
    double diameter() const;
    double area() const;
    double distance(const Vec2& p) const;
    Vec2 closestPoint(const Vec2& p) const;
    Line closestEdge(const Vec2 &p) const;
    Vec2 closestNormal(const Vec2& p) const;
    Polygon convexHull() const;

    bool isConvex() const;
    void setConvex();

    QRectF boundingRect() const;
    QPainterPath shape() const;

    void addVertex(const Vec2 &p);
    void addVertex(double x, double y);
    Polygon& operator<<(const Vec2 &p);
    Polygon& operator<<(const Vector<Vec2> &vp);
    void removeVertex(const Vec2& p);

    virtual double intersects(const Hpm2D &kf) const;
    virtual double intersects(const Unicycle &u) const;
    virtual bool intersects(const Polygon &p) const;
    virtual bool intersects(const Line& l) const;
    virtual bool intersects(const Vec2& v) const;
    virtual bool intersects(const Vec2& v, double radius) const;
    Vec2 intersection(const Line &l) const;
    Vec2 rayIntersection(const Vec2& from, const Vec2& to) const;


    void setUnitTriangle();
    void setUnitSquare();
    void setUnitOctogon();
    void setUnitHexagon();

    void draw(QPainter* painter) const;
    void draw(const QColor &color) const;

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream& in);

};

QDebug operator<<(QDebug dbg, const Polygon &o);
QDataStream& operator<<(QDataStream& out, const Polygon &o);
QDataStream& operator>>(QDataStream& in, Polygon &o);


#endif // POLYGON_H_

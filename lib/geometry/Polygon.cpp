#include "Polygon.h"
#include "Box.h"
#include "board/State.h"
#include "board/Config.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "clipper2/clipper.h"
using namespace Clipper2Lib;

// The Polygon class is a general purpose polygon that consists of a number
// of vertices and a 2D transform given by a translation (x,y) and an orientation
// theta. (x,y,theta) is referred to as the pose or the transformation of a polygon.
//
// Polygons must have at least 3 vertices that are stored as edges in a LinkedList.
// Polygons are essentially closed paths where the first and the last vertex of the
// path are the same. The LinkedList has certain advantages in terms of memory
// management in exchange for not having random access to the vertices. The edges
// can only be accessed in a sequence (looped over) using an iterator.
// Here is a simple example:
//
// Polygon p;
// p.setUnitHexagon();
// ListIterator<Line> it = p.edgeIterator();
// while (it.hasNext())
// {
//    Line& e = it.next(); // access to an edge
//    Vec2& vertex = e.p1(); // access to a single vertex
// }
//
// Here, we obtained a mutable reference to edge e that can be used to read and
// write the coordinates of this edge. You can also take a const reference if you
// are in a const setting (const Line& e = it.next()). You can also take a copy of
// the edge if you don't want to modify the polygon itself (Line e = it.next()).
// Furthermore, you can use the edge iterator (even after you stepped them with next())
// in order to insert(), replace(), or remove() vertices or edges of the polygon at
// the current position of the iterator. You can populate a Polygon by adding
// vertices or edges with the appendVertex(), appendEdge() functions and the <<
// operator. The edges of a polygon can be blocking lines or sight lines. You have
// control over the type of the edge when providing the edges through the setEdges()
// or the appendEdge() functions. Furthermore, there are convenient constructors and
// set*() functions (e.g. setUnitSquare()) for constructing polygons that can then
// be scale()-d, translate()-d, and rotate()-d. Polygons are memory preserving so
// they can be clear()-ed and reused.
//
// The winding of a polygon can be positively oriented (counter clockwise, CCW) or
// negatively oriented (clockwise, CW). This choice is up to you and the winding is
// essentially determined by the order you construct the a Polygon. Polygons do not
// care whether they are in CCW or in CW winding. All functions transparently
// support both types of winding. Perhaps you should just keep in mind that for CCW
// oriented polygons the inside of the polygon is on the left side of the edges and
// for CW oriented polygons on the right side.
//
// The Polygon class provides means to query its transformation (x,y,theta) relative
// to the world using pose(), pos(), and orientation(). You can also manipulate the
// pose using the setPose(), setPos(), setOrientation(), translate(), rotate(), and
// turn() functions. The Polygon follows Pose2D arithmetic and you can simply do this:
// Polygon transformedPolygon = polygon + polygon.pose();
// in order to transform the polygon from local coordinates to world coordiantes.
// There is a Pose2D class in the util package that describes the Pose2D arithmetic,
// which is basically the transformation between coordinate frames defined by (x,y,theta).
//
// We call a polygon that has a nonzero transformation "untransformed" in the
// sense that its vertices are given relative to the local frame that is specified by
// its pose. In this untransformed state, the intuitive translate() and rotate()
// operations can be chained at almost no cost as they only modify the transformation
// of the polygon, but not its vertices. When calling the transform() function, the
// transformation is consumed and the vertices of the polygon are converted to world
// coordinates. pos() and orientation() are set to zero. After calling transform(), the
// rotate() and turn() operations will not work as expected as the polygon is no longer
// rotated around its earlier reference point, but around the world origin. The scale()
// function might also yield unexpected (but correct) results. The functions of the
// Polygon are programmed in a way that they work no matter if the polygon is transformed
// or not. The main reason to transform() a polygon is to obtain its vertices in world
// coordinates, which you can also do directly with the getTransformedVertices() function.
// However, after calling transform(), the intersects() collision checking functions run
// faster as the transformation step can be skipped. A possible use case for calling
// transform() is when you are no longer modifying a polygon, but want to perform many
// collision checks with it.
//
// The boundingBox(), triangulation(), getVertices(), and getEdges() functions return
// their objects in the local coordinate frame of the polygon. This is equal to world
// cordinates if the polygon has been transform()-ed. The getTransformedVertices() and
// getTransformedEdges() functions provide access to the vertices and edges in world
// coordinates without modifying the polygon. For a triangulation to world coordinates,
// you can use the Pose2D arithmetic like so:
//
// Polygon p;
// p.setUnitHexagon();
// p.setPose(pose);
// Vector<Polygon> trigs = p.triangulation() + p.pose();
//
// Polygons carry a number of collision checking functions, such as the intersects(Vec2)
// function that decides whether a point is inside the polygon or not. Furthermore,
// Polygons can also be intersected with other polygons, Lines, Unicycle bangs, and
// holonomic Hpm2D bangs.
//
// Polygons can also be grow()-n and offset()-ed to be shrunk or enlarged. Polygons can
// be unite()-ed or clip()-ed with other Polygon objects. Finally, Polygons can be
// simplify()-ed with the Douglas Peucker algorithm.

int Polygon::idCounter = 0;

// Creates an empty polygon.
Polygon::Polygon()
{
    id = idCounter++;
    boundingBoxValid = false;
    convexityFlag = 0;
    windingFlag = 0;
    setPos(0, 0);
    setOrientation(0);
}

// Box constructor.
// y and y are the coordinates of a reference point in the center.
// w and h are the half width and height along the x and y axis, respectively.
// The created box is ccw and transformed with a pose of (0,0,0).
Polygon::Polygon(const Box& box)
{
    id = idCounter++;
    boundingBoxValid = false;
    convexityFlag = 1;
    windingFlag = 1;
    setPos(0, 0);
    setOrientation(0);

    edges.push_back(Line(Vec2(box.left(), box.top()), Vec2(box.left(), box.bottom())));
    edges.push_back(Line(Vec2(box.left(), box.bottom()), Vec2(box.right(), box.bottom())));
    edges.push_back(Line(Vec2(box.right(), box.bottom()), Vec2(box.right(), box.top())));
    edges.push_back(Line(Vec2(box.right(), box.top()), Vec2(box.left(), box.top())));
}

// Box constructor.
// y and y are the coordinates of a reference point in the center.
// w and h are the half width and height along the x and y axis, respectively.
// The created box is ccw and untransformed with a pose of (x,y,0).
Polygon::Polygon(double x, double y, double w, double h)
{
    id = idCounter++;
    boundingBoxValid = false;
    convexityFlag = 1;
    windingFlag = 1;
    setPos(x, y);
    setOrientation(0);

    edges.push_back(Line(Vec2(-w, h), Vec2(-w, -h)));
    edges.push_back(Line(Vec2(-w, -h), Vec2(w, -h)));
    edges.push_back(Line(Vec2(w, -h), Vec2(w, h)));
    edges.push_back(Line(Vec2(w, h), Vec2(-w, h)));
}

// Triangle constructor. The vertices are given in world coordinates.
Polygon::Polygon(const Vec2 &v0, const Vec2 &v1, const Vec2 &v2)
{
    id = idCounter++;
    boundingBoxValid = false;
    convexityFlag = 1;
    windingFlag = 0;
    setPos(0, 0);
    setOrientation(0);

    edges.push_back(Line(v0, v1));
    edges.push_back(Line(v1, v2));
    edges.push_back(Line(v2, v0));
}

// Quadrangle constructor. The vertices are given in world coordinates.
Polygon::Polygon(const Vec2 &v0, const Vec2 &v1, const Vec2 &v2, const Vec2 &v3)
{
    id = idCounter++;
    boundingBoxValid = false;
    convexityFlag = 0;
    windingFlag = 0;
    setPos(0, 0);
    setOrientation(0);

    edges.push_back(Line(v0, v1));
    edges.push_back(Line(v1, v2));
    edges.push_back(Line(v2, v3));
    edges.push_back(Line(v3, v0));
}

// Returns the id of the polygon.
int Polygon::getId() const
{
    return id;
}

// Sets the polygon id.
void Polygon::setId(int id)
{
    this->id = id;
}

// Resets the id counter to 0.
void Polygon::resetId()
{
    idCounter = 0;
}

// Renumbers the edge ids from zero up.
void Polygon::rewriteEdgeIds()
{
    int counter = 0;
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
        it.next().setId(counter++);
}

// Discards all edges and resets the pose to 0.
// This is useful for the recycling of Polygon objects.
void Polygon::clear()
{
    x = 0;
    y = 0;
    theta = 0;
    boundingBoxValid = false;
    convexityFlag = 0;
    windingFlag = 0;
    edges.clear();
}

// Returns the number of edges of this polygon.
uint Polygon::size() const
{
    return edges.size();
}

// Returns true if the polygon has no edges assigned.
bool Polygon::isEmpty() const
{
    return edges.isEmpty();
}

// Returns the pose (x,y,theta) of the Polygon.
Pose2D Polygon::pose() const
{
    return Pose2D(x,y,theta);
}

// Sets the pose (x,y,theta) of the polygon.
// This results in an untransformed polygon whose vertices are expressed in local coordinates
// relative to the given pose.
void Polygon::setPose(const Pose2D &pose)
{
    this->x = pose.x;
    this->y = pose.y;
    this->theta = pose.heading();
}

// Sets the pose (x,y,theta) of the polygon.
// This results in an untransformed polygon whose vertices are expressed in local coordinates
// relative to the given pose.
void Polygon::setPose(double x, double y, double theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

// Sets the pose (p.x, p.y, theta) of the polygon.
// This results in an untransformed polygon whose vertices are expressed in local coordinates
// relative to the given pose.
void Polygon::setPose(const Vec2& p, double theta)
{
    this->x = p.x;
    this->y = p.y;
    this->theta = theta;
}

// Sets the (x,y) position of the polygon.
// This results in an untransformed polygon whose vertices are expressed in local coordinates
// relative to the given position. The orientation of the polygon remains unchanged.
void Polygon::setPos(double x, double y)
{
    this->x = x;
    this->y = y;
}

// Sets the (x,y) position of the polygon.
// This results in an untransformed polygon whose vertices are expressed in local coordinates
// relative to the given position. The orientation of the polygon remains unchanged.
void Polygon::setPos(const Vec2& p)
{
    this->x = p.x;
    this->y = p.y;
}

// Returns the (x,y) position of the polygon.
Vec2 Polygon::pos() const
{
    return Vec2(x, y);
}

// Translates the polygon by (dx,dy).
// This is a transform accumulating function. Only the position of the polygon is changed,
// but not the coordinates of the vertices. You need to call transform() in order to convert
// to world coordinates.
void Polygon::translate(double dx, double dy)
{
    setPos(this->x+dx, this->y+dy);
}

// Translates the polygon by d.
// This is a transform accumulating function. Only the position of the polygon is changed,
// but not the coordinates of the vertices. You need to call transform() in order to convert
// to world coordinates.
void Polygon::translate(const Vec2 &d)
{
    setPos(this->x+d.x, this->y+d.y);
}

// Rotates the polygon counter clockwise by the angle "a" given in radians
// around the world origin. If the polygon is untransformed, the position of the polygon is
// also affected! Only the pose of the polygon is changed, but not the vertex coordinates.
// See turn() for a rotation of the polygon around its local origin.
void Polygon::rotate(double a)
{
    setPos(pos().rotated(a));
    turn(a);
}

// Returns the orientation theta of the polygon.
double Polygon::orientation() const
{
    return theta;
}

// Sets the orientation theta of the polygon.
// Only the pose of the polygon is changed, but not the vertex coordinates.
void Polygon::setOrientation(double a)
{
    theta = a;
}

// Turns the polygon "in place" around it's local origin. The function only adds a to its orientation.
// The position of the polygon is not affected. This is in contrast to the rotate() method that rotates
// the entire polygon around the world origin (which also implies a turn()). Only the pose of the polygon
// is changed, but not the vertex coordinates.
void Polygon::turn(double a)
{
    setOrientation(theta+a);
}

// Grows (or shrinks) the polygon by delta. Delta is the distance by how many meters the
// vertices are pushed outwards along the half angle between the two neighbouring edges.
// If delta is negative, the vertices are pulled invards and the polygon is shrunk.
// This is a fast, but not well defined procedure that works well on convex and star-shaped
// polygons, but may create self-intersections and other unexpected results on non convex
// polygons. For a slower, but well defined kind of growing, try the offset() function.
void Polygon::grow(double delta)
{
    boundingBoxValid = false;

    isCCW();

    // Compute an offset to each vertex first.
    Vec2 offset[size()];
    int i = 0;
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        Vec2 v1 = it.peekPrev().lineVector();
        Vec2 v2 = it.peekCur().lineVector();
        it.next();

        v1.normalize();
        v2.normalize();
        Vec2 v3 = (v1-v2)/2;
        v3.normalize();

        offset[i] = windingFlag*sgn(v1.x*v2.y-v1.y*v2.x)*delta*v3;
        i++;
    }

    // Apply the offsets to the vertices.
    it = edgeIterator();
    i = 0;
    while (it.hasNext())
    {
        it.peekPrev().setP2(it.peekPrev().p2()+offset[i]);
        it.cur().setP1(it.cur().p1()+offset[i]);
        i++;
        it.next();
    }
}

// Scales (multiplies) the polygon vertices by the factors sx and sy.
// When the polygon is in an untransformed state and the center is
// the centroid of the polygon, this scaling has a growing effect.
// If the polygon is transformed, the result might not be what you expect.
void Polygon::scale(double sx, double sy)
{
    boundingBoxValid = false;

    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        Vec2 v = it.cur().p1();
        v.x *= sx;
        v.y *= sy;

        it.peekPrev().setP2(v);
        it.cur().setP1(v);
        it.next();
    }
}

// Scales (multiplies) the polygon vertices by the factor s.
// When the polygon is in an untransformed state and the center is
// the centroid of the polygon, this scaling has a growing effect.
// If the polygon is transformed, the result might not be what you expect.
void Polygon::scale(double s)
{
    scale(s, s);
}

// Reverses the order of the vertices from CW to CCW or vice versa.
// This is an O(N) operation.
void Polygon::reverseOrder()
{
    boundingBoxValid = false;
    edges.reverse();
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        Vec2 temp = it.cur().p1();
        it.cur().setP1(it.cur().p2());
        it.cur().setP2(temp);
        it.next();
    }

    windingFlag = -windingFlag;
}

// Ensures that the vertices of the polygon are given in
// counter clockwise (CCW) order. It can only detect and
// reverse CW order. It cannot fix an arbitrary order.
// This is an O(N) operation, even if the polygon is already
// in CCW order.
void Polygon::ensureCCW()
{
    if (!isCCW())
        reverseOrder();
}

// Ensures that the vertices of the polygon are given in
// clockwise (CW) order. It can only detect and reverse CCW order.
// It cannot fix an arbitrary order. This is an O(N) operation,
// even if the polygon is already in CW order.
void Polygon::ensureCW()
{
    if (isCCW())
        reverseOrder();
}

// Returns true if the winding of the polygon is counter clockwise (CCW).
// This is an O(N) operation, even if the polygon is already in CCW order.
// However, the result is cached so subsequent calls to this function are cheap.
bool Polygon::isCCW() const
{
    //qDebug() << "Polygon::isCCW() id:" << getId() << "flag:" << windingFlag;
    if (windingFlag != 0)
        return (windingFlag > 0);

    // The winding of the polygon is determined according to:
    // https://www.element84.com/blog/determining-the-winding-of-a-polygon-given-as-a-set-of-ordered-points
    // and https://stackoverflow.com/a/1165943/1906572
    ListIterator<Line> it = edgeIterator();
    double signedArea = 0;
    do
    {
        const Vec2& v1 = it.cur().p1();
        const Vec2& v2 = it.cur().p2();
        signedArea += v1.x*v2.y-v2.x*v1.y;
        it.next();
    } while (it.hasNext());

    windingFlag = sgn(signedArea);
    //qDebug() << "  winding flag determined to:" << windingFlag << "from" << signedArea;
    return (windingFlag > 0);
}

// Returns true if the winding of the polygon is clockwise (CW).
// This is an O(N) operation, even if the polygon is already in CW order.
// However, the result is cached so subsequent calls to this function are cheap.
bool Polygon::isCW() const
{
    return !isCCW();
}

// This function sets the CCW winding indicator. No check is performed!
// You can do this if you know what you are doing and want to save time
// on a CCW check, but if you mess this up, you only have yourself to blame.
void Polygon::setCCW()
{
    windingFlag = 1;
}

// This function sets the CW winding indicator. No check is performed!
// You can do this if you know what you are doing and want to save time
// on a CCW check, but if you mess this up, you only have yourself to blame.
void Polygon::setCW()
{
    windingFlag = -1;
}

// Computes the centroid of the polygon.
// The centroid is given in world coordinates (transformed).
Vec2 Polygon::centroid() const
{
    Vec2 c;
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
        c += it.next().p1();
    c /= size();
    c += pos();
    return c;
}

// Returns the edges of the polygon as a LinkedList of lines.
// The edges are given in a way that edge[i].p2() = edge[i+1].p1().
// This is also true for the last edge where edge[last].p2() = edge[0].p1().
// The edges are given in local coordinates (untransformed).
// If you want the edges in world coordinates, transform() the polygon first, or
// use getTransformedEdges(), or do getEdges() + pose().
const LinkedList<Line>& Polygon::getEdges() const
{
    return edges;
}

// Returns the edges of the polygon in world coordinates.
// The edges are given in a way that edge[i].p2() = edge[i+1].p1().
// This is also true for the last edge where edge[last].p2() = edge[0].p1().
// If the polygon is already transformed, the returned edges
// are the same as returned by getEdges().
LinkedList<Line> Polygon::getTransformedEdges() const
{
    return getEdges() + pose();
}

// Returns a LinkedList of the vertices (corners) of the polygon.
// The vertices are given in local coordinates (untransformed).
// If you want the vertices in world coordinates, use getTransformedVertices()
// or do getVertices() + pose().
LinkedList<Vec2> Polygon::getVertices() const
{
    LinkedList<Vec2> verts;
    ListIterator<Line> edgeIt = edgeIterator();
    while (edgeIt.hasNext())
        verts << edgeIt.next().p1();
    return verts;
}

// Returns the vertices in world coordinates.
// If the polygon is already transformed, the returned vertices
// are the same as returned by getVertices().
LinkedList<Vec2> Polygon::getTransformedVertices() const
{
    return getVertices() + pose();
}

// Sets (overwrites) the vertices of the polygon. Since Polygons are actually
// kept as a list of edges, new edges have to be created that the vertices v
// as their end points. All edge types are set to blocking.
void Polygon::setVertices(const LinkedList<Vec2> &v)
{
    boundingBoxValid = false;
    convexityFlag = 0;
    windingFlag = 0;
    edges.clear();
    ListIterator<Vec2> it = v.begin();
    while (it.hasNext())
    {
        edges << Line(it.cur(), it.peekNext());
        it.next();
    }
}

// Sets (overwrites) the edges of the polygon with the given Lines.
// It is assumed that e[i].p2() = e[i+1].p1() and you have to make sure of
// this yourself, otherwise you get a messed up polygon. The type of the edges
// (sight line or blocking line) is preserved as given by e.
void Polygon::setEdges(const LinkedList<Line> &e)
{
    boundingBoxValid = false;
    convexityFlag = 0;
    windingFlag = 0;

    // Set the edges to the given ones.
    edges = e;

    // Close the loop if needed.
    if (edges.last().p2() != edges.first().p1())
        edges << Line(edges.last().p2(), edges.first().p1());
}

// Appends an edge to the polygon at the tail. If l.p1() is not equal to the
// last vertex of the polygon, the line is ignored and no edge is added.
void Polygon::appendEdge(const Line& l)
{
    if  (!edges.isEmpty() && !edges.begin().atEnd() && (l.p1() != edges.last().p1() || l.p2() == edges.last().p2()))
    {
        qDebug() << state.frameId << "Polygon::appendEdge(const Line l)" << l << "ungood edge ignored.";
        qDebug() << "last edge:" << edges.last();
        return;
    }

    boundingBoxValid = false;
    convexityFlag = 0;

    // Remove the last edge connecting the last vertex with the first.
    edges.pop();

    // Append the new edge.
    edges << l;

    // Close the loop from the last to the first vertex.
    edges << Line(edges.last().p2(), edges.first().p1());
}

// Returns the axis aligned bounding box of the polygon in local
// coordinates (untransformed). If you are using the aabb for your
// own purposes and want to have it in world coordinates, transform()
// the polygon first or add the pose of the polygon to the box:
// Box aabb = boundingBox() + pose(). No matter if the Polygon is
// CCW or CW, the bounding box is always CCW.
const Box &Polygon::boundingBox() const
{
    if (boundingBoxValid)
        return aabb;
    boundingBoxValid = true;

    ListIterator<Line> it = edgeIterator();
    Vec2 v = it.next().p1();
    double left = v.x;
    double right = v.x;
    double top = v.y;
    double bottom = v.y;
    while (it.hasNext())
    {
        Vec2 v = it.next().p1();

        left = min(v.x, left);
        right = max(v.x, right);
        top = max(v.y, top);
        bottom = min(v.y, bottom);
    }

    aabb.set(top, left, bottom, right);
    return aabb;
}

// Returns the diameter of the polygon.
// The diameter of a polygon is the largest distance between any pairs of vertices.
// This is an O(N) operation.
double Polygon::diameter() const
{
    double d = 0;
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        Vec2 v1 = it.next().p1();
        ListIterator<Line> it2 = it;
        while (it2.hasNext())
        {
            Vec2 v2 = it2.next().p1();
            d = qMax(d, (v1-v2).norm());
        }
    }
    return d;
}

// Computes the area of the polygon. Computing the area also yields the winding
// of the polygon as a side effect. This is an O(N) operation.
double Polygon::area() const
{
    double a = 0;
    ListIterator<Line> it = edgeIterator();
    do
    {
        const Vec2& v1 = it.cur().p1();
        const Vec2& v2 = it.cur().p2();
        a += v1.x*v2.y-v2.x*v1.y;
        it.next();
    } while (it.hasNext());
    windingFlag = sgn0(a);
    return fabs(0.5*a);
}

// Computes the shortest distance between this polygon and the point p.
// p is expected to be given in world coordinates. The distance is computed
// between p and the point on the boundary of the polygon that is closest to p.
// If p is inside the polygon, the distance is still positive. This is an O(N) operation.
double Polygon::distance(const Vec2 &p) const
{
    return (p-closestPoint(p)).norm();
}

// Returns the closest point on the polygon boundary to the given point p.
// p is given in world coordinates and the returned closest point is also given
// in world coordinates. This is an O(N) operation.
IntersectionPoint Polygon::closestPoint(const Vec2 &p) const
{
    //qDebug() << "Polygon::closestPoint(const Vec2 &p):" << p << "polId:" << getId();

    // Transform the point to local coordinates.
    Vec2 point = p - pose();

    IntersectionPoint ip;

    // Determine the closest point.
    double minDist = std::numeric_limits<double>::max();
    Vec2 pp;
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        const Line& edge = it.cur();
        bool tangential;
        pp = edge.closestPoint(point, &tangential);
        //qDebug() << "   edge" << edge << "cp" << pp;
        double d = (pp-point).norm2();
        if (d <= minDist) // <= is used because sometimes the closest point is p2 of an edge which is the same as p1 of the next edge and p1 is better
        {
            minDist = d;
            ip.ip = pp;
            ip.edge = edge;
            ip.edgeIterator = it;
            ip.tangential = tangential;
            ip.size = edges.last().getId()+1;
        }
        it.next();
    }

    // Transform the closest point back to world coordinates.
    ip.ip += pose();

    return ip;
}

// Returns the locally closest point on the polygon boundary to the given point p.
// Starting at the edge indicated by the iterator inIt, a monotonically decreasing
// sequence of closest points is followed in both directions, and the closest of
// all found closest points is returned. This is much faster than a full blown
// closest point search on the entire polygon, but may not always find the globally
// closest point. p is given in world coordinates and the returned closest point is
// also given in world coordinates.
IntersectionPoint Polygon::localClosestPoint(const Vec2 &p, ListIterator<Line> inIt) const
{
    // Transform the point to local coordinates.
    Vec2 point = p - pose();

    IntersectionPoint ipLeft;
    IntersectionPoint ipRight;
    bool tangential;

    // Determine the closest point on the originating edge.
    Vec2 origCp = inIt.cur().closestPoint(point, &tangential);
    double minDist = (origCp-point).norm2();
    Vec2 localCp = origCp;

    ipRight.ip = localCp;
    ipRight.edge = inIt.cur();
    ipRight.edgeIterator = inIt;
    ipRight.tangential = tangential;
    ipLeft = ipRight;

    // Look into the "next" direction.
    ListIterator<Line> it = inIt;
    it.next();
    bool done = false;
    while (it != inIt && !done)
    {
        const Line& edge = it.cur();

        localCp = edge.closestPoint(point, &tangential);
        double d = (localCp-point).norm2();
        if (d < minDist)
        {
            minDist = d;
            ipRight.ip = localCp;
            ipRight.edge = edge;
            ipRight.edgeIterator = it;
            ipRight.tangential = tangential;
        }
        else
        {
            done = true;
        }
        it.next();
    }

    // Look into the "prev" direction.
    it = inIt;
    it.prev();
    done = false;
    minDist = (origCp-point).norm2();
    while (it != inIt && !done)
    {
        const Line& edge = it.cur();

        localCp = edge.closestPoint(point, &tangential);
        double d = (localCp-point).norm2();
        if (d < minDist)
        {
            minDist = d;
            ipLeft.ip = localCp;
            ipLeft.edge = edge;
            ipLeft.edgeIterator = it;
            ipLeft.tangential = tangential;
        }
        else
        {
            done = true;
        }
        it.prev();
    }

    if ((ipLeft.ip-point).norm2() < (ipRight.ip-point).norm2())
    {
        // Transform the closest point back to world.
        ipLeft.ip += pose();
        return ipLeft;
    }
    else
    {
        // Transform the closest point back to world.
        ipRight.ip += pose();
        return ipRight;
    }

    return ipLeft;
}

// Returns the closest edge of the polygon to the given point p. p is given in world
// coordinates and the returned edge is given as a Line also in world coordinates.
// More than one edge can have the same distance to a point. In such case, it's a
// bit of luck which edge is returned.
Line Polygon::closestEdge(const Vec2 &p) const
{
    // Transform the point to local coordinates.
    Vec2 point = p - pose();

    // Determine the closest edge.
    double minDist = std::numeric_limits<double>::max();
    Vec2 cp;
    Line cl;
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        const Line& edge = it.next();
        cp = edge.closestPoint(point);
        double n = (cp-point).norm2();
        if (n < minDist)
        {
            minDist = n;
            cl = edge;
        }
    }

    // Transform the closest edge back to world.
    cl += pose();

    return cl;
}

// Returns the normal of the closest edge or vertex of the polygon to the
// given point p. p is given in world coordinates and the returned normal
// vector is also given in world coordinates. The normal always points to
// the left of the polygon boundary. The winding of the polygon determines
// where the left is the outside or the inside. Edge normals are perpendicular
// to their edge and vertex normals point along the half angle between the
// adjacent edges.
Vec2 Polygon::closestNormal(const Vec2 &p) const
{
    // Transform the point to local coordinates.
    Vec2 point = p - pose();

    // Determine the closest point.
    double minDist = std::numeric_limits<double>::max();
    Vec2 closestPoint;
    Vec2 closestNormal;
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        const Vec2& v1 = it.peekPrev().p1();
        const Vec2& v2 = it.peekCur().p1();
        const Vec2& v3 = it.peekNext().p1();
        Vec2 dv1 = v2-v1;
        Vec2 dv2 = v3-v2;
        it.next();

        // Compute the closest point.
        double alpha = max(0.0, min(1.0, ((point-v2)*dv2)/(dv2*dv2)));

        //qDebug() << "   Checking" << v2 << v3 << alpha;

        if (alpha >= 1.0 - EPSILON)
        {
            //qDebug() << "      beyond far end";
            continue;
        }

        if (alpha < EPSILON) // The closest point is a vertex.
        {
            closestPoint = v2;
            double n = (closestPoint-point).norm();
            if (n < minDist)
            {
                minDist = n;

                // Compute the vertex normal.
                dv1.normalize();
                dv2.normalize();
                Vec2 vv = 0.5*(dv1-dv2);
                vv.normalize();
                closestNormal = sgn(dv1.x*dv2.y-dv1.y*dv2.x)*vv;
                //qDebug() << "      vertex" << v2 << "is closest." << n << "normal:" << closestNormal;
            }
        }
        else // The closest point is on the edge.
        {
            closestPoint = v2 + alpha*dv2;
            double n = (closestPoint-point).norm();
            if (n < minDist)
            {
                minDist = n;
                closestNormal = dv2.normalized();
                closestNormal.flip(-1);
                //qDebug() << "   edge" << v2 << v3 << "is closest" << n << "in point:" << closestPoint << "normal:" << closestNormal;
            }
        }
    }

    // Transform to world.
    closestNormal.frotate(theta);

    return closestNormal;
}

// Returns the closest point and the normal of the closest edge or vertex
// of the polygon to the given point p. p is given in world coordinates and
// the returned vectors are also given in world coordinates. The normal always points to
// the left of the polygon boundary. The winding of the polygon determines
// where the left is the outside or the inside. Edge normals are perpendicular
// to their edge and vertex normals point along the half angle between the
// adjacent edges.
void Polygon::closestPointNormal(const Vec2 &p, Vec2 &point, Vec2 &normal) const
{
    // Transform the point to local coordinates.
    Vec2 pp = p - pose();

    // Determine the closest point.
    double minDist = std::numeric_limits<double>::max();
    Vec2 closestPoint;
    Vec2 closestNormal;
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        const Vec2& v1 = it.peekPrev().p1();
        const Vec2& v2 = it.peekCur().p1();
        const Vec2& v3 = it.peekNext().p1();
        Vec2 dv1 = v2-v1;
        Vec2 dv2 = v3-v2;
        it.next();

        // Compute the closest point.
        double alpha = max(0.0, min(1.0, ((pp-v2)*dv2)/(dv2*dv2)));

        //qDebug() << "   Checking" << v2 << v3 << alpha;

        if (alpha >= 1.0 - EPSILON)
        {
            // The closest point is p2.
            //qDebug() << "      beyond far end";
            continue; // next edge
        }

        if (alpha < EPSILON)
        {
            // The closest point is p1.
            double n = (v2-pp).norm();
            if (n < minDist)
            {
                minDist = n;

                closestPoint = v2;

                // Compute the vertex normal.
                dv1.normalize();
                dv2.normalize();
                Vec2 vv = 0.5*(dv1+dv2);
                vv.flip();
                closestNormal = vv;
                //qDebug() << "   vertex" << v2 << "is closest." << n << "normal:" << closestNormal;
            }
        }
        else // The closest point is on the edge.
        {
            double n = ((v2 + alpha*dv2)-pp).norm();
            if (n < minDist)
            {
                minDist = n;

                closestPoint = v2 + alpha*dv2;
                closestNormal = dv2.normalized();
                closestNormal.flip();
                //qDebug() << "   edge" << v2 << v3 << "is closest" << n << "in point:" << closestPoint << "normal:" << closestNormal << dv2 << dv2.normal();
            }
        }
    }

    // Transform to world.
    closestPoint += pose();
    closestNormal.frotate(theta);

    point = closestPoint;
    normal = closestNormal;

    return;
}

// Returns the convex hull of the polygon as another Polygon.
// The returned polygon has the same pose as this one and it's
// in an untransformed state. Its winding is ccw.
Polygon Polygon::convexHull() const
{
    if (edges.isEmpty())
        return Polygon();

    // We are using OpenCV's convexHull() function here.
    // To my knowledge, cv::convexHull() implements Slanky's 1982 algorithm, which is incorrect.
    // It is desirable to reimplement Melkman 1987 and thereby also avoid the opencv dependancy.
    // http://cgm.cs.mcgill.ca/~athens/cs601/Melkman.html

    std::vector<cv::Point2f> pol;
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        Vec2 v = it.next().p1();
        pol.push_back(cv::Point2f(v.x, v.y));
    }

    std::vector<cv::Point2f> chPoints;
    cv::convexHull(pol, chPoints); // opencv function

    Polygon ch;
    for (uint i = 0; i < chPoints.size(); i++)
        ch << Vec2(chPoints[i].x, chPoints[i].y);
    ch.setPose(pose());

    return ch;
}

// Triangulates the Polygon into a set of triangles using the Ear Clipping method.
// The triangles are returned in local coordinates (untransformed). The triangulation fails
// if the polygon has self intersections. If the triangulation fails, the function returns a
// less than complete set of triangles.
Vector<Polygon> Polygon::triangulation(bool debug) const
{
    // The computations are carried out according to:
    // Joseph O'Rourke, Computational Geometry in C, Second Edition, Chapter 1.6.

    //qDebug() << "Triangulating" << id << size() << getVertices().first() << getVertices().last();
    thread_local Vector<Polygon> trigs; // This will be the final result.
    trigs.clear();

    Polygon workingCopy = *this; // We need an editable copy.
    workingCopy.setPose(Pose2D());
    workingCopy.ensureCCW();

    while (workingCopy.size() > 3)
    {
        bool earDetected = false;
        ListIterator<Line> it = workingCopy.edgeIterator();
        uint counter = 0;
        while (it.hasNext())
        {
            const Vec2& v0 = it.prevIt().peekPrev().p1();
            const Vec2& v1 = it.peekPrev().p1();
            const Vec2& v2 = it.cur().p1();
            const Vec2& v3 = it.peekNext().p1();
            const Vec2& v4 = it.nextIt().peekNext().p1();

            counter++;

            // Detect ear. Every polygon has at least two ears.
            // It has to be a convex corner with a diagonal that does not intersect any other edges.
            // The diagonal also has to lie within the inside cones of the adjacent edges.
            // The tip of the ear is at v2. The diagonal is from v1 to v3. v0 and v4 are used for the cone test.
            Line diagonal(v1,v3);
            bool isInCone1 = Line(v0,v2).isRightOf(v1) ? (v0-v1).isLeftOf(v3-v1) && (v2-v1).isRightOf(v3-v1) : !((v0-v1).isRightOf(v3-v1) && (v2-v1).isLeftOf(v3-v1));
            bool isInCone2 = Line(v2,v4).isRightOf(v3) ? (v2-v3).isLeftOf(v1-v3) && (v4-v3).isRightOf(v1-v3) : !((v2-v3).isRightOf(v1-v3) && (v4-v3).isLeftOf(v1-v3));
            earDetected = diagonal.isRightOf(v2) && isInCone1 && isInCone2 && !workingCopy.intersects(diagonal, true);
            if (debug)
                qDebug() << counter << "eartip:" << v2 << "diag:" << diagonal << "isRightOf(v2):" << diagonal.isRightOf(v2)
                         <<  "isInCone:" << isInCone1 << isInCone2
                           << "intersects()" << workingCopy.intersects(diagonal, true)
                           << "earDetected:" << earDetected;
            if (earDetected) // Ear detected.
            {
                trigs << Polygon(v1, v2, v3);
                workingCopy.removeVertex(it);
                if (debug)
                    qDebug() << counter << "Ear clipped. Removed vertex:" << v2 << "diagonal:" << diagonal << "remaining size:" << workingCopy.size() << "ccw:" << workingCopy.isCCW();
                break;
            }
            else
            {
                it.next();
            }
        }

        if (!earDetected)
        {
            if (debug)
                qDebug() << state.frameId << "Polygon::triangulate(): Ear detection failed!" << workingCopy.size();
            //qDebug() << workingCopy;
            //trigs.clear();
            //trigs << workingCopy;
            return trigs;
        }
    }

    if (workingCopy.size() != 3)
    {
        if (debug)
            qDebug() << "Polygon::triangulate(): The exit size is not 3!";
        return trigs;
    }

    //qDebug() << "Triangulation finished." << *this;
    trigs << workingCopy;
    return trigs;
}

// Removes the vertex from the polygon that is p1 of the edge indicated by the iterator it.
// Two edges are replaced by one such that p2 of the previous edge is set to p2 of the edge
// indicated by the iterator and then the indicated edge is removed. The type of the new edge
// is set to be the same type as the previous edge. The iterator is forwarded to the next edge.
void Polygon::removeVertex(ListIterator<Line> &it)
{
    it.peekPrev().setP2(it.cur().p2());
    edges.remove(it);
}

// Removes the edge indicated by the iterator it from the polygon.
// Two edges are replaced by one such that p2 of the previous edge is set to p2 of the edge
// indicated by the iterator and then the indicated edge is removed. The type of the new edge
// is set to be the same type as the previous edge. The iterator is forwarded to the next edge.
void Polygon::removeEdge(ListIterator<Line> &it)
{
    it.peekPrev().setP2(it.cur().p2());
    edges.remove(it);
}

// Inserts a vertex into the polygon edge identified by the iterator it.
// The new vertex splits the indicated edge into two edges, both of which
// will have the same edge type as the edge had before. It silenty ignores
// cases where v = it.p1 or v = it.p2.
void Polygon::insertVertex(ListIterator<Line> &it, const Vec2 &v)
{
    if (v == it.cur().p1() || v == it.cur().p2())
        return;
    Line newEdge(v, it.cur().p2(), it.cur().getType());
    it.cur().setP2(v);
    edges.insert(it.nextIt(), newEdge);
}

// Sheds vertices that do not modify the area of the polygon by more than delta.
void Polygon::prune(double delta)
{
    //qDebug() << "Polygon::prune(double delta):" << delta << "polId:" << getId() << "size:" << size();
    ListIterator<Line> edgeIt = edgeIterator();
    double signedArea = 0;
    while (edgeIt.hasNext())
    {
        const Vec2& v0 = edgeIt.peekPrev().p1();
        const Vec2& v1 = edgeIt.cur().p1();
        const Vec2& v2 = edgeIt.cur().p2();
        //double ar = v0.x*(v1.y-v2.y) + v1.x*(v2.y-v0.y) + v2.x*(v0.y-v1.y);
        double ar1 = v1.x*v2.y-v2.x*v1.y;
        signedArea += ar1;
        double ar = v0.x*(v1.y-v2.y) + v0.y*(v2.x-v1.x) + ar1;
        //qDebug() << "  edge" << edgeIt.cur() << "ar:" << ar;
        if (fabs(0.5*ar) < delta)
        {
            removeEdge(edgeIt);
            //qDebug() << "edge removed." << edgeIt.hasNext();
        }
        else
        {
            edgeIt.next();
        }
    }

    //qDebug() << "Final size:" << size();

    windingFlag = sgn(signedArea);
    boundingBoxValid = false;
    convexityFlag = 0;
}

// Sheds concave vertices that do not modify the area of the polygon by more than delta.
// This way, the polygon only grows in the outward direction, but never inward.
void Polygon::pruneOut(double delta)
{
    //qDebug() << "Polygon::prune(double delta):" << delta << "polId:" << getId() << "size:" << size();
    ListIterator<Line> edgeIt = edgeIterator();
    double signedArea = 0;
    while (edgeIt.hasNext())
    {
        const Vec2& v0 = edgeIt.peekPrev().p1();
        const Vec2& v1 = edgeIt.cur().p1();
        const Vec2& v2 = edgeIt.cur().p2();
        //double ar = v0.x*(v1.y-v2.y) + v1.x*(v2.y-v0.y) + v2.x*(v0.y-v1.y);
        double ar1 = v1.x*v2.y-v2.x*v1.y;
        signedArea += ar1; // also computes the signed area and with that the winding flag
        double ar = v0.x*(v1.y-v2.y) + v0.y*(v2.x-v1.x) + ar1;
        //qDebug() << "  edge" << edgeIt.cur() << "ar:" << ar;
        if (0.5*ar >= 0 && 0.5*ar < delta)
        {
            removeEdge(edgeIt);
            //qDebug() << "edge removed." << edgeIt.hasNext();
        }
        else
        {
            edgeIt.next();
        }
    }

    //qDebug() << "Final size:" << size();

    windingFlag = sgn(signedArea);
    boundingBoxValid = false;
}

// Sheds convex vertices that do not modify the area of the polygon by more than delta
// This way, the polygon only shrinks in the inward direction, but never grows outward.
void Polygon::pruneIn(double delta)
{
    //qDebug() << "Polygon::prune(double delta):" << delta << "polId:" << getId() << "size:" << size();
    ListIterator<Line> edgeIt = edgeIterator();
    double signedArea = 0;
    while (edgeIt.hasNext())
    {
        const Vec2& v0 = edgeIt.peekPrev().p1();
        const Vec2& v1 = edgeIt.cur().p1();
        const Vec2& v2 = edgeIt.cur().p2();
        //double ar = v0.x*(v1.y-v2.y) + v1.x*(v2.y-v0.y) + v2.x*(v0.y-v1.y);
        double ar1 = v1.x*v2.y-v2.x*v1.y;
        signedArea += ar1; // also computes the signed area and with that the winding flag
        double ar = v0.x*(v1.y-v2.y) + v0.y*(v2.x-v1.x) + ar1;
        //qDebug() << "  edge" << edgeIt.cur() << "ar:" << ar;
        if (0.5*ar <= 0 && 0.5*ar > -delta)
        {
            removeEdge(edgeIt);
            //qDebug() << "edge removed." << edgeIt.hasNext();
        }
        else
        {
            edgeIt.next();
        }
    }

    //qDebug() << "Final size:" << size();

    windingFlag = sgn(signedArea);
    boundingBoxValid = false;
}

// Simplifies this polygon according to the Douglas Peucker (DP) algorithm.
// https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
// As the result of this function, the polygon sheds vertices that do not modify
// the polygon by much.
void Polygon::simplify(double epsilon)
{
    // When executing the DP algorithm on the polygon, we do not want to combine
    // sight lines and blocking lines to an edge. So the edges of the polygon are
    // first segmented into chains of lines of the same type and then the segments
    // are Douglas Peuckerized each on their own.

    boundingBoxValid = false;

    thread_local LinkedList<Line> dpResultBuffer; // A buffer for the Douglas Peucker smoothing.
    dpResultBuffer.clear();
    ListIterator<Line> edgeIt = edgeIterator();
    ListIterator<Line> fromIt = edgeIt;
    int type = edgeIt.peekCur().getType();
    while (edgeIt.hasNext())
    {
        //qDebug() << "checking" << edgeIt << edgeIt.cur().length();
        if (edgeIt.peekCur().getType() != type)
        {
            //qDebug() << "segment found from" << fromIt << "to" << edgeIt.prevIt();
            douglasPeuckerSub(epsilon, fromIt, edgeIt.prevIt(), dpResultBuffer);
            type = edgeIt.peekCur().getType();
            fromIt = edgeIt;
        }
        edgeIt.next();
    }
    douglasPeuckerSub(epsilon, fromIt, edgeIt.prevIt(), dpResultBuffer);

    // Treat the case where the last point can be discarded.
    Line line(dpResultBuffer.last().p1(), dpResultBuffer.first().p2());
    if (dpResultBuffer.first().getType() == dpResultBuffer.last().getType()
            && fabs(line.orthogonalDistance(dpResultBuffer.first().p1())) < epsilon)
    {
        dpResultBuffer.first().setP1(dpResultBuffer.last().p1());
        dpResultBuffer.pop();
    }

    edges = dpResultBuffer;

    return;
}

// Recursive subroutine that computes the Douglas Peucker algorithm
// on a set of edges accessible by the iterators fromIt and toIt.
void Polygon::douglasPeuckerSub(double epsilon, ListIterator<Line> fromIt, ListIterator<Line> toIt, LinkedList<Line> &result) const
{
    //qDebug() << " dp sub called from:" << fromIt << "to:" << toIt;

    double dmax = 0;
    ListIterator<Line> maxIt = fromIt;

    // Construct a line from the first point in the set to the last.
    Line line(fromIt.peekCur().p1(), toIt.peekCur().p2());

    // Find the point with the maximum distance to the line.
    ListIterator<Line> workIt = fromIt;
    while (workIt != toIt)
    {
        double d = fabs(line.orthogonalDistance(workIt.peekCur().p2()));
        if (d > dmax)
        {
            dmax = d;
            maxIt = workIt;
        }
        workIt.next();
    }

    // If max distance is greater than epsilon, split the sequence
    // at the maximum distance point and simplify each half recursively.
    if (dmax > epsilon)
    {
        douglasPeuckerSub(epsilon, fromIt, maxIt, result);
        douglasPeuckerSub(epsilon, maxIt.nextIt(), toIt, result);
    }

    // Otherwise replace all points with the first and the last and end the recursion.
    else
    {
        result << Line(fromIt.cur().id, fromIt.cur().p1(), toIt.cur().p2(), fromIt.cur().getType());
        //qDebug() << " line" << result.last() << "added to buffer type:" << fromIt.cur().getType();
        //qDebug() << " dmax:" << epsilon << dmax << maxIt.cur();
    }

    return;
}

// Declares the polygon to be convex.
// Nothing is checked so you better know what you are doing.
void Polygon::setConvex()
{
    convexityFlag = 1;
}

// Returns true if the polygon is self-intersecting.
// This is an expensive O(N²) operation.
bool Polygon::isSelfIntersecting() const
{
    ListIterator<Line> itOuter = edgeIterator();
    ListIterator<Line> itInner = edgeIterator();
    itInner.next();
    while (!itOuter.atEnd())
    {
        while (itInner.hasNext())
        {
            if (itOuter.cur().intersects(itInner.cur()))
            {
                //qDebug() << "Polygon edge" << itOuter << "intersects with" << itInner;
                return true;
            }
            itInner.next();
        }
        itOuter.next();
        itInner = itOuter;
        itInner.next();
    }

    return false;
}

// Repairs small self intersection loops of up to "range" edges.
// Because of the bounded range, this algorithm is in O(N) instead of O(N^2),
// but it misses loops that are larger than the range.
bool Polygon::repairSelfIntersections(int range)
{
    ListIterator<Line> itOuter = edgeIterator();
    while (itOuter.hasNext())
    {
        ListIterator<Line> itInner = itOuter.prevIt();
        int counter = 0;
        while (counter < range)
        {
            counter++;
            itInner.prev();
            //qDebug() << "  " << counter << "testing" << itInner << "and" << itOuter;
            if (itInner.cur().intersects(itOuter.cur()))
            {
                //qDebug() << "   simple intersection detected between" << itOuter << "and" << itInner << state.frameId;

                // Fix up the polygon using the self intersection point as a vertex.
                Vec2 ip = itInner.cur().intersection(itOuter.cur());
                itInner.cur().setP2(ip);
                itOuter.cur().setP1(ip);

                // Remove the edges from the loop.
                ListIterator<Line> deleteIt = itInner;
                deleteIt.next();
                while (deleteIt != itOuter)
                {
                    //qDebug() << "removing edge" << deleteIt;
                    edges.remove(deleteIt);
                }
            }
        }
        itOuter.next();
    }

    return true;
}

// Returns true if the polygon is convex. The test works on CCW and CW wound polygons.
// This is an O(N) operation. The result is cached, however, so subsequent calls to this
// function cost next to nothing.
bool Polygon::isConvex() const
{
    // In order to make this test work for CCW and for CW windings, we simply test if
    // all corners of the polygon are the same type of turn (left or right).

    if (convexityFlag == 0) // 0 means unknown.
    {
        convexityFlag = 1; // Assume convexity.
        double lastTurnType = 0;

        ListIterator<Line> it = edgeIterator();
        while (it.hasNext())
        {
            const Vec2& v1 = it.peekPrev().p1();
            const Vec2& v2 = it.peekCur().p1();
            const Vec2& v3 = it.peekNext().p1();

            double turnType = (v2.x-v1.x)*(v3.y-v2.y)-(v2.y-v1.y)*(v3.x-v2.x);
            if (lastTurnType * turnType <= 0)
            {
                convexityFlag = -1; // Set to non-convexity.
                break;
            }
            lastTurnType = turnType;

            it.next();
        }
    }

    return (convexityFlag == 1);
}

// Returns true if the polygon is in a transformed state, i.e. its pose
// is zero and the vertices are in world coordinates.
bool Polygon::isTransformed() const
{
    return (x < EPSILON && x > -EPSILON && y < EPSILON && y > -EPSILON && theta < EPSILON && theta > -EPSILON);
}

// Performs a collision check with the edges of this polygon and the
// holonomic bang described by the kf. If no collision occurs, -1 is returned.
// Otherwise the relative time dt is returned to indicate the future time of
// a collision relative to the bang. If the collision would occur later than
// the dt in the keyframe, -1 is returned. Note that no containment check is
// performed. The entire bang could be inside the polygon and no collision is
// reported. Only intersections with the edges of the polygon are found.
// The input kf is given in world coordinates.
double Polygon::intersects(const Hpm2D &inputKf) const
{
    //qDebug() << "  Polygon::intersects(Hpm2D):" << kf;

    Hpm2D kf = inputKf - pose();

    // Bounding box check.
    boundingBox();
    if (!aabb.intersects(kf.boundingBox()))
        return -1;

    double ct = -1;

    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        const Line& edge = it.next();

        // Intersect the edge with the hpm2d bang.
        double cct = kf.intersects(edge);

        // Keep track of the smallest collision time so far.
        if (cct >= 0 && (ct < 0 || cct < ct))
            ct = cct;
    }

    return ct;
}

// Performs a collision check with the edges of this polygon and the
// unicycle bang described by u given in world coordinates. If no collision
// occurs, -1 is returned. Otherwise, the relative time dt is returned to
// indicate the future time of a collision relative to the bang. If the
// collision would occur later than the dt in u, -1 is returned. Note that
// no containment check is performed. The entire bang could be inside the
// polygon and no collision is reported. Only intersections with the edges
// of the polygon are found.
double Polygon::intersects(const Unicycle &inputUni, bool debug) const
{
    if (debug)
        qDebug() << "   Polygon::intersects(Unicycle):" << inputUni << "pol" << this;

    Unicycle u = inputUni - pose();

    // Bounding box check.
    boundingBox();
    if (!aabb.intersects(u.boundingBox()))
        return -1;

    double ct = -1;

    // Full edge check.
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        const Line& edge = it.next();
        if (debug)
            qDebug() << "   checking edge" << edge;

        // Intersect the edge with the unicycle.
        double cct = u.intersects(edge);

        // Keep track of the smallest collision time so far.
        if (cct >= 0 && (ct < 0 || cct < ct))
        {
            if (debug)
                qDebug() << "   intersects edge" << edge << "at" << cct;
            ct = cct;
        }
    }

    return ct;
}

// Returns true if the polygon pol intersects with this one. If both polygons are convex,
// the SAT algorithm is used to reliably detect all kinds of intersections including
// containment. If at least one of the polygons is non-convex, only a weaker corner
// containment check is used, which misses overlapping polygons where no vertex is
// contained by the other, for example when two squares perfectly overlap, but one
// square is rotated by 45 degrees. A more reliable algorithm would have to decompose
// the polygons into convex sections and check if any of them intersect. This approximation
// is much faster and reasonably reliable in realistic environments. This function requires
// the polygons to be in the same coordinate frame, i.e. to be transformed or to have the
// same pose. The intersection test works as you would expect regardless of the windings
// of the polygons.
bool Polygon::intersects(const Polygon &pol) const
{
    //qDebug() << "  Polygon::intersects(p):" << p;
    //qDebug() << "  this:" << *this;

    // Bounding box check. Reject cases whose bounding boxes don't overlap.
    boundingBox();
    if (!aabb.intersects(pol.boundingBox()))
        return false;

    if (isConvex() && pol.isConvex())
    {
        // This is an implementation of the SAT algorithm as described here:
        // http://www.dyn4j.org/2010/01/sat/
        // The concept is that if two convex polygons do not intersect, there
        // is a separating line between them, and one of the edges of either
        // polygon must be such a separating line. So the algorithm tests for
        // every edge if one of the polygons lies entirely on the one side and
        // the other polygon on the other. If such an edge is found, the
        // polygons do not intersect. If no edge can be found, the polygons
        // intersect. "Left of" and "right of" tests are performed using the
        // scalar product of the left normal of an edge and the vertex to test.
        // I deviate from the algorithm shown on the website in a way that
        // instead of projecting both polygons on every edge normal and trying
        // to find the separation, I test for every edge only the vertices of
        // the *other* polygon. The polygon the edge was taken from always lies
        // entirely on the positive side of the edge. If all vertices of the
        // other polygon lie on the negative side of the edge, we know there is
        // no intersection. This way, I save computation time by computing the
        // cross product only for the vertices of one polygon, and quickly
        // discarding an edge as soon as a cross product evaluates positive.

        isCCW(); // Make sure the winding flag is set.

        // Test the edges of the source against the points of the target.
        ListIterator<Line> sourceIterator = edgeIterator();
        ListIterator<Line> targetIterator = pol.edgeIterator();
        while (sourceIterator.hasNext())
        {
            const Vec2& v1 = sourceIterator.peekCur().p1();
            const Vec2& v2 = sourceIterator.peekNext().p1();
            sourceIterator.next();

            bool allPointsAreRightOf = true;
            targetIterator.reset();
            while (targetIterator.hasNext())
            {
                const Vec2& p = targetIterator.next().p1();
                double side = (v2.x-v1.x)*(p.y-v1.y)-(v2.y-v1.y)*(p.x-v1.x);
                if (side*windingFlag >= 0)
                {
                    allPointsAreRightOf = false;
                    break; // next source edge
                }
            }

            if (allPointsAreRightOf)
                return false;
        }

        pol.isCCW(); // Make sure the winding flag is set.

        // Test the edges of the target against the points of the source.
        targetIterator.reset();
        while (targetIterator.hasNext())
        {
            const Vec2& v1 = targetIterator.peekCur().p1();
            const Vec2& v2 = targetIterator.peekNext().p1();
            targetIterator.next();

            bool allPointsAreRightOf = true;
            sourceIterator.reset();
            while (sourceIterator.hasNext())
            {
                const Vec2& p = sourceIterator.next().p1();
                double side = (v2.x-v1.x)*(p.y-v1.y)-(v2.y-v1.y)*(p.x-v1.x);
                if (side*windingFlag >= 0)
                {
                    allPointsAreRightOf = false;
                    break; // next target edge
                }
            }

            if (allPointsAreRightOf)
                return false;
        }

        return true;
    }
    else
    {
        // For non-convex polygons, only a corner containment test is used.
        // All corners of one polygon are tested to lie in the other, and vice versa.
        // If at least one test is positive, the polygons intersect for sure and
        // the test can be aborted. This test misses cases where two polygons
        // overlap, but none of the corners are contained. For example, two
        // squares lying on top of each other but one is rotated by 45 degrees.

        ListIterator<Line> sourceIterator = edgeIterator();
        while (sourceIterator.hasNext())
        {
            const Vec2& v0 = sourceIterator.next().p1();
            if (pol.intersects(v0))
                return true;
        }

        ListIterator<Line> targetIterator = pol.edgeIterator();
        while (targetIterator.hasNext())
        {
            const Vec2& v0 = targetIterator.next().p1();
            if (intersects(v0))
                return true;
        }

        // Test the centroids, too, to catch a few of those "cross examples".
        if (pol.intersects(centroid()))
            return true;
        if (intersects(pol.centroid()))
            return true;

        return false;
    }

    return false;
}

// Returns true if one of the edges of pol intersects with one of the edges of this polygon.
// This function requires
// the polygons to be in the same coordinate frame, i.e. to be transformed or to have the
// same pose. The intersection test works as you would expect regardless of the windings
// of the polygons.
bool Polygon::intersectsEdges(const Polygon &pol) const
{
    //qDebug() << "  Polygon::intersects(p):" << p;
    //qDebug() << "  this:" << *this;

    // Bounding box check. Reject cases whose bounding boxes don't overlap.
    boundingBox();
    if (!aabb.intersects(pol.boundingBox()))
        return false;

    ListIterator<Line> sourceIterator = edgeIterator();
    while (sourceIterator.hasNext())
    {
        const Line& e0 = sourceIterator.next();
        if (pol.intersects(e0))
            return true;
    }

    return false;
}

// Returns true if the polygon intersects with the point p. This is true if p is in
// the interior area of the polygon. When boundaryIntersect is true (default), then
// p also intersects with the polygon when it's right on one of the edges (epsilon
// strict). For polygons with CCW winding, the interior area is to left of the edges,
// for polygons with CW winding the interior is to the right of the edges. This
// distinction is handeled automatically. The point p must be given in world
// coordinates. There is a speed up if the polygon is convex.
bool Polygon::intersects(const Vec2 &v, bool boundaryIntersect, bool debug) const
{
    // Transform the point to local coordinates.
    Vec2 p = v - pose();

    if (debug)
        qDebug() << "  Polygon::intersects(Vec2):" << v << p << "with polygon:" << this;

    // Bounding box check.
    boundingBox();
    if (!aabb.intersects(p))
        return false;

    isCCW(); // Make sure the winding flag is set.

    if (isConvex())
    {
        // Point intersection with a convex polygon is implemented using
        // a scalar product test with every edge. The point has to be to the left of
        // every edge in order to be contained by a convex polygon with CCW winding
        // or it has to be to the right of every edge when the winding is CW.
        // http://totologic.blogspot.de/2014/01/accurate-point-in-triangle-test.html
        // As soon as one edge is found the point lies on the wrong side of, the
        // algorithm can terminate and report that there is no collision.

        ListIterator<Line> it = edgeIterator();
        while (it.hasNext())
        {
            const Vec2& v1 = it.cur().p1();
            const Vec2& v2 = it.cur().p2();
            double side = (p.x-v1.x)*(v2.y-v1.y)-(p.y-v1.y)*(v2.x-v1.x);
            if (debug)
                qDebug() << "testing edge" << it.cur() << "p:" << p << "vs:" << v1 << v2 << "side:" << side;
            if (side*windingFlag >= EPSILON)
                return false;
            it.next();
        }

        return true;
    }
    else
    {
        // For the point intersection test with a non-convex polygon we are using the
        // winding number algorithm: https://en.wikipedia.org/wiki/Point_in_polygon
        // It is slower than the convex test because it cannot abort early.
        // On the upside, the winding number algorithm does not care whether the
        // polygon is CCW or CW, so it is perfect for our purposes.

        int wn = 0;

        // Loop through all edges of the polygon.
        ListIterator<Line> it = edgeIterator();
        while (it.hasNext())
        {
            const Vec2& v1 = it.peekCur().p1();
            const Vec2& v2 = it.peekNext().p1();
            it.next();

            if (v1.y <= p.y)
            {
                if (v2.y > p.y) // an upward crossing
                {
                    double side = (p.x-v1.x)*(v2.y-v1.y)-(p.y-v1.y)*(v2.x-v1.x);
                    if (debug)
                        qDebug() << "edge" << v1 << v2 << "upward crossing" << side;
                    if (fabs(side) <= EPSILON) // Special case right on the edge.
                        return boundaryIntersect;
                    if (side*windingFlag < 0)
                        wn++; // have a valid up intersect
                }
            }
            else
            {
                if (v2.y <= p.y) // a downward crossing
                {
                    double side = (p.x-v1.x)*(v2.y-v1.y)-(p.y-v1.y)*(v2.x-v1.x);
                    if (debug)
                        qDebug() << "edge" << v1 << v2 << "downward crossing" << side;
                    if (fabs(side) <= EPSILON) // Special case right on the edge.
                        return boundaryIntersect;
                    if (side*windingFlag > 0)
                        wn--; // have a valid down intersect
                }
            }
        }

        if (debug)
            qDebug() << "The winding number is:" << wn << "Zero means no intersection.";

        return (wn != 0);
    }

    return true;
}

// Returns true if the polygon intersects the circle specified by its center p
// and its radius. This is true if the point is in the interior area of the polygon
// irrespective of it being CCW or CW) or the circle touches the polygon.
// The center of the circle p is given in world coordinates.
bool Polygon::intersects(const Vec2 &p, double radius) const
{
    // Containment check.
    if (intersects(p))
        return true;

    // Distance check.
    return (distance(p) <= radius);
}

// Returns true if the Line l intersects any edge of this polygon.
// The parameter Line l must be given in world coordinates.
// This is an edge intersection test only. It does not detect containment.
// By default, SightLine edge types do not intersect the passed Line l,
// unless you set the intersectSightLines parameter to true.
bool Polygon::intersects(const Line& l, bool intersectSightLines, bool debug) const
{
//    if (debug)
//        qDebug() << "      Polygon::intersects(Line): pid:" << getId() << "l:" << l;

    // Transform the line into the reference frame of the polygon.
    Line line = l - pose();

    // Bounding box check.
    boundingBox();
    if (!aabb.intersects(line))
        return false;

    // Full edge check.
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        const Line& edge = it.next();
//        if (debug)
//            qDebug() << "         checking line" << line << "with edge" << edge;
        //if ((edge.isBlockingLine() || intersectSightLines) && edge.intersects(line, debug))
        if (edge.intersects(line, debug))
        {
//            if (debug)
//                qDebug() << "         edge" << edge << "intersects with line" << line;// << "in point" << edge.intersection(line);
            return true;
        }
    }

    return false;
}

// Computes an intersection point between the boundary of this polygon and the
// line l. If the line does not intersect the boundary, a zero Vec2() is returned.
// If the line intersects the boundary twice, the intersection point that happens
// to be found first is returned. If you want to have the intersection point
// guaranteed where the line hits the polygon first, use the rayIntersection()
// function. The input line is expected in world coordinates and the
// intersection point is also returned in world coordinates.
IntersectionPoint Polygon::intersection(const Line &inputLine) const
{
    IntersectionPoint ip;

    // Transform the line into the reference frame of the polygon.
    Line l = inputLine - pose();

    Vec2 ipp;
    ListIterator<Line> e = edgeIterator();
    while (e.hasNext())
    {
        const Line& edge = e.cur();
        ipp = edge.intersection(l);
        if (!ipp.isNull())
        {
            ip.ip = ipp + pose();
            ip.edge = edge;
            ip.edgeIterator = e;
            return ip;
        }
        e.next();
    }

    return ip;
}

// Returns the point at which a ray from p1 to p2 of the line first intersects the polygon.
// If the ray does not intersect, the intersection point equals p2.
IntersectionPoint Polygon::rayIntersection(const Line &ray) const
{
    Line lray = ray;
    lray -= pose();

    IntersectionPoint ip;
    ip.ip = lray.p2();
    double dMin = ray.length();
    int intersectionCounter = 0;

    ListIterator<Line> e = edgeIterator();
    while (e.hasNext())
    {
        const Line& edge = e.peekCur();
        Vec2 ipp = edge.intersection(lray);
        if (!ipp.isNull())
        {
            double dd = (ipp-lray.p1()).norm();
            //qDebug() << "   line" << ray.id << "with edge:" << edge.id << "ip:" << ip << "dd:" << dd << "dMin:" << dMin << fabs(dd-dMin);

            ip.tangential = false;
            if (intersectionCounter > 0 && fabs(dd-dMin) < 1.0E-4)
            {
                ip.tangential = true;
                //qDebug() << "   Double intersection at edge:" << edge.id << "ip:" << ipp << "dd:" << dd << "dMin:" << dMin;
            }

            if (dd > EPSILON && dd < dMin)
            {
                ip.ip = ipp;
                ip.edge = edge;
                ip.edgeIterator = e;
                dMin = dd;
                intersectionCounter++;
            }
        }
        e.next();
    }

    ip.ip += pose();
    return ip;
}

// Returns the point at which the path intersects the polygon for the first time.
// If the path does not intersect the boundary of the polygon, the last point of
// the path is returned. The path and the returned intersection point are in world
// coordinates.
IntersectionPoint Polygon::pathIntersection(const Vector<Vec2>& path) const
{
    //qDebug() << "Polygon::pathIntersection(path):" << path;

    for (uint i = 1; i < path.size(); i++)
    {
        IntersectionPoint ipp = rayIntersection(Line(path[i-1], path[i]));
        if (ipp.ip != path[i])
            return ipp;
    }

    IntersectionPoint ip;
    ip.ip = path.last();
    return ip;
}

// Returns true if this polygon entirely contains the other.
// This function requires the polygons to be in the same coordinate frame, i.e. to be transformed
// or to have the same pose. The intersection test works as you would expect regardless of the
// windings of the polygons.
bool Polygon::contains(const Polygon &pol) const
{
    ListIterator<Line> it = pol.edgeIterator();
    while (it.hasNext())
        if (!intersects(it.next().p1()))
            return false;
    return true;
}

// Clips the line with this polygon, i.e. removes the parts of the line that are
// covered by the polygon and returns the pieces of the line that are outside the
// polygon. This can result in any number of line pieces. The line is given in world
// coordinates. The returned lines are also in world coordinates.
Vector<Line> Polygon::clipLine(const Line &inputLine) const
{
    Line line = inputLine-pose();
    //qDebug() << "Polygon::clipLine(const Line &inputLine)" << line;

    // Collect a vector of intersection points between the line and the polygon.
    // The end points of the line are also added.
    Vector<Vec2> intersectionPoints;
    intersectionPoints << inputLine.leftVertex();
    Vec2 ip;
    ListIterator<Line> e = edgeIterator();
    while (e.hasNext())
    {
        const Line& edge = e.next();
        ip = edge.intersection(line); // local
        if (!ip.isNull())
            intersectionPoints << ip+pose(); // world
    }
    intersectionPoints << inputLine.rightVertex();
    intersectionPoints.sort(); // Sort by x coordinate.

    // Check if the first point is inside or outside the polygon.
    bool in = intersects(intersectionPoints.first()); // leftmost vertex

    // In an in and out alternating succession, collect the line pieces that are outside.
    Vector<Line> clippedLines;
    for (uint i = 0; i < intersectionPoints.size()-1; i++)
    {
        if (!in)
            clippedLines << Line(intersectionPoints[i], intersectionPoints[i+1]);
        in = !in;
    }

    return clippedLines;
}

// Clips this polygon with the cp polygon such that only the parts of this polygon
// remain that do not overlap the cp polygon. If this polygon is A and cp is B,
// then you get A-B. Both polygons need to be in the same frame of reference,
// i.e. both must have the same pose. All edge types are set to blocking.
const LinkedList<Polygon> &Polygon::clipped(const Polygon &cp) const
{
    PathD sub;
    ListIterator<Line> vit = edgeIterator();
    while (vit.hasNext())
    {
        Vec2 v = vit.next().p1();
        sub.push_back(PointD(v.x, v.y));
    }

    PathD clp;
    ListIterator<Line> cpit = cp.edgeIterator();
    while (cpit.hasNext())
    {
        Vec2 v = cpit.next().p1();
        clp.push_back(PointD(v.x, v.y));
    }

    PathsD pols, clips;
    pols.push_back(sub);
    clips.push_back(clp);

    thread_local LinkedList<Polygon> resultPolygons;
    resultPolygons.clear();

    // Execute the Clipper union operation.
    ClipperD clipper(4); // 4 digits precision
    clipper.AddSubject(pols);
    clipper.AddClip(clips);
    clipper.ReverseSolution = true;
    clipper.Execute(ClipType::Difference, FillRule::NonZero, resultPolygons);

    return resultPolygons;
}

// Clips this polygon with the cp polygon such that only the parts of this polygon
// remain that do not overlap the cp polygon. If this polygon is A and cp is B,
// then you get A-B. Both polygons need to be in the same frame of reference,
// i.e. both must have the same pose. All edge types are set to blocking.
const LinkedList<Polygon>& Polygon::clipped(const Vector<Polygon> &cp) const
{
    PathsD pols;
    PathD sub;
    ListIterator<Line> vit = edgeIterator();
    while (vit.hasNext())
    {
        Vec2 v = vit.next().p1();
        sub.push_back(PointD(v.x, v.y));
    }
    pols.push_back(sub);

    PathsD clips;
    for (uint i = 0; i < cp.size(); i++)
    {
        PathD clp;
        ListIterator<Line> it = cp[i].edgeIterator();
        while (it.hasNext())
        {
            const Vec2& v = it.next().p1();
            clp.push_back(PointD(v.x, v.y));
        }
        clips.push_back(clp);
    }

    thread_local LinkedList<Polygon> resultPolygons;
    resultPolygons.clear();

    // Execute the Clipper union operation.
    ClipperD clipper(4); // 4 digits precision
    clipper.AddSubject(pols);
    clipper.AddClip(clips);
    clipper.ReverseSolution = true;
    clipper.Execute(ClipType::Difference, FillRule::NonZero, resultPolygons);

    return resultPolygons;
}

// Clips this polygon with the convex cp polygon such that only the parts of this
// polygon remain that overlap the cp polygon. If this polygon is A and cp is B,
// then you get A^B. Both polygons need to be in the same frame of reference,
// i.e. both must have the same pose. All edge types are set to blocking.
void Polygon::clipConvex(const Polygon &clipPolygon, bool debug)
{
    // The computations are carried out according to the Sutherland Hodgman algorithm.
    // https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm

    if (debug)
        qDebug() << "Polygon::clipConvex(const Polygon &clipPolygon):" << this;

    if (clipPolygon.isEmpty())
    {
        clear();
        return;
    }

    Polygon outputPol = *this;
    Polygon inputPol;

    ListIterator<Line> clipEdgeIt = clipPolygon.edgeIterator();
    while (clipEdgeIt.hasNext())
    {
        const Line& clipEdge = clipEdgeIt.next();
        Vec2 lv = clipEdge.lineVector();
        if (debug)
            qDebug() << "Next clip edge:" << clipEdge;

        inputPol = outputPol;
        outputPol.clear();

        ListIterator<Line> inputEdgeIt = inputPol.edgeIterator();
        while (inputEdgeIt.hasNext())
        {
            const Line& inputEdge = inputEdgeIt.next();
            if (debug)
                qDebug() << "  Next input edge:" << inputEdge;

            const Vec2& cp = inputEdge.p2();  // current point
            const Vec2& pp = inputEdge.p1();  // prev point

            if (((cp-clipEdge.p1()).det(lv))*clipPolygon.windingFlag > 0) // p2 (cp) is inside
            {
                if (debug)
                    qDebug() << "   p2" << cp << "is inside.";
                if (((pp-clipEdge.p1()).det(lv))*clipPolygon.windingFlag <= 0 ) // p1 (pp) is not inside
                {
                    if (debug)
                        qDebug() << "   p1" << pp << "is not inside. intersection adding" << clipEdge.intersectionInf(inputEdge);
                    outputPol << clipEdge.intersectionInf(inputEdge); // intersection point
                }
                if (debug)
                    qDebug() << "   adding p2" << cp;
                outputPol << inputEdge.p2();
            }
            else if (((pp-clipEdge.p1()).det(lv))*clipPolygon.windingFlag > 0) // p1 is inside
            {
                if (debug)
                    qDebug() << "   p1" << pp << "is inside. intersection adding" << clipEdge.intersectionInf(inputEdge);
                outputPol << clipEdge.intersectionInf(inputEdge); // intersection point
            }
        }
    }

    *this = outputPol;

    return;
}

// Clips this polygon with the Box such that only the parts of this polygon
// remain that overlap the box polygon. If this polygon is A and box is B,
// then you get A^B. Both polygons need to be in the same frame of reference,
// i.e. both must have the same pose. All edge types are set to blocking.
// The winding of either polygon does not matter.
void Polygon::clipBox(const Box &box, bool debug)
{
    // Create a polygon
    Polygon boxPolygon;
    boxPolygon << box.topLeft() << box.bottomLeft() << box.bottomRight() << box.topRight();
    boxPolygon.setConvex();
    boxPolygon.setCCW();
    clipConvex(boxPolygon, debug);
    return;
}

// Grows (or shrinks) the polygon by delta. Delta is the distance by how many meters the
// polygon is grown or shrunk, if delta is negative. Offsetting is not the translation of
// a polygon, but a modification of its size. After the growing, the resulting polygon is
// simplified by removing meaningless vertices with the Douglas Peucker algorithm that
// deviate less than eps from the line through its neighbours. The offsetting of a polygon
// can result in multiple polygons. This function returns all of them. The returned polygons
// are in a transformed state.
const Vector<Polygon>& Polygon::offseted(double delta) const
{
    // The offsetting operation is performed using the Clipper2 library.
    // http://www.angusj.com/clipper2/Docs/Overview.htm
    // For some reason I don't understand, offsetting with Clipper works best on polygons
    // in local coordinates.

    PathsD subj;
    PathD path;
    ListIterator<Line> eit = edgeIterator();
    while (eit.hasNext())
    {
        const Vec2& v = eit.next().p1();
        path.push_back(PointD(v.x, v.y));
    }
    subj.push_back(path);

    PathsD solution = InflatePaths(subj, delta, JoinType::Round, EndType::Polygon, 4);

    thread_local Vector<Polygon> polygons;
    polygons.clear();
    for (uint i = 0; i < solution.size(); i++)
    {
        Polygon pol;
        //pol.setType(getType());
        // What about the winding?
        for (uint j = 0; j < solution[i].size(); j++)
            pol.appendVertex(solution[i][j].x, solution[i][j].y);
        polygons << pol;
    }
    return polygons;
}

// Computes the union of the polygons. All input polygons need to be in the
// same frame of reference, i.e., all must have the same pose. The order of
// the input polygons does not matter, but their winding does have an
// influence on the result. The union of a polygon containing another polygon
// with the same winding equals the enclosing polygon. However, if the enclosed
// polygon has the opposite winding, the result is the enclosing polygon with
// a hole in it. The order and the winding of the returned polygons encodes
// the hierarchy among the components. The first polygon in the vector is
// always positively oriented (ccw) and it is on the highest level of hierarchy,
// possibly enclosing oher polygons on deeper levels. The polygons are listed
// levelwise whereby the winding alternates from level to level. A change of
// winding indicates the next level of hierarchy. The edge types of the result
// polygons are set to blocking.
const LinkedList<Polygon> &Polygon::unify(const Vector<Polygon> &polygons, bool reverse)
{
    // This function uses the Clipper2 library.
    // http://www.angusj.com/clipper2/

    // It is still unclear whether it is better to use the PolyTree
    // structure to retrieve the polygon hierarchy, or if determining
    // the winding is sufficient.

    PathsD pols;
    for (uint i = 0; i < polygons.size(); i++)
    {
        PathD clp;
        ListIterator<Line> it = polygons[i].edgeIterator();
        while (it.hasNext())
        {
            const Vec2& v = it.next().p1();
            clp.push_back(PointD(v.x, v.y));
        }
        pols.push_back(clp);
    }

    thread_local LinkedList<Polygon> resultPolygons;

    // Execute the Clipper union operation.
    PolyTreeD polyTree;
    ClipperD clipper(4);
    clipper.AddSubject(pols);
    clipper.ReverseSolution = reverse;
    //clipper.Execute(ClipType::Union, FillRule::NonZero, resultPolygons);
    clipper.Execute(ClipType::Union, FillRule::NonZero, polyTree);
    PolyTreeToPolygonsD(polyTree, resultPolygons);
    ListIterator<Polygon> rit = resultPolygons.begin();
    while (rit.hasNext())
    {
        //qDebug() << "id:" << rit.cur().getId() << "size:" << rit.cur().size() << "area:" << rit.cur().area() << "ccw:" << rit.cur().isCCW();
        if (rit.cur().size() < 5 && rit.cur().area() < max(0.01, config.gmPolygonPruning))
        {
            resultPolygons.remove(rit);
        }
        else
        {
            rit.next();
        }
    }

    return resultPolygons;
}

// Computes the union of the polygons. All input polygons need to have the same pose.
// The order of the input polygons does not matter, but their winding does have an
// influence on the result. The union of a polygon containing another polygon
// with the same winding equals the enclosing polygon. However, if the enclosed
// polygon has the opposite winding, the result is the enclosing polygon with
// a hole in it. The order and the winding of the returned polygons encodes
// the hierarchy among the components. The first polygon in the vector is
// always positively oriented (ccw) and it is on the highest level of hierarchy,
// possibly enclosing oher polygons on deeper levels. The polygons are listed
// levelwise whereby the winding alternates from level to level. A change of
// winding indicates the next level of hierarchy. The edge types of the result
// polygons are set to blocking.
const LinkedList<Polygon> &Polygon::unify(const LinkedList<Polygon> &polygons, bool reverse)
{
    // This function uses the Clipper2 library.
    // http://www.angusj.com/clipper2/

    // Convert the polygons to PathsD as required by Clipper.
    PathsD pols;
    ListIterator<Polygon> it = polygons.begin();
    while (it.hasNext())
    {
        PathD clp;
        Polygon& pol = it.next();
        ListIterator<Line> it = pol.edgeIterator();
        while (it.hasNext())
        {
            const Vec2& v = it.next().p1();
            clp.push_back(PointD(v.x, v.y));
        }
        pols.push_back(clp);
    }

    thread_local LinkedList<Polygon> resultPolygons;

    // Execute the Clipper union operation.
    PolyTreeD polyTree;
    ClipperD clipper(4);
    clipper.AddSubject(pols);
    clipper.ReverseSolution = reverse;
    //clipper.Execute(ClipType::Union, FillRule::NonZero, resultPolygons);
    clipper.Execute(ClipType::Union, FillRule::NonZero, polyTree);
    PolyTreeToPolygonsD(polyTree, resultPolygons);
    ListIterator<Polygon> rit = resultPolygons.begin();
    while (rit.hasNext())
    {
        //qDebug() << "id:" << rit.cur().getId() << "size:" << rit.cur().size() << "area:" << rit.cur().area() << "ccw:" << rit.cur().isCCW();
        if (rit.cur().size() < 5 && rit.cur().area() < max(0.01, config.gmPolygonPruning))
        {
            resultPolygons.remove(rit);
        }
        else
        {
            rit.next();
        }
    }

    return resultPolygons;
}

// Grows (or shrinks) the polygons by delta. Delta is the distance by how many meters the
// polygons are grown (or shrunk if delta is negative). Offsetting is not the translation of
// a polygon, but a modification of its size. After the offsetting, the resulting polygons
// are simplified by removing meaningless vertices with the Douglas Peucker algorithm that
// deviate less than eps from the line through its neighbours. The offsetting operation can
// result in multiple polygons. Input polygons can merge or split. This function returns all
// of them. The returned polygons are in a transformed state. All input polygons need to be
// in the same frame of reference, i.e., all must have the same pose. The order of the input
// polygons does not matter, but their winding does have an influence on the result.
const LinkedList<Polygon> &Polygon::offset(const LinkedList<Polygon> &polygons, double delta)
{
    // The offsetting operation is performed using the Clipper2 library.
    // http://www.angusj.com/clipper2/Docs/Overview.htm
    // For some reason I don't understand, offsetting with Clipper works best on polygons
    // in local coordinates.

    PathsD subj;
    ListIterator<Polygon> it = polygons.begin();
    while (it.hasNext())
    {
        PathD path;
        ListIterator<Line> eit = it.next().edgeIterator();
        while (eit.hasNext())
        {
            const Vec2& v = eit.next().p1();
            path.push_back(PointD(v.x, v.y));
        }
        subj.push_back(path);
    }

    PathsD solution = InflatePaths(subj, delta, JoinType::Round, EndType::Polygon, 4);

    thread_local LinkedList<Polygon> outPolygons;
    outPolygons.clear();
    for (uint i = 0; i < solution.size(); i++)
    {
        Polygon pol;
        for (uint j = 0; j < solution[i].size(); j++)
            pol.appendVertex(solution[i][j].x, solution[i][j].y);
        outPolygons << pol;
    }

    return outPolygons;
}

const LinkedList<Polygon> &Polygon::clip(const LinkedList<Polygon> &subjects, const Vector<Polygon> &clippers, bool reverse)
{
    PathsD pols;
    ListIterator<Polygon> pit = subjects.begin();
    while (pit.hasNext())
    {
        PathD sub;
        ListIterator<Line> vit = pit.next().edgeIterator();
        while (vit.hasNext())
        {
            const Vec2& v = vit.next().p1();
            sub.push_back(PointD(v.x, v.y));
        }
        pols.push_back(sub);
    }

    PathsD clips;
    for (uint i = 0; i < clippers.size(); i++)
    {
        PathD clp;
        ListIterator<Line> it = clippers[i].edgeIterator();
        while (it.hasNext())
        {
            const Vec2& v = it.next().p1();
            clp.push_back(PointD(v.x, v.y));
        }
        clips.push_back(clp);
    }

    thread_local LinkedList<Polygon> resultPolygons;
    resultPolygons.clear();

    // Execute the Clipper union operation.
    ClipperD clipper(4); // 4 digits precision
    clipper.AddSubject(pols);
    clipper.AddClip(clips);
    clipper.ReverseSolution = true;
    clipper.Execute(ClipType::Difference, FillRule::NonZero, resultPolygons);

    return resultPolygons;
}

// Generates a CCW unit triangle.
void Polygon::setUnitTriangle()
{
    clear();
    appendVertex(-1.0, -1.0);
    appendVertex(1.0, -1.0);
    appendVertex(1.0, 1.0);
    convexityFlag = 1;
    windingFlag = 1;
}

// Generates a CCW unit square.
void Polygon::setUnitSquare()
{
    clear();
    appendVertex(-1.0, 1.0);
    appendVertex(-1.0, -1.0);
    appendVertex(1.0, -1.0);
    appendVertex(1.0, 1.0);
    convexityFlag = 1;
    windingFlag = 1;
}

// Generates a CCW unit hexagon.
void Polygon::setUnitHexagon()
{
    clear();
    double s = tan(30*DEG_TO_RAD);
    appendVertex(1.0, s);
    appendVertex(0.0, 2*s);
    appendVertex(-1.0, s);
    appendVertex(-1.0, -s);
    appendVertex(0.0, -2*s);
    appendVertex(1.0, -s);
    convexityFlag = 1;
    windingFlag = 1;
    return;
}

// Generates a CCW unit octogon.
void Polygon::setUnitOctogon()
{
    clear();
    appendVertex(-0.5, 1.0);
    appendVertex(-1.0, 0.5);
    appendVertex(-1.0, -0.5);
    appendVertex(-0.5, -1.0);
    appendVertex(0.5, -1.0);
    appendVertex(1.0, -0.5);
    appendVertex(1.0, 0.5);
    appendVertex(0.5, 1.0);
    convexityFlag = 1;
    windingFlag = 1;
}

// Generates a CCW unit N-ogon with n corners.
void Polygon::setUnitNogon(uint n)
{
    double theta = PII / double(n);
    double c = fcos(theta);
    double s = fsin(theta);
    double t;

    double x = 1.0; //we start at angle = 0
    double y = 0;
    clear();
    for(int ii = 0; ii < n; ii++)
    {
        appendVertex(x, y);
        t = x;
        x = c * x - s * y;
        y = s * t + c * y;
    }
}

// Draws the polygon on a QPainter.
// It does not matter whether the polygon is transformed or not.
void Polygon::draw(QPainter *painter, const QPen &pen, const QBrush &brush, double opacity) const
{
    painter->save();
    painter->translate(x, y);
    painter->rotate(theta*RAD_TO_DEG);

    painter->setPen(pen);
    painter->setBrush(brush);
    painter->setOpacity(opacity);

    // Draw the polygon area.
    painter->setPen(Qt::NoPen);
    QPainterPath pp;
    pp.moveTo(edges.first().p1());
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
        pp.lineTo(it.next().p1());
    pp.lineTo(edges.first().p1());
    painter->drawPath(pp);

    // Draw the outline.
    it = edgeIterator();
    while (it.hasNext())
    {
        QPen penny = pen;
        const Line& line = it.next();
        if (line.getType() == Line::SightLine)
        {
            penny.setStyle(Qt::DotLine);
            penny.setCosmetic(true);
            penny.setWidth(1);
        }
        painter->setPen(penny);
        painter->drawLine(line.p1(), line.p2());
    }

    painter->restore();
}

// Draws a label with the polygon id on a QPainter.
void Polygon::drawLabel(QPainter *painter) const
{
    painter->save();
    painter->translate(centroid());
    painter->scale(0.3, -0.3);
    painter->setOpacity(0.8);
    painter->drawText(QPointF(), QString::number(getId()));
    painter->restore();
}

// Draws a label with the polygon id on a QPainter.
void Polygon::drawEdgeLabels(QPainter *painter) const
{
    ListIterator<Line> lit = edgeIterator();
    while (lit.hasNext())
        lit.next().drawLabel(painter);
}

// Draws the polygon in an OpenGL context.
void Polygon::draw(const QPen& pen, const QBrush& brush, double opacity) const
{
    glPushMatrix();
    glMultMatrixd(pose().getMatrix());

    VecN<4> currentColor;
    glGetDoublev(GL_CURRENT_COLOR, currentColor.data()); // remember the color we had before drawing

    // Set the desired color.
    if (brush.color().isValid())
        glColor4f(brush.color().redF(), brush.color().greenF(), brush.color().blueF(), opacity);

    // Draw the surface.
    if (isConvex())
    {
        glBegin(GL_POLYGON);
        ListIterator<Line> it = edgeIterator();
        while (it.hasNext())
        {
            Vec2 v = it.next().p1();
            glVertex2f(v.x, v.y);
        }
        glEnd();
    }
    else
    {
        Vector<Polygon> triangles = triangulation();
        glBegin(GL_TRIANGLES);
        for (uint i = 0; i < triangles.size(); i++)
        {
            ListIterator<Line> it = triangles[i].edgeIterator();
            while (it.hasNext())
                glVertex2dv(it.next().p1());
        }
        glEnd();
    }

    // Draw the outline in black.
    // Sight lines are marked with thin stipple.
    glColor3f(0,0,0);
    ListIterator<Line> contour = edgeIterator();
    while (contour.hasNext())
    {
        const Line& line = contour.next();
        if (line.isBlockingLine())
        {
            glLineWidth(pen.width());
            glEnable(GL_LINE_SMOOTH);
            glDisable(GL_LINE_STIPPLE);
        }
        else
        {
            glLineWidth(1);
            glDisable(GL_LINE_SMOOTH);
            glEnable(GL_LINE_STIPPLE);
            glLineStipple(2, 0xAAAA);
        }
        line.draw();
    }

    glEnable(GL_LINE_SMOOTH);
    glDisable(GL_LINE_STIPPLE);

    // Mark the vertices.
    GLlib::setColor(drawUtil.brush.color());
    ListIterator<Line> it3 = edgeIterator();
    while (it3.hasNext())
    {
        Vec2 v = it3.next().p1();
        glPushMatrix();
        glTranslated(v.x, v.y, 0);
        GLlib::drawCircle(0.005,0.005);
        glPopMatrix();
    }

    glColor4dv(currentColor.data()); // restore the saved color
    glPopMatrix();
}

// Appends a vertex to the polygon.
void Polygon::appendVertex(double x, double y, int type)
{
    appendVertex(Vec2(x,y), type);
    return;
}

// Appends a vertex to the polygon. The resulting edge from the previous vertex
// of the polygon to the added vertex will be of type "type" (Line::Blocking by default).
// If the vertex p equals the last vertex of the polygon, it is silently ignored.
void Polygon::appendVertex(const Vec2 &p, int type)
{
    // Blank state case.
    if (edges.isEmpty())
    {
        edges << Line(p, p, type);
        return;
    }

    // If the vertex p equals the last vertex of the polygon, it is silently ignored.
    if (p == edges.last().p1())
        return;

    // If only one vertex has ever been appended.
    if (edges.first().p1() == edges.first().p2())
    {
        edges.first().setP2(p);
        edges.first().setType(type);
        return;
    }

    // If two vertices have been appended so far, we finally have a triangle.
    if (edges.begin().atEnd()) // size() == 1, but faster
    {
        edges << Line(edges.last().p2(), p, type);
        edges << Line(p, edges.first().p1(), type); // Connect the last vertex with the first. The type is unclear.
        return;
    }

    // Update the edges.
    edges.last().setP2(p);
    edges.last().setType(type);
    edges << Line(p, edges.first().p1(), type); // Connect the last vertex with the first. The type is unclear.

    convexityFlag = 0;
    boundingBoxValid = false;
}

// Appends a vertex to the polygon.
Polygon& Polygon::operator<<(const Vec2 &p)
{
    appendVertex(p);
    return *this;
}

// Appends an edge to the polygon.
Polygon& Polygon::operator<<(const Line &l)
{
    appendEdge(l);
    return *this;
}

// Appends a LinkedList of vertices to the polygon.
Polygon& Polygon::operator<<(const LinkedList<Vec2> &vp)
{
    ListIterator<Vec2> it = vp.begin();
    while (it.hasNext())
        appendVertex(it.next());
    return *this;
}

// Removes the given vertex from the polygon, if it exists.
void Polygon::removeVertex(const Vec2 &p)
{
    boundingBoxValid = false;
    convexityFlag = 0;
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        if (it.cur().p1() == p)
        {
            removeEdge(it);
            return;
        }
        it.next();
    }
}

// Returns an iterator that can be used to conveniently cycle
// through the edges of the polygon. Here is a simple example of
// iterating over the edges of a polygon:
//
// Polygon p;
// p.setUnitHexagon();
// ListIterator<Line> it = p.edgeIterator();
// while (it.hasNext())
//    Line& e = it.next();
//
// Here, we obtained a mutable reference to edge e that you can use to read and
// write the coordinates of the edge. Keep in mind that the edge1.p2 = edge2.p1
// constraint must be fullfilled if you want to modify the vertex coordinates.
// You can take a const reference if you are in a const setting (const Line& e = it.next()).
// You can also take a copy of the edge if you don't want to modify the polygon itself
// (Line e = it.next()). If you want to iterate over the vertices of the polygon, you
// can do that like so:
//
// Polygon p;
// p.setUnitHexagon();
// ListIterator<Line> it = p.edgeIterator();
// while (it.hasNext())
//    Vec2 v = it.next().p1();
//
ListIterator<Line> Polygon::edgeIterator() const
{
    return edges.begin();
}

// Consumes the current transformation in a way that it transforms all
// vertices to world coordinates and then resets the pose to zero.
// However, after the polygon has been transformed, some transformation functions like
// rotate and scale behave differently from what you may expect. The Pose arithmetic
// still work normally after transformation.
void Polygon::transform()
{
    if (isTransformed()) // little speedup
        return;

    double c = fcos(theta);
    double s = fsin(theta);

    ListIterator<Line> vit = edgeIterator();
    while (vit.hasNext())
    {
        Vec2 v = vit.cur().p1();
        v.rotate(s, c);
        v.x += x;
        v.y += y;
        vit.peekPrev().setP2(v);
        vit.cur().setP1(v);
        vit.next();
    }

    setPos(0, 0);
    setOrientation(0);
    boundingBoxValid = false;
}

// "Untransforms" the polygon in a way that the centroid becomes the position
// and the vertices are expressed with respect to the centroid. The orientation
// is set to zero.
void Polygon::untransform()
{
    Vec2 c = centroid();
    ListIterator<Line> eit = edgeIterator();
    while (eit.hasNext())
    {
        Vec2 v = eit.next().p1();
        v -= c;
        eit.peekPrev().setP2(v);
        eit.cur().setP1(v);
        eit.next();
    }

    setPos(c);
}

// Returns true if the pose and all vertices of this polygon are equal to the other.
bool Polygon::operator==(const Polygon& other) const
{
    //qDebug() << "Polygon::operator==() called. this:" << *this << "other:" << other;
    ListIterator<Line> it1 = edgeIterator();
    ListIterator<Line> it2 = other.edgeIterator();
    if (pose() != other.pose())
    {
        //qDebug() << "  poses don't match:" << pose() << other.pose();
        return false;
    }
    while (it1.hasNext() && it2.hasNext())
    {
        if (it1.cur() != it2.cur())
        {
            //qDebug() << "  vertex don't match:" << it1.cur() << it2.cur();
            return false;
        }
        it1.next();
        it2.next();
    }

    return (it1.hasNext() == it2.hasNext());
}

// Maps the Polygon l from the coordinate frame of Pose p to world coordinates.
Polygon operator+(const Polygon& l, const Pose2D& p)
{
    Polygon c = l;
    c += p;
    return c;
}

// Maps the Polygon l into the coordinate frame of the Pose p.
Polygon operator-(const Polygon& l, const Pose2D& p)
{
    Polygon c = l;
    c -= p;
    return c;
}

// Maps the polygon from the frame given by Pose to world coordinates.
// The polygon is assumed to be in local in the frame given by Pose and is
// transformed to world coordinates by Pose arithmetic.
// Only the pose of the polygon is changed, but not the vertex coordinates.
void operator+=(Polygon& pol, const Pose2D& p)
{
    pol.setPose(pol.pose() + p);
    pol.transform();
}

// Maps the polygon into the local coordinates of the frame given by Pose.
// Only the pose of the polygon is changed, but not the vertex coordinates.
void operator-=(Polygon& pol, const Pose2D& p)
{
    pol.setPose(pol.pose() - p);
    pol.transform();
}

// Maps the vector of polygons from the frame given by Pose2D to world coordinates.
// The polygons are assumed to be in local in the frame given by Pose and are
// transformed to world coordinates by Pose arithmetic. The resulting polygons
// are transformed.
Vector<Polygon> operator+(const Vector<Polygon> &v, const Pose2D &p)
{
    Vector<Polygon> tmp;
    tmp.resize(v.size());
    for (int i = 0; i < v.size(); ++i)
    {
        tmp[i] = v[i] + p;
        tmp[i].transform();
    }
    return tmp;
}

// Maps the vector of polygons into the local coordinates of the frame given by Pose2D.
// The resulting polygons are transformed.
Vector<Polygon> operator-(const Vector<Polygon> &v, const Pose2D &p)
{
    Vector<Polygon> tmp;
    tmp.resize(v.size());
    for (int i = 0; i < v.size(); ++i)
    {
        tmp[i] = v[i] - p;
        tmp[i].transform();
    }
    return tmp;
}

// Maps the vector of polygons from the frame given by Pose to world coordinates.
// The polygons are assumed to be in local in the frame given by Pose and is
// transformed to world coordinates by Pose arithmetic.
// The resulting polygons are transformed.
void operator+=(Vector<Polygon> &v, const Pose2D &p)
{
    for (int i = 0; i < v.size(); ++i)
    {
        v[i] += p;
        v[i].transform();
    }
}

// Maps the polygons into the local coordinates of the frame given by Pose.
// The resulting polygons are transformed.
void operator-=(Vector<Polygon> &v, const Pose2D &p)
{
    for (int i = 0; i < v.size(); ++i)
    {
        v[i] -= p;
        v[i].transform();
    }
}

// Maps the list of polygons from the frame given by Pose2D to world coordinates.
// The polygons are assumed to be in local in the frame given by Pose and are
// transformed to world coordinates by Pose arithmetic. The resulting polygons
// are transformed.
LinkedList<Polygon> operator+(const LinkedList<Polygon> &v, const Pose2D &p)
{
    LinkedList<Polygon> tmp;
    ListIterator<Polygon> it = v.begin();
    while (it.hasNext())
    {
        tmp << it.next() + p;
        tmp.last().transform();
    }
    return tmp;
}

// Maps the list of polygons into the local coordinates of the frame given by Pose2D.
// The resulting polygons are transformed.
LinkedList<Polygon> operator-(const LinkedList<Polygon> &v, const Pose2D &p)
{
    LinkedList<Polygon> tmp;
    ListIterator<Polygon> it = v.begin();
    while (it.hasNext())
    {
        tmp << it.next() - p;
        tmp.last().transform();
    }
    return tmp;
}

// Maps the list of polygons from the frame given by Pose to world coordinates.
// The polygons are assumed to be in local in the frame given by Pose and is
// transformed to world coordinates by Pose arithmetic.
// The resulting polygons are transformed.
void operator+=(LinkedList<Polygon> &v, const Pose2D &p)
{
    ListIterator<Polygon> it = v.begin();
    while (it.hasNext())
    {
        Polygon& pol = it.next();
        pol += p;
        pol.transform();
    }
}

// Maps the list of polygons into the local coordinates of the frame given by Pose.
// The resulting polygons are transformed.
void operator-=(LinkedList<Polygon> &v, const Pose2D &p)
{
    ListIterator<Polygon> it = v.begin();
    while (it.hasNext())
    {
        Polygon& pol = it.next();
        pol -= p;
        pol.transform();
    }
}

// Writes the polygon into a data stream.
void Polygon::streamOut(QDataStream &out) const
{
    out << x;
    out << y;
    out << theta;
    out << edges;
}

// Reads the polygon from a data stream.
void Polygon::streamIn(QDataStream &in)
{
    boundingBoxValid = false;
    convexityFlag = 0;
    windingFlag = 0;
    in >> x;
    in >> y;
    in >> theta;
    in >> edges;
}

// Writes the polygon into a data stream.
QDataStream& operator<<(QDataStream& out, const Polygon &o)
{
    o.streamOut(out);
    return out;
}

// Reads the polygon from a data stream.
QDataStream& operator>>(QDataStream& in, Polygon &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const Polygon &o)
{
    if (dbg.autoInsertSpaces())
        dbg << "id:" << o.getId()
            << "pose:" << o.pose()
            << "ccw:" << o.isCCW()
            << "conv:" << o.isConvex()
            << "area:" << o.area()
            << "\nedges:" << o.getEdges();
    else
        dbg << "id: " << o.getId()
            << " pose: " << o.pose()
            << " ccw: " << o.isCCW()
            << " conv: " << o.isConvex()
            << " area:" << o.area()
            << "\nedges: " << o.getEdges();
    return dbg;
}

QDebug operator<<(QDebug dbg, const Polygon* o)
{
    if (dbg.autoInsertSpaces())
        dbg << "id:" << o->getId()
            << "pose:" << o->pose()
            << "ccw:" << o->isCCW()
            << "conv:" << o->isConvex()
            << "int:" << o->isSelfIntersecting()
            << "area:" << o->area()
            << "size:" << o->size();
    else
        dbg << "id: " << o->getId()
            << " pose: " << o->pose()
            << " ccw: " << o->isCCW()
            << " conv: " << o->isConvex()
            << " int: " << o->isSelfIntersecting()
            << " area:" << o->area()
            << " size:" << o->size();
    return dbg;
}

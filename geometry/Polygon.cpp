#include "Polygon.h"
#include "globals.h"
#include "blackboard/State.h"
#include "blackboard/Config.h"
#include "Box.h"
#include "util/ColorUtil.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <GL/glu.h>
#include "clipper.hpp"

// The Polygon class is a general purpose polygon that consists of a number
// of vertices and a 2D transform given by a translation (x,y) and an orientation
// (theta). The mathematical order of the transformation is first rotation then
// translation. To human intuition this is the same as first moving the polygon
// somewhere and then rotating it around its center.
//
// Polygons must have at least 3 vertices. The vertices are stored in a LinkedList
// and must be specified in a counter clockwise order. The LinkedList has an
// advantage in terms of memory management in exchange for not having random
// access to the vertices, but typically the vertices are accessed in a sequence
// (looped over) and random access is not needed. You can add vertices with the
// addVertex() function and the << operator. There are convenient constructors
// and set*() functions (e.g. setUnitSquare()) for constructing polygons that can
// then be scale()-d, translate()-d, and rotate()-d. Polygons are memory-preserving
// so it makes sense to clear() and reuse them.

// The Polygon class provides means to query and manipulate its transformation
// (x,y,theta) relative to the world using pos(), rotation(), translate(), and rotate().
// We call a polygon that has a nonzero transformation "untransformed" in the
// sense that its vertices are given relative to the frame that is specified by
// the transformation given by pos() and rotation(). In this untransformed state,
// the intuitive translate() and rotate() operations can be chained at low cost
// as they only modify the transformation of the polygon, but not its vertices.
// When calling the transform() function, the transformation is "consumed" and
// the vertices of the polygon are converted to world coordinates and pos() and
// rotation() are set to zero. After calling transform(), the rotate() operation
// will not work as expected as the polygon is no longer rotated around its center,
// but around the world origin. Also, the boundingBox() and the getEdges() functions
// make most sense after the polygon has been transform()-ed. Some of the more
// complex intersect() methods such as for Hpm2d or Unicycle or Polygon require the
// polygon to be transformed. In general, you may want to first construct polygons
// whose vertices are expressed relative to their centroid and then accumulate
// transformations that are applied to the polygon over time, e.g. positioning in
// a map and transformation into the robot reference frame. Then, call transform()
// to consume the transformation and have everything work in world coordinates
// thereafter.

int Polygon::idCounter = 0;

Polygon::Polygon()
{
    id = idCounter++;
    boundingBoxValid = false;
    convexityFlag = -1;
    edgesAreComputed = false;
    setPos(0, 0);
    setRotation(0);
}

// Box constructor for convenience.
Polygon::Polygon(double x, double y, double w, double h)
{
    id = idCounter++;
    boundingBoxValid = false;
    convexityFlag = 1;
    edgesAreComputed = false;
    setPos(x, y);
    setRotation(0);

    vertices << Vec2(-w, h);
    vertices << Vec2(-w, -h);
    vertices << Vec2(w, -h);
    vertices << Vec2(w, h);
}

// Triangle constructor.
Polygon::Polygon(const Vec2 &v0, const Vec2 &v1, const Vec2 &v2)
{
    id = idCounter++;
    boundingBoxValid = false;
    convexityFlag = 1;
    edgesAreComputed = false;
    setPos(0, 0);
    setRotation(0);

    vertices << v0;
    vertices << v1;
    vertices << v2;
}

// Sets the color for this polygon to be drawn with.
void Polygon::setColor(const QColor& col)
{
    color = col;
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

// Convenience box set function.
void Polygon::set(double w, double h)
{
    clear();
    convexityFlag = 1;

    vertices << Vec2(-w, h);
    vertices << Vec2(-w, -h);
    vertices << Vec2(w, -h);
    vertices << Vec2(w, h);
}

// Discards all vertices and resets the transformation to 0.
// It is useful for the recycling of Polygon objects.
void Polygon::clear()
{
    vertices.clear();
    x = 0;
    y = 0;
    theta = 0;
    boundingBoxValid = false;
    convexityFlag = -1;
    edgesAreComputed = false;
}

// Returns the number of vertices of this polygon.
int Polygon::size() const
{
    return vertices.size();
}

// Returns the vertices as a QPolygonF. This is used for Qt compatibility.
// The QPolygonF is given in local coordinates (untransformed).
QPolygonF Polygon::polygon() const
{
    QPolygonF pol;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
        pol << it.next();
    return pol;
}

// Returns the pose (x,y,theta) of the Polygon.
Pose2D Polygon::pose() const
{
    return Pose2D(x,y,theta);
}

// Sets the pose (x,y,theta) of the polygon.
void Polygon::setPose(const Pose2D &pose)
{
    setPos(pose.pos());
    setRotation(pose.heading());
}

// Sets the pose (x,y,theta) of the polygon.
void Polygon::setPose(double x, double y, double theta)
{
    setPos(x,y);
    setRotation(theta);
}

// Sets the (x,y) position of the polygon.
void Polygon::setPos(double x, double y)
{
    this->x = x;
    this->y = y;
}

// Sets the (x,y) position of the polygon.
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
// This is a transform accumulating function.
void Polygon::translate(double dx, double dy)
{
    setPos(this->x+dx, this->y+dy);
}

// Translates the polygon by d.
// This is a transform accumulating function.
void Polygon::translate(const Vec2 &d)
{
    setPos(this->x+d.x, this->y+d.y);
}

// Rotates the polygon counter clockwise by the angle "a" given in radians.
// This is a transform accumulating function.
void Polygon::rotate(double a)
{
    setRotation(theta+a);
}

// Returns the rotation theta of the polygon.
double Polygon::rotation() const
{
    return theta;
}

// Sets the rotation theta of the polygon.
void Polygon::setRotation(double a)
{
    theta = a;
}

// Maps the polygon into the frame given by Pose.
// The polygon is assumed to be in world coordinates and is transformed to the local
// coordinates of the frame given by Pose. It works no matter if the polygon is
// transformed or not.
void Polygon::operator-=(const Pose2D &o)
{
    setPos((pos()-Vec2(o.x,o.y)).rotated(-o.z));
    rotate(-o.z);
}

// Maps the polygon to world coordinates from the frame given by Pose.
// The polygon is assumed to be in local in the frame given by Pose and is
// transformed to world coordinates. It works no matter if the polygon is
// transformed or not.
void Polygon::operator+=(const Pose2D &o)
{
    rotate(o.z);
    setPos(pos().rotated(o.z)+Vec2(o.x,o.y));
}

// Grows (or shrinks) the polygon by delta. Delta is the distance by how many meters the
// vertices are pushed outwards along the half angle between the two neighbouring edges.
// If delta is negative, the vertices are pulled invards and the polygon is shrunk.
// The vertices must be given in a counter clockwise order such that the outside of the
// polygon is always to the right. The polygon doesn't have to be convex and it can be
// in an utransformed state. Be aware that for non-convex polygons this operation may
// result in self-intersections.
void Polygon::grow(double delta)
{
    // Compute an offset to each vertex fisrt.
    Vec2 offset[size()];
    int i = 0;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2 v1 = (it.peekCur() - it.peekPrev()).normalized();
        Vec2 v2 = (it.peekNext() - it.peekCur()).normalized();
        Vec2 v3 = (v1+v2).normal().normalized();
        offset[i] = delta*v3;
        i++;

        it.next();
    }

    // Apply the offsets to the vertices.
    it = vertices.begin();
    i = 0;
    while (it.hasNext())
    {
        Vec2& v = it.next();
        v += offset[i];
        i++;
    }

    boundingBoxValid = false;
    edgesAreComputed = false;
}

// Scales (multiplies) the polygon vertices by the factors sx and sy.
// When the polygon is in an untransformed state and the center is
// say the centroid of the polygon, this scaling has a growing effect.
// If the polygon is transformed, the result might not be what you
// expect.
void Polygon::scale(double sx, double sy)
{
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2& v = it.next();
        v.x *= sx;
        v.y *= sy;
    }

    boundingBoxValid = false;
    edgesAreComputed = false;
}


// Clips this polygon with the cp polygon such that only the parts of this polygon
// remain that overlap the cp polygon. If this polygon is A and cp is B,
// then you get A intersect B.
Vector<Polygon> Polygon::clipIntersect(const Polygon &cp)
{
    double clipperFactor = 1000;

    ClipperLib::Path sub;
    ListIterator<Vec2> vit = vertexIterator();
    while (vit.hasNext())
    {
        const Vec2& v = vit.next();
        sub << ClipperLib::IntPoint(v.x*clipperFactor, v.y*clipperFactor);
    }

    ClipperLib::Path clp;
    ListIterator<Vec2> cpit = cp.vertexIterator();
    while (cpit.hasNext())
    {
        const Vec2& v = cpit.next();
        clp << ClipperLib::IntPoint(v.x*clipperFactor, v.y*clipperFactor);
    }

    //get the intersection of the subject and clip polygons ...
    ClipperLib::Clipper clpr;
    clpr.AddPath(sub, ClipperLib::ptSubject, true);
    clpr.AddPath(clp, ClipperLib::ptClip, true);
    ClipperLib::Paths solution;
    clpr.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);

    Vector<Polygon> ret;
    for (uint i = 0; i < solution.size(); i++)
    {
        Polygon p;
        for (uint j = 0; j < solution[i].size(); j++)
            p.addVertex((double)solution[i][j].X/clipperFactor, (double)solution[i][j].Y/clipperFactor);
        ret << p;
    }

    return ret;
}

// Reverses the order of the vertices.
// This is sometimes needed to restore the CCW order.
void Polygon::reverseOrder()
{
    vertices.reverse();
    boundingBoxValid = false;
    edgesAreComputed = false;
}

// Computes the centroid of the polygon.
// The centroid is given in world coordinates (transformed).
Vec2 Polygon::centroid() const
{
    Vec2 c;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
        c += it.next();
    c /= size();
    c += pos();
    return c;
}

// Returns the edges of the polygon as a list of lines.
// The edges are given in local coordinates (untransformed).
// The order of the returned edges is the same as the order of the vertices (CCW).
const LinkedList<Line>& Polygon::getEdges() const
{
    if (edgesAreComputed)
    {
        return edges;
    }
    edgesAreComputed = true;

    edges.clear();
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        edges << Line(it.peekCur(), it.peekNext());
        it.next();
    }
    return edges;
}

// Returns a LinkedList of the vertices (corners) of the polygon.
// The vertices will be given in local coordinates (untransformed).
const LinkedList<Vec2> &Polygon::getVertices() const
{
    return vertices;
}

// Returns the transformed vertices.
LinkedList<Vec2> Polygon::getTransformedVertices() const
{
    LinkedList<Vec2> transformedVertices;

    if (isTransformed()) // little speedup
        return vertices;

    double c = fcos(theta);
    double s = fsin(theta);
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2& v = it.next();
        v.rotate(s, c);
        v.x += x;
        v.y += y;
        transformedVertices << v;
    }

    return transformedVertices;
}

// Sets (overwrites) the vertices of the polygon.
// This resets the transform, too.
void Polygon::setVertices(const LinkedList<Vec2> &v)
{
    clear();
    vertices = v;
}

// Returns the axis aligned bounding box of the polygon in local
// coordinates (untransformed). This is enough for the intersects()
// methods of this polygon, but if you are using the aabb for your
// own purposes, be aware that the polygon should be transformed
// for the bounding box to be in world coordinates.
const Box &Polygon::boundingBox() const
{
    if (boundingBoxValid)
        return aabb;
    boundingBoxValid = true;

    ListIterator<Vec2> it = vertices.begin();
    const Vec2& v = it.next();
    double left = v.x;
    double right = v.x;
    double top = v.y;
    double bottom = v.y;
    while (it.hasNext())
    {
        const Vec2& v = it.next();

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
// The polygon does not need to be convex or transformed.
double Polygon::diameter() const
{
    double d = 0;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        const Vec2& v1 = it.next();
        ListIterator<Vec2> it2 = it;
        while (it2.hasNext())
        {
            const Vec2& v2 = it2.next();
            d = qMax(d, (v1-v2).norm());
        }
    }
    return d;
}

// Returns the area of the polygon.
// The polygon does not have to be convex or transformed.
double Polygon::area() const
{
    double a = 0;
    ListIterator<Vec2> it = vertexIterator();
    do
    {
        const Vec2& v1 = it.peekCur();
        const Vec2& v2 = it.peekNext();
        a += v1.x*v2.y-v2.x*v1.y;
        it.next();
    } while (it.hasNext());
    return fabs(0.5*a);
}

// Computes the shortest distance between this polygon and the point p.
// p is expected to be given in world coordinates, but this polygon does
// not need to be transformed. The distance is computed between p and the
// point on the boundary of the polygon that is closest to p, so inside a
// polygon you would still get a positive distance.
double Polygon::distance(const Vec2 &p) const
{
    return (p-closestPoint(p)).norm();
}

// Returns the closest point on the polygon boundary to the given point p.
// p is given in world coordinates and the returned closest point
// is also given in world coordinates (transformed). The polygon
// itself does not need to be in transformed state.
Vec2 Polygon::closestPoint(const Vec2 &p) const
{
    // Transform the point to local coordinates.
    Vec2 point = p;
    if (!isTransformed())
        point -= pose();

    // Determine the closest point.
    double minDist = std::numeric_limits<double>::max();
    Vec2 cp,pp;
    ListIterator<Line> e = edgeIterator();
    while (e.hasNext())
    {
        const Line& edge = e.next();
        pp = edge.closestPoint(point);
        double n = (pp-point).norm2();
        if (n < minDist)
        {
            cp = pp;
            minDist = n;
        }
    }

    // Transform the closest point back to world.
    if (!isTransformed())
        cp += pose();

    return cp;
}

// Returns the closest edge of the polygon to the given point p.
// p is given in world coordinates and the returned edge is given
// as a Line also in world coordinates (transformed). The polygon
// itself does not need to be in transformed state. More than one
// edge can have the same distance to a point. In such case, it
// is a bit of luck which edge is returned.
Line Polygon::closestEdge(const Vec2 &p) const
{
    // Transform the point to local coordinates.
    Vec2 point = p;
    if (!isTransformed())
        point -= pose();

    // Determine the closest point.
    double minDist = std::numeric_limits<double>::max();
    Vec2 cp;
    Line cl;
    ListIterator<Line> e = edgeIterator();
    while (e.hasNext())
    {
        const Line& edge = e.next();
        cp = edge.closestPoint(point);
        double n = (cp-point).norm2();
        if (n < minDist)
        {
            minDist = n;
            cl = edge;
        }
    }

    // Transform the closest line back to world.
    if (!isTransformed())
        cl += pose();

    return cl;
}

// Returns the normal of the closest edge or vertex of the polygon to the
// given point p. The normal always points to the outside of the polygon.
// Edge normals are perpendicular to the edge and vertex normals point
// along the half angle between the adjacent edges.
// p is given in world coordinates and the returned normal vector is given
// also in world coordinates (transformed). The polygon itself does not
// need to be in a transformed state.
Vec2 Polygon::closestNormal(const Vec2 &p) const
{
    // Transform the point to local coordinates.
    Vec2 point = p;
    if (!isTransformed())
        point -= pose();

    // Determine the closest point.
    double minDist = std::numeric_limits<double>::max();
    Vec2 closestPoint;
    Vec2 closestNormal;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        const Vec2& v1 = it.peekPrev();
        const Vec2& v2 = it.peekCur();
        const Vec2& v3 = it.peekNext();
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
                closestNormal = (dv1+dv2).normal().normalized();
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
                closestNormal.flip();
                //qDebug() << "   edge" << v2 << v3 << "is closest" << n << "in point:" << closestPoint << "normal:" << closestNormal;
            }
        }
    }

    // Transform to world.
    closestNormal.rotate(theta);

    return closestNormal;
}

Polygon Polygon::convexHull() const
{
    if (vertices.isEmpty())
        return Polygon();

    std::vector<cv::Point2f> pol;
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2& v = it.next();
        pol.push_back(cv::Point2f(v.x, v.y));
    }

    std::vector<cv::Point2f> chPoints;
    cv::convexHull(pol, chPoints);

    Polygon ch;
    for (uint i = 0; i < chPoints.size(); i++)
        ch << Vec2(chPoints[i].x, chPoints[i].y);
    ch.setPos(pos());
    ch.setRotation(rotation());

    return ch;
}

// Triangulates the Polygon into a set of triangles using the Ear Clipping method.
// If the triangulation fails (it does sometimes), an empty list is returned.
// The triangles are returned in local coordinates (untransformed).
Vector<Polygon> Polygon::triangulate() const
{
    //qDebug() << "Triangulating" << *this;
    Vector<Polygon> trigs;

    Polygon workingCopy = *this;

    while (workingCopy.size() > 3)
    {
        bool earDetected = false;
        ListIterator<Vec2> it = workingCopy.vertexIterator();
        while (it.hasNext())
        {
            const Vec2& v0 = it.peekPrev();
            const Vec2& v1 = it.peekCur();
            const Vec2& v2 = it.peekNext();
            it.next();

            //qDebug() << v0 << v1 << v2 << Vec2(v2-v1).isLeftOf(v1-v0) << "intersects:" << copy.intersects(Line(v0, v2), false);

            // Detect ear. Every polygon has at least two ears.
            // It has to be a concave corner that doesn't contain any other vertex.
            if (Vec2(v2-v1).isLeftOf(v1-v0)) // concave corner detected
            {
                Polygon triangle(v0, v1, v2);

                // Check the other vertices of the working polygon if they lie inside the ear.
                bool vertexContainedInEar = false;
                ListIterator<Vec2> it2 = workingCopy.vertexIterator();
                while (it2.hasNext())
                {
                    const Vec2& v = it2.next();
                    if (v != v0 && v != v1 && v != v2 && triangle.intersects(v))
                    {
                        vertexContainedInEar = true;
                        break; // break out of the vertex containment check
                    }
                }

                if (!vertexContainedInEar)
                {
                    // Clip the ear.
                    trigs << triangle;
                    workingCopy.removeVertex(v1);
                    earDetected = true;
                    break; // restart a new ear search
                }
            }
        }

        if (!earDetected)
        {
            //qDebug() << "Polygon::triangulate(): No ear detected! Would loop forever.";
            //trigs.clear();
            return trigs;
        }
    }

    //qDebug() << "Triangulating finished." << *this;
    trigs << workingCopy;
    return trigs;
}

// Declares the polygon to be convex.
// Nothing is checked so you better know what you are doing.
void Polygon::setConvex()
{
    convexityFlag = 1;
}

// Returns true if the polygon is convex.
// The vertices have to be given in counterclockwise order, otherwise the test result is undefined.
// The polygon does not need to be transformed.
bool Polygon::isConvex() const
{
    if (convexityFlag == -1) // -1 means unknown.
    {
        convexityFlag = 1; // Assume convexity.
        ListIterator<Vec2> it = vertices.begin();
        while (it.hasNext())
        {
            const Vec2& v1 = it.peekPrev();
            const Vec2& v2 = it.peekCur();
            const Vec2& v3 = it.peekNext();

            if ((v2.x-v1.x)*(v3.y-v2.y)-(v2.y-v1.y)*(v3.x-v2.x) < 0) // Right turn test.
            {
                convexityFlag = 0; // Set to non-convexity.
                break;
            }

            it.next();
        }
    }

    return convexityFlag;
}

// Returns true if the polygon is in a transformed state, i.e.
// its transformation has been consumed and its vertices have been
// transformed to world coordinates.
bool Polygon::isTransformed() const
{
    return (x < EPSILON && x > -EPSILON && y < EPSILON && y > -EPSILON && theta < EPSILON && theta > -EPSILON);
}

// Performs a collision check with the edges of this polygon and the
// holonomic bang described by the kf. If no collision occurs, -1 is returned.
// Otherwise the relative time dt is returned to indicate the future time of
// a collision relative to this bang. If the collision would occur later than
// the dt in the keyframe, -1 is returned. Note that no containment check is
// performed. The entire bang could be inside the polygon and no collision is
// reported. Only intersections with the edges of the polygon are found.
// The polygon has to be in a transformed state and the kf is given in world.
double Polygon::intersects(const Hpm2D &kf) const
{
    //qDebug() << "  Polygon::intersects(Hpm2D):" << kf;

    // Bounding box check.
    boundingBox();
    if (!aabb.intersects(kf.boundingBox()))
        return -1;

    double ct = -1;

    ListIterator<Line> e = edgeIterator();
    while (e.hasNext())
    {
        const Line& edge = e.next();

        // Intersect the edge with the hpm2d bang.
        double cct = kf.intersects(edge);

        // Keep track of the smallest collision time so far.
        if (cct >= 0 && (ct < 0 || cct < ct))
            ct = cct;
    }

    return ct;
}

// Performs a collision check with the edges of this polygon and the
// unicycle bang described by u. If no collision occurs, -1 is returned.
// Otherwise, the relative time dt is returned to indicate the future time of
// a collision relative to this bang. If the collision would occur later than
// the dt in u, -1 is returned. Note that no containment check is performed.
// The entire bang could be inside the polygon and no collision is reported.
// Only intersections with the edges of the polygon are found.
// The polygon has to be in a transformed state and the u is given in world.
double Polygon::intersects(const Unicycle &u) const
{
    //qDebug() << "  Polygon::intersects(Unicycle):" << u;

    // Bounding box check.
    boundingBox();
    if (!aabb.intersects(u.boundingBox()))
        return -1;

    double ct = -1;

    // Full edge check.
    ListIterator<Line> e = edgeIterator();
    while (e.hasNext())
    {
        const Line& edge = e.next();

        // Intersect the edge with the unicycle.
        double cct = u.intersects(edge);

        // Keep track of the smallest collision time so far.
        if (cct >= 0 && (ct < 0 || cct < ct))
            ct = cct;
    }

    return ct;
}

// Returns true if the polygon p intersects with this one. If both
// polygons are convex, the SAT algorithm is used to reliably detect
// all kinds of intersections (also containment). If at least one of
// the polygons is non-convex, only a weaker corner containment check
// is used, which misses overlapping polygons where no vertex is
// contained by the other, for example when two squares perfectly
// overlap, but then one square is rotated by 45 degrees. A more
// reliable algorithm would have to decompose the polygons into convex
// sections and check if any of them intersect. This would probably be
// much slower than this point inclusion mockup.
// Assuming both polygons are transformed.
bool Polygon::intersects(const Polygon &p) const
{
    //qDebug() << "  Polygon::intersects(p):" << p;
    //qDebug() << "  this:" << *this;

    // Bounding box check. Reject cases whose bounding boxes don't overlap.
    boundingBox();
    if (!aabb.intersects(p.boundingBox()))
        return false;

    // The polygons have to be in a transformed state for the SAT.

    if (isConvex() && p.isConvex())
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
        // no intersection. This way, I might miss an edge whose normal could be
        // a separating axis, even if both polygons lie on the same side of the
        // edge, and so I have to look at more edges than the standard SAT, but
        // I save computation time by computing the cross product only for the
        // vertices of one polygon, and quickly discarding an edge as soon as
        // a cross product evaluates positive. Not sure which way it's faster,
        // but this way is easier to code.

        // Test the edges of the source against the points of the target.
        ListIterator<Vec2> sourceIterator = vertexIterator();
        ListIterator<Vec2> targetIterator = p.vertexIterator();
        while (sourceIterator.hasNext())
        {
            const Vec2& v1 = sourceIterator.peekCur();
            const Vec2& v2 = sourceIterator.peekNext();
            sourceIterator.next();

            bool allPointsAreRightOf = true;
            targetIterator.reset();
            while (targetIterator.hasNext())
            {
                const Vec2& p = targetIterator.next();
                if ((v2.x-v1.x)*(p.y-v1.y)-(v2.y-v1.y)*(p.x-v1.x) >= 0) // right of test with scalar product
                {
                    allPointsAreRightOf = false;
                    break; // next source edge
                }
            }

            if (allPointsAreRightOf)
                return false;
        }

        // Test the edges of the target against the points of the source.
        targetIterator.reset();
        while (targetIterator.hasNext())
        {
            const Vec2& v1 = targetIterator.peekCur();
            const Vec2& v2 = targetIterator.peekNext();
            targetIterator.next();

            bool allPointsAreRightOf = true;
            sourceIterator.reset();
            while (sourceIterator.hasNext())
            {
                const Vec2& p = sourceIterator.next();
                if ((v2.x-v1.x)*(p.y-v1.y)-(v2.y-v1.y)*(p.x-v1.x) >= 0) // right of test with scalar product
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
        // All corners of one polygon are tested to lie in the other, vice versa.
        // If at least one test is positive, the polygons intersect for sure and
        // the test can be aborted. This test misses cases where two polygons
        // overlap, but none of the corners are contained. For example, two
        // squares lying on top of each other but one is rotated by 45 degrees.

        ListIterator<Vec2> sourceIterator = vertexIterator();
        while (sourceIterator.hasNext())
        {
            const Vec2& v0 = sourceIterator.next();
            if (p.intersects(v0))
                return true;
        }

        ListIterator<Vec2> targetIterator = p.vertexIterator();
        while (targetIterator.hasNext())
        {
            const Vec2& v0 = targetIterator.next();
            if (intersects(v0))
                return true;
        }

        // Test the centroids, too, to catch a few of those "cross examples"
        // explained above. I have observed it happening.
        if (p.intersects(centroid()))
            return true;
        if (intersects(p.centroid()))
            return true;

        return false;
    }

    return false;
}

// Returns true if the line l intersects with any edge of this polygon.
// This is an edge intersection test only that will not detect containment.
// There are no requirements on the polygon. It does not have to be convex.
// The polygon does not have to be transformed.
bool Polygon::intersects(const Line& line) const
{
    //qDebug() << "      Polygon::intersects(Line):" << l << "p:" << getId();

    // Transform the line into the reference frame of the polygon.
//    Line line = l;
//    if (!isTransformed())
//    {
//        qDebug() << "line needs to be transformed.";
//        line.translate(-x, -y);
//        line.rotate(-theta);
//    }

    // Bounding box check.
    boundingBox();
    if (!aabb.intersects(line))
        return false;

    // Full edge check.
    ListIterator<Line> e = edgeIterator();
    while (e.hasNext())
    {
        const Line& edge = e.next();
        if (edge.intersects(line))
        {
            //qDebug() << "         edge" << edge << "intersects with line" << line;
            return true;
        }
    }

    return false;
}

// Returns true if the polygon contains the point p. It will return true for a point
// lying exactly on the boundary of the polygon. The point is given in world coordinates,
// but the polygon does not have to be transformed. There is a speed up if the polygon
// is convex.
bool Polygon::intersects(const Vec2 &v) const
{
    //qDebug() << "  Polygon::intersects(Vec2):" << v << "with polygon id:" << id << isTransformed();

    // Transform the point to local coordinates.
    Vec2 p = v;
    if (!isTransformed())
    {
        p -= pos();
        p.frotate(-theta);
    }

    // Bounding box check.
    boundingBox();
    if (!aabb.intersects(p))
        return false;

    if (isConvex())
    {
        // Point intersection with a convex polygon is implemented using
        // a scalar product test with every edge. The point has to be to the left of
        // every edge in order to be contained in a counter clockwise convex polygon.
        // http://totologic.blogspot.de/2014/01/accurate-point-in-triangle-test.html
        // As soon as one edge is found the point lies on the right of, the algorithm
        // can abort and report that there is no collision.

        ListIterator<Vec2> it = vertices.begin();
        while (it.hasNext())
        {
            const Vec2& v1 = it.peekCur();
            const Vec2& v2 = it.peekNext();
            it.next();

            // The strictness of this rightof test determines whether a point on the
            // boundary of the polygon counts as included or not. A strict rightof test
            // includes the boundary.
            if ((p.x-v1.x)*(v2.y-v1.y)-(p.y-v1.y)*(v2.x-v1.x) >= 0) // Strict p rightof edge test.
                return false;
        }

        return true;
    }
    else
    {
        // A point intersection test with a non-convex polygon is almost as easy.
        // We are using the winding number algorithm as described here:
        // http://geomalgorithms.com/a03-_inclusion.html

        int wn = 0;

        // Loop through all edges of the polygon.
        ListIterator<Vec2> it = vertices.begin();
        while (it.hasNext())
        {
            const Vec2& v1 = it.peekCur();
            const Vec2& v2 = it.peekNext();
            it.next();

            if (v1.y <= p.y)
            {
                if (v2.y > p.y) // an upward crossing
                {
                    double side = (p.x-v1.x)*(v2.y-v1.y)-(p.y-v1.y)*(v2.x-v1.x);
                    if (fabs(side) <= EPSILON) // Special case right on the line.
                        return true;
                    if (side < 0) // Strict p leftof edge test.
                        wn++; // have a valid up intersect
                }
            }
            else
            {
                if (v2.y <= p.y) // a downward crossing
                {
                    double side = (p.x-v1.x)*(v2.y-v1.y)-(p.y-v1.y)*(v2.x-v1.x);
                    if (fabs(side) <= EPSILON) // Special case right on the line.
                        return true;
                    if (side > 0) // Strict p rightof edge test.
                        wn--; // have a valid down intersect
                }
            }
        }

        return (wn != 0);
    }

    return true;
}

// Returns true if the polygon intersects the circle specified by its center p
// and its radius. The center of the circle is given in world coordinates, but
// the polygon does not have to be transformed.
bool Polygon::intersects(const Vec2 &p, double radius) const
{
    // Containment check.
    if (intersects(p))
        return true;

    // Distance check.
    return (distance(p) <= radius);
}

// Computes the intersection point between the boundary of this polygon and the
// line l. If the line does not intersect the boundary, a zero Vec2() is returned.
// If the line intersects the boundary twice, the intersection point that is
// first found is returned.
Vec2 Polygon::intersection(const Line &l) const
{
    Vec2 ip;

    ListIterator<Line> e = edgeIterator();
    while (e.hasNext())
    {
        const Line& edge = e.next();
        ip = edge.intersection(l);
        if (!ip.isNull())
            return ip;
    }

    return ip;
}

// Returns the point at which a ray from from to to first intersects the polygon.
// If the ray does not intersect, to is returned. From and to are given in world
// coordinates. The polygon does not need to be transformed.
Vec2 Polygon::rayIntersection(const Vec2 &from, const Vec2 &to) const
{
    Vec2 lfrom = from;
    Vec2 lto = to;
    if (!isTransformed())
    {
        lfrom -= pose();
        lto -= pose();
    }

    Vec2 best = lto;
    Line l(lfrom, lto);
    double d = l.length();

    ListIterator<Line> e = edgeIterator();
    while (e.hasNext())
    {
        const Line& edge = e.next();
        Vec2 ip = edge.intersection(l);
        if (!ip.isNull())
        {
            double dd = (ip-lfrom).norm();
            if (dd < d)
            {
                best = ip;
                d = dd;
            }
        }
    }

    if (!isTransformed())
        best += pose();

    return best;
}

// Generates a unit triangle.
void Polygon::setUnitTriangle()
{
    clear();
    addVertex(0, 0.5);
    addVertex(-0.7, -0.5);
    addVertex(0.7, -0.5);
    convexityFlag = 1;
}

// Generates a unit square.
void Polygon::setUnitSquare()
{
    clear();
    addVertex(-1, 1);
    addVertex(-1, -1);
    addVertex(1, -1);
    addVertex(1, 1);
    convexityFlag = 1;
}

// Generates a unit hexagon.
void Polygon::setUnitHexagon()
{
    clear();
    double s = tan(30*DEG_TO_RAD);
    addVertex(1, s);
    addVertex(0, 2*s);
    addVertex(-1, s);
    addVertex(-1, -s);
    addVertex(0, -2*s);
    addVertex(1, -s);
    convexityFlag = 1;
    return;
}

// Generates a unit octogon.
void Polygon::setUnitOctogon()
{
    clear();
    addVertex(-0.5, 1.0);
    addVertex(-1.0, 0.5);
    addVertex(-1.0, -0.5);
    addVertex(-0.5, -1.0);
    addVertex(0.5, -1.0);
    addVertex(1.0, -0.5);
    addVertex(1.0, 0.5);
    addVertex(0.5, 1.0);
    convexityFlag = 1;
}

// Draws the polygon on a QPainter.
// It does not matter whether the polygon is transformed or not.
void Polygon::draw(QPainter *painter) const
{
    painter->save();
    painter->translate(x, y);
    painter->rotate(theta*RAD_TO_DEG);
    painter->drawPath(shape());

    // Draw points at the vertices.
//    painter->save();
//    painter->setOpacity(1.0);
//    painter->setPen(colorUtil.pen);
//    painter->setBrush(colorUtil.brush);
//    ListIterator<Vec2> it = vertexIterator();
//    while (it.hasNext())
//    {
//        const Vec2& v = it.next();
//        painter->drawEllipse(v, 0.008, 0.008);
//    }
//    painter->restore();

    painter->restore();
}

// Draws the polygon in an OpenGL context.
void Polygon::draw(const QColor& color) const
{
    glPushMatrix();
    glTranslated(x, y, 0);
    glRotated(theta*RAD_TO_DEG, 0, 0, 1);

    //qDebug() << "Polygon draw:" << color << color.redF() << color.greenF() << color.blueF() << color.alphaF();
    //boundingBox().draw();

    VecN<4> currentColor;
    glGetDoublev(GL_CURRENT_COLOR, currentColor.data()); // remember the color we had before drawing

    if (color.isValid())
        glColor4d(color.redF(), color.greenF(), color.blueF(), color.alphaF());

    if (isConvex())
    {
        glBegin(GL_POLYGON);
        ListIterator<Vec2> it = vertexIterator();
        while (it.hasNext())
        {
            const Vec2& v = it.next();
            glVertex2f(v.x, v.y);
        }
        glEnd();
    }
    else
    {
        Vector<Polygon> triangles = triangulate();
        glBegin(GL_TRIANGLES);
        for (uint i = 0; i < triangles.size(); i++)
        {
            ListIterator<Vec2> it = triangles[i].vertexIterator();
            while (it.hasNext())
                glVertex2dv(it.next());
        }
        glEnd();
    }

    glColor3f(0, 0, 0);
    glLineWidth(3);
    glEnable(GL_LINE_SMOOTH);
    glBegin(GL_LINE_LOOP);
    ListIterator<Vec2> it2 = vertexIterator();
    while (it2.hasNext())
    {
        const Vec2& v = it2.next();
        glVertex2d(v.x, v.y);
    }
    glEnd();
    glColor4dv(currentColor.data()); // restore the saved color
    glPopMatrix();
}

// This method is used by the Qt graphics view framework.
QRectF Polygon::boundingRect() const
{
    return polygon().boundingRect();
}

// Returns the shape of the polygon as a QPainterPath in local coordinates.
// shape() is only used for easy drawing with QPainter.
QPainterPath Polygon::shape() const
{
    QPainterPath pp;
    pp.moveTo(vertices.first());
    ListIterator<Vec2> it = vertices.begin();
    it.next();
    while (it.hasNext())
    {
        const Vec2& v = it.next();
        pp.lineTo(v);
    }
    pp.lineTo(vertices.first());
    return pp;
}

// Appends a vertex to the polygon.
void Polygon::addVertex(double x, double y)
{
    addVertex(Vec2(x,y));
    return;
}

// Appends a vertex to the polygon.
void Polygon::addVertex(const Vec2 &p)
{
    vertices << p;
    boundingBoxValid = false;
    convexityFlag = -1;
    edgesAreComputed = false;
    return;
}

// Appends a vertex to the polygon.
Polygon& Polygon::operator<<(const Vec2 &p)
{
    addVertex(p);
    return *this;
}

// Appends a vector of vertices to the polygon.
Polygon& Polygon::operator<<(const Vector<Vec2> &vp)
{
    for (uint i = 0; i < vp.size(); i++)
        addVertex(vp[i]);
    return *this;
}

// Removes the given vertex from the polygon, if it exists.
void Polygon::removeVertex(const Vec2 &p)
{
    vertices.remove(p);
    boundingBoxValid = false;
    convexityFlag = -1;
    edgesAreComputed = false;
}

// Returns an iterator that can be used to conveniently cycle
// through the corners of the polygon.
ListIterator<Vec2> Polygon::vertexIterator() const
{
    return vertices.begin();
}

// Returns an iterator that can be used to conveniently cycle
// through the edges of the polygon.
ListIterator<Line> Polygon::edgeIterator() const
{
    if (!edgesAreComputed)
        getEdges();
    return edges.begin();
}

// Consumes the current transformation in a way that it transforms all
// vertices to world coordinates and then resets the transformation to zero.
// After the polygon has been transformed, it's better not to use any
// transformation functions. Especially the rotation will not work as expected.
void Polygon::transform()
{
    if (isTransformed()) // little speedup
        return;

    double c = fcos(theta);
    double s = fsin(theta);

    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2& v = it.next();
        v.rotate(s, c);
        v.x += x;
        v.y += y;
    }

    setPos(0, 0);
    setRotation(0);
    boundingBoxValid = false;
    edgesAreComputed = false;
}

// Untransforms the polygon in a way that the centroid becomes the transformation
// and the vertices are expressed with respect to the centroid.
void Polygon::untransform()
{
    Vec2 c = centroid();
    ListIterator<Vec2> it = vertices.begin();
    while (it.hasNext())
    {
        Vec2& v = it.next();
        v -= c;
    }

    setPos(c);
}

// Writes the polygon into a data stream.
void Polygon::streamOut(QDataStream &out) const
{
    out << x;
    out << y;
    out << theta;
    out << vertices;
}

// Reads the polygon from a data stream.
void Polygon::streamIn(QDataStream &in)
{
    in >> x;
    in >> y;
    in >> theta;
    in >> vertices;
    edgesAreComputed = false;
    boundingBoxValid = false;
    convexityFlag = -1;
}

QDataStream& operator<<(QDataStream& out, const Polygon &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, Polygon &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const Polygon &o)
{
    if (dbg.autoInsertSpaces())
        dbg << "id:" << o.getId() << "pose:" << o.pose() << "points:" << o.getVertices();
    else
        dbg << "id: " << o.getId() << " pose: " << o.pose() << " points: " << o.getVertices();
    return dbg;
}

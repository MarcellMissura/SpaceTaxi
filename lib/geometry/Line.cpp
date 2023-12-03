#include "Line.h"
#include "board/Config.h"
#include "lib/util/DrawUtil.h"
#include "GL/gl.h"
#include <cmath>

// The Line object is a line segment defined by two points (x1,y1) and (x2,y2).

Line::Line()
{
    id = -1;
    type = BlockingLine;
    ang = NAN;
}

// Creates a Line with the points p1 and p2.
Line::Line(int id, const Vec2 &p1, const Vec2 &p2, int t)
{
    this->id = id;
    set(p1, p2, t);
}

// Creates a Line with the points p1 and p2.
Line::Line(const Vec2 &p1, const Vec2 &p2, int t)
{
    id = -1;
    set(p1, p2, t);
}

// Creates a Line from (x1,y1) to (x2,y2).
Line::Line(double x1_, double y1_, double x2_, double y2_, int t)
{
    id = -1;
    set(x1_, y1_, x2_, y2_, t);
}

// Returns true if both vertices of this and the other line match.
bool Line::operator==(const Line &other) const
{
    return (p1() == other.p1() && p2() == other.p2());
}

// Returns true unless both vertices of this and the other line match.
bool Line::operator!=(const Line &other) const
{
    return !(p1() == other.p1() && p2() == other.p2());
}

// Sets the end points of the line.
void Line::set(const Vec2 &p1, const Vec2 &p2, int t)
{
    set(p1.x, p1.y, p2.x, p2.y, t);
}

// Sets the coordinates of the line such that in the end x1 < x2.
void Line::set(double x1_, double y1_, double x2_, double y2_, int t)
{
    this->type = t;
    this->vp1.x = x1_;
    this->vp2.x = x2_;
    this->vp1.y = y1_;
    this->vp2.y = y2_;
    ang = NAN;
}

int Line::getId() const
{
    return id;
}

void Line::setId(int value)
{
    id = value;
}

bool Line::isBlockingLine() const
{
    return (type == BlockingLine);
}

bool Line::isSightLine() const
{
    return (type == SightLine);
}

void Line::setBlockingLine()
{
    type = BlockingLine;
}

void Line::setSightLine()
{
    type = SightLine;
}

void Line::setType(int t)
{
    type = t;
}

int Line::getType() const
{
    return type;
}

// Returns the Euclidean length of the line.
double Line::length() const
{
    return sqrt((vp2.x-vp1.x)*(vp2.x-vp1.x)+(vp2.y-vp1.y)*(vp2.y-vp1.y));
}

// Returns the squared Euclidean length of the line.
double Line::length2() const
{
    return (vp2.x-vp1.x)*(vp2.x-vp1.x)+(vp2.y-vp1.y)*(vp2.y-vp1.y);
}

// Returns the angle of the line with respect to the x-axis in the range -PI to PI.
// The range from -PI to PI results from a line actually being a vector from p1 to p2.
double Line::angle() const
{
    if (isnan(ang))
        ang = atan2((vp2.y-vp1.y),(vp2.x-vp1.x));
    return ang;
}

// Returns the angle of this line with respect to the other line in the range -PI to PI.
// The range from -PI to PI results from a line actually being a vector from p1 to p2.
double Line::angle(const Line& line) const
{
    return ffpicut(angle()-line.angle());
}

// Returns the normalized normal of the line.
// The normal is pointing to the left of the line with respect to the p1 to p2 direction.
Vec2 Line::normal() const
{
    return Vec2(vp1.y-vp2.y, vp2.x-vp1.x)/length();
}

// Returns the vector pointing from p1 to p2.
Vec2 Line::lineVector() const
{
    return (vp2-vp1);
}

// Evaluates the line at position x, i.e. returns the y
// coordinate at x even if x is outside of the line segment.
double Line::evaluateAt(double x) const
{
    return vp1.y + (vp2.y-vp1.y)*(x-vp1.x)/(vp2.x-vp1.x);
}

// Returns a point p at a position along the line determined by coeff
// such that p = p1 + coeff(p2-p1).
Vec2 Line::interpolate(double coeff) const
{
    return vp1 + coeff * (vp2 - vp1);
}

// Interpolates this line with l such that this = coeff*this + (1-coeff)*l.
void Line::interpolate(const Line &l, double coeff)
{
    set(vp1.x+coeff*(l.vp1.x-vp1.x), vp1.y+coeff*(l.vp1.y-vp1.y), vp2.x+coeff*(l.vp2.x-vp2.x), vp2.y+coeff*(l.vp2.y-vp2.y));
}

// Returns an interpolated line between this line and l such that Line = coeff*this + (1-coeff)*l.
Line Line::interpolated(const Line &l, double coeff) const
{
    Line line = *this;
    line.interpolate(l, coeff);
    return line;
}

// Returns the vector v projected onto this line.
Vec2 Line::projection(const Vec2 &v) const
{
    return (v-vp1).projectedOnVector(lineVector())+vp1;
}

// Projects line l onto this one and then computes their union.
// A new line is returned and this line remains unmodified.
Line Line::projectedUnion(const Line &l) const
{
    Vec2 l1 = lineVector();
    double o1 = ((l.p1()-p1())*l1)/(l1*l1);
    double o2 = ((l.p2()-p1())*l1)/(l1*l1);
    return Line(p1()+min(0.0,min(o1,o2))*l1, p1()+max(1.0,max(o1,o2))*l1);
}

// Projects line l onto this one and then computes their union modifying this line.
void Line::projectionUnion(const Line &l)
{
    Vec2 left1 = p1();
    Vec2 right1 = p2();
    Vec2 left2 = l.p1();
    Vec2 right2 = l.p2();
    Vec2 l1 = right1-left1;
    double o1 = ((left2-left1)*l1)/(l1*l1);
    double o2 = ((right2-left1)*l1)/(l1*l1);
    setP1(left1+min(0.0,min(o1,o2))*l1);
    setP2(left1+max(1.0,max(o1,o2))*l1);
}

// Returns the projection distance of line l onto this line.
// The projection distance is the sum of the absolute orthogonal
// distances of p1 and p2 of line l to this line.
double Line::projectionDistance(const Line &l) const
{
    double len = length();
    if (len < EPSILON)
        return (l.vp1-vp1).norm();
    double t1 = (vp2.y-vp1.y);
    double t2 = (vp2.x-vp1.x);
    double l1 = fabs(((vp1.x-l.vp1.x)*t1 - t2*(vp1.y-l.vp1.y)));
    double l2 = fabs(((vp1.x-l.vp2.x)*t1 - t2*(vp1.y-l.vp2.y)));
    return (l1+l2)/len;
}

// Returns true if v lies exactly on the line.
bool Line::intersects(const Vec2 &v) const
{
    return distance(v) < EPSILON;
}

// The shortest distance between this line and the point p.
// Note that the shortest distance is either the perpendicular of
// the line through point p, or the distance to one of the end points.
double Line::distance(const Vec2 &p) const
{
    //https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment#
    return (p-closestPoint(p)).norm();
}

// Returns the signed orthogonal distance between this line and the point p,
// even if the orthogonal of the line through point p is outside of the line.
// The sign of the orthogonalDistance is positive when the point is on the
// left of the line and negative when the point is on the right of the line
// with respect to the p1 to p2 direction.
double Line::orthogonalDistance(const Vec2 &p) const
{
    double l = length();
    if (l < EPSILON)
        return (p-vp1).norm();
    return (p-vp1).det(vp2-vp1) / l;
}

// Returns the projection overlap between this line and the line l.
// The projection overlap is the length of the segment where this
// line and the projection of l onto this line overlap. The overlap
// is zero if the lines touch only by their end points. The overlap
// is negative if the lines do not overlap. It is then the (negative)
// distance between their closest end points.
double Line::projectionOverlap(const Line &l) const
{
    Vec2 l1 = vp2-vp1;
    double ll = l1.length();
    double o1 = (l.vp1-vp1)*l1/ll;
    double o2 = (l.vp2-vp1)*l1/ll;
    return min(ll, max(o1,o2)) - max(0.0,min(o1,o2));
}

// Treats the lines as vectors from p1 to p2 and returns their dot product.
double Line::dot(const Line &l) const
{
    return (vp2-vp1).dot(l.vp2-l.vp1);
}

// Treats the lines as vectors from p1 to p2 and returns their det product.
double Line::det(const Line& l) const
{
    return (vp2-vp1).det(l.vp2-l.vp1);
}

// Returns the dot product (p2-p1)*(v-p1).
double Line::dot(const Vec2& v) const
{
    return (vp2-vp1).dot(v-vp1);
}

// Returns the det product (p2-p1)*(v-p1).
double Line::det(const Vec2& v) const
{
    return (vp2-vp1).det(v-vp1);
}

// Returns the closest point of this line to the given point p.
// Note that the closest point is either one of the end points, or
// the point where the perpendicular of the line through p
// intersects with the line.
Vec2 Line::closestPoint(const Vec2 &p, bool *tangential) const
{
    Vec2 l = (vp2-vp1);
    lineVector();
    if (l.isNull())
        return vp1;
    double t = max(0.0, min(1.0, ((p-vp1)*l)/(l*l)));
    Vec2 pp = vp1 + t*l;
    if (tangential != 0)
        *tangential = (t == 0 || t == 1.0);
    return pp;
}

// Translates the line by d.
void Line::translate(const Vec2 &d)
{
    translate(d.x, d.y);
}

// Returns a copy of this Line translated by (dx, dy).
Line Line::translated(double dx, double dy) const
{
    Line l = *this;
    l.translate(dx, dy);
    return l;
}

// Returns a copy of this Line translated by (d.x, d.y).
Line Line::translated(const Vec2& d) const
{
    Line l = *this;
    l.translate(d);
    return l;
}

// Translates the line by (dx,dy).
void Line::translate(double dx, double dy)
{
    vp1.x += dx;
    vp1.y += dy;
    vp2.x += dx;
    vp2.y += dy;
}

// Rotates the line around the origin, i.e., applies a rotation
// by the given angle to both end points.
void Line::rotate(double angle)
{
    if (fabs(angle) < EPSILON)
        return;

    double c = fcos(angle);
    double s = fsin(angle);
    rotate(s,c);
}

// Fast rotate for cases where the sin and cos of the angle are known.
// The line is rotated around the origin of the coordinate system.
void Line::rotate(double s, double c)
{
    if (fabs(s) < EPSILON)
        return;

    // Rotate p1.
    double x_ = vp1.x;
    double y_ = vp1.y;
    vp1.x = x_*c + y_*-s;
    vp1.y = x_*s + y_*c;

    // Rotate p2.
    x_ = vp2.x;
    y_ = vp2.y;
    vp2.x = x_*c + y_*-s;
    vp2.y = x_*s + y_*c;

    ang = NAN;
}

// Returns a line rotated by angle.
// The end points are rotated around the origin just like rotate() would do.
Line Line::rotated(double angle) const
{
    Line l = *this;
    l.rotate(angle);
    return l;
}

// Returns a line rotated by sin s and cos c.
// The end points are rotated around the origin just like rotate(s,c) would do.
Line Line::rotated(double s, double c) const
{
    Line l = *this;
    l.rotate(s,c);
    return l;
}

// Scales the line by multiplying its end points with s.
void Line::scale(double s)
{
    vp1.x *= s;
    vp1.y *= s;
    vp2.x *= s;
    vp2.y *= s;
    set(vp1.x, vp1.y, vp2.x, vp2.y);
}

// Returns a "sorted" line such that x1 < x2 and if x1 == x2 then y1 < y2;
Line Line::sorted() const
{
    Line line = *this;
    line.sort();
    return line;
}

// Returns a mutable reference to the x coordinate of the first endpoint of this line.
double& Line::x1()
{
    return vp1.x;
}

// Returns a mutable reference to the x coordinate of the second endpoint of this line.
double& Line::x2()
{
    return vp2.x;
}

// Returns a mutable reference to the y coordinate of the first endpoint of this line.
double& Line::y1()
{
    return vp1.y;
}

// Returns a mutable reference to the y coordinate of the second endpoint of this line.
double& Line::y2()
{
    return vp2.y;
}

// Returns a const reference to the x coordinate of the first endpoint of this line.
const double& Line::x1() const
{
    return vp1.x;
}

// Returns a const reference to the x coordinate of the second endpoint of this line.
const double& Line::x2() const
{
    return vp2.x;
}

// Returns a const reference to the y coordinate of the first endpoint of this line.
const double& Line::y1() const
{
    return vp1.y;
}

// Returns a const reference to the y coordinate of the second endpoint of this line.
const double& Line::y2() const
{
    return vp2.y;
}

// Returns a mutable reference to the first endpoint of this line.
Vec2& Line::p1()
{
    return vp1;
}

// Returns a mutable reference to the second endpoint of this line.
Vec2& Line::p2()
{
    return vp2;
}

// Returns a const reference to the first endpoint of this line.
const Vec2& Line::p1() const
{
    return vp1;
}

// Returns a const reference to the second endpoint of this line.
const Vec2& Line::p2() const
{
    return vp2;
}

// Sets the first endpoint of the line.
void Line::setP1(const Vec2 &p)
{
    vp1 = p;
    ang = NAN;
}

// Sets the second endpoint of the line.
void Line::setP2(const Vec2 &p)
{
    vp2 = p;
    ang = NAN;
}

// Flips the line so that p1 becomes p2 and p2 becomes p1.
void Line::flip()
{
    Vec2 temp = vp1;
    vp1 = vp2;
    vp2 = temp;
    ang = NAN;
}

// "Sorts" the line such that x1 < x2 and if x1 == x2 then y1 < y2;
void Line::sort()
{
    if (vp1.x > vp2.x || (vp1.x == vp2.x && vp1.y > vp2.y))
        this->flip();
}

// Returns the center point of the line.
Vec2 Line::center() const
{
    return Vec2(0.5*(vp1.x+vp2.x), 0.5*(vp1.y+vp2.y));
}

// Returns a, the slope of the line.
double Line::a() const
{
    if (isVertical())
        return INFINITY;
    return (vp2.y-vp1.y)/(vp2.x-vp1.x);
}

// Returns b, the y intercept of the line.
double Line::b() const
{
    if (isVertical())
        return vp1.y;
    return vp1.y-vp1.x*a();
}

// Returns the left border of the line.
double Line::left() const
{
    return min(vp1.x, vp2.x);
}

// Returns the right border of the line.
double Line::right() const
{
    return max(vp1.x, vp2.x);
}

// Returns the top border of the line.
double Line::top() const
{
    return max(vp1.y, vp2.y);
}

// Returns the bottom border of the line.
double Line::bottom() const
{
    return min(vp1.y, vp2.y);
}

// Returns the left vertex of the line.
Vec2 Line::leftVertex() const
{
    if (vp1.x <= vp2.x)
        return vp1;
    return vp2;
}

// Returns the right vertex of the line.
Vec2 Line::rightVertex() const
{
    if (vp1.x > vp2.x)
        return vp1;
    return vp2;
}

// Returns the top vertex of the line.
Vec2 Line::topVertex() const
{
    if (vp1.y > vp2.x)
        return Vec2(vp1.x, vp1.y);
    return Vec2(vp2.x, vp2.y);
}

// Returns the bottom vertex of the line.
Vec2 Line::bottomVertex() const
{
    if (vp1.y <= vp2.y)
        return vp1;
    return vp2;
}

// Determines if this and the given line segment l intersect.
// Line segments intersect if they cross or if an end point of one line touches the
// the other line. To help with visibility graph construction, identical lines and
// lines who touch only in their end points are not considered to be intersecting.
// This is because otherwise visibility graph edges would always collide with the
// polygon edges they connect to. If the end point of one line touching the other
// line anywhere else were not considered an intersection, a line could penetrate
// a polygon through one of its vertices or even cross the polygon through two
// vertices. The touching of an end point is epsilon strict, the same as the point
// intersection test in a polygon so that if a point collides with the polygon, so
// does every line that connects to the point.
bool Line::intersects(const Line &l, bool debug) const
{
    // Quick "bounding box" check.
    if (l.right() < left() || l.left() > right() || l.top() < bottom() || l.bottom() > top())
        return false;

    double dx = vp2.x-vp1.x;
    double dy = vp2.y-vp1.y;
    double p11 = ((l.vp1.x-vp1.x)*dy-(l.vp1.y-vp1.y)*dx);
    double p12 = ((l.vp2.x-vp1.x)*dy-(l.vp2.y-vp1.y)*dx);
    if (fabs(p11) < EPSILON) p11 = 0;
    if (fabs(p12) < EPSILON) p12 = 0;

    // Colinear lines are not considered to be intersecting.
    if (fabs(p11) < EPSILON && fabs(p12) < EPSILON) // Colinear?
    {
        return false;

        // Enable the code below for an overlap check.
//        if (isVertical())
//            return !(max(y1,y2) < min(l.y1,l.y2)+EPSILON || min(y1,y2) > max(l.y1,l.y2)-EPSILON);
//        qDebug() << max(x1,x2) << min(l.x1,l.x2) << (max(x1,x2) < min(l.x1,l.x2)+EPSILON);
//        qDebug() << min(x1,x2) << max(l.x1,l.x2)-EPSILON << (min(x1,x2) > max(l.x1,l.x2)-EPSILON);
//        return !(max(x1,x2) < min(l.x1,l.x2)+EPSILON || min(x1,x2) > max(l.x1,l.x2)-EPSILON);
    }

    // Our special case: lines touching only in the end points do not intersect.
    if ((fabs(vp1.x-l.vp1.x) < EPSILON && fabs(vp1.y-l.vp1.y) < EPSILON)
          || (fabs(vp1.x-l.vp2.x) < EPSILON && fabs(vp1.y-l.vp2.y) < EPSILON)
          || fabs(vp2.x-l.vp1.x) < EPSILON && fabs(vp2.y-l.vp1.y) < EPSILON
          || fabs(vp2.x-l.vp2.x) < EPSILON && fabs(vp2.y-l.vp2.y) < EPSILON)
    {
//        if (debug)
//            qDebug() << "            Touching" << p11 << p12;
        return false;
    }

    dx = l.vp2.x-l.vp1.x;
    dy = l.vp2.y-l.vp1.y;
    double p21 = ((vp1.x-l.vp1.x)*dy-(vp1.y-l.vp1.y)*dx);
    double p22 = ((vp2.x-l.vp1.x)*dy-(vp2.y-l.vp1.y)*dx);
    if (fabs(p21) < EPSILON) p21 = 0;
    if (fabs(p22) < EPSILON) p22 = 0;

//    if (debug)
//        qDebug() << "            ps:" << p11 << p12 << "and" << p21 << p22 << "should have opposing signs for intersection." << p11*p12 << p21*p22;
    return (p11*p12 <= 0 && p21*p22 <= 0);
}

// Computes the intersection point between this line and Line l.
// If the lines do not intersect, a zero Vec2() is returned.
Vec2 Line::intersection(const Line &l) const
{
    // Quick "bounding box" check.
    if (l.right() < left() || l.left() > right() || l.top() < bottom() || l.bottom() > top())
        return Vec2();

    Vec2 v;
    double x12 = vp1.x - vp2.x;
    double x34 = l.vp1.x - l.vp2.x;
    double y12 = vp1.y - vp2.y;
    double y34 = l.vp1.y - l.vp2.y;
    double c = x12 * y34 - y12 * x34;

    // Skip colinear lines.
    if (fabs(c) < EPSILON)
        return v;

    double a = vp1.x * vp2.y - vp1.y * vp2.x;
    double b = l.vp1.x * l.vp2.y - l.vp1.y * l.vp2.x;
    double x = (a * x34 - b * x12) / c;
    double y = (a * y34 - b * y12) / c;

    if ( (isVertical() || (x >= left() && x <= right()))
          && (l.isVertical() || (x >= l.left() && x <= l.right()))
          && (isHorizontal() || (y >= bottom() && y <= top()))
          && (l.isHorizontal() || (y >= l.bottom() && y <= l.top()))
       )
    {
        v.x = x;
        v.y = y;
    }

    return v;
}

// Computes the intersection point between this line and Line l, even if
// the intersection point is not within the boundaries of the line segments.
// If the lines are parallel, a zero Vec2() is returned.
Vec2 Line::intersectionInf(const Line &l) const
{
    Vec2 v;

    double x12 = vp1.x - vp2.x;
    double x34 = l.vp1.x - l.vp2.x;
    double y12 = vp1.y - vp2.y;
    double y34 = l.vp1.y - l.vp2.y;
    double c = x12 * y34 - y12 * x34;

    // Colinear case.
    if (fabs(c) < EPSILON)
        return v;

    double a = vp1.x * vp2.y - vp1.y * vp2.x;
    double b = l.vp1.x * l.vp2.y - l.vp1.y * l.vp2.x;
    v.x = (a * x34 - b * x12) / c;
    v.y = (a * y34 - b * y12) / c;

    return v;
}

// Finds the intersection of the ray given by (direction, origin) with this line segment.
// From https://rootllama.wordpress.com/2014/06/20/ray-line-segment-intersection-test-in-2d/
// and https://stackoverflow.com/a/32146853
Vec2 Line::rayIntersection(const Vec2 &direction, const Vec2 &origin) const
{
    static const Vec2 NANVEC(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    const Vec2& v1 = origin - p1();
    const Vec2& v2 = p2() - p1();
    const Vec2 v3(-direction.y, direction.x);

    double dotProduct = v2*v3;

    // The ray and the line segment are parallel and do not intersect.
    if (fabs(dotProduct) < EPSILON)
        return NANVEC;

    double t1 = -v2.det(v1) / dotProduct;
    double t2 = (v1*v3) / dotProduct;
    if (t1 >= 0.0 && (t2 >= 0.0 && t2 <= 1.0))
        return p1() + t2 * (p2() - p1());

    return NANVEC;
}

// Decides if v1 and v2 are on the same side of this line or not.
bool Line::sameSide(const Vec2 &v1, const Vec2 &v2) const
{
    double dx = vp2.x-vp1.x;
    double dy = vp2.y-vp1.y;
    return (((v1.x-vp1.x)*dy-(v1.y-vp1.y)*dx) * ((v2.x-vp1.x)*dy-(v2.y-vp1.y)*dx) > 0);
}

// Decides if the vertices of line l are on the same side of this line or not.
bool Line::sameSide(const Line &l) const
{
    double dx = vp2.x-vp1.x;
    double dy = vp2.y-vp1.y;
    return (((l.vp1.x-vp1.x)*dy-(l.vp1.y-vp1.y)*dx) * ((l.vp2.x-vp1.x)*dy-(l.vp2.y-vp1.y)*dx) > 0);
}

// Returns true if the point p is left of the line with respect to the p1 to p2 direction.
// It returns false if the point is right on the line.
bool Line::isLeftOf(const Vec2 &p) const
{
    return ((p.x-vp1.x)*(vp2.y-vp1.y)-(p.y-vp1.y)*(vp2.x-vp1.x) < 0);
}

// Returns true if the point p is right of the line with respect to the p1 to p2 direction.
// It returns false if the point is right on the line.
bool Line::isRightOf(const Vec2 &p) const
{
    return ((p.x-vp1.x)*(vp2.y-vp1.y)-(p.y-vp1.y)*(vp2.x-vp1.x) > 0);
}

// Returns true when the line is vertical.
// If the end points are identical, the line is considered to be vertical.
bool Line::isVertical() const
{
    return (fabs(vp2.x-vp1.x) < EPSILON);
}

// Returns true when the line is horizontal.
// If the end points are identical, the line is considered to be horizontal.
bool Line::isHorizontal() const
{
    return (fabs(vp2.y-vp1.y) < EPSILON);
}

// Draws the line with a QPainter.
void Line::draw(QPainter *painter, const QPen& pen, double opacity) const
{
    if (p1() == p2())
        return;
    painter->save();
    painter->setPen(pen);
    painter->setOpacity(opacity);
    QLineF l(vp1.x,vp1.y,vp2.x,vp2.y);
    painter->drawLine(l);
    painter->restore();
}

// Draws a label for the line on QPainter.
void Line::drawLabel(QPainter *painter, const QPen &pen, double opacity, double rotation) const
{
    painter->save();
    QFont font;
    font.setFamily("Arial");
    font.setPointSize(1);
    painter->setFont(font);
    painter->setPen(pen);
    painter->setOpacity(opacity);
    Vec2 c = center();
    painter->translate(c.x + 0.01, c.y + 0.02);
    painter->rotate(RAD_TO_DEG*rotation);
    painter->scale(0.05, -0.05);
    painter->drawText(QPointF(), QString::number(id));
    painter->restore();
}

// Draws the line in an OpenGL context.
// Set the glColor() and the glLineWidth() before calling this function
// to control how the line is drawn.
void Line::draw() const
{
    glBegin(GL_LINES);
    glVertex2d(vp1.x, vp1.y);
    glVertex2d(vp2.x, vp2.y);
    glEnd();
}

// Writes the line into a data stream.
void Line::streamOut(QDataStream &out) const
{
    out << id;
    out << p1();
    out << p2();
    out << type;
    out << ang;
}

// Reads the line from a data stream.
void Line::streamIn(QDataStream &in)
{
    in >> id;
    in >> p1();
    in >> p2();
    in >> type;
    in >> ang;
}

QDataStream& operator<<(QDataStream& out, const Line &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, Line &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const Line &o)
{
    dbg.setAutoInsertSpaces(false);
    dbg << o.id << (o.isBlockingLine()?"b":"s") << " [" << o.x1() << ", " << o.y1() << "] to [" << o.x2() << ", " << o.y2() <<"] ";
    dbg.setAutoInsertSpaces(true);
    return dbg;
}

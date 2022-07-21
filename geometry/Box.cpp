#include "Box.h"
#include "blackboard/Config.h"
#include <GL/glu.h>

// The Box is an axis aligned rectangle typically used as a bounding box
// to accelerate collision check routines. Its (x,y) position defines the
// world coordinates of a reference point of the box while t,l,b,r define
// the distance of the top, left, bottom, and right border with respect to
// the reference point. This representation makes it easy to compute the
// Minkowski sum of two boxes, which is needed to compute the bounding box
// of the swept volume of a moving polygon. Most importantly, Box implements
// intersect() methods for a number of geometric primitives for extra fast
// but approximate collision detection.

Box::Box()
{
    x = 0;
    y = 0;
    t = 0;
    l = 0;
    b = 0;
    r = 0;
}

// Sets the bounding box by providing the world coordinates of the top, left,
// bottom, and right borders.
Box::Box(double t, double l, double b, double r)
{
    // The reference point is set to be the bottom left corner.
    this->x = l;
    this->y = b;
    this->t = t-y;
    this->l = 0;
    this->b = 0;
    this->r = r-x;
}

// Sets the bounding box by providing the world coordinates of the reference
// point and the distances to the top, left, bottom, right borders relative
// to the reference point.
Box::Box(double x, double y, double t, double l, double b, double r)
{
    this->x = x;
    this->y = y;
    this->t = t;
    this->l = l;
    this->b = b;
    this->r = r;
}

// Sets the bounding box by providing the world coordinates of the top, left,
// bottom, and right borders.
void Box::set(double t, double l, double b, double r)
{
    // The reference point is set to be the bottom left corner.
    this->x = l;
    this->y = b;
    this->t = t-y;
    this->l = 0;
    this->b = 0;
    this->r = r-x;
}

// Sets the bounding box by providing the world coordinates of the reference
// point and the distances to the top, left, bottom, right borders relative
// to the reference point.
void Box::set(double x, double y, double t, double l, double b, double r)
{
    this->x = x;
    this->y = y;
    this->t = t;
    this->l = l;
    this->b = b;
    this->r = r;
}

// Returns the height of the box.
double Box::height() const
{
    return t-b;
}

// Returns the width of the box.
double Box::width() const
{
    return r-l;
}

// Returns the world x coordinate of the left margin.
double Box::left() const
{
    return x+l;
}

// Returns the world x coordinate of the right margin.
double Box::right() const
{
    return x+r;
}

// Returns the world y coordinate of the top margin.
double Box::top() const
{
    return y+t;
}

// Returns the world y coordinate of the bottom margin.
double Box::bottom() const
{
    return y+b;
}

// Returns the world coordinates of the top left corner.
Vec2 Box::topLeft() const
{
    return Vec2(left(), top());
}

// Returns the world coordinates of the top right corner.
Vec2 Box::topRight() const
{
    return Vec2(right(), top());
}

// Returns the world coordinates of the bottom left corner.
Vec2 Box::bottomLeft() const
{
    return Vec2(left(), bottom());
}

// Returns the world coordinates of the bottom right corner.
Vec2 Box::bottomRight() const
{
    return Vec2(right(), bottom());
}

// Sets the left border of the box in world.
void Box::setLeft(double ll)
{
    l = ll-x;
}

// Sets the right border of the box in world.
void Box::setRight(double rr)
{
    r = rr-x;
}

// Sets the top border of the box in world.
void Box::setTop(double tt)
{
    t = tt-y;
}

// Sets the bottom border of the box in world.
void Box::setBottom(double bb)
{
    b = bb-y;
}

// Sets the world (x,y) position of the box.
void Box::setPos(double x, double y)
{
    this->x = x;
    this->y = y;
}

// Sets the world (x,y) position of the box.
void Box::setPos(const Vec2& p)
{
    this->x = p.x;
    this->y = p.y;
}

// Returns the world (x,y) position of the box.
Vec2 Box::pos() const
{
    return Vec2(x, y);
}

// Translates the box by (dx,dy).
void Box::translate(double dx, double dy)
{
    setPos(this->x+dx, this->y+dy);
}

// Translates the box by (dx,dy).
void Box::translate(const Vec2 &d)
{
    setPos(this->x+d.x, this->y+d.y);
}

// Expands the box by s meaning the top, left, bottom, and right borders are all moved outwards by s.
// You can pass a negative s in order to shrink the box.
void Box::grow(double s)
{
    setLeft(left()-s);
    setRight(right()+s);
    setTop(top()+s);
    setBottom(bottom()-s);
}

// Returns true if the box has no extent.
bool Box::isEmpty() const
{
    return (t > -EPSILON && t < EPSILON
            && l > -EPSILON && l < EPSILON
            && b > -EPSILON && b < EPSILON
            && r > -EPSILON && r < EPSILON);
}

// Computes the Minkowski sum of this box and the other Box o.
void Box::operator+=(const Box &o)
{
    t += o.t;
    l += o.l;
    b += o.b;
    r += o.r;
}

// Computes the Minkowski sum of this box and the other Box o.
Box Box::operator+(const Box &o) const
{
    Box box = *this;
    box += o;
    return box;
}

// Returns true if the point p is on the edge of the bounding box.
bool Box::isOnBounds(const Vec2 &p) const
{
    double epsi = 0.001;
    if (p.y > top()+epsi || p.x > right()+epsi || p.y < bottom()-epsi || p.x < left()-epsi)
        return false;
    if (p.y > top()-epsi || p.x > right()-epsi || p.y < bottom()+epsi || p.x < left()+epsi)
        return true;
    return false;
}

// Returns true if the box contains the point p.
// The check is very strict and returns true only if the point is truly
// inside the box and not on the border. The strictness is needed for
// the bounding box test in the visibility graph.
bool Box::intersects(const Vec2 &p) const
{
    return (p.y <= top() && p.y >= bottom() && p.x >= left() && p.x <= right());
}

// Returns true if the box intersects the circle with the center point p and radius.
bool Box::intersects(const Vec2 &p, double radius) const
{
    return (p.y <= top()+radius && p.y >= bottom()-radius && p.x >= left()-radius && p.x <= right()+radius);
}

// Returns true if the box o intersects with this one.
bool Box::intersects(const Box &o) const
{
    return !(o.bottom() > top() || o.right() < left() || o.top() < bottom() || o.left() > right());
}

// Returns true if the *bounding box* of the line l intersects this box.
bool Box::intersects(const Line &l) const
{
    return !(l.left() > right() || l.right() < left()
             || l.bottom() > top() || l.top() < bottom());
}

// Computes the intersection point between the boundary of this box and the line l.
// If the line does not intersect the boundary, a zero Vec2() is returned.
// If the line intersects the boundary twice, the intersection point that is found
// first according to the top, right, bottom, left order is returned.
Vec2 Box::intersection(const Line &l) const
{
    //qDebug() << "Box::intersection(const Line &l)" << l;
    //qDebug() << "this:" << *this;

    Vec2 ip;
    Line borderLine;

    // top
    borderLine.set(topLeft(), topRight());
    ip = borderLine.intersection(l);
    if (!ip.isNull())
        return ip;

    // right
    borderLine.set(bottomRight(), topRight());
    ip = borderLine.intersection(l);
    if (!ip.isNull())
        return ip;

    // bottom
    borderLine.set(bottomLeft(), bottomRight());
    ip = borderLine.intersection(l);
    if (!ip.isNull())
        return ip;

    // left
    borderLine.set(bottomLeft(), topLeft());
    ip = borderLine.intersection(l);

    return ip;
}

// Draws the box on a QPainter.
void Box::draw(QPainter *painter) const
{
    QPainterPath pp;
    pp.moveTo(left(), top());
    pp.lineTo(right(), top());
    pp.lineTo(right(), bottom());
    pp.lineTo(left(), bottom());
    pp.lineTo(left(), top());
    painter->drawPath(pp);
}

// Draws the box in an OpenGL environment.
void Box::draw() const
{
    glBegin(GL_POLYGON);
    glVertex2dv(bottomLeft());
    glVertex2dv(bottomRight());
    glVertex2dv(topRight());
    glVertex2dv(topLeft());
    glEnd();

    glColor3f(0, 0, 0);
    glEnable(GL_LINE_SMOOTH);
    glBegin(GL_LINE_LOOP);
    glVertex2dv(bottomLeft());
    glVertex2dv(bottomRight());
    glVertex2dv(topRight());
    glVertex2dv(topLeft());
    glEnd();
}

void Box::streamOut(QDataStream& out) const
{
    out << x;
    out << y;
    out << t;
    out << l;
    out << b;
    out << r;
}

void Box::streamIn(QDataStream& in)
{
    in >> x;
    in >> y;
    in >> t;
    in >> l;
    in >> b;
    in >> r;
}

QDebug operator<<(QDebug dbg, const Box &o)
{
    dbg << "tl:" << o.topLeft() << "br:" << o.bottomRight() << "pos:" << o.pos() << "width:" << o.width() << "height:" << o.height();
    return dbg;
}

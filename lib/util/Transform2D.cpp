#include "Transform2D.h"

// The Transform2D represents the position and the orientation
// on a plane. The position is the (x,y) coordinates on
// the Cartesian plane, the orientation (z) is the direction
// the robot is facing. The Transform2D class provides arithmetic
// functions for transforming a point or a Transform2D into the
// local frame of a Transform2D (e.g. Vec2 - Transform2D), and for
// transforming from local coordinates to world coordinates
// (e.g. Vec2 + Transform2D).

Transform2D::Transform2D()
{
    x = 0;
    y = 0;
    z = 0;
}

Transform2D::Transform2D(const Vec2 &o, double zo)
{
    x = o.x;
    y = o.y;
    z = zo;
}

Transform2D::Transform2D(double xo, double yo, double zo)
{
    x = xo;
    y = yo;
    z = zo;
}

// Sets the (x,y) position of the Transform2D.
void Transform2D::setPos(double x, double y)
{
    this->x = x;
    this->y = y;
}

// Sets the position of the Transform2D to p and the heading to the angle of to.
void Transform2D::set(const Vec2 &p, const Vec2 &to)
{
    setPos(p);
    setHeading(to.angle());
}

// Sets the (x,y) position of the Transform2D.
void Transform2D::setPos(const Vec2& p)
{
    this->x = p.x;
    this->y = p.y;
}

// Returns the (x,y) position of the Transform2D.
Vec2 Transform2D::pos() const
{
    return Vec2(x, y);
}

// Translates the Transform2D by (dx,dy).
void Transform2D::translate(double dx, double dy)
{
    setPos(this->x+dx, this->y+dy);
}

// Translates the Transform2D by d.
void Transform2D::translate(const Vec2 &d)
{
    setPos(this->x+d.x, this->y+d.y);
}

// Rotates the heading of the Transform2D counter clockwise by the angle "a" given in radians.
void Transform2D::rotate(double a)
{
    setHeading(z+a);
}

// Returns the heading theta of the Transform2D.
double Transform2D::heading() const
{
    return z;
}

// Sets the heading theta of the Transform2D.
void Transform2D::setHeading(double a)
{
    //z = a;
    z = ffpicut(a);
}

void Transform2D::scale(double sx, double sy)
{
    this->x *= sx;
    this->y *= sy;
}

void Transform2D::scale(const Vec2 &s)
{
    return scale(s.x, s.y);
}

// Inverts this Transform2D.
Transform2D Transform2D::operator-() const
{
    Transform2D c = *this;
    c.x = -c.x;
    c.y = -c.y;
    c.z = -c.z;
    return c;
}

// Maps this Transform2D into the coordinate frame of the given Transform2D o.
Transform2D Transform2D::operator-(const Transform2D &o) const
{
    Transform2D c = *this;
    c -= o;
    return c;
}

// Maps this pose from the coordinate frame of the given Transform2D o to the world coordinates.
Transform2D Transform2D::operator+(const Transform2D &o) const
{
    Transform2D c = *this;
    c += o;
    return c;
}

// Maps this Transform2D into the coordinate frame of the given Transform2D o.
void Transform2D::operator-=(const Transform2D &o)
{
    x -= o.x;
    y -= o.y;
    z -= o.z;
    z = ffpicut(z);

    if (o.z < EPSILON && o.z > -EPSILON)
        return;

    double c = fcos(-o.z);
    double s = fsin(-o.z);
    double x_ = x;
    x = x_*c + y*-s;
    y = x_*s + y*c;

    return;
}

// Maps this Transform2D from the coordinate frame of the given Transform2D o to world coordinates.
void Transform2D::operator+=(const Transform2D &o)
{
    if (o.z > EPSILON || o.z < -EPSILON)
    {
        double c = fcos(o.z);
        double s = fsin(o.z);
        double x_ = x;
        x = x_*c + y*-s;
        y = x_*s + y*c;
    }

    x += o.x;
    y += o.y;
    z += o.z;
    z = ffpicut(z);
}

Transform3D Transform2D::getMatrix() const
{
    Transform3D mat;
    mat.setFromParams(x, y, 0, 0, 0, z);
    return mat;
}

// Maps the Vec2 v into the coordinate frame of the Transform2D p.
Vec2 operator-(const Vec2& v, const Transform2D& p)
{
    Vec2 c = v;
    c -= p;
    return c;
}

// Maps the Vec2 v from the coordinate frame of Transform2D p to world coordinates.
Vec2 operator+(const Vec2& v, const Transform2D& p)
{
    Vec2 c = v;
    c += p;
    return c;
}

// Maps the Vec3 v into the coordinate frame of the Transform2D p.
Vec3 operator-(const Vec3& v, const Transform2D& p)
{
    Vec3 c = v;
    c -= p;
    return c;
}

// Maps the Vec2 v from the coordinate frame of Transform2D p to world coordinates.
Vec3 operator+(const Vec3& v, const Transform2D& p)
{
    Vec3 c = v;
    c += p;
    return c;
}

// Maps the Vec2 v into the coordinate frame of the Transform2D p.
void operator-=(Vec2& v, const Transform2D& p)
{
    v.x -= p.x;
    v.y -= p.y;
    v.frotate(-p.z);
}

// Maps the Vec2 v from the coordinate frame of Transform2D p to world coordinates.
void operator+=(Vec2& v, const Transform2D& p)
{
    v.frotate(p.z);
    v.x += p.x;
    v.y += p.y;
}

// Maps the Vec3 v into the coordinate frame of the Transform2D p.
void operator-=(Vec3& v, const Transform2D& p)
{
    v.x -= p.x;
    v.y -= p.y;
    v.frotate(-p.z);
}

// Maps the Vec3 v from the coordinate frame of Transform2D p to world coordinates.
void operator+=(Vec3& v, const Transform2D& p)
{
    v.frotate(p.z);
    v.x += p.x;
    v.y += p.y;
}

Vector<Vec3> operator+(const Vector<Vec3> &v, const Transform2D &p)
{
    static Vector<Vec3> tmp;

    tmp.resize(v.size());
    for (int i = 0; i < v.size(); ++i) {
        tmp[i] = v[i] + p;
    }

    return tmp;
}

Vector<Vec3> operator-(const Vector<Vec3> &v, const Transform2D &p)
{
    static Vector<Vec3> tmp;

    tmp.resize(v.size());
    for (int i = 0; i < v.size(); ++i) {
        tmp[i] = v[i] - p;
    }

    return tmp;
}

void operator+=(Vector<Vec3> &v, const Transform2D &p)
{
    for (int i = 0; i < v.size(); ++i) {
        v[i] += p;
    }
}

void operator-=(Vector<Vec3> &v, const Transform2D &p)
{
    for (int i = 0; i < v.size(); ++i) {
        v[i] -= p;
    }
}

void Transform2D::streamOut(QDataStream& out) const
{
    out << x;
    out << y;
    out << z;
}

void Transform2D::streamIn(QDataStream &in)
{
    in >> x;
    in >> y;
    in >> z;
}

QDataStream& operator<<(QDataStream& out, const Transform2D &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, Transform2D &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const Transform2D &o)
{
    dbg << o.pos() << o.heading();
    return dbg;
}

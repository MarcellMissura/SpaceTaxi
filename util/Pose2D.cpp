#include "Pose2D.h"

// The Pose represents the position and the orientation
// on a plane. The position is the (x,y) coordinates on
// the Cartesian plane, the orientation (z) is the direction
// the robot is facing. The Pose class provides arithmetic
// functions for transforming a point or a pose into the
// local frame of a Pose (e.g. Vec2 - Pose), and for
// transforming from local coordinates to world coordinates
// (e.g. Vec2 + Pose).

Pose2D::Pose2D()
{
    x = 0;
    y = 0;
    z = 0;
}

Pose2D::Pose2D(const Vec2 &o, double zo)
{
    x = o.x;
    y = o.y;
    z = zo;
}

Pose2D::Pose2D(double xo, double yo, double zo)
{
    x = xo;
    y = yo;
    z = zo;
}

// Sets the (x,y) position of the Pose.
void Pose2D::setPos(double x, double y)
{
    this->x = x;
    this->y = y;
}

// Sets the position of the pose to p and the heading to the angle of to.
void Pose2D::set(const Vec2 &p, const Vec2 &to)
{
    setPos(p);
    setHeading(to.angle());
}

// Sets the (x,y) position of the Pose.
void Pose2D::setPos(const Vec2& p)
{
    this->x = p.x;
    this->y = p.y;
}

// Returns the (x,y) position of the Pose.
Vec2 Pose2D::pos() const
{
    return Vec2(x, y);
}

// Translates the Pose by (dx,dy).
void Pose2D::translate(double dx, double dy)
{
    setPos(this->x+dx, this->y+dy);
}

// Translates the Pose by d.
void Pose2D::translate(const Vec2 &d)
{
    setPos(this->x+d.x, this->y+d.y);
}

// Rotates the heading of the Pose counter clockwise by the angle "a" given in radians.
void Pose2D::rotate(double a)
{
    setHeading(z+a);
}

// Returns the heading theta of the Pose.
double Pose2D::heading() const
{
    return z;
}

// Sets the heading theta of the Pose.
void Pose2D::setHeading(double a)
{
    //z = a;
    z = ffpicut(a);
}

void Pose2D::scale(double sx, double sy)
{
    this->x *= sx;
    this->y *= sy;
}

void Pose2D::scale(const Vec2 &s)
{
    return scale(s.x, s.y);
}

// Inverts this pose.
Pose2D Pose2D::operator-() const
{
    Pose2D c = *this;
    c.x = -c.x;
    c.y = -c.y;
    c.z = -c.z;
    return c;
}

// Maps this pose into the coordinate frame of the given Pose o.
Pose2D Pose2D::operator-(const Pose2D &o) const
{
    Pose2D c = *this;
    c -= o;
    return c;
}

// Maps this pose from the coordinate frame of the given Pose o to the world coordinates.
Pose2D Pose2D::operator+(const Pose2D &o) const
{
    Pose2D c = *this;
    c += o;
    return c;
}

// Maps this pose into the coordinate frame of the given Pose o.
void Pose2D::operator-=(const Pose2D &o)
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

// Maps this pose from the coordinate frame of the given Pose o to world coordinates.
void Pose2D::operator+=(const Pose2D &o)
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

Transform3D Pose2D::getMatrix() const
{
    Transform3D mat;
    mat.setFromParams(x, y, 0, 0, 0, z);
    return mat;
}

// Maps the Vec2 v into the coordinate frame of the Pose p.
Vec2 operator-(const Vec2& v, const Pose2D& p)
{
    Vec2 c = v;
    c -= p;
    return c;
}

// Maps the Vec2 v from the coordinate frame of Pose p to world coordinates.
Vec2 operator+(const Vec2& v, const Pose2D& p)
{
    Vec2 c = v;
    c += p;
    return c;
}

// Maps the Vec3 v into the coordinate frame of the Pose p.
Vec3 operator-(const Vec3& v, const Pose2D& p)
{
    Vec3 c = v;
    c -= p;
    return c;
}

// Maps the Vec2 v from the coordinate frame of Pose p to world coordinates.
Vec3 operator+(const Vec3& v, const Pose2D& p)
{
    Vec3 c = v;
    c += p;
    return c;
}

// Maps the Vec2 v into the coordinate frame of the Pose p.
void operator-=(Vec2& v, const Pose2D& p)
{
    v.x -= p.x;
    v.y -= p.y;
    v.frotate(-p.z);
}

// Maps the Vec2 v from the coordinate frame of Pose p to world coordinates.
void operator+=(Vec2& v, const Pose2D& p)
{
    v.frotate(p.z);
    v.x += p.x;
    v.y += p.y;
}

// Maps the Vec3 v into the coordinate frame of the Pose p.
void operator-=(Vec3& v, const Pose2D& p)
{
    v.x -= p.x;
    v.y -= p.y;
    v.frotate(-p.z);
}

// Maps the Vec3 v from the coordinate frame of Pose p to world coordinates.
void operator+=(Vec3& v, const Pose2D& p)
{
    v.frotate(p.z);
    v.x += p.x;
    v.y += p.y;
}

Vector<Vec3> operator+(const Vector<Vec3> &v, const Pose2D &p)
{
    static Vector<Vec3> tmp;

    tmp.resize(v.size());
    for (int i = 0; i < v.size(); ++i) {
        tmp[i] = v[i] + p;
    }

    return tmp;
}

Vector<Vec3> operator-(const Vector<Vec3> &v, const Pose2D &p)
{
    static Vector<Vec3> tmp;

    tmp.resize(v.size());
    for (int i = 0; i < v.size(); ++i) {
        tmp[i] = v[i] - p;
    }

    return tmp;
}

void operator+=(Vector<Vec3> &v, const Pose2D &p)
{
    for (int i = 0; i < v.size(); ++i) {
        v[i] += p;
    }
}

void operator-=(Vector<Vec3> &v, const Pose2D &p)
{
    for (int i = 0; i < v.size(); ++i) {
        v[i] -= p;
    }
}

void Pose2D::streamOut(QDataStream& out) const
{
    out << x;
    out << y;
    out << z;
}

void Pose2D::streamIn(QDataStream &in)
{
    in >> x;
    in >> y;
    in >> z;
}

QDataStream& operator<<(QDataStream& out, const Pose2D &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, Pose2D &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const Pose2D &o)
{
    dbg << o.pos() << o.heading();
    return dbg;
}

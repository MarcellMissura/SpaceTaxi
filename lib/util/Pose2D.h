#ifndef POSE2D_H
#define POSE2D_H

#include "Vec3.h"
#include "Vec2.h"
#include "Transform3D.h"
#include "Vector.h"
#include "lib/geometry/Line.h"
#include <QTransform>

class Pose2D
{
public:

    double x,y,z;

    Pose2D();
    Pose2D(const Vec2 &o, double zo=0);
    Pose2D(double xo, double yo, double zo=0);

    operator QTransform() const {return getQTransform();}

public:

    bool isNull() const;
    void setNull();
    void setZero();
    bool isNan() const;
    void setNan();

    Vec2 pos() const;
    void setPos(const Vec2& p);
    void setPos(double x, double y);
    void set(const Vec2& p, const Vec2& to);
    void set(const Vec2& p, double rot);
    void set(double x, double y, double theta);
    double orientation() const;
    void setOrientation(double a);
    double heading() const;
    void setHeading(double a);
    void translate(double dx, double dy);
    void translate(const Vec2& d);
    void rotate(double a);
    void turn(double a);
    Pose2D turned(double a) const;
    double norm() const;

    double dist(const Pose2D& p) const;
    double distxy(const Pose2D& p) const;

    bool operator==(const Pose2D &o) const;
    bool operator!=(const Pose2D &o) const;

    Pose2D operator-(const Pose2D &o) const;
    Pose2D operator+(const Pose2D &o) const;
    void operator-=(const Pose2D &o);
    void operator+=(const Pose2D &o);
    Pose2D operator-() const;

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream &in);

    Transform3D getMatrix() const;
    QTransform getQTransform() const;

    operator const double*() const {return (double*)this;}

    Pose2D operator*(double s) const
    {
        Pose2D c = *this;
        c *= s;
        return c;
    }

    Pose2D operator/(double s) const
    {
        Pose2D c = *this;
        c /= s;
        return c;
    }

    void operator*=(double s)
    {
        x *= s;
        y *= s;
        z *= s;
    }

    void operator/=(double s)
    {
        x /= s;
        y /= s;
        z /= s;
    }

    // Returns the componentwise subtraction Pose2D(x-p.x, y-p.y, z-p.z).
    // This is only rarely used while the - operator is already used for mapping
    // from world to local coordinates.
    Pose2D diff(const Pose2D& p) const
    {
        Pose2D d;
        d.x = x - p.x;
        d.y = y - p.y;
        d.z = z - p.z;
        return d;
    }

    // Componentwise subtraction x-p.x, y-p.y, z-p.z.
    // This is only rarely used while the -= operator is already used for mapping
    // from world to local coordinates.
    void diffed(const Pose2D& p)
    {
        x -= p.x;
        y -= p.y;
        z -= p.z;
        return;
    }

    // Returns the componentwise addition Pose2D(x+p.x, y+p.y, z+p.z).
    // This is only rarely used while the + operator is already used for mapping
    // from local to world coordinates.
    Pose2D sum(const Pose2D& p) const
    {
        Pose2D d;
        d.x = x + p.x;
        d.y = y + p.y;
        d.z = z + p.z;
        return d;
    }

    // Componentwise addition x+p.x, y+p.y, z+p.z.
    // This is only rarely used while the += operator is already used for mapping
    // from local to world coordinates.
    void summed(const Pose2D& p)
    {
        x += p.x;
        y += p.y;
        z += p.z;
        return;
    }
};

extern Vec2 operator+(const Vec2& v, const Pose2D& p);
extern Vec2 operator-(const Vec2& v, const Pose2D& p);
extern Vec3 operator+(const Vec3& v, const Pose2D& p);
extern Vec3 operator-(const Vec3& v, const Pose2D& p);
extern Vector<Vec2> operator+(const Vector<Vec2>& v, const Pose2D& p);
extern Vector<Vec2> operator-(const Vector<Vec2>& v, const Pose2D& p);
extern Vector<Vec3> operator+(const Vector<Vec3>& v, const Pose2D& p);
extern Vector<Vec3> operator-(const Vector<Vec3>& v, const Pose2D& p);
extern LinkedList<Vec2> operator+(const LinkedList<Vec2>& v, const Pose2D& p);
extern LinkedList<Vec2> operator-(const LinkedList<Vec2>& v, const Pose2D& p);
extern LinkedList<Vec3> operator+(const LinkedList<Vec3>& v, const Pose2D& p);
extern LinkedList<Vec3> operator-(const LinkedList<Vec3>& v, const Pose2D& p);

extern void operator+=(Vec2& v, const Pose2D& p);
extern void operator-=(Vec2& v, const Pose2D& p);
extern void operator+=(Vec3& v, const Pose2D& p);
extern void operator-=(Vec3& v, const Pose2D& p);
extern void operator+=(Vector<Vec2>& v, const Pose2D& p);
extern void operator-=(Vector<Vec2>& v, const Pose2D& p);
extern void operator+=(Vector<Vec3>& v, const Pose2D& p);
extern void operator-=(Vector<Vec3>& v, const Pose2D& p);
extern void operator+=(LinkedList<Vec2>& v, const Pose2D& p);
extern void operator-=(LinkedList<Vec2>& v, const Pose2D& p);
extern void operator+=(LinkedList<Vec3>& v, const Pose2D& p);
extern void operator-=(LinkedList<Vec3>& v, const Pose2D& p);

extern Line operator+(const Line& l, const Pose2D& p);
extern Line operator-(const Line& l, const Pose2D& p);
extern void operator+=(Line& l, const Pose2D& p);
extern void operator-=(Line& l, const Pose2D& p);
extern Vector<Line> operator+(const Vector<Line>& v, const Pose2D& p);
extern Vector<Line> operator-(const Vector<Line>& v, const Pose2D& p);
extern void operator+=(Vector<Line>& v, const Pose2D& p);
extern void operator-=(Vector<Line>& v, const Pose2D& p);
extern LinkedList<Line> operator+(const LinkedList<Line>& v, const Pose2D& p);
extern LinkedList<Line> operator-(const LinkedList<Line>& v, const Pose2D& p);
extern void operator+=(LinkedList<Line>& v, const Pose2D& p);
extern void operator-=(LinkedList<Line>& v, const Pose2D& p);

QDataStream& operator<<(QDataStream& out, const Pose2D &o);
QDataStream& operator>>(QDataStream& in, Pose2D &o);
QDebug operator<<(QDebug dbg, const Pose2D &o);

#endif

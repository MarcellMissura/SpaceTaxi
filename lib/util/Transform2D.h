#ifndef TRANSFORM2D_H
#define TRANSFORM2D_H

#include "Vec3.h"
#include "Vec2.h"
#include "Transform3D.h"
#include "Vector.h"

class Transform2D
{
public:

    double x,y,z;

    Transform2D();
    Transform2D(const Vec2 &o, double zo=0);
    Transform2D(double xo, double yo, double zo=0);

public:

    Vec2 pos() const;
    void setPos(const Vec2& p);
    void setPos(double x, double y);
    void set(const Vec2& p, const Vec2& to);
    double heading() const;
    void setHeading(double a);
    void scale(double sx, double sy);
    void scale(const Vec2& s);
    void translate(double dx, double dy);
    void translate(const Vec2& d);
    void rotate(double a);

    Transform2D operator-(const Transform2D &o) const;
    Transform2D operator+(const Transform2D &o) const;
    Transform2D operator-() const;
    void operator-=(const Transform2D &o);
    void operator+=(const Transform2D &o);

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream &in);

    Transform3D getMatrix() const;

    operator const double*() const {return (double*)this;}
};

extern Vec2 operator+(const Vec2& v, const Transform2D& p);
extern Vec2 operator-(const Vec2& v, const Transform2D& p);

extern void operator+=(Vec2& v, const Transform2D& p);
extern void operator-=(Vec2& v, const Transform2D& p);



extern Vec2 operator+(const Vec2& v, const Transform2D& p);
extern Vec2 operator-(const Vec2& v, const Transform2D& p);
extern Vec3 operator+(const Vec3& v, const Transform2D& p);
extern Vec3 operator-(const Vec3& v, const Transform2D& p);
extern Vector<Vec3> operator+(const Vector<Vec3>& v, const Transform2D& p);
extern Vector<Vec3> operator-(const Vector<Vec3>& v, const Transform2D& p);

extern void operator+=(Vec2& v, const Transform2D& p);
extern void operator-=(Vec2& v, const Transform2D& p);
extern void operator+=(Vec3& v, const Transform2D& p);
extern void operator-=(Vec3& v, const Transform2D& p);
extern void operator+=(Vector<Vec3>& v, const Transform2D& p);
extern void operator-=(Vector<Vec3>& v, const Transform2D& p);


QDataStream& operator<<(QDataStream& out, const Transform2D &o);
QDataStream& operator>>(QDataStream& in, Transform2D &o);
QDebug operator<<(QDebug dbg, const Transform2D &o);

#endif

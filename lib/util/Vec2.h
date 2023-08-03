#ifndef VEC2_H_
#define VEC2_H_

#include "VecN.h"
#include <QPointF>


class Vec2 : public VecN<2>
{

public:

    Vec2() : VecN<2>() {}
    Vec2(double o) : VecN<2>(o) {}
    Vec2(const VecN<2> &o) : VecN<2>(o) {}
    Vec2(const VecN<1> &o, double xo) : VecN<2>(o, xo) {}
    Vec2(double xo, const VecN<1> &o) : VecN<2>(xo, o) {}
    Vec2(const VecN<3> &o) : VecN<2>(o) {}
    Vec2(const double* o) : VecN<2>(o) {}
    Vec2(double xo, double yo, double zo=0, double wo=0, double ao=0, double bo=0, double co=0) : VecN<2>(xo, yo, zo, wo, ao, bo, co) {}
    Vec2(const QPointF &p) : VecN<2>(p.x(), p.y()) {}

    operator QPointF() const {return QPointF(x, y);}
    bool operator <(const Vec2& v) const {return ((x-v.x) < -EPSILON || (fabs(x-v.x) < EPSILON && (y-v.y) < -EPSILON));}
    bool operator <=(const Vec2& v) const {return ( (x-v.x) < -EPSILON || (fabs(x-v.x) < EPSILON && (y-v.y) < EPSILON));}
    bool operator >(const Vec2& v) const {return !(*this<=v);}
    bool operator >=(const Vec2& v) const {return !(*this<v);}

    void operator+=(const Vec2 &o) {VecN<2>::operator +=(o);}
    void operator-=(const Vec2 &o) {VecN<2>::operator -=(o);}

    Vec2 operator+(const Vec2 &o) const
    {
        Vec2 c = *this;
        c += o;
        return c;
    }

    Vec2 operator-(const Vec2 &o) const
    {
        Vec2 c = *this;
        c -= o;
        return c;
    }

public:

    void scale(double s)
    {
        VecN<2>::scale(s);
    }

    void scale(double fx, double fy)
    {
        x *= fx;
        y *= fy;
    }

    void translate(double dx, double dy)
    {
        x += dx;
        y += dy;
    }

    // Rotates the vector by the angle.
    void rotate(double angle)
    {
        if (angle < EPSILON && angle > -EPSILON)
            return;
        double c = cos(angle);
        double s = sin(angle);
        double x_ = x;
        x = x_*c + y*-s;
        y = x_*s + y*c;
    }

    Vec2 rotated(double angle) const
    {
        Vec2 v(*this);
        v.rotate(angle);
        return v;
    }

    // Rotates the vector by the angle using a fast approximation of sin and cos.
    void frotate(double angle)
    {
        if (angle < EPSILON && angle > -EPSILON)
            return;
        double c = fcos(angle);
        double s = fsin(angle);
        double x_ = x;
        x = x_*c + y*-s;
        y = x_*s + y*c;
    }

    Vec2 frotated(double angle) const
    {
        Vec2 v(*this);
        v.frotate(angle);
        return v;
    }

    // Fast rotate for cases where the sin and cos of the angle are known.
    void rotate(double s, double c)
    {
        double x_ = x;
        x = x_*c + y*-s;
        y = x_*s + y*c;
    }

    // Fast rotate for cases where the sin and cos of the angle are known.
    Vec2 rotated(double s, double c) const
    {
        Vec2 v(*this);
        v.rotate(s,c);
        return v;
    }

    // Rotates (flips) the vector by PI/2 in counter clockwise direction (left).
    // If the given sign is -1, the vector is flipped in the clockwise direction.
    // Essentially, this function computes an unnormalized positive normal.
    void flip(int sign=1)
    {
        double x_ = x;
        x = -sign*y;
        y = sign*x_;
    }

    // Rotates (flips) the vector by PI/2 in counter clockwise direction (left).
    // If the given sign is -1, the vector is flipped in the clockwise direction.
    // Essentially, this function computes an unnormalized positive normal.
    Vec2 flipped(int sign=1)
    {
        Vec2 v(*this);
        v.flip(sign);
        return v;
    }

    // Returns the unnormalized positive normal.
    Vec2 normal() const
    {
        Vec2 v = *this;
        v.flip();
        return v;
    }

    // Returns the angle from this to the given vector in the range -PI to PI.
    double angleTo(const Vec2 &o) const
    {
        double dot = x*o.x + y*o.y; // dot product
        double det = x*o.y - y*o.x; // determinant
        return atan2(det, dot);
    }

    // Returns the angle of this vector with respect to the x axis in the range -PI to PI.
    double angle() const
    {
        return atan2(y, x);
    }

    // Returns the angle of this vector with respect to the x axis in the range -PI to PI.
    // This version uses a fast atan2 approximation with a negligible error.
    double fangle() const
    {
        return fatan2(y, x);
    }

    // Returns true if this vector is left of the other vector.
    // In the special case when the vectors are colinear it will return false.
    bool isLeftOf(const Vec2 &o) const
    {
        return det(o) > 0;
    }

    // Returns true if this vector is right of the other vector o.
    // In the special case when the vectors are colinear it will return false.
    bool isRightOf(const Vec2 &o) const
    {
        return det(o) < 0;
    }

    // Decides if this vector is colinear with the other.
    bool isColinearWith(const Vec2& o) const
    {
        return (fabs(det(o)) < EPSILON);
    }

    // Returns the determinant of this vector with o, which is the same as the
    // dot product with o rotated by 90 degrees. v1.det(v2) is > 0 if v1 is left of v2.
    double det(const Vec2& o) const
    {
        return y*o.x-x*o.y;
    }

    // Returns the dot product of this vector with o.
    double dot(const Vec2& o) const
    {
        return x*o.x+y*o.y;
    }
};

#endif

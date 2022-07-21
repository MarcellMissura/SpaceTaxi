#ifndef KEYFRAME2D_H_
#define KEYFRAME2D_H_

#include "Keyframe.h"
#include "util/Vec3.h"
#include "util/Vec2.h"
#include "pml/hpm2D.h"

class Keyframe2D;
extern Keyframe2D operator*(const double scalar, const Keyframe2D& v);
extern Keyframe2D operator*(const Keyframe2D& v, const double scalar);
extern Keyframe2D operator/(const Keyframe2D& v, const double scalar);
extern Keyframe2D operator/(const double scalar, const Keyframe2D& v);

class Keyframe2D
{
public:

    int idx;
    double t;
    double dt;
    double x;
    double vx;
    double ax;
    double y;
    double vy;
    double ay;

    Keyframe2D();
    Keyframe2D(double dt, double x, double vx, double y, double vy);
    ~Keyframe2D(){}

    Keyframe2D(const Vec3& v){idx=0; t=0; dt=0; x=v.x; vx=0; ax=0; y=v.y; vy=0; ay=0;}
    Keyframe2D(const Vec2& v){idx=0; t=0; dt=0; x=v.x; vx=0; ax=0; y=v.y; vy=0; ay=0;}
    Keyframe2D& operator=(const Vec3& v) {x=v.x; vx=0; y=v.y; vy=0; return *this;}
    Keyframe2D& operator=(const Vec2& v) {x=v.x; vx=0; y=v.y; vy=0; return *this;}

    Keyframe2D(const Hpm2D& v){idx=0; t=0; dt=0; x=v.x; vx=v.vx; ax=v.ax; y=v.y; vy=v.vy; ay=v.ay;}
    Keyframe2D& operator=(const Hpm2D& v) {x=v.x; vx=v.vx; ax=v.ax; y=v.vy; vy=v.vy; ay=v.ay; return *this;}

    Keyframe2D operator-() const {return Keyframe2D(dt, -x, -vx, -y, -vy);}

    Keyframe2D operator+(const Keyframe2D& v) const
    {
        Keyframe2D lm = *this;
        lm.x += v.x;
        lm.vx += v.vx;
        lm.y += v.y;
        lm.vy += v.vy;
        return lm;
    }

    Keyframe2D operator+(const Vec2& v) const
    {
        Keyframe2D lm = *this;
        lm.x += v.x;
        lm.y += v.y;
        return lm;
    }

    Keyframe2D operator-(const Keyframe2D& v) const
    {
        Keyframe2D lm = *this;
        lm.x -= v.x;
        lm.vx -= v.vx;
        lm.y -= v.y;
        lm.vy -= v.vy;
        return lm;
    }

    Keyframe2D operator-(const Vec2& v) const
    {
        Keyframe2D lm = *this;
        lm.x -= v.x;
        lm.y -= v.y;
        return lm;
    }

    void operator+=(const Keyframe2D& v)
    {
        x+=v.x;
        vx+=v.vx;
        y+=v.y;
        vy+=v.vy;
    }

    void operator+=(const Vec2& v)
    {
        x+=v.x;
        y+=v.y;
    }

    void operator-=(const Keyframe2D& v)
    {
        x-=v.x;
        vx-=v.vx;
        y-=v.y;
        vy-=v.vy;
    }

    void operator-=(const Vec2& v)
    {
        x-=v.x;
        y-=v.y;
    }

    void operator*=(const double scalar)
    {
        x*=scalar;
        vx*=scalar;
        y*=scalar;
        vy*=scalar;
    }

    void operator/=(const double scalar)
    {
        x/=scalar;
        vx/=scalar;
        y/=scalar;
        vy/=scalar;
    }

    bool operator==(const Keyframe2D& v) const
    {
        return (fabs(x-v.x) <= EPSILON
                && fabs(y-v.y) <= EPSILON
                && fabs(vx-v.vx) <= EPSILON
                && fabs(vy-v.vy) <= EPSILON);
    }

    bool operator!=(const Keyframe2D& v) const
    {
        return (fabs(x-v.x) > EPSILON
                || fabs(y-v.y) > EPSILON
                || fabs(vx-v.vx) > EPSILON
                || fabs(vy-v.vy) > EPSILON);
    }

    bool operator<(const Keyframe2D& v) const
    {
        return (t < v.t);
    }

    bool operator<=(const Keyframe2D& v) const
    {
        return (t <= v.t);
    }

    bool operator>(const Keyframe2D& v) const
    {
        return (t > v.t);
    }

    bool operator>=(const Keyframe2D& v) const
    {
        return (t >= v.t);
    }

    void reset();
    void set(double dt, double x, double vx, double y, double vy);
    void forward();
    void forward(double dt);
    void forward(double dt, double ax, double ay);
    Keyframe2D forwarded();
    Keyframe2D forwarded(double dt);
    Keyframe2D forwarded(double dt, double ax, double ay);

    Vec2 pos() const {return Vec2(x, y);}
    Vec2 vel() const {return Vec2(vx, vy);}
    Vec2 acc() const {return Vec2(ax, ay);}

    // Sets the position.
    void setPos(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

    // Sets the position.
    void setPos(const Vec2 &p)
    {
        x = p.x;
        y = p.y;
    }

    // Sets the velocity vector.
    void setVel(const Vec2 &v)
    {
        vx = v.x;
        vy = v.y;
    }

    // Sets the velocity vector.
    void setVel(double vx, double vy)
    {
        this->vx = vx;
        this->vy = vy;
    }

    // Sets the acceleration.
    void setAcc(const Vec2 &a)
    {
        ax = a.x;
        ay = a.y;
    }

    // Sets the acceleration.
    void setAcc(double ax, double ay)
    {
        this->ax = ax;
        this->ay = ay;
    }

    Keyframe getX() const {return Keyframe(dt, x, vx);}
    Keyframe getY() const {return Keyframe(dt, y, vy);}

};

QDebug operator<<(QDebug dbg, const Keyframe2D &v);
QDataStream& operator<<(QDataStream& out, const Keyframe2D &o);
QDataStream& operator>>(QDataStream& in, Keyframe2D &o);

#endif /* KEYFRAME2D_H_ */

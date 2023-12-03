#ifndef KEYFRAME2D_H_
#define KEYFRAME2D_H_
#include "Keyframe.h"
#include "lib/util/Vec2.h"

class Keyframe2D
{
public:
    Keyframe2D();
    Keyframe2D(double t, double x, double y, double vx, double vy, double ax=0, double ay=0, double jx=0, double jy=0);
    ~Keyframe2D(){}

    double t; // absolute time
    double dt; // relative time
    double l; // arc length
    double x;
    double y;
    double vx;
    double vy;
    double ax;
    double ay;
    double jx;
    double jy;

    Vec2 pos() const {return Vec2(x, y);}
    Vec2 vel() const {return Vec2(vx, vy);}
    Vec2 acc() const {return Vec2(ax, ay);}
    Vec2 jerk() const {return Vec2(jx, jy);}

    void setT(double t);
    void setDt(double dt);
    void setPos(const Vec2& v);
    void setVel(const Vec2& v);
    void setAcc(const Vec2& v);
    void setJerk(const Vec2& v);

    void forward();
    void forwardJ();
    void forwardA();
    void forward(double dt);
    void forwardJ(double dt);
    void forwardA(double dt);
    void forward(double dt, const Vec2& j);
    void forwardJ(double dt, const Vec2& j);
    void forwardA(double dt, const Vec2& a);

    bool operator<(const Keyframe2D& v) const {return (t < v.t);}
    bool operator<=(const Keyframe2D& v) const {return (t <= v.t);}
    bool operator>(const Keyframe2D& v) const {return (t > v.t);}
    bool operator>=(const Keyframe2D& v) const {return (t >= v.t);}
    bool operator==(const Keyframe2D& v) const {return (t == v.t);}
    bool operator!=(const Keyframe2D& v) const {return (t != v.t);}

    Keyframe getX() const {return Keyframe(dt, x, vx, ax);}
    Keyframe getY() const {return Keyframe(dt, y, vy, ay);}
};

QDebug operator<<(QDebug dbg, const Keyframe2D &v);

#endif /* KEYFRAME2D_H_ */

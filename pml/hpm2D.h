#ifndef HPM2D_H_
#define HPM2D_H_
#include "util/Vec2.h"
#include "geometry/Line.h"
#include "geometry/Box.h"
#include <QPainter>

class Hpm2D
{
public:

    double t;
    double dt;
    double x;
    double vx;
    double ax;
    double jx;
    double y;
    double vy;
    double ay;
    double jy;

public:
	Hpm2D();
    ~Hpm2D(){}

    void reset();

    void set(double x, double y, double vx=0, double vy=0, double ax=0, double ay=0, double jx=0, double jy=0);

    Vec2 pos() const;
    Vec2 vel() const;
    Vec2 acc() const;
    Vec2 jerk() const;
    double heading() const;

    void setPos(const Vec2& p);
    void setPos(double x, double y);
    void setVel(const Vec2& v);
    void setVel(double vx, double vy);
    void setAcc(const Vec2& c);
    void setAcc(double ax, double ay);
    void setJerk(const Vec2& j);
    void setJerk(double jx, double jy);

    void translate(double dx, double dy);
    void translate(const Vec2& d);

    void predict();
    Hpm2D predicted() const;
    void predict(double dt);
    Hpm2D predicted(double dt) const;

    const Box& boundingBox() const;

    double intersects(const Line& line) const;

    bool operator==(const Hpm2D& l);
    bool operator!=(const Hpm2D& l);

    void draw(QPainter* painter) const;

private:
    mutable bool boundingBoxValid;
    mutable Box aabb;
};

QDebug operator<<(QDebug dbg, const Hpm2D &s);
QDataStream& operator<<(QDataStream& out, const Hpm2D &o);
QDataStream& operator>>(QDataStream& in, Hpm2D &o);

#endif // HPM2D_H_

#ifndef UNICYCLE_H_
#define UNICYCLE_H_
#include "util/Vec2.h"
#include "util/Pose2D.h"
#include "util/Vector.h"
#include "pml/hpm2D.h"
#include "geometry/Line.h"
#include "geometry/Box.h"
#include <QPainter>

class Unicycle
{

public:

    double dt; // relative time
    double x, y, theta;
    double v, w;
    double a, b;

    Unicycle();
    Unicycle(double x, double y, double theta, double v, double w);
    ~Unicycle(){}

    // Conversion to and from Hpm2D
    Unicycle(const Hpm2D& v);
    Unicycle& operator=(const Hpm2D& v);
    operator Hpm2D() const;

    Vec2 pos() const;
    void setPos(const Vec2& p);
    void setPos(double x, double y);
    void translate(double dx, double dy);
    void translate(const Vec2& d);

    double heading() const;
    void setHeading(double theta);
    void turn(double dtheta);

    void rotate(double phi);
    Unicycle rotated(double phi) const;

    Pose2D pose() const;
    void setPose(const Pose2D &v);
    void setPose(double xx, double yy, double th);

    Vec2 vel() const;
    void setVel(const Vec2& v);
    void setVel(double v, double w);

    Vec2 acc() const;
    void setAcc(const Vec2& c);
    void setAcc(double a, double b);

    void predict();
    Unicycle predicted() const;
    void predict(double dt);
    Unicycle predicted(double dt) const;
    void simulate(double dt);

    Vec2 icc() const;
    double orbitAngle() const;
    const Box& boundingBox() const;

    double intersects(const Line& line) const;
    double intersects(const Line& line, const Vec2 &lineVelocity) const;

    bool operator==(const Unicycle& u) const
    {
        return (fabs(x-u.x) <= EPSILON
                && fabs(y-u.y) <= EPSILON
                && fabs(v-u.v) <= EPSILON
                && fabs(w-u.w) <= EPSILON);
    }

    bool operator!=(const Unicycle& u) const
    {
        return (fabs(x-u.x) > EPSILON
                || fabs(y-u.y) > EPSILON
                || fabs(v-u.v) > EPSILON
                || fabs(w-u.w) > EPSILON);
    }

    void draw(QPainter* painter) const;
    void draw() const;

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream& in);

private:
    mutable bool boundingBoxValid;
    mutable Box aabb;
    mutable Vector<double> brackets;
};

QDebug operator<<(QDebug dbg, const Unicycle &o);
QDataStream& operator<<(QDataStream& out, const Unicycle &o);
QDataStream& operator>>(QDataStream& in, Unicycle &o);



#endif

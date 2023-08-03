#ifndef KEYFRAME_H_
#define KEYFRAME_H_
#include "lib/util/Vec2.h"
#include <QDebug>

class Keyframe
{
public:
    Keyframe();
    Keyframe(double dt, double x, double v, double a=0);
    ~Keyframe(){}

    double t; // absolute time
    double dt; // relative time
    double x;
    double v;
    double a;
    int idx;

    void set(double t, double x, double v, double a=0);
    Vec2 pos();
    void setPos(Vec2);
    void moveBy(Vec2);

    void forward();
    void forward(double dt);
    void forward(double dt, double a);

    bool operator<(const Keyframe& v) const {return (t < v.t);}
    bool operator<=(const Keyframe& v) const {return (t <= v.t);}
    bool operator>(const Keyframe& v) const {return (t > v.t);}
    bool operator>=(const Keyframe& v) const {return (t >= v.t);}
    bool operator==(const Keyframe& v) const {return (t == v.t);}
    bool operator!=(const Keyframe& v) const {return (t != v.t);}
};

QDebug operator<<(QDebug dbg, const Keyframe &k);

#endif /* KEYFRAME_H_ */

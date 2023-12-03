#ifndef BANGBANG_H_
#define BANGBANG_H_
#include "Keyframe.h"
#include "lib/util/Vector.h"
#include <QPainter>

class BangBang
{
public:

    Vector<Keyframe> keyframes;
    Vector<Keyframe> ctrl;

    double V;
    double A;

public:

    BangBang();
    ~BangBang(){}

    void clear();
    void reset();

    void setA(double A);
    void setV(double V);

    void addKeyframes(const Vector<Keyframe>& inputFrames);
    void addKeyframe(const Keyframe& kf);
    void addKeyframe(double t, double x=0, double v=0, double a=0);
    Vector<Keyframe>& getKeyframes();

    const Vector<Keyframe>& getTimedControlSequence();
    const Vector<Keyframe>& getTimeOptimalControlSequence();
    const Vector<Keyframe>& getTimeOptimalControlSequence2();

    Keyframe evaluateAt(double dt) const;
    double getTotalTime() const;

    void draw(QPainter& painter, const QPen& pen) const;

private:
    void move(double dt, double a, int idx=0);
};

QDebug operator<<(QDebug dbg, const BangBang& o);
QDebug operator<<(QDebug dbg, const BangBang* o);

#endif // BANGBANG_H_

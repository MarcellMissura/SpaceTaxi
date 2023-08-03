#ifndef BANGBANG_H_
#define BANGBANG_H_
#include "Keyframe.h"
#include "lib/util/Vector.h"

class BangBang
{
public:

    Vector<Keyframe> keyframes;
    Vector<Keyframe> ctrl;

    double VU;
    double VL;
    double A;

public:

    BangBang();
    ~BangBang(){}

    void clear();
    void reset();

    void setA(double A);
    void setV(double V);
    void setVU(double VU);
    void setVL(double VL);

    void addKeyframes(const Vector<Keyframe>& inputFrames);
    void addKeyframe(const Keyframe& kf);
    void addKeyframe(double dt, double x=0, double v=0, double a=0);

    const Vector<Keyframe>& getTimedControlSequence();
    const Vector<Keyframe>& getTimeOptimalControlSequence();
    const Vector<Keyframe>& getTimeOptimalControlSequence2();

    Keyframe evaluateAt(const Vector<Keyframe>& ctrl, double dt) const;

private:
    void move(double dt, double a, int idx=0);
};

#endif // BANGBANG_H_

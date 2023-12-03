#ifndef KEYFRAMEPLAYER_H_
#define KEYFRAMEPLAYER_H_
#include "globals.h"
#include "Keyframe.h"
#include "util/List.h"
#include <QList>
#include <QString>
#include <QPainter>
#include <QColor>

class KeyframePlayer
{
public:

    double VU;
    double VL;
	double A;
    double J;

    List<Keyframe> keyframes;
    mutable List<Keyframe> ctrl;

    QString name;

    bool showVelocity;
    bool showAcceleration;
    bool showJerk;

    double peakAcceleration;
    double peakVelocity;
    double computationTime;

    KeyframePlayer();
    virtual ~KeyframePlayer(){}

    virtual void clear();
    virtual void reset();

    virtual void setA(double A);
    virtual void setV(double V);
    virtual void setVU(double VU);
    virtual void setVL(double VL);
    virtual void setJ(double J);

    virtual void addKeyframes(const List<Keyframe>& keyframes);
    virtual void addKeyframe(const Keyframe& kf);
    virtual void addKeyframe(double t, double x=0, double v=0, double a=0, double j=0);

    virtual const List<Keyframe>& getTimedControlSequence();
    virtual const List<Keyframe>& getTimeOptimalControlSequence();

    virtual Keyframe evaluateAt(const List<Keyframe>& ctrl, double t) const;
    virtual void paint(QPainter& painter, QColor c);

    virtual void maximizeSmoothness();
    virtual void minimizeTime();
    virtual void measureComputationTime();

    virtual double getPeakVelocity() const;
    virtual double getPeakAcceleration() const;
    virtual double getComputationTime() const;

    void rewriteAbsoluteTimes();
    void rewriteRelativeTimes();

};

#endif // KEYFRAMEPLAYER_H_

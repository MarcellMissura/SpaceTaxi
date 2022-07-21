#include "BangBang2D.h"
#include "globals.h"
#include <QPainterPath>
#include "util/ColorUtil.h"
#include "blackboard/Config.h"
#include <QMutexLocker>

// Instantiates a keyframe player no acceleration and velocity bounds.
BangBang2D::BangBang2D()
{

}

// Sets the acceleration limit to A.
void BangBang2D::setA(double A)
{
    X.setA(A);
    Y.setA(A);
}

// Sets the velocity limit to V.
void BangBang2D::setV(double V)
{
    X.setV(V);
    Y.setV(V);
}

// Sets the upper velocity limit to VU.
void BangBang2D::setVU(double VU)
{
    X.setVU(VU);
    Y.setVU(VU);
}

// Sets the lower velocity limit to VL.
void BangBang2D::setVL(double VL)
{
    X.setVL(VL);
    Y.setVL(VL);
}

// Sets the upper velocity limit to VU.
void BangBang2D::setVU(Vec2 VU)
{
    X.setVU(VU.x);
    Y.setVU(VU.y);
}

// Sets the lower velocity limit to VL.
void BangBang2D::setVL(Vec2 VL)
{
    X.setVL(VL.x);
    Y.setVL(VL.y);
}

// Clears the keyframe player. It deletes all keyframes.
void BangBang2D::clear()
{
    QMutexLocker locker(&mutex);

    X.clear();
    Y.clear();
}

// Clears the keyframe player. It deletes all keyframes.
void BangBang2D::reset()
{
    clear();
}

// Adds a keyframe to the motion sequence. dt is the relative time with respect to
// the previous frame, x and vx are the position and the velocity in the x direction,
// and y and vy are the position and the velocity in the y dimension.
void BangBang2D::addKeyframe(double dt, double x, double vx, double y, double vy)
{
    Keyframe2D kf(dt, x, vx, y, vy);
    addKeyframe(kf);
}

// Adds a keyframe to the motion sequence.
void BangBang2D::addKeyframe(const Keyframe2D &kf)
{
    QMutexLocker locker(&mutex);
    X.addKeyframe(kf.dt, kf.x, kf.vx);
    Y.addKeyframe(kf.dt, kf.y, kf.vy);
}

void BangBang2D::addKeyframes(const Vector<Keyframe2D> &kfs)
{
    for (int k = 0; k < kfs.size(); k++)
        addKeyframe(kfs[k]);
}

// Generates the piecewise constant acceleration motion control sequence that encodes the
// *synchronized* motion trajectory through the given keyframes. Synchronized means that
// every keyframe is reached reached at the same time in the x and y directions.
// Please note that the acceleration limit A has to be set beforehand.
Vector<Keyframe2D> BangBang2D::getTimedControlSequence()
{
    // Compute the timed control sequences separately for X and Y.
    Vector<Keyframe> ctrlX = X.getTimedControlSequence();
    Vector<Keyframe> ctrlY = Y.getTimedControlSequence();

    // Splice the X and Y ctrl sequences to a two dimensional one with a merge.
    Vector<Keyframe2D> ctrl;
    int i = 0;
    int j = 0;
    bool finished = false;
    while (!finished)
    {
        finished = true;
        if (i < ctrlX.size() && (j >= ctrlY.size() || ctrlX[i].t < ctrlY[j].t))
        {
            Keyframe kfX = ctrlX[i];
            Keyframe kfY = Y.evaluateAt(ctrlY, kfX.t);
            Keyframe2D kf;
            kf.t = kfX.t;
            kf.x = kfX.x;
            kf.vx = kfX.v;
            kf.ax = kfX.a;
            kf.y = kfY.x;
            kf.vy = kfY.v;
            kf.ay = kfY.a;
            ctrl << kf;

            i++;
            finished = false;
        }
        else if (j < ctrlY.size() && (i >= ctrlX.size() || ctrlY[j].t < ctrlX[i].t))
        {
            Keyframe kfY = ctrlY[j];
            Keyframe kfX = X.evaluateAt(ctrlX, kfY.t);
            Keyframe2D kf;
            kf.t = kfX.t;
            kf.x = kfX.x;
            kf.vx = kfX.v;
            kf.ax = kfX.a;
            kf.y = kfY.x;
            kf.vy = kfY.v;
            kf.ay = kfY.a;
            ctrl << kf;

            j++;
            finished = false;
        }
    }


    // Prune the control sequence. Throw out keyframes with a too small time difference.
    for (int i = ctrl.size()-1; i > 0; i--)
        if (fabs(ctrl[i].t-ctrl[i-1].t) < 1.0E-6)
            ctrl.removeAt(i);

    // Rewrite the correct dts.
    for (int i = 0; i < ctrl.size()-1; i++)
        ctrl[i].dt = ctrl[i+1].t-ctrl[i].t;

    return ctrl;
}


// Generates the piecewise constant acceleration motion control sequence that encodes the
// *synchronized* time optimal motion trajectory through the given keyframes. Synchronized
// means that every keyframe is reached at the same time in the x and y directions.
// The absolute and relative times of the keyframes are rewritten during this process to
// reflect the times of the resulting motion. Please note that the acceleration limit A has
// to be set beforehand.
Vector<Keyframe2D> BangBang2D::getTimeOptimalControlSequence()
{
    //qDebug() << "   BangBang2D::getTimeOptimalControlSequence()";
    //qDebug() << "   Keyframes X:" << X.keyframes;
    //qDebug() << "   Keyframes Y:" << Y.keyframes;

    // Compute the time optimal ctrl sequence for both dimensions.
    // We do this to rewrite the keyframes to the optimal times.
    X.getTimeOptimalControlSequence();
    Y.getTimeOptimalControlSequence();

    // Synchronize the keyframes by rewriting the times, each to the time of the more restrictive dimension.
    // Both dimensions must have an equal amount of keyframes that will be matched pairwise.
    double t = 0;
    double dt;
    for (int i = 0; i < X.keyframes.size(); i++)
    {
        dt = qMax(X.keyframes[i].dt, Y.keyframes[i].dt);
        X.keyframes[i].dt = dt;
        Y.keyframes[i].dt = dt;
        X.keyframes[i].t = t;
        Y.keyframes[i].t = t;
        t += dt;
    }

    //qDebug() << "   rewritten keyframes X:" << X.keyframes;
    //qDebug() << "   rewritten keyframes Y:" << Y.keyframes;

    // Recompute the control sequences to synchronize the times.
    const Vector<Keyframe>& ctrlX = X.getTimedControlSequence();
    const Vector<Keyframe>& ctrlY = Y.getTimedControlSequence();

    //qDebug() << "   ctrl X:" << ctrlX;
    //qDebug() << "   ctrl Y:" << ctrlY;

    // Merge the ctrl sequences of all dimensions to one two dimensional sequence.
    Vector<Keyframe2D> ctrl;

    int i = 0;
    int j = 0;
    Keyframe kfX = ctrlX[0];
    Keyframe kfY = ctrlY[0];
    Keyframe2D kf;
    while (i < ctrlX.size() && j < ctrlY.size())
    {
        // Synchronized.
        if (fabs(ctrlX[i].t-ctrlY[j].t) < 1.0E-6)
        {
            kfX = ctrlX[i];
            kfY = ctrlY[j];
            i++;
            j++;
        }

        // X has an in between keyframe.
        else if(ctrlX[i].t < ctrlY[j].t)
        {
            kfX = ctrlX[i];
            kfY = Y.evaluateAt(ctrlY, kfX.t);
            i++;
        }

        // Y has an in between keyframe.
        else
        {
            kfY = ctrlY[j];
            kfX = X.evaluateAt(ctrlX, kfY.t);
            j++;
        }

        kf.t = kfX.t;
        kf.idx = kfX.idx;
        kf.x = kfX.x;
        kf.vx = kfX.v;
        kf.ax = kfX.a;
        kf.y = kfY.x;
        kf.vy = kfY.v;
        kf.ay = kfY.a;
        ctrl << kf;
    }

    //qDebug() << "   ctrl merged:" << ctrl;

    // Rewrite the dts.
    for (int i = 0; i < ctrl.size()-1; i++)
        ctrl[i].dt = ctrl[i+1].t-ctrl[i].t;

    //qDebug() << "   final ctrl:" << ctrl;

    return ctrl;
}

// Computes a threeway pass from the first keyframe over the second keyframe to the
// third keyframe. Yes, there has to be exactly three keyframes. Only the first keyframe
// is allowed to have nonzero velocity. On the upside, the optimal pass through velocity
// through the middle keyframe is computed automatically.
Vector<Keyframe2D> BangBang2D::threeWayPass()
{
    //qDebug() << "   BangBang2D::threeWayPass()";
    //qDebug() << "   Keyframes X:" << X.keyframes;
    //qDebug() << "   Keyframes Y:" << Y.keyframes;

    // Compute the time optimal ctrl sequence for both dimensions.
    // We do this to rewrite the keyframes to the optimal times.
    X.threeWayPass();
    Y.threeWayPass();

    // Synchronize the keyframes by rewriting the times, each to the time of the more restrictive dimension.
    // Both dimensions must have an equal amount of keyframes that will be matched pairwise.
    double t = 0;
    double dt;
    for (int i = 0; i < X.keyframes.size(); i++)
    {
        dt = qMax(X.keyframes[i].dt, Y.keyframes[i].dt);
        X.keyframes[i].dt = dt;
        Y.keyframes[i].dt = dt;
        X.keyframes[i].t = t;
        Y.keyframes[i].t = t;
        t += dt;
    }

    //qDebug() << "   rewritten keyframes X:" << X.keyframes;
    //qDebug() << "   rewritten keyframes Y:" << Y.keyframes;

    // Recompute the control sequences to synchronize the times.
    const Vector<Keyframe>& ctrlX = X.getTimedControlSequence();
    const Vector<Keyframe>& ctrlY = Y.getTimedControlSequence();

    //qDebug() << "   ctrl X:" << ctrlX;
    //qDebug() << "   ctrl Y:" << ctrlY;

    // Merge the ctrl sequences of both dimensions to a two dimensional sequence.
    Vector<Keyframe2D> ctrl;

    int i = 0;
    int j = 0;
    Keyframe kfX = ctrlX[0];
    Keyframe kfY = ctrlY[0];
    Keyframe2D kf;
    while (i < ctrlX.size() && j < ctrlY.size())
    {
        // Synchronized.
        if (fabs(ctrlX[i].t-ctrlY[j].t) < 1.0E-6)
        {
            kfX = ctrlX[i];
            kfY = ctrlY[j];
            i++;
            j++;
        }

        // X has an in between keyframe.
        else if(ctrlX[i].t < ctrlY[j].t)
        {
            kfX = ctrlX[i];
            kfY = Y.evaluateAt(ctrlY, kfX.t);
            i++;
        }

        // Y has an in between keyframe.
        else
        {
            kfY = ctrlY[j];
            kfX = X.evaluateAt(ctrlX, kfY.t);
            j++;
        }

        kf.t = kfX.t;
        kf.idx = kfX.idx;
        kf.x = kfX.x;
        kf.vx = kfX.v;
        kf.ax = kfX.a;
        kf.y = kfY.x;
        kf.vy = kfY.v;
        kf.ay = kfY.a;
        ctrl << kf;
    }

    // Rewrite the dts.
    for (int i = 0; i < ctrl.size()-1; i++)
        ctrl[i].dt = ctrl[i+1].t-ctrl[i].t;

    //qDebug() << "   final ctrl:" << ctrl;

    return ctrl;
}

// Evaluates the given ctrl sequence at a given time dt and returns the calculated
// motion state. dt is interpreted as relative to the first frame in ctrl (dt robustness).
// It is best if the ctrl signal starts at absolute time 0, but it is not necessary.
Keyframe2D BangBang2D::evaluateAt(const Vector<Keyframe2D> &ctrl, double dt) const
{
    if (ctrl.isEmpty())
        return Keyframe2D();

    int index = ctrl.size()-1;
    while (index > 0 && dt < ctrl[index].t-ctrl[0].t-EPSILON)
        index--;

    Keyframe2D kf = ctrl[index];
    kf.forward(dt-kf.t);
    return kf;
}

// Draws the trajectory described by the ctrl sequence onto the QPainter up until the
// relative time dt.
void BangBang2D::draw(QPainter *painter, const Vector<Keyframe2D> &ctrl, double dt) const
{
    if (ctrl.size() < 2 || ctrl[0] == ctrl[1])
        return;

    // Draw the trajectory backwards. I forgot why it is better this way.
    double ddt = 0.025;
    dt = (dt < 0) ? ctrl.last().t-ctrl.first().t : qMin(dt, ctrl.last().t-ctrl.first().t);
    double ct = dt;
    Keyframe2D kf = evaluateAt(ctrl, ct);
    while (ct > ddt)
    {
        ct -= ddt;
        Keyframe2D kfold = kf;
        kf = evaluateAt(ctrl, ct);
        painter->drawLine(QLineF(kfold.x, kfold.y, kf.x, kf.y));
    }
    if (kf != ctrl[0])
        painter->drawLine(QLineF(kf.x, kf.y, ctrl[0].x, ctrl[0].y));

    // The switching points.
    double s = 1.3/painter->transform().m11();
    for (int i = 0; i < ctrl.size(); i++)
    {
        if (ctrl[i].t-ctrl.first().t > dt)
            break;
        painter->drawEllipse(QPointF(ctrl[i].x, ctrl[i].y), s, s);
    }
}

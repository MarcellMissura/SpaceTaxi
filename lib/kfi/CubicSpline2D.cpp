#include "CubicSpline2D.h"
#include "lib/util/DrawUtil.h"

// Instantiates a motion player no acceleration and velocity bounds.
CubicSpline2D::CubicSpline2D()
{

}

// Clears the keyframe player. It deletes all keyframes.
void CubicSpline2D::clear()
{
    X.clear();
    Y.clear();
}

// Clears the keyframe player. It deletes all keyframes.
void CubicSpline2D::reset()
{
    clear();
}

// Adds a keyframe to the motion sequence. dt is the relative time with respect to
// the previous frame, x and vx are the position and the velocity in the x direction,
// and y and vy are the position and the velocity in the y dimension.
void CubicSpline2D::addKeyframe(double t, double x, double vx, double y, double vy)
{
    Keyframe2D kf(t, x, y, vx, vy);
    addKeyframe(kf);
}

// Adds a keyframe to the motion sequence.
void CubicSpline2D::addKeyframe(const Keyframe2D &kf)
{
    X.addKeyframe(kf.t, kf.x, kf.vx);
    Y.addKeyframe(kf.t, kf.y, kf.vy);
}

// Adds a Vector of keyframes to the motion sequence.
void CubicSpline2D::addKeyframes(const Vector<Keyframe2D> &kfs)
{
    for (int k = 0; k < kfs.size(); k++)
        addKeyframe(kfs[k]);
}

// Generates the piecewise constant jerk motion control sequence that encodes the
// *synchronized* time optimal motion trajectory through the given keyframes. Synchronized
// means that every keyframe is reached at the same time in the x and y directions.
// The absolute and relative times of the keyframes are rewritten during this process to
// reflect the times of the resulting motion. Please note that the acceleration limit A
// and the velocity limit V have to be set beforehand.
Vector<Keyframe2D> CubicSpline2D::getControlSequence()
{
//    qDebug() << "   CubicSpline2D::getControlSequence()";
//    qDebug() << "   Keyframes X:" << X.getKeyframes();
//    qDebug() << "   Keyframes Y:" << Y.getKeyframes();

    // Recompute the control sequences to synchronize the times.
    const Vector<Keyframe>& ctrlX = X.getControlSequence();
    const Vector<Keyframe>& ctrlY = Y.getControlSequence();

//    qDebug() << "   ctrl X:" << ctrlX;
//    qDebug() << "   ctrl Y:" << ctrlY;

    // Merge the ctrl sequences of both dimensions to a two dimensional sequence.
    ctrl.clear();

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
            kfY = Y.evaluateAt(kfX.t);
            i++;
        }

        // Y has an in between keyframe.
        else
        {
            kfY = ctrlY[j];
            kfX = X.evaluateAt(kfY.t);
            j++;
        }

        kf.t = kfX.t;
        kf.x = kfX.x;
        kf.vx = kfX.v;
        kf.ax = kfX.a;
        kf.jx = kfX.j;
        kf.y = kfY.x;
        kf.vy = kfY.v;
        kf.ay = kfY.a;
        kf.jy = kfY.j;
        ctrl << kf;
    }

//    qDebug() << "   ctrl merged:" << ctrl;

    // Rewrite the dts.
    for (int i = 0; i < ctrl.size()-1; i++)
        ctrl[i].dt = ctrl[i+1].t-ctrl[i].t;

//    qDebug() << "   final ctrl:" << ctrl;

    return ctrl;
}

// Returns the total time of the motion.
double CubicSpline2D::getTotalTime() const
{
    return ctrl.last().t;
}

// Evaluates the given ctrl sequence at a given time dt and returns the calculated
// motion state. dt is interpreted as relative to the first frame in ctrl (dt robustness).
// It is best if the ctrl signal starts at absolute time 0, but it is not necessary.
Keyframe2D CubicSpline2D::evaluateAt(double dt) const
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

// Draws the trajectory described by the ctrl sequence onto the QPainter.
void CubicSpline2D::draw(QPainter *painter, const QPen &pen) const
{
    if (ctrl.size() < 2 || ctrl[0] == ctrl[1])
        return;

    painter->save();
    painter->setPen(pen);

    // Draw the motion trajectory.
    double stepSize = 0.01;
    Keyframe2D oldState = ctrl[0];
    for (double t = stepSize; t <= ctrl.last().t; t=t+stepSize)
    {
        Keyframe2D newState = evaluateAt(t);
        painter->drawLine(QPointF(oldState.x, oldState.y), QPointF(newState.x, newState.y));
        oldState = newState;
    }

    // Draw the ctrl frames and the keyframes.
    double screenScale = 1.0/painter->transform().m11();
    painter->setBrush(pen.color());
    for (int i = 0; i < ctrl.size(); i++)
        painter->drawEllipse(QPointF(ctrl[i].x, ctrl[i].y), 3.0*screenScale, 3.0*screenScale);

    painter->restore();
}

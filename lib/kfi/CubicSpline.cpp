#include "CubicSpline.h"

// This is a cubic spline-based non-linear keyframe interpolator.
// https://en.wikipedia.org/wiki/Spline_interpolation
// A cubic spline always meets the boundary conditions and disrespects
// the velocity, acceleration, and jerk limits.

// Instantiates a keyframe player.
CubicSpline::CubicSpline()
{

}

// Clears the keyframe player. It deletes all keyframes.
void CubicSpline::clear()
{
    keyframes.clear();
    ctrl.clear();
}

// Clears the keyframe player. It deletes all keyframes.
void CubicSpline::reset()
{
    clear();
}

// Appends a set of keyframes to the motion sequence.
void CubicSpline::addKeyframes(const Vector<Keyframe> &keyframes)
{
    for (int i = 0; i < keyframes.size(); i++)
        addKeyframe(keyframes[i]);
}

// Appends a keyframe to the motion sequence.
void CubicSpline::addKeyframe(const Keyframe &kf)
{
    keyframes << kf;
}

// Appends a keyframe to the motion sequence. The time parameter dt
// is interpreted as relative time with respect to the keyframe before.
void CubicSpline::addKeyframe(double t, double x, double v, double a)
{
    Keyframe kf(t, x, v, a);
    addKeyframe(kf);
}

// Returns a mutable reference to the keyframes inside the interpolator.
Vector<Keyframe> &CubicSpline::getKeyframes()
{
    return keyframes;
}

// Generates and returns the ctrl structure that describes the piecewise
// constant jerk profile of the spline. The times, locations, and velocities
// of the keyframes are always exactly met. The keyframes need to have their
// absolute times (t) to be set to sensible values for this to work.
const Vector<Keyframe> &CubicSpline::getControlSequence()
{
    ctrl.clear();
    for (int i = 0; i < keyframes.size(); i++)
        ctrl << keyframes[i];
    for (int i = 0; i < keyframes.size()-1; i++)
    {
        // Convert to standard notation.
        double t0 = keyframes[i].t;
        double x0 = keyframes[i].x;
        double v0 = keyframes[i].v;
        double t1 = keyframes[i+1].t;
        double x1 = keyframes[i+1].x;
        double v1 = keyframes[i+1].v;
        double dt = t1-t0;
        double dx = x1-x0;

        double a0 = (6*dx-2*dt*(2*v0+v1))/(dt*dt);
        double a1 = 2*(v1-v0)/dt - a0;
        double j = (a1-a0)/dt;

        ctrl[i].dt = dt;
        ctrl[i].a = a0;
        ctrl[i].j = j;
    }

    return ctrl;
}

// Evaluates the ctrl sequence at a given time dt relative to the first frame
// and returns the calculated motion state. getControlSequence() must have been
// called before this function.
Keyframe CubicSpline::evaluateAt(double dt) const
{
//    qDebug() << "   Keyframe BangBang::evaluateAt()" << dt << ctrl;
    if (ctrl.isEmpty())
        return Keyframe();

    int index = ctrl.size()-1;
    while (index > 0 && dt < ctrl[index].t-ctrl[0].t-EPSILON)
        index--;

    Keyframe kf = ctrl[index];
    kf.forward(dt-kf.t);
//    qDebug() << "   " << index << dt << kf;
    return kf;
}

// Generic QPainter paint code.
void CubicSpline::draw(QPainter& painter, const QPen& pen) const
{
    if (ctrl.isEmpty())
        return;

    painter.save();
    painter.setPen(pen);

    // Draw the motion trajectory.
    double stepSize = 0.01;
    Keyframe oldState = ctrl[0];
    for (double t = stepSize; t <= ctrl.last().t; t=t+stepSize)
    {
        Keyframe newState = evaluateAt(t);
        painter.drawLine(QPointF(oldState.t, oldState.x), QPointF(newState.t, newState.x));
        oldState = newState;
    }

    // Draw the ctrl frames and the keyframes.
    double screenScale = 1.0/painter.transform().m11();
    painter.setBrush(pen.color());
    for (int i = 0; i < ctrl.size(); i++)
    {
        Keyframe kf = ctrl[i];
        QPointF v1 = QPointF(kf.t, kf.x) - QPointF((20*screenScale)*cos(atan(kf.v)), (20*screenScale)*sin(atan(kf.v)));
        QPointF v2 = QPointF(kf.t, kf.x) + QPointF((20*screenScale)*cos(atan(kf.v)), (20*screenScale)*sin(atan(kf.v)));
        painter.drawLine(v1, v2);
        painter.drawEllipse(QPointF(kf.t, kf.x), 3.0*screenScale, 3.0*screenScale);
    }

    painter.restore();
}

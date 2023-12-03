#include "KeyframePlayer.h"
#include "util/Statistics.h"
#include <util/StopWatch.h>
#include "learner/Grid.h"

// This is a generic KeyframePlayer base class that does not provide a specific implementation.
// A KeyframePlayer allows to compute a smooth trajectory from a set of keyframes -- the time,
// position, and velocity in specific points of the trajectory. Jerk, acceleration, and velocity
// bounds can be set to model the capabilities of a represented physical system.
//
// To use a keyframe player, and it will have to be one of the specific implementaitons such as
// the CubicSpline, the Reflexxes, or the BangBang keyframe player, instantiate it and set the
// velocity (V), acceleration (A), and jerk (J) bounds first. Then provide keyframes with the
// addKeyframe() methods. Then, the getTimeOptimalControlSequence() or getTimedControlSequence()
// methods can be used to generate a piecewise constant jerk or acceleration representation (ctrl)
// of the motion trajectory. This representation can then be queried at any continuous point
// using evaluateAt(), or painted on a QPainter object with draw(). See the documentation of
// each method for more information. Example:
//
// RMLKeyframePlayer kfp;
// kfp.setJ(10);
// kfp.setA(10);
// kfp.setV(10);
// kfp.addKeyframe(0, 0, 0); // Time 0s, position 0m, velocity 0mps.
// kfp.addKeyframe(1, 2, 10); // Time 1s, position 2m, velocity 10mps.
// kfp.addKeyframe(2, 4, 0); // Time 2s, position 4m, velocity 0mps.
// QList<Keyframe> ctrl = kfp.getTimedControlSequence();
//
// Additionally, provided optimization methods can be used to
// optimize (rewrite!) the velocities of the intermediate keyframes (not the first,
// not the last) to minimize the travel time, or the smoothness of the trajectory.


// Instantiates a keyframe player with A=1, V=1 and VX=0.5.
KeyframePlayer::KeyframePlayer()
{
    VU = 0;
    VL = 0;
    A = 0;
    J = 0;

    showVelocity = false;
    showAcceleration = false;
    showJerk = false;
    name = "KeyframePlayer";
    peakAcceleration = 0;
    peakVelocity = 0;
    computationTime = 0;
}

// Sets the jerk limit to J.
void KeyframePlayer::setJ(double J)
{
    this->J = J;
}

// Sets the acceleration limit to A.
void KeyframePlayer::setA(double A)
{
	this->A = A;
}

// Sets the upper and lower velocity limits to V.
void KeyframePlayer::setV(double V)
{
    this->VU = V;
    this->VL = -V;
}

// Sets the upper velocity limit to VU.
void KeyframePlayer::setVU(double VU)
{
    this->VU = VU;
}

// Sets the lower velocity limit to VL.
void KeyframePlayer::setVL(double VL)
{
    this->VL = VL;
}

// Clears the keyframe player. It deletes all keyframes.
void KeyframePlayer::clear()
{
    keyframes.clear();
    ctrl.clear();
}

// Clears the keyframe player. It deletes all keyframes.
void KeyframePlayer::reset()
{
    clear();
}

// Appends a keyframe to the keyframe player. The time parameter dt
// is interpreted as the time interval of the keyframe, i.e. the duration for which
// the acceleration / jerk is applied. New keyframes are always appended to the
// end of the list of already added keyframes.
void KeyframePlayer::addKeyframe(double dt, double x, double v, double a, double j)
{
    Keyframe kf(dt, x, v, a, j);
    addKeyframe(kf);
}

// Appends a keyframe to the keyframe player.
// New keyframes are always appended to the end of the list of already added keyframes.
void KeyframePlayer::addKeyframe(const Keyframe& kf)
{
    keyframes << kf;
}

// Appends a set of keyframes to the keyframe player.
// New keyframes are always appended to the end of the list of already added keyframes.
void KeyframePlayer::addKeyframes(const List<Keyframe> &keyframes)
{
    for (int i = 0; i < keyframes.size(); i++)
        addKeyframe(keyframes[i]);
}

// Generates the piecewise constant jerk/acceleration motion control sequence that encodes the
// time optimal motion trajectory through the given keyframes. The times of the given
// keyframes are ignored. Please note that the dynamic bounds (J,A,V) have to be set beforehand.
const List<Keyframe> &KeyframePlayer::getTimeOptimalControlSequence()
{
    return keyframes;
}

// From the given keyframes, a set of control frames are computed that encode the piecewise
// constant jerk/acceleration motion trajectory that touches the keyframes at their given times t,
// unless this would violate the jerk, acceleration, and velocity limits.
const List<Keyframe> &KeyframePlayer::getTimedControlSequence()
{
    return keyframes;
}

// Optimizes the keyframes in a way that the "smoothness" of the motion trajectory is maximized.
void KeyframePlayer::maximizeSmoothness()
{

}

// Optimizes the velocity of the intermediate keyframes (not the first, not the last) to obtain
// the minimal time motion over all keyframes. This is a grid search based generic algorithm
// that is slow and only discretization-optimal, but it works for any number of keyframes and
// all types of keyframe players.
void KeyframePlayer::minimizeTime()
{
    return;

    if (keyframes.size() < 3 || A <= 0)
        return;

    // Compute a grid over the input space, which is the velocities of the intermediate keyframes.
    Grid grid;
    grid.setDim(keyframes.size()-2);
    grid.setN(51);
    grid.setMin(VL);
    grid.setMax(VU);
    grid.rasterize();

    // Perform a grid search over all velocity combinations and find the minimal time one.
    double mint = -1;  // init to -1
    double minn = 0;
    const double* x;
    for (uint n=0; n < grid.getNodeCount(); n++)
    {
        x = grid.getNodeCoordinates(n);
        for (int i=1; i < keyframes.size()-1; i++)
            keyframes[i].v = x[i-1];
        getTimeOptimalControlSequence();
        double t = ctrl.last().t;
        if (t < mint || mint == -1)
        {
            mint = t;
            minn = n;
        }
    }

    // Write the best result into the keyframes.
    x = grid.getNodeCoordinates(minn);
    for (int i=1; i < keyframes.size()-1; i++)
        keyframes[i].v = x[i-1];
}

// Measures the average computation time in microseconds to compute the trajectory parameters
// for one keyframe pair by averaging over 10000 samples.
void KeyframePlayer::measureComputationTime()
{
    StopWatch stopWatch;
    int samples = 10000;

    QList<Keyframe> ran;
    for (int i = 0; i < 2*samples; i++)
    {
        Keyframe kf;
        kf.x = Statistics::uniformSample(-10, 10);
        kf.v = Statistics::uniformSample(-10, 10);
        ran << kf;
    }

    List<Keyframe> keyframesBefore = keyframes;
    computationTime = 0;
    stopWatch.start();
    for (int i = 0; i < samples; i++)
    {
//        clear();
//        addKeyframe(ran[2*i]);
//        addKeyframe(ran[2*i+1]);
//        getTimeOptimalControlSequence();
        getTimedControlSequence();
        //double t = ctrl.isEmpty() ? 0 : ctrl.last().t/2;
        //evaluateAt(ctrl, t);
    }

    // Computation time in microseconds.
    computationTime = 1000000 * stopWatch.elapsedTime()/samples;

    clear();
    for (int i = 0; i < keyframesBefore.size(); i++)
        addKeyframe(keyframesBefore[i]);
}

// Returns the maximum absolute velocity, the L-infinity norm of the velocity profile.
double KeyframePlayer::getPeakVelocity() const
{
    return peakVelocity;
}


// Returns the maximum absolute acceleration, the L-infinity norm of the acceleration profile.
double KeyframePlayer::getPeakAcceleration() const
{
    return peakAcceleration;
}

// Returns the computation time needed to process the current motion trajectory.
double KeyframePlayer::getComputationTime() const
{
    return computationTime;
}

// Rewrites the absolute times of the loaded keyframes based on the relative times.
void KeyframePlayer::rewriteAbsoluteTimes()
{
    keyframes[0].t = 0;
    for (int i = 1; i < keyframes.size(); i++)
        keyframes[i].t = keyframes[i-1].t+keyframes[i-1].dt;
}

// Rewrites the relative times of the loaded keyframes based on the absolute times.
void KeyframePlayer::rewriteRelativeTimes()
{
    for (int i = 0; i < keyframes.size()-1; i++)
        keyframes[i].dt = keyframes[i+1].t-keyframes[i].t;
    keyframes[keyframes.size()-1].dt = 0;
}

// Evaluates the given ctrl sequence at a given time t and returns the calculated
// motion state. Expects the absolute times (t) of the ctrl sequence to be set to
// sensible values. The parameter t is interpreted as relative to the first frame
// in ctrl (t robustness). It is best if the ctrl signal starts at absolute time 0,
// but it is not necessary.
Keyframe KeyframePlayer::evaluateAt(const List<Keyframe> &ctrl, double t) const
{
    //qDebug() << "KeyframePlayer::evaluateAt():" << t;
    //qDebug() << ctrl;

    if (ctrl.isEmpty())
        return Keyframe();

    int index = ctrl.size()-1;
    while (index > 0 && t < ctrl[index].t-ctrl[0].t-1.0E-6)
        index--;

    Keyframe kf = ctrl[index];
    kf.forward(t-kf.t);
    return kf;
}

// Generic QPainter paint code.
void KeyframePlayer::paint(QPainter& painter, QColor c)
{
    if (ctrl.isEmpty())
        return;

    painter.save();

    QPen pen = QPen(c);
    pen.setCosmetic(true);
    pen.setWidth(1);
    painter.setPen(pen);

    // Draw the motion trajectory.
    double stepSize = 0.01;
    Keyframe oldState = ctrl[0];
    for (double t = stepSize; t <= ctrl.last().t; t=t+stepSize)
    {
        Keyframe newState = evaluateAt(ctrl, t);

        painter.drawLine(QPointF(oldState.t, oldState.x), QPointF(newState.t, newState.x));
        if (showVelocity)
            painter.drawLine(QPointF(oldState.t, oldState.v), QPointF(newState.t, newState.v));
        if (showAcceleration)
            painter.drawLine(QPointF(oldState.t, oldState.a), QPointF(newState.t, newState.a));
        if (showJerk)
            painter.drawLine(QPointF(oldState.t, oldState.j), QPointF(newState.t, newState.j));

        oldState = newState;
    }

    // Draw the ctrl frames and the keyframes.
    double screenScale = 1.0/painter.transform().m11();
    pen.setColor(QColor("black"));
    painter.setPen(pen);
    painter.setBrush(c);
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


#include "BangBang2D.h"

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

// Clears the keyframe player. It deletes all keyframes.
void BangBang2D::clear()
{
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
    Keyframe2D kf(dt, x, y, vx, vy);
    addKeyframe(kf);
}

// Adds a keyframe to the motion sequence.
void BangBang2D::addKeyframe(const Keyframe2D &kf)
{
    X.addKeyframe(kf.dt, kf.x, kf.vx);
    Y.addKeyframe(kf.dt, kf.y, kf.vy);
}

// Adds a Vector of keyframes to the motion sequence.
void BangBang2D::addKeyframes(const Vector<Keyframe2D> &kfs)
{
    for (int k = 0; k < kfs.size(); k++)
        addKeyframe(kfs[k]);
}

// Generates the piecewise constant acceleration motion control sequence that encodes the
// *synchronized* time optimal motion trajectory through the given keyframes. Synchronized
// means that every keyframe is reached at the same time in the x and y directions.
// The absolute and relative times of the keyframes are rewritten during this process to
// reflect the times of the resulting motion. Please note that the acceleration limit A
// and the velocity limit V have to be set beforehand.
Vector<Keyframe2D> BangBang2D::getControlSequence(bool debug)
{
    if (debug)
    {
        qDebug() << "   BangBang2D::getControlSequence()";
        qDebug() << "   Keyframes X:" << X.getKeyframes();
        qDebug() << "   Keyframes Y:" << Y.getKeyframes();
    }

    // Compute the time optimal ctrl sequence for both dimensions.
    // We do this to rewrite the keyframes to the optimal times.
    X.getTimeOptimalControlSequence();
    Y.getTimeOptimalControlSequence();

    if (debug)
    {
        //qDebug() << "   Time optimal keyframes X:" << X.getKeyframes();
        //qDebug() << "   Time optimal keyframes Y:" << Y.getKeyframes();
        qDebug() << "   Time optimal ctrl X:" << &X.getTimeOptimalControlSequence();
        qDebug() << "   Time optimal ctrl Y:" << &Y.getTimeOptimalControlSequence();
    }

    // Synchronize the keyframes by rewriting the times, each to the time of the more restrictive dimension.
    // Both dimensions must have an equal amount of keyframes that will be matched pairwise.
    Vector<Keyframe>& kfx = X.getKeyframes();
    Vector<Keyframe>& kfy = Y.getKeyframes();
    double t = 0;
    double dt;
    for (int i = 0; i < kfx.size(); i++)
    {
        dt = qMax(kfx[i].dt, kfy[i].dt);
        kfx[i].dt = dt;
        kfy[i].dt = dt;
        kfx[i].t = t;
        kfy[i].t = t;
        t += dt;
    }

    if (debug)
    {
        //qDebug() << "   Synchronized keyframes X:" << kfx;
        //qDebug() << "   Synchronized keyframes Y:" << kfy;
        qDebug() << "   Synchronized ctrl X:" << &X.getTimedControlSequence();
        qDebug() << "   Synchronized ctrl Y:" << &Y.getTimedControlSequence();
    }

    // Recompute the control sequences to synchronize the times.
    const Vector<Keyframe>& ctrlX = X.getTimedControlSequence();
    const Vector<Keyframe>& ctrlY = Y.getTimedControlSequence();

    // Merge the ctrl sequences of all dimensions to one two dimensional sequence.
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
        kf.dt = kfX.dt;
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

    if (debug)
        qDebug() << "   Merged ctrl:" << ctrl;

    return ctrl;
}

// Evaluates the given ctrl sequence at a given time dt and returns the calculated
// motion state. dt is interpreted as relative to the first frame in ctrl (dt robustness).
// It is best if the ctrl signal starts at absolute time 0, but it is not necessary.
Keyframe2D BangBang2D::evaluateAt(double dt, bool debug) const
{
    if (debug)
        qDebug() << "BangBang2D::evaluateAt()" << dt << ctrl;
    if (ctrl.isEmpty())
        return Keyframe2D();
    if (dt > ctrl.last().t)
        return ctrl.last();

    int index = ctrl.size()-1;
    while (index > 0 && dt < ctrl[index].t-EPSILON)
        index--;

    Keyframe2D kf = ctrl[index];
    kf.forward(dt-kf.t);
    if (debug)
        qDebug() << "   " << index << "dt:" << dt-kf.t << kf;
    return kf;
}

// Evaluates the given ctrl sequence at a given arc length dl and returns the calculated
// motion state. getTotalLength() must have been called before this function.
Keyframe2D BangBang2D::evaluateAtArcLength(double dl) const
{
    qDebug() << "BangBang2D::evaluateAtArcLength:" << dl;
    qDebug() << ctrl;
    for (uint i = 0; i < ctrl.size()-1; i++)
    {
        if (ctrl[i].l <= dl && ctrl[i+1].l > dl)
        {
            // Use regula falsi (Illinois) to solve the equation arcLength(dt) - dl = 0 for dt.
            double z, fz; // The estimated argument and its function value.
            double bz, bf = 1.0E6; // The best argument and its function value so far.
            double x1 = 0;
            double f1 = ctrl[i].l - dl;
            double x2 = ctrl[i].dt;
            double f2 = ctrl[i+1].l - dl;
            Keyframe2D kf = ctrl[i];
            qDebug() << "Illinois init i" << i << "x1:" << x1 << "f1:" << f1 << "z:" << z << "fz:" << fz << "x2:" << x2 << "f2:" << f2;
            for (int k = 0; k < 3; k++)
            {
                // Compute the function value with a linear interpolation inside the bracket.
                double s = (f2-f1)/(x2-x1);
                z = x1 - f1/s;
                kf.dt = z;
                fz = kf.l + arcLength(kf) - dl;

                // Remember the best solution so far.
                if (fabs(fz) < bf)
                {
                    bz = z;
                    bf = fabs(fz);
                }

                if (fz*f2 < 0)
                {
                    x1 = x2;
                    f1 = f2;
                    x2 = z;
                    f2 = fz;
                }
                else
                {
                    f1 *= 0.5;
                    x2 = z;
                    f2 = fz;
                }

                qDebug() << "Illinois" << k << "x1:" << x1 << "f1:" << f1 << "z:" << z
                         << "fz:" << fz << "x2:" << x2 << "f2:" << f2 << "best:" << bz << bf;
            }

            kf.forward(bz);
            qDebug() << "Result:" << ctrl[i] << "->" << kf;
            return kf;
        }
    }
}

// Returns the total time of the motion.
double BangBang2D::getTotalTime() const
{
    return ctrl.last().t;
}

// Returns the total arc length of the motion trajectory.
double BangBang2D::getTotalArcLength()
{
    double length = 0;
    for (int i = 0; i < ctrl.size()-1; i++)
    {
        length += arcLength(ctrl[i]);
        ctrl[i+1].l = length;
    }
    return length;
}

// Determines the arc length of a 2D bang.
double BangBang2D::arcLength(const Keyframe2D &kf, bool debug) const
{
    // double vx0, double vy0, double ax, double ay, double dt
    double vx0 = kf.vx;
    double vy0 = kf.vy;
    double ax = kf.ax;
    double ay = kf.ay;
    double dt = kf.dt;

    // Special case: ax and ay are both zero.
    if (fabs(ax) < EPSILON && fabs(ay) < EPSILON)
        return (dt*kf.vel()).norm();

    // wolfram code: integrate sqrt( (x0+a*t)² + (y0+b*t)² ) // x0 = vxy, y0 = vy0, a = ax; b = ay
    //F = 1/2 (sqrt(a^2 t^2 + 2 a t x0 + b^2 t^2 + 2 b t y0 + x0^2 + y0^2) ((a x0 + b y0)/(a^2 + b^2) + t) + ((b x0 - a y0)^2 log(sqrt(a^2 + b^2) sqrt(a^2 t^2 + 2 a t x0 + b^2 t^2 + 2 b t y0 + x0^2 + y0^2) + a^2 t + a x0 + b^2 t + b y0))/(a^2 + b^2)^(3/2));

    double sq = sqrt((vx0+ax*dt)*(vx0+ax*dt) + (vy0+ay*dt)*(vy0+ay*dt));
    double sq0 = sqrt(vx0*vx0 + vy0*vy0);
    double aabb = ax*ax + ay*ay;
    double saabb = sqrt(aabb);
    double ax0by0 = ax*vx0 + ay*vy0;
    double bx0ay0 = ay*vx0 - ax*vy0;
    double bxpow = ((bx0ay0*bx0ay0)/pow(aabb, 1.5));
    double F_t = sq*(ax0by0/aabb + dt) + bxpow*log(saabb*sq + ax0by0 + aabb*dt + EPSILON);
    double F_0 = sq0*(ax0by0/aabb) + bxpow*log(saabb*sq0 + ax0by0 + EPSILON);

//    if (debug)
//    {
        if (F_t != F_t)
        {
            qDebug() << "F_t didn't work."
                     << "input:" << vx0 << vy0 << ax << ay << dt
                     << "sq:" << sq << "aabb:" << aabb << saabb << "ax0by0:" << ax0by0
                     << "log:" << log(saabb*sq + ax0by0 + aabb*dt + EPSILON) << saabb*sq + ax0by0 + aabb*dt + EPSILON << "F_t:" << 0.5*F_t;
            qDebug() << "ctrl:" << ctrl;
        }

        if (F_0 != F_0)
        {
            qDebug() << "F_0 didn't work."
                     << "input:" << vx0 << vy0 << ax << ay << dt
                     << "sq0:" << sq0 << "aabb:" << aabb << saabb << "ax0by0:" << ax0by0
                     << "log:" << log(saabb * sq0 + ax0by0 + EPSILON) << saabb * sq0 + ax0by0 + EPSILON << "F_0:" << 0.5*F_0;
            qDebug() << "ctrl:" << ctrl;
        }
//    }

    return 0.5*(F_t-F_0);
}

// Draws the trajectory described by the ctrl sequence onto the QPainter.
void BangBang2D::draw(QPainter *painter, const QPen &pen) const
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
        if (newState != oldState)
            painter->drawLine(oldState.pos(), newState.pos());
        oldState = newState;
    }

    // Draw the ctrl frames.
    painter->setBrush(pen.color());
    for (int i = 0; i < ctrl.size(); i++)
        painter->drawEllipse(ctrl[i].pos(), 0.03, 0.03);

    painter->restore();
}

QDebug operator<<(QDebug dbg, const BangBang2D &o)
{
    dbg << o.ctrl;
    return dbg;
}

QDebug operator<<(QDebug dbg, const BangBang2D* o)
{
    dbg << o->ctrl;
    return dbg;
}

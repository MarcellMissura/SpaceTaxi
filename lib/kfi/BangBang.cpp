#include "BangBang.h"

// This is a bang-bang control based non-linear keyframe interpolator that touches
// a sequence of given keyframes (x,v) with a continuous, acceleration and velocity
// bounded motion trajectory. To use the keyframe player, instantiate it, set the
// maximum acceleration setA(A) and maximum velocity setV(V) parameters and provide
// a set of keyframes using one of the addKeyframe() methods. Then you can obtain the
// motion control commands with getTimeOptimalControlSequence() and getTimedControlSequence()
// and use the evaluateAt(ctrl, dt) method to query the motion state at any time dt.
//
//	Example:
//
//	BangBang kp;
//	kp.setA(10.0); // acceleration bound
//	kp.setV(10.0); // optional velocity bound
//	kp.addKeyframe(0, 1.0, 2.0);
//	kp.addKeyframe(3.0, 4.0, 5.0);
//
//	// Evaluate at certain points in time.
//  Vector<Keyframe> ctrl = kp.getControlSequence();
//	for (double t = 0; t < kp.totalTime(); t = t+0.01)
//	{
//		Keyframe state = kp.evaluate(ctrl, t);
//		doSomethingWith(State);
//	}

// Instantiates a blank keyframe player with acc and vel bounds set to 0.
BangBang::BangBang()
{
    V = 0;
    A = 0;
}

// Sets the acceleration limit to A.
void BangBang::setA(double A)
{
    this->A = A;
}

// Sets the velocity limit to V.
// The velocity limit is optional. If it is not set, no velocity limit is applied.
void BangBang::setV(double V)
{
    this->V = V;
}

// Clears the keyframe player. It deletes all keyframes.
void BangBang::clear()
{
    keyframes.clear();
    ctrl.clear();
}

// Clears the keyframe player. It deletes all keyframes.
void BangBang::reset()
{
    clear();
}

// Appends a set of keyframes to the motion sequence.
void BangBang::addKeyframes(const Vector<Keyframe> &keyframes)
{
    for (int i = 0; i < keyframes.size(); i++)
        addKeyframe(keyframes[i]);
}

// Appends a keyframe to the motion sequence.
void BangBang::addKeyframe(const Keyframe &kf)
{
    keyframes << kf;
}

// Appends a keyframe to the motion sequence. The time parameter dt
// is interpreted as relative time with respect to the keyframe before.
void BangBang::addKeyframe(double t, double x, double v, double a)
{
    Keyframe kf(t, x, v, a);
    addKeyframe(kf);
}

// Returns a mutable reference to the keyframes inside the interpolator.
Vector<Keyframe> &BangBang::getKeyframes()
{
    return keyframes;
}

// From the given keyframes, a set of control frames are computed that encode the
// velocity and acceleration bounded, piecewise constant acceleration motion trajectory
// that touches the keyframes at the desired position with the right velocity at the right
// time, unless this would violate the velocity and acceleration bounds. The method expects
// a loaded set of keyframes with correct absolute times t. They keyframes are not rewritten.
const Vector<Keyframe>& BangBang::getTimedControlSequence()
{
    ctrl.clear();
    if (keyframes.isEmpty() || A <= 0)
        return ctrl;

    // Initialize the first ctrl frame with the current state.
    Keyframe kf = keyframes[0];
    ctrl << kf;
    ctrl[0].t = 0; // Just in case.

    // Out of velocity bounds correction.
    int idx = 0;
    if (V != 0 && kf.v > V)
    {
        double dt = min((kf.v-V)/A, (keyframes.last().t-keyframes[0].t));
        while (idx < keyframes.size()-1 && dt >= (keyframes[idx+1].t-keyframes[0].t))
            idx++;
        move(dt, -A);
    }
    else if (V != 0 && kf.v < -V)
    {
        double dt = min((kf.v+V)/A, (keyframes.last().t-keyframes[0].t));
        while (idx < keyframes.size()-1 && dt >= (keyframes[idx+1].t-keyframes[0].t))
            idx++;
        move(dt, A);
    }

    for (int i = idx; i < keyframes.size()-1; i++)
    {
        const Keyframe& kf0 = ctrl.last();
        Keyframe kf1 = keyframes[i+1];

        // For nice notation.
        double v0 = kf0.v;
        double v1 = kf1.v;
        double dt = kf1.t-keyframes[0].t-kf0.t;
        double dx = kf1.x-kf0.x;
        double sv = v0+v1;
        double svsq = (v0*v0+v1*v1);

        // Determine an initial unbounded acceleration and the sign of the approach (accelerate or brake).
        double sqrta = sqrt(4*(dx*dx-dt*dx*sv) + 2*dt*dt*svsq);
        double rest = 2*dx-dt*sv;
        double a1 = (sqrta+rest)/(dt*dt);
        double a2 = (-sqrta+rest)/(dt*dt);
        double a = fabs(a1) > fabs(a2) ? a1 : a2;

        // If a is near 0, we have a special coasting case where no control is required.
        if (fabs(a) < 1.0E-6)
        {
            move(dt, 0, i+1);
            continue;
        }

        // Apply the acceleration limit.
        a = qBound(-A, a, A);

        // Determine the bang times.
        double t1 = qMin(dt-sqrt((dt*v0-dx)/a + 0.5*dt*dt), dt);
        double t2 = dt-t1;

        // Check the velocity limit.
        double vt1 = v0+a*t1;
        if (V == 0 || (vt1 <= V && vt1 >= -V))
        {
            move(t1, a, i);
            move(t2, -a, i+1);
        }
        else
        {
            // The velocity limit is reached on the way. A section of the motion trajectory
            // needs to be replaced with v = VU (or v = VL) and a = 0.
            double V_ = vt1 > V ? V : -V;
            a = (2*V_*(V_-sv)+svsq)/(2*(dt*V_-dx));
            a = sgn(V_)*sgn(a)*a;
            a = qBound(-A, a, A);

            double dVv0 = V_-v0;

            double t1 = qBound(0.0, dVv0/a, dt);
            double t3 = qBound(0.0, sqrt( -dVv0*dVv0 - 2*a*(dx-dt*V_))/fabs(a), dt-t1);
            double t2 = dt-t1-t3;

            //qDebug() << "kf0:" << kf0 << "kf1:" << kf1;
            //qDebug() << "a:" << a << "t1:" << t1 << "t2:" << t2 << "t3:" << t3;

            move(t1, a, i);
            move(t2, 0, i);
            move(t3, -a, i+1);
        }
    }

    ctrl.last().dt = 0;
    ctrl.last().a = 0;

    return ctrl;
}

// Generates the piecewise constant acceleration motion control sequence that encodes the
// time optimal motion trajectory through the given keyframes. In case of the time optimal
// motion, the position and the velocity of each keyframe is always precisely met, even in
// overshoot and undershoot cases. The absolute and relative times of the loaded keyframes
// are rewritten to reflect the timing of the time optimal motion.
// Note that the acceleration limit A and the velocity limit V have to be set beforehand.
const Vector<Keyframe>& BangBang::getTimeOptimalControlSequence()
{
    //qDebug() << "BangBang::getTimeOptimalControlSequence()";
    //qDebug() << "keyframes:" << keyframes;

    ctrl.clear();
    if (keyframes.isEmpty() || A <= 0)
        return ctrl;

    keyframes[0].t = 0;
    Keyframe kf = keyframes[0];
    ctrl << kf;
    ctrl[0].t = 0; // Just in case.

    // Out of bounds correction.
    if (V != 0 && kf.v > V)
        move((kf.v-V)/A, -A);
    else if (V != 0 && kf.v < -V)
        move((kf.v+V)/A, A);

    //qDebug() << "oob:" << ctrl;

    for (int i = 0; i < keyframes.size()-1; i++)
    {
        const Keyframe& kf0 = ctrl.last();

        // For nice notation.
        double dx = keyframes[i+1].x-kf0.x;
        double v0 = kf0.v;
        double v1 = (V != 0) ? qBound(-V, keyframes[i+1].v, V) : keyframes[i+1].v;
        double v00 = v0*v0;
        double v11 = v1*v1;

        //qDebug() << "kf" << i << "dx:" << dx << "v0:" << v0 << "v1:" << v1;


        double sigma_d = sgn(dx);
        bool undershoot = (sigma_d*v1 > 0 && v11 > v00+2*A*fabs(dx) + EPSILON);
        bool overshoot = (sigma_d*v0 > 0 && v00 > v11+2*A*fabs(dx) + EPSILON);
        double sigma_h =  undershoot || overshoot ? -1 : 1;
        double sigma = sigma_d*sigma_h;
        double sA = sigma*A;

        double sq = sqrt((v00+v11)/2 + dx*sA);
        double t1 = (sigma*sq-v0)/sA;
        double t2 = (sigma*sq-v1)/sA;

        //qDebug() << sigma << "t:" << t1 << t2;

        // If the maximum velocity is reached on the way, replace the peak with a maximum velocity section.
        double vpeak = v0+sA*t1;
        if (V != 0 && (vpeak > V || vpeak < -V))
        {
            double V_ = sigma_d > 0 ? V : -V;
            double t1 = (V_-v0)/sA;
            double dx1 = (v0+0.5*sA*t1)*t1;
            double t3 = (V_-v1)/sA;
            double dx3 = (V_-0.5*sA*t3)*t3;
            double t2 = (dx-dx1-dx3)/V_;
            //qDebug() << "   " << t1 << t2 << t3 << "V:" << V << "v0:" << v0 << "v1:" << v1;

            move(t1, sA, i);
            move(t2, 0, i);
            move(t3, -sA, i+1);
        }
        else
        {
            move(t1, sA, i);
            move(t2, -sA, i+1);
        }

        // The optimal motion always hits the target precisely.
        // We can set the desired values here to clean the drift.
        ctrl.last().x = keyframes[i+1].x;
        ctrl.last().v = v1;

        // Update the keyframe times.
        keyframes[i+1].t = ctrl.last().t;
        keyframes[i].dt = keyframes[i+1].t-keyframes[i].t;
    }

    ctrl.last().dt = 0;
    ctrl.last().a = 0;

    //qDebug() << "ctrl:" << ctrl;

    return ctrl;
}

// Generates the piecewise constant acceleration motion control sequence that encodes the
// time optimal motion trajectory through the given keyframes. In this case of the time optimal
// motion, however, the position and the velocity of each keyframe are NOT always precisely met.
// In overshoot and undershoot cases, the first contact with the target position overwrites the
// target velocity. The absolute and relative times of the loaded keyframes are rewritten to
// reflect the timing of this motion. Note that the acceleration limit A and the velocity limit
// V have to be set beforehand.
const Vector<Keyframe>& BangBang::getTimeOptimalControlSequence2()
{
    //qDebug() << "BangBang::getTimeOptimalControlSequence()";
    //qDebug() << "keyframes:" << keyframes;

    ctrl.clear();
    if (keyframes.isEmpty() || A <= 0)
        return ctrl;

    keyframes[0].t = 0;
    Keyframe kf = keyframes[0];
    ctrl << kf;
    ctrl[0].t = 0; // Just in case.

    // Out of bounds correction.
    if (V != 0 && kf.v > V)
        move((kf.v-V)/A, -A);
    else if (V != 0 && kf.v < -V)
        move((kf.v+V)/A, A);

    for (int i = 0; i < keyframes.size()-1; i++)
    {
        const Keyframe& kf0 = ctrl.last();

        // For nice notation.
        double dx = keyframes[i+1].x-kf0.x;
        double v0 = kf0.v;
        double v1 = (V != 0) ? qBound(-V, keyframes[i+1].v, V) : keyframes[i+1].v;
        double v00 = v0*v0;
        double v11 = v1*v1;

        double sigma_d = sgn(dx);
        bool undershoot = (sigma_d*v1 > 0 && v11 > v00+2*A*fabs(dx));
        bool overshoot = (sigma_d*v0 > 0 && v00 > v11+2*A*fabs(dx));
        double sA = sigma_d*A;

        // Overshoot case.
        if (overshoot)
        {
            double t1 = (-sqrt(v00+2*-sA*dx)+v0) / sA;
            double t2 = (sqrt(v00+2*-sA*dx)+v0) / sA;
            double t = t1 >= 0 ? t1 : t2;
            move(t, -sA, i);
        }
        else if (undershoot)
        {
            double t = (sqrt(v00+2*sA*dx)-v0) / sA;
            move(t, sA, i);
        }
        else
        {
            double sigma_h = (undershoot || overshoot) ? -1 : 1;
            double sigma = sigma_d*sigma_h;
            sA = sigma_h*A;

            double sq = sqrt((v00+v11)/2 + dx*sA);
            double t1 = (sigma*sq-v0)/sA;
            double t2 = (sigma*sq-v1)/sA;

            //qDebug() << sigma << "t:" << t1 << t2;

            // If the maximum velocity is reached on the way, replace the peak with a maximum velocity section.
            double vpeak = v0+sA*t1;
            if (V != 0 && (vpeak > V || vpeak < -V))
            {
                double V_ = sigma_d > 0 ? V : -V;
                double t1 = (V_-v0)/sA;
                double dx1 = (v0+0.5*sA*t1)*t1;
                double t3 = (V_-v1)/sA;
                double dx3 = (V_-0.5*sA*t3)*t3;
                double t2 = (dx-dx1-dx3)/V_;
                //qDebug() << "   " << t1 << t2 << t3 << "V:" << V << "v0:" << v0 << "v1:" << v1;

                move(t1, sA, i);
                move(t2, 0, i);
                move(t3, -sA, i+1);
            }
            else
            {
                move(t1, sA, i);
                move(t2, -sA, i+1);
            }

            // The optimal motion always hits the target precisely.
            // We can set the desired values here to clean the drift.
            ctrl.last().x = keyframes[i+1].x;
            ctrl.last().v = v1;
        }

        // Update the keyframe times.
        keyframes[i+1].t = ctrl.last().t;
        keyframes[i].dt = keyframes[i+1].t-keyframes[i].t;
    }

    ctrl.last().dt = 0;
    ctrl.last().a = 0;

    //qDebug() << "ctrl:" << ctrl;

    return ctrl;
}

// Private method that updates the ctrl structure with a
// motion increment defined by acceleration a and time dt.
void BangBang::move(double dt, double a, int idx)
{
    // Skip irrelevant motion increments.
    if (dt < EPSILON)
        return;

    ctrl.last().dt = dt;
    ctrl.last().a = a;
    ctrl << ctrl.last();
    ctrl.last().forward();
    ctrl.last().idx = idx;
}

// Evaluates the given ctrl sequence at a given time dt relative to the first frame
// in ctrl and returns the calculated motion state.
Keyframe BangBang::evaluateAt(double dt) const
{
    //qDebug() << "   Keyframe BangBang::evaluateAt()" << dt << ctrl;
    if (ctrl.isEmpty())
        return Keyframe();

    int index = ctrl.size()-1;
    while (index > 0 && dt < ctrl[index].t-ctrl[0].t-EPSILON)
        index--;

    Keyframe kf = ctrl[index];
    kf.forward(min(dt-kf.t-ctrl[0].t, kf.dt));
    //qDebug() << "   " << index << dt << kf;
    return kf;
}

// Returns the total time of the motion.
double BangBang::getTotalTime() const
{
    return ctrl.last().t-ctrl[0].t;
}

// Generic QPainter paint code.
void BangBang::draw(QPainter& painter, const QPen& pen) const
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

QDebug operator<<(QDebug dbg, const BangBang &o)
{
    dbg << o.ctrl;
    return dbg;
}

QDebug operator<<(QDebug dbg, const BangBang* o)
{
    dbg << o->ctrl;
    return dbg;
}

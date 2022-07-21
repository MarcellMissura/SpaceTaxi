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
//	kp.setV(10.0); // velocity bound
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
    VU = 0;
    VL = 0;
    A = 0;
}

// Sets the acceleration limit to A.
void BangBang::setA(double A)
{
    this->A = A;
}

// Sets the upper and lower velocity limits to V.
void BangBang::setV(double V)
{
    this->VU = V;
    this->VL = -V;
}

// Sets the upper velocity limit to V.
void BangBang::setVU(double VU)
{
    this->VU = VU;
}

// Sets the lower velocity limit to V.
void BangBang::setVL(double VL)
{
    this->VL = VL;
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
void BangBang::addKeyframe(double dt, double x, double v, double a)
{
    Keyframe kf(dt, x, v, a);
    addKeyframe(kf);
}

// Internal method that updates the ctrl structure with a
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

    // Out of velocity bounds correction.
    int idx = 0;
    if (kf.v > VU)
    {
        double t = (kf.v-VU)/A;
        double dt = t-keyframes[0].dt;
        while (dt > 0)
        {
            idx++;
            dt -= keyframes[idx].dt;
        }
        move(t, -A);
    }
    else if (kf.v < VL)
    {
        double t = (VL-kf.v)/A;
        double dt = t-keyframes[0].dt;
        while (dt > 0)
        {
            idx++;
            dt -= keyframes[idx].dt;
        }
        move(t, A);
    }

    for (int i = idx; i < keyframes.size()-1; i++)
    {
        const Keyframe& kf0 = ctrl.last();
        Keyframe kf1 = keyframes[i+1];

        // For nice notation.
        double v0 = kf0.v;
        double v1 = kf1.v;
        double dt = kf1.t-kf0.t; // This requires that the absolute keyframe times are correct, but it has to be done this way because of the out of bounds correction.
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
        if (vt1 <= VU && vt1 >= VL)
        {
            move(t1, a, i);
            move(t2, -a, i+1);
        }
        else
        {
            // The velocity limit is reached on the way. A section of the motion trajectory
            // needs to be replaced with v = VU (or v = VL) and a = 0.
            double V = vt1 > VU ? VU : VL;
            a = (2*V*(V-sv)+svsq)/(2*(dt*V-dx));
            a = sgn(V)*sgn(a)*a;
            a = qBound(-A, a, A);

            double dVv0 = V-v0;

            double t1 = qBound(0.0, dVv0/a, dt);
            double t3 = qBound(0.0, sqrt( -dVv0*dVv0 - 2*a*(dx-dt*V))/fabs(a), dt-t1);
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
// Please note that the acceleration limit A has to be set beforehand.
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

    // Out of bounds correction.
    if (kf.v > VU)
        move((kf.v-VU)/A, -A);
    else if (kf.v < VL)
        move((VL-kf.v)/A, A);

    for (int i = 0; i < keyframes.size()-1; i++)
    {
        const Keyframe& kf0 = ctrl.last();

        // For nice notation.
        double dx = keyframes[i+1].x-kf0.x;
        double v0 = kf0.v;
        double v1 = qBound(VL, keyframes[i+1].v, VU);
        double v00 = v0*v0;
        double v11 = v1*v1;

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
        if (vpeak > VU || vpeak < VL)
        {
            double V = sigma_d > 0 ? VU : VL;
            double t1 = (V-v0)/sA;
            double dx1 = (v0+0.5*sA*t1)*t1;
            double t3 = (V-v1)/sA;
            double dx3 = (V-0.5*sA*t3)*t3;
            double t2 = (dx-dx1-dx3)/V;
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
// reflect the timing of this motion. Please note that the acceleration limit A and the velocity
// limit V have to be set beforehand.
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

    // Out of bounds correction.
    if (kf.v > VU)
        move((kf.v-VU)/A, -A);
    else if (kf.v < VL)
        move((VL-kf.v)/A, A);

    for (int i = 0; i < keyframes.size()-1; i++)
    {
        const Keyframe& kf0 = ctrl.last();

        // For nice notation.
        double dx = keyframes[i+1].x-kf0.x;
        double v0 = kf0.v;
        double v1 = qBound(VL, keyframes[i+1].v, VU);
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
            if (vpeak > VU || vpeak < VL)
            {
                double V = sigma_d > 0 ? VU : VL;
                double t1 = (V-v0)/sA;
                double dx1 = (v0+0.5*sA*t1)*t1;
                double t3 = (V-v1)/sA;
                double dx3 = (V-0.5*sA*t3)*t3;
                double t2 = (dx-dx1-dx3)/V;
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

// For a three keyframe situation, it computes the pass through velocity for the keyframe
// in the middle such that the total time of travel is minimized. This method uses a grid
// search method and is thus slow and has a discretization error.
void BangBang::minimizeTime()
{
    if (keyframes.size() < 3 || A <= 0)
        return;

    double mint = -1;
    double minv = VL;

    keyframes[1].v = minv;
    getTimeOptimalControlSequence();
    mint = ctrl.last().t;

    static const double step = 0.1;
    for (double v = VL+step; v <= VU; v = v+step)
    {
        keyframes[1].v = v;
        getTimeOptimalControlSequence();
        double t = ctrl.last().t;
        if (t < mint)
        {
            mint = t;
            minv = v;
        }
    }

    keyframes[1].v = minv;
}

// Computes the time optimal interception trajectory that starts with keyframe 0,
// touches the accelerating target in keyframe 1, and proceeds to keyframe 2.
void BangBang::intercept()
{
    if (keyframes.size() < 3)
        return;

    qDebug() << "BangBang::intercept()";
    qDebug() << "keyframes:" << keyframes;

    Keyframe x = keyframes[0];
    Keyframe y = keyframes[1];
    Keyframe z = keyframes[2];

    // Test for inclusion.
    BangBang bb;
    bb.setA(A);
    bb.setVU(VU);
    bb.setVL(VL);
    bb.addKeyframe(keyframes[0]);
    bb.addKeyframe(keyframes[2]);
    Vector<Keyframe> bbctrl = bb.getTimeOptimalControlSequence();
    Keyframe yy = y;
    for (int i = 0; i < bbctrl.size()-1; i++)
    {
        Keyframe xx = bbctrl[i];
        double dx = xx.x-yy.x;
        double dv = xx.v-yy.v;
        double da = xx.a-yy.a;
        double sq = sqrt((dv*dv)/(da*da)-2*dx/da);
        double ti1 = -sq-dv/da;
        double ti2 = sq-dv/da;
        double ti = (ti1 >= 0) ? ti1 : ti2;
        //qDebug() << "x:" << xx << "y:" << yy << "ti" << ti << ti1 << ti2;
        if (ti >= 0 && ti <= xx.dt)
        {
            qDebug() << "We have an inclusion at t:" << ti << "after" << i;
            //qDebug() << bbctrl;

            // Construct a ctrl that includes the interception point.
            ctrl.clear();
            for (int j=0; j < i; j++)
                ctrl << bbctrl[j];
            double ddt = xx.dt-ti;
            xx.dt = ti;
            ctrl << xx;
            xx.forward();
            xx.dt = ddt;
            ctrl << xx;
            for (int j=i+1; j < bbctrl.size(); j++)
                ctrl << bbctrl[j];

            //return;
        }

        yy.forward(xx.dt);
    }

    // No inclusion, too bad.


    // Compute the shortest possible interception time.
    x.a = A;
    double dx = x.x-y.x;
    double dv = x.v-y.v;
    double da = x.a-y.a;
    double sq = sqrt((dv*dv)/(da*da)-2*dx/da);
    double tp1 = -sq-dv/da;
    double tp2 = sq-dv/da;
    da = -x.a-y.a;
    sq = sqrt((dv*dv)/(da*da)-2*dx/da);
    double tn1 = -sq-dv/da;
    double tn2 = sq-dv/da;
    double tp = (tp1 >= 0) ? tp1 : tp2;
    double tn = (tn1 >= 0) ? tn1 : tn2;
    double minInterceptionTime = (tp >= 0 && (tn!=tn || tp < tn)) ? tp : tn;

    qDebug() << "No inclusion. Min interception time:" << minInterceptionTime << tp1 << tp2 << tn1 << tn2;
}

// Computes a threeway pass from the first keyframe over the second keyframe to the
// third keyframe. Yes, there has to be exactly three keyframes. Only the first keyframe
// is allowed to have nonzero velocity. On the upside, the optimal pass through velocity
// through the middle keyframe is computed automatically.
const Vector<Keyframe> &BangBang::threeWayPass()
{
    ctrl.clear();
    if (keyframes.size() < 3)
        return ctrl;

    //qDebug() << "BangBang::threeWayPass()";
    //qDebug() << "keyframes:" << keyframes;

    // We are looking for v1.
    double v1 = 0;

    Keyframe x = keyframes[0];
    Keyframe y = keyframes[1];
    Keyframe z = keyframes[2];

    int dxsign1 = sgn(y.x-x.x);
    double dx1 = fabs(y.x-x.x);
    int dxsign2 = sgn(z.x-y.x);
    double dx2 = fabs(z.x-y.x);

    double vplus1 = sqrt(x.v*x.v + 2*A*dx1);
    double vminus1 = qMax(0.0, sgn(x.v)*dxsign1*sqrt(x.v*x.v - 2*A*dx1));

    double vplus2 = dxsign1*dxsign2*sqrt(z.v*z.v + 2*A*dx2);
    double vminus2 = dxsign1*dxsign2*qMax(0.0, -sgn(z.v)*dxsign2*sqrt(z.v*z.v - 2*A*dx2));

    //qDebug() << "vplus1:" << vplus1 << "vminus1:" << vminus1 << "vplus2:" << vplus2 << "vminus2:" << vminus2;

    // Test for inclusion. Guaranteed optimal.
    if (vplus1 >= vminus2 && vplus1 <= vplus2)
    {
        v1 = dxsign1*vplus1;
        //qDebug() << "Inclusion. v1:" << v1;
    }
    else if (vplus2 >= vminus1 && vplus2 <= vplus1)
    {
        v1 = dxsign2*vplus2;
        //qDebug() << "Inclusion. v1:" << v1;
    }

    // Test for overshoot.
    else if (vminus1 > 0)
    {
        v1 = dxsign1*vminus1;
        //qDebug() << "Overshoot. v1:" << v1;
    }

    // Otherwise it's 0.
    else
    {
        v1 = 0;
        //qDebug() << "Handshake. v1:" << v1;
    }

    // Overwrite keyframe velocity.
    keyframes[1].v = v1;
    return getTimeOptimalControlSequence();
}

// Evaluates the given ctrl sequence at a given time dt relative to the first frame
// in ctrl and returns the calculated motion state.
Keyframe BangBang::evaluateAt(const Vector<Keyframe> &ctrl, double dt) const
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


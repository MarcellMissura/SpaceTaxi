#include "Keyframe2D.h"
#include <QDebug>

// The Keyframe object describes the state of a point mass in motion at a specific point in time.
// The Keyframe supports both a 2nd order (acceleration) context, where the motion state is
// (t, x, v) as in time, position, and velocity, and a 3rd order (jerk) context, where the motion
// state is (t, x, v, a) time, position, velocity, and acceleration. The time of a Keyframe is
// encoded in two ways in the members t and dt, which have the following meaning. t is the absolute
// time of the keyframe expressed in seconds, where absolute typically means with respect to the
// time of the first keyframe in a sequence that should have the time t=0. dt is a time interval,
// or a relative time, which is interpreted as a control instruction. In the acceleration context
// this means that the acceleration a should be applied for the duration of time dt to the state
// (t, x, v) of the keyframe. The execution of this instruction is implemented by the forwardA()
// method. In the jerk context, the instruction means the jerk j is applied for the duration of
// time dt to the state (t, x, v, a) of the keyframe. This is implemented by the forwardJ() method.
// In general, the Keyframe object is something that the specific implementations of the
// KeyframePlayer (Spline, Reflexxes, and BangBang) operate on. The parameterized constructor
// Keyframe(t, x, v, a, j) is designed for manual construction of keyframes and takes an *absolute*
// time as argument! Furthermore, the Keyframe object supports Qt debugging with qDebug():
//
// Keyframe kf;
// qDebug() << kf;


Keyframe2D::Keyframe2D()
{
    t = 0;
    dt = 0;
    l = 0;
	x = 0;
    y = 0;
	vx = 0;
    vy = 0;
	ax = 0;
	ay = 0;
    jx = 0;
    jy = 0;
}

Keyframe2D::Keyframe2D(double t, double x, double y, double vx, double vy, double ax, double ay, double jx, double jy)
{
    this->t = t;
    this->dt = 0;
	this->x = x;
    this->y = y;
	this->vx = vx;
    this->vy = vy;
	this->ax = 0;
	this->ay = 0;
    this->jx = 0;
    this->jy = 0;
}

// Sets the absolute time t of the keyframe;
void Keyframe2D::setT(double t)
{
    this->t = t;
}

// Sets the relative time dt of the keyframe.
void Keyframe2D::setDt(double dt)
{
    this->dt = dt;
}

// Sets the position.
void Keyframe2D::setPos(const Vec2 &v)
{
    x = v.x;
    y = v.y;
}

// Sets the velocity vector.
void Keyframe2D::setVel(const Vec2 &v)
{
    vx = v.x;
    vy = v.y;
}

// Sets the acceleration.
void Keyframe2D::setAcc(const Vec2 &a)
{
    ax = a.x;
    ay = a.y;
}

// Sets the jerk.
void Keyframe2D::setJerk(const Vec2 &j)
{
    jx = j.x;
    jy = j.y;
}


// Forwards the keyframe by the relative time (dt) in the keyframe using the jerk (j) in the keyframe.
void Keyframe2D::forward()
{
    forwardJ();
}

// Forwards the keyframe by the relative time (dt) in the keyframe using the jerk (j) in the keyframe.
void Keyframe2D::forwardJ()
{
    x += ((jx/6*dt+ax/2)*dt+vx)*dt;
    vx += (jx/2*dt+ax)*dt;
    ax += jx*dt;
    y += ((jy/6*dt+ay/2)*dt+vy)*dt;
    vy += (jy/2*dt+ay)*dt;
    ay += jy*dt;
    t += dt;
    dt = 0;
}

// Forwards the keyframe by the relative time (dt) in the keyframe using the acceleration (a)
// in the keyframe.
void Keyframe2D::forwardA()
{
    x += (vx+0.5*ax*dt)*dt;
    vx += ax*dt;
    y += (vy+0.5*ay*dt)*dt;
    vy += ay*dt;
    t += dt;
    dt = 0;
}

// Forwards the keyframe by the given relative time dt using the jerk (j) in the keyframe.
void Keyframe2D::forward(double dt)
{
    forwardJ(dt);
}

// Forwards the keyframe by the given relative time dt using the jerk (j) in the keyframe.
void Keyframe2D::forwardJ(double dt)
{
    x += ((jx/6*dt+ax/2)*dt+vx)*dt;
    vx += (jx/2*dt+ax)*dt;
    ax += jx*dt;
    y += ((jy/6*dt+ay/2)*dt+vy)*dt;
    vy += (jy/2*dt+ay)*dt;
    ay += jy*dt;
    t += dt;
    this->dt -= dt;
}

// Forwards the keyframe by the given relative time dt and the acceleration (a) in the keyframe.
void Keyframe2D::forwardA(double dt)
{
    x += (vx+0.5*ax*dt)*dt;
    vx += ax*dt;
    y += (vy+0.5*ay*dt)*dt;
    vy += ay*dt;
    t += dt;
    this->dt -= dt;
}

// Forwards the keyframe by the given relative time dt using the given jerk j.
void Keyframe2D::forward(double dt, const Vec2& j)
{
    forwardJ(dt, j);
}

// Forwards the keyframe by the given relative time dt using the given jerk j.
void Keyframe2D::forwardJ(double dt, const Vec2& j)
{
    x += ((j.x/6*dt+ax/2)*dt+vx)*dt;
    vx += (j.x/2*dt+ax)*dt;
    ax += j.x*dt;
    y += ((j.y/6*dt+ay/2)*dt+vy)*dt;
    vy += (j.y/2*dt+ay)*dt;
    ay += j.y*dt;
    t += dt;
    this->dt -= dt;
}

// Forwards the keyframe by the given relative time dt using the given acceleration a.
void Keyframe2D::forwardA(double dt, const Vec2& a)
{
    x += (vx+0.5*a.x*dt)*dt;
    vx += a.x*dt;
    y += (vy+0.5*a.y*dt)*dt;
    vy += a.y*dt;
    t += dt;
    this->dt -= dt;
}

QDebug operator<<(QDebug dbg, const Keyframe2D &v)
{
    bool ais = dbg.autoInsertSpaces();
    dbg.nospace();
    dbg << "(t:" << v.t
        << " dt:" << v.dt
        << " l:" << v.l
        << " p:" << v.pos()
        << " v:" << v.vel()
        << " a:" << v.acc()
        //<< " j:" << v.jerk()
        << ") ";
    dbg.setAutoInsertSpaces(ais);
    return dbg;
}

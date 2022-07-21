#include "Keyframe2D.h"
#include <QDebug>

Keyframe2D::Keyframe2D()
{
	t = 0;
    dt = 0;
	x = 0;
	vx = 0;
	ax = 0;
	y = 0;
	vy = 0;
	ay = 0;

    idx = 0;
}

void Keyframe2D::reset()
{
    t = 0;
    dt = 0;
    x = 0;
    vx = 0;
    ax = 0;
    y = 0;
    vy = 0;
    ay = 0;

    idx = 0;
}

Keyframe2D::Keyframe2D(double dt, double x, double vx, double y, double vy)
{
    t = 0;
    this->dt = dt;
	this->x = x;
	this->vx = vx;
	this->ax = 0;
	this->y = y;
	this->vy = vy;
	this->ay = 0;
    idx = 0;
}

void Keyframe2D::set(double dt, double x, double vx, double y, double vy)
{
    t = 0;
    this->dt = dt;
	this->x = x;
	this->vx = vx;
	this->ax = 0;
	this->y = y;
	this->vy = vy;
	this->ay = 0;
    idx = 0;
}

// Forwards the keyframe using the time and the acceleration in the keyframe.
void Keyframe2D::forward()
{
    x += (vx+0.5*ax*dt)*dt;
    vx += ax*dt;
    y += (vy+0.5*ay*dt)*dt;
    vy += ay*dt;
    t += dt;
}

// Forwards the keyframe by the time dt using the acceleration in the keyframe.
void Keyframe2D::forward(double dt)
{
    x += (vx+0.5*ax*dt)*dt;
    vx += ax*dt;
    y += (vy+0.5*ay*dt)*dt;
    vy += ay*dt;
    t += dt;
}


// Forwards the keyframe by the time dt and the accelerations ax and ay.
void Keyframe2D::forward(double dt, double ax, double ay)
{
    x += (vx+0.5*ax*dt)*dt;
    vx += ax*dt;
    y += (vy+0.5*ay*dt)*dt;
    vy += ay*dt;
    t += dt;
}


// Forwards the keyframe using the time and the acceleration in the keyframe.
Keyframe2D Keyframe2D::forwarded()
{
    Keyframe2D kf = *this;
    kf.forward();
    return kf;
}

// Forwards the keyframe by the time dt using the acceleration in the keyframe.
Keyframe2D Keyframe2D::forwarded(double dt)
{
    Keyframe2D kf = *this;
    kf.forward(dt);
    return kf;
}


// Forwards the keyframe by the time dt and the accelerations ax and ay.
Keyframe2D Keyframe2D::forwarded(double dt, double ax, double ay)
{
    Keyframe2D kf = *this;
    kf.forward(dt, ax, ay);
    return kf;
}


Keyframe2D operator*(double scalar, const Keyframe2D& v)
{
    return Keyframe2D(scalar*v.t, scalar*v.x, scalar*v.vx, scalar*v.y, scalar*v.vy);
}
Keyframe2D operator*(const Keyframe2D& v, const double scalar)
{
    return Keyframe2D(scalar*v.t, scalar*v.x, scalar*v.vx, scalar*v.y, scalar*v.vy);
}

Keyframe2D operator/(const Keyframe2D& v, const double scalar)
{
    return Keyframe2D(v.t/scalar, v.x/scalar, v.vx/scalar, v.y/scalar, v.vy/scalar);
}

Keyframe2D operator/(const double scalar, const Keyframe2D& v)
{
    return Keyframe2D(v.t/scalar, v.x/scalar, v.vx/scalar, v.y/scalar, v.vy/scalar);
}

QDebug operator<<(QDebug dbg, const Keyframe2D &v)
{
    bool ais = dbg.autoInsertSpaces();
    dbg.nospace();
    dbg << "(idx:" << v.idx
        << " t:" << v.t
        << " dt:" << v.dt
        << " p:" << v.pos()
        << " v:" << v.vel()
        << " a:" << v.acc() << ") ";
    dbg.setAutoInsertSpaces(ais);
    return dbg;
}

QDataStream& operator<<(QDataStream& out, const Keyframe2D &o)
{
    out << o.t;
    out << o.dt;
    out << o.x;
    out << o.vx;
    out << o.ax;
    out << o.y;
    out << o.vy;
    out << o.ay;
    return out;
}

QDataStream& operator>>(QDataStream& in, Keyframe2D &o)
{
    in >> o.t;
    in >> o.dt;
    in >> o.x;
    in >> o.vx;
    in >> o.ax;
    in >> o.y;
    in >> o.vy;
    in >> o.ay;
    return in;
}

#include "Keyframe.h"

Keyframe::Keyframe()
{
	t = 0;
    dt = 0;
	x = 0;
	v = 0;
	a = 0;
    idx = 0;
}

Keyframe::Keyframe(double dt, double x, double v, double a)
{
    t = 0;
    this->dt = dt;
	this->x = x;
	this->v = v;
    this->a = a;
    idx = 0;
}

void Keyframe::set(double dt, double x, double v, double a)
{
    t = 0;
    this->dt = dt;
	this->t = t;
	this->x = x;
	this->v = v;
    this->a = a;
}

// Forwards the keyframe using the time and the acceleration in the keyframe.
void Keyframe::forward()
{
    x += (v+0.5*a*dt)*dt;
    v += a*dt;
    t += dt;
}

// Forwards the keyframe by the time dt using the acceleration in the keyframe.
void Keyframe::forward(double dt)
{
    x += (v+0.5*a*dt)*dt;
    v += a*dt;
    t += dt;
}

// Forwards the keyframe by the time dt and the acceleration a.
void Keyframe::forward(double dt, double a)
{
    x += (v+0.5*a*dt)*dt;
    v += a*dt;
    t += dt;
}

// Gui support methods to drag and drop a keyframe.
Vec2 Keyframe::pos()
{
    return Vec2(t,x);
}
void Keyframe::setPos(Vec2 v)
{
	t = v.x;
	x = v.y;
}
void Keyframe::moveBy(Vec2 v)
{
	t += v.x;
	x += v.y;
}

QDebug operator<<(QDebug dbg, const Keyframe &k)
{
    dbg.nospace() << "(t:" << k.t << ", dt:" << k.dt << ", x:" << k.x << ", v:" << k.v << ", a:" << k.a << ")";
	return dbg.space();
}

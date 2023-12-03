#include "Keyframe.h"

Keyframe::Keyframe()
{
	t = 0;
    dt = 0;
	x = 0;
	v = 0;
	a = 0;
    j = 0;
    idx = 0;
}

Keyframe::Keyframe(double t, double x, double v, double a)
{
    this->t = t;
    this->dt = 0;
	this->x = x;
	this->v = v;
    this->a = a;
    j = 0;
    idx = 0;
}

void Keyframe::set(double t, double x, double v, double a)
{
    this->t = t;
    this->dt = 0;
	this->x = x;
	this->v = v;
    this->a = a;
    j = 0;
}

// Forwards the keyframe using the time and the acceleration in the keyframe.
void Keyframe::forward()
{
    x += ((j/6*dt+a/2)*dt+v)*dt;
    v += (j/2*dt+a)*dt;
    a += j*dt;
    t += dt;
    dt = 0;
}

// Forwards the keyframe by the time dt using the acceleration in the keyframe.
void Keyframe::forward(double dt)
{
    x += ((j/6*dt+a/2)*dt+v)*dt;
    v += (j/2*dt+a)*dt;
    a += j*dt;
    t += dt;
    dt = 0;
}

// Forwards the keyframe by the time dt and the acceleration a.
void Keyframe::forward(double dt, double a)
{
    x += ((j/6*dt+a/2)*dt+v)*dt;
    v += (j/2*dt+a)*dt;
    a += j*dt;
    t += dt;
    dt = 0;
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
    //dbg.nospace() << "(t:" << k.t << ", dt:" << k.dt << ", x:" << k.x << ", v:" << k.v << ", a:" << k.a << ", j:" << k.j << ")";
    dbg.nospace() << "(t:" << k.t << ", dt:" << k.dt << ", x:" << k.x << ", v:" << k.v << ", a:" << k.a << ")";
	return dbg.space();
}

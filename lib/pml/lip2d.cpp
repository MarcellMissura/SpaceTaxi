#include "lip2d.h"
#include "lip.h"

Lip2D::Lip2D()
{
    x = 0;
    vx = 0;
    y = 0;
    vy = 0;
    zmpx = 0;
    zmpy = 0;
}

Lip2D::Lip2D(double x, double vx, double y, double vy)
{
    this->x = x;
    this->vx = vx;
    this->y = y;
    this->vy = vy;
}

void Lip2D::set(double x, double vx, double y, double vy)
{
    this->x = x;
    this->vx = vx;
    this->y = y;
    this->vy = vy;
}


// Simulates the pendulum model for dt amount of time. No step is induced.
void Lip2D::predict(double dt)
{
    Lip lip;
    lip.set(x-zmpx, vx, C); // update with zmp
    lip.predict(dt);
    x = lip.x-zmpx; // new location without zmp!
    vx = lip.vx;

    lip.set(y-zmpy, vy, C); // update with zmp
    lip.predict(dt);
    y = lip.x-zmpy; // new location without zmp!
    vy = lip.vx;
}

// Returns a copy at time t relative to the current state. No step is induced.
Lip2D Lip2D::predicted(double t)
{
    Lip2D lm = *this;
    lm.predict(t);
    return lm;
}

void Lip2D::setZmp(double zx, double zy)
{
    zmpx = zx;
    zmpy = zy;
}

Lip2D operator*(double scalar, const Lip2D& v)
{
    Lip2D lm;
	lm = v;
	lm.x = scalar * v.x;
	lm.vx = scalar * v.vx;
	lm.y = scalar * v.y;
	lm.vy = scalar * v.vy;
	return lm;
}

Lip2D operator*(const Lip2D& v, const double scalar)
{
	return operator*(scalar, v);
}

QDebug operator<<(QDebug dbg, const Lip2D &o)
{
    dbg << "x:" << o.x << "y:" << o.y << "vx:" << o.vx << "vy:" << o.vy;
    return dbg;
}

#include "hpm.h"
#include <math.h>
#include "globals.h"

// This is a one dimensional holonomic point mass obeying simple Newtonian constant velocity physics.

Hpm::Hpm()
{
    x = 0;
	v = 0;
    a = 0;
}

Hpm::Hpm(double x, double v, double a)
{
    this->x = x;
	this->v = v;
    this->a = a;
}

// Resets the state to 0.
void Hpm::reset()
{
    x = 0;
    v = 0;
    a = 0;
}

// Sets the pendulum into a given state (x, vc) leaving the gravitational constant C unchanged.
void Hpm::set(double x, double v, double a)
{
	this->x = x;
	this->v = v;
    this->a = a;
}

// Calculates a future state in time seconds using Euler integration.
void Hpm::simulate(double a, double time)
{
	double timestep = 0.00001;
    double dt = 0;
    while (time-dt > timestep)
	{
		x += timestep*v;
		v += timestep*a;
        dt += timestep;
	}

    timestep = (time-dt);
	x += timestep*v;
	v += timestep*a;
}

// Calculates a future state using the analytical solution of the equations of motion.
void Hpm::predict(double a, double dt)
{
    x = (0.5*a*dt+v)*dt+x;
    v = a*dt+v;
}

// Returns a predicted state at time t in the future.
Hpm Hpm::predicted(double a, double dt)
{
	Hpm l(x, v);
    l.predict(a, dt);
	return l;
}

Hpm operator*(double scalar, const Hpm& l)
{
    return Hpm(scalar*l.x, scalar*l.v);
}

Hpm operator*(const Hpm& l, const double scalar)
{
    return Hpm(scalar*l.x, scalar*l.v);
}

Hpm operator/(double scalar, const Hpm& l)
{
    return Hpm(l.x/scalar, l.v/scalar);
}

Hpm operator/(const Hpm& l, const double scalar)
{
    return Hpm(l.x/scalar, l.v/scalar);
}

QDebug operator<<(QDebug dbg, const Hpm &v)
{
    dbg << "[" << v.x << ", " << v.v << "]";
    return dbg;
}

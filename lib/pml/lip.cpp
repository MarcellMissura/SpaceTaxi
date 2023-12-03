#include "lip.h"
#include "lib/globals.h"
#include <math.h>

// This is a one dimensional linear inverted pendulum model.

Lip::Lip()
{
	C = 10;
	x = 0;
	vx = 0;
}

Lip::Lip(double x, double vx, double C)
{
	this->x = x;
	this->vx = vx;
	this->C = C;
}

// Resets the state to 0.
void Lip::reset()
{
	x = 0.0;
	vx = 0.0;
}

// Sets the pendulum into a given state (x, vc) leaving the gravitational constant C unchanged.
void Lip::set(double x, double vx)
{
	this->x = x;
	this->vx = vx;
}

// Sets the pendulum into a given state (x, vx) with a gravitational constant C.
void Lip::set(double x, double vx, double C)
{
	this->x = x;
	this->vx = vx;
	this->C = C;
}

// Calculates a future state of the limp in t seconds from now using Euler
// integration of the differential equations of the Linear Inverted Pendulum
// Model. The future state is written into the state variables x and vx.
void Lip::simulate(double time)
{
    double ax;
    double dt = 0.000001;
    double t = 0.0;
    while (time-t > dt)
    {
        ax = C*fsin(x);
        vx += dt*ax;
        x += dt*vx;
        t += dt;
    }
}

// Calculates a future state of the limp using the analytical solution
// of the Linear Inverted Pendulum Model. The given time parameter should
// be relative to now. The future state will be written into the state
// variables x and vx. Negative times are allowed.
void Lip::predict(double t)
{
	double sC = sqrt(C);
	double c1 = 0.5*(x + vx/sC);
	double c2 = 0.5*(x - vx/sC);
	x = c1*exp(sC*t) + c2*exp(-sC*t);
	vx = c1*sC*exp(sC*t) - c2*sC*exp(-sC*t);
}

// Returns a predicted pendulum state at time t in the future.
Lip Lip::predicted(double t) const
{
	Lip l(x, vx, C);
	l.predict(t);
	return l;
}

// Returns the current orbital energy of the pendulum.
double Lip::energy() const
{
	return 0.5*(vx*vx - C*x*x);
}

// Returns the orbital energy for the provided pendulum parameters.
double Lip::energy(double xx, double vxx, double CC)
{
	return 0.5*(vxx*vxx - CC*xx*xx);
}

// Given the current state of the limp and assuming t = 0, at what time t
// is the limp going to reach location x? Please note that there can be 0
// (the queried state is never reached), 1 (the queried state is reached
// exactly once), or 2 (the queried state is reached twice) solutions.
// Also, any number of the possible solutions can be negative, which means
// that the queried location has been reached once or twice in the past.
// In case there are no solutions, the function will return NaN.
// Otherwise the function returns either the one solution found, or the
// greater of two possible solutions, which can still be negative.
double Lip::tLoc(double xt) const
{
    // Note to self:
    // To obtain the smaller solution, -sqrt() has to be used.

    double sC = sqrt(C);
    double c1 = (x+vx/sC);
    double c2 = (x-vx/sC);
    double t = log(xt/c1 + sqrt((xt*xt)/(c1*c1) - c2/c1)) / sC;
    return t;
}


// Given the current state of the limp and assuming t = 0, at what time t
// is the limp going to reach velocity vx? Please note that there can be 0
// (the queried state is never reached), 1 (the queried state is reached
// exactly once), or 2 (the queried state is reached twice) solutions.
// Also, any number of the possible solutions can be negative, which means
// that the queried velocity has been reached once or twice in the past.
// In case there are no solutions, the function will return NaN.
// Otherwise the function returns either the one solution found, or the
// greater of two possible solutions, which can still be negative.
double Lip::tVel(double vx) const
{
    // Note to self:
    // To obtain the smaller solution, -sqrt() has to be used.

    double sC = sqrt(C);
    double c1 = (x + vx/sC);
    double c2 = (x - vx/sC);
    double t = log(vx/(c1*sC) + sqrt( (vx*vx)/(c1*c1*C) + c2/c1)) / sC;
    return t;
}

Lip operator*(double scalar, const Lip& l)
{
	return Lip(scalar * l.x, scalar * l.vx, l.C);
}

Lip operator*(const Lip& l, const double scalar)
{
	return Lip(scalar * l.x, scalar * l.vx, l.C);
}

Lip operator/(double scalar, const Lip& l)
{
	return Lip(l.x / scalar, l.vx / scalar, l.C);
}

Lip operator/(const Lip& l, const double scalar)
{
	return Lip(l.x / scalar, l.vx / scalar, l.C);
}

QDebug operator<<(QDebug dbg, const Lip &o)
{
    dbg << "x:" << o.x << "vx:" << o.vx;
    return dbg;
}

#include "poc.h"
#include <math.h>
#include <QDebug>
#include <complex>
#include "globals.h"

// The poc is a pole on cart model.
// The state of the poc is the position of the cart x, the cart velocity vx,
// the angle of the pendulum phi, the angular velocity of the pendulum vphi,
// and the angular acceleration of the pendulum aphi.
// The control input is the acceleration of the cart ax. The cart acceleration
// has an influence on the angular momentum of the pendulum.

Poc::Poc()
{
    phi = 0;
    vphi = 0;
    aphi = 0;
    x = 0;
    vx = 0;

    L = 1.0;
}

Poc::Poc(double phi, double vphi, double x, double vx)
{
    this->phi = phi;
    this->vphi = vphi;
    this->x = x;
    this->vx = vx;

    L = 1.0;
}

// Resets the state to 0.
void Poc::reset()
{
    phi = 0;
    vphi = 0;
    aphi = 0;
    x = 0;
    vx = 0;
}

// Sets the poc into a given state.
void Poc::set(double phi, double vphi, double x, double vx)
{
    this->phi = phi;
    this->vphi = vphi;
    this->x = x;
    this->vx = vx;
}

// Sets the length of the pendulum on the cart.
void Poc::setL(double L)
{
    this->L = L;
}

// Computes a future state using Euler integration of the precise, non-linear equation of motion.
// This is an accurate, but slow method.
void Poc::simulate(double time, double ax)
{
    double dt = 0.000001;
    int steps = time*1000000;
    double gl = G/L;
    for (int i = 0; i < steps; i++)
    {
        aphi = gl*sin(phi) + (ax/L)*cos(phi);
        vphi += dt*aphi;
        phi += dt*vphi;
        x += dt*vx;
        vx += dt*ax;
    }
}

// Computes a future state using Euler integration of the precise, non-linear equation of motion.
// This is an accurate, but slow method.
void Poc::simulate2(double time, double ax)
{
    double dt = 0.01;
    int steps = time*100;
    double gl = G/L;
    for (int i = 0; i < steps; i++)
    {
        aphi = gl*sin(phi) + (ax/L)*cos(phi);
        vphi += dt*aphi;
        phi += dt*vphi;
        x += dt*vx;
        vx += dt*ax;
    }
}

// Predicts a future state based on the linearized system equations.
void Poc::predictTaylor(double t, double ax)
{
    // The equation of motion for the poc is a(phi) = (G/L)*sin(phi) + (ax/L)*cos(phi).
    // Here, we use a Taylor Expansion-based approach where we approximate a(phi) = a(phi0) + a'(phi0)*(phi-phi0)
    // a(phi) = (G/L)*sin(phi0) + (ax/L)*cos(phi0) + ((G/L)*cos(phi) - (ax/L)*sin(phi)) * (phi - phi0).
    // We fold all constants into the parameters a and b such that aphi = a*phi + b and obtain:

    double cp = fcos(phi);
    double sp = fsin(phi);
    double gl = G/L;
    double axl = ax/L;
    double a = gl * cp - axl * sp;
    double b = gl * (sp - cp * phi) + axl * (cp + sp * phi);

    double k = b/a;
    std::complex<double> sa = std::sqrt(std::complex<double>(a,0));
    std::complex<double> c1 = 0.5*(phi + vphi/sa + k);
    std::complex<double> c2 = 0.5*(phi - vphi/sa + k);
    std::complex<double> img1 = c1*exp(sa*t) + c2*exp(-sa*t) - k;
    std::complex<double> img2 = sa*(c1*exp(sa*t) - c2*exp(-sa*t));

    phi = img1.real();
    vphi = img2.real();
    x = 0.5*ax*t*t + vx*t + x;
    vx = ax*t + vx;
}


// Predicts a future state based on linearized system equations.
void Poc::predictNaive(double t, double ax)
{
    // The equation of motion for the poc is aphi = (G/L)*sin(phi) + (ax/L)*cos(phi).
    // In this approach, we replace sin(phi) with phi and cos(phi) with (1-2*phi/pi)
    // and obtain a linearized version for aphi:
    // aphi = (G/L)*phi - (ax/L)*(1 - 2*phi/pi).
    // We fold all constants into the parameters a and b such that
    // aphi = a*phi + b.
    // Then, we can use the standard exponential form for solving the ODE.

    double a = G/L - (2.0*ax)/(L*PI);
    if (phi < 0)
        a = G/L + (2.0*ax)/(L*PI);
    double b = ax/L;

    double k = b/a;
    std::complex<double> sa = std::sqrt(std::complex<double>(a,0));
    std::complex<double> c1 = 0.5*(phi + vphi/sa + k);
    std::complex<double> c2 = 0.5*(phi - vphi/sa + k);
    std::complex<double> img1 = c1*exp(sa*t) + c2*exp(-sa*t) - k;
    std::complex<double> img2 = c1*sa*exp(sa*t) - c2*sa*exp(-sa*t);

    phi = img1.real();
    vphi = img2.real();
    x = 0.5*ax*t*t + vx*t + x;
    vx = ax*t + vx;
}

QDebug operator<<(QDebug dbg, const Poc &p)
{
    dbg << "[" << p.x << ", " << p.vx << ", " << p.phi << ", " << p.vphi << "]";
    return dbg;
}

#ifndef GLOBS_H_
#define GLOBS_H_
#include <cmath>
#include <iostream>

const double G = 9.81;
const double PI = 3.1415926535897932384626433832795;
const double PII = 2.0*PI;
const double PI2 = 1.5707963267948965579989817342721; // pi half
const double PI4 = 0.78539816339744830961566084581988; // pi quarter
const double PI23 = 2.094395102; // two thirds pi
const double PI34 = 3.0*PI/4.0; // three quaters pi
const double PI32 = 4.71238898038468985769396507492; // three half pi
const double SPI = 1.7724538509055160272981674833411; // sqrt of pi
const double RAD_TO_DEG =  180.0 / PI;
const double DEG_TO_RAD =  PI / 180.0;
const double EPSILON = 1.0E-5;

// Checks if number is (almost) zero.
static bool isnull(double n) { return (n < EPSILON && n > -EPSILON); }

// Min max bound trio.
template <typename T>
static const T &min(const T &a, const T &b) { return (a < b) ? a : b; }
template <typename T>
static const T &max(const T &a, const T &b) { return (a < b) ? b : a; }
template <typename T>
static const T &min(const T &a, const T &b, const T &c) { return min(min(a,b),c); }
template <typename T>
static const T &max(const T &a, const T &b, const T &c) { return max(max(a,b),c); }
template <typename T>
static const T &bound(const T &l, const T &val, const T &u){ return max(l, min(u, val)); }

// Returns the sign of the argument. +1 if the argument is positive,
// -1 if the argument is negative, and 0 if the argument is zero.
template <typename T>
static int sgn0(const T a) { return (fabs(a) < EPSILON ? 0 : a < 0 ? -1 : 1); }

// Returns the sign of the argument. +1 if the argument is positive,
// -1 if the argument is negative.
template <typename T>
static int sgn(const T a) { return (a < 0 ? -1 : 1); }

// Maps the argument to an angle expressed in radians between -PI and PI.
static double picut(double x) { return  x < 0 ? fmod(x-PI, PII)+PI : fmod(x+PI, PII)-PI;}

// A possibly faster version of picut, but the runtime grows with the input.
static double fpicut(double x)
{
    while (x > PI) x -= PII;
    while (x < -PI) x += PII;
    return x;
}

// An even faster picut version that assumes the angle to be given in [-PII, PII].
static double ffpicut(double x) { return  x > PI ? x-PII : x < -PI ? x+PII : x;}

// Maps the argument to an angle expressed in radians between 0 and 2*PI.
static double pi2cut(double x)
{
    while (x < 0) x += PII;
    while (x > PII) x -= PII;
    return x;
}

// A half picut version that maps to an angle range between [-PI/2, PI/2] and assumes
// the angle to be given in [-PI, PI]. This function makes most sense for an angle between lines.
static double pihalfcut(double x) { return  x > PI2 ? x-PI : x < -PI2 ? x+PI : x;}

// Returns an interpolated angle between from and to by an amount of frac.
// The resulting angle is from + frac*(to-from) interpolated along the smaller portion
// of the circle and mapped to the range [-PI, PI].
static double interpolateAngle(double from, double to, double frac)
{
    double pto = fpicut(to);
    double pfrom = fpicut(from);
    double diff = pto - pfrom;
    if (diff > PI) diff = diff-PII;
    else if (diff < -PI) diff = diff+PII;
    return ffpicut(pfrom + frac*diff);
}

// A better version of modulo (x%y) that wraps rather than returning a number < 0.
// E.g., mod(-3, 10) is 7 and not -3, mod(-17, 10) is 3 and not -7.
static int mod(int x, int y) { return y == 0 ? abs(x) : (x >= 0 ? x%y : ((x%y)+y)%y);}

// This is a 20x faster sine implementation with just a tiny error.
static double fsin(double x)
{
    // This algorithm is taken from http://mooooo.ooo/chebyshev-sine-approximation/
    // "Approximating sin(x) to 5 ULP with Chebyshev polynomials" by Collin Wallace

    x = fpicut(x);

    static double coeffs[6] = {
        -0.10132118,
         0.0066208798,        // x^3
        -0.00017350505,       // x^5
         0.0000025222919,     // x^7
        -0.000000023317787,   // x^9
         0.00000000013291342, // x^11
    };

    double pi_major = 3.1415927;
    double pi_minor = -0.00000008742278;
    double x2 = x*x;
    double p11 = coeffs[5];
    double p9  = p11*x2 + coeffs[4];
    double p7  = p9*x2  + coeffs[3];
    double p5  = p7*x2  + coeffs[2];
    double p3  = p5*x2  + coeffs[1];
    double p1  = p3*x2  + coeffs[0];
    return (x - pi_major - pi_minor) * (x + pi_major + pi_minor) * p1 * x;
}

// This is a 40x faster sine implementation with just a small error.
static double fsin2(double x)
{
    // This algorithm is based on the Bhaskara I sine approximation.
    // https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula

    x = fpicut(x);
    return (16.0*x*(PI-x))/(5.0*PI*PI - 4.0*x*(PI-x));
}

// This is a 20x faster cosine implementation with just a tiny error.
static double fcos(double x)
{
    return fsin(x+PI2);
}

// A twice as fast version of atan2 but with a small error.
static double fatan2(double y, double x)
{
    // This formula is taken from http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
    // Sreeraman Rajan et al., Efficient Approximations for the Arctangent Function

    // Special cases.
    if (x == 0 && y == 0)
        return 0;
    if (y == 0)
    {
        if (x > 0)
            return 0;
        return PI;
    }
    if (x == 0)
        return sgn(y)*PI2;

    // Octants 1 and 8.
    if (x > 0 && fabs(y) <= x)
    {
        double x_ = y/x;
        return x_*(1.0584 - sgn(y)*0.273*x_);
    }

    // Octants 2 and 3.
    if (y > 0 && y >= fabs(x))
    {
        double y_ = x/y;
        return PI2-y_*(1.0584-sgn(x)*0.273*y_);
    }

    // Octants 4 and 5.
    if (x < 0 && fabs(y) <= -x)
    {
        double x_ = y/x;
        return sgn(y)*PI+x_*(1.0584 + sgn(y)*0.273*x_);
    }

    // Octants 6 and 7.
    if (y < 0 && fabs(x) <= -y)
    {
        double y_ = x/y;
        return -PI2-y_*(1.0584+sgn(x)*0.273*y_);
    }

    std::cout << "Unhandled case in fatan2! x:" << x << " y:" << y
              << " " << (y < 0) << " " << (fabs(x) <= -y) << std::endl;
    return 0;
}

// An unsuccessful approximation of the exp function.
static double fexp(double x)
{
    x = 1.0 + x / 1024;
    x *= x; x *= x; x *= x; x *= x;
    x *= x; x *= x; x *= x; x *= x;
    x *= x; x *= x;
    return x;
}


#endif /* GLOBS_H_ */

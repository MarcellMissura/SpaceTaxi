#ifndef VECN_H_
#define VECN_H_
#include <math.h>
#include <QDebug>
#include <QDataStream>
#include "globals.h"

// The VecN is a generic vector of doubles where the number of dimensions N is specified
// as a template parameter. For example, VecN<3> v; creates a three dimensional vector.
// Literal references v.x, v.y, and v.z are available to access the first three elements
// of the vector (if available). Please refer to the inline documentation of the functions
// for more information.

template <int N=3>
class VecN
{
    double v[N];
    
public:
    
    // .x, .y amd .z element access operators for convenience.
    double& x;
    double& y;
    double& z;
    double& w; // also w for the fourth element when using an NVec<4> as a quaternion.
    
    VecN() : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memset(v, 0, N*sizeof(double));
    }

    VecN(const VecN<N> &o) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        if (this != &o)
            memcpy(v, &o, N*sizeof(double));
    }
    
    VecN(const VecN<N-1> &o, double xo) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memcpy(v, &o, (N-1)*sizeof(double));
        v[N-1] = xo;
    }
    
    VecN(double xo, const VecN<N-1> &o) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        v[0] = xo;
        memcpy(v+1, &o, (N-1)*sizeof(double));
    }
    
    VecN(const VecN<N+1> &o) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memcpy(v, &o, N*sizeof(double));
    }
    
    VecN(const double* xo) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memcpy(v, xo, N*sizeof(double));
    }
    
    VecN(double xo) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        for (int i = 0; i < N; i++)
            v[i] = xo;
    }
    
    VecN(const double* xo, double yo) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memcpy(v, xo, (N-1)*sizeof(double));
        v[N-1] = yo;
    }
    
    VecN(double xo, double yo, double zo=0, double wo=0, double ao=0, double bo=0, double co=0) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memset(v, 0, N*sizeof(double));
        
        v[0] = xo;
        if (N > 1)
            v[1] = yo;
        if (N > 2)
            v[2] = zo;
        if (N > 3)
            v[3] = wo;
        if (N > 4)
            v[4] = ao;
        if (N > 5)
            v[5] = bo;
        if (N > 6)
            v[6] = co;
    }
    
    void operator=(const VecN<N> &o)
    {
        for (int i = 0; i < N; i++)
            v[i] = o[i];
    }
    
    void operator=(double o)
    {
        for (int i = 0; i < N; i++)
            v[i] = o;
    }
    
    void operator=(const double* o)
    {
        for (int i = 0; i < N; i++)
            v[i] = o[i];
    }
    
    void operator%=(const VecN<N> &o)
    {
        for (int i = 0; i < N; i++)
            v[i] *= o[i];
    }
    
    void operator+=(const VecN<N> &o)
    {
        for (int i = 0; i < N; i++)
            v[i] += o[i];
    }
    
    void operator-=(const VecN<N> &o)
    {
        for (int i = 0; i < N; i++)
            v[i] -= o[i];
    }
    
    void operator+=(double o)
    {
        for (int i = 0; i < N; i++)
            v[i] += o;
    }
    
    void operator-=(double o)
    {
        for (int i = 0; i < N; i++)
            v[i] -= o;
    }
    
    void operator*=(double s)
    {
        scale(s);
    }
    
    void scale(double s)
    {
        for (int i = 0; i < N; i++)
            v[i] *= s;
    }
    
    void operator/=(double s)
    {
        scale(1.0/s);
    }
    
    VecN<N> operator+(const VecN<N> &o) const
    {
        VecN<N> c = *this;
        c += o;
        return c;
    }
    
    VecN<N> operator%(const VecN<N> &o) const
    {
        VecN<N> c = *this;
        c %= o;
        return c;
    }
    
    VecN<N> operator-(const VecN<N> &o) const
    {
        VecN<N> c = *this;
        c -= o;
        return c;
    }
    
    VecN<N> operator+(double o) const
    {
        VecN<N> c = *this;
        c += o;
        return c;
    }
    
    VecN<N> operator-(double o) const
    {
        VecN<N> c = *this;
        c -= o;
        return c;
    }
    
    VecN<N> operator-() const
    {
        VecN<N> c = *this;
        c *= -1;
        return c;
    }
    
    // Scalar product.
    double operator*(const VecN<N> &o) const
    {
        double s = 0;
        for (int i = 0; i < N; i++)
            s += v[i]*o[i];
        return s;
    }
    
    VecN<N> operator*(double s) const
    {
        VecN<N> c = *this;
        c *= s;
        return c;
    }
    
    VecN<N> operator/(double s) const
    {
        VecN<N> c = *this;
        c /= s;
        return c;
    }
    
    // Epsilon check of this vector being equal with the other.
    bool operator==(const VecN<N> &o) const
    {
        for (int i = 0; i < N; i++)
            if (fabs(v[i]-o[i]) > EPSILON)
                return false;
        return true;
    }
    
    // Epsilon check of this vector being different from the other.
    bool operator!=(const VecN<N> &o) const
    {
        return !(*this == o);
    }
    
    double & operator[](int i)
    {
        return v[i];
    }
    
    double operator[](int i) const
    {
        return v[i];
    }
    
    double last() const
    {
        return v[N-1];
    }
    
    double & last()
    {
        return v[N-1];
    }
    
    // Returns the number of dimensions. This is not the norm.
    int size() const
    {
        return N;
    }
    
    // The length of the vector. Same as norm().
    double length() const
    {
        return norm();
    }
    
    // Euklidean norm.
    double norm() const
    {
        double sum = 0;
        for (int i = 0; i < N; i++)
            sum += v[i]*v[i];
        return sqrt(sum);
    }
    
    // Squared Euklidean norm (a bit faster).
    double norm2() const
    {
        double sum = 0;
        for (int i = 0; i < N; i++)
            sum += v[i]*v[i];
        return sum;
    }
    
    // Manhattan (L1) norm.
    double norm1() const
    {
        double sum = 0;
        for (int i = 0; i < N; i++)
            sum += fabs(v[i]);
        return sum;
    }

    // P norm.
    double normp(double p) const
    {
        double sum = 0;
        for (int i = 0; i < N; i++)
            sum += pow(fabs(v[i]), p);
        return pow(sum, 1.0/p);
    }
    
    // Returns the smaller, always positive angle between this and the given vector.
    double angleTo(const VecN<N> &o) const
    {
        return acos(qBound(-1.0, ((*this)*o) / (norm()*o.norm()), 1.0));
    }

    // Fills the vector with random values between 0 and 1.
    void randomize(double from=0, double to=1.0)
    {
        for (int i = 0; i < N; i++)
            v[i] = from + ((double)rand()/RAND_MAX) * (to-from);
    }

    // Returns a random vector with components between 0 and 1.
    static VecN<N> random()
    {
        VecN<N> c;
        for (int i = 0; i < N; i++)
            c[i] = (double)rand()/RAND_MAX;
        return c;
    }

    // Projects this vector onto the vector o.
    void projectOnVector(const VecN<N> &o)
    {
        if (o.isNull())
            *this = 0.0;
        else
            *this = (((*this)*o)/(o*o))*o;
    }

    // Projects this vector onto the vector o.
    VecN<N> projectedOnVector(const VecN<N>& o)
    {
        VecN<N> c = *this;
        c.projectOnVector(o);
        return c;
    }

    
    // Projects this vector onto the plane that is defined by the normal.
    void projectOnPlane(const VecN<N>& normal)
    {
        *this -= (((*this)*normal) / normal.norm2()) * normal;
    }

    // Projects this vector onto the plane that is defined by the normal.
    VecN<N> projectedOnPlane(const VecN<N>& normal)
    {
        VecN<N> c = *this;
        c.projectOnPlane(normal);
        return c;
    }

    // Replaces all components with their absolute values.
    void abs()
    {
        for (int i=0; i<N; i++)
            v[i] = fabs(v[i]);
    }
    
    // Returns the largest component.
    double max() const
    {
        double m = v[0];
        for (int i=1; i<N; i++)
            m = fmax(v[i], m);
        return m;
    }
    
    // Normalizes this vector to the given length (default 1.0).
    void normalize(double length=1.0)
    {
        double n = norm();
        for (int i=0; i<N; i++)
            v[i] = length*v[i]/n;
    }
    
    // Normalizes this vector to the given length (default 1.0).
    VecN<N> normalized(double length=1.0) const
    {
        VecN<N> c = *this;
        c.normalize(length);
        return c;
    }
    
    // Bounds all components of this vector to lie between the respective components of l and u.
    void bound(const VecN<N>& l, const VecN<N>& u)
    {
        for (int i = 0; i < N; i++)
            v[i] = qBound(l[i], v[i], u[i]);
    }
    
    // Bounds all components of this vector to lie between the respective components of l and u.
    VecN<N> bounded(const VecN<N>& l, const VecN<N>& u) const
    {
        VecN<N> c = *this;
        c.bound(l, u);
        return c;
    }
    
    // Bounds all components of this vector to lie between l and u.
    void bound(const double& l, const double& u)
    {
        for (int i = 0; i < N; i++)
            v[i] = qBound(l, v[i], u);
    }
    
    // Bounds all components of this vector to lie between l and u.
    VecN<N> bounded(const double& l, const double& u) const
    {
        VecN<N> c = *this;
        c.bound(l, u);
        return c;
    }
    
    // Sets all values of the vector to x.
    void set(double x)
    {
        for (int i = 0; i < N; i++)
            v[i] = x;
    }
    
    // Sets the vector to the provided values.
    void set(double xo, double yo, double zo=0, double wo=0, double ao=0, double bo=0, double co=0)
    {
        v[0] = xo;
        if (N > 1)
            v[1] = yo;
        if (N > 2)
            v[2] = zo;
        if (N > 3)
            v[3] = wo;
        if (N > 4)
            v[4] = ao;
        if (N > 5)
            v[5] = bo;
        if (N > 6)
            v[6] = co;
    }
    
    // Epsilon checks the vector for being null.
    bool isNull() const
    {
        for (int i = 0; i < N; i++)
            if (v[i] > 1.0E-5 || v[i] < -1.0E-5)
                return false;
        return true;
    }

    // Sets all values of the VecN to zero.
    void setNull()
    {
        for (int i = 0; i < N; i++)
            v[i] = 0;
    }

    // Returns true if the vector contains at least one nan value.
    bool isNan() const
    {
        for (int i = 0; i < N; i++)
            if (isnan(v[i]))
                return true;
        return false;
    }
    
    // OpenGL support.
    operator const double*() const {return v;}
    const double* data() const {return v;}
    double* data() {return v;}
};

template <int N>
VecN<N> operator*(double s, const VecN<N> &o)
{
    return VecN<N>(o) * s;
}

template <int N>
QDebug operator<<(QDebug dbg, const VecN<N> &o)
{
    if (dbg.autoInsertSpaces())
    {
        dbg.setAutoInsertSpaces(false);
        dbg << "[" << o[0];
        for (int i = 1; i < N; i++)
            dbg << ", " << o[i];
        dbg << "] ";
        dbg.setAutoInsertSpaces(true);
    }
    else
    {
        dbg << "[" << o[0];
        for (int i = 1; i < N; i++)
            dbg << ", " << o[i];
        dbg << "] ";
    }
    return dbg;
}

template <int N>
QTextStream& operator<<(QTextStream& out, const VecN<N> &o)
{
    out << "[" << o[0];
    for (int i = 1; i < N; i++)
        out << ", " << o[i];
    out << "]";
    return out;
}

template <int N>
QDataStream& operator<<(QDataStream& out, const VecN<N> &o)
{
    for (int i = 0; i < N; i++)
        out << o[i];
    return out;
}

template <int N>
QDataStream& operator>>(QDataStream& in, VecN<N> &o)
{
    for (int i = 0; i < N; i++)
        in >> o[i];
    return in;
}

#endif

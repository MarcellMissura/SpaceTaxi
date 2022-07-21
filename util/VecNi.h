#ifndef VECNI_H_
#define VECNI_H_
#include "globals.h"
#include <math.h>
#include <QDebug>
#include <QDataStream>

//#include "VecNu.h"
template <int N>
class VecNu;

// The NVec is a generic vector template where the number of dimensions N can be specified
// as a template parameter. For example, NVec<3> v; creates a three dimensional vector.
// Literal references v.x, v.y, and v.z are available to access the first three elements
// of the vector (if available). Other than that, the NVec supports a couple of standard
// operators. Take a look.

template <int N=3>
class VecNi
{
    int v[N];

public:

	// .x, .y amd .z element access operators for convenience.
    int& x;
    int& y;
    int& z;
    int& w;

    VecNi() : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
	{
        memset(v, 0, N*sizeof(int));
	}

    VecNi(const VecNi<N> &o) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        if (this != &o)
            memcpy(v, &o, N*sizeof(int));
    }

    VecNi(const VecNi<N-1> &o, int xo) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memcpy(v, &o, (N-1)*sizeof(int));
        v[N-1] = xo;
    }

    VecNi(int xo, const VecNi<N-1> &o) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        v[0] = xo;
        memcpy(v+1, &o, (N-1)*sizeof(int));
    }

    VecNi(const VecNi<N+1> &o) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memcpy(v, &o, N*sizeof(int));
    }

    VecNi(const int* xo) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memcpy(v, xo, N*sizeof(int));
    }

    VecNi(const int* xo, int yo) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memcpy(v, xo, (N-1)*sizeof(int));
        v[N-1] = yo;
    }

    VecNi(int xo, int yo, int zo=0, int wo=0, int ao=0, int bo=0, int co=0) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
	{
        memset(v, 0, N*sizeof(int));

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

    VecNi(const VecNi<3> &o1, const VecNi<3> &o2) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memset(v, 0, N*sizeof(int));

        v[0] = o1[0];
        if (N > 1)
            v[1] = o1[0];
        if (N > 2)
            v[2] = o1[1];
        if (N > 3)
            v[3] = o1[2];
        if (N > 4)
            v[4] = o2[0];
        if (N > 5)
            v[5] = o2[1];
        if (N > 6)
            v[6] = o2[2];
    }

    void operator=(const VecNi<N> &o)
	{
		if (this != &o)
            memcpy(v, &o, N*sizeof(int));
	}

    void operator=(int o)
    {
        for (int i = 0; i < N; i++)
            v[i] = o;
    }

    void operator=(const int* o)
    {
        for (int i = 0; i < N; i++)
            v[i] = o[i];
    }

    void operator+=(const VecNi<N> &o)
	{
		for (int i = 0; i < N; i++)
			v[i] += o[i];
	}

    void operator+=(const VecNu<N> &o)
    {
        for (int i = 0; i < N; i++)
            v[i] += o[i];
    }

    void operator-=(const VecNi<N> &o)
	{
		for (int i = 0; i < N; i++)
			v[i] -= o[i];
	}

    void operator-=(const VecNu<N> &o)
    {
        for (int i = 0; i < N; i++)
            v[i] -= o[i];
    }

    VecNi operator+(const VecNi<N> &o) const
    {
        VecNi<N> c = *this;
        c += o;
        return c;
    }

    VecNi operator+(const VecNu<N> &o) const
    {
        VecNi<N> c = *this;
        c += o;
        return c;
    }

    VecNi operator-(const VecNi<N> &o) const
    {
        VecNi<N> c = *this;
        c -= o;
        return c;
    }

    VecNi operator-(const VecNu<N> &o) const
    {
        VecNi<N> c = *this;
        c -= o;
        return c;
    }

    void operator+=(int o)
	{
		for (int i = 0; i < N; i++)
			v[i] += o;
	}

    void operator-=(int o)
	{
		for (int i = 0; i < N; i++)
			v[i] -= o;
	}

    void operator+=(uint o)
    {
        for (int i = 0; i < N; i++)
            v[i] += o;
    }

    void operator-=(uint o)
    {
        for (int i = 0; i < N; i++)
            v[i] -= o;
    }

    VecNi operator+(int o) const
    {
        VecNi<N> c = *this;
        c += o;
        return c;
    }

    VecNi operator+(uint o) const
    {
        VecNi<N> c = *this;
        c += o;
        return c;
    }

    VecNi operator-(int o) const
    {
        VecNi<N> c = *this;
        c -= o;
        return c;
    }

    VecNi operator-(uint o) const
    {
        VecNi<N> c = *this;
        c -= o;
        return c;
    }

    VecNi operator-() const
    {
        VecNi<N> c = *this;
        c *= -1;
        return c;
    }

    void operator*=(int s)
	{
		for (int i = 0; i < N; i++)
			v[i] *= s;
	}

    void operator/=(int s)
	{
		for (int i = 0; i < N; i++)
			v[i] /= s;
	}

    // Scalar product.
    int operator*(const VecNi<N> &o) const
    {
        int s = 0;
        for (int i = 0; i < N; i++)
            s += v[i]*o[i];
        return s;
    }

    VecNi operator*(int s) const
	{
        VecNi<N> c = *this;
		c *= s;
		return c;
	}

    VecNi operator/(int s) const
	{
        VecNi<N> c = *this;
		c /= s;
		return c;
	}

    bool operator==(const VecNi<N> &o) const
	{
		for (int i = 0; i < N; i++)
            if (fabs(v[i]-o[i]) > EPSILON)
				return false;
		return true;
	}

    bool operator!=(const VecNi<N> &o) const
	{
		return !(*this == o);
	}

    int & operator[](int i)
	{
		return v[i];
	}

    int operator[](int i) const
	{
		return v[i];
	}

    int last() const
	{
		return v[N-1];
	}

    int & last()
    {
        return v[N-1];
    }

    // Returns the number of dimensions. This is not the norm.
    int size() const
	{
		return N;
	}

    // Euklidean norm.
    double norm() const
	{
        int sum = 0;
		for (int i = 0; i < N; i++)
			sum += v[i]*v[i];
		return sqrt(sum);
	}

    // Squared Euklidean norm (a bit faster).
    uint norm2() const
	{
        int sum = 0;
		for (int i = 0; i < N; i++)
			sum += v[i]*v[i];
		return sum;
	}

    // Manhattan (L1) norm.
    uint norm1() const
    {
        int sum = 0;
        for (int i = 0; i < N; i++)
            sum += fabs(v[i]);
        return sum;
    }

    // Returns the smaller, always positivie angle between this and the given vector.
    double angle(const VecNi<N> &o) const
    {
        return acos(qBound(-1.0, ((*this)*o) / (norm()*o.norm()), 1.0));
    }

	// Fills the vector with random values between 0 and 1.0.
    void randomize()
	{
		for (int i = 0; i < N; i++)
            v[i] = (int)qrand()/RAND_MAX;
	}

    // Projects this vector onto the vector o.
    void projectOnVector(const VecNi<N> &o)
    {
        *this = (((*this)*o)/(o*o))*o;
    }

    // Projects this vector onto the plane defined by its normal.
    void projectOnPlane(const VecNi<N>& normal)
    {
        *this -= (((*this)*normal) / normal.norm2()) * normal;
    }

    // Replaces all components with their absolute values.
    void abs()
    {
        for (int i=0; i<N; i++)
            v[i] = fabs(v[i]);
    }

    // Normalizes this vector to the given length (default 1.0).
    void normalize(int length=1.0)
    {
        int n = norm();
        for (int i=0; i<N; i++)
            v[i] = length*v[i]/n;
    }

    // Bounds all components of this vector to lie between the respective components of l and u.
    void bound(const VecNi<N>& l, const VecNi<N>& u)
    {
        for (int i = 0; i < N; i++)
            v[i] = qBound(l[i], v[i], u[i]);
    }

    // Bounds all components of this vector to lie between l and u, respectively.
    void bound(const int& l, const int& u)
    {
        for (int i = 0; i < N; i++)
            v[i] = qBound(l, v[i], u);
    }

    // Sets all values of the vector to x.
    void set(int x)
    {
        for (int i = 0; i < N; i++)
            v[i] = x;
    }

    // Sets the vector to the provided values.
    void set(int xo, int yo, int zo=0, int wo=0, int ao=0, int bo=0, int co=0)
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

    // OpenGL support.
    operator const int*() const {return v;}
    const int* data() const {return v;}
    int* data() {return v;}
};

template <int N>
VecNi<N> operator*(int s, const VecNi<N> &o)
{
    return VecNi<N>(o) * s;
}

template <int N>
QDebug operator<<(QDebug dbg, const VecNi<N> &o)
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
QTextStream& operator<<(QTextStream& out, const VecNi<N> &o)
{
    out << "[" << o[0];
    for (int i = 1; i < N; i++)
        out << ", " << o[i];
    out << "]";
    return out;
}

template <int N>
QDataStream& operator<<(QDataStream& out, const VecNi<N> &o)
{
    for (int i = 0; i < N; i++)
        out << o[i];
    return out;
}

template <int N>
QDataStream& operator>>(QDataStream& in, VecNi<N> &o)
{
    for (int i = 0; i < N; i++)
        in >> o[i];
    return in;
}

#endif //VecNi

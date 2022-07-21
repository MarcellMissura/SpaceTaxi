#ifndef VECNU_H_
#define VECNU_H_
#include "globals.h"
#include <math.h>
#include <QDebug>
#include <QDataStream>
#include "VecNi.h"

// The VecNu is a generic vector of unsigned ints where the number of dimensions N
// can be specified as a template parameter. For example, VecNu<3> v; creates a three
// dimensional vector. Literal references v.x, v.y, and v.z are available to access the first
// three elements of the vector (if available). Other than that, the VecNu supports a couple
// of standard operators. Take a look.

template <int N=3>
class VecNu
{
    uint v[N];

public:

	// .x, .y amd .z element access operators for convenience.
    uint& x;
    uint& y;
    uint& z;
    uint& w;

    VecNu() : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
	{
        memset(v, 0, N*sizeof(uint));
	}

    VecNu(const VecNu<N> &o) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        if (this != &o)
            memcpy(v, &o, N*sizeof(uint));
    }

    VecNu(const VecNu<N-1> &o, uint xo) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memcpy(v, &o, (N-1)*sizeof(uint));
        v[N-1] = xo;
    }

    VecNu(uint xo, const VecNu<N-1> &o) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        v[0] = xo;
        memcpy(v+1, &o, (N-1)*sizeof(uint));
    }

    VecNu(const VecNu<N+1> &o) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memcpy(v, &o, N*sizeof(uint));
    }

    VecNu(const uint* xo) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memcpy(v, xo, N*sizeof(uint));
    }

    VecNu(const uint* xo, uint yo) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memcpy(v, xo, (N-1)*sizeof(uint));
        v[N-1] = yo;
    }

    VecNu(uint xo, uint yo, uint zo=0, uint wo=0, uint ao=0, uint bo=0, uint co=0) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
	{
        memset(v, 0, N*sizeof(uint));

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

    VecNu(const VecNu<3> &o1, const VecNu<3> &o2) : x(v[0]), y(v[N > 1 ? 1 : 0]), z(v[N > 2 ? 2 : 0]), w(v[N > 3 ? 3 : 0])
    {
        memset(v, 0, N*sizeof(uint));

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

    void operator=(const VecNu<N> &o)
	{
		if (this != &o)
            memcpy(v, &o, N*sizeof(uint));
	}

    void operator=(uint o)
    {
        for (int i = 0; i < N; i++)
            v[i] = o;
    }

    void operator=(const uint* o)
    {
        for (int i = 0; i < N; i++)
            v[i] = o[i];
    }

    void operator+=(const VecNu<N> &o)
	{
		for (int i = 0; i < N; i++)
			v[i] += o[i];
	}

    void operator+=(const VecNi<N> &o)
    {
        for (int i = 0; i < N; i++)
            v[i] += o[i];
    }

    void operator-=(const VecNu<N> &o)
	{
		for (int i = 0; i < N; i++)
			v[i] -= o[i];
	}

    void operator-=(const VecNi<N> &o)
    {
        for (int i = 0; i < N; i++)
            v[i] -= o[i];
    }

    VecNu operator+(const VecNu<N> &o) const
    {
        VecNu<N> c = *this;
        c += o;
        return c;
    }

    VecNu operator+(const VecNi<N> &o) const
    {
        VecNu<N> c = *this;
        c += o;
        return c;
    }

    VecNu operator-(const VecNu<N> &o) const
    {
        VecNu<N> c = *this;
        c -= o;
        return c;
    }

    VecNu operator-(const VecNi<N> &o) const
    {
        VecNu<N> c = *this;
        c -= o;
        return c;
    }

    void operator+=(uint o)
	{
		for (int i = 0; i < N; i++)
			v[i] += o;
	}

    void operator+=(int o)
    {
        for (int i = 0; i < N; i++)
            v[i] += o;
    }

    void operator-=(uint o)
	{
		for (int i = 0; i < N; i++)
			v[i] -= o;
	}

    void operator-=(int o)
    {
        for (int i = 0; i < N; i++)
            v[i] -= o;
    }

    VecNu operator+(uint o) const
    {
        VecNu<N> c = *this;
        c += o;
        return c;
    }

    VecNu operator+(int o) const
    {
        VecNu<N> c = *this;
        c += o;
        return c;
    }

    VecNu operator-(uint o) const
    {
        VecNu<N> c = *this;
        c -= o;
        return c;
    }

    VecNu operator-(int o) const
    {
        VecNu<N> c = *this;
        c -= o;
        return c;
    }

    VecNu operator-() const
    {
        VecNu<N> c = *this;
        c *= -1;
        return c;
    }


    void operator*=(uint s)
	{
		for (int i = 0; i < N; i++)
			v[i] *= s;
	}

    void operator/=(uint s)
	{
		for (int i = 0; i < N; i++)
			v[i] /= s;
	}

    // Scalar product.
    uint operator*(const VecNu<N> &o) const
    {
        uint s = 0;
        for (int i = 0; i < N; i++)
            s += v[i]*o[i];
        return s;
    }

    VecNu operator*(uint s) const
	{
        VecNu<N> c = *this;
		c *= s;
		return c;
	}

    VecNu operator/(uint s) const
	{
        VecNu<N> c = *this;
		c /= s;
		return c;
	}

    bool operator==(const VecNu<N> &o) const
	{
		for (int i = 0; i < N; i++)
            if (fabs(v[i]-o[i]) > EPSILON)
				return false;
		return true;
	}

    bool operator!=(const VecNu<N> &o) const
	{
		return !(*this == o);
	}

    uint & operator[](int i)
	{
		return v[i];
	}

    uint operator[](int i) const
	{
		return v[i];
	}

    uint last() const
	{
		return v[N-1];
	}

    uint & last()
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
        uint sum = 0;
		for (int i = 0; i < N; i++)
			sum += v[i]*v[i];
		return sqrt(sum);
	}

    // Squared Euklidean norm (a bit faster).
    uint norm2() const
	{
        uint sum = 0;
		for (int i = 0; i < N; i++)
			sum += v[i]*v[i];
		return sum;
	}

    // Manhattan (L1) norm.
    uint norm1() const
    {
        uint sum = 0;
        for (int i = 0; i < N; i++)
            sum += fabs(v[i]);
        return sum;
    }

    // Returns the smaller, always positivie angle between this and the given vector.
    uint angle(const VecNu<N> &o) const
    {
        return acos(qBound(-1.0, ((*this)*o) / (norm()*o.norm()), 1.0));
    }

	// Fills the vector with random values between 0 and 1.0.
    void randomize()
	{
		for (int i = 0; i < N; i++)
            v[i] = (uint)qrand()/RAND_MAX;
	}

    // Projects this vector onto the vector o.
    void projectOnVector(const VecNu<N> &o)
    {
        *this = (((*this)*o)/(o*o))*o;
    }

    // Projects this vector onto the plane defined by its normal.
    void projectOnPlane(const VecNu<N>& normal)
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
    void normalize(uint length=1.0)
    {
        uint n = norm();
        for (int i=0; i<N; i++)
            v[i] = length*v[i]/n;
    }

    // Bounds all components of this vector to lie between the respective components of l and u.
    void bound(const VecNu<N>& l, const VecNu<N>& u)
    {
        for (int i = 0; i < N; i++)
            v[i] = qBound(l[i], v[i], u[i]);
    }

    // Bounds all components of this vector to lie between l and u, respectively.
    void bound(const uint& l, const uint& u)
    {
        for (int i = 0; i < N; i++)
            v[i] = qBound(l, v[i], u);
    }

    // Sets all values of the vector to x.
    void set(uint x)
    {
        for (int i = 0; i < N; i++)
            v[i] = x;
    }

    // Sets the vector to the provided values.
    void set(uint xo, uint yo, uint zo=0, uint wo=0, uint ao=0, uint bo=0, uint co=0)
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
    operator const uint*() const {return v;}
    const uint* data() const {return v;}
    uint* data() {return v;}
};

template <int N>
VecNu<N> operator*(uint s, const VecNu<N> &o)
{
    return VecNu<N>(o) * s;
}

template <int N>
QDebug operator<<(QDebug dbg, const VecNu<N> &o)
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
QTextStream& operator<<(QTextStream& out, const VecNu<N> &o)
{
    out << "[" << o[0];
    for (int i = 1; i < N; i++)
        out << ", " << o[i];
    out << "]";
    return out;
}

template <int N>
QDataStream& operator<<(QDataStream& out, const VecNu<N> &o)
{
    for (int i = 0; i < N; i++)
        out << o[i];
    return out;
}

template <int N>
QDataStream& operator>>(QDataStream& in, VecNu<N> &o)
{
    for (int i = 0; i < N; i++)
        in >> o[i];
    return in;
}

#endif //VecNu

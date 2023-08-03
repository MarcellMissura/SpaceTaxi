#ifndef LIP2D_H_
#define LIP2D_H_

#include <QDebug>


class Lip2D;
extern Lip2D operator*(const double scalar, const Lip2D& v);
extern Lip2D operator*(const Lip2D& v, const double scalar);

class Lip2D
{

public:

    double C;
	double x;
	double vx;
	double y;
	double vy;
    double zmpx;
    double zmpy;

    Lip2D();
    Lip2D(double x, double vx, double y, double vy);
    ~Lip2D(){}

    void set(double x, double vx, double y, double vy);

    void predict(double dt);
    Lip2D predicted(double t);

    void setZmp(double zx, double zy);


    inline Lip2D operator+(const Lip2D& v) const
	{
        Lip2D lm = *this;
		lm.x += v.x;
		lm.vx += v.vx;
		lm.y += v.y;
		lm.vy += v.vy;
		return lm;
	}
    inline Lip2D operator-() const
	{
        Lip2D lm = *this;
		lm.x = -lm.x;
		lm.vx = -lm.vx;
		lm.y = -lm.y;
		lm.vy = -lm.vy;

		return lm;
	}
    inline Lip2D operator-(const Lip2D& v) const
	{
        Lip2D lm = *this;
		lm.x -= v.x;
		lm.vx -= v.vx;
		lm.y -= v.y;
		lm.vy -= v.vy;
		return lm;
	}
    inline Lip2D operator*(const Lip2D& v) const
	{
        Lip2D lm = *this;
		lm.x *= v.x;
		lm.vx *= v.vx;
		lm.y *= v.y;
		lm.vy *= v.vy;
		return lm;
	}
    inline void operator*=(const Lip2D& v)
	{
		x *= v.x;
		vx *= v.vx;
		y *= v.y;
		vy *= v.vy;
	}
    inline void operator+=(const Lip2D& v)
	{
		x+=v.x;
		vx+=v.vx;
		y+=v.y;
		vy+=v.vy;
	}
    inline void operator-=(const Lip2D& v)
	{
		x-=v.x;
		vx-=v.vx;
		y-=v.y;
		vy-=v.vy;
	}
	inline void operator*=(const double scalar)
	{
		x*=scalar;
		vx*=scalar;
		y*=scalar;
		vy*=scalar;
	}
	inline void operator/=(const double scalar)
	{
		x/=scalar;
		vx/=scalar;
		y/=scalar;
		vy/=scalar;
	}
    inline bool operator==(const Lip2D& v) const
	{
		return (x==v.x) && (vx==v.vx) && (y==v.y) && (vy==v.vy);
	}
    inline bool operator!=(const Lip2D& v) const
	{
		return (x!=v.x) || (vx!=v.vx) || (y!=v.y) || (vy!=v.vy);
	}
};

QDebug operator<<(QDebug dbg, const Lip2D &o);

#endif

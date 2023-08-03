#ifndef HPM_H_
#define HPM_H_

#include <QDebug>
#include <QList>

// This is a one dimensional holonomic point mass with simple constant velocity Newtonian physics.

class Hpm;

extern Hpm operator*(const double scalar, const Hpm& l);
extern Hpm operator*(const Hpm& l, const double scalar);
extern Hpm operator/(const double scalar, const Hpm& l);
extern Hpm operator/(const Hpm& l, const double scalar);

class Hpm
{
public:

    double x, v, a;

	Hpm();
    Hpm(double x, double v, double a=0);
    ~Hpm(){}

	void reset();

    void set(double x, double v, double a=0);

	void simulate(double a, double time);
	void predict(double a, double time);
	Hpm predicted(double a, double time);

    Hpm operator+(const Hpm& l) const {return Hpm(x+l.x, v+l.v);}
    Hpm operator-() const {return Hpm(-x, -v);}
    Hpm operator-(const Hpm& l) const {return Hpm(x-l.x, v-l.v);}
    Hpm& operator*=(const double scalar){x*=scalar; v*=scalar; return *this;}
    Hpm& operator/=(const double scalar){x/=scalar; v/=scalar; return *this;}
    Hpm& operator+=(const Hpm& l){x+=l.x; v+=l.v; return *this;}
    Hpm& operator-=(const Hpm& l){x-=l.x; v-=l.v; return *this;}
    bool operator==(const Hpm& l){return (x==l.x && v==l.v);}
    bool operator!=(const Hpm& l){return (x!=l.x || v!=l.v);}
};

QDebug operator<<(QDebug dbg, const Hpm &s);

#endif /* HPM_H_ */

#ifndef LIP_H_
#define LIP_H_
#include <QDebug>

class Lip;

extern Lip operator*(const double scalar, const Lip& l);
extern Lip operator*(const Lip& l, const double scalar);
extern Lip operator/(const double scalar, const Lip& l);
extern Lip operator/(const Lip& l, const double scalar);

class Lip
{
public:

    double C;
    double x, vx;

    Lip();
    Lip(double x, double vx, double C);
    ~Lip(){}

    void reset();

    void set(double x, double vx);
    void set(double x, double vx, double C);

    void simulate(double time);
    void predict(double time);
    Lip predicted(double time) const;
    double tLoc(double x) const;
    double tVel(double x) const;

    double energy() const;
    double static energy(double x, double v, double C);

    inline Lip operator+(const Lip& l) const {return Lip(x+l.x, vx+l.vx, C);}
    inline Lip operator-() const {return Lip(-x, -vx, C);}
    inline Lip operator-(const Lip& l) const {return Lip(x-l.x, vx-l.vx, C);}
    inline Lip& operator*=(const double scalar){x*=scalar; vx*=scalar; return *this;}
    inline Lip& operator/=(const double scalar){x/=scalar; vx/=scalar; return *this;}
    inline Lip& operator+=(const Lip& l){x+=l.x; vx+=l.vx; return *this;}
    inline Lip& operator-=(const Lip& l){x-=l.x; vx-=l.vx; return *this;}

};

QDebug operator<<(QDebug dbg, const Lip &o);

#endif /* LIP_H_ */

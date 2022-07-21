#ifndef POC_H_
#define POC_H_

#include <QDebug>

class Poc
{
public:

    double phi;
    double vphi;
    double aphi;
    double x;
    double vx;
    double ax;

    double L;

    Poc();
    Poc(double phi, double vphi, double x, double vx);
    ~Poc(){}

    void reset();
    void set(double phi, double vphi, double x, double vx);
    void setL(double L);

    void simulate(double time, double ax=0);
    void simulate2(double time, double ax=0);
    void predictTaylor(double time, double ax=0);
    void predictNaive(double time, double ax=0);

};

QDebug operator<<(QDebug dbg, const Poc &p);

#endif /* POC_H_ */

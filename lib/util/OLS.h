#ifndef OLS_H_
#define OLS_H_
#define ARMA_DONT_USE_WRAPPER

#include <armadillo>
#include "lib/util/Vector.h"
#include "lib/util/Vec3.h"
#include "lib/util/Vec2.h"

// This is an ordinary (linear) least squares regressor for 3D points.
// https://en.wikipedia.org/wiki/Ordinary_least_squares
// It fits a plane into data points in 3D.
// Use addDataPoint() to feed the OLS with data. Then, use init() to initialze
// the regression parameters. After initialization, you can use evaluateAt(x) to
// query for an estimate at location x, or getNormal() to ask for the plane normal.

class OLS
{
    int loadedPoints; // number of actually loaded points
    bool initialized; // status check flag
    Vector<Vec3> data;
    arma::Col<double> beta;

public:

    OLS();
    ~OLS(){}

    void init();
    void reset();

    int getLoadedPointCount() const;
    void addDataPoint(const Vec3& p);
    double evaluateAt(const Vec2 &p) const;
    Vec3 getNormal() const;
    double getIntercept() const;

    // OpenGL drawing code.
    void draw(uint sampleFactor=1) const;

    void print() const;

};

QDebug operator<<(QDebug dbg, const OLS &o);

#endif // OLS

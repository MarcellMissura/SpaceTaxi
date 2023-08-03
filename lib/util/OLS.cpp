#include "OLS.h"
#include <QColor>
#include "blackboard/Config.h"
#include "util/GLlib.h"
#include "util/ColorUtil.h"
#include <GL/glu.h>

// This is an ordinary (linear) least squares regressor.
// Use addDataPoint() to feed the OLS with data. Then, use init() to initialze
// the regression parameters. After initialization, you can use evaluateAt(x) to
// query for an estimate at location x, or getNormal() to ask for the plane normal.

// Boring constructor.
OLS::OLS()
{
    loadedPoints = 0;
    beta.set_size(3);
}

// Clears all data points and resets the OLS search to a blank state.
void OLS::reset()
{
    loadedPoints = 0;
    data.clear();
    beta.clear();
}

// Returns the number of points in the data set.
int OLS::getLoadedPointCount() const
{
    return loadedPoints;
}

// Initializes the OLS.
// This method should be called after all data points have been added.
void OLS::init()
{
    if (loadedPoints < 3)
    {
        qDebug() << "OLS init(): not enough points to determine the plane.";
        return;
    }

    // Build X and Y.
    using namespace arma;
    Mat<double> X;
    X.set_size(data.size(), 3);
    Col<double> Y;
    Y.set_size(data.size());
    for (int i = 0; i < data.size(); i++)
    {
        X(i,0) = data[i].x;
        X(i,1) = data[i].y;
        X(i,2) = 1;
    }

    for (int i = 0; i < data.size(); i++)
        Y[i] = data[i].z;

    // Compute beta.
    beta = solve(X,Y); //(X.t()*X).i()*X.t()*Y; //
    //qDebug() <<beta[0]<<beta[1]<<beta[2];
    initialized = true;
}

// Adds a single point to the data point set.
// Do this first and then call init() to perform the OLS.
// You can add more points after calling init(), but call init() again.
void OLS::addDataPoint(const Vec3& p)
{
    initialized = false;
    data.push_back(p);
    loadedPoints++;
}

// Evaluates the linear regression at point p.
double OLS::evaluateAt(const Vec2& p) const
{
    return p.x*beta[0]+p.y*beta[1]+beta[2];
}

// Returns the normal to the regressed plane.
Vec3 OLS::getNormal() const
{
    if (loadedPoints < 3)
       return Vec3(0.0, 0.0, 0.0);

    Vec3 normal(-beta[0], -beta[1], 1);
    normal.normalize();
    return normal;
}

// Returns the intercept of the regressed plane.
double OLS::getIntercept() const
{
    return beta[2];
}

// Prints all the loaded points.
void OLS::print() const
{
    qDebug() << data;
}

// OpenGL drawing code that draws the location of the stored data points.
void OLS::draw(uint sampleFactor) const
{
    glPushMatrix();
    glPointSize(3);
    glColor3f(0,0,0);
    glBegin(GL_POINTS);
    sampleFactor = qMax((uint)1, sampleFactor);
    for (int k=0; k < loadedPoints; k=k+sampleFactor)
        glVertex3dv(data.at(k));
    glEnd();
    glPopMatrix();
}

QDebug operator<<(QDebug dbg, const OLS &o)
{
    o.print();
}

#ifndef SAMPLEGRID_H_
#define SAMPLEGRID_H_

#include "lib/util/Vector.h"
#include "lib/util/Vec2u.h"
#include "lib/util/Vec3.h"
#include "lib/util/OLS.h"
#include "lib/geometry/Polygon.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <QPainter>

// A sample s = (p,n) is a point p and a normal n that together
// desribe a plane. A sample is gained from the RGB-D image, so
// it also has pixel coordinates, a bufferIdx in the point buffer,
// a grid index in the sample grid.
struct Sample
{
    Vec2u imagePx;
    Vec2u gridIdx;
    int bufferIdx = 0;
    Vec3 p;
    Vec3 n;
    double angle = 0;
    bool in = true;
    int clusterId = -1;
    static Vec3 up;

    Sample()
    {
        //n.z = 1.0;
    }

    bool operator>(const Sample &o) const
    {
        return (up*p > up*o.p);
    }
    bool operator<(const Sample &o) const
    {
        return (up*p < up*o.p);
    }
    bool operator==(const Sample& o) const
    {
        return (bufferIdx == o.bufferIdx);
    }
    void operator+=(const Sample& o)
    {
        n += o.n;
        p += o.p;
    }
    void operator/=(double o)
    {
        n /= o;
        n.normalize();
        p /= o;
    }

    // The plane distance between two samples.
    // The normals have to be normalized!
    double distance(const Sample& o) const
    {
        double d1 = fabs(n*(o.p-p));
        double d2 = fabs(o.n*(o.p-p));
        double d3 = 1.0-n*o.n;
        return d1+d2;
    }

    // Distance of v to the plane described by this sample.
    double distance(const Vec3& v) const
    {
        return n*(v-p);
    }

    // Returns the plane height z at the coordinates v.
    // v is given in world coordinates.
    double evaluateAt(const Vec2& v) const
    {
        return (p*n-v.x*n.x-v.y*n.y)/n.z;
    }

    // Projects the vector v onto the plane described by this sample.
    Vec3 projectTo(const Vec3& v) const
    {
        return Vec3(v.x, v.y, evaluateAt(v));
    }
};

class SampleGrid
{
    Vector< Vector<Sample> > samples; // samples in a 2D grid structure
    Vector<Sample> prunedSamples; // Only the pruned samples in a vector.

    Vector<Vector<Sample> > planes; // All plane segments.
    Vector<Sample> planeAvg; // The averages of all plane segments.
    Vector<Sample> floorSegment; // contains the set of points that make up the floor
    Sample floorPlane; // one representative of the floor plane in (normal,point) form.

    Vector<Sample> planeCluster; // temporary
    Vec3 upVector;
    OLS ols;

    int debug;

public:

    SampleGrid();
    ~SampleGrid(){}

    void init(uint width, uint height);
    void update(const Vector<Vec3> &pointBuffer);

    void setUpVector(const Vec3& up);
    Vec3 getUpVector() const;

    Sample fitPlaneFlood();
    Sample fitPlaneFlood2();
    void findAllPlanes();

    void drawSamples(QPainter *painter) const;
    void drawSamples() const;
    void drawAllPlanes() const;

    bool isInitialized() const {return !samples.isEmpty();}

    int getDebug() const;
    void setDebug(int value);

private:
    void floodFill(const Vec2u &parentIdx);
    void prune();
    bool isIn(const Vec2u& gridIdx) const;

};

QDebug operator<<(QDebug dbg, const SampleGrid &o);
QDebug operator<<(QDebug dbg, const Sample &o);

#endif

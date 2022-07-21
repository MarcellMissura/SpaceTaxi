#ifndef DYNAMICWINDOWAPPROACH_H
#define DYNAMICWINDOWAPPROACH_H
#include <QMutex>
#include <QPainter>
#include "geometry/GeometricModel.h"
#include "geometry/DynamicGeometricModel.h"
#include "geometry/GridModel.h"
#include "util/Vector.h"

struct HDWA_Trajectory
{
    Vec2 acc;
    Vector<Hpm2D> ctrl;
    double colt;
    double geometricClearance;
    double targetDistance;
    double gridClearance;
    double backwardsPenalty;
    double f;
    double f1;
    double f2;

    HDWA_Trajectory()
    {
        colt = -1;
        geometricClearance = 0;
        targetDistance = 0;
        gridClearance = 0;
        backwardsPenalty = 0;
        f = 0;
        f1 = 0;
        f2 = 0;
        ctrl << Hpm2D();
        ctrl << Hpm2D();
    }
};


class HolonomicDWA
{
    int debug;
    const DynamicGeometricModel* geometricModel;
    const GridModel* gridModel;
    Vec2 target;
    Hpm2D start;

    Vector<HDWA_Trajectory> trajectories;
    int bestTrajectoryId;

    int trajectoryType;

public:
    HolonomicDWA();
    ~HolonomicDWA();

    void setDebug(int d);
    void setGeometricModel(const DynamicGeometricModel &gm);
    void setGridModel(const GridModel &gm);
    void setTarget(const Vec2 &wp);
    void setStart(const Hpm2D &wp);
    void setTrajectoryType(int type);

    Vec2 search();

    void draw(QPainter *painter) const;
};

#endif // DYNAMICWINDOWAPPROACH_H

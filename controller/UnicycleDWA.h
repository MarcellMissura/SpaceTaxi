#ifndef ARCDWA_H_
#define ARCDWA_H_
#include <QMutex>
#include <QPainter>
#include "geometry/DynamicGeometricModel.h"
#include "geometry/GeometricModel.h"
#include "geometry/GridModel.h"
#include "util/Vector.h"
#include "pml/unicycle.h"

struct UDWA_Trajectory
{
    Vec2 acc;
    Vector<Unicycle> ctrl;
    double geometricClearance;
    double gridClearance;
    double carrotDistance;
    double carrotAngle;
    double f;
    double f1;
    double f2;

    UDWA_Trajectory()
    {
        geometricClearance = 0;
        gridClearance = 0;
        carrotDistance = 0;
        carrotAngle = 0;
        f = 0;
        f1 = 0;
        f2 = 0;
        ctrl << Unicycle();
        ctrl << Unicycle();
    }
};


class UnicycleDWA
{
    int debug;
    const DynamicGeometricModel* dynamicGeometricModel;
    const GeometricModel* geometricModel;
    const GridModel* gridModel;
    Vec2 carrot;
    Pose2D target;
    Unicycle start;

    Vector<UDWA_Trajectory> trajectories;
    int bestTrajectoryId;

    int trajectoryType;

public:
    UnicycleDWA();
    ~UnicycleDWA();

    void setDebug(int d);
    void setDynamicGeometricModel(const DynamicGeometricModel &gm);
    void setGeometricModel(const GeometricModel &gm);
    void setGridModel(const GridModel &gm);
    void setCarrot(const Vec2 &wp);
    void setTarget(const Pose2D &wp);
    void setStart(const Unicycle &wp);
    void setTrajectoryType(int type);

    Vec2 search();

    void draw(QPainter *painter) const;
    void draw() const;
private:
    double heuristic_pathrtr(const Vector<Vec2> &path, const Pose2D &from, const Pose2D &to) const;
};

#endif // ARCDWA_H_

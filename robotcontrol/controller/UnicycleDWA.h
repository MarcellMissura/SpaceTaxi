#ifndef ARCDWA_H_
#define ARCDWA_H_
#include <QMutex>
#include <QPainter>
#include "lib/geometry/GeometricModel.h"
#include "lib/geometry/GridModel.h"
#include "lib/util/Vector.h"
#include "lib/pml/unicycle.h"

struct UDWA_Trajectory
{
    Vec2 acc;
    Vector<Unicycle> ctrl;
    double geometricClearance;
    double gridClearance;
    double carrotDistance;
    double f;
    double f1;
    double f2;

    UDWA_Trajectory()
    {
        geometricClearance = 0;
        gridClearance = 0;
        carrotDistance = 0;
        f = 0;
        f1 = 0;
        f2 = 0;
        ctrl << Unicycle();
        ctrl << Unicycle();
    }
};


class UnicycleDWA
{
    const GeometricModel* geometricModel;
    const GridModel* gridModel;
    Vec2 carrot;
    Unicycle start;

    Vector<UDWA_Trajectory> trajectories;
    int bestTrajectoryId;

    int trajectoryType;

public:
    UnicycleDWA();
    ~UnicycleDWA();

    void setGeometricModel(const GeometricModel &gm);
    void setGridModel(const GridModel &gm);
    void setCarrot(const Vec2 &wp);
    void setStart(const Unicycle &wp);
    void setTrajectoryType(int type);

    Vec2 search(bool debug=false);

    void draw(QPainter *painter) const;
    void draw() const;
};

#endif // ARCDWA_H_

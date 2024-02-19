#ifndef UNICYCLESEARCHNODE_H
#define UNICYCLESEARCHNODE_H

#include "lib/util/Vector.h"
#include "lib/pml/unicycle.h"
#include "lib/geometry/Path.h"
#include "lib/geometry/Polygon.h"
#include <QPainter>

class UnicycleSearchNode : public Unicycle
{
public:

    uint id;
    Vec2 action;
    uint depth;
    uint type;
    const UnicycleSearchNode* parent;
    bool closed;
    int collided;
    double g;
    double h;
    double f; // cost + heuristic

    Path path; // Needed only for visualization.
    Polygon safetyZone; // Needed only for visualization.

    UnicycleSearchNode();
    ~UnicycleSearchNode(){}

    void reset();
    void setDepth(uint d);
    uint getDepth();
    void propagate(const UnicycleSearchNode* u, const Vec2 &acc, double dt);
    UnicycleSearchNode flipped() const;

    Vector<UnicycleSearchNode> trace() const;
    Vec2 rootAction() const;

    // Stuff needed for priority queueing.
    uint pidx;
    bool cmp(const UnicycleSearchNode* n2) const {return (f < n2->f);}
    uint getPidx() const{return pidx;}
    void setPidx(uint k) {pidx = k;}

    void draw() const;
    void draw(QPainter* painter) const;
    void drawNoseCircle(QPainter* painter, double r=0.02, QBrush brush = Qt::NoBrush) const;
    void drawTrajectory(QPainter* painter) const;
};

QDebug operator<<(QDebug dbg, const UnicycleSearchNode &n);
QDebug operator<<(QDebug dbg, const UnicycleSearchNode* n);

#endif

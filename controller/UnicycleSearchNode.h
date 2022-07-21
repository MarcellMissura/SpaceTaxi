#ifndef UNICYCLESEARCHNODE_H
#define UNICYCLESEARCHNODE_H

#include "util/Vector.h"
#include "pml/unicycle.h"
#include "geometry/VisibilityGraph.h"
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
    char collided;
    double g;
    double h;
    double f; // cost + heuristic

    Vector<Vec2> path; // Needed only for visualization.

    UnicycleSearchNode();
    ~UnicycleSearchNode(){}

    void reset();
    void setDepth(uint d);
    uint getDepth();
    void propagate(const UnicycleSearchNode* u, const Vec2 &acc);
    UnicycleSearchNode flipped() const;

    Vector<UnicycleSearchNode> trace() const;
    Vec2 rootAction() const;

    // Stuff needed for priority queueing.
    uint pidx;
    bool cmp(const UnicycleSearchNode* n2) const {return (f < n2->f);}
    uint getPidx() const{return pidx;}
    void setPidx(uint k) {pidx = k;}

    void drawNoseCircle(QPainter* painter, double r=0.02, QBrush brush = Qt::NoBrush) const;
    void drawTrajectory(QPainter* painter) const;
};

QDebug operator<<(QDebug dbg, const UnicycleSearchNode &n);
QDebug operator<<(QDebug dbg, const UnicycleSearchNode* n);

#endif

#ifndef GridSearchNode_H_
#define GridSearchNode_H_

#include "util/Vec2u.h"
#include "util/Vec2.h"
#include <QPainter>

class GridSearchNode
{
public:

    GridSearchNode* parent;
    GridSearchNode* successor;
    bool closed;
    bool blocked;
    uint n;
    Vec2u stateIdx;
    Vec2 pos;
    double g;
    double h;
    double f; // cost + heuristic

    GridSearchNode();
    ~GridSearchNode(){}

    void reset();
    void draw(QPainter* painter) const;
    void draw() const;

    // Stuff needed for priority queueing.
    uint pidx;
    bool cmp(const GridSearchNode* n2) const {return (f < n2->f);}
    uint getPidx() const{return pidx;}
    void setPidx(uint k) {pidx = k;}
};

QDebug operator<<(QDebug dbg, const GridSearchNode &n);
QDebug operator<<(QDebug dbg, const GridSearchNode* n);

#endif

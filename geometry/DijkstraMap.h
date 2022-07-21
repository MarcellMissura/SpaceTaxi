#ifndef DIJKSTRAMAP_H
#define DIJKSTRAMAP_H
#include "GridSearchNode.h"
#include "util/Grid.h"
#include "util/Vec3.h"
#include "util/PriorityQueue.h"
#include "util/Vec2i.h"
#include <QPainter>

class GridModel;

class DijkstraMap : public Grid
{
    Vector< Vector<GridSearchNode> > nodes; // A set of nodes.
    Vec2i actions[8]; // The action set will be filled in the init method.
    double cost[8]; // Cost for each action, also filled in the init method.
    PriorityQueue<GridSearchNode*> q; // The priority queue.

    mutable Vector<Vec2> path;
    int debug;
    int expansions;

public:

    DijkstraMap();
    ~DijkstraMap(){}

    void setDebug(int d);

    void computeDijkstraMap(const GridModel& gridModel, const Vec2 &target);

    double dijkstraHeuristic(const Vec2& start) const;
    const Vector<Vec2> &dijkstraPath(const Vec2& start);
    const Vector<Vec2> &getPath() const;

    bool hasLineOfSight(const Vec2u& cellIdxA, const Vec2u& cellIdxB) const;
    void blockBorder();
    void setBlocked(const Vec2u& idx);
    bool isBlocked(const Vec2u& idx) const;

    void draw() const;
    void draw(QPainter* painter) const;
    void drawPath() const;
    void drawPath(QPainter* painter) const;
};

#endif

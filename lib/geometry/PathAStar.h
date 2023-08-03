#ifndef PATHASTAR_H_
#define PATHASTAR_H_
#include "GridModel.h"
#include "GridSearchNode.h"
#include "lib/util/PriorityQueue.h"
#include "lib/util/Vec2i.h"
#include <QPainter>

class PathAStar
{
    Vector<GridSearchNode> nodes; // A set of nodes.
    Vec2i actions[8]; // The action set will be filled in the init method.
    double cost[8]; // Cost for each action, also filled in the init method.
    PriorityQueue<GridSearchNode*> q; // The priority queue. Every A* needs one.
    GridSearchNode* bestScoreNode; // Pointer to the best score node.

    Vec2u startState;
    Vec2u targetState;

    const GridModel* gridModel;
    int debug;

public:
    int expansions;
    PathAStar();
    ~PathAStar(){}

public:
	void init();
    void reset();

    // Setting search parameters.
    void setDebug(int d);

    void setGridModel(const GridModel *gm);
    void setStartState(const Vec2 &p);
    void setTargetState(const Vec2 &p);
    bool aStarSearch();
    bool lazyThetaStarSearch();
    Vec2 getWaypoint(double d=0);
    Vector<Vec2> getPath() const;

    void draw(QPainter* painter) const;
};

#endif /* PATHASTAR_H_ */

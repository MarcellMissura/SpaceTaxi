#ifndef STABORTINGASTAR_H
#define STABORTINGASTAR_H

#include "UnicycleSearchNode.h"
#include "lib/geometry/GeometricModel.h"
#include "lib/geometry/GridModel.h"
#include "lib/geometry/DijkstraMap.h"
#include "lib/util/DataGrid.h"
#include "lib/util/LinkedList.h"
#include "lib/util/PriorityQueue.h"
#include "lib/util/StopWatch.h"

class ShortTermAbortingAStar
{
    // Start and target are always needed.
    UnicycleSearchNode startState;
    Pose2D targetState;

    bool stuck; // Stuckness mode indicator.

    // The output variates.
    UnicycleSearchNode* bestScoreNode;
    UnicycleSearchNode* bestHeuristicNode;
    UnicycleSearchNode* bestSolutionNode;
    UnicycleSearchNode* deepestNode;

    // Priority queue, open list, closed list, and action set.
    PriorityQueue<UnicycleSearchNode*> Q;
    LinkedList<UnicycleSearchNode> open; // The open list is mostly just for visualization and could be removed.
    DataGrid<uchar> closedMap;
    Vector<uint> closedList; // The closed list is only used to reset the closedMap faster.
    Vector<Vec2> actionSet;

    // Data structures for collision checking and heuristic evaluation.
    GridModel* sensedGrid; // not const because of init Dijkstra
    GeometricModel* localMap; // not const because of computeDynamicPath

    // Hull polygons for collision checking.
    Polygon hullPolygon;
    Polygon planPolygon;

    StopWatch stopWatch;

public:
    int expansions;
    int processed;
    int heuristicPathsComputed;
    int collided;
    int closed; // How many states have been discarded by the closed grid.
    int dried; // How many times the queue has ran out.
    int opened;
    int depth;
    bool finished; // Has the A* finished?
    double score;
    double executionTime;
    int trajectoryType;
    int heuristicType;
    double timeLimit;

    ShortTermAbortingAStar();
    ~ShortTermAbortingAStar(){}

    void init();
    void reset();

    // Start and target state.
    void setStartState(const Unicycle& uni);
    void setTargetState(const Pose2D& target);
    const Pose2D& getTargetState() const;

    // Grid and geometric models.
    void setGridModel(GridModel &gm);
    void setGeometricModel(GeometricModel &gm);

    bool aStarSearch(int debug=0);

    // Analytic functions.
    Vec2 getAction() const;
    Vector<UnicycleSearchNode> getPlan() const;
    double getExecutionTime() const;
    bool isFinished();
    int getExpansions() const;
    int getOpenListSize() const;
    int getQSize() const;
    int getPathsComputed() const;

    // QPainter visualization.
    void draw(QPainter* painter) const;
    void drawVisibilityGraph(QPainter *painter) const;
    void draw() const;

    void setStuck(bool value);

private:
    double heuristic(const Pose2D& from, const Pose2D &to);
    double heuristic_euklidean(const Pose2D& from, const Pose2D &to) const;
    double heuristic_patheuklidean(const Vector<Vec2> &path, const Pose2D& from, const Pose2D &to) const;
    double heuristic_rtr(const Pose2D& from, const Pose2D &to) const;
    double heuristic_rtr_min(const Pose2D& from, const Pose2D &to) const;
    double heuristic_rtr_max(const Pose2D& from, const Pose2D &to) const;
    double heuristic_pathrtr(const Vector<Vec2> &path, const Pose2D& from, const Pose2D &to) const;
    bool collisionCheck(const UnicycleSearchNode &u) const;
    void computeActionSet();
};

#endif
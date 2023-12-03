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
    Polygon hullPolygon;
    Polygon planPolygon;
    GridModel* sensedGrid; // not const because of init Dijkstra
    GeometricModel* localMap; // not const because of computeDynamicPath

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

    void setTimeLimit(double tl);

    // Grid and geometric models.
    void setGridModel(GridModel &gm);
    void setGeometricModel(GeometricModel &gm);

    // The search itself.
    bool aStarSearch(int debug=0);

    void setStuck(bool value);

    // Analytic functions.
    Vec2 getAction() const;
    Vector<UnicycleSearchNode> getPlan() const;
    double getExecutionTime() const;
    bool isFinished();
    int getExpansions() const;
    int getOpenListSize() const;
    int getQSize() const;
    int getPathsComputed() const;

    // Visualization.
    void draw(QPainter* painter) const;
    void draw() const;

private:
    void computeActionSet();
    Polygon getSafetyPolygon(double vel) const;
    bool collisionCheck(const UnicycleSearchNode &u) const;
    double heuristic(const UnicycleSearchNode& from, const Pose2D &to);
    double heuristic_euklidean(const Pose2D& from, const Pose2D &to) const;
    double heuristic_patheuklidean(const Path &path, const Pose2D& from, const Pose2D &to) const;
    double heuristic_rtr(const Pose2D& from, const Pose2D &to, bool debug=false) const;
    double heuristic_dock_rtr(const Pose2D& from, const Pose2D &to, bool debug=false) const;
    double heuristic_rtr_min(const Pose2D& from, const Pose2D &to) const;
    double heuristic_rtr_max(const Pose2D& from, const Pose2D &to) const;
    double heuristic_pathrtr(const Path &path, const Pose2D& from, const Pose2D &to) const;
};

#endif

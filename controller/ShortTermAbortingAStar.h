#ifndef STABORTINGASTAR_H
#define STABORTINGASTAR_H

#include "geometry/GeometricModel.h"
#include "geometry/DynamicGeometricModel.h"
#include "geometry/GridModel.h"
#include "geometry/DijkstraMap.h"
#include "UnicycleSearchNode.h"
#include "util/DataGrid.h"
#include "util/LinkedList.h"
#include "util/PriorityQueue.h"
#include "util/StopWatch.h"

class ShortTermAbortingAStar
{
    // Start and target are always needed.
    UnicycleSearchNode startState;
    Pose2D targetState;

    bool stuck;

    // The output variates.
    UnicycleSearchNode* bestScoreNode;
    UnicycleSearchNode* bestHeuristicNode;
    UnicycleSearchNode* bestSolutionNode;
    UnicycleSearchNode* deepestNode;

    // Priority queue, open list, closed list, and action set.
    PriorityQueue<UnicycleSearchNode*> Q;
    LinkedList<UnicycleSearchNode> open;
    DataGrid<uchar> closedMap;
    Vector<uint> closedList;
    Vector<Vec2> actionSet;

    // Data structures for various purposes.
    GridModel* sensedGrid;
    GridModel* dilatedSensedGrid;
    const GeometricModel* staticGeometricModel;
    const GeometricModel* expandedStaticGeometricModel;
    const DynamicGeometricModel* dynamicGeometricModel;
    const DynamicGeometricModel* expandedDynamicGeometricModel;
    Vector<GeometricModel> predictedUnifiedModels;
    Vector<DynamicGeometricModel> predictedDynamicModels;
    DynamicGeometricModel erasedDynamicModel;
    Polygon hullPolygon;
    Polygon planPolygon;

    StopWatch stopWatch;
    int debug;

public:
    int expansions;
    int processed;
    int heuristicPathsComputed;
    int collided;
    int closed; // How many states have been discarded by the closed grid.
    int dried; // How many times the queue has ran out.
    bool finished; // Has the A* finished?
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
    void setSensedGrid(GridModel &gm);
    void setDilatedSensedGrid(GridModel &gm);
    void setStaticGeometricModel(const GeometricModel &gm);
    void setDynamicGeometricModel(const DynamicGeometricModel &gm);
    void setExpandedStaticGeometricModel(const GeometricModel &gm);
    void setExpandedDynamicGeometricModel(const DynamicGeometricModel &gm);

    bool aStarSearch();

    // Analytic functions.
    Vec2 getAction() const;
    Vector<UnicycleSearchNode> getPlan() const;
    double getExecutionTime() const;
    bool isFinished() const;
    int getExpansions() const;
    int getOpenListSize() const;
    int getQSize() const;
    int getPathsComputed() const;
    void setDebug(int d);

    // QPainter visualization.
    void draw(QPainter* painter) const;
    void drawVisibilityGraph(QPainter *painter) const;
    void draw() const;

    bool getStuck() const;
    void setStuck(bool value);

private:
    double heuristic(const Pose2D& from, const Pose2D &to, int depth);
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

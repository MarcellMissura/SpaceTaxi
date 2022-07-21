#ifndef UNICYCLEAGENT_H_
#define UNICYCLEAGENT_H_
#include "UnicycleObstacle.h"
#include "geometry/GeometricModel.h"
#include "geometry/DynamicGeometricModel.h"
#include "geometry/GridModel.h"
#include "geometry/PathAStar.h"
#include "geometry/VisibilityGraph.h"
#include "controller/UnicycleDWA.h"
#include "controller/ShortTermAbortingAStar.h"
#include "controller/RuleBase.h"

class UnicycleAgent : public UnicycleObstacle
{
public:

    int agentId;

    // Input
    int trajectoryPlanningMethod;
    int trajectoryType;
    int heuristicType;
    int predictionType;
    Vector<Vec2> worldDropOffPoints;
    Vector<Obstacle> worldStaticObstacles;
    GeometricModel worldExpandedStaticObstacles;
    Vector<UnicycleObstacle> worldUnicycleObstacles;

    Vector<uint> dropOffPointQueue;
    uint targetDropOffId; // The id of the target drop off point.
    Vec2 mainTarget; // The world coordinates of the main target.
    Pose2D intermediateTarget; // The local coordinates of the intermediate target at the boundary of the sensed grid.
    Pose2D carrot; // Nearby target in local coordinates that steers dumb controllers.
    Vec2 joystickCarrot;

    // Reflex controller variables.
    int stuckDetectionCounter;
    int stuckTimer;
    bool isStuck;
    bool isTooClose;
    Vec2 awayFromObst;
    bool inCollision;
    bool collided;

    Vector<Vec2> staticWorldPath;
    Vector<Vec2> dynamicPath;
    bool staticWorldPathSuccess;
    bool dynamicPathSuccess;
    bool trajectorySuccess;

    uint score;
    uint collisions;
    uint stucks;
    uint closes;
    double milage;
    double pathTime;
    double trajectoryTime;

    // Grid, geometric, and ray models.
    GridModel sensedGrid;
    GridModel dilatedSensedGrid;
    GeometricModel staticGeometricModel;
    GeometricModel expandedStaticGeometricModel;
    DynamicGeometricModel dynamicGeometricModel;
    DynamicGeometricModel expandedDynamicGeometricModel;
    GeometricModel unifiedGeometricModel;
    Vector<double> rays;

    // Path and motion controllers.
    UnicycleDWA unicycleDWA;
    ShortTermAbortingAStar shortTermAbortingAStar;
    RuleBase ruleBase;

    Polygon hullPolygon;
    Vec2 cp1;
    Vec2 cp2;

public:

    UnicycleAgent();
    ~UnicycleAgent(){}

    virtual const QString getName() const;
    void setAgentId(int agentId);
    int getAgentId() const;
    bool isBot() const;
    bool isFirstAgent() const;

    void init(const Vec2 &p);
    void step();
    void sense();
    void act();
    void learn();

    void collisionResponse(const Obstacle* o);

    void draw(QPainter* painter) const;
    void drawPlan(QPainter* painter) const;

    void setWorldDropOffPoints(const Vector<Vec2> &value);
    void setWorldExpandedStaticObstacles(const Vector<Obstacle> &value);
    void setWorldStaticObstacles(const Vector<Obstacle> &value);
    void setWorldUnicycleObstacles(const Vector<UnicycleObstacle> &value);

    void setParams(int trajectoryPlanningMethod, int trajectoryType, int predictionType, int heuristicType, uint frequency=0);

    bool intersects(const Polygon &p) const;

private:
    void computeRayModel();
    Vec2 pdControlTo(const Vec2& v) const;
    Vec2 forceReaction(const Vec2& v) const;
};

QDebug operator<<(QDebug dbg, const UnicycleAgent& o);
QDebug operator<<(QDebug dbg, const UnicycleAgent* o);

#endif

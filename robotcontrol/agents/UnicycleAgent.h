#ifndef UNICYCLEAGENT_H_
#define UNICYCLEAGENT_H_
#include "UnicycleObstacle.h"
#include "lib/geometry/GeometricModel.h"
#include "lib/geometry/GridModel.h"
#include "robotcontrol/perception/LaserSensor.h"
#include "robotcontrol/controller/UnicycleDWA.h"
#include "robotcontrol/controller/ShortTermAbortingAStar.h"
#include "robotcontrol/controller/RuleBase.h"

class UnicycleAgent : public UnicycleObstacle
{
public:

    int agentId;
    //Polygon hullPolygon; // A bit bigger than the actual geometry of the agent.

    // Motion control parameters.
    int trajectoryPlanningMethod; // PD, DWA, STAA* or RuleBase.
    int trajectoryType; // Circle, B0 or Fresnel
    int heuristicType; // PathRTR, Dijkstra, Euklidean
    int predictionType; // Unicycle, Holonomic or none.

    // World Map
    GeometricModel worldMap; // The assumed map the robot has built for itself.
    GeometricModel clippedWorldMap; // The world map clipped to the local map.
    Vector<Vec2> dropOffPoints; // The targets the robots need to drive to.
    Vector<UnicycleObstacle> worldDynamicObstacles; // Cheat access to the moving objects in the world.
    GeometricModel worldPolygons; // Cheat access to the world polygons used only for the simulation of the laser rays.

    // Local map sensing. Grid, geometric, and ray models.
    LaserSensor laserSensor; // Simulated laser rays.
    Polygon visibilityPolygon; // The awesome visibility polygon.
    GridModel occupanyGrid; // First line grid sensing from the laser points.
    GridModel sensedGrid; // Local occupancy grid and costmap computed from the sensor input.
    GeometricModel localMap;
    Vector<double> rays;

    // Navigation handles.
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
    Vec2 stucknessImpulse;
    bool ebActive;
    double ebA;
    Vec2 impactPoint;
    bool inCollision;
    bool collided;

    // Path stuffs.
    Vector<Vec2> worldPath;
    Vector<Vec2> dynamicPath;
    Vector<Vec2> staticPath;
    bool worldPathSuccess;
    bool dynamicPathSuccess;
    bool staticPathSuccess;
    bool trajectorySuccess;

    // Motion controllers.
    UnicycleDWA unicycleDWA;
    ShortTermAbortingAStar shortTermAbortingAStar;
    RuleBase ruleBase;

    // Statistics.
    uint score;
    uint collisions;
    uint stucks;
    uint closes;
    double milage;
    double pathTime;
    double trajectoryTime;

public:

    UnicycleAgent();
    ~UnicycleAgent(){}

    virtual const QString getName() const;
    void setAgentId(int agentId);
    int getAgentId() const;
    bool isBot() const;
    bool isFirstAgent() const;

    void init(const Vec2 &initialPos);
    void step();
    void sense();
    void act();
    void learn();

    void collisionResponse(const Obstacle* o);

    void draw(QPainter* painter) const;
    void drawBot(QPainter* painter) const;

    void setWorldDropOffPoints(const Vector<Vec2> &dops);
    void setWorldMap(const GeometricModel &pols);
    void setWorldPolygons(const GeometricModel &pols);
    void setWorldUnicycleObstacles(const Vector<UnicycleObstacle> &value);

    void setParams(int trajectoryPlanningMethod, int trajectoryType, int predictionType, int heuristicType, uint frequency=0);

private:
    void computeRayModel();
    void simulateLaserSensor();
    Vec2 pdControlTo(const Vec2& v) const;
    Vec2 forceReaction(const Vec2& v) const;
};

QDebug operator<<(QDebug dbg, const UnicycleAgent& o);
QDebug operator<<(QDebug dbg, const UnicycleAgent* o);

#endif

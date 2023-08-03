#ifndef WORLD_H_
#define WORLD_H_

#include "robotcontrol/agents/Agent.h"
#include "robotcontrol/agents/UnicycleAgent.h"
#include "lib/util/Vector.h"
#include "lib/Box2D/Box2D.h"

class World : public b2ContactListener
{
    b2World box2DSim; // Box2D physical simulation (box2d.org)
    static QMutex mutex;

public:

    double width;
    double height;
    int mapId;
    int numAgents;
    double simulationTimeStep;

    GeometricModel polygons; // Convex polygons that make up a map.
    GeometricModel worldMap; // inflated world obstacles
    Vector<UnicycleAgent> unicycleAgents;
    Vector<Vec2> dropOffPoints;

    void setParams(int trajectoryPlanningMethod, int trajectoryType, int predictionType, int heuristicType, uint frequency);

public:

    World();
    ~World(){}

    void init();
    void reset();
    void step();
    void stepAgents();

    void setMap(int m=1, int numAgents=1);
    QString getMapName(uint map) const;

    const Vector<Polygon> &getStaticObstacles() const;
    Vector<UnicycleObstacle> getUnicycleObstacles(int excludeId=-1) const;
    const Vector<Vec2>& getDropOffPoints() const;

    void physicsTransformIn();
    void physicsTransformOut();
    void physicsControl();

    void logObstaclesToTxt();

    void draw(QPainter *painter) const;
    void drawBackground(QPainter *painter) const;

private:

    void BeginContact(b2Contact* contact);
    void EndContact(b2Contact *contact);
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold){}
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse){}

    void addAgents(uint numAgents=1);
    void buildSimulation();
    void buildVoid();
    void buildSimple();
    void buildUTrap();
    void buildApartment();
    void buildTunnel();
    void buildOffice();
    void buildWarehouse();
    void buildClutter();
    void buildFromFile();

};

QDebug operator<<(QDebug dbg, const World &w);
QDataStream& operator<<(QDataStream& out, const World &w);
QDataStream& operator>>(QDataStream& in, World &w);

#endif

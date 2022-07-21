#ifndef WORLD_H_
#define WORLD_H_

#include "agents/Agent.h"
#include "agents/UnicycleAgent.h"
#include "util/Vector.h"
#include "Box2D/Box2D.h"

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

    GeometricModel polygons;
    GeometricModel staticObstacles; // core map obstacles
    GeometricModel expandedStaticObstacles; // minimally grown map obstacles by inradius
    Vector<UnicycleAgent> unicycleAgents;
    Vector<Vec2> dropOffPoints;

    void setParams(int trajectoryPlanningMethod, int trajectoryType, int predictionType, int heuristicType, uint frequency=0);

public:

    World();
    ~World(){}

    void init();
    void reset();
    void step();
    void stepAgents();

    void save() const;
    void load();

    void setMap(int m=1, int agents=1);
    QString getMapName(uint map) const;

    const Vector<Obstacle> &getStaticObstacles() const;
    Vector<UnicycleObstacle> getUnicycleObstacles(int excludeId=-1) const;
    const Vector<Vec2>& getDropOffPoints() const;

    void BeginContact(b2Contact* contact);
    void EndContact(b2Contact *contact);
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold){}
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse){}

    void physicsTransformIn();
    void physicsTransformOut();
    void physicsControl();

    void logObstaclesToTxt();

    void preDraw(QPainter *painter) const;
    void draw(QPainter *painter) const;

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream& in);

private:

    void addAgents();
    void buildSimulation();
    void buildVoid();
    void buildSimple();
    void buildUTrap();
    void buildOutdoor();
    void buildApartment();
    void buildWarehouse();
    void buildOffice();
    void buildClutter();
    void buildFromFile();

};

QDebug operator<<(QDebug dbg, const World &w);
QDataStream& operator<<(QDataStream& out, const World &w);
QDataStream& operator>>(QDataStream& in, World &w);

#endif

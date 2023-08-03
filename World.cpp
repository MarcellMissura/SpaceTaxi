#include "World.h"
#include "globals.h"
#include "blackboard/Command.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "lib/util/Statistics.h"
#include "lib/util/ColorUtil.h"
#include "lib/util/Vec2.h"
#include "lib/util/StopWatch.h"
#include "lib/util/Logger.h"
#include "lib/geometry/Box.h"
#include <QFile>
#include <QDataStream>

// The world object acts as central repository for all agents
// and obstacles. The world also contains a Box2D simulation
// for physical simulation of the scene. The world adds all the
// objects to the simulation, steps the simulation, and transfers
// the new state of the objects from the simulation into their
// representation inside this class. The world registers itself
// (is a) contact listener of the Box2D simulation so that collision
// events can be handeled in the BeginContact() and EndContact()
// methods. The world offers an interface to the objects and
// returns them in their most recent state. Since so many
// components want to see different parts of the world, and the gui
// needs access to everything for the visualization, the world is
// placed onto the State blackboard where it's globally accessible
// and everything in the world is public for easy access.
// The world can rebuild itself in different configurations. Use
// the setMap(mapId) function to instantly switch to a new map.

QMutex World::mutex;

World::World() : b2ContactListener()
{
    width = 10; // width of the world in meters, will be overwritten when loading a map
    height = 10; // width of the world in meters, will be overwritten when loading a map
    mapId = 3;
    numAgents = 1;
    simulationTimeStep = 1.0/command.frequency; // Default 10 Hz.
    box2DSim.SetContactListener(this);
}

void World::init()
{
    state.world.setParams(command.trajectoryPlanningMethod, command.trajectoryType, command.predictionType, command.heuristic, command.frequency);
    reset();
}

// Rebuilds the world from scratch using the currently set mapId and number of agents parameter.
void World::reset()
{
    //QMutexLocker locker(&mutex);

    // Clear all data structures.
    polygons.clear(); // The raw polygons the map is made of.
    worldMap.clear(); // The expanded static obstacles as percieved by the agents.
    unicycleAgents.clear(); // The moving unicycle agents.
    dropOffPoints.clear(); // The drop off points (pois).

    // Reset the obstacle ids.
    // This is so that when a new map is built, the ids are counted again from 0.
    Polygon::resetId();

    // Build the map. These build routines fill the "polygons" GeometricModel
    // with convex polygons that are generated programmatically. The convexity
    // is important for the Box2D simulation.

    if (mapId == 0)
        buildVoid();
    else if (mapId == 1)
        buildSimple();
    else if (mapId == 2)
        buildUTrap();
    else if (mapId == 3)
        buildTunnel();
    else if (mapId == 4)
        buildApartment();
    else if (mapId == 5)
        buildOffice();
    else if (mapId == 6)
        buildWarehouse();
    else if (mapId == 7)
        buildClutter();

    polygons.transform(); // The static polygons of the map come out transformed.

    // Compute a dilated world map from the union of the polygons.
    // This is the map the robot builds for itself.
    worldMap.worldImport(polygons.getPolygons());

    // Add moving agents to the map.
    // The number of agents added depends on a configuration variable.
    addAgents(numAgents);

    // Build the Box2D simulation using the generated obstacles.
    buildSimulation();
}

// An obstacle free environment.
void World::buildVoid()
{
    width = 10;
    height = 15;

    // drop off points
    Vec2 b(0,config.voidRadius*qMin(width,height));
    Vec2 m(width/2, height/2);
    for (double phi = 0; phi < PII; phi += PII/config.voidDropOffPoints)
    {
        Vec2 v = b.rotated(phi);
        dropOffPoints.push_back(m+v);
    }

    // Special experiment setup.
    dropOffPoints.clear();
    dropOffPoints << Vec2(1, 7.5) << Vec2(5, 5.5) << Vec2(9, 7.5) << Vec2(5, 11.5);
}

// A very simple setting with two drop off points and one obstacle in the middle.
void World::buildSimple()
{
    width = 20;
    height = 20;

    // drop off points
    //dropOffPoints.push_back(Vec2(width/2, height/5));
    //dropOffPoints.push_back(Vec2(width/2, 4*height/5));

    // drop off points
    Vec2 b(0,config.simpleRadius*qMin(width,height));
    Vec2 m(width/2, height/2);
    for (double phi = 0; phi < PII; phi += PII/config.simpleDropOffPoints)
    {
        Vec2 v = b.rotated(phi);
        dropOffPoints.push_back(m+v);
    }

    double thickness = 0.1;
    double dx,dy,x,y;

    // Outer walls.
    dx = thickness;
    dy = width/2;
    x = height/2;
    y = 0;
    Polygon p1(x, y, dx, dy);
    p1.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p1); // right

    dx = thickness;
    dy = width/2;
    x = -height/2;
    y = 0;
    Polygon p2(x, y, dx, dy);
    p2.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p2); // left

    dx = height/2;
    dy = thickness;
    x = 0;
    y = -width/2;
    Polygon p3(x, y, dx, dy);
    p3.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p3); // bottom

    dx = height/2;
    dy = thickness;
    x = 0;
    y = width/2;
    Polygon p4(x, y, dx, dy);
    p4.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p4); // top

    polygons.translate(width/2, height/2);
    polygons.transform();


    // The polygon in the middle.
    Polygon ob;
    ob << Vec2(-10, 1.5)
       << Vec2(-10, -1.5)
       << Vec2(-7, -3)
       << Vec2(7, -3)
       << Vec2(10, -1.5)
       << Vec2(10, 1.5)
       << Vec2(7, 5)
       << Vec2(-7, 5);
    ob.scale(0.5, 0.5);
    ob.setPos(width/2, height/2);
    polygons.addPolygon(ob);
}

// A locality trap with a U shaped obstacle.
void World::buildUTrap()
{
    width = 15;
    height = 15;

    // drop off points
    dropOffPoints.push_back(Vec2(width/2, height/2));
    dropOffPoints.push_back(Vec2(width/2, height/2-4.5));

    double thickness = 0.1;
    double dx,dy,x,y;

    // Outer walls.
    dx = thickness;
    dy = width/2;
    x = height/2;
    y = 0;
    Polygon p1(x, y, dx, dy);
    p1.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p1); // right

    dx = thickness;
    dy = width/2;
    x = -height/2;
    y = 0;
    Polygon p2(x, y, dx, dy);
    p2.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p2); // left

    dx = height/2;
    dy = thickness;
    x = 0;
    y = -width/2;
    Polygon p3(x, y, dx, dy);
    p3.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p3); // bottom

    dx = height/2;
    dy = thickness;
    x = 0;
    y = width/2;
    Polygon p4(x, y, dx, dy);
    p4.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p4); // top

    polygons.translate(width/2, height/2);
    polygons.transform();

    // U Polygons
    double uOffset = -2.5;
    double w = 0.5;
    double h = 3.5;

    x = width/2;
    y = height/2+uOffset;
    polygons.addPolygon(Polygon(x, y, h, w));

    x = width/2+h-w;
    y = height/2+h-w+uOffset;
    polygons.addPolygon(Polygon(x, y, w, h));

    x = width/2-h+w;
    y = height/2+h-w+uOffset;
    polygons.addPolygon(Polygon(x, y, w, h));
}

void World::buildApartment()
{
    width = 10;
    height = 12;
    double thickness = 0.1;

    // Docking ports.

    // drop off points
    dropOffPoints.push_back(Vec2(0.8, 3.3));
    dropOffPoints.push_back(Vec2(7.4, 3.8));
    dropOffPoints.push_back(Vec2(10.5, 3.8));
    dropOffPoints.push_back(Vec2(2.6, 3.2));
    dropOffPoints.push_back(Vec2(10.5, 10.5));
    dropOffPoints.push_back(Vec2(3.4, 10.7));
    dropOffPoints.push_back(Vec2(6.3, 10));
    dropOffPoints.push_back(Vec2(0, 10.5));
    dropOffPoints.push_back(Vec2(0, 14));
    dropOffPoints.push_back(Vec2(3, 14));
    dropOffPoints.push_back(Vec2(6, 14));
    dropOffPoints.push_back(Vec2(9, 14));
    dropOffPoints.push_back(Vec2(12, 14));
    dropOffPoints.push_back(Vec2(15, 14));
    dropOffPoints.push_back(Vec2(1, 17)); // 10
    dropOffPoints.push_back(Vec2(1, 19));
    dropOffPoints.push_back(Vec2(4.5, 17.6));
    dropOffPoints.push_back(Vec2(6.5, 20.5));
    dropOffPoints.push_back(Vec2(7.7, 18.1));
    dropOffPoints.push_back(Vec2(2.6, 18.3));
    dropOffPoints.push_back(Vec2(16, 20.1));
    dropOffPoints.push_back(Vec2(14, 18));
    dropOffPoints.push_back(Vec2(16, 18));
    dropOffPoints.push_back(Vec2(18, 18));
    dropOffPoints.push_back(Vec2(18.06, 14.04));
    dropOffPoints.push_back(Vec2(14.07, 11));
    dropOffPoints.push_back(Vec2(18.8, 10.04));
    dropOffPoints.push_back(Vec2(14.4, 6.1));
    dropOffPoints.push_back(Vec2(18.1, 4.6));
    dropOffPoints.push_back(Vec2(14.1, 4.5));
    dropOffPoints.push_back(Vec2(15.8, 5.3));
    dropOffPoints.push_back(Vec2(20.9, 5.0));

    double dx,dy,x,y;

    // Outer walls.
    dx = thickness;
    dy = width/2;
    x = height/2;
    y = 0;
    Polygon p1(x, y, dx, dy);
    p1.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p1); // right

    dx = thickness;
    dy = width/2;
    x = -height/2;
    y = 0;
    Polygon p133(x, y, dx, dy);
    p133.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p133); // left

    dx = height/2;
    dy = thickness;
    x = 0;
    y = -width/2;
    Polygon p3(x, y, dx, dy);
    p3.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p3);

    dx = height/2;
    dy = thickness;
    x = 0;
    y = width/2;
    Polygon p4(x, y, dx, dy);
    p4.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p4);

    // Living room.
    dx = 1;
    dy = thickness;
    x = -height/2+dx;
    y = 0;
    Polygon p5(x, y, dx, dy);
    p5.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p5);

    dx = 1.75;
    dy = thickness;
    x = -0.75;
    y = 0;
    Polygon p6(x, y, dx, dy);
    p6.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p6);

    dx = thickness;
    dy = width/4;
    x = 1;
    y = -width/4;
    Polygon p7(x, y, dx, dy);
    p7.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p7);

    // Kitchen
    dx = 0.5;
    dy = thickness;
    x = -height/2+dx;
    y = 2;
    Polygon p8(x, y, dx, dy);
    p8.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p8);

    dx = 3;
    dy = thickness;
    x = -0.8;
    y = 2;
    Polygon p9(x, y, dx, dy);
    p9.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p9);

    dx = thickness;
    dy = 1.5;
    x = 0;
    y = 3.5;
    Polygon p10(x, y, dx, dy);
    p10.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p10);

    dx = thickness;
    dy = 0.85;
    x = -1.8;
    y = 2.85;
    Polygon p11(x, y, dx, dy);
    p11.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p11);


    dx = 1.3;
    dy = thickness;
    x = height/2-dx;
    y = 2;
    Polygon p12(x, y, dx, dy);
    p12.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p12);

    dx = 1.3;
    dy = thickness;
    x = height/2-dx;
    y = 0;
    Polygon p18(x, y, dx, dy);
    p18.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p18);

    dx = 1.7;
    dy = thickness;
    x = height/2-dx;
    y = -2;
    Polygon p13(x, y, dx, dy);
    p13.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p13);

    dx = 0.3;
    dy = thickness;
    x = 1.3;
    y = -2;
    Polygon p14(x, y, dx, dy);
    p14.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p14);

    dx = thickness;
    dy = 0.5;
    x = 3.5;
    y = 0;
    Polygon p15(x, y, dx, dy);
    p15.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p15);

    dx = thickness;
    dy = 0.25;
    x = 3.5;
    y = 1.75;
    Polygon p16(x, y, dx, dy);
    p16.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p16);

    dx = thickness;
    dy = 0.25;
    x = 3.5;
    y = -1.75;
    Polygon p17(x, y, dx, dy);
    p17.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p17);

    // Couch 1
    dx = 0.45;
    dy = 1.0;
    x = -2.6;
    y = -3.5;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    // Couch 2
    dx = 1.0;
    dy = 0.45;
    x = -3.6;
    y = -1.8;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    // TV
    dx = 0.1;
    dy = 0.8;
    x = -5.5;
    y = -3.7;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    // Couch table
    dx = 0.4;
    dy = 0.4;
    x = -4.2;
    y = -3.5;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    // Dining table
    dx = 0.5;
    dy = 0.8;
    x = -0.5;
    y = -2.5;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    // Chairs
    dx = 0.2;
    dy = 0.2;
    x = -0.5;
    y = -2.5 + 1.1;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = -0.5;
    y = -2.5 - 1.1;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = -0.5 + 0.8;
    y = -2.5 + 0.4;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = -0.5 + 0.8;
    y = -2.5 - 0.4;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = -0.5 - 0.8;
    y = -2.5 + 0.4;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = -0.5 - 0.8;
    y = -2.5 - 0.4;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    // Kitchen line
    dx = 0.35;
    dy = 1.35;
    x = -5.6;
    y = 3.4;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 1.35;
    dy = 0.35;
    x = -4.6;
    y = 4.6;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    // Kitchen shelf
    dx = 0.1;
    dy = 0.6;
    x = -2.05;
    y = 3.0;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.6;
    dy = 0.1;
    x = -2.6;
    y = 2.25;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    // Storage room shelf
    dx = 0.2;
    dy = 1.4;
    x = -0.3;
    y = 3.5;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.5;
    dy = 0.15;
    x = -1.0;
    y = 2.25;
    polygons.addPolygon(Polygon(x, y, dx, dy));


    // Beds
    dx = 1.0;
    dy = 0.45;
    x = 1.1;
    y = 4.5;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 1.0;
    dy = 0.45;
    x = 4.9;
    y = 4.5;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 1.0;
    dy = 0.45;
    x = 4.85;
    y = -2.6;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    // Desks with chairs
    dx = 0.3;
    dy = 0.6;
    x = 0.5;
    y = 3.0;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = 1.1;
    y = 3.0;
    polygons.addPolygon(Polygon(x, y, dx, dy));


    dx = 0.3;
    dy = 0.6;
    x = 5.6;
    y = 3.0;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = 5.0;
    y = 3.0;
    polygons.addPolygon(Polygon(x, y, dx, dy));


    dx = 0.6;
    dy = 0.3;
    x = 5.0;
    y = -4.6;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = 5.0;
    y = -4.0;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    // More shelves.
    dx = 0.5;
    dy = 0.14;
    x = 3.2;
    y = -2.3;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.2;
    dy = 1.0;
    x = 1.35;
    y = -3.5;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.7;
    dy = 0.3;
    x = 3.0;
    y = -4.5;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.15;
    dy = 0.7;
    x = 1.3;
    y = -1.0;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    // Bathroom stuff.
    dx = 0.2;
    dy = 0.2;
    x = 5.7;
    y = -0.5;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.4;
    dy = 0.4;
    x = 5.5;
    y = -1.5;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.3;
    dy = 0.15;
    x = 4.4;
    y = -1.7;
    polygons.addPolygon(Polygon(x, y, dx, dy));


    dx = 0.2;
    dy = 0.2;
    x = 4.8;
    y = 1.7;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.3;
    dy = 0.7;
    x = 5.6;
    y = 1.0;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    dx = 0.3;
    dy = 0.15;
    x = 4.4;
    y = 0.3;
    polygons.addPolygon(Polygon(x, y, dx, dy));

    polygons.translate(width/2, height/2);
    polygons.transform();
    polygons.scale(2);

    Box bb = polygons.getBoundingBox();
    polygons.translate(-bb.bottomLeft());
    polygons.transform();
    for (uint i = 0; i < dropOffPoints.size(); i++)
        dropOffPoints[i] -= bb.bottomLeft();
    width = bb.width();
    height = bb.height();
}

// An indoor level with narrow doorways and corridors.
void World::buildTunnel()
{
    width = 75;
    height = 60;

    // Drop off points
    dropOffPoints.push_back(Vec2(width/6, height/5));
    dropOffPoints.push_back(Vec2(width/6, 4*height/5));

    dropOffPoints.push_back(Vec2(3*width/6, height/5));
    dropOffPoints.push_back(Vec2(3*width/6, 4*height/5));

    dropOffPoints.push_back(Vec2(5*width/6, height/5));
    dropOffPoints.push_back(Vec2(5*width/6, 4*height/5));

    // Polygons
    double w,h,x,y;

    // Bottom row of inner walls.
    w = 1;
    h = height/4-4-config.tunnelCorridorWidth/4;
    x = width/3;
    y = h;
    polygons.addPolygon(Polygon(x, y, w, h));

    w = 1;
    h = height/4-4-config.tunnelCorridorWidth/4;
    x = 2*width/3;
    y = h+8+1;
    polygons.addPolygon(Polygon(x, y, w, h));


    // Top row of inner walls.
    w = 1;
    h = height/4-4-config.tunnelCorridorWidth/4;
    x = width/3;
    y = height-h;
    polygons.addPolygon(Polygon(x, y, w, h));

    w = 1;
    h = height/4-4-config.tunnelCorridorWidth/4;
    x = 2*width/3;
    y = height-h-9;
    polygons.addPolygon(Polygon(x, y, w, h));


    // corridor bottom wall
    w = width/2-7;
    h = 1;
    x = w;
    y = height/2-config.tunnelCorridorWidth/2;
    polygons.addPolygon(Polygon(x, y, w, h));

    // corridor top wall
    w = width/2-7;
    h = 1;
    x = w+14;
    y = height/2+config.tunnelCorridorWidth/2;
    polygons.addPolygon(Polygon(x, y, w, h));


    // bottom wall
    w = width/2;
    h = 1;
    x = w;
    y = h;
    polygons.addPolygon(Polygon(x, y, w, h));

    // top wall
    w = width/2;
    h = 1;
    x = w;
    y = height-h;
    polygons.addPolygon(Polygon(x, y, w, h));

    // left wall
    w = 1;
    h = height/2;
    x = w;
    y = h;
    polygons.addPolygon(Polygon(x, y, w, h));

    // right wall
    w = 1;
    h = height/2;
    x = width-w;
    y = h;
    polygons.addPolygon(Polygon(x, y, w, h));

    double scale = 0.24;
    polygons.transform();
    polygons.scale(scale);
    polygons.transform();
    for (uint i = 0; i < dropOffPoints.size(); i++)
        dropOffPoints[i] *= scale;
    width *= scale;
    height *= scale;
}

// Builds an office building like map.
void World::buildOffice()
{
    double roomSize = config.officeRoomSize; // width and height of one room in meters
    double wallThickness = config.officeWallThickness; // in percent of one unit
    double doorWidth = qMax(0.0, qMin(config.officeDoorWidth, 1.0-4.0*wallThickness)); // in percent of one unit
    int rooms = qMax(config.officeRooms, 7.0); // number of rooms per unit

    double w,h,x,y;

    // Set the width and height of the map.
    width = roomSize*rooms;
    height = width;

    double ss = 0.5 * (1.0 - doorWidth) / 2.0; // doorwall size

    // Build a unit square (room).
    Vector<Polygon> square;
    x = 0.5; y = wallThickness; w = 0.5; h = wallThickness; // bottom
    square.push_back(Polygon(x, y, w, h));
    y = 0.5; x = wallThickness; h = 0.5; w = wallThickness; // left
    square.push_back(Polygon(x, y, w, h));
    x = 0.5; y = 1.0-wallThickness; w = 0.5; h = wallThickness; // top
    square.push_back(Polygon(x, y, w, h));
    y = 0.5; x = 1.0-wallThickness; h = 0.5; w = wallThickness; // right
    square.push_back(Polygon(x, y, w, h));

    // Make a unit down room.
    Vector<Polygon> downRoom;
    x = ss; y = wallThickness; w = ss; h = wallThickness; // bottom
    downRoom.push_back(Polygon(x, y, w, h));
    x = 1.0-ss; y = wallThickness; w = ss; h = wallThickness; // bottom
    downRoom.push_back(Polygon(x, y, w, h));
    y = 0.5; x = wallThickness; h = 0.5; w = wallThickness; // left
    downRoom.push_back(Polygon(x, y, w, h));
    x = 0.5; y = 1.0-wallThickness; w = 0.5; h = wallThickness; // top
    downRoom.push_back(Polygon(x, y, w, h));
    y = 0.5; x = 1.0-wallThickness; h = 0.5; w = wallThickness; // right
    downRoom.push_back(Polygon(x, y, w, h));

    // Make a unit up room.
    Vector<Polygon> upRoom;
    x = 0.5; y = wallThickness; w = 0.5; h = wallThickness; // bottom
    upRoom.push_back(Polygon(x, y, w, h));
    y = 0.5; x = wallThickness; h = 0.5; w = wallThickness; // left
    upRoom.push_back(Polygon(x, y, w, h));
    x = ss; y = 1.0-wallThickness; w = ss; h = wallThickness; // top
    upRoom.push_back(Polygon(x, y, w, h));
    x = 1.0-ss; y = 1.0-wallThickness; w = ss; h = wallThickness; // top
    upRoom.push_back(Polygon(x, y, w, h));
    y = 0.5; x = 1.0-wallThickness; h = 0.5; w = wallThickness; // right
    upRoom.push_back(Polygon(x, y, w, h));

    // Make a unit left room.
    Vector<Polygon> leftRoom;
    x = 0.5; y = wallThickness; w = 0.5; h = wallThickness; // bottom
    leftRoom.push_back(Polygon(x, y, w, h));
    y = ss; x = wallThickness; h = ss; w = wallThickness; // left
    leftRoom.push_back(Polygon(x, y, w, h));
    y = 1.0-ss; x = wallThickness; h = ss; w = wallThickness; // left
    leftRoom.push_back(Polygon(x, y, w, h));
    x = 0.5; y = 1.0-wallThickness; w = 0.5; h = wallThickness; // top
    leftRoom.push_back(Polygon(x, y, w, h));
    y = 0.5; x = 1.0-wallThickness; h = 0.5; w = wallThickness; // right
    leftRoom.push_back(Polygon(x, y, w, h));

    // Make a unit right room.
    Vector<Polygon> rightRoom;
    x = 0.5; y = wallThickness; w = 0.5; h = wallThickness; // bottom
    rightRoom.push_back(Polygon(x, y, w, h));
    y = 0.5; x = wallThickness; h = 0.5; w = wallThickness; // left
    rightRoom.push_back(Polygon(x, y, w, h));
    x = 0.5; y = 1.0-wallThickness; w = 0.5; h = wallThickness; // top
    rightRoom.push_back(Polygon(x, y, w, h));
    y = ss; x = 1.0-wallThickness; h = ss; w = wallThickness; // right
    rightRoom.push_back(Polygon(x, y, w, h));
    y = 1.0-ss; x = 1.0-wallThickness; h = ss; w = wallThickness; // right
    rightRoom.push_back(Polygon(x, y, w, h));


    // Construct one section of office building.
    Vector<Polygon> building;
    Vector<Vec2> dop;

    // Outer ring of rooms.

    // bottom line
    for (int k = 1; k < rooms-1; k++)
    {
        Vector<Polygon> room;
        room << upRoom;
        for (int i = 0; i < room.size(); i++)
        {
            room[i].translate(k, 0);
            room[i].transform();
        }

        building << room;

        dop << Vec2(0.5+k, 0.5);
    }

    // left line
    for (int k = 1; k < rooms-1; k++)
    {
        Vector<Polygon> room;
        room << rightRoom;
        for (int i = 0; i < room.size(); i++)
        {
            room[i].translate(0, k);
            room[i].transform();
        }

        building << room;

        dop << Vec2(0.5, 0.5+k);
    }

    // top line
    for (int k = 1; k < rooms-1; k++)
    {
        Vector<Polygon> room;
        room << downRoom;
        for (int i = 0; i < room.size(); i++)
        {
            room[i].translate(k, rooms-1);
            room[i].transform();
        }

        building << room;

        dop << Vec2(0.5+k, rooms-0.5);
    }

    // right line
    for (int k = 1; k < rooms-1; k++)
    {
        Vector<Polygon> room;
        room << leftRoom;
        for (int i = 0; i < room.size(); i++)
        {
            room[i].translate(rooms-1, k);
            room[i].transform();
        }

        building << room;

        dop << Vec2(rooms-0.5, 0.5+k);
    }

    // Corner rooms.
    x = 0.5; y = 0.5; w = 0.5; h = 0.5;
    building.push_back(Polygon(x, y, w, h));
    x = rooms-0.5; y = 0.5; w = 0.5; h = 0.5;
    building.push_back(Polygon(x, y, w, h));
    x = 0.5; y = rooms-0.5; w = 0.5; h = 0.5;
    building.push_back(Polygon(x, y, w, h));
    x = rooms-0.5; y = rooms-0.5; w = 0.5; h = 0.5;
    building.push_back(Polygon(x, y, w, h));



    // Inner ring of rooms.

    // bottom line
    for (int k = 3; k < rooms-3; k++)
    {
        Vector<Polygon> room;
        room << downRoom;
        for (int i = 0; i < room.size(); i++)
        {
            room[i].translate(k, 2);
            room[i].transform();
        }

        building << room;

        dop << Vec2(0.5+k, 2.5);
    }

    // left line
    for (int k = 2; k < rooms-2; k++)
    {
        Vector<Polygon> room;
        room << leftRoom;
        for (int i = 0; i < room.size(); i++)
        {
            room[i].translate(2, k);
            room[i].transform();
        }

        building << room;

        dop << Vec2(2.5, 0.5+k);
    }

    // top line
    for (int k = 3; k < rooms-3; k++)
    {
        Vector<Polygon> room;
        room << upRoom;
        for (int i = 0; i < room.size(); i++)
        {
            room[i].translate(k, rooms-3);
            room[i].transform();
        }

        building << room;

        dop << Vec2(0.5+k, rooms-2.5);
    }

    // right line
    for (int k = 2; k < rooms-2; k++)
    {
        Vector<Polygon> room;
        room << rightRoom;
        for (int i = 0; i < room.size(); i++)
        {
            room[i].translate(rooms-3, k);
            room[i].transform();
        }

        building << room;

        dop << Vec2(rooms-2.5, 0.5+k);
    }

    // Center
    x = 0.5*rooms; y = 0.5*rooms; w = 0.5*(rooms-6.0); h = w;
    building.push_back(Polygon(x, y, w, h));


    // Make the building.
    for (int k = 0; k < building.size(); k++)
        building[k].transform();
    polygons.addPolygons(building);
    dropOffPoints << dop;


    // Scale all obstacles and drop off points by the room size.
    polygons.scale(roomSize);
    for (int i = 0; i < dropOffPoints.size(); i++)
        dropOffPoints[i].scale(roomSize, roomSize);
}

void World::buildWarehouse()
{
    int shelves = 5;
    double shelfWidth = 1;
    double shelfHeight = 6;
    double aisleWidth = config.wareHouseAisleWidth;
    double avenueWidth = 3;
    width = 2*avenueWidth + shelves*shelfWidth + (shelves-1)*aisleWidth;
    height = 3*avenueWidth + 2*shelfHeight;

    // Drop off points
    for (uint i = 0; i < shelves; i++)
    {
        double d = config.gmDilationRadius-0.02;

        Vec2 v;
        v.x = avenueWidth + i*(shelfWidth+aisleWidth) - d;
        v.y = height - avenueWidth - shelfHeight/4;
        dropOffPoints.push_back(v);

        v.x = avenueWidth + i*(shelfWidth+aisleWidth) + shelfWidth + d;
        v.y = height - avenueWidth - shelfHeight/4;
        dropOffPoints.push_back(v);


        v.x = avenueWidth + i*(shelfWidth+aisleWidth) - d;
        v.y = height - avenueWidth - shelfHeight + shelfHeight/4;
        dropOffPoints.push_back(v);

        v.x = avenueWidth + i*(shelfWidth+aisleWidth) + shelfWidth + d;
        v.y = height - avenueWidth - shelfHeight + shelfHeight/4;
        dropOffPoints.push_back(v);


        v.x = avenueWidth + i*(shelfWidth+aisleWidth) - d;
        v.y = avenueWidth + shelfHeight - shelfHeight/4;
        dropOffPoints.push_back(v);

        v.x = avenueWidth + i*(shelfWidth+aisleWidth) + shelfWidth + d;
        v.y = avenueWidth + shelfHeight - shelfHeight/4;
        dropOffPoints.push_back(v);


        v.x = avenueWidth + i*(shelfWidth+aisleWidth) - d;
        v.y = avenueWidth + shelfHeight/4;
        dropOffPoints.push_back(v);

        v.x = avenueWidth + i*(shelfWidth+aisleWidth) + shelfWidth + d;
        v.y = avenueWidth + shelfHeight/4;
        dropOffPoints.push_back(v);
    }

    dropOffPoints.push_back(Vec2(avenueWidth/2, avenueWidth/2));
    dropOffPoints.push_back(Vec2(width-avenueWidth/2, avenueWidth/2));
    dropOffPoints.push_back(Vec2(width-avenueWidth/2, height-avenueWidth/2));
    dropOffPoints.push_back(Vec2(avenueWidth/2, height-avenueWidth/2));

    // Polygons
    double w,h,x,y;

    // Top row of shelves.
    for (uint i = 0; i < shelves; i++)
    {
        w = shelfWidth/2;
        h = shelfHeight/2;
        x = avenueWidth+w + i*(shelfWidth+aisleWidth);
        y = height - avenueWidth - h;
        polygons.addPolygon(Polygon(x, y, w, h));
    }

    // Bottom row of shelves.
    for (uint i = 0; i < shelves; i++)
    {
        w = shelfWidth/2;
        h = shelfHeight/2;
        x = avenueWidth+w + i*(shelfWidth+aisleWidth);
        y = avenueWidth + h;
        polygons.addPolygon(Polygon(x, y, w, h));
    }


    // bottom wall
    w = width/2;
    h = 0.25;
    x = w;
    y = h;
    polygons.addPolygon(Polygon(x, y, w, h));

    // top wall
    w = width/2;
    h = 0.25;
    x = w;
    y = height-h;
    polygons.addPolygon(Polygon(x, y, w, h));

    // left wall
    w = 0.25;
    h = height/2;
    x = w;
    y = h;
    polygons.addPolygon(Polygon(x, y, w, h));

    // right wall
    w = 0.25;
    h = height/2;
    x = width-w;
    y = h;
    polygons.addPolygon(Polygon(x, y, w, h));

    polygons.transform();
}

// A map with lots of little triangles.
void World::buildClutter()
{
    width = config.clutterWidth;
    height = config.clutterHeight;

    // Random drop off points
    for (int i = 0; i < config.clutterDropOffPoints; i++)
    {
        Vec2 v;
        bool good = false;
        int counter = 0;
        while (!good && counter++ < 100)
        {
            v.randomize();
            v.scale(width, height);
            good = true;
            for (uint j = 0; j < dropOffPoints.size(); j++)
            {
                if ((dropOffPoints[j]-v).norm() < 2*config.worldDropOffRadius)
                {
                    good = false;
                    break;
                }
            }

            if (v.x > width-(config.gmDilationRadius+config.worldDropOffRadius)
                ||
                v.x < (config.gmDilationRadius+config.worldDropOffRadius)
                    ||
                v.y > height-(config.gmDilationRadius+config.worldDropOffRadius)
                    ||
                v.y < (config.gmDilationRadius+config.worldDropOffRadius))
                good = false;
        }
        dropOffPoints.push_back(v);
    }

    double thickness = 0.1;
    double dx,dy,x,y;

    // Outer walls.
    dx = thickness;
    dy = width/2;
    x = height/2;
    y = 0;
    Polygon p1(x, y, dx, dy);
    p1.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p1); // right

    dx = thickness;
    dy = width/2;
    x = -height/2;
    y = 0;
    Polygon p2(x, y, dx, dy);
    p2.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p2); // left

    dx = height/2;
    dy = thickness;
    x = 0;
    y = -width/2;
    Polygon p3(x, y, dx, dy);
    p3.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p3); // bottom

    dx = height/2;
    dy = thickness;
    x = 0;
    y = width/2;
    Polygon p4(x, y, dx, dy);
    p4.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addPolygon(p4); // top

    polygons.translate(width/2, height/2);
    polygons.transform();

    // Random polygons.
    for (int i = 0; i < config.clutterObstacles; i++)
    {
        Polygon pol;
        bool good = false;
        while (!good)
        {
            // Determine a random transformation.
            double sx = Statistics::uniformSample(config.clutterObstaclesSizeMin, config.clutterObstaclesSizeMax);
            double sy = Statistics::uniformSample(config.clutterObstaclesSizeMin, config.clutterObstaclesSizeMax);
            double x = Statistics::uniformSample(sx, width-sx);
            double y = Statistics::uniformSample(sy, height-sy);
            double theta = Statistics::uniformSample(-PI, PI);

            //qDebug() << "x:" << sx << x << width-sx << "y:" << sy << y << height-sy;

            // Apply the random transformation to the unit octogon.
            pol.setUnitOctogon();
            pol.scale(sx, sy);
            pol.translate(x, y);
            pol.turn(theta);
            pol.transform();

            // drop off point intersection test
            good = true;
            Polygon o = pol;
            o.grow(config.gmDilationRadius);
            o.grow(config.worldDropOffRadius);
            for (int j = 0; j < dropOffPoints.size(); j++)
            {
                if (o.intersects(dropOffPoints[j]))
                {
                    good = false;
                    break;
                }
            }
        }

        polygons.addPolygon(pol);
    }
}

// Builds a map from data/polygons.txt
void World::buildFromFile()
{
    width = 100;
    height = 65;

    // drop off points
    dropOffPoints.push_back(Vec2(width/10, height/3));
    dropOffPoints.push_back(Vec2(width/2, height/10));
    dropOffPoints.push_back(Vec2(9*width/10, height/3));
    dropOffPoints.push_back(Vec2(width/10, 2*height/3));
    dropOffPoints.push_back(Vec2(width/2, 9*height/10));
    dropOffPoints.push_back(Vec2(9*width/10, 2*height/3));

    QString line;
    QStringList tokenList;
    bool ok;

    // Open the data file.
    QFile file;
    file.setFileName("data/polygons.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Could not open file.";
        return;
    }
    QTextStream in(&file);

    Polygon obst;
    while (!in.atEnd())
    {
        line = in.readLine();
        if (line.startsWith("//") || line.startsWith('#'))
            continue;

        if (line.size() < 2)
        {
            polygons.addPolygon(obst);
            obst.clear();
            continue;
        }

        tokenList = line.split(QRegExp("\\s+"), QString::SkipEmptyParts);
        double y = tokenList[0].toDouble(&ok)*12-15;
        double x = -tokenList[1].toDouble(&ok)*12+40;
        obst << Vec2(x, y);
    }

    polygons.addPolygon(obst);

    file.close();
}

// (Re)Build the simulation by adding all agents and obstacles to the Box2D simulation.
// The function clears the simulation first, so it can be used to rebuild the simulation
// from scratch any time.
void World::buildSimulation()
{
    // Clear the simulation first. Clear the forces and destroy all bodies.
    box2DSim.ClearForces();
    b2Body* node = box2DSim.GetBodyList();
    while (node)
    {
        b2Body* b = node;
        node = node->GetNext();
        box2DSim.DestroyBody(b);
    }

    // Set the gravity to zero for our top down world.
    box2DSim.SetGravity(b2Vec2(0.0f, 0));

    // Static polygons.
    // The simulation environment must be built from the polygons, because
    // in Box2D obstacles have to be convex with a vertex count of up to 16.
    ListIterator<Polygon> polygonsIt = polygons.getPolygons().begin();
    while (polygonsIt.hasNext())
    {
        Polygon& pol = polygonsIt.next();
        b2BodyDef bodyDef;
        bodyDef.type = b2_staticBody;
        b2Body* body = box2DSim.CreateBody(&bodyDef);
        b2PolygonShape shape;
        b2Vec2 vertices[pol.size()];
        ListIterator<Line> it = pol.edgeIterator();
        int j = 0;
        while (it.hasNext())
        {
            Vec2& v = it.next().p1();
            vertices[j++] = b2Vec2(v.x, v.y);
        }
        shape.Set(vertices, pol.size());
        body->CreateFixture(&shape, 0.0f);
        body->id = pol.getId();
        body->SetUserData(0); // The static polygons don't get a body pointer.
    }

    // Unicycle agents.
    // This is where the agents get their body pointer set.
    for (int i = 0; i < unicycleAgents.size(); i++)
    {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.linearDamping = config.agentLinearDamping;
        bodyDef.angularDamping = config.agentAngularDamping;
        bodyDef.position.Set(unicycleAgents[i].pos().x, unicycleAgents[i].pos().y);
        bodyDef.angle = unicycleAgents[i].orientation();
        b2Body* body = box2DSim.CreateBody(&bodyDef);

        b2PolygonShape shape;
        b2Vec2 vertices[unicycleAgents[i].size()];
        ListIterator<Line> it = unicycleAgents[i].edgeIterator();
        int j = 0;
        while (it.hasNext())
        {
            Vec2& v = it.next().p1();
            vertices[j++] = b2Vec2(v.x, v.y);
        }
        shape.Set(vertices, unicycleAgents[i].size());

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        fixtureDef.density = 1.0f;
        fixtureDef.friction = config.agentFriction; // This doesn't really do much.

        // This is ghost mode. Agents can't collide with each other.
        if (command.ghostMode /*&& unicycleAgents[i].getAgentId() > 0*/) // uncomment the agent id > 0 check to enable collisions for bots only.
        {
            fixtureDef.filter.categoryBits = 0x0002;
            fixtureDef.filter.maskBits = 0x0001;
        }

        body->CreateFixture(&fixtureDef);

        b2MassData massData;
        massData.center = b2Vec2(0,0);
        massData.mass = 1.0;
        massData.I = 1.0;
        body->SetMassData(&massData);

        body->id = unicycleAgents[i].getId();
        body->SetUserData(&(unicycleAgents[i]));
        unicycleAgents[i].setBody(body);
        unicycleAgents[i].physicsTransformOut();
    }
}

// Calls the step() function of all agents. The step() functions compute
// the controls for the respective agent. These controls will then be
// applied to the physical body of the agent, e.g. as an acting force.
// Then the physical simulation computes the next motion increment.
void World::stepAgents()
{
    for (int i = 0; i < unicycleAgents.size(); i++)
        unicycleAgents[i].step();
}

// The world step function is the main control loop of the framework.
// It calls the step() functions of all agents so that they can compute
// their controls. Then it steps the physical simulation which moves all
// bodies by one motion increment according to their controls and the laws
// of physics. Then it transfers the new physical state from the bodies
// into the agents.
void World::step()
{
    QMutexLocker locker(&mutex);

    // Step the physical simulation.
    // The amount of simulated time is determined by the selected frequency
    // (10 Hz, 20 Hz, 30 Hz). For example, if 10 Hz has been selected, the
    // simulated time is 100 ms.
    float32 timeStep = simulationTimeStep;
    int32 velocityIterations = 10;
    int32 positionIterations = 5;
    box2DSim.Step(timeStep, velocityIterations, positionIterations);

    // Update the state of the world objects from the physical simulation using their body pointers.
    physicsTransformIn();

    // Step the agent control functions. The agent control functions compute
    // accelerations that navigate the agent to its target.
    // The step() functions of the agents run a sense() - act() loop that computes
    // the accelerations and writes them into the agent itself. They will be
    // grabbed from there and applied to the bodies in the physical simulation for
    // a short amount of time.
    stepAgents();

    // In ghost robot mode, the agents don't collide in the physical simulation
    // and we have to care for the collisions manually.
    if (command.ghostMode)
    {
        Polygon polMain; // = unicycleAgents[0]; ???
        polMain.setVertices(unicycleAgents[0].getVertices());
        polMain.setPose(unicycleAgents[0].pose());
        polMain.transform();
        for (int i = 1; i < unicycleAgents.size(); i++)
        {
            Polygon pol;
            pol.setVertices(unicycleAgents[i].getVertices());
            pol.setPose(unicycleAgents[i].pose());
            pol.transform();
            //qDebug() << "agent" << i << "vertices" << unicycleAgents[i].getVertices() << "pose:" << unicycleAgents[i].pose();
            if (polMain.intersects(pol))
            {
                //qDebug() << "agent" << i << "intersects main agent.";
                unicycleAgents[0].collisionResponse(&unicycleAgents[i]);
            }
        }
    }

    // Apply the computed accelerations to the bodies in the physical simulation.
    physicsControl();
}

// Returns a list of the static obstacles in the world.
const Vector<Polygon>& World::getStaticObstacles() const
{
    return worldMap.getPolygons();
}

// Returns a vector of the unicycle agents in the world.
// Using the excludeId, a single unicycle agent can be excluded from the vector.
Vector<UnicycleObstacle> World::getUnicycleObstacles(int excludeId) const
{
    Vector<UnicycleObstacle> obst;
    for (int i = 0; i < unicycleAgents.size(); i++)
    {
        if (unicycleAgents[i].isActive())
            if (unicycleAgents[i].getAgentId() != excludeId)
                obst << unicycleAgents[i];
    }
    return obst;
}

// Returns the drop off points.
const Vector<Vec2> &World::getDropOffPoints() const
{
    return dropOffPoints;
}

// Contact detection callback function. The simulation calls this function
// when a new contact has been detected.
void World::BeginContact(b2Contact* contact)
{
    if (contact->IsTouching())
    {
        b2Body* body1 = contact->GetFixtureA()->GetBody();
        b2Body* body2 = contact->GetFixtureB()->GetBody();

        // Default collision handlers.
        if (body1->id > 0)
        {
            if (body1->GetUserData() != 0)
                ((Obstacle*)(body1->GetUserData()))->collisionResponse((Obstacle*)(body2->GetUserData()));
        }
        if (body2->id > 0)
        {
            if (body2->GetUserData() != 0)
                ((Obstacle*)(body2->GetUserData()))->collisionResponse((Obstacle*)(body1->GetUserData()));
        }
    }
}

// Contact end callback function. Contact termination events originate from here.
void World::EndContact(b2Contact *contact)
{
    if (!contact->IsTouching())
    {
        b2Body* body1 = contact->GetFixtureA()->GetBody();
        b2Body* body2 = contact->GetFixtureB()->GetBody();
        //qDebug() << contact->IsTouching() << body2->id << body1->id;

        // identify objects by body1->id and do something if you want.
    }
}

// Transfers the motion states of the bodies from the physical simulation into their
// representing objects inside the world. This function is called in every iteration
// of the main control loop.
void World::physicsTransformIn()
{
    for (int i = 0; i < unicycleAgents.size(); i++)
        unicycleAgents[i].physicsTransformIn();
    for (int i = 0; i < unicycleAgents.size(); i++)
        unicycleAgents[i].setWorldUnicycleObstacles(getUnicycleObstacles(unicycleAgents[i].getAgentId()));
}

// Transfers the motion states from the objects inside the world through their body
// pointers into the physical simulation. This function is called when a historical
// state is loaded.
void World::physicsTransformOut()
{
    // The easiest thing to do is to just rebuild the whole simulation.
    buildSimulation();
}

// Applies the forces to the world objects in the physical simulation through their body
// pointers. This function is called in every iteration of the main control loop after the
// control functions of the objects have been executed.
void World::physicsControl()
{
    for (int i = 0; i < unicycleAgents.size(); i++)
        unicycleAgents[i].physicsControl();
}

// Switches to the map with the id w.
void World::setMap(int m, int numAgents)
{
    // Set the random seed here if you want to reproduce a specific run.
    //Statistics::setSeed(298438517); // set the seed here if you want to see a specific run
    //qDebug() << "Seed set to" << Statistics::getSeed() << "check:" << Statistics::randomNumber();
    mapId = m;
    this->numAgents = numAgents;
    reset();
}

// Sets the world parameters that determine things like the trajectory planner of the agents
// (PD, DWA, A*, RuleBase), the trajectory type being used (arc, B0, Fresnel), the prediction
// type (linear, unicycle), the heuristic (MC, Dijkstra, Euklidean), and most importantly, the
// control frequency (10 Hz, 20 Hz, 30Hz) and with that the time step of the physical simulation.
void World::setParams(int trajectoryPlanningMethod, int trajectoryType, int predictionType, int heuristicType, uint frequency)
{
    for (int i = 0; i < unicycleAgents.size(); i++)
        unicycleAgents[i].setParams(trajectoryPlanningMethod, trajectoryType, predictionType, heuristicType, frequency);
    simulationTimeStep = 1.0/frequency;
}

// Adds unicycle agents to the world. The agents start on a random
// drop-off point which are assumed to be non-colliding.
void World::addAgents(uint numAgents)
{
    if (dropOffPoints.size() < numAgents)
    {
        qDebug() << "Warning! Not enough drop off points to spawn all agents! dropOffPoints:" << dropOffPoints.size() << "agents:" << numAgents;
        numAgents = dropOffPoints.size();
    }

    // Permute the drop off poinsts to be used as random starting positions for the agents.
    Vector<Vec2> permutedDropOffPoints = dropOffPoints;
    for (uint d = 0; d < dropOffPoints.size(); d++)
        permutedDropOffPoints.swap(d, Statistics::randomInt(0, dropOffPoints.size()-1));

    for (int i = 0; i < numAgents; i++)
    {
        UnicycleAgent ua;
        ua.setAgentId(i); // The first main agent has the id 0.
        ua.init(permutedDropOffPoints[i]);
        ua.setPos(permutedDropOffPoints[i]);
        ua.setOrientation(Statistics::uniformSample(-PI, PI));
        ua.setWorldDropOffPoints(getDropOffPoints());
        ua.setWorldMap(worldMap);
        ua.setWorldPolygons(polygons);
        unicycleAgents.push_back(ua); // Note that the because of the push the agent needs to be copyable.
    }
}

// Returns a human readable map name.
QString World::getMapName(uint map) const
{
    QString mapName;
    if (map == 0)
        mapName = "void";
    if (map == 1)
        mapName = "simple";
    if (map == 2)
        mapName = "u";
    if (map == 3)
        mapName = "apartment";
    if (map == 4)
        mapName = "warehouse";
    if (map == 5)
        mapName = "office";
    if (map == 6)
        mapName = "clutter";

    return mapName;
}

void World::logObstaclesToTxt()
{
    if (dropOffPoints.size() < 2)
    {
        qDebug() << "Not enough drop off points specified";
        return;
    }

    Vector<Polygon> polygons = worldMap.getPolygons();

    qDebug() << "Writing the log.";

    Logger log("data/polygons.txt");

    // Write the start and goal states.
    log << dropOffPoints[0];
    log++;
    log << dropOffPoints[1];
    log++;
    log++;

    // Write the polygons.
    unsigned int cornercounter = 0;
    for (int i = 0; i < polygons.size(); i++)
    {
        ListIterator<Line> it = polygons[i].edgeIterator();
        while (it.hasNext())
        {
            log << it.next().p1();
            log++;
            cornercounter++;
        }

        log++;
    }

    qDebug() << "Polygons:" << polygons.size() << "corners:" << cornercounter << ".";
}

// Draws the world on the QPainter.
// The world visualization is drawn on top of the GraphicsScene. Right now, world.draw()
// is used exclusively for all visualization and the GraphicsScene does nothing.
void World::draw(QPainter *painter) const
{
    QMutexLocker locker(&mutex);

    // Simulation debug overlay.
    // This is some very generic code that draws all bodies in the simulation onto a QPainter.
    // It helps to check if the visual representation of the game objects match their bodies
    // in the Box2D simulation.
    if (command.showSimulationDebugDraw)
    {
        painter->save();
        painter->setPen(drawUtil.penBlueThick);
        const b2Body* node = box2DSim.GetBodyList();

        while (node)
        {
            const b2Body* b = node;
            node = node->GetNext();

            if (!b->IsActive())
                continue;

            b2Vec2 pos = b->GetPosition();
            double angle = b->GetAngle();

            painter->save();
            painter->translate(pos.x, pos.y);
            painter->rotate(angle*RAD_TO_DEG);

            const b2Fixture* fixture = b->GetFixtureList();
            const b2PolygonShape* shape = static_cast<const b2PolygonShape*>(fixture->GetShape());
            int vertexCount = shape->GetVertexCount();
            for (int i = 0; i < vertexCount; i++)
                painter->drawLine(QPointF(shape->GetVertex(i).x, shape->GetVertex(i).y),
                                  QPointF(shape->GetVertex((i+1)%vertexCount).x, shape->GetVertex((i+1)%vertexCount).y));
            painter->restore();
        }

        painter->restore();
    }

    // Draw the world map. This is the map the agent is making.
    if (command.showWorldMap)
        worldMap.draw(painter, drawUtil.penDashed, drawUtil.brushWhite, QBrush(QColor::fromRgb(245,245,245)));

    // The raw world polygons.
    if (command.showWorldPolygons)
        polygons.draw(painter, drawUtil.pen, drawUtil.brush, Qt::NoBrush);

    // The drop off points.
    if (command.showDropOffPoints)
    {
        for (int i = 0; i < dropOffPoints.size(); i++)
        {
            painter->save();
            double r = config.worldDropOffRadius;
            painter->translate(dropOffPoints[i]);
            painter->setPen(drawUtil.penThinDashed);
            painter->drawEllipse(QPointF(), r, r);
            painter->scale(0.03, -0.03);
            if (i > 9)
                painter->drawText(QPointF(-8, 5), QString::number(i));
            else
                painter->drawText(QPointF(-4, 5), QString::number(i));
            painter->restore();
        }
    }

    // Draw the agents.
    for (int i = 0; i < unicycleAgents.size(); i++)
        unicycleAgents[i].draw(painter);
}

// Draws the world background on the QPainter.
// The world background is additional visualization that is drawn underneath (behind) the
// GraphicsScene. The world background draws things that are underneath the agents. There
// is not much point to it as all these things can be drawn in the draw() method just as
// well *before* the agents are drawn.
void World::drawBackground(QPainter *painter) const
{
    QMutexLocker locker(&mutex);

    // Trajectory trace.
    // When enabled, it shows the history of all taxi positions in the state.
    if (command.showTrajectoryTrace)
    {
        double s = 1.0/painter->transform().m11();
        painter->save();

        painter->setPen(drawUtil.penBlueThick);
        painter->setBrush(drawUtil.brushRed);
        for (int i = 0; i < state.size(); i++)
            painter->drawEllipse(state[i].uniTaxi.pos(), s, s);

        painter->restore();
    }
}

QDebug operator<<(QDebug dbg, const World &w)
{
    Vector<Polygon> obst = w.worldMap.getPolygons();
    dbg << "Static Obstacles:" << "\n";
    for (int i = 0; i < obst.size(); i++)
        dbg << "   " << obst[i]  << "\n";

    Vector<UnicycleObstacle> obst3 = w.getUnicycleObstacles();
    dbg << "Unicycle Obstacles:" << "\n";
    for (int i = 0; i < obst3.size(); i++)
        dbg << "   " << obst3[i]  << "\n";

    return dbg;
}


#include "World.h"
#include "globals.h"
#include "blackboard/Command.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "util/Statistics.h"
#include "util/ColorUtil.h"
#include "util/Vec2.h"
#include "util/StopWatch.h"
#include "geometry/Box.h"
#include <QFile>
#include <QDataStream>
#include "util/Logger.h"

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
// the setWorld() function to instantly switch to a new map.

QMutex World::mutex;

World::World() : b2ContactListener()
{
    width = 10; // width of the world in meters
    height = 10; // height of the world in meters
    mapId = 4;
    numAgents = 1;
    simulationTimeStep = config.rcIterationTime;
}

void World::init()
{
    box2DSim.SetContactListener(this);
    numAgents = max(config.unicycleAgents, 1.0);
    reset();
}

// Rebuilds the world from scratch using the current mapId.
void World::reset()
{
    //QMutexLocker locker(&mutex);

    // Clear all data structures.
    dropOffPoints.clear();
    polygons.clear();
    staticObstacles.clear();
    unicycleAgents.clear();

    // Reset the obstacle ids.
    // This is so that when a new map is built, the ids are counted again from 0.
    Obstacle::resetId();

    // Build the map. These build routines fill the static obstacles list
    // with obstacles that are generated programmatically.
    if (mapId == 0)
        buildVoid();
    else if (mapId == 1)
        buildSimple();
    else if (mapId == 2)
        buildUTrap();
    else if (mapId == 3)
        buildOutdoor();
    else if (mapId == 4)
        buildApartment();
    else if (mapId == 5)
        buildWarehouse();
    else if (mapId == 6)
        buildOffice();
    else if (mapId == 7)
        buildClutter();
    polygons.transform(); // The static polygons of the map come out transformed.


    // Compute a core and an expanded geometric model of the static obstacles.
    // This is the world map the robot builds for itself (some day) and an
    // expanded version of it that's dilated at least by the minimal radius of
    // the robot for path planning. The expansion is computed by first converting
    // the polygons of the map to grid, using a dilate operation, and converting
    // the grid back to polygons.
    double dilation = config.gridWorldDilationRadius;
    Box bb = polygons.getBoundingBox();
    GridModel worldGrid;
    worldGrid.setDim(2);
    worldGrid.setN(Vec2u(bb.width()/config.gridCellSize+1, bb.height()/config.gridCellSize+1));
    worldGrid.setMin(bb.bottomLeft()-Vec2(config.gridWorldDilationRadius));
    worldGrid.setMax(bb.topRight()+Vec2(config.gridWorldDilationRadius));
    worldGrid.init();
    worldGrid.computeOccupancyGrid(polygons.getObstacles());
    //staticObstacles.setFromGrid(worldGrid); // core
    staticObstacles.setObstacles(polygons.getObstacles());
    worldGrid.dilate(dilation); // This parameter defines the gap sensitivity of the path planner.

    expandedStaticObstacles.setFromGrid(worldGrid); // expansion

//    qDebug() << "Built map" << getMapName(mapId) << "with" << staticObstacles.getVertexCount()
//             << "expanded:" << expandedStaticObstacles.getVertexCount()
//             << "raw:" << polygons.getVertexCount()
//             << "vertices.";

    // Add moving agents to the map.
    // The number of agents added depends on a configuration variable.
    addAgents();

    // Hacky bugfix.
    if (mapId == 7 || mapId == 3)
    {
        staticObstacles.buildFlatHierarchy();
        expandedStaticObstacles.buildFlatHierarchy();
    }

    // Prune.
    staticObstacles.prune(unicycleAgents[0].pos());
    expandedStaticObstacles.prune(unicycleAgents[0].pos());
    for (uint i = 0; i < unicycleAgents.size(); i++)
    {
        unicycleAgents[i].setWorldStaticObstacles(staticObstacles.getObstacles());
        unicycleAgents[i].setWorldExpandedStaticObstacles(expandedStaticObstacles.getObstacles());
    }

    // Build the Box2D simulation using the polygonal obstacles generated so far.
    // Most importantly, this is where the obstacles get their body pointer set.
    // The simulation must be built from the polygons, because they need to be
    // convex with a maximum vertex count of 16.
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

    Obstacle ob;
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
    polygons.addObstacle(ob);
}

// A locality trap with a U shaped obstacle.
void World::buildUTrap()
{
    width = 15;
    height = 15;

    // drop off points
    //dropOffPoints.push_back(Vec2(width/2, height/4));
    //dropOffPoints.push_back(Vec2(width/2, 3*height/4));
    dropOffPoints.push_back(Vec2(width/2, height/2+0.2));

    // drop off points
    Vec2 b(0,config.uRadius*qMin(width,height));
    Vec2 m(width/2, height/2);
    for (double phi = 0; phi < PII; phi += PII/config.uDropOffPoints)
    {
        Vec2 v = b.rotated(phi);
        dropOffPoints.push_back(m+v);
    }

    // Obstacles
    double uOffset = -1.0;
    double w = 0.5;
    double h = 3.5;

    double x = width/2;
    double y = height/2+uOffset;
    polygons.addObstacle(Obstacle(x, y, h, w));

    x = width/2+h-w;
    y = height/2+h-w+uOffset;
    polygons.addObstacle(Obstacle(x, y, w, h));

    x = width/2-h+w;
    y = height/2+h-w+uOffset;
    polygons.addObstacle(Obstacle(x, y, w, h));
}

// An "outdoor" example with randomized obstacles.
void World::buildOutdoor()
{
    width = config.outdoorWidth;
    height = config.outdoorHeight;

    // drop off points
    dropOffPoints.push_back(Vec2(width/10, height/3));
    dropOffPoints.push_back(Vec2(width/2, height/10));
    dropOffPoints.push_back(Vec2(9*width/10, height/3));
    dropOffPoints.push_back(Vec2(width/10, 2*height/3));
    dropOffPoints.push_back(Vec2(width/2, 9*height/10));
    dropOffPoints.push_back(Vec2(9*width/10, 2*height/3));

    // obstacles
    for (int i = 0; i < config.outdoorObstaclesNr; i++)
    {
        Obstacle obs;
        bool good = false;
        while (!good)
        {
            obs.clear();
            obs << Vec2(-config.outdoorObstaclesMinWidth, 0.5*config.outdoorObstaclesMinHeight)
               << Vec2(-config.outdoorObstaclesMinWidth, -0.5*config.outdoorObstaclesMinHeight)
               << Vec2(-0.7*config.outdoorObstaclesMinWidth, -config.outdoorObstaclesMinHeight)
               << Vec2(0.7*config.outdoorObstaclesMinWidth, -config.outdoorObstaclesMinHeight)
               << Vec2(config.outdoorObstaclesMinWidth, -0.5*config.outdoorObstaclesMinHeight)
               << Vec2(config.outdoorObstaclesMinWidth, 0.5*config.outdoorObstaclesMinHeight)
               << Vec2(0.7*config.outdoorObstaclesMinWidth, config.outdoorObstaclesMinHeight)
               << Vec2(-0.7*config.outdoorObstaclesMinWidth, config.outdoorObstaclesMinHeight);

            double sx = Statistics::uniformSample(1.0, config.outdoorObstaclesMaxWidth/config.outdoorObstaclesMinWidth);
            double sy = Statistics::uniformSample(1.0, config.outdoorObstaclesMaxHeight/config.outdoorObstaclesMinHeight);
            double x = Statistics::uniformSample(0, width);
            double y = Statistics::uniformSample(0, height);
            double theta = Statistics::uniformSample(-PI2, PI2);

            obs.scale(sx, sy);
            obs.translate(x, y);
            obs.rotate(theta);
            //obs.transform();

            // Drop off point intersection test.
            good = true;
            for (int j = 0; j < dropOffPoints.size(); j++)
            {
                Obstacle copy = obs;
                copy.grow(config.gridWorldDilationRadius);
                if (copy.intersects(dropOffPoints[j], config.worldDropOffRadius))
                {
                    good = false;
                    break;
                }
            }
        }

        polygons.addObstacle(obs);
    }
}

void World::buildApartment()
{
    width = 10;
    height = 12;
    double thickness = 0.1;

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
    Obstacle p1(x, y, dx, dy);
    p1.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p1); // right

    dx = thickness;
    dy = width/2;
    x = -height/2;
    y = 0;
    Obstacle p133(x, y, dx, dy);
    p133.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p133); // left

    dx = height/2;
    dy = thickness;
    x = 0;
    y = -width/2;
    Obstacle p3(x, y, dx, dy);
    p3.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p3);

    dx = height/2;
    dy = thickness;
    x = 0;
    y = width/2;
    Obstacle p4(x, y, dx, dy);
    p4.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p4);

    // Living room.
    dx = 1;
    dy = thickness;
    x = -height/2+dx;
    y = 0;
    Obstacle p5(x, y, dx, dy);
    p5.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p5);

    dx = 1.75;
    dy = thickness;
    x = -0.75;
    y = 0;
    Obstacle p6(x, y, dx, dy);
    p6.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p6);

    dx = thickness;
    dy = width/4;
    x = 1;
    y = -width/4;
    Obstacle p7(x, y, dx, dy);
    p7.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p7);

    // Kitchen
    dx = 0.5;
    dy = thickness;
    x = -height/2+dx;
    y = 2;
    Obstacle p8(x, y, dx, dy);
    p8.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p8);

    dx = 3;
    dy = thickness;
    x = -0.8;
    y = 2;
    Obstacle p9(x, y, dx, dy);
    p9.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p9);

    dx = thickness;
    dy = 1.5;
    x = 0;
    y = 3.5;
    Obstacle p10(x, y, dx, dy);
    p10.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p10);

    dx = thickness;
    dy = 0.85;
    x = -1.8;
    y = 2.85;
    Obstacle p11(x, y, dx, dy);
    p11.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p11);


    dx = 1.3;
    dy = thickness;
    x = height/2-dx;
    y = 2;
    Obstacle p12(x, y, dx, dy);
    p12.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p12);

    dx = 1.3;
    dy = thickness;
    x = height/2-dx;
    y = 0;
    Obstacle p18(x, y, dx, dy);
    p18.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p18);

    dx = 1.7;
    dy = thickness;
    x = height/2-dx;
    y = -2;
    Obstacle p13(x, y, dx, dy);
    p13.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p13);

    dx = 0.3;
    dy = thickness;
    x = 1.3;
    y = -2;
    Obstacle p14(x, y, dx, dy);
    p14.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p14);

    dx = thickness;
    dy = 0.5;
    x = 3.5;
    y = 0;
    Obstacle p15(x, y, dx, dy);
    p15.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p15);

    dx = thickness;
    dy = 0.25;
    x = 3.5;
    y = 1.75;
    Obstacle p16(x, y, dx, dy);
    p16.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p16);

    dx = thickness;
    dy = 0.25;
    x = 3.5;
    y = -1.75;
    Obstacle p17(x, y, dx, dy);
    p17.setColor(QColor::fromRgbF(0, 0, 0, 0.8));
    polygons.addObstacle(p17);

    // Couch 1
    dx = 0.45;
    dy = 1.0;
    x = -2.6;
    y = -3.5;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    // Couch 2
    dx = 1.0;
    dy = 0.45;
    x = -3.6;
    y = -1.8;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    // TV
    dx = 0.1;
    dy = 0.8;
    x = -5.5;
    y = -3.7;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    // Couch table
    dx = 0.4;
    dy = 0.4;
    x = -4.2;
    y = -3.5;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    // Dining table
    dx = 0.5;
    dy = 0.8;
    x = -0.5;
    y = -2.5;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    // Chairs
    dx = 0.2;
    dy = 0.2;
    x = -0.5;
    y = -2.5 + 1.1;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = -0.5;
    y = -2.5 - 1.1;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = -0.5 + 0.8;
    y = -2.5 + 0.4;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = -0.5 + 0.8;
    y = -2.5 - 0.4;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = -0.5 - 0.8;
    y = -2.5 + 0.4;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = -0.5 - 0.8;
    y = -2.5 - 0.4;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    // Kitchen line
    dx = 0.35;
    dy = 1.35;
    x = -5.5;
    y = 3.5;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 1.35;
    dy = 0.35;
    x = -4.5;
    y = 4.5;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    // Kitchen shelf
    dx = 0.1;
    dy = 0.6;
    x = -2.05;
    y = 3.0;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.6;
    dy = 0.1;
    x = -2.6;
    y = 2.25;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    // Storage room shelf
    dx = 0.2;
    dy = 1.35;
    x = -0.35;
    y = 3.5;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.5;
    dy = 0.15;
    x = -1.0;
    y = 2.25;
    polygons.addObstacle(Obstacle(x, y, dx, dy));


    // Beds
    dx = 1.0;
    dy = 0.45;
    x = 1.1;
    y = 4.4;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 1.0;
    dy = 0.45;
    x = 4.85;
    y = 4.4;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 1.0;
    dy = 0.45;
    x = 4.85;
    y = -2.6;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    // Desks with chairs
    dx = 0.3;
    dy = 0.6;
    x = 0.5;
    y = 3.0;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = 1.1;
    y = 3.0;
    polygons.addObstacle(Obstacle(x, y, dx, dy));


    dx = 0.3;
    dy = 0.6;
    x = 5.6;
    y = 3.0;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = 5.0;
    y = 3.0;
    polygons.addObstacle(Obstacle(x, y, dx, dy));


    dx = 0.6;
    dy = 0.3;
    x = 5.0;
    y = -4.6;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.2;
    dy = 0.2;
    x = 5.0;
    y = -4.0;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    // More shelves.
    dx = 0.5;
    dy = 0.14;
    x = 3.2;
    y = -2.3;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.2;
    dy = 1.0;
    x = 1.35;
    y = -3.5;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.7;
    dy = 0.3;
    x = 3.0;
    y = -4.5;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.15;
    dy = 0.7;
    x = 1.3;
    y = -1.0;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    // Bathroom stuff.
    dx = 0.2;
    dy = 0.2;
    x = 5.7;
    y = -0.5;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.4;
    dy = 0.4;
    x = 5.5;
    y = -1.5;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.3;
    dy = 0.15;
    x = 4.4;
    y = -1.7;
    polygons.addObstacle(Obstacle(x, y, dx, dy));


    dx = 0.2;
    dy = 0.2;
    x = 4.8;
    y = 1.7;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.3;
    dy = 0.7;
    x = 5.6;
    y = 1.0;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

    dx = 0.3;
    dy = 0.15;
    x = 4.4;
    y = 0.3;
    polygons.addObstacle(Obstacle(x, y, dx, dy));

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
void World::buildWarehouse()
{
    width = 100;
    height = 65;

    // drop off points
    dropOffPoints.push_back(Vec2(width/7, height/4));
    dropOffPoints.push_back(Vec2(6*width/7, 3*height/4));

    dropOffPoints.push_back(Vec2(2.5*width/7, height/4));
    dropOffPoints.push_back(Vec2(4.5*width/7, 3*height/4));

    dropOffPoints.push_back(Vec2(4.5*width/7, height/4));
    dropOffPoints.push_back(Vec2(2.5*width/7, 3*height/4));

    dropOffPoints.push_back(Vec2(6*width/7, height/4));
    dropOffPoints.push_back(Vec2(width/7, 3*height/4));

    // Obstacles
    double w,h,x,y;

    w = 1;
    h = 10;
    x = 25;
    y = 19;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 1;
    h = 10;
    x = 50;
    y = 1+h;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 1;
    h = 10;
    x = 75;
    y = 19;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 1;
    h = 10;
    x = 25;
    y = height-9-h;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 1;
    h = 10;
    x = 50;
    y = height-1-h;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 1;
    h = 10;
    x = 75;
    y = height-9-h;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 49;
    h = 1;
    x = 1+w;
    y = 1+h;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 43;
    h = 1;
    x = 1+w;
    y = 28;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 43;
    h = 1;
    x = width-w-1;
    y = 37;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 49;
    h = 1;
    x = 1+w;
    y = 63;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 1;
    h = 13;
    x = 1+w;
    y = 1+h;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 1;
    h = 13;
    x = width-1-w;
    y = 1+h;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 1;
    h = 13;
    x = 1+w;
    y = height-1-h;
    polygons.addObstacle(Obstacle(x, y, w, h));

    w = 1;
    h = 13;
    x = width-1-w;
    y = height-1-h;
    polygons.addObstacle(Obstacle(x, y, w, h));

    double scale = 0.24;
    polygons.transform();
    polygons.scale(scale);
    polygons.transform();
    for (uint i = 0; i < dropOffPoints.size(); i++)
        dropOffPoints[i] *= scale;
    width *= scale;
    height *= scale;
}

// A map with lots of little triangles.
void World::buildClutter()
{
    width = config.clutterWidth;
    height = config.clutterHeight;

    // random drop off points
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
        }
        dropOffPoints.push_back(v);
    }

    // random obstacles
    for (int i = 0; i < config.clutterObstacles; i++)
    {
        Obstacle obs;
        bool good = false;
        while (!good)
        {
            // Starts with a unit octogon.
            obs.setUnitOctogon();

            // A random transformation.
            double sx = Statistics::uniformSample(config.clutterObstaclesSizeMin, config.clutterObstaclesSizeMax);
            double sy = Statistics::uniformSample(config.clutterObstaclesSizeMin, config.clutterObstaclesSizeMax);
            double x = Statistics::uniformSample(sx, width-sx);
            double y = Statistics::uniformSample(sy, height-sy);
            double theta = Statistics::uniformSample(-PI, PI);

            // Apply the transformation.
            obs.scale(sx, sy);
            obs.translate(x, y);
            obs.rotate(theta);
            obs.transform();

            // drop off point intersection test
            good = true;
            for (int j = 0; j < dropOffPoints.size(); j++)
            {
                Obstacle o = obs;
                o.grow(config.gmPolygonExpansionMargin);
                o.grow(config.worldDropOffRadius);
                if (o.intersects(dropOffPoints[j]))
                {
                    good = false;
                    break;
                }
            }
        }

        polygons.addObstacle(obs);
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

    Obstacle obst;
    while (!in.atEnd())
    {
        line = in.readLine();
        if (line.startsWith("//") || line.startsWith('#'))
            continue;

        if (line.size() < 2)
        {
            polygons.addObstacle(obst);
            obst.clear();
            continue;
        }

        tokenList = line.split(QRegExp("\\s+"), QString::SkipEmptyParts);
        double y = tokenList[0].toDouble(&ok)*12-15;
        double x = -tokenList[1].toDouble(&ok)*12+40;
        obst << Vec2(x, y);
    }

    polygons.addObstacle(obst);

    file.close();
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
    Vector<Obstacle> square;
    x = 0.5; y = wallThickness; w = 0.5; h = wallThickness; // bottom
    square.push_back(Obstacle(x, y, w, h));
    y = 0.5; x = wallThickness; h = 0.5; w = wallThickness; // left
    square.push_back(Obstacle(x, y, w, h));
    x = 0.5; y = 1.0-wallThickness; w = 0.5; h = wallThickness; // top
    square.push_back(Obstacle(x, y, w, h));
    y = 0.5; x = 1.0-wallThickness; h = 0.5; w = wallThickness; // right
    square.push_back(Obstacle(x, y, w, h));

    // Make a unit down room.
    Vector<Obstacle> downRoom;
    x = ss; y = wallThickness; w = ss; h = wallThickness; // bottom
    downRoom.push_back(Obstacle(x, y, w, h));
    x = 1.0-ss; y = wallThickness; w = ss; h = wallThickness; // bottom
    downRoom.push_back(Obstacle(x, y, w, h));
    y = 0.5; x = wallThickness; h = 0.5; w = wallThickness; // left
    downRoom.push_back(Obstacle(x, y, w, h));
    x = 0.5; y = 1.0-wallThickness; w = 0.5; h = wallThickness; // top
    downRoom.push_back(Obstacle(x, y, w, h));
    y = 0.5; x = 1.0-wallThickness; h = 0.5; w = wallThickness; // right
    downRoom.push_back(Obstacle(x, y, w, h));

    // Make a unit up room.
    Vector<Obstacle> upRoom;
    x = 0.5; y = wallThickness; w = 0.5; h = wallThickness; // bottom
    upRoom.push_back(Obstacle(x, y, w, h));
    y = 0.5; x = wallThickness; h = 0.5; w = wallThickness; // left
    upRoom.push_back(Obstacle(x, y, w, h));
    x = ss; y = 1.0-wallThickness; w = ss; h = wallThickness; // top
    upRoom.push_back(Obstacle(x, y, w, h));
    x = 1.0-ss; y = 1.0-wallThickness; w = ss; h = wallThickness; // top
    upRoom.push_back(Obstacle(x, y, w, h));
    y = 0.5; x = 1.0-wallThickness; h = 0.5; w = wallThickness; // right
    upRoom.push_back(Obstacle(x, y, w, h));

    // Make a unit left room.
    Vector<Obstacle> leftRoom;
    x = 0.5; y = wallThickness; w = 0.5; h = wallThickness; // bottom
    leftRoom.push_back(Obstacle(x, y, w, h));
    y = ss; x = wallThickness; h = ss; w = wallThickness; // left
    leftRoom.push_back(Obstacle(x, y, w, h));
    y = 1.0-ss; x = wallThickness; h = ss; w = wallThickness; // left
    leftRoom.push_back(Obstacle(x, y, w, h));
    x = 0.5; y = 1.0-wallThickness; w = 0.5; h = wallThickness; // top
    leftRoom.push_back(Obstacle(x, y, w, h));
    y = 0.5; x = 1.0-wallThickness; h = 0.5; w = wallThickness; // right
    leftRoom.push_back(Obstacle(x, y, w, h));

    // Make a unit right room.
    Vector<Obstacle> rightRoom;
    x = 0.5; y = wallThickness; w = 0.5; h = wallThickness; // bottom
    rightRoom.push_back(Obstacle(x, y, w, h));
    y = 0.5; x = wallThickness; h = 0.5; w = wallThickness; // left
    rightRoom.push_back(Obstacle(x, y, w, h));
    x = 0.5; y = 1.0-wallThickness; w = 0.5; h = wallThickness; // top
    rightRoom.push_back(Obstacle(x, y, w, h));
    y = ss; x = 1.0-wallThickness; h = ss; w = wallThickness; // right
    rightRoom.push_back(Obstacle(x, y, w, h));
    y = 1.0-ss; x = 1.0-wallThickness; h = ss; w = wallThickness; // right
    rightRoom.push_back(Obstacle(x, y, w, h));


    // Construct one section of office building.
    Vector<Obstacle> building;
    Vector<Vec2> dop;

    // Outer ring of rooms.

    // bottom line
    for (int k = 1; k < rooms-1; k++)
    {
        Vector<Obstacle> room;
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
    for (int k = 2; k < rooms-1; k++)
    {
        Vector<Obstacle> room;
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
        Vector<Obstacle> room;
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
        Vector<Obstacle> room;
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
    building.push_back(Obstacle(x, y, w, h));
    x = rooms-0.5; y = 0.5; w = 0.5; h = 0.5;
    building.push_back(Obstacle(x, y, w, h));
    x = 0.5; y = rooms-0.5; w = 0.5; h = 0.5;
    building.push_back(Obstacle(x, y, w, h));
    x = rooms-0.5; y = rooms-0.5; w = 0.5; h = 0.5;
    building.push_back(Obstacle(x, y, w, h));



    // Inner ring of rooms.

    // bottom line
    for (int k = 3; k < rooms-3; k++)
    {
        Vector<Obstacle> room;
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
        Vector<Obstacle> room;
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
        Vector<Obstacle> room;
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
        Vector<Obstacle> room;
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
    building.push_back(Obstacle(x, y, w, h));


    // Make the building.
    for (int k = 0; k < building.size(); k++)
        building[k].transform();
    polygons.addObstacles(building);
    dropOffPoints << dop;


    // Scale all obstacles and drop off points by the room size.
    polygons.scale(roomSize);
    for (int i = 0; i < dropOffPoints.size(); i++)
        dropOffPoints[i].scale(roomSize, roomSize);
}

void World::logObstaclesToTxt()
{
    if (dropOffPoints.size() < 2)
    {
        qDebug() << "Not enough drop off points specified";
        return;
    }

    Vector<Obstacle> polygons = expandedStaticObstacles.getObstacles();

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
        ListIterator<Vec2> it = polygons[i].vertexIterator();
        while (it.hasNext())
        {
            log << it.next();
            log++;
            cornercounter++;
        }

        log++;
    }

    qDebug() << "Polygons:" << polygons.size() << "corners:" << cornercounter << ".";
}

// (Re)Build the simulation by adding all the bodies of the obstacles,
// cars, pedestrians, the passenger, and the taxi, to the Box2D simulation.
// It clears first, so it can be used to rebuild the simulation from the bodies
// in the word at any time. This function will rewrite the body pointers and
// also the back reference from the body pointer to the world object.
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

    // We set the gravity to zero since we a have a top down world.
    box2DSim.SetGravity(b2Vec2(0.0f, 0));

    // obstacles
    for (int i = 0; i < polygons.size(); i++)
    {
        b2BodyDef bodyDef;
        bodyDef.type = b2_staticBody;
        b2Body* body = box2DSim.CreateBody(&bodyDef);
        b2PolygonShape shape;
        b2Vec2 vertices[polygons.getObstacle(i).size()];
        ListIterator<Vec2> it = polygons.getObstacle(i).vertexIterator();
        int j = 0;
        while (it.hasNext())
        {
            Vec2& v = it.next();
            vertices[j++] = b2Vec2(v.x, v.y);
        }
        shape.Set(vertices, polygons.getObstacle(i).size());
        body->CreateFixture(&shape, 0.0f);
        body->id = polygons.getObstacle(i).getId();
        body->SetUserData(&(polygons.getObstacle(i)));
        polygons.getObstacle(i).setBody(body);
        polygons.getObstacle(i).physicsTransformOut(); // If the obstacle comes in transformed state, is this even needed?
    }

    // Unicycle agents.
    for (int i = 0; i < unicycleAgents.size(); i++)
    {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.linearDamping = config.agentLinearDamping;
        bodyDef.angularDamping = config.agentAngularDamping;
        bodyDef.position.Set(unicycleAgents[i].pos().x, unicycleAgents[i].pos().y);
        bodyDef.angle = unicycleAgents[i].rotation();
        b2Body* body = box2DSim.CreateBody(&bodyDef);

        b2PolygonShape shape;
        b2Vec2 vertices[unicycleAgents[i].size()];
        ListIterator<Vec2> it = unicycleAgents[i].vertexIterator();
        int j = 0;
        while (it.hasNext())
        {
            Vec2& v = it.next();
            vertices[j++] = b2Vec2(v.x, v.y);
        }
        shape.Set(vertices, unicycleAgents[i].size());

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &shape;
        fixtureDef.density = 1.0f;
        fixtureDef.friction = config.agentFriction; // This doesn't really do much.

        // This is ghost mode.
        // Dumb agents can't collide with each other.
        if (command.ghostMode /*&& unicycleAgents[i].getAgentId() > 0*/) // uncomment the agent id > 0 check to enable collisions.
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

// Loads a binary saved world state.
void World::load()
{
    QMutexLocker locker(&mutex);

    QFile file("data/world.bin");
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "World::load(): Could not open file" << file.fileName();
        return;
    }

    QDataStream in(&file);
    in >> mapId;
    in >> staticObstacles;
    in >> unicycleAgents;
    file.close();

    buildSimulation();

    for (int i = 0; i < unicycleAgents.size(); i++)
        if (!unicycleAgents[i].isActive())
            unicycleAgents[i].deactivate();
}

// Saves the world in a binary file.
void World::save() const
{
    QFile file("data/world.bin");
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "World::save(): Could not write to file" << file.fileName();
        return;
    }

    QDataStream out(&file);
    out << mapId;
    out << staticObstacles;
    out << unicycleAgents;
    file.close();
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
    //QMutexLocker locker(&mutex);

    // Step the physical simulation.
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

    // In ghost robot mode, the objects in the physical simulation don't collide and we
    // have to care for the collisions manually.
    if (command.ghostMode)
    {
        Polygon polMain;
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
const Vector<Obstacle>& World::getStaticObstacles() const
{
    return staticObstacles.getObstacles();
}

// Returns a list of the unicycle agents in the world.
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

void World::setParams(int trajectoryPlanningMethod, int trajectoryType, int predictionType, int heuristicType, uint frequency)
{
    for (int i = 0; i < unicycleAgents.size(); i++)
        unicycleAgents[i].setParams(trajectoryPlanningMethod, trajectoryType, predictionType, heuristicType, frequency);

    if (frequency == 10)
        simulationTimeStep = 0.1;
    else if (frequency == 20)
        simulationTimeStep = 0.05;
    else
        simulationTimeStep = 0.03;
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
            ((Obstacle*)(body1->GetUserData()))->collisionResponse((Obstacle*)(body2->GetUserData()));
        }
        if (body2->id > 0)
        {
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
//    for (int i = 0; i < staticObstacles.size(); i++)
//        staticObstacles.getObstacle(i).physicsTransformIn(); // Not really needed is it?
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

// Applies the forces to the world objects in the physical simulation through their body pointers.
// This function is called in every iteration of the main control loop after the control
// functions of the objects have been executed.
void World::physicsControl()
{
    for (int i = 0; i < unicycleAgents.size(); i++)
        unicycleAgents[i].physicsControl();
    for (int i = 0; i < staticObstacles.size(); i++)
        staticObstacles.getObstacle(i).physicsControl();
}

// Switches to the map with the id w.
// A seed can be forced by setting the parameter s to reproduce a specific sequence of maps.
void World::setMap(int m, int agents)
{
    //Statistics::setSeed(298438517); // set the seed here if you want to see a specific run
    //qDebug() << "Seed set to" << 298438517 << "check:" << Statistics::randomNumber();
    mapId = m;
    numAgents = agents;
    reset();
    //qDebug() << "agents:" << unicycleAgents;
}

// Draws the world overlay on the QPainter.
// The world overlay is additional visualization that is drawn on top of the GraphicsScene.
// The GraphicsScene uses the Qt graphics framework to visualize the polygons in state.world.
// The world overlay adds paths and trajectories, perception overlays, and the plan animation
// feature.
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
        painter->setPen(colorUtil.penBlueThick);
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

    // Draw the world map polygons.
    // This is the assumed map the agent is making.
    if (command.showWorldMap)
    {
        if (!staticObstacles.getObstacles().isEmpty() && staticObstacles.getObstacles()[0].type == Obstacle::FreeSpace)
        {
            painter->save();
            painter->setPen(colorUtil.pen);
            //painter->setBrush(colorUtil.brushLightGray);
            painter->fillRect(0, 0, state.world.width, state.world.height, colorUtil.brushGray);
            painter->drawRect(0, 0, state.world.width, state.world.height);
            painter->restore();
        }
        if (command.showExpandedWorldMap)
            expandedStaticObstacles.draw(painter, colorUtil.penDashed, colorUtil.brushLightGray, 0.5);
        staticObstacles.draw(painter, colorUtil.pen, colorUtil.brush, 0.5);
    }

    // The raw polygons.
    if (command.showWorldObstacles)
        polygons.draw(painter, colorUtil.pen, colorUtil.brush, 0.8);

    // The drop off points.
    if (command.showDropOffPoints)
    {
        for (int i = 0; i < dropOffPoints.size(); i++)
        {
            painter->save();
            double r = config.worldDropOffRadius;
            painter->translate(dropOffPoints[i]);
            painter->setPen(colorUtil.penThinDashed);
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

// Draws the world underlay on the QPainter.
// The world overlay is additional visualization that is drawn underneath the GraphicsScene.
// The GraphicsScene uses the Qt graphics framework to visualize the polygons in state.world.
// The world underlay adds paths and trajectories that are drawn underneath the agents.
void World::preDraw(QPainter *painter) const
{
    QMutexLocker locker(&mutex);

    // Plan trace.
    // It draws a history of all plans in the state.
    if (command.showPlanTrace)
    {
        painter->save();
        painter->setPen(colorUtil.penRed);
        for (int i = 0; i < state.size(); i++)
            state[i].uniTaxi.drawPlan(painter);
        painter->restore();
    }

    // Trajectory trace.
    // When enabled, it shows the history of all taxi positions in the state.
    if (command.showTrajectoryTrace)
    {
        double s = 1.0/painter->transform().m11();
        painter->save();

        painter->setPen(colorUtil.penBlueThick);
        painter->setBrush(colorUtil.brushRed);
        for (int i = 0; i < state.size(); i++)
            painter->drawEllipse(state[i].uniTaxi.pos(), s, s);

        painter->restore();
    }
}

// Adds unicycle agents to the world. The agents start on a random
// drop-off point which are assumed to be non-colliding. The number
// of agents to be added is set by setMap().
void World::addAgents()
{
    if (dropOffPoints.size() < numAgents)
    {
        qDebug() << "Warning! Not enough drop off points to spawn all agents! dropOffPoints:" << dropOffPoints.size() << "agents:" << numAgents;
        numAgents = dropOffPoints.size();
    }
    Vector<Vec2> permutedDropOffPoints = dropOffPoints;
    for (uint d = 0; d < dropOffPoints.size(); d++)
        permutedDropOffPoints.swap(d, Statistics::randomInt(0, dropOffPoints.size()-1));

    int id = 0;
    for (int i = 0; i < numAgents; i++)
    {
        UnicycleAgent ua;
        ua.setAgentId(id++);
        ua.init(permutedDropOffPoints[i]);
        ua.setPos(permutedDropOffPoints[i]);
        ua.setRotation(Statistics::uniformSample(-PI, PI));
        ua.setWorldDropOffPoints(getDropOffPoints());
        ua.setWorldStaticObstacles(staticObstacles.getObstacles());
        ua.setWorldExpandedStaticObstacles(expandedStaticObstacles.getObstacles());
        unicycleAgents.push_back(ua);
    }
}

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
        mapName = "outdoor";
    if (map == 4)
        mapName = "apartment";
    if (map == 5)
        mapName = "warehouse";
    if (map == 6)
        mapName = "office";
    if (map == 7)
        mapName = "clutter";

    return mapName;
}

QDebug operator<<(QDebug dbg, const World &w)
{
    Vector<Obstacle> obst = w.staticObstacles.getObstacles();
    dbg << "Static Obstacles:" << "\n";
    for (int i = 0; i < obst.size(); i++)
        dbg << "   " << obst[i]  << "\n";

    Vector<UnicycleObstacle> obst3 = w.getUnicycleObstacles();
    dbg << "Unicycle Obstacles:" << "\n";
    for (int i = 0; i < obst3.size(); i++)
        dbg << "   " << obst3[i]  << "\n";

    return dbg;
}

void World::streamOut(QDataStream& out) const
{
    out << width;
    out << height;
    out << mapId;
    out << dropOffPoints;
    out << staticObstacles;
    out << unicycleAgents;
}

void World::streamIn(QDataStream& in)
{
    in >> width;
    in >> height;
    in >> mapId;
    in >> dropOffPoints;
    in >> staticObstacles;
    in >> unicycleAgents;
}

QDataStream& operator<<(QDataStream& out, const World &w)
{
    w.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, World &w)
{
    w.streamIn(in);
    return in;
}

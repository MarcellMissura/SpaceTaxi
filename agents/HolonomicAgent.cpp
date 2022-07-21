#include "HolonomicAgent.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "blackboard/Command.h"
#include "util/StopWatch.h"
#include "util/ColorUtil.h"
#include "KeyframePlayer/BangBang2D.h"
#include "geometry/Box.h"
#include "VelocityProfile.h"
#include "util/Statistics.h"

// The HolonomicAgent is the agent that drives the holonomic taxi in the game.
// It behaves like (is a) HolonomicObstacle and has an integrated sense()
// act() robot control loop in the step() method. The taxi has a couple of
// perception data structures such as the occupancy grid, the nf1 table, the
// visibility graph, and the WorldModel which is a 2D polygonal scene. It has
// a bunch of controllers that compute waypoints, a trajectory, and finally
// an acceleration for x and y to reach a given traget quickly without
// colliding with a possibly moving obstacle.

HolonomicAgent::HolonomicAgent() : HolonomicObstacle()
{
    id = 0;
}

void HolonomicAgent::init(const Vec2 &p)
{
    setPos(p);
    setRotation(0);

    double w = 0.5*config.agentWidth;
    double h = 0.5*config.agentHeight;

    addVertex(Vec2(-w, h));
    addVertex(Vec2(-w, -h));
    addVertex(Vec2(w, -h));
    addVertex(Vec2(w, h));

    mainTarget.x = x;
    mainTarget.y = y;

    gridModel.init();
    pathAstar.init();
}

// Update the world model.
void HolonomicAgent::sense()
{

}

// Compute an action. This method results in the acceleration of the taxi
// (which are the controls) being set in the x and y directions.
void HolonomicAgent::act()
{
    StopWatch stopWatch;

//##############
//# HIGH LAYER #
//##############
// High level planning. Where to put the main target in the game?

    // If the target has been reached, pick a new random drop off point as target.
    if ((pos()-mainTarget.pos()).norm() < config.worldDropOffRadius)
    {
        int newTargetDropOffId = 0;
        do
        {
            newTargetDropOffId = Statistics::uniformSample(0, state.world.dropOffPoints.size()-EPSILON);
        } while (targetDropOffId == newTargetDropOffId);
        targetDropOffId = newTargetDropOffId;
        mainTarget.setPos(state.world.dropOffPoints[targetDropOffId]);
    }

//##############
//# PATH LAYER #
//##############
// Path planning. Planning a shortest path to the main target.

    intermediateTarget = mainTarget;

    // Path A*. Plain vanilla grid search with Euklidean distance.
    if (command.pathPlanningMethod == command.PathAStar)
    {
        stopWatch.start();
        pathAstar.setDebug(config.debugLevel);
        pathAstar.setGridModel(gridModel);
        pathAstar.setStartState(currentState.pos());
        pathAstar.setTargetState(mainTarget.pos());
        pathAstar.search();
        intermediateTarget.setPos(pathAstar.getWaypoint(config.astarTargetOffset));
        state.pathTime = stopWatch.elapsedTimeMs();
        //qDebug() << "A* time:" << state.pathAstarTime;
    }

    // Lazy Theta*. An any angle A* variate.
    if (command.pathPlanningMethod == command.LazyThetaStar)
    {
        stopWatch.start();
        lazyThetaStar.setDebug(config.debugLevel);
        lazyThetaStar.setGridModel(gridModel);
        lazyThetaStar.setStartState(currentState.pos());
        lazyThetaStar.setTargetState(mainTarget.pos());
        lazyThetaStar.search();
        intermediateTarget.setPos(lazyThetaStar.getWaypoint(config.astarTargetOffset));
        state.pathTime = stopWatch.elapsedTimeMs();
        //qDebug() << "LazyTheta* time:" << state.pathAstarTime;
    }

    // Full Visibility Graph Search.
    if (command.pathPlanningMethod == command.FullConstruct)
    {
        stopWatch.start();
        visibilityGraph.setDebug(config.debugLevel);
        visibilityGraph.setGeometricModel(geometricModel);
        visibilityGraph.setStart(currentState.pos());
        visibilityGraph.setTarget(mainTarget.pos());
        visibilityGraph.fullConstruct();
        visibilityGraph.search();
        intermediateTarget.setPos(visibilityGraph.getWaypoint(config.gmTargetOffset));
        state.pathTime = stopWatch.elapsedTimeMs();
        //qDebug() << "Vis graph search:" << state.visibilityGraphSearchTime;
        //qDebug() << "Collision checks:" << worldModel.collisonChecks;
    }

    // Naive Visibility Graph Search.
    if (command.pathPlanningMethod == command.NaiveConstruct)
    {
        stopWatch.start();
        visibilityGraph.setDebug(config.debugLevel);
        visibilityGraph.setGeometricModel(geometricModel);
        visibilityGraph.setStart(currentState.pos());
        visibilityGraph.setTarget(mainTarget.pos());
        visibilityGraph.naiveConstruct();
        visibilityGraph.search();
        intermediateTarget.setPos(visibilityGraph.getWaypoint(config.gmTargetOffset));
        state.pathTime = stopWatch.elapsedTimeMs();
        //qDebug() << "Vis graph search:" << state.visibilityGraphSearchTime;
        //qDebug() << "Collision checks:" << worldModel.collisonChecks;
    }

    // Minimal Visibility Graph Search.
    if (command.pathPlanningMethod == command.MinimalConstruct)
    {
        stopWatch.start();
        visibilityGraph.setDebug(config.debugLevel);
        visibilityGraph.setGeometricModel(geometricModel);
        visibilityGraph.setStart(currentState.pos());
        visibilityGraph.setTarget(mainTarget.pos());
        visibilityGraph.minimalConstruct(currentState.pos());
        state.pathTime = stopWatch.elapsedTimeMs();
        //qDebug() << "Vis graph search:" << state.visibilityGraphSearchTime;
        //qDebug() << "Collision checks:" << worldModel.collisonChecks;

        VelocityProfile b;
        intermediateTarget = b.getWaypoint(currentState, visibilityGraph.getPath(), config.gmTargetOffset);
    }

//####################
//# CONTROLLER LAYER #
//####################
// Action planning. High rate dynamic planning with bounded computation time.

    stopWatch.start();

    if (command.keyboard)
    {
        // Apply control force to holoTaxi (holoTaxi mass has to be 1.0).
        double ax = qBound(-config.agentLinearAccelerationLimit, command.ax, config.agentLinearAccelerationLimit);
        double ay = qBound(-config.agentLinearAccelerationLimit, command.ay, config.agentLinearAccelerationLimit);
        setAcc(ax, ay);
    }
    else if (command.trajectoryPlanningMethod == command.QuadraticDWA || command.trajectoryPlanningMethod == command.ArcDWA)
    {
        // Dynamic Window Approach for a holonomic vehicle.
        stopWatch.start();
        holonomicDWA.setDebug(config.debugLevel);
        holonomicDWA.setGeometricModel(geometricModel);
        holonomicDWA.setGridModel(gridModel);
        holonomicDWA.setStart(currentState);
        holonomicDWA.setTarget(intermediateTarget);
        Vec2 acc = holonomicDWA.search();
        acc.x = (fabs(vx) >= config.agentLinearVelocityLimit) ? 0 : acc.x;
        acc.y = (fabs(vy) >= config.agentLinearVelocityLimit) ? 0 : acc.y;
        setAcc(acc);
        state.dwaTime = stopWatch.elapsedTimeMs();
    }
    else
    {
        // Simple holonomic controller. Assumes obstacle free passage
        // and computes a bang bang trajectory for holonomic motion.

        BangBang2D kfp;
        kfp.setA(config.agentLinearAccelerationLimit);
        kfp.setV(config.agentLinearVelocityLimit);
        kfp.addKeyframe(currentState);
        kfp.addKeyframe(intermediateTarget);
        Vector<Keyframe2D> ctrl =  kfp.getTimeOptimalControlSequence();
        if (ctrl.size() > 0)
            setAcc(ctrl[0].acc());
    }
}

// Collision handler. The parameter is a pointer to the object the taxi collided with.
// This one is used to maintain a global counter of taxi collisions.
void HolonomicAgent::collisionResponse(const Obstacle *o)
{
    state.collisions++;
}

void HolonomicAgent::draw(QPainter *painter) const
{
    if (command.showOccupancyGrid)
        gridModel.draw(painter);

    if (command.showGeometricModel)
    {
        GeometricModel wm = geometricModel;
        //wm.predict(command.planAnimationTime);
        wm.draw(painter);

        if (command.showPlanAnimation)
        {
            VelocityProfile v;
            Hpm2D wp = v.getWaypoint(currentState, visibilityGraph.getPath(), command.planAnimationTime);
            QPen pen;
            painter->save();
            pen.setWidthF(0.1);
            painter->setPen(pen);
            painter->drawEllipse(wp.pos(), 0.1, 0.1);
            painter->restore();
        }
    }

    if (command.pathPlanningMethod == command.FullConstruct || command.pathPlanningMethod == command.NaiveConstruct || command.pathPlanningMethod == command.MinimalConstruct)
        visibilityGraph.draw(painter);

    if (command.pathPlanningMethod == command.PathAStar)
        pathAstar.draw(painter);

    if (command.pathPlanningMethod == command.LazyThetaStar)
        lazyThetaStar.draw(painter);

    if (command.trajectoryPlanningMethod == command.QuadraticDWA)
        holonomicDWA.draw(painter);
}

// Robot control step.
void HolonomicAgent::step()
{
    StopWatch sw;
    sw.start();
    sense();
    state.senseTime = sw.elapsedTimeMs();
    sw.start();
    act();
    state.actTime = sw.elapsedTimeMs();
}

// Generates a human readable name.
const QString HolonomicAgent::getName() const
{
    QString name = "HolonomicAgent" + QString::number(id);
    return name;
}

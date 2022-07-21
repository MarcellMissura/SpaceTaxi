#include "UnicycleAgent.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "blackboard/Command.h"
#include "controller/VelocityProfile.h"
#include "geometry/Collision.h"
#include "util/Statistics.h"
#include "util/StopWatch.h"
#include "util/ColorUtil.h"

// The UnicycleAgent is the controller that drives the unicycle agents in the game.
// It behaves like (is a) UnicycleObstacle and has an integrated sense()-act() loop.

UnicycleAgent::UnicycleAgent() : UnicycleObstacle()
{
    agentId = 0;
    targetDropOffId = 0;
    stuckTimer = 0;
    stuckDetectionCounter = 0;
    isTooClose = false;
    isStuck = false;
    score = 0;
    milage = 0;
    collisions = 0;
    inCollision = false;
    collided = false;
    stucks = 0;
    closes = 0;
    staticWorldPathSuccess = true;
    trajectorySuccess = true;
    pathTime = 0;
    trajectoryTime = 0;

    trajectoryPlanningMethod = command.trajectoryPlanningMethod;
    trajectoryType = command.trajectoryType;
    predictionType = command.predictionType;
    heuristicType = command.heuristic;
}

// Initializes the polygon that represents the agent and inits
// the involved data structures.
void UnicycleAgent::init(const Vec2& p)
{
    // Set up the polygon that describes the agent.
    double w = 0.5*config.agentWidth;
    double h = 0.5*config.agentHeight;
    addVertex(Vec2(-w, h));
    addVertex(Vec2(-w, -h));
    addVertex(Vec2(w, -0.8*h));
    addVertex(Vec2(w, 0.8*h));
    setConvex();

    //double w = 0.5*config.agentWidth;
    //double h = 0.5*config.agentHeight;
    int subDivs = 5;
    hullPolygon.clear();
    for (int i = 0; i < subDivs; i++)
    {
        double frac = (double)i/subDivs;
        hullPolygon.addVertex(Vec2(-w, h + frac*(-h-h)));
    }
    for (int i = 0; i < subDivs; i++)
    {
        double frac = (double)i/subDivs;
        hullPolygon.addVertex(Vec2(-w + frac*(w+w), -h + frac*(-0.8*h+h)));
    }
    for (int i = 0; i < subDivs; i++)
    {
        double frac = (double)i/subDivs;
        hullPolygon.addVertex(Vec2(w, -0.8*h + frac*(0.8*h+0.8*h)));
    }
    for (int i = 0; i < subDivs; i++)
    {
        double frac = (double)i/subDivs;
        hullPolygon.addVertex(Vec2(w + frac*(-w-w), 0.8*h + frac*(h-0.8*h)));
    }
    hullPolygon.grow(config.gmPolygonExpansionMargin);
    hullPolygon.setConvex();

    // Set the initial pose of the agent.
    setPos(p);
    setRotation(0);

    // Init the main target to lie on the agent.
    mainTarget.x = x;
    mainTarget.y = y;

    // Init path and trajectory controllers.
    shortTermAbortingAStar.init();

    // The assumed first line of perception: a rectangular occupancy grid around the robot.
    // The area and resolution of the sensed grid is initialized based on config parameters.
    sensedGrid.setDim(2);
    sensedGrid.setN(Vec2u(config.gridHeight/config.gridCellSize+1, config.gridWidth/config.gridCellSize+1));
    sensedGrid.setMin(Vec2(-config.gridHeight/2+config.gridOffset, -config.gridWidth/2));
    sensedGrid.setMax(Vec2(config.gridHeight/2+config.gridOffset, config.gridWidth/2));
    sensedGrid.init();
    dilatedSensedGrid = sensedGrid;

    if (command.trajectoryPlanningMethod == command.RuleBase)
    {
        ruleBase.load("data/rulebase.dat");
        qDebug() << ruleBase.size() << "rules loaded";
    }
}

// Takes in the world drop off points input.
void UnicycleAgent::setWorldDropOffPoints(const Vector<Vec2> &value)
{
    worldDropOffPoints = value;
    targetDropOffId = 0;

    // Generate 150 targets randomly chosen from the available drop off points.
    dropOffPointQueue.clear();
    for (uint i = 0; i < 150; i++)
    {
        uint idx = Statistics::randomInt(0, worldDropOffPoints.size()-1);
        while (!dropOffPointQueue.empty() && idx == dropOffPointQueue.last())
            idx = Statistics::randomInt(0, worldDropOffPoints.size()-1);
        dropOffPointQueue << idx;
    }
}

// Takes in the world expanded static obstacles input.
void UnicycleAgent::setWorldExpandedStaticObstacles(const Vector<Obstacle> &value)
{
    worldExpandedStaticObstacles.setObstacles(value);
}

// Takes in the world static obstacles input.
void UnicycleAgent::setWorldStaticObstacles(const Vector<Obstacle> &value)
{
    worldStaticObstacles = value;
}

// Takes in the world unicycles input.
void UnicycleAgent::setWorldUnicycleObstacles(const Vector<UnicycleObstacle> &value)
{
    worldUnicycleObstacles = value;
}

// Sets the trajectory and controller parameters.
void UnicycleAgent::setParams(int trajectoryPlanningMethod, int trajectoryType, int predictionType, int heuristicType, uint frequency)
{
    this->trajectoryPlanningMethod = trajectoryPlanningMethod;
    this->trajectoryType = trajectoryType;
    this->predictionType = predictionType;
    this->heuristicType = heuristicType;
    shortTermAbortingAStar.trajectoryType = trajectoryType;
    shortTermAbortingAStar.heuristicType = heuristicType;
    if (frequency == 10)
    {
        timeStep = 0.1;
        shortTermAbortingAStar.timeLimit = 84;
    }
    else if (frequency == 20)
    {
        timeStep = 0.05;
        shortTermAbortingAStar.timeLimit = 36;
    }
    else
    {
        timeStep = 0.03;
        shortTermAbortingAStar.timeLimit = 19;
    }
}

bool UnicycleAgent::intersects(const Polygon &p) const
{
    return Polygon::intersects(p);
}

// In the sense method, a world represenation is computed. Typically, this a map of the environment.
void UnicycleAgent::sense()
{
    // Sense the local occupancy map (the sensed grid).
    // The sensed grid is a local 8m x 8m occupancy grid, our assumed input. It is nearly centered
    // around the robot but is also pushed forward a bit so that the robot would see more to the
    // front than to the back. The world polygons are transformed into the robot frame and the
    // sensed grid is computed by drawing the polygons onto the grid like onto an image. Then,
    // the sensed grid is processed to a local map of polygons (the sensed polygons). Separate
    // geometric models are extracted for static and for dynamic obstacles. The sensed grid is
    // dilated and blurred after extraction.
    GeometricModel transformedWorldPolygons;
    transformedWorldPolygons.setObstacles(worldStaticObstacles);
    transformedWorldPolygons -= pose(); // Transform from world to local coordinates.
    transformedWorldPolygons.transform();
    sensedGrid.computeOccupancyGrid(transformedWorldPolygons.getObstacles());
    dilatedSensedGrid = sensedGrid;
    sensedGrid.dilate(config.staaCollisionGridDilation);
    dilatedSensedGrid.dilate(config.gridSensedDilationRadius);
    dilatedSensedGrid.blur(config.gridBlurRadius);

    staticGeometricModel.setObstacles(worldStaticObstacles);
    staticGeometricModel -= pose();
    staticGeometricModel.transform();
    staticGeometricModel.clip(sensedGrid.boundingBox());

    expandedStaticGeometricModel.setObstacles(worldExpandedStaticObstacles.getObstacles());
    expandedStaticGeometricModel -= pose();
    expandedStaticGeometricModel.transform();
    expandedStaticGeometricModel.clip(sensedGrid.boundingBox());

    // Compute the dynamic geometric model from the unicycle agents in the world.
    // Only the ones located in the area of the sensed grid are added to the model.
    // The dynamic geometric model is expressed in local coordinates.
    dynamicGeometricModel.clear();
    expandedDynamicGeometricModel.clear();
    Vector<UnicycleObstacle> uo = worldUnicycleObstacles;
    for (uint i = 0; i < uo.size(); i++)
    {
        uo[i] -= pose(); // Transform from world to local coordinates.
        if (sensedGrid.contains(uo[i].pos()))
        {
            if (predictionType == command.Holonomic)
                uo[i].setVel(uo[i].v,0);
            else if (predictionType == command.None)
                uo[i].setVel(0,0);
            uo[i].setAcc(0,0);
            dynamicGeometricModel.addObstacle(uo[i]);
            UnicycleObstacle ob;
            ob.setVertices(hullPolygon.getVertices());
            ob.setPose(uo[i].pose());
            ob.setVel(uo[i].vel());
            expandedDynamicGeometricModel.addObstacle(ob);
        }
    }
    //expandedDynamicGeometricModel.grow(config.gmPolygonExpansionMargin);
    //expandedDynamicGeometricModel.transform(); // why?

    computeRayModel();
}

// Compute an action. This method results in the acceleration of the agent being set.
void UnicycleAgent::act()
{
    //##############
    //# HIGH LAYER #
    //##############
    // High level planning. Where to put the main target?
    // The main target is the (x,y) location of a drop-off point on the map
    // in world coordinates.

    // If the target has been reached, pick a new random drop off point as target.
    if ((pos()-mainTarget).norm() < config.worldDropOffRadius)
    {
        if (targetDropOffId != 0)
            score++;

        if (targetDropOffId == dropOffPointQueue.size())
        {
            qDebug() << "agent" << getAgentId() << "MISSION COMPLETE.";
            targetDropOffId++;
        }

        if (targetDropOffId < dropOffPointQueue.size())
        {
            mainTarget = worldDropOffPoints[dropOffPointQueue[targetDropOffId++]];
            //qDebug() << "agent" << getAgentId() << "updated main target" << mainTarget << dropOffPointQueue[targetDropOffId-1];
        }
    }

    //##############
    //# PATH LAYER #
    //##############
    // The path layer searches the world map for an obstacle-avoiding shortest path to the main target.
    // Two paths are computed, one that regards only the static obstacles in the map, and one that also
    // takes the dynamic obstacles within the sensed grid into account. These paths are then used to extract the
    // intermediate target and the carrot. The intermediate target is taken from the static world path where it
    // intersects the boundary of the sensed grid. The carrot is taken at a short distance along the dynamic world
    // path so that it would lead the PD controller and the DWA controllers around moving obstacles. However, it
    // often happens that a moving obstacle blocks a narrow passage and either no dynamic path can be found, or an
    // alternative path is found that deviates strongly from the static path.
    // If the dynamic path is blocked or suspicious, the carrot is taken from the static path after all.
    // All paths are computed with the Minimal Construct algorithm in a GeometricModel.

    // For the computation of both paths (static and dynamic) a unified geometric model is built successively.
    // First, the expanded world obstacles and the expanded local obstacles are both added to the model. The obstacles
    // are expanded a bit so that paths cannot go through too narrow spaces. The locally sensed dilated polygons
    // are added to make sure the intermediate target does not end up inside it. The starting point of the world
    // path is the location of the agent and the end point is the main target. Both points are moved out of the
    // obstacle along the closest edge normal if they happen to be inside an obstacle. Due to this trick, the
    // static world path can always be found unless there really is no path in the map from start to target.

    unifiedGeometricModel.clear();
    unifiedGeometricModel.setObstacles(worldExpandedStaticObstacles.getObstacles());
    unifiedGeometricModel -= pose(); // Convert to local coordinates.
    unifiedGeometricModel.transform();
    unifiedGeometricModel.rewriteIds();

    StopWatch sw;
    sw.start();
    const Vector<Vec2>& pp = unifiedGeometricModel.computePath(Vec2(), mainTarget-pose());
    staticWorldPathSuccess = !pp.isEmpty();
    if (staticWorldPathSuccess)
    {
        staticWorldPath = pp;

        // Determine the intermediate target.
        // The intermediate target is expressed in local coordinates.
        // The intermediate target is the intersection of the static world path and
        // the boundary of the sensed grid.
        Box box = sensedGrid.boundingBox();
        box.grow(-0.1);
        intermediateTarget = unifiedGeometricModel.pathBoxIntersection(box);
    }
    else
    {
        qDebug() << "agent" << getAgentId() << state.frameId << "Static world path computation failed from" << pos() << "to:" << mainTarget;
//        state.stop = 1;
//        unifiedGeometricModel.setDebug(50);
//        unifiedGeometricModel.computePath(Vec2(), mainTarget-pose());
//        unifiedGeometricModel.setDebug(0);
    }

    // Now compute the dynamic path regarding also the moving obstacles that are seen in the sensed grid.
    // To this end, we construct a unified geometric model that includes both the static and the dynamic
    // obstacles. The dynamic path is computed in local coordinates only up to the intermediate target.
    // The dynamic path is then used to extract the carrot a short ditance away from the agent. The carrot
    // is the reason why the dynamic path is computed using a unified geometric model so that short-horizon
    // controllers would be guided around moving obstacles. Sometimes, moving obstacles cover the target point
    // Such obstacles are erased from the model before path computation. Also, moving obstacles can block narrow
    // passages in which either no dynamic path can be found at all, or en entirely different path is found.
    // We should be able to use the local static model instead of the world static model.

    unifiedGeometricModel.reset();
    DynamicGeometricModel erasedDynamicModel = expandedDynamicGeometricModel;
    erasedDynamicModel.eraseObstaclesAt(intermediateTarget.pos());
    unifiedGeometricModel.addObstacles(erasedDynamicModel.getObstacles());
    unifiedGeometricModel.rewriteIds();
    unifiedGeometricModel.transform();
//    if (isFirstAgent())
//        unifiedGeometricModel.setDebug(config.debugLevel);
    const Vector<Vec2>& dpp = unifiedGeometricModel.computePath(Vec2(), mainTarget-pose());
    dynamicPathSuccess = !dpp.isEmpty();

//    if (isFirstAgent() && !dynamicPathSuccess)
//    {
//        qDebug() << state.frameId << "Dynamic global path computation failed to:" << intermediateTarget;
//        state.stop = 1;
//        unifiedGeometricModel.setDebug(5);
//        unifiedGeometricModel.getPathMoveOut(Vec2(), intermediateTarget);
//        unifiedGeometricModel.setDebug(0);
//    }

    // Check if the dynamic path deviates from the static path.
    if (dynamicPathSuccess)
    {
        dynamicPath = dpp; // It already comes in local coordinates.

        // The deviation between the static path and the dynamic path is computed in a way that the static
        // path and the reverse of the dynamic path are concatenated to a polygon and the area of the polygon
        // is computed. If the paths are equal, the area of this polygon is zero. If the area is larger than
        // a certain threshold, the two paths deviate strongly.
        Polygon pol;
        for (uint i = 0; i < staticWorldPath.size(); i++)
            pol << staticWorldPath[i];
        Vector<Vec2> dp = dynamicPath;
        dp.reverse();
        for (uint i = 0; i < dp.size(); i++)
            pol << dp[i];
        if (pol.area() > 20)
            dynamicPathSuccess = false;
    }

    // If the dynamic path is still good, take the carrot from it.
    if (dynamicPathSuccess && command.useDynamicPath && !(isBot() && command.ghostMode))
    {
        double dt = config.DWA_carrotOffset;
        if (trajectoryPlanningMethod == command.PD)
            dt = config.UPD_carrotOffset;
        VelocityProfile vp;
        Unicycle u;
        u.setVel(vel());
        Hpm2D h = vp.getWaypoint(u, dynamicPath, dt);
        carrot.setPos(h.pos());
        carrot.setHeading(h.heading());
    }

    // If the dynamic path failed, extract the carrot from the static path instead.
    else
    {
        double dt = config.DWA_carrotOffset;
        if (trajectoryPlanningMethod == command.PD)
            dt = config.UPD_carrotOffset;
        VelocityProfile vp;
        Unicycle u;
        u.setVel(vel());
        Hpm2D h = vp.getWaypoint(u, staticWorldPath, dt);
        carrot.setPos(h.pos());
        carrot.setHeading(h.heading());
    }

    pathTime = sw.elapsedTimeMs();
    if (isFirstAgent())
        state.pathTime = pathTime;


    //################
    //# REFLEX LAYER #
    //################
    // These are simple overriding controllers that act in situations where the agent is
    // either in a state of collision (force field reflex), i.e., inside the inflated area
    // of an obstacle where path planning and dwa both fail, or it is stuck in one place
    // for too long (stuckness reflex).




    // Stuckness reflex.
    // When the agent has been stuck for a while, this reflex executes PD control towards the intermediate target
    // for a whort time. You can observe this behavior when you switch to keyboard control with "K" and then do
    // nothing just watch. It can only detect when an agent is not moving, but not when the agent is stuck in an
    // infinite loop.
    stuckDetectionCounter = (vel().norm() < 0.3 && !isStuck) ? stuckDetectionCounter+1 : 0;

    //qDebug() << state.frameId << stuckDetectionCounter << "vel:" << vel().norm() << "stucks:" << stucks;

    // Activation.
    if (stuckDetectionCounter > 20 && state.frameId > 10)
    {
        stuckTimer = 10;
        if (trajectoryPlanningMethod == command.STAA)
            stuckTimer = 40;
        stuckDetectionCounter = 0;
        stucks++;
    }

    // If active, pd control towards the carrot.
    isStuck = (stuckTimer > 0);
    if (isStuck)
    {
        stuckTimer--;
        awayFromObst = carrot.pos();
        if (isBot() && command.ghostMode)
            awayFromObst = -carrot.pos();
        if (awayFromObst.norm() > 1.0)
            awayFromObst.normalize();
        //qDebug() << "Agent" << getName() << "is stuck. awayFromObst" << awayFromObst;
        if (command.stucknessReflex && (trajectoryPlanningMethod != command.STAA || (isBot() && command.ghostMode)))
        {
            setAcc(forceReaction(awayFromObst));
            //qDebug() << "Agent" << getName() << "is stuck." << acc();
            return; // Override the control layer.
        }
    }

    // PD controller force field reflex. The force field reflex activates when the closest point
    // of the local geometric map and/or the closest point of the dynamic agents is inside the
    // hull polygon. It computes a force vector that pushes the agent away from the obstacles
    // in the opposite direction away from the closest point(s).
    isTooClose = false;
    if (command.forceFieldReflex && !(isBot() && command.ghostMode) && trajectoryPlanningMethod == command.PD)
    {
        awayFromObst.set(0);
        cp2.set(0);
        if (!dynamicGeometricModel.isEmpty())
        {
            Vec2 cpp = dynamicGeometricModel.closestPoint(Vec2());
            if (hullPolygon.intersects(cpp))
            {
                cp2 = cpp;
                awayFromObst += -cp2.normalized();
                awayFromObst.normalize();
                isTooClose = true;
                //qDebug() << "Force field should activate. cp2:" << cp2 << awayFromObst;
            }
        }
        cp1.set(0);
        if (!staticGeometricModel.isEmpty() && isTooClose)
        {
            Vec2 cpp = staticGeometricModel.closestPoint(Vec2());
            if (hullPolygon.intersects(cpp))
            {
                cp1 = cpp;
                awayFromObst = -cp1.normalized();
                isTooClose = true;
                //qDebug() << "Force field should activate. cp1:" << cp1 << awayFromObst;
            }
        }

    }

    // DWA Force field reflex.
    // The force field reflex activates when the agent is inside the expanded static geometric model. It computes a force vector that pushes
    // the agent away from the obstacle along the nearest normal. This is most robust if
    // the unexpanded polygons are used to compute the normal.
    else if (command.forceFieldReflex && !(isBot() && command.ghostMode) && trajectoryPlanningMethod == command.DWA)
    {
        awayFromObst.set(0);
        cp2.set(0);
        if (expandedDynamicGeometricModel.staticPointCollisionCheck(Vec2()) > -1)
        {
            Vec2 cpp = dynamicGeometricModel.closestPoint(Vec2());
            cp2 = cpp;
            awayFromObst += -cp2.normalized();
            awayFromObst.normalize();
            isTooClose = true;
            //qDebug() << "Force field should activate. cp2:" << cp2 << awayFromObst;
        }
        cp1.set(0);
        if (isTooClose && expandedStaticGeometricModel.pointCollisionCheck(Vec2()) > -1)
        {
            Vec2 cpp = staticGeometricModel.closestPoint(Vec2());
            cp1 = cpp;
            awayFromObst = -cp1.normalized();
            isTooClose = true;
            //qDebug() << "Force field should activate. cp1:" << cp1 << awayFromObst;
        }

    }

    if (isTooClose)
    {
        closes++;

        //qDebug() << "overriding control with" << awayFromObst;
        Vec2 forward(1,0);
        double activation = fabs(awayFromObst * forward);
        Vec2 pdAcc = pdControlTo(carrot.pos());
        setAcc(pdAcc);
        pdAcc = acc();
        Vec2 forceAcc = forceReaction(awayFromObst);
        setAcc(forceAcc);
        forceAcc = acc();
        Vec2 accOut = (1.0-activation)*pdAcc + (activation)*forceAcc;
        setAcc(accOut);
        //qDebug() << "activation:" << activation << "pd:" << pdAcc << "force:" << forceAcc << "acc:" << acc();
        return; // Override the control layer.
    }



    //####################
    //# CONTROLLER LAYER #
    //####################
    // Action planning. High rate dynamic planning with bounded computation time.
    // The controller layer computes and sets the acceleration of the agent using Aborting A* or DWA or a PD controller.

    // Dumb agent override.
    if (command.ghostMode && isBot())
    {
        // Dumb agents simply execute a PD controller towards the carrot taken from the static path.
        setAcc(pdControlTo(carrot.pos()));
        return;
    }

    // Keyboard control override.
    joystickCarrot = carrot.pos();
    if (command.joystick && isFirstAgent())
    {
        ruleBase.setDebug(config.debugLevel);
        ruleBase.query(rays, carrot.pos());
        joystickCarrot = Vec2(-command.ay, command.ax) + pos() - pose();
        setAcc(pdControlTo(joystickCarrot));
    }
    else if (command.keyboard && isFirstAgent())
    {
        //carrot.setPos(Vec2(-command.ay*config.UPD_carrotOffset, command.ax*config.UPD_carrotOffset) + pos() - pose());
        //setAcc(pdControlTo(carrot.pos()));
        setAcc(command.ax, command.ay);
    }
    else if (trajectoryPlanningMethod == command.RuleBase)
    {
        ruleBase.setDebug(config.debugLevel);
        Vec2 ccarrot = ruleBase.query(rays, carrot.pos());
        setAcc(pdControlTo(ccarrot));
        //qDebug() << ccarrot << ccarrot.pos().norm2() << pdControlTo(ccarrot.pos()) << acc();
    }
    else if (trajectoryPlanningMethod == command.DWA)
    {
        // This is a Unicycle DWA used to control a nonholonomic agent.

        StopWatch sw;
        sw.start();

        Unicycle localUni;
        localUni.setVel(vel());

        // Dynamic Window Approach based on nonholonomic trajectory types:
        // arc, B0 spline, and Fresnel integrals.
        //unicycleDWA.setDebug(config.debugLevel);
        unicycleDWA.setTrajectoryType(trajectoryType);
        unicycleDWA.setDynamicGeometricModel(expandedDynamicGeometricModel); // For collision checking.
        unicycleDWA.setGeometricModel(expandedStaticGeometricModel); // For path searches
        unicycleDWA.setGridModel(dilatedSensedGrid);
        unicycleDWA.setStart(localUni);
        unicycleDWA.setCarrot(carrot.pos());
        unicycleDWA.setTarget(intermediateTarget);
        Vec2 acc = unicycleDWA.search();
        double a = acc.x;
        double b = acc.y;
        setAcc(a, b);

        // Measure execution time.
        trajectoryTime = sw.elapsedTimeMs();
        if (isFirstAgent())
            state.trajectoryTime = sw.elapsedTimeMs();
    }
    else if (trajectoryPlanningMethod == command.STAA)
    {
        StopWatch sw;
        sw.start();

        Unicycle localUni;
        localUni.setVel(vel());

//        if (isFirstAgent())
//            shortTermAbortingAStar.setDebug(config.debugLevel);
        shortTermAbortingAStar.trajectoryType = trajectoryType;
        shortTermAbortingAStar.heuristicType = heuristicType;
        shortTermAbortingAStar.setStaticGeometricModel(staticGeometricModel);
        shortTermAbortingAStar.setExpandedStaticGeometricModel(expandedStaticGeometricModel);
        shortTermAbortingAStar.setDynamicGeometricModel(dynamicGeometricModel);
        shortTermAbortingAStar.setExpandedDynamicGeometricModel(expandedDynamicGeometricModel);
        shortTermAbortingAStar.setSensedGrid(sensedGrid);
        shortTermAbortingAStar.setDilatedSensedGrid(dilatedSensedGrid);
        shortTermAbortingAStar.setStartState(localUni);
        shortTermAbortingAStar.setTargetState(intermediateTarget);
        if (command.stucknessReflex)
            shortTermAbortingAStar.setStuck(stuckTimer > 0);
        trajectorySuccess = shortTermAbortingAStar.aStarSearch();
        Vec2 acc = shortTermAbortingAStar.getAction();
        setAcc(acc);

        if (isFirstAgent())
        {
            state.aasExpansions = shortTermAbortingAStar.expansions;
            //state.aasOpen = shortTermAbortingAStar.open.size();
            state.aasClosed = shortTermAbortingAStar.closed; // This actually means how many expansions coming from closed cells were ignored.
            state.aasCollided = shortTermAbortingAStar.collided;
            state.aasProcessed = shortTermAbortingAStar.processed;
            state.aasDried = shortTermAbortingAStar.dried;
            state.aasFinished = shortTermAbortingAStar.finished;
        }

// For debugging STAA* fails...
//        if (isFirstAgent() && !trajectorySuccess)
//        {
//            qDebug() << "agent:" << getAgentId() << "frame:" << state.frameId << "STAA* failed. acc:" << shortTermAbortingAStar.getAction() << "expansions:" << shortTermAbortingAStar.getExpansions() << "processed:" << shortTermAbortingAStar.processed;
//            state.stop = 1;
//            shortTermAbortingAStar.setDebug(10);
//            command.useTimeAbort = false;
//            config.staasExpansionLimit = shortTermAbortingAStar.getExpansions()+1;
//            shortTermAbortingAStar.trajectoryType = trajectoryType;
//            shortTermAbortingAStar.setStaticGeometricModel(staticGeometricModel);
//            shortTermAbortingAStar.setExpandedStaticGeometricModel(expandedStaticGeometricModel);
//            shortTermAbortingAStar.setDynamicGeometricModel(dynamicGeometricModel);
//            shortTermAbortingAStar.setExpandedDynamicGeometricModel(expandedDynamicGeometricModel);
//            shortTermAbortingAStar.setSensedGrid(sensedGrid);
//            shortTermAbortingAStar.setDilatedSensedGrid(dilatedSensedGrid);
//            shortTermAbortingAStar.setStartState(localUni);
//            shortTermAbortingAStar.setTargetState(intermediateTarget);
//            shortTermAbortingAStar.aStarSearch();
//            shortTermAbortingAStar.setDebug(0);
//        }

        // Measure execution time.
        trajectoryTime = sw.elapsedTimeMs();
        if (isFirstAgent())
            state.trajectoryTime = sw.elapsedTimeMs();
    }
    else if (trajectoryPlanningMethod == command.PD)
    {
        StopWatch sw;
        sw.start();

        // Simple PD controller that steers towards the carrot.
        // When the dynamic path fails or deviates too strongly, a and b are set to zero.
        Vec2 acc; // = 0
        if (dynamicPathSuccess)
            acc = pdControlTo(carrot.pos());
        setAcc(acc);

        // Measure execution time.
        trajectoryTime = sw.elapsedTimeMs();
        if (isFirstAgent())
            state.trajectoryTime = sw.elapsedTimeMs();
    }
}

void UnicycleAgent::learn()
{
    if (command.learn)
    {
        ruleBase.addRule(rays, carrot.pos(), joystickCarrot);
        ruleBase.save("data/rulebase.dat");
    }
}

// Computes the ray sensing model.
void UnicycleAgent::computeRayModel()
{
    // Generate and collision check the rays.
    Vec2 base(1,0);
    base.normalize(config.raysLength);
    rays.clear();
    for (int i = 0; i < config.raysNumber; i++)
    {
        Vec2 ray1 = base.rotated(-config.raysAngleRange + i*2*config.raysAngleRange/(config.raysNumber-1));
        Vec2 ray2 = staticGeometricModel.rayIntersection(Vec2(), ray1);
        Vec2 ray3 = dynamicGeometricModel.rayIntersection(Vec2(), ray2);
        rays << ray3.norm();
    }
}

// This is a simple PD controller that computes accelerations towards the target q.
// It also includes a dead band around q where the control output is zero.
Vec2 UnicycleAgent::pdControlTo(const Vec2 &q) const
{
    Vec2 forward(1,0);
    double projectedTransErr = q * forward;
    double rotErr = forward.angleTo(q);
    double a = config.UPD_Kp_lin * projectedTransErr + config.UPD_Kd_lin * v;
    double b = config.UPD_Kp_rot * rotErr + config.UPD_Kd_rot * w;
//    if (q.norm2() < 0.025) // Dead band.
//        return Vec2();
    return Vec2(a,b);
}

// Takes in a normalized force vector and computes the accelerations.
Vec2 UnicycleAgent::forceReaction(const Vec2 &q) const
{
    Vec2 forward(1,0);
    Vec2 sideward(0,1);
    double projectedTransErr = q * forward;
    double projectedRotErr = q * sideward;
    double a = config.UPD_Kp_lin * projectedTransErr + config.UPD_Kd_lin * v;
    double b = (config.UPD_Kp_rot * projectedRotErr + config.UPD_Kd_rot * w);
    //qDebug() << "in:" << q << "trans:" << projectedTransErr << v << a << "rot:" << projectedRotErr << w << b;
//    if (q.norm2() < 0.025) // Dead band.
//        return Vec2();
    return Vec2(a,b);
}

// Robot control step.
void UnicycleAgent::step()
{
    StopWatch sw;
    sw.start();
    sense();
    if (isFirstAgent())
        state.senseTime = sw.elapsedTimeMs();
    sw.start();
    act();
    if (isFirstAgent())
        state.actTime = sw.elapsedTimeMs();
    learn();
    if (inCollision && !collided) // This is to filter out multiple collisions.
        inCollision = false;
    collided = false;
}

// Sets the agent id.
void UnicycleAgent::setAgentId(int agentId)
{
    this->agentId = agentId;
}

// Returns the agent id.
int UnicycleAgent::getAgentId() const
{
    return agentId;
}

// Returns true unless this is the "main" agent with the id 0.
bool UnicycleAgent::isBot() const
{
    return (getAgentId() > 0);
}

// Returns true if this is the "main" agent with the id 0.
bool UnicycleAgent::isFirstAgent() const
{
    return (getAgentId() == 0);
}

// Collision handler. The parameter is a pointer to the object the agent collided with.
// This is used to maintain a global counter of agent collisions.
void UnicycleAgent::collisionResponse(const Obstacle *o)
{
    //qDebug() << "Collision callback inCol:" << inCollision << "cols:" << collisions;
    if (!inCollision)
        collisions++;
    inCollision = true;
    collided = true;
    //qDebug() << state.frameId << "Collision" << getName() << "with" << o->getName() << "vel:" << vel() << o->isStatic();
}

// QPainter drawing code.
void UnicycleAgent::draw(QPainter *painter) const
{
    // The main target.
    if (!isBot() && command.showTargets && mainTarget.norm() > EPSILON)
    {
        QPolygonF cross = colorUtil.getCrossPolygon();
        painter->save();
        painter->translate(mainTarget);
        painter->scale(0.125, 0.125); // determines the size of the cross
        painter->rotate(45);
        painter->setPen(colorUtil.pen);
        painter->setBrush(colorUtil.brushRed);
        painter->setOpacity(0.8);
        painter->drawPolygon(cross);
        painter->setOpacity(1.0);
        painter->setBrush(colorUtil.brush);
        painter->drawEllipse(QPointF(), 0.1, 0.1);
        painter->restore();
    }

    // Everything hereafter is drawn in local coordinates.
    painter->save();
    painter->translate(pos());
    painter->rotate(rotation()*RAD_TO_DEG);

    if (isFirstAgent())
    {
        // The sensed grid.
        if (command.showSensedGrid > 1)
        {
            dilatedSensedGrid.draw(painter, colorUtil.brushOrange);
            sensedGrid.draw(painter, colorUtil.brushRed);
            //globalOnlySensedGrid.draw(painter, colorUtil.brushRed);
            //localOnlySensedGrid.draw(painter, colorUtil.brushYellow);
        }

        // ...and its border.
        if (command.showSensedGrid > 0)
        {
            sensedGrid.drawBorder(painter);
        }

        // The Dijkstra map.
        if (command.showDijkstraMap)
        {
            dilatedSensedGrid.drawDijkstraMap(painter);
        }

        // The local geometric models (static and dynamic).
        if (command.showGeometricModel)
        {
            expandedStaticGeometricModel.draw(painter, colorUtil.penThick, colorUtil.brushOrange);
            staticGeometricModel.draw(painter, colorUtil.penThick, colorUtil.brushRed);
            //dynamicGeometricModel.draw(painter, colorUtil.penThick, colorUtil.brushRed);
            expandedDynamicGeometricModel.draw(painter, colorUtil.penThick, colorUtil.brushBlue, 0.1);
            //unifiedGeometricModel.draw(painter, colorUtil.penThick, colorUtil.brushBlue);
            sensedGrid.drawBorder(painter);
        }
    }


    // The body.
    if (command.showBody)
    {
        painter->setPen(colorUtil.penThick);
        if (isBot())
            painter->setBrush(colorUtil.brushRed);
        else
            painter->setBrush(colorUtil.brushYellow);
        Polygon body = *this;
        body -= pose();
        body.draw(painter);
    }


    // Label showing the agent id when debugLevel > 0.
    if (config.debugLevel > 2)
    {
        painter->save();
        painter->setPen(colorUtil.pen);
        painter->scale(-0.04, 0.04);
        painter->drawText(QPointF(), QString::number(getAgentId()));
        painter->restore();
    }

    // The pos dot.
    painter->save();
    painter->setPen(colorUtil.pen);
    painter->setBrush(colorUtil.brush);
    painter->scale(0.03, 0.03);
    painter->drawEllipse(QPointF(0, 0), 1, 1);
    painter->restore();

    // Draw the velocity vector.
    Vec2 vv = v*Vec2(1,0);
    if (!vv.isNull())
    {
        painter->save();
        painter->scale(0.2, 0.2);
        painter->drawLine(Vec2(), vv);
        painter->translate(vv);
        painter->rotate(vv.angle()*RAD_TO_DEG);
        painter->rotate(45);
        painter->setPen(colorUtil.penThick);
        painter->drawLine(Vec2(), Vec2(-0.2*vv.norm(), 0));
        painter->rotate(-90);
        painter->drawLine(Vec2(), Vec2(-0.2*vv.norm(), 0));
        painter->restore();
    }

    // The force field force vector.
    if (isTooClose)
    {
//        painter->save();
//        painter->setPen(colorUtil.penRedThick);
//        painter->drawLine(Vec2(), awayFromObst);
//        painter->translate(awayFromObst);
//        painter->rotate(180*awayFromObst.angle()/PI);
//        painter->rotate(45);
//        painter->drawLine(Vec2(), Vec2(-0.2*awayFromObst.norm(), 0));
//        painter->rotate(-90);
//        painter->drawLine(Vec2(), Vec2(-0.2*awayFromObst.norm(), 0));
//        painter->restore();

        // Hull polygon
        painter->setOpacity(0.1);
        painter->setPen(colorUtil.pen);
        painter->setBrush(colorUtil.brushMagenta);
        hullPolygon.draw(painter);
        painter->setOpacity(1.0);

        if (!cp1.isNull())
        {
            painter->save();
            painter->setPen(colorUtil.penRedThick);
            painter->drawLine(cp1, Vec2());
            painter->rotate(RAD_TO_DEG*(cp1.angle()+PI));
            painter->rotate(45);
            painter->drawLine(Vec2(), Vec2(-0.2*cp1.norm(), 0));
            painter->rotate(-90);
            painter->drawLine(Vec2(), Vec2(-0.2*cp1.norm(), 0));
            painter->restore();
        }

        if (!cp2.isNull())
        {
            painter->save();
            painter->setPen(colorUtil.penRedThick);
            painter->drawLine(cp2, Vec2());
            painter->rotate(RAD_TO_DEG*(cp2.angle()+PI));
            painter->rotate(45);
            painter->drawLine(Vec2(), Vec2(-0.2*cp2.norm(), 0));
            painter->rotate(-90);
            painter->drawLine(Vec2(), Vec2(-0.2*cp2.norm(), 0));
            painter->restore();
        }
    }

    // The stuckness vector.
    if (isStuck && !awayFromObst.isNull())
    {
        painter->save();
        painter->setPen(colorUtil.penGreenThick);
        painter->drawLine(Vec2(), awayFromObst);
        painter->translate(awayFromObst);
        painter->rotate(180*awayFromObst.angle()/PI);
        painter->rotate(45);
        painter->drawLine(Vec2(), Vec2(-0.2*awayFromObst.norm(), 0));
        painter->rotate(-90);
        painter->drawLine(Vec2(), Vec2(-0.2*awayFromObst.norm(), 0));
        painter->restore();
    }

    // Bots are done.
    if (isBot())
    {
        painter->restore();
        return;
    }

    // The intermediate target.
    if (command.showTargets && intermediateTarget.pos().norm() > EPSILON && trajectoryPlanningMethod == command.STAA)
    {
        QPolygonF cross = colorUtil.getCrossPolygon();
        painter->save();
        painter->translate(intermediateTarget.pos());
        painter->scale(0.08, 0.08); // determines the size of the cross
        painter->rotate(45);
        painter->setPen(colorUtil.pen);
        painter->setBrush(colorUtil.brushBlue);
        painter->setOpacity(0.8);
        painter->drawPolygon(cross);
        painter->setOpacity(1.0);
        painter->setBrush(colorUtil.brush);
        painter->drawEllipse(QPointF(), 0.1, 0.1);
        painter->restore();
    }

    // The carrot.
    if (command.showTargets && carrot.pos().norm() > EPSILON
        && (command.trajectoryPlanningMethod == command.DWA
            || command.trajectoryPlanningMethod == command.PD))
    {
        QPolygonF cross = colorUtil.getCrossPolygon();
        painter->save();
        painter->translate(carrot.pos());
        painter->scale(0.06, 0.06); // determines the size of the cross
        painter->rotate(45);
        painter->setPen(colorUtil.pen);
        painter->setBrush(colorUtil.brushOrange);
        painter->setOpacity(0.8);
        painter->drawPolygon(cross);
        painter->setOpacity(1.0);
        painter->setBrush(colorUtil.brush);
        painter->drawEllipse(QPointF(), 0.1, 0.1);
        painter->restore();
    }

    // The joystick carrot.
    if (command.showTargets && joystickCarrot.norm() > EPSILON && command.joystick)
    {
        QPolygonF cross = colorUtil.getCrossPolygon();
        painter->save();
        painter->translate(joystickCarrot);
        painter->scale(0.06, 0.06); // determines the size of the cross
        painter->rotate(45);
        painter->setPen(colorUtil.pen);
        painter->setBrush(colorUtil.brushGreen);
        painter->setOpacity(0.8);
        painter->drawPolygon(cross);
        painter->setOpacity(1.0);
        painter->setBrush(colorUtil.brush);
        painter->drawEllipse(QPointF(), 0.1, 0.1);
        painter->restore();
    }

    // The world paths.
    if (command.showWorldPath)
    {
        // Draw the static world path.
        if (staticWorldPathSuccess)
        {
            painter->save();
            painter->setPen(colorUtil.penBlueThick);
            painter->setBrush(colorUtil.brushBlue);
            painter->setOpacity(0.2);
            for (int i = 1; i < staticWorldPath.size(); i++)
                painter->drawLine(QLineF(staticWorldPath[i].x, staticWorldPath[i].y, staticWorldPath[i-1].x, staticWorldPath[i-1].y));
            painter->restore();
        }

        // Draw the dynamic path.
        if (dynamicPathSuccess)
        {
            painter->save();
            painter->setPen(colorUtil.penRedThick);
            painter->setBrush(colorUtil.brushRed);
            painter->setOpacity(0.2);
            for (int i = 1; i < dynamicPath.size(); i++)
                painter->drawLine(QLineF(dynamicPath[i].x, dynamicPath[i].y, dynamicPath[i-1].x, dynamicPath[i-1].y));
            painter->restore();
        }

        // Draw the velocity profile.
        if (false)
        {
            VelocityProfile vp;
            Unicycle u = *this;
            double totalTime = vp.getTotalTime(u, staticWorldPath);
            QPen pen;
            painter->save();
            pen.setWidthF(0.02);
            painter->setPen(pen);
            for (double t = 0; t < totalTime; t+= 0.2)
            {
                Hpm2D wp = vp.getWaypoint(u, staticWorldPath, t);
                painter->drawEllipse(wp.pos(), 0.1, 0.1);
            }
            painter->restore();
        }
    }

    // Visibility graphs.
    if (command.showWorldVisibilityGraph)
        unifiedGeometricModel.drawVisibilityGraph(painter); // This is used for global path searches.
    if (command.showLocalVisibilityGraph)
        shortTermAbortingAStar.drawVisibilityGraph(painter); // This is used for local path searches in the heuristic of AA*.

    // Draw the ray model.
    if (command.showRayModel)
    {
        painter->save();
        Vec2 base(1,0);
        for (int i = 0; i < rays.size(); i++)
        {
            Vec2 ray = base.rotated(-config.raysAngleRange + i*2*config.raysAngleRange/(config.raysNumber-1));
            ray.normalize(rays[i]);

            if (rays[i] < config.raysLength-EPSILON)
            {
                painter->setBrush(colorUtil.brushRed);
                painter->setPen(colorUtil.penRed);
            }
            else
            {
                painter->setBrush(colorUtil.brush);
                painter->setPen(colorUtil.pen);
            }
            painter->drawLine(QPointF(), ray);
            painter->drawEllipse(ray, 0.03, 0.03);
        }
        painter->restore();
    }

    // Controller visualization.
    if (trajectoryPlanningMethod == command.DWA)
    {
        unicycleDWA.draw(painter);
    }
    else if (trajectoryPlanningMethod == command.STAA)
    {
        shortTermAbortingAStar.draw(painter);
    }

    else if (trajectoryPlanningMethod == command.RuleBase)
    {
        ruleBase.draw(painter);
    }

    painter->restore();
}

// Draws the current plan of the unicycle.
void UnicycleAgent::drawPlan(QPainter *painter) const
{
    painter->save();
    painter->translate(pos());
    painter->rotate(rotation()*RAD_TO_DEG);
    painter->setPen(colorUtil.penThin);
    Vector<UnicycleSearchNode> trace = shortTermAbortingAStar.getPlan();
    trace.removeLast();
    trace.reverse();
    for (uint i = 0; i < trace.size(); i++)
        trace[i].drawTrajectory(painter);
    painter->restore();
}

// Generates a human readable name.
const QString UnicycleAgent::getName() const
{
    QString name = "UnicycleAgent" + QString::number(getAgentId());
    return name;
}

QDebug operator<<(QDebug dbg, const UnicycleAgent &o)
{
    if (dbg.autoInsertSpaces())
        dbg << o.getName() << "pos:" << o.pos() << "angle:" << o.rotation() << "vel:" << o.vel() << "acc:" << o.acc();
    else
        dbg << o.getName() << " pos: " << o.pos() << " angle: " << o.rotation() << " vel: " << o.vel() << " acc: " << o.acc();
    return dbg;
}

QDebug operator<<(QDebug dbg, const UnicycleAgent* o)
{
    if (dbg.autoInsertSpaces())
        dbg << o->getName() << "pos:" << o->pos() << "angle:" << o->rotation() << "vel:" << o->vel() << "acc:" << o->acc();
    else
        dbg << o->getName() << " pos: " << o->pos() << " angle: " << o->rotation() << " vel: " << o->vel() << " acc: " << o->acc();
    return dbg;
}

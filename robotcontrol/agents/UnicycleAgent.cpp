#include "UnicycleAgent.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "blackboard/Command.h"
#include "robotcontrol/controller/VelocityProfile.h"
#include "lib/geometry/Collision.h"
#include "lib/util/Statistics.h"
#include "lib/util/StopWatch.h"
#include "lib/util/ColorUtil.h"

// The UnicycleAgent is the controller that drives the unicycle agents in the game.
// It behaves like (is a) UnicycleObstacle and has an integrated sense()-act() loop.

UnicycleAgent::UnicycleAgent() : UnicycleObstacle()
{
    agentId = 0;
    targetDropOffId = 0;
    stuckTimer = 0;
    stuckDetectionCounter = 0;
    ebActive = false;
    ebA = 0;
    isStuck = false;
    score = 0;
    milage = 0;
    collisions = 0;
    inCollision = false;
    collided = false;
    stucks = 0;
    closes = 0;
    worldPathSuccess = true;
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
void UnicycleAgent::init(const Vec2& initialPos)
{
    // Set up the polygon that describes the agent.
    double w = 0.5*config.agentWidth;
    double h = 0.5*config.agentHeight;
    clear();
    appendVertex(Vec2(-w, h));
    appendVertex(Vec2(-w, -h));
    appendVertex(Vec2(w, -0.8*h));
    appendVertex(Vec2(w, 0.8*h));
    reverseOrder();
    setConvex();
    setCW();

    hullPolygon.clear();
    hullPolygon.appendVertex(Vec2(-w, h));
    hullPolygon.appendVertex(Vec2(-w, -h));
    hullPolygon.appendVertex(Vec2(-0.9*w, -h));
    hullPolygon.appendVertex(Vec2(0.9*w, -0.8*h));
    hullPolygon.appendVertex(Vec2(w, -0.8*h));
    hullPolygon.appendVertex(Vec2(w, 0.8*h));
    hullPolygon.appendVertex(Vec2(0.9*w, 0.8*h));
    hullPolygon.appendVertex(Vec2(-0.9*w, h));
    hullPolygon.reverseOrder();
    hullPolygon.setConvex();
    hullPolygon.setCW();
    hullPolygon.grow(config.gmAgentDilation);

    // Set the initial pose of the agent.
    setPos(initialPos);
    setOrientation(0);

    // Init the main target to lie on the agent.
    mainTarget = pos();

    // Init path and trajectory controllers.
    shortTermAbortingAStar.init();

    // The sensed grid is a rectangular occupancy grid around the robot.
    // The area and resolution of the sensed grid is initialized based on config parameters.
    sensedGrid.setDim(2);
    sensedGrid.setN(Vec2u(config.gridHeight/config.gridCellSize+1, config.gridWidth/config.gridCellSize+1));
    sensedGrid.setMin(Vec2(-config.gridHeight/2+config.gridOffset, -config.gridWidth/2));
    sensedGrid.setMax(Vec2(config.gridHeight/2+config.gridOffset, config.gridWidth/2));
    sensedGrid.init();
    occupanyGrid = sensedGrid;

    //if (command.trajectoryPlanningMethod == command.RuleBase)
    //{
        ruleBase.load("data/rulebase.dat");
        qDebug() << ruleBase.size() << "rules loaded";
    //}
}

// Takes in the world drop off points input.
void UnicycleAgent::setWorldDropOffPoints(const Vector<Vec2> &dops)
{
    dropOffPoints = dops;
    targetDropOffId = 0;

    // Generate 200 targets randomly chosen from the available drop off points.
    dropOffPointQueue.clear();
    for (uint i = 0; i < 200; i++)
    {
        uint idx = Statistics::randomInt(0, dropOffPoints.size()-1);
        while (!dropOffPointQueue.empty() && idx == dropOffPointQueue.last())
            idx = Statistics::randomInt(0, dropOffPoints.size()-1);
        dropOffPointQueue << idx;
    }
}

// Takes in the world expanded static obstacles input.
// This is what we consider to be the actual map the robot has built for itself.
void UnicycleAgent::setWorldMap(const GeometricModel &pols)
{
    worldMap = pols;
}

// Sets the world polygons. This is a little cheat that gives the agent
// access to the actual polygons that make up the world. They are used
// for the computation of the sensor simulation and the ray model.
void UnicycleAgent::setWorldPolygons(const GeometricModel &pols)
{
    worldPolygons = pols;
}

// Takes in the world unicycles input.
void UnicycleAgent::setWorldUnicycleObstacles(const Vector<UnicycleObstacle> &obst)
{
    worldDynamicObstacles = obst;
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

// In the sense method, a world represenation is computed that can be
// used for planning and localization.
void UnicycleAgent::sense()
{
    // We already have a GeometricModel of the world map in worldMap.
    // It comes in through the setWorldMap() function which the World
    // calls once after it has built a new map and then the world map
    // remains constant until the map is switched by the user. Even
    // though the world map is computed from the simulated environment,
    // it is fairly realistic and close to what a robot using a 2D lidar
    // would produce when building a map with LineSlam. The world map
    // is kept in world coordinates and used for path planning.

    // We compute the clipped world map though, the world map clipped
    // to the local map.
    clippedWorldMap = worldMap;
    clippedWorldMap -= pose(); // local
    clippedWorldMap.clip(sensedGrid.boundingBox());

    // Compute the 2D lidar sensor simulation and the ray model.
    // Both are computed by casting rays from the robot and finding the
    // first intersection point, if any, with the world polygons and the
    // dynamic world obstacles. The ray model is a low res model of laser
    // rays used as input for the rule base controller.
    simulateLaserSensor();
    computeRayModel();

    // From the laser sensor, we gain the visiblity polygon.
    // The visibility polygon is used for building the plygonal map and
    // for clearing free space.
    visibilityPolygon = laserSensor.getVisibilityPolygon();

    // We use the simulated lidar to compute the occupancy grid as the robot would do.
    // The occupancy grid is a local 8m x 8m occupancy grid. It is nearly centered around
    // the robot but is also pushed forward a bit so that the robot would see more to the
    // front than to the back.
    occupanyGrid.computeOccupancyGrid(laserSensor.readPointBuffer());

    // Here is a little simulation cheat where I delete cells from the occupancy grid
    // where the dynamic obstacles are.
    for (uint i = 0; i < worldDynamicObstacles.size(); i++)
    {
        UnicycleObstacle uo = worldDynamicObstacles[i];
        uo -= pose(); // Transform from world to local coordinates.
        if (sensedGrid.contains(uo.pos()))
        {
            uo.grow(config.gridCellSize);
            uo.transform();
            occupanyGrid.clearPolygon(uo);
        }
    }

    // The occupancy grid is dilated a little bit in order to
    // connect single cells to contiguous regios.
    occupanyGrid.dilate(config.gridSensedDilationRadius);

    // The sensed grid is computed by blurring the occupancy grid.
    //occupanyGrid.clearPolygon(visibilityPolygon); // This is to combat too much dilation.
    sensedGrid = occupanyGrid;
    sensedGrid.dilate(config.gridBlurRadius);
    sensedGrid.blur(config.gridBlurRadius);
    sensedGrid.max(occupanyGrid);



    // The occupany grid is processed to a local map of polygons (the sensed polygons).
    // Sensed polygons are important in order to avoid obstacles that are not in the map.
    // The dilated sensed polygons are used for path planning inside the local map.
    // The extraction of the sensed polygons from the occupancy grid is problematic. The
    // occupany map computed from the laser rays requires a bit of dilation in order to
    // connect sensed regions to contiguous thin clusters of cells that then also result
    // in thin polygons that are unreliable (might degenerate to a line) and rather useless
    // for collision checking because of tunneling. More dilation leads to overly extended
    // occupied regions that can even block narrow passages, even when the visibility
    // polygon is used for clearing after dilation. It would be better to try a new approach
    // and generate the sensed polygons directly from the laser rays somehow.
    localMap.clear();
    localMap.setPolygons(occupanyGrid.extractPolygons()); // local
    localMap.dilate(config.gmDilationRadius);

    // The clipped world map is added to the local map. This has the advantage that the map
    // polygons can be taken into account when path planning in the local map. This is actually
    // crucial, otherwise we get silly paths leading only around the polygons we can see but
    // through obstacles that are in the map. Adding the clipped world map requires good
    // localization.
    localMap += clippedWorldMap; // local

    // Now add UnicycleObstacle models to the local map taken from the unicycle agents in the world.
    // This is a simulation cheat as we are not yet able to detect and separate moving obstacles.
    // Only the moving obstacles inside the area of the local map are added. The unicycle obstacles
    // have an extended hull polygon that is used for path planning.
    for (uint i = 0; i < worldDynamicObstacles.size(); i++)
    {
        UnicycleObstacle uo = worldDynamicObstacles[i];
        uo -= pose(); // Transform from world to local coordinates.
        //if (visibilityPolygon.intersects(uo.pos()))
        if (sensedGrid.contains(uo.pos()))
        {
            if (predictionType == command.Holonomic)
                uo.setVel(uo.v,0);
            else if (predictionType == command.None)
                uo.setVel(0,0);
            uo.setAcc(0,0);
            localMap.addObstacle(uo); // local but untransformed
        }
    }

    localMap.setBounds(sensedGrid.boundingBox());
    localMap.renumber();
    localMap.autoPredict(); // Predict the future states of the agents.
    localMap.transform();
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
            mainTarget = dropOffPoints[dropOffPointQueue[targetDropOffId++]];
            //qDebug() << "agent" << getAgentId() << "updated main target" << mainTarget << dropOffPointQueue[targetDropOffId-1];
        }
    }

    //##############
    //# PATH LAYER #
    //##############
    // The path layer searches the world map for an obstacle-avoiding shortest path to the main target.
    // A Visibility Graph approach is used to compute the shortest path with the Minimal Construct algorithm.
    // https://www.hrl.uni-bonn.de/publications/missura18iros.pdf
    // The path is determined in two steps. First, a global path is computed in the world map. From the
    // global path an intermediate target is extracted at the boundary of the local map. Then, in order to
    // take obstacles into account that are not in the map, we use a local sensed map of the static and the
    // dynamic obstacles to compute a "dynamic" path that replaces the world path up to the intermediate
    // target. However, it often happens that a moving obstacle blocks a narrow passage or a doorway and
    // no dynamic path can be found. If the dynamic path fails, we fall back to static path up to the
    // intermediate target using only the static obstacles in the local map. If that fails as well, we just
    // use the world path. If there is no world path, there really is no way to the target and the robot
    // does nothing. The resulting path is then used to extract the carrot that serves as a short term
    // target for the PD controller, the DWA controller, and the RuleBase controller. If the carrot is
    // taken from the dynamic path, it helps leading these controllers around moving obstacles. The STAA*
    // controller plans towards the intermediate target and is thus not influenced by the carrot.
    // For the path computation, expanded obstacles are used so that paths cannot lead through too narrow
    // spaces and keep distance from obstacles. Sometimes, for example due to localization errors, the
    // starting point or the target might end up inside an obstacle, where no path can be found. We make
    // the path computation robust by moving the start and the target into free space.

    StopWatch sw;
    sw.start();

    // 1. Compute the world path.
    worldPathSuccess = worldMap.computeStaticPath(pos(), mainTarget);
    const Vector<Vec2>& pp = worldMap.getPath();
    if (worldPathSuccess)
    {
        // Transform the path to local coordinates.
        worldPath = pp-pose();

        // Determine the intermediate target.
        // The intermediate target is the intersection of the static world path and the
        // boundary of the sensed grid. The intermediate target is expressed in local coordinates.
        Box box = sensedGrid.boundingBox();
        box.grow(-0.1); // Exclude the exact boundary to avoid problems.
        intermediateTarget = box.intersection(worldPath);
    }
    else
    {
        // It's a big issue if we cannot find the world path. Probably there is no way to the target at all.
        // We can keep going for a short while with the path and the intermediate target we had last.

        qDebug() << "agent" << getAgentId() << state.frameId << "Static world path computation failed from" << pos() << "to:" << mainTarget;
        state.stop = 1;
        worldMap.computeStaticPath(pos(), mainTarget, 50);
    }

    // 2. Compute the static path in case it is needed as a fallback.
    staticPathSuccess = localMap.computeStaticPath(Vec2(), intermediateTarget.pos());
    staticPath = localMap.getPath();

//    if (isFirstAgent() && !staticPathSuccess)
//        qDebug() << state.frameId << "Static path computation failed to:" << intermediateTarget;

    // 3. Now compute the "dynamic" path up to the intermediate target using the sensed polygons and
    // the moving obstacles that are seen in the local map. Sometimes, moving obstacles cover the
    // target. Such obstacles are erased from the model before path computation. Also, moving obstacles can
    // block narrow passages and then no dynamic path can be found at all.
    localMap.resetSearch();
    dynamicPathSuccess = localMap.computeDynamicPath(Vec2(), intermediateTarget.pos());
    dynamicPath = localMap.getPath();

    if (isFirstAgent() && !dynamicPathSuccess)
    {
        //qDebug() << state.frameId << "Dynamic path computation failed to:" << intermediateTarget;
        //state.stop = 1;
        //localMap.resetSearch();
        //localMap.computeDynamicPath(Vec2(), intermediateTarget.pos(), 50);
    }


    // Carrot extraction.

    // If the dynamic path is good, take the carrot from it.
    if (dynamicPathSuccess && command.useDynamicPath)
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
    else if (staticPathSuccess)
    {
        double dt = config.DWA_carrotOffset;
        if (trajectoryPlanningMethod == command.PD)
            dt = config.UPD_carrotOffset;
        VelocityProfile vp;
        Unicycle u;
        u.setVel(vel());
        Hpm2D h = vp.getWaypoint(u, staticPath, dt);
        carrot.setPos(h.pos());
        carrot.setHeading(h.heading());
    }

    // If both the dynamic and the static path failed, try the world path.
    else if (worldPathSuccess)
    {
        double dt = config.DWA_carrotOffset;
        if (trajectoryPlanningMethod == command.PD)
            dt = config.UPD_carrotOffset;
        VelocityProfile vp;
        Unicycle u;
        u.setVel(vel());
        Hpm2D h = vp.getWaypoint(u, worldPath, dt);
        carrot.setPos(h.pos());
        carrot.setHeading(h.heading());
    }

    //qDebug() << "extracted carrot:" << carrot << dynamicPathSuccess << staticPathSuccess << worldPathSuccess;

    pathTime = sw.elapsedTimeMs();
    if (isFirstAgent())
        state.pathTime = pathTime;


    //################
    //# REFLEX LAYER #
    //################
    // These are simple overriding controllers that act in situations where the agent is
    // on a collision course (emergency brake reflex) or it is stuck in one place for too
    // long (stuckness reflex).

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
        stucknessImpulse = carrot.pos();
        if (stucknessImpulse.norm() > 1.0)
            stucknessImpulse.normalize();
        //qDebug() << "Agent" << getName() << "is stuck. awayFromObst" << awayFromObst;
        if (command.stucknessReflex && (trajectoryPlanningMethod != command.STAA))
        {
            setAcc(forceReaction(stucknessImpulse));
            //qDebug() << "Agent" << getName() << "is stuck." << acc();
            return; // Override the control layer.
        }
    }

    // Emergency brake reflex.
    // The emergency break reflex activates when the agent is on a collision course with a velocity
    // higher than what is permitted in order to execute a safe braking maneuver and come to a full
    // stop before the collision occurs. As a safety margin, only 80% of the possbile effort is
    // assumed for the braking. When the brake reflex is active, the reflex cancels out the part of
    // the acceleration space that would lead to a collision, i.e., it applies the "brake" itself
    // and leaves the trajectory controller to apply any remaining acceleration that then should be
    // even safer. The activation of the brake reflex is based on two prospected impact points that
    // are computed by ray casting a forward ray from the front left and the front right corner of
    // the agent. The distance of the shorter ray determines what velocity is permitted in order to
    // be able to come to a stop before the impact. In order to avoid activation jittering, there is
    // an inbuilt hysteresis where the reflex deactivates only below a lower velocity margin than
    // what is needed to activate the reflex.

    // The math: combine x(t) and v(t) to v(x)
    // x(t) = x0 + v0*t + (1/2)a*t*t;
    // v(t) = v0 + a*t
    // v(x) = t(x)*a + v0;
    // x² + px + q = 0
    // x = -p/2 +- sqrt(p²/4 - q)
    // (1/2)a*t*t + v0*t + x0 = x(t)
    // t² + (2*v0/a)*t + 2*(x0-x(t))/a = 0
    // t(x) = -v0/a +- sqrt(v0²/a² - 2dx/a)
    // t(x) = -v0/a +- a*sqrt(v0² - 2adx)
    // t(x) = (-v0 +- sqrt(v0² - 2adx)) / a
    // v(x) = sqrt(v0² + 2ax)

    double aIn = 0.8*config.agentLinearAccelerationLimit;
    double aOut = 0.6*config.agentLinearAccelerationLimit;
    double minDistance = 0.15;
    double w = 0.5*config.agentWidth;
    double h = 0.5*config.agentHeight;
    Vec2 tl = Vec2(sgn(v)*w,h);
    Vec2 tr = Vec2(sgn(v)*w,-h);
    Vec2 impactPoint1 = localMap.rayIntersection(tl, Vec2(sgn(v)*2,h));
    Vec2 impactPoint2 = localMap.rayIntersection(tr, Vec2(sgn(v)*2,-h));
    impactPoint = impactPoint1.norm() < impactPoint2.norm() ? impactPoint1 : impactPoint2;
    double distToImpact = min((impactPoint1-tl).norm(), (impactPoint2-tr).norm()) - minDistance;
    double impactVelocityIn = sgn(v)*sqrt(2*aIn*distToImpact); // Activation velocity.
    double impactVelocityOut = sgn(v)*sqrt(2*aOut*distToImpact); // Deactivation velocity.
    if (distToImpact <= 0 || sgn(v)*(v-impactVelocityIn) > 0 || (ebActive && (sgn(v)*(v-impactVelocityOut) > 0)))
    {
        // 0 = v(x) = sqrt(v0² + 2ax)
        // 0 = v0² + 2ax
        // a = -v0²/2x
        ebActive = command.emergencyBrakeReflex;
        ebA = (-(v*v)*sgn(v)) / (2*distToImpact); // The acceleration that needs to be set to stop at minDistance before impact.
        if (distToImpact <= 0)
            ebA = -v/timeStep;
//        qDebug() << state.frameId << "Emergency break active!"
//                 << "acc:" << acc()
//                 << "dist:" << distToImpact << (distToImpact <= 0)
//                 << "v:" << v << impactVelocityIn << impactVelocityOut << (sgn(v)*(v-impactVelocityIn) > 0)
//                 << "a:" << ebA;
    }
    else
    {
        ebActive = false;
    }


    //####################
    //# CONTROLLER LAYER #
    //####################
    // Action planning. High rate dynamic trajectory planning with bounded computation time.
    // The controller layer computes and sets the acceleration of the agent using Aborting A* or DWA or a PD controller.

    // Keyboard control override.
    joystickCarrot = carrot.pos();
    if (command.joystick && isFirstAgent())
    {
        ruleBase.query(rays, carrot.pos());
        joystickCarrot = Vec2(-command.ay, command.ax) + pos() - pose();
        Vec2 acc = pdControlTo(joystickCarrot);
        if (ebActive)
            acc.x = min(acc.x, ebA); // Apply the emergency break.
        setAcc(acc);
    }
    else if (command.keyboard && isFirstAgent())
    {
        Vec2 acc(command.ax, command.ay);
        if (ebActive)
            acc.x = min(acc.x, ebA); // Apply the emergency break.
        setAcc(acc);
    }
    else if (trajectoryPlanningMethod == command.PD)
    {
        StopWatch sw;
        sw.start();

        // Simple PD controller that steers towards the carrot.
        // When the dynamic path fails or deviates too strongly, a and b are set to zero.
        Vec2 acc = pdControlTo(carrot.pos());
        if (ebActive)
            acc.x = min(acc.x, ebA); // Apply the emergency break.
        setAcc(acc);

        // Measure execution time.
        trajectoryTime = sw.elapsedTimeMs();
        if (isFirstAgent())
            state.trajectoryTime = sw.elapsedTimeMs();
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
        unicycleDWA.setGeometricModel(localMap);
        unicycleDWA.setGridModel(sensedGrid);
        unicycleDWA.setStart(localUni);
        unicycleDWA.setCarrot(carrot.pos());
        Vec2 acc = unicycleDWA.search();
        if (ebActive)
            acc.x = min(acc.x, ebA); // Apply the emergency break.
        setAcc(acc);

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

        shortTermAbortingAStar.setGeometricModel(localMap);
        shortTermAbortingAStar.setGridModel(sensedGrid);
        shortTermAbortingAStar.setStartState(localUni);
        shortTermAbortingAStar.setTargetState(intermediateTarget);
        if (command.stucknessReflex)
            shortTermAbortingAStar.setStuck(stuckTimer > 0);
        trajectorySuccess = shortTermAbortingAStar.aStarSearch(isFirstAgent() ? config.debugLevel : 0);
        Vec2 acc = shortTermAbortingAStar.getAction();
        setAcc(acc);

        if (isFirstAgent())
        {
            state.aasExpansions = shortTermAbortingAStar.expansions;
            state.aasOpen = shortTermAbortingAStar.opened;
            state.aasClosed = shortTermAbortingAStar.closed; // This actually means how many expansions coming from closed cells were ignored.
            state.aasCollided = shortTermAbortingAStar.collided;
            state.aasProcessed = shortTermAbortingAStar.processed;
            state.aasDried = shortTermAbortingAStar.dried;
            state.aasFinished = shortTermAbortingAStar.finished;
            state.aasDepth = shortTermAbortingAStar.depth;
            state.aasScore = shortTermAbortingAStar.score;
        }

        // Measure execution time.
        trajectoryTime = sw.elapsedTimeMs();
        if (isFirstAgent())
            state.trajectoryTime = sw.elapsedTimeMs();
    }
    else if (trajectoryPlanningMethod == command.RuleBase)
    {
        Vec2 ccarrot = ruleBase.query(rays, carrot.pos());
        Vec2 acc = pdControlTo(ccarrot);
        if (ebActive)
            acc.x = min(acc.x, ebA); // Apply the emergency break.
        setAcc(acc);
        //qDebug() << ccarrot << ccarrot.pos().norm2() << pdControlTo(ccarrot.pos()) << acc();
    }
}

void UnicycleAgent::learn()
{
    if (command.learn)
    {
        ruleBase.addRule(rays, carrot.pos(), joystickCarrot);
        if (isFirstAgent())
            ruleBase.save("data/rulebase.dat");
    }
}

// Computes the ray model from the world polygons and the world dynamic obstacles.
void UnicycleAgent::computeRayModel()
{
    // Actually I would prefer computing the ray model by pooling the 2D laser rays
    // so that it would become an end to end control system independent of the
    // polygonal perception pipeline.

    GeometricModel localWorldPolygons = worldPolygons;
    localWorldPolygons -= pose();
    localWorldPolygons.clip(Box(config.raysLength, -config.raysLength, -config.raysLength, config.raysLength));

    GeometricModel localWorldDynamicObstacles;
    localWorldDynamicObstacles.setObstacles(worldDynamicObstacles);
    localWorldDynamicObstacles -= pose();
    Vec2 base(1,0);
    base.normalize(config.raysLength);
    rays.clear();
    for (int i = 0; i < config.raysNumber; i++)
    {
        Vec2 ray1 = base.rotated(-config.raysAngleRange + i*2*config.raysAngleRange/(config.raysNumber-1));
        Vec2 ray2 = localWorldPolygons.rayIntersection(Vec2(), ray1);
        Vec2 ray3 = localWorldDynamicObstacles.rayIntersection(Vec2(), ray2);
        rays << ray3.norm();
    }
}

// Simulates a 2D lidar sensor.
// After calling this function, the laserSensor will be loaded with data.
void UnicycleAgent::simulateLaserSensor()
{
    GeometricModel localWorldPolygons = worldPolygons;
    localWorldPolygons -= pose();
    localWorldPolygons.clip(Box(config.laserLength, -config.laserLength, -config.laserLength, config.laserLength));

    GeometricModel localWorldDynamicObstacles;
    localWorldDynamicObstacles.setObstacles(worldDynamicObstacles);
    localWorldDynamicObstacles -= pose();
    Vec2 base(1,0);
    base.normalize(config.laserLength);
    rays.clear();
    Vector<Vec2> laserPoints;
    for (int i = 0; i < config.laserNumber; i++)
    {
        Vec2 ray1 = base.rotated(-config.laserAngleRange + i*2*config.laserAngleRange/(config.laserNumber-1));
        Vec2 ray2 = localWorldPolygons.rayIntersection(Vec2(), ray1);
        Vec2 ray3 = localWorldDynamicObstacles.rayIntersection(Vec2(), ray2);
        laserPoints << ray3;
    }

    laserSensor.writePointBuffer(laserPoints);
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
    // Bots are drawn differently.
    if (isBot())
    {
        drawBot(painter);
        return;
    }

    if (command.showWorldVisibilityGraph)
        worldMap.drawVisibilityGraph(painter); // This is used for global path searches.

    // The main target.
    if (command.showTargets && mainTarget.norm() > EPSILON)
    {
        double size = 0.125; // determines the size of the cross
        painter->save();
        painter->translate(mainTarget);
        painter->scale(size, size);
        painter->rotate(45);
        painter->setPen(drawUtil.pen);
        painter->setBrush(drawUtil.brushRed);
        painter->setOpacity(0.8);
        painter->drawPolygon(drawUtil.getCrossPolygon());
        painter->setOpacity(1.0);
        painter->setBrush(drawUtil.brush);
        painter->drawEllipse(QPointF(), 0.1, 0.1);
        painter->restore();
    }

    // Everything hereafter is drawn in local coordinates.
    painter->save();
    painter->translate(pos());
    painter->rotate(orientation()*RAD_TO_DEG);

    // The sensed grid.
    if (command.showSensedGrid)
    {
        sensedGrid.draw(painter, drawUtil.brushOrange);
        sensedGrid.drawBorder(painter);
    }

    // The Dijkstra map.
    if (command.showDijkstraMap)
    {
        sensedGrid.drawDijkstraMap(painter);
    }

    // The local geometric map.
    if (command.showSensedPolygons)
    {
        localMap.draw(painter, drawUtil.penThick, drawUtil.brushRed, drawUtil.brushOrange);
        sensedGrid.drawBorder(painter);
    }

    // The Visibility Polygon.
    if (command.showVisibilityPolygon)
    {
        visibilityPolygon.draw(painter, drawUtil.pen, drawUtil.brushGreen);
    }

    // The ray model.
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
                // Colliding rays in red.
                painter->setBrush(drawUtil.brushRed);
                painter->setPen(drawUtil.penRedThin);
            }
            else
            {
                // Full length rays in black.
                painter->setBrush(drawUtil.brush);
                painter->setPen(drawUtil.penThin);
            }
            painter->drawLine(QPointF(), ray);
            painter->drawEllipse(ray, 0.03, 0.03);
        }
        painter->restore();
    }

    // The rule base visualization (similar to ray model).
    if (trajectoryPlanningMethod == command.RuleBase)
    {
        ruleBase.draw(painter);
    }

    // The simulated lidar.
    if (command.showLidar > 0)
        laserSensor.draw(painter);


    // The body.
    if (command.showBody)
    {
        Polygon body = *this;
        body -= pose();
        if (isBot())
            body.draw(painter, drawUtil.penThick, drawUtil.brushRed, 1.0);
        else
            body.draw(painter, drawUtil.penThick, drawUtil.brushYellow, 1.0);
    }

    // The pos dot.
    painter->save();
    painter->setPen(drawUtil.pen);
    painter->setBrush(drawUtil.brush);
    painter->scale(0.03, 0.03);
    painter->drawEllipse(QPointF(0, 0), 1, 1);
    painter->restore();

    // Label showing the agent id.
    if (config.debugLevel > 3)
    {
        painter->save();
        painter->setPen(drawUtil.pen);
        painter->scale(-0.04, 0.04);
        painter->drawText(QPointF(), QString::number(getAgentId()));
        painter->restore();
    }

    // The velocity vector.
    Vec2 vv = 0.5*v*Vec2(1,0);
    if (!vv.isNull())
    {
        painter->save();
        painter->setPen(drawUtil.penThick);
        painter->drawPath(drawUtil.getArrow(Vec2(), vv));
        painter->restore();
    }

    // The emergency brake reflex.
    if (ebActive && trajectoryPlanningMethod != command.STAA)
    {
        // Impact point.
        painter->save();
        painter->setPen(drawUtil.penRed);
        painter->setBrush(drawUtil.brushRed);
        painter->translate(impactPoint);
        painter->scale(0.05, 0.05);
        painter->drawEllipse(QPointF(0, 0), 1, 1);
        painter->restore();

        // Break impulse vector.
        if (!impactPoint.isNull())
        {
            painter->save();
            painter->setPen(drawUtil.penRedThicker);
            painter->drawPath(drawUtil.getArrow(impactPoint, Vec2()));
            painter->restore();
        }
    }

    // The stuckness impulse.
    if (isStuck && !stucknessImpulse.isNull())
    {
        painter->save();
        painter->setPen(drawUtil.penGreenThick);
        painter->drawPath(drawUtil.getArrow(Vec2(), stucknessImpulse));
        painter->restore();
    }

    // The intermediate target as a blue cross.
    if (command.showTargets && intermediateTarget.pos().norm() > EPSILON)
    {
        painter->save();
        painter->translate(intermediateTarget.pos());
        painter->scale(0.08, 0.08); // determines the size of the cross
        painter->rotate(45);
        painter->setPen(drawUtil.pen);
        painter->setBrush(drawUtil.brushBlue);
        painter->setOpacity(0.8);
        painter->drawPolygon(drawUtil.getCrossPolygon());
        painter->setOpacity(1.0);
        painter->setBrush(drawUtil.brush);
        painter->drawEllipse(QPointF(), 0.1, 0.1);
        painter->restore();
    }

    // The carrot as a small orange cross.
    if (command.showTargets && carrot.pos().norm() > EPSILON
        && (command.trajectoryPlanningMethod == command.DWA
            || command.trajectoryPlanningMethod == command.PD
            || command.trajectoryPlanningMethod == command.RuleBase))
    {
        painter->save();
        painter->translate(carrot.pos());
        painter->scale(0.06, 0.06); // determines the size of the cross
        painter->rotate(45);
        painter->setPen(drawUtil.pen);
        painter->setBrush(drawUtil.brushOrange);
        painter->setOpacity(0.8);
        painter->drawPolygon(drawUtil.getCrossPolygon());
        painter->setOpacity(1.0);
        painter->setBrush(drawUtil.brush);
        painter->drawEllipse(QPointF(), 0.1, 0.1);
        painter->restore();
    }

    // The joystick carrot as a small green cross.
    if (command.showTargets && joystickCarrot.norm() > EPSILON && command.joystick)
    {
        painter->save();
        painter->translate(joystickCarrot);
        painter->scale(0.06, 0.06); // determines the size of the cross
        painter->rotate(45);
        painter->setPen(drawUtil.pen);
        painter->setBrush(drawUtil.brushGreen);
        painter->setOpacity(0.8);
        painter->drawPolygon(drawUtil.getCrossPolygon());
        painter->setOpacity(1.0);
        painter->setBrush(drawUtil.brush);
        painter->drawEllipse(QPointF(), 0.1, 0.1);
        painter->restore();
    }

    // The paths. World, dynamic and static.
    if (command.showWorldPath)
    {
        // Draw the world path.
        if (worldPathSuccess)
        {
            painter->save();
            painter->setPen(drawUtil.penBlueThick);
            painter->setBrush(drawUtil.brushBlue);
            painter->setOpacity(0.2);
            for (int i = 1; i < worldPath.size(); i++)
                painter->drawLine(QLineF(worldPath[i].x, worldPath[i].y, worldPath[i-1].x, worldPath[i-1].y));
            painter->restore();
        }

        // Draw the dynamic path.
        if (dynamicPathSuccess)
        {
            painter->save();
            painter->setPen(drawUtil.penRedThick);
            painter->setBrush(drawUtil.brushRed);
            painter->setOpacity(0.2);
            for (int i = 1; i < dynamicPath.size(); i++)
                painter->drawLine(QLineF(dynamicPath[i].x, dynamicPath[i].y, dynamicPath[i-1].x, dynamicPath[i-1].y));
            painter->restore();
        }

        // Draw the static path.
        if (staticPathSuccess)
        {
            painter->save();
            painter->setPen(drawUtil.penGreenThick);
            painter->setBrush(drawUtil.brushGreen);
            painter->setOpacity(0.2);
            for (int i = 1; i < staticPath.size(); i++)
                painter->drawLine(QLineF(staticPath[i].x, staticPath[i].y, staticPath[i-1].x, staticPath[i-1].y));
            painter->restore();
        }
    }

    // Local Visibility graph.
    if (command.showLocalVisibilityGraph)
        localMap.drawVisibilityGraph(painter);
        //shortTermAbortingAStar.drawVisibilityGraph(painter); // This is used for local path searches in the heuristic of AA*.

    // Controller visualization.
    if (trajectoryPlanningMethod == command.DWA)
    {
        unicycleDWA.draw(painter);
    }
    if (trajectoryPlanningMethod == command.STAA)
    {
        shortTermAbortingAStar.draw(painter);
    }

    painter->restore();
}

// QPainter drawing code for bots.
void UnicycleAgent::drawBot(QPainter *painter) const
{
    // Everything hereafter is drawn in local coordinates.
    painter->save();
    painter->translate(pos());
    painter->rotate(orientation()*RAD_TO_DEG);

    // The body.
    if (command.showBody)
    {
        Polygon body = *this;
        body -= pose();
        body.draw(painter, drawUtil.penThick, drawUtil.brushRed, 1.0);
    }

    // The pos dot.
    painter->save();
    painter->setPen(drawUtil.pen);
    painter->setBrush(drawUtil.brush);
    painter->scale(0.03, 0.03);
    painter->drawEllipse(QPointF(0, 0), 1, 1);
    painter->restore();

    // Label showing the agent id.
    if (config.debugLevel > 3)
    {
        painter->save();
        painter->setPen(drawUtil.pen);
        painter->scale(-0.04, 0.04);
        painter->drawText(QPointF(), QString::number(getAgentId()));
        painter->restore();
    }

    painter->restore();
    return;
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
        dbg << o.getName() << "pos:" << o.pos() << "angle:" << o.orientation() << "vel:" << o.vel() << "acc:" << o.acc();
    else
        dbg << o.getName() << " pos: " << o.pos() << " angle: " << o.orientation() << " vel: " << o.vel() << " acc: " << o.acc();
    return dbg;
}

QDebug operator<<(QDebug dbg, const UnicycleAgent* o)
{
    if (dbg.autoInsertSpaces())
        dbg << o->getName() << "pos:" << o->pos() << "angle:" << o->orientation() << "vel:" << o->vel() << "acc:" << o->acc();
    else
        dbg << o->getName() << " pos: " << o->pos() << " angle: " << o->orientation() << " vel: " << o->vel() << " acc: " << o->acc();
    return dbg;
}

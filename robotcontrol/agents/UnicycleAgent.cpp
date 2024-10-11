#include "UnicycleAgent.h"
#include "board/Config.h"
#include "board/Command.h"
#include "board/State.h"
#include "lib/kfi/VelocityProfile.h"
#include "lib/kfi/BangBang2D.h"
#include "lib/kfi/Bezier2D.h"
#include "lib/util/Statistics.h"
#include "lib/util/StopWatch.h"
#include "lib/util/DrawUtil.h"

// The UnicycleAgent is the controller that drives the unicycle agents in the simulation.
// It behaves like (is a) UnicycleObstacle and has an integrated sense()-act() loop.

UnicycleAgent::UnicycleAgent() : UnicycleObstacle()
{
    agentId = 0;

    stuckTimer = 0;
    stuckDetectionCounter = 0;
    ebActive = false;
    ebActivationCounter = 0;
    isStuck = false;

    safetyActivationTime = 0;
    safetyActive = false;

    inCollision = false;
    collided = false;

    worldPathSuccess = true;
    trajectorySuccess = true;
    staticPathSuccess = true;
    dynamicPathSuccess = true;
    trajectorySuccess = true;
    atTarget = false;
    targetNavGoalId = 0;

    trajectoryPlanningMethod = command.trajectoryPlanningMethod;
    trajectoryType = command.trajectoryType;
    predictionType = command.predictionType;
    heuristicType = command.heuristic;

    score = 0;
    collisions = 0;
    closes = 0;
    stucks = 0;
    milage = 0;
    pathTime = 0;
    trajectoryTime = 0;
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

    // In the hull polygon for collision checks.
    // The hull polygon is inside the inherited UnicycleObstacle
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
    // The initial pose comes out of the simulation.
    setPos(initialPos);
    setOrientation(0);

    // Init the main target to lie on the agent.
    mainTarget = pose();
    atTarget = true;

    // Init path and trajectory controllers.
    shortTermAbortingAStar.init();

    ruleBase.load("data/rulebase.dat");
    qDebug() << ruleBase.size() << "rules loaded";

    // The costmap and the occupancy grid are rectangular local grid maps around the robot.
    // The area and resolution of the grid is initialized based on config parameters.
    costmap.setDim(2);
    costmap.setN(Vec2u(config.gridHeight/config.gridCellSize+1, config.gridWidth/config.gridCellSize+1));
    costmap.setMin(Vec2(-config.gridHeight/2+config.gridOffset, -config.gridWidth/2));
    costmap.setMax(Vec2(config.gridHeight/2+config.gridOffset, config.gridWidth/2));
    costmap.init();
    occupanyGrid = costmap;
}

// Takes in the world nav goals.
void UnicycleAgent::setWorldNavGoals(const Vector<Vec2> &goals)
{
    navGoals = goals;
    targetNavGoalId = 0;

    // Generate 200 targets randomly chosen from the available nav goals.
    navGoalQueue.clear();
    for (uint i = 0; i < 200; i++)
    {
        uint idx = Statistics::randomInt(0, navGoals.size()-1);
        while (!navGoalQueue.empty() && idx == navGoalQueue.last())
            idx = Statistics::randomInt(0, navGoals.size()-1);
        navGoalQueue << idx;
    }
}

// Takes in the world map as input. This function provides cheat access to a world
// map constructed from simulation. This is what we consider to be the actual map
// the robot has built for itself.
void UnicycleAgent::setWorldMap(const GeometricModel &pols)
{
    worldMap = pols;
    worldMap.prune(pos());
}

// Sets the world polygons. This is a little cheat that gives the agent
// access to the actual polygons that make up the world. They are used
// for the computation of the sensor simulation and the ray model.
void UnicycleAgent::setWorldPolygons(const GeometricModel &pols)
{
    worldPolygons = pols;
}

// Takes in the world unicycles input. I am not sure what this is still doing here.
void UnicycleAgent::setWorldUnicycleObstacles(const Vector<UnicycleObstacle> &obst)
{
    worldDynamicObstacles = obst;
}

// Sets the main target for the agent. This overrides the automatic nav goal queue.
// Queue execution continues after the manually set target has been reached.
void UnicycleAgent::setMainTarget(const Pose2D &p)
{
    mainTarget = p;
    mainTarget.z = fpicut(mainTarget.z);
    atTarget = false;
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
    if (frequency == 5)
    {
        timeStep = 0.2;
        shortTermAbortingAStar.timeLimit = 180;
    }
    else if (frequency == 10)
    {
        timeStep = 0.1;
        shortTermAbortingAStar.timeLimit = 84;
    }
    else if (frequency == 20)
    {
        timeStep = 0.05;
        shortTermAbortingAStar.timeLimit = 36;
    }
    else // frequency = 30
    {
        timeStep = 0.03;
        shortTermAbortingAStar.timeLimit = 19;
    }
}

// In the sense method, a world representation is computed that can be
// used for localization and motion planning.
void UnicycleAgent::sense()
{
    //####################
    //# LASER PROCESSING #
    //####################
    StopWatch sw;
    sw.start();

    // Compute the 2D lidar sensor simulation and the ray model.
    // Both are computed by casting rays from the robot and finding the
    // first intersection point, if any, with the world polygons and the
    // dynamic world obstacles. The ray model is a low res model of laser
    // rays used as input for the rule base controller.
    simulateLaserSensor();
    computeRayModel();

    // From the laser sensor, we gain the visiblity polygon.
    // The visibility polygon is used for building the polygonal map and
    // for clearing free space.
    visibilityPolygon = laserSensor.extractVisibilityPolygon();

    state.laserTime = sw.elapsedTimeMs();



    //########
    //# SLAM #
    //########

    // We already have a GeometricModel of the world map in worldMap.
    // It comes in through the setWorldMap() function which the World
    // calls once after it has built a new map and then the world map
    // remains constant until the map is switched by the user. Even
    // though the world map is computed from the simulated environment,
    // it is fairly realistic and close to what a robot using a 2D lidar
    // would produce when building a map with LineSlam. The world map
    // is kept in world coordinates and used for path planning.

    // Geometric slam. Track the pose of the robot using the laser data and build the world map.
//    sw.start();
//    if (command.slamEnabled)
//        worldMap.slam(pose(), laserSensor.extractLines(), visibilityPolygon, laserSensor.extractSensedPolygons());
//    state.slamTime = sw.elapsedTimeMs();



    //#############
    //# LOCAL MAP #
    //#############
    sw.start();

    // We use the simulated lidar to compute the occupancy grid as the robot would do.
    // The occupancy grid is a local 8m x 8m occupancy grid. It is nearly centered around
    // the robot but is also pushed forward a bit so that the robot would see more to the
    // front than to the back. The occupancy grid is dilated a little bit in order to
    // connect single cells to contiguous regions.
    occupanyGrid.computeOccupancyGrid(laserSensor.readPointBuffer());
    occupanyGrid.dilate(config.gridSensedDilationRadius);

    // The costmap is computed by blurring the occupancy grid.
    //occupanyGrid.clearPolygon(visibilityPolygon); // This is to combat too much dilation.
    costmap = occupanyGrid;
    costmap.dilate(config.gridBlurRadius);
    costmap.blur(config.gridBlurRadius);
    costmap.max(occupanyGrid); // Make sure occupied cells stay occupied.

    // The local geometric map is computed by clipping the wold map with an 8x8 box.
    // This simplicity is possible because the world map is accurately updated in every
    // frame and contains all obstacles seen by the sensors. To perform the clipping,
    // we transform the local map to world coordinates, then clip, and transform the
    // result back to local coordinates.
    localMap = worldMap;
    localMap.clipConvex(costmap.boundingBox() + pose());
    localMap -= pose();

    // Now add UnicycleObstacle models to the local map taken from the unicycle agents in the world.
    // This is a simulation cheat as we are not yet able to detect and separate moving obstacles.
    // Only the moving obstacles inside the area of the local map are added. The unicycle obstacles
    // have an extended hull polygon that is used for path planning.
    for (uint i = 0; i < worldDynamicObstacles.size(); i++)
    {
        UnicycleObstacle uo = worldDynamicObstacles[i];
        uo -= pose(); // Transform from world to local coordinates.
        //if (visibilityPolygon.intersects(uo.pos()))
        if (costmap.contains(uo.pos()))
        {
            if (predictionType == command.Holonomic)
                uo.setVel(uo.v,0);
            else if (predictionType == command.None)
                uo.setVel(0,0);
            uo.setAcc(0,0);
            localMap.addObstacle(uo); // local but untransformed
        }
    }

    localMap.setBounds(costmap.boundingBox());
    localMap.renumber();
    localMap.autoPredict(); // Predict the future states of the agents.
    localMap.transform();
}

// Compute an action. This method results in the acceleration (a,b) of the agent being set.
void UnicycleAgent::act()
{
    //##############
    //# HIGH LAYER #
    //##############
    // High level planning. Where to put the main target?
    // The main target is the (x,y, theta) pose in the map in world coordinates.
    // The main target planner could be some sort of a robotic fleet orchestration
    // system that cooperatively plans the tasks of multiple robots and commands them
    // to navigate to specific points of intereset (POIs) and execute manipulation or
    // pick and place tasks at the POIs. For now, we only have a simple mock-up of
    // this planner and provide a random sequence of POIs for each agent.

    //qDebug() << "pose:" << pose() << "main target:" << mainTarget << "norm:" << (mainTarget-pose()).norm() << "dist:" << mainTarget.dist(pose()) << "max:" << (mainTarget-pose()).norm();

    // If the target has been reached, pick a new random target.
    // The metric that decides whether a target has been reached is quite crucial for accuracy.
    if (mainTarget.distxy(pose()) < config.agentTargetReachedDistance)
    {
        score++;
        targetNavGoalId = (targetNavGoalId + 1) % navGoalQueue.size();
        mainTarget = navGoals[navGoalQueue[targetNavGoalId]];
        //qDebug() << "agent" << getAgentId() << "updated main target" << mainTarget << dropOffPointQueue[targetDropOffId-1];
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
    // starting point or the target might end up inside an obstacle, where no path can be found. The path
    // computation is made robust to such disturbances by moving the start and the target into free space. 

    StopWatch sw;
    sw.start();

    // 1. Compute the world path and determine the intermediate target.
    worldPathSuccess = worldMap.computeStaticPath(pos(), mainTarget.pos());
    worldPath = worldMap.getPath()-pose();
    if (worldPathSuccess)
    {
        // Determine the intermediate target.
        // The intermediate target is the intersection of the world path and the boundary
        // of the sensed grid. The intermediate target is expressed in local coordinates.
        Box box = costmap.boundingBox();
        box.grow(-0.1); // Exclude the exact boundary to avoid problems.
        intermediateTarget = box.intersection(worldPath.getVertices());

        if (box.intersects(mainTarget.pos() - pose()))
            intermediateTarget.setHeading(mainTarget.heading() - orientation());
    }
    else
    {
        // It's a big issue if we cannot find the world path. Probably there is no way to the target at all.
        // We can keep going for a short while with the path and the intermediate target we had last.

        qDebug() << state.frameId << "World path computation failed from" << pos() << "to:" << mainTarget;
        //state.stop = true;
        worldMap.computeStaticPath(pos(), mainTarget.pos(), 50);
    }

    state.pathLength = worldPath.length();

    // 2. Compute the local static path in case it is needed as a fallback.
    staticPathSuccess = localMap.computeStaticPath(Vec2(), intermediateTarget.pos());
    staticPath = localMap.getPath();

//    if (isFirstAgent() && !staticPathSuccess)
//        qDebug() << state.frameId << "Static path computation failed to:" << intermediateTarget;

    // 3. Now compute the "dynamic" path up to the intermediate target using the sensed polygons and
    // the moving obstacles that are seen in the local map. Sometimes, moving obstacles cover the
    // target. Such obstacles are erased from the model before path computation. Also, moving obstacles
    // can block narrow passages and then no dynamic path can be found at all.
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

    // 4. Carrot extraction.
    // Determine the used path to extract the carrot from.
    if (dynamicPathSuccess && command.useDynamicPath)
        usedPath = dynamicPath;
    else if (staticPathSuccess)
        usedPath = staticPath;
    else if (worldPathSuccess)
        usedPath = worldPath;

    carrot.setNull();
    double dt = config.UPD_carrotOffset;
    if (trajectoryPlanningMethod == command.DWA)
        dt = config.DWA_carrotOffset;
    VelocityProfile vp;
    Unicycle u;
    u.setVel(vel());
    carrot = vp.getWaypoint(u, usedPath.getVertices(), dt).pos();
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



    // Safety zone reflex.
    // The safety zone in front of the robot is dynamically computed depending on the velocity.
    // If any obstacle is detected within the safety box, an emergency stop is triggered that
    // stops the robot for three seconds.
    if (command.safetyZoneReflex)
    {
        if (safetyActive)
        {
            //qDebug() << safetyActivationTime << stopWatch.programTime();
            if (stopWatch.programTime() - safetyActivationTime > 3)
                safetyActive = false;
            setTxVel(0, 0);
            return;
        }

        if (v > 0.3)
        {
            Polygon sz = getSafetyPolygon(v);
            if (!localMap.polygonCollisionCheck(sz).isEmpty())
            {
                safetyActive = true;
                safetyActivationTime = stopWatch.programTime();
                setTxVel(0, 0);
                return;
            }
        }
    }


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
        stucknessImpulse = carrot;
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




    //####################
    //# CONTROLLER LAYER #
    //####################
    // Action planning. High rate dynamic trajectory planning with bounded computation time.
    // The controller layer computes and sets the acceleration of the agent using one of the
    // inbuilt controllers. (Aborting A*, DWA, Rule Base,PD controller).

    // Joystick and keyboard control override.
    joystickCarrot = carrot;
    if (command.joystick && isFirstAgent())
    {
        ruleBase.query(rays, carrot);
        joystickCarrot = Vec2(-command.ay, command.ax) + pos() - pose();
        setAcc(pdControlTo(joystickCarrot));
    }
    else if (command.keyboard && isFirstAgent())
    {
        setAcc(command.ax, command.ay);

        // Emergency brake reflex.
        // The emergency break reflex activates when the agent is on a collision course.
        // When the brake reflex is active, the reflex stops the robot by commanding zero velocity
        // for the duration of one second.
        if (command.emergencyBrakeReflex)
        {
            Polygon uni = predicted(config.ebPredictionTime) - pose();
            uni.transform();
            uni.grow(config.staaPolygonGrowth);
            //qDebug() << localMap.polygonCollisionCheck(uni).isEmpty() << uni;
            if (ebActive || !localMap.polygonCollisionCheck(uni).isEmpty())
            {
                // Activation
                if (!ebActive)
                {
                    ebActive = true;
                    ebActivationCounter = command.frequency; // activates for one second
                }

                // Deactivation
                if (ebActivationCounter == 0)
                    ebActive = false;

                ebActivationCounter--;
                setTxVel(0,0);
                return;
            }
        }
    }
    else if (trajectoryPlanningMethod == command.PD)
    {
        // Simple PD controller that steers towards the carrot.
        // It's a decent controller to follow a path, but it cannot
        // avoid obstacles and cannot fullfill the target orientation.
        // The PD controller can be used for acceleration (setAcc())
        // and for velocity control (setTxVel()) with different parameters.
        // I found that for velocity control only the p parameters are needed.

        StopWatch sw;
        sw.start();

        setAcc(pdControlTo(carrot));

        trajectoryTime = sw.elapsedTimeMs();
        if (isFirstAgent())
            state.trajectoryTime = sw.elapsedTimeMs();
    }
    else if (trajectoryPlanningMethod == command.Bezier)
    {
        // This is a Bezier curve-based controller.
        // It is a little more powerful than the PD controller as it can
        // attain a target orientation.

        StopWatch sw;
        sw.start();

        Pose2D localTarget = mainTarget - pose();
        Bezier2D bez;
        Vec2 v1(1,0);
        v1.rotate(localTarget.heading());
        bez.setP1(localTarget.pos()-config.bezierStretch*localTarget.pos().norm()*v1);
        bez.addKeyframe(0, 0, 0, 0, 0);
        bez.addKeyframe(0, localTarget.x, 0, localTarget.y, 0);
        carrot = bez.evaluateAt(config.bezierDt).pos();

        Vec2 acc = pdControlTo(carrot);
        setAcc(acc);

        trajectoryTime = sw.elapsedTimeMs();
        if (isFirstAgent())
            state.trajectoryTime = sw.elapsedTimeMs();
    }
    else if (trajectoryPlanningMethod == command.Reel)
    {
        // Reel controller for docking.

        StopWatch sw;
        sw.start();

        Pose2D localTarget = mainTarget - pose();
        Vec2 v1(1,0);
        v1.rotate(localTarget.heading());
        Vec2 reel = -localTarget.pos().projectedOnVector(-v1);
        reel.normalize(max(reel.norm()-config.reelCarrotOffset, 0.0));
        carrot = localTarget.pos()+reel;
        setAcc(pdControlTo(carrot));

        trajectoryTime = sw.elapsedTimeMs();
        if (isFirstAgent())
            state.trajectoryTime = sw.elapsedTimeMs();
    }
    else if (trajectoryPlanningMethod == command.DWA)
    {
        // This is a Dynamic Window Approach (DWA) for unicycles.

        StopWatch sw;
        sw.start();

        Unicycle localUni;
        localUni.setVel(vel());

        // Dynamic Window Approach based on nonholonomic trajectory types:
        // arc, B0 spline, and Fresnel integral.
        //unicycleDWA.setDebug(config.debugLevel);
        unicycleDWA.setTrajectoryType(trajectoryType);
        unicycleDWA.setGeometricModel(localMap);
        unicycleDWA.setGridModel(costmap);
        unicycleDWA.setStart(localUni);
        unicycleDWA.setCarrot(carrot);
        Vec2 acc = unicycleDWA.search();
        setAcc(acc);

        trajectoryTime = sw.elapsedTimeMs();
        if (isFirstAgent())
            state.trajectoryTime = sw.elapsedTimeMs();
    }
    else if (trajectoryPlanningMethod == command.STAA)
    {
        // The Short Term Aborting A* controller is the most powerful
        // controller we have.

        StopWatch sw;
        sw.start();

        // Construct the start state including the current velocity.
        Unicycle localUni;
        localUni.setVel(vel());

        // Note: we assume zero velocity for the target state.

        shortTermAbortingAStar.setGeometricModel(localMap);
        shortTermAbortingAStar.setGridModel(costmap);
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
        Vec2 ccarrot = ruleBase.query(rays, carrot);
        Vec2 acc = pdControlTo(ccarrot);
        setAcc(acc);
    }
    else if (trajectoryPlanningMethod == command.SpeedControl)
    {
        // Speed control: trying to avoid the safety zone reflex activation.
        // The idea of this controller is that it determines the closest point in the geometric
        // map and if the closest point lies within the safety zone, it reduces the commanded
        // velocity such that the closest point no longer lies in the box. The commanded velocity
        // comes out of pd control. This is not a very successful controller as it leads to
        // oscillations and sometimes still makes a mistake around corners and triggers the safety
        // zone reflex.

        setAcc(pdControlTo(carrot));

        // Defintition of the safety polygon:
        // double x = max(0.0, vel * 2.6/1.5);
        // double y = vel <= 0.5 ? 0.5 : (0.5 + 0.5/0.6 * (vel-0.5));

        Polygon st = getSafetyPolygon(v);
        Vec2 cp = localMap.getClosestPoint(Vec2());
        double d = localMap.distance(Vec2());
        double vp = (d < 0.5) ? 0.5 : (6.0/5.0)*(d-(0.5/6.0));
        vp = (d < 0.5) ? 0.5 : d*(1.5/2.6);
        vp *= 0.75;
        //qDebug() << "closest point:" << cp << cp.norm() << d << "dx:" << st.getVertices().first() << "v:" << v << "vp:" << vp;
        if (v > vp)
        {
            //qDebug() << "reducing vel from" << v << vp;
            setTxVel(vp, w);
            return;
        }
    }
}

void UnicycleAgent::learn()
{
    if (command.learn)
    {
        ruleBase.addRule(rays, carrot, joystickCarrot);
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


// Clears a polygonal region from the map as described by pol.
void UnicycleAgent::clearMap(const Polygon &pol)
{
    //worldMap.clearPolygon(pol);
}

// Fills a polygonal region in the map as described by pol.
void UnicycleAgent::fillMap(const Polygon &pol)
{
    //worldMap.fillPolygon(pol);
}

// Simulates a 2D lidar sensor.
// After calling this function, the laserSensor will be loaded with data.
void UnicycleAgent::simulateLaserSensor()
{
    LaserInfo laserInfo;
    laserInfo.angleMax = config.simLaserAngleRange;
    laserInfo.angleMin = -config.simLaserAngleRange;
    laserInfo.angleIncrement = 2.0*config.simLaserAngleRange/config.simLaserRays;
    laserInfo.rangeMax = config.simLaserLength;
    laserInfo.rays = config.simLaserRays;
    laserSensor.setLaserInfo(laserInfo);


    GeometricModel localWorldPolygons = worldPolygons;
    localWorldPolygons -= pose();
    localWorldPolygons.clip(Box(config.simLaserLength, -config.simLaserLength, -config.simLaserLength, config.simLaserLength));

    GeometricModel localWorldDynamicObstacles;
    localWorldDynamicObstacles.setObstacles(worldDynamicObstacles);
    localWorldDynamicObstacles -= pose();
    Vec2 base(1,0);
    base.normalize(config.simLaserLength);
    rays.clear();
    Vector<Vec2> laserPoints;
    Vector<double> laserRanges;
    for (int i = 0; i < config.simLaserRays; i++)
    {
        Vec2 ray1 = base.rotated(-config.simLaserAngleRange + i*2*config.simLaserAngleRange/(config.simLaserRays-1));
        Vec2 ray2 = localWorldPolygons.rayIntersection(Vec2(), ray1);
        Vec2 ray3 = localWorldDynamicObstacles.rayIntersection(Vec2(), ray2);
        laserPoints << ray3;
        laserRanges << ray3.length();
    }

    //laserSensor.writePointBuffer(laserPoints);
    laserSensor.writeRangeBuffer(laserRanges);
}

// This is a simple PD controller that computes accelerations towards the target q.
// The PD controller can be used for acceleration (setAcc()) and for velocity control
// (setTxVel()) with different parameters. I found that for velocity control only the
// p parameters are needed.
Vec2 UnicycleAgent::pdControlTo(const Vec2 &q) const
{
    Vec2 forward(1,0);
    double projectedTransErr = q * forward;
    double rotErr = forward.angleTo(q);
    double a = config.UPD_Plin * projectedTransErr + config.UPD_Dlin * v;
    double b = config.UPD_Prot * rotErr + config.UPD_Drot * w;
    if (q.norm() < 0.01) // Dead band.
        return Vec2();
    return Vec2(a,b);
}

// Takes in a normalized force vector and computes the accelerations as if
// the car were pushed by the force.
Vec2 UnicycleAgent::forceReaction(const Vec2 &q) const
{
    Vec2 forward(1,0);
    Vec2 sideward(0,1);
    double projectedTransErr = q * forward;
    double projectedRotErr = q * sideward;
    double a = config.UPD_Plin * projectedTransErr + config.UPD_Dlin * v;
    double b = (config.UPD_Prot * projectedRotErr + config.UPD_Drot * w);
    return Vec2(a,b);
}

// Extracts a carrot dist meters along the path. The orientation of the returned
// Pose2D aligns with the path at the intersected point. If dist is larger than
// the entire length of the path, the last point of the path is returned.
Pose2D UnicycleAgent::extractCarrot(const Vector<Vec2> &path, double dist) const
{
    if (path.isEmpty())
        return Pose2D();

    uint i = 0;
    while (i < path.size()-2 && dist > (path[i+1]-path[i]).norm())
    {
        dist -= (path[i+1]-path[i]).norm();
        i++;
    }

    Pose2D carrot;
    carrot.setPos(path[i] + min((dist/(path[i+1]-path[i]).norm()), 1.0) * (path[i+1]-path[i]));
    carrot.setHeading((path[i+1]-path[i]).fangle());
    return carrot;
}

// Computes the safety polygon given the current velocity vel
Polygon UnicycleAgent::getSafetyPolygon(double vel) const
{
    // The parameters the determine the size of the safety polygon depending
    // on the input velocity approximate the safety polygon of G4T4.
    Polygon sz;
    if (vel > 0.16)
    {
        double x = max(0.0, vel * 2.6/1.5);
        double y = vel <= 0.5 ? 0.5 : (0.5 + 0.5/0.6 * (vel-0.5));
        sz << Vec2(x, y) << Vec2(0,y) << Vec2(0, -y) << Vec2(x, -y);
    }
    return sz;
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

    // World visibility graph.
    if (command.showWorldVisibilityGraph)
        worldMap.drawVisibilityGraph(painter); // This is used for global path searches.

    // The main target.
    if (command.showTargets && mainTarget.norm() > EPSILON)
        drawUtil.drawNoseCircle(painter, mainTarget, drawUtil.penThick, drawUtil.brushRed, 0.25);

    // Everything hereafter is drawn in local coordinates!
    painter->save();
    painter->translate(pos());
    painter->rotate(orientation()*RAD_TO_DEG);

    // Safety Zone
    if (command.showSafetyZone)
    {
        Polygon sz = getSafetyPolygon(v);
        if (!localMap.polygonCollisionCheck(sz).isEmpty())
            sz.draw(painter, drawUtil.pen, drawUtil.brushRed, 0.3);
        else
            sz.draw(painter, drawUtil.pen, drawUtil.brushRed, 0.05);
    }

    // Speed Controller
    if (command.trajectoryPlanningMethod == command.SpeedControl)
    {
        Vec2 cp = localMap.getClosestPoint(Vec2());
        drawUtil.drawLine(painter, Vec2(), cp, drawUtil.penRedThin);
    }

    // Bezier controller
    if (command.trajectoryPlanningMethod == command.Bezier)
    {
        Pose2D localTarget = mainTarget - pose();
        Bezier2D bez;
        Vec2 v1(1,0);
        v1.rotate(localTarget.heading());
        bez.setP1(localTarget.pos()-config.bezierStretch*localTarget.pos().norm()*v1);
        bez.addKeyframe(0, 0, 0, 0, 0);
        bez.addKeyframe(0, localTarget.x, 0, localTarget.y, 0);
        bez.draw(painter, drawUtil.penGreenDashed);
    }

    // Reel controller
    if (command.trajectoryPlanningMethod == command.Reel)
    {
        Pose2D localTarget = mainTarget - pose();
        Vec2 v1(1,0);
        v1.rotate(localTarget.heading());
        Vec2 reel = -localTarget.pos().projectedOnVector(-v1);
        reel.normalize(max(reel.norm()-config.reelCarrotOffset, 0.0));

        painter->save();
        painter->setPen(drawUtil.penGrayThinDashed);
        if (!reel.isNull())
            painter->drawLine(localTarget.pos(), localTarget.pos()+reel);
        painter->restore();
    }


    // The sensed grid.
    if (command.showCostmap)
    {
        costmap.draw(painter, drawUtil.brushOrange);
        costmap.drawBorder(painter);
    }

    // The Dijkstra map.
    if (command.showDijkstraMap)
        costmap.drawDijkstraMap(painter);

    // The local map.
    if (command.showLocalMap)
    {
        localMap.draw(painter, drawUtil.penThick, drawUtil.brushRed, drawUtil.brushOrange);
        costmap.drawBorder(painter);
    }

    // The Visibility Polygon.
    if (command.showVisibilityPolygon)
        visibilityPolygon.draw(painter, drawUtil.pen, drawUtil.brushLightGreen, 0.4);

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

    // The rule base controller visualization (similar to ray model).
    if (trajectoryPlanningMethod == command.RuleBase)
        ruleBase.draw(painter);

    // The simulated lidar.
    if (command.showLaser > 0)
        laserSensor.draw(painter);

    // The body.
    if (command.showBody)
    {
        Polygon body = *this;
        body -= pose();
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
        drawUtil.drawArrow(painter, Vec2(), vv, drawUtil.penThick);

    // The emergency brake reflex.
    if (command.emergencyBrakeReflex)
    {
        Polygon uni = predicted(config.ebPredictionTime) - pose();
        uni.transform();
        uni.grow(config.staaPolygonGrowth);
        //qDebug() << localMap.polygonCollisionCheck(uni, true);
        if (!localMap.polygonCollisionCheck(uni).isEmpty())
            uni.draw(painter, drawUtil.penRedThick, Qt::NoBrush, 0.5);
        else
            uni.draw(painter, drawUtil.penBlueThick, Qt::NoBrush, 0.5);
    }

    // The stuckness impulse.
    if (isStuck && !stucknessImpulse.isNull())
        drawUtil.drawArrow(painter, Vec2(), stucknessImpulse, drawUtil.penGreenThick);

    // The intermediate target as a blue cross.
    if (command.showTargets && intermediateTarget.pos().norm() > EPSILON)
        drawUtil.drawNoseCircle(painter, intermediateTarget, drawUtil.penThick, drawUtil.brushBlue, 0.2);

    // The carrot as a small orange cross.
    if (command.showTargets && carrot.norm() > EPSILON)
        drawUtil.drawCross(painter, carrot, drawUtil.penThick, drawUtil.brushOrange, 0.05);

    // The joystick carrot as a small green cross.
    if (command.showTargets && joystickCarrot.norm() > EPSILON && command.joystick)
        drawUtil.drawCross(painter, joystickCarrot, drawUtil.pen, drawUtil.brushGreen, 0.06);

    // The paths. World, dynamic and static.
    if (command.showPaths)
    {
        // Draw the world path.
        if (worldPathSuccess)
            worldPath.draw(painter, drawUtil.penBlueThick, 0.2);

        // Draw the dynamic path.
        if (dynamicPathSuccess)
            dynamicPath.draw(painter, drawUtil.penRedThick, 0.2);

        // Draw the static path.
        if (staticPathSuccess)
            staticPath.draw(painter, drawUtil.penGreenThick, 0.2);
    }

    // Local Visibility graph.
    if (command.showLocalVisibilityGraph)
        localMap.drawVisibilityGraph(painter);
        //shortTermAbortingAStar.drawVisibilityGraph(painter); // This is used for local path searches in the heuristic of AA*.

    // The DWA controller
    if (trajectoryPlanningMethod == command.DWA)
        unicycleDWA.draw(painter);

    // The STAA controller
    if (trajectoryPlanningMethod == command.STAA)
        shortTermAbortingAStar.draw(painter);

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

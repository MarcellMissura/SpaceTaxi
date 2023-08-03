#include "UnicycleRobot.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"
#include "blackboard/State.h"
#include "robotcontrol/controller/VelocityProfile.h"
#include "lib/util/GLlib.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/Statistics.h"

UnicycleRobot::UnicycleRobot() : UnicycleObstacle()
{
    inited = false;

    stuckTimer = 0;
    stuckDetectionCounter = 0;
    ebActive = false;
    ebA = 0;
    isStuck = false;

    worldPathSuccess = true;
    trajectorySuccess = true;
    atTarget = false;

    targetDropOffId = 0;
    score = 0;
    milage = 0;
    collisions = 0;
    stucks = 0;
    closes = 0;
    worldPathSuccess = true;
    trajectorySuccess = true;
    pathTime = 0;
    trajectoryTime = 0;
}

// Initialization after construction.
void UnicycleRobot::init()
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

    // Init the main target to lie on the agent.
    mainTarget = pose();
    atTarget = true;

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

    if (command.trajectoryPlanningMethod == command.RuleBase)
    {
        ruleBase.load("data/rulebase.dat");
        qDebug() << ruleBase.size() << "rules loaded";
    }
}

// Resets the state of the robot. Clears the map and zeros the pose.
void UnicycleRobot::reset()
{
    setPose(Pose2D());
    geometricMap.clear();
}

// Takes in one frame of sensor input.
void UnicycleRobot::setInput(const LaserSensor &laserInput, const Pose2D &odomInput)
{
    laserSensor.writePointBuffer(laserInput.readPointBuffer());
    odomPose = odomInput;
}

// Sets the main target for the robot.
void UnicycleRobot::setMainTarget(const Pose2D &pose)
{
    mainTarget = pose;
    mainTarget.z = fpicut(mainTarget.z);
    atTarget = false;
}

// Sets the initial pose. The initial pose is in world coordinates
// and defines the starting point in the map.
void UnicycleRobot::setInitialPose(const Pose2D &p)
{
    this->initialPose = p;
    setPose(p);
}

// In the sense method, a world represenation is computed that can be
// used for planning and localization.
void UnicycleRobot::sense()
{
    Pose2D localOdomPose = odomPose; // unmutexed until problem seen

    if (!inited)
    {
        inited = true;
        initialPose = pose(); // We start in any random place.
        initialOdomPose = localOdomPose; // The odom pose is given relative to some other random place.
    }

    // Convert the odom pose into the frame of the initial pose.
    localOdomPose = (localOdomPose - initialOdomPose) + initialPose;

    if (command.keepPoseHistory)
    {
        odomHistory << localOdomPose; // Only for visualization.
        poseHistory << pose(); // Only for visualization.
    }
    if (command.showOdometry)
        odomHistory << localOdomPose; // Only for visualization.

    // Use the odometry sensor as a prior estimate of the current pose.
    // If the odometry sensor is *not* used, the dead reckoning through the predict() function
    // computes a pose prior based on the control input.
    if (command.useOdomAsPrior)
        setPose(localOdomPose);
    else
        predict(1.0/command.frequency); // dead reckoning

    //qDebug() << odomPose << localOdomPose << localOdomPose
    //qDebug() << "pose:" << pose() << pose() - initialPose << "odomPose:" << localOdomPose << odomPose << "diff:" << odomDiff;

    // Laser data smoothing.
    laserSensor.filter();

    // Retrieve the most recent visibility polygon from the laser sensor.
    visibilityPolygon = laserSensor.getVisibilityPolygon();

    // Geometric slam. Track the pose of the robot using laser data and build the map.
    if (command.slamEnabled)
    {
        Pose2D currentPose = geometricMap.slam(pose(), laserSensor.extractLines(), visibilityPolygon);
        setPose(currentPose);
    }


    // Sense the local map.
    // The local map is a 8m x 8m rectangle. It is nearly centered around the robot
    // but is also pushed forward a bit so that the robot would see more to the front
    // than to the back.

    // We compute the clipped world map, the world map clipped to the local map.
    clippedWorldMap = geometricMap.getGeometricModel();
    clippedWorldMap -= pose(); // local
    clippedWorldMap.clip(sensedGrid.boundingBox());

    // We use the laser sensor to compute the occupancy grid.
    // The occupancy grid is a local 8m x 8m occupancy grid. It is nearly centered around
    // the robot but is also pushed forward a bit so that the robot would see more to the
    // front than to the back.
    occupanyGrid.computeOccupancyGrid(laserSensor.readPointBuffer());

    // The occupancy grid is dilated a little bit in order to
    // connect single cells to contiguous regions.
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
    localMap.setBounds(sensedGrid.boundingBox());
    localMap.renumber();
    localMap.autoPredict(); // Predict the future states of the agents.
    localMap.transform();

    computeRayModel(); // Used for rule base control.
}

// Compute an action. This method results in the acceleration (a,b) of the agent being set.
void UnicycleRobot::act()
{
    // noop for experiments
    //setAcc(0,0);
    //setVel(0,0);
    //return;

    //##############
    //# HIGH LAYER #
    //##############
    // High level planning. Where to put the main target?
    // The main target is the Pose2D of a POI in the map in world coordinates.
    // This is waiting for the order management to start working.

    // Target has been reached.
    if (!atTarget && (mainTarget.distxy(pose()) < config.agentTargetReachedDistance))
    {
        //qDebug() << "Main target reached.";
        atTarget = true;
        setAcc(0,0);
        setVel(0,0);
        return;
    }

    // When we are at target, do nothing until a new target has been set.
    if (atTarget && !(command.keyboard || command.joystick))
        return;


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
    worldPathSuccess = worldMap.computeStaticPath(pos(), mainTarget.pos());
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

        qDebug() << state.frameId << "World path computation failed from" << pos() << "to:" << mainTarget;
        state.stop = 1;
        worldMap.computeStaticPath(pos(), mainTarget.pos(), 50);
    }

    // 2. Compute the static path in case it is needed as a fallback.
    staticPathSuccess = localMap.computeStaticPath(Vec2(), intermediateTarget.pos());
    staticPath = localMap.getPath();

//    if (!staticPathSuccess)
//        qDebug() << state.frameId << "Static path computation failed to:" << intermediateTarget;

    // 3. Now compute the "dynamic" path up to the intermediate target using the sensed polygons and
    // the moving obstacles that are seen in the local map. Sometimes, moving obstacles cover the
    // target. Such obstacles are erased from the model before path computation. Also, moving obstacles can
    // block narrow passages and then no dynamic path can be found at all.
    localMap.resetSearch();
    dynamicPathSuccess = localMap.computeDynamicPath(Vec2(), intermediateTarget.pos());
    dynamicPath = localMap.getPath();

//    if (!dynamicPathSuccess)
//        qDebug() << state.frameId << "Dynamic path computation failed to:" << intermediateTarget;


    // 4. Carrot extraction.

    // If the dynamic path is good, take the carrot from it.
    if (dynamicPathSuccess && command.useDynamicPath)
    {
        double dt = config.DWA_carrotOffset;
        if (command.trajectoryPlanningMethod == command.PD)
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
        if (command.trajectoryPlanningMethod == command.PD)
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
        if (command.trajectoryPlanningMethod == command.PD)
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
        if (command.trajectoryPlanningMethod == command.STAA)
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
        if (command.stucknessReflex && (command.trajectoryPlanningMethod != command.STAA))
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
            ebA = -v*command.frequency;
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

    joystickCarrot = carrot.pos();
    if (command.joystick)
    {
        ruleBase.query(rays, carrot.pos());
        joystickCarrot = Vec2(command.v, command.w);
        Vec2 acc = (Vec2(command.v, command.w) - vel()) * command.frequency;
        if (ebActive)
            acc.x = min(acc.x, ebA); // Apply the emergency break.
        setAcc(acc);
    }
    else if (command.keyboard)
    {
        Vec2 acc = (Vec2(command.v, command.w) - vel()) * command.frequency;
        if (ebActive)
            acc.x = min(acc.x, ebA); // Apply the emergency break.
        setAcc(acc);
    }
    else if (command.trajectoryPlanningMethod == command.PD)
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
        state.trajectoryTime = sw.elapsedTimeMs();
    }
    else if (command.trajectoryPlanningMethod == command.DWA)
    {
        // This is a Unicycle DWA used to control a nonholonomic agent.

        StopWatch sw;
        sw.start();

        Unicycle localUni;
        localUni.setVel(vel());

        // Dynamic Window Approach based on nonholonomic trajectory types:
        // arc, B0 spline, and Fresnel integrals.
        //unicycleDWA.setDebug(config.debugLevel);
        unicycleDWA.setTrajectoryType(command.trajectoryType);
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
        state.trajectoryTime = sw.elapsedTimeMs();
    }
    else if (command.trajectoryPlanningMethod == command.STAA)
    {
        StopWatch sw;
        sw.start();

        Unicycle localUni;
        localUni.setVel(vel());

        // Set the time limit for STAA.
        if (command.frequency == 10)
            shortTermAbortingAStar.setTimeLimit(84);
        else if (command.frequency == 20)
            shortTermAbortingAStar.setTimeLimit(36);
        else
            shortTermAbortingAStar.setTimeLimit(19);

        shortTermAbortingAStar.setGeometricModel(localMap);
        shortTermAbortingAStar.setGridModel(sensedGrid);
        shortTermAbortingAStar.setStartState(localUni);
        shortTermAbortingAStar.setTargetState(intermediateTarget);
        if (command.stucknessReflex)
            shortTermAbortingAStar.setStuck(stuckTimer > 0);
        trajectorySuccess = shortTermAbortingAStar.aStarSearch();
        Vec2 acc = shortTermAbortingAStar.getAction();
        setAcc(acc);

        state.aasExpansions = shortTermAbortingAStar.expansions;
        state.aasOpen = shortTermAbortingAStar.opened;
        state.aasClosed = shortTermAbortingAStar.closed; // This actually means how many expansions coming from closed cells were ignored.
        state.aasCollided = shortTermAbortingAStar.collided;
        state.aasProcessed = shortTermAbortingAStar.processed;
        state.aasDried = shortTermAbortingAStar.dried;
        state.aasFinished = shortTermAbortingAStar.finished;
        state.aasDepth = shortTermAbortingAStar.depth;
        state.aasScore = shortTermAbortingAStar.score;

        // Measure execution time.
        trajectoryTime = sw.elapsedTimeMs();
        state.trajectoryTime = sw.elapsedTimeMs();
    }
    else if (command.trajectoryPlanningMethod == command.RuleBase)
    {
        Vec2 ccarrot = ruleBase.query(rays, carrot.pos());
        Vec2 acc = pdControlTo(ccarrot);
        if (ebActive)
            acc.x = min(acc.x, ebA); // Apply the emergency break.
        setAcc(acc);
        //qDebug() << ccarrot << ccarrot.pos().norm2() << pdControlTo(ccarrot.pos()) << acc();
    }
}

void UnicycleRobot::learn()
{
    if (command.learn)
    {
        ruleBase.addRule(rays, carrot.pos(), joystickCarrot);
        ruleBase.save("data/rulebase.dat");
    }
}

// Executes one robot control step.
void UnicycleRobot::step()
{
    stopWatch.start();
    setInput(state.laserInput, state.odomInput);
    sense();
    state.senseTime = stopWatch.elapsedTimeMs();
    stopWatch.start();
    act();
    state.actTime = stopWatch.elapsedTimeMs();
    //predict(1.0/command.frequency); // dead reckoning
    if (atTarget && !(command.keyboard || command.joystick))
        setVel(0,0);
    state.executionTime = state.senseTime+state.actTime;
    //qDebug() << state.senseTime << state.actTime;
}

// Computes the ray sensing model.
void UnicycleRobot::computeRayModel()
{
    // Actually I would prefer computing the ray model by pooling the 2D laser rays
    // so that it would become an end to end control system independent of the
    // polygonal perception pipeline.

    Vec2 base(1,0);
    base.normalize(config.raysLength);
    rays.clear();
    for (int i = 0; i < config.raysNumber; i++)
    {
        Vec2 ray1 = base.rotated(-config.raysAngleRange + i*2*config.raysAngleRange/(config.raysNumber-1));
        Vec2 ray2 = localMap.rayIntersection(Vec2(), ray1);
        rays << ray2.norm();
    }
}

// This is a simple PD controller that computes accelerations towards the target q.
// It also includes a dead band around q where the control output is zero.
Vec2 UnicycleRobot::pdControlTo(const Pose2D &p) const
{
    Vec2 forward(1,0);
    double projectedTransErr = p.pos() * forward;
    double targetAngleError = forward.angleTo(p.pos());
    double targetOrientationError = p.heading();
    double targetOrientationFactor = max(config.UPD_targetOrientationThreshold - fabs(projectedTransErr), 0.0) / config.UPD_targetOrientationThreshold;
    double rotErr = targetOrientationFactor*targetOrientationError + (1.0-targetOrientationFactor)*targetAngleError;
    double a = config.UPD_Kp_lin * projectedTransErr + config.UPD_Kd_lin * v;
    double b = config.UPD_Kp_rot * rotErr + config.UPD_Kd_rot * w;
    if (p.norm() < 0.025) // Dead band.
        return Vec2();
    return Vec2(a,b);
}

// Takes in a normalized force vector and computes the accelerations.
Vec2 UnicycleRobot::forceReaction(const Vec2 &q) const
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

// Returns the target velocity to be sent to the real robot.
Vec2 UnicycleRobot::getTxVel()
{
    return vel();
}

// Draws the raw laser points.
void UnicycleRobot::drawLaserPoints() const
{
    if (command.showLaser == 0)
        return;
    laserSensor.draw();
}

// Draws the odometry history.
void UnicycleRobot::drawOdometry() const
{
    if (!command.showOdometry)
        return;
    for (uint i = 0; i < odomHistory.size(); i++)
        GLlib::drawNoseCircle(odomHistory[i], drawUtil.ivory, 0.5*config.agentRadius);
    //GLlib::drawNoseCircle(odomDiff+poseHistory.last(), drawUtil.transparent, 0.6*config.agentRadius);
}

void UnicycleRobot::drawPose() const
{
    if (command.showPose == 0)
        return;

    if (command.showPose == 2)
    {
        for (uint i = 0; i < poseHistory.size(); i++)
            GLlib::drawNoseCircle(poseHistory[i], Qt::green, 0.5*config.agentRadius);
    }

    if (atTarget)
        GLlib::drawNoseCircle(pose(), Qt::green, config.agentRadius);
    else
        GLlib::drawNoseCircle(pose(), Qt::blue, config.agentRadius);
}

// The body.
void UnicycleRobot::drawBody() const
{
    if (!command.showBody)
        return;

    glColor4f(drawUtil.brushGreen.color().redF(), drawUtil.brushGreen.color().greenF(), drawUtil.brushGreen.color().blueF(), 0.9);
    GLlib::drawCylinder(config.agentRadius, config.agentHeight); // 90 cm high
}

// Draws the local grid model.
void UnicycleRobot::drawGridModel() const
{
    if (command.showSensedGrid)
    {
        sensedGrid.draw(drawUtil.brushRed);
        glTranslated(0,0,0.001);
        sensedGrid.drawBorder();
    }
}

// Draws the local geometric model.
void UnicycleRobot::drawGeometricModel() const
{
    if (command.showSensedPolygons)
    {
        localMap.draw(drawUtil.penThick, drawUtil.brushOrange, 0.5);
        glTranslated(0,0,0.001);
        sensedGrid.drawBorder();
    }
}

// Draws the ray model.
void UnicycleRobot::drawRayModel() const
{
    if (!command.showRayModel)
        return;
    glColor4f(0.8, 0.5, 0.1, 0.8);
    glPointSize(5);
    glBegin(GL_POINTS);
    for (uint i = 0; i < rayModel.size(); i++)
        glVertex3d(rayModel[i].x, rayModel[i].y, 0.01);
    glEnd();
}

// Map visualization.
void UnicycleRobot::drawWorldMap() const
{
    geometricMap.draw();
}

// Draws a visualization of the visibility polygon extracted from the laser sensor.
void UnicycleRobot::drawVisibilityPolygon() const
{
    if (!command.showVisibilityPolygon)
        return;

    glPushMatrix();
    visibilityPolygon.draw(drawUtil.penThick, drawUtil.brushGreen, 0.2);
    //qDebug() << visibilityPolygon;

    glTranslated(0,0,0.01);

    Polygon boundedVisPol = visibilityPolygon;
    ListIterator<Line> ei = boundedVisPol.edgeIterator();
    while (ei.hasNext())
    {
        Line& edge = ei.next();
        Vec2& p1 = edge.p1();
        Vec2& p2 = edge.p2();
        if (p1.length() > config.slamVisibilityPolygonBound)
            p1.normalize(config.slamVisibilityPolygonBound);
        if (p2.length() > config.slamVisibilityPolygonBound)
            p2.normalize(config.slamVisibilityPolygonBound);
    }
    Vector<Polygon> pols = boundedVisPol.offseted(-config.slamVisibilityPolygonShrinking, config.laserDouglasPeuckerEpsilon);
    for (uint i = 0; i < pols.size(); i++)
        pols[i].draw(drawUtil.penThick, drawUtil.brushGreen, 0.2);

    glPopMatrix();
}

// Draws the static and the dynamic world path.
void UnicycleRobot::drawWorldPaths() const
{
    if (!command.showPaths)
        return;

    // Draw the static world path.
    if (worldPathSuccess)
    {
        glColor3f(0.0,0.0,0.8);
        for (uint i = 1; i < worldPath.size(); i++)
            GLlib::drawLine(worldPath[i], worldPath[i-1], 0.01);
    }

    // Draw the dynamic path.
    if (dynamicPathSuccess)
    {
        glColor3f(0.8,0.0,0.0);
        for (uint i = 1; i < dynamicPath.size(); i++)
            GLlib::drawLine(dynamicPath[i], dynamicPath[i-1], 0.01);
    }
}

// Draws the main target.
void UnicycleRobot::drawMainTarget() const
{
    if (command.showTargets && (mainTarget-pose().pos()).norm() > EPSILON)
    {
        GLlib::drawNoseCircle(mainTarget, Qt::red, config.agentRadius);
        glColor3f(0, 0, 0);
        GLlib::drawFilledCircle(0.01); // global
    }
}

// Draws the local targets. Carrot and intermediate.
void UnicycleRobot::drawTargets() const
{
    if (!command.showTargets)
        return;

    // The intermediate target.
    if (intermediateTarget.pos().norm() > EPSILON)
        GLlib::drawNoseCircle(intermediateTarget, Qt::blue, 0.5*config.agentRadius); // local

    // The carrot.
    if (carrot.pos().norm() > EPSILON && (command.trajectoryPlanningMethod == command.DWA
            || command.trajectoryPlanningMethod == command.PD))
    {
        glPushMatrix();
        glTranslated(carrot.x, carrot.y, 0);
        glColor3f(0, 1.0, 0);
        GLlib::drawCross(0.08);
        glColor3f(0, 0, 0);
        GLlib::drawFilledCircle(0.01); // local
        glPopMatrix();
    }
}

// Draws a visualization of the motion plan.
void UnicycleRobot::drawMotionPlan() const
{
    if (!command.showMotionPlan)
        return;
    if (command.trajectoryPlanningMethod == command.DWA)
        unicycleDWA.draw(); // local, thick
    else if (command.trajectoryPlanningMethod == command.STAA)
        shortTermAbortingAStar.draw(); // local, thick
}

// Draws the local visibility graph.
void UnicycleRobot::drawVisibilityGraph() const
{
    if (command.showLocalVisibilityGraph)
        localMap.drawVisibilityGraph();
}

// Main OpenGL draw function.
void UnicycleRobot::draw()
{
    drawWorldMap();

    glTranslated(0, 0, 0.001);

    drawMainTarget(); // global

    glTranslated(0, 0, 0.001);

    drawOdometry(); // global
    drawPose(); // global

    // Now transform into the local coordinate frame.
    // Everything hereafter is drawn in local coordinates.
    glPushMatrix();
    glMultMatrixd(pose().getMatrix());

    // Draw the local models. Grid model, geometric model, visibility graph.
    drawGridModel(); // local, 0.003 thick
    drawGeometricModel(); // local, 0.001 thick
    drawVisibilityPolygon(); // local

    glTranslated(0, 0, 0.001);

    drawRayModel(); // local
    drawVisibilityGraph(); // the local one

    // The carrot and the intermediate target (local).
    drawTargets();

    // The static and dynamic world paths (local)
    drawWorldPaths();

    // Motion controller visualization (local).
    drawMotionPlan();

    drawLaserPoints(); // local

    drawBody(); // local

    glPopMatrix(); // end local coordinate frame
}

// QPainter drawing code.
void UnicycleRobot::draw(QPainter *painter) const
{
    geometricMap.draw(painter);

    if (command.showWorldVisibilityGraph)
        worldMap.drawVisibilityGraph(painter); // This is used for global path searches.

    // The main target.
    if (command.showTargets && mainTarget.norm() > EPSILON)
        drawUtil.drawCross(painter, mainTarget.pos(), drawUtil.pen, drawUtil.brushRed, 0.125);

    // The odometry history.
    if (command.showOdometry)
    {
        painter->save();
        painter->translate(initialPose.pos());
        painter->rotate(initialPose.heading()*RAD_TO_DEG);
        for (uint i = 0; i < odomHistory.size(); i++)
            drawUtil.drawNoseCircle(painter, odomHistory[i], drawUtil.penThin, drawUtil.brushIvory, 0.5*config.agentRadius);
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
    if (command.trajectoryPlanningMethod == command.RuleBase)
    {
        ruleBase.draw(painter);
    }

    // The laser sensor.
    if (command.showLaser > 0)
        laserSensor.draw(painter);

    // The body.
    if (command.showBody)
    {
        Polygon body = *this;
        body -= pose();
        body.draw(painter, drawUtil.penThick, drawUtil.brushYellow);

        // The pos dot.
        painter->save();
        painter->setPen(drawUtil.pen);
        painter->setBrush(drawUtil.brush);
        painter->scale(0.02, 0.02);
        painter->drawEllipse(QPointF(0, 0), 1, 1);
        painter->restore();
    }

    // The velocity vector.
    Vec2 vv = 0.5*v*Vec2(1,0);
    if (!vv.isNull())
    {
        drawUtil.drawArrow(painter, Vec2(), vv, drawUtil.penThick);
    }

    // The emergency brake reflex.
    if (ebActive && command.trajectoryPlanningMethod != command.STAA)
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
            drawUtil.drawArrow(painter, impactPoint, Vec2(), drawUtil.penRedThicker);
    }

    // The stuckness impulse.
    if (isStuck && !stucknessImpulse.isNull())
        drawUtil.drawArrow(painter, Vec2(), stucknessImpulse, drawUtil.penGreenThick);

    // The intermediate target as a blue cross.
    if (command.showTargets && intermediateTarget.pos().norm() > EPSILON)
    {
        drawUtil.drawCross(painter, intermediateTarget.pos(), drawUtil.pen, drawUtil.brushBlue, 0.08);
    }

    // The carrot as a small orange cross.
    if (command.showTargets && carrot.pos().norm() > EPSILON
        && (command.trajectoryPlanningMethod == command.DWA
            || command.trajectoryPlanningMethod == command.PD
            || command.trajectoryPlanningMethod == command.RuleBase))
    {
        drawUtil.drawCross(painter, carrot.pos(), drawUtil.pen, drawUtil.brushOrange, 0.06);
    }

    // The joystick carrot as a small green cross.
    if (command.showTargets && joystickCarrot.norm() > EPSILON && command.joystick)
        drawUtil.drawCross(painter, joystickCarrot, drawUtil.pen, drawUtil.brushGreen, 0.06);

    // The paths. World, dynamic and static.
    if (command.showPaths)
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
    if (command.trajectoryPlanningMethod == command.DWA)
    {
        unicycleDWA.draw(painter);
    }
    if (command.trajectoryPlanningMethod == command.STAA)
    {
        shortTermAbortingAStar.draw(painter);
    }

    painter->restore();
}

// Saves the line map and the polygon map to a file.
void UnicycleRobot::saveMap() const
{
    QFile file("data/map.dat");
    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "Couldn't open map file for writing" << file.fileName();
        return;
    }

    QDataStream out(&file);
    out << geometricMap;
    file.close();
}

// Loads the line map and the polygon map from a file.
void UnicycleRobot::loadMap()
{
    QFile file("data/map.dat");
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "Couldn't open map file for reading" << file.fileName();
        return;
    }

    geometricMap.clear();
    QDataStream in(&file);
    in >> geometricMap;
    file.close();
}

QDebug operator<<(QDebug dbg, const UnicycleRobot &o)
{
    dbg << o.pose();
    return dbg;
}

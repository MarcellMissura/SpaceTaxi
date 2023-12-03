#include "UnicycleRobot.h"
#include "board/Config.h"
#include "board/Command.h"
#include "board/State.h"
#include "lib/kfi/VelocityProfile.h"
#include "lib/util/GLlib.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/Statistics.h"

UnicycleRobot::UnicycleRobot() : UnicycleObstacle()
{
    worldPathSuccess = true;
    staticPathSuccess = true;
    dynamicPathSuccess = true;
    trajectorySuccess = true;
    atTarget = false;
    targetDropOffId = 0;

    score = 0;
    collisions = 0;
    stucks = 0;
    milage = 0;
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

    // The costmap and the occupancy grid are rectangular local grid maps around the robot.
    // The area and resolution of the grid is initialized based on config parameters.
    costmap.setDim(2);
    costmap.setN(Vec2u(config.gridHeight/config.gridCellSize+1, config.gridWidth/config.gridCellSize+1));
    costmap.setMin(Vec2(-config.gridHeight/2+config.gridOffset, -config.gridWidth/2));
    costmap.setMax(Vec2(config.gridHeight/2+config.gridOffset, config.gridWidth/2));
    costmap.init();
    occupanyGrid = costmap;
}

// Resets the state of the robot. Clears the map and zeros the pose.
void UnicycleRobot::reset()
{
    setPose(Pose2D());
    worldMap.clear();
}

// Takes in one frame of sensor input.
void UnicycleRobot::setInput(const LaserSensor &laserInput, const OdomSensor &odomSensorr)
{
    Vector<double> rb = laserInput.readRangeBuffer();
    rb.reverse(); // proant hack
    laserSensor.writeRangeBuffer(rb);
    odomSensor.writeOdomPose(odomSensorr.readOdomPose());
}

// Sets the main target for the robot.
void UnicycleRobot::setMainTarget(const Pose2D &pose)
{
    mainTarget = pose;
    mainTarget.z = fpicut(mainTarget.z);
    atTarget = false;
}

// Sets the pose of the robot assuming that it's a localized pose in the map.
// This allows the operator to tell the robot where it is by clicking on the map.
void UnicycleRobot::setInitialPose(const Pose2D &p)
{
    this->initialPose = p;
    setPose(p);
    worldMap.localizeAt(p);
}

// In the sense method, a world represenation is computed that can be
// used for localization and motion planning.
void UnicycleRobot::sense()
{
    //####################
    //# LASER PROCESSING #
    //####################

    // The raw laser data is filtered (temporal, spatial, speckle) and processed to a visibility
    // polygon, detected lines, and a detected docking frame.

    StopWatch sw;
    sw.start();

    // Retrieve the most recent visibility polygon from the laser sensor.
    visibilityPolygon = laserSensor.extractVisibilityPolygon();

    // Detect docking frames.
    //laserSensor.extractTriangleMarker();

    // Line extraction.
    laserSensor.extractLines();

    state.laserTime = sw.elapsedTimeMs();
    //qDebug() << state.frameId << "Laser time:" << state.laserTime;


    //#############
    //# LOCAL MAP #
    //#############
    sw.start();

    // The local map is a 8m x 8m rectangle. It is nearly centered around the robot
    // but is also pushed forward a bit so that the robot would see more to the front
    // than to the back. The local map contains a grid representation (costmap) and a
    // geometric representation (polygons) of the immediate surroundings of the robot.

    // We compute the occupancy grid by occupying cells that contain at least one laser
    // point. The occupancy grid is a local 8m x 8m grid. It is nearly centered around
    // the robot but is also pushed forward a bit so that the robot would see more to the
    // front than to the back. The occupancy grid is dilated in order to connect single
    // cells to contiguous regions.
    occupanyGrid.computeOccupancyGrid(laserSensor.readPointBuffer());
    occupanyGrid.dilate(config.gridSensedDilationRadius);

    // The costmap is computed by blurring the occupancy grid.
    //occupanyGrid.clearPolygon(visibilityPolygon); // This is to combat too much dilation.
    costmap = occupanyGrid;
    costmap.dilate(config.gridBlurRadius);
    costmap.blur(config.gridBlurRadius);
    costmap.max(occupanyGrid); // Make sure occupied cells stay occupied.

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
    localMap.setPolygons(occupanyGrid.extractPolygons()); // local and transformed
    localMap.dilate(config.gmPolygonDilation); // needed for slam
    localMap.simplify(config.gmDouglasPeuckerEpsilon);

    state.localMapTime = sw.elapsedTimeMs();

    //########
    //# SLAM #
    //########
    sw.start();

    // Use the odometry sensor as a prior estimate of the current pose.
    // If the odometry sensor is *not* used, the dead reckoning through the predict() function
    // computes a pose prior based on the control input.
    if (command.useOdometry)
        setPose(odomSensor.readOdomIncrement() + pose());
    else
        predict(1.0/command.frequency); // dead reckoning

    // Geometric slam. Track the pose of the robot using laser data and build the map.
    if (command.slamEnabled)
    {
        Pose2D currentPose = worldMap.slam(pose(), laserSensor.extractLines(), visibilityPolygon, laserSensor.extractSensedPolygons());
        setPose(currentPose);
    }

    poseHistory << pose(); // Keeping this only for visualization.

    state.slamTime = sw.elapsedTimeMs();
    //qDebug() << state.frameId << "Slam time:" << state.slamTime;



    //##################
    //# WORLD CLIPPING #
    //##################
    sw.start();

    // We compute the clipped world map and add it to the local map. The clipped world
    // map is the world map clipped to the local map. This is needed so that the local
    // planners can take obstacles into account even if they are currently not being seen.
    // The procedure transforms the entire map to local coordinates and then clips the
    // polygons with an axis aligned box. It would be faster to transform the box to world
    // coordinates instead and then clip and transform the result back to local coordinates.
    // The clipped world map is added to the local map. This is actually crucial, otherwise
    // we get silly paths leading only around the polygons we can see but through obstacles
    // that are in the world map. Adding the clipped world map requires good localization
    // and it is best to perform this step after slam.
    clippedWorldMap = worldMap.getGeometricModel();
    clippedWorldMap -= pose(); // This transforms the entire map to local coordinates.
    clippedWorldMap.clip(costmap.boundingBox());
    localMap += clippedWorldMap; // local
    localMap.setBounds(costmap.boundingBox());
    localMap.renumber(); // Required for local path planning.
    localMap.autoPredict(); // Predict the future states of the agents.
    localMap.transform();

    state.localMapTime += sw.elapsedTimeMs();
    //qDebug() << state.frameId << "Local map time:" << state.localMapTime;
}

// Compute an action. This method results in the acceleration (a,b) of the agent being set.
void UnicycleRobot::act()
{
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
    }

    // When we are at target, do nothing until a new target has been set.
    if (atTarget && !(command.keyboard || command.joystick))
    {
        setVel(0,0);
        return;
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

    // 1. Compute the world path and determine the intermediate target.
    worldPathSuccess = worldMap.computeStaticPath(pos(), mainTarget.pos());
    const Vector<Vec2>& pp = worldMap.getPath();
    worldPath.set(pp);
    //qDebug() << worldPath;

//    double pt = sw.elapsedTimeMs();
//    qDebug() << "Path time:" << pt;

    if (worldPathSuccess)
    {
        // Transform the path to local coordinates.
        worldPath -= pose();

        // Determine the intermediate target.
        // The intermediate target is the intersection of the static world path and the
        // boundary of the sensed grid. The intermediate target is expressed in local coordinates.
        Box box = costmap.boundingBox();
        box.grow(-0.1); // Exclude the exact boundary to avoid problems.
        intermediateTarget = box.intersection(worldPath.getVertices());
    }
    else
    {
        // It's a big issue if we cannot find the world path. Probably there is no way to the target at all.
        // We can keep going for a short while with the path and the intermediate target we had last.

        //qDebug() << state.frameId << "World path computation failed from" << pos() << "to:" << mainTarget;
        //state.stop = true;
        //worldMap.computeStaticPath(pos(), mainTarget.pos(), 50);
    }

    state.pathLength = worldPath.length();

    // 2. Compute the static path in case it is needed as a fallback.
    staticPathSuccess = localMap.computeStaticPath(Vec2(), intermediateTarget.pos());
    staticPath.set(localMap.getPath());

//    if (!staticPathSuccess)
//        qDebug() << state.frameId << "Static path computation failed to:" << intermediateTarget;

    // 3. Now compute the "dynamic" path up to the intermediate target using the sensed polygons and
    // the moving obstacles that are seen in the local map. Sometimes, moving obstacles cover the
    // target. Such obstacles are erased from the model before path computation. Also, moving obstacles can
    // block narrow passages and then no dynamic path can be found at all.
    localMap.resetSearch();
    dynamicPathSuccess = localMap.computeDynamicPath(Vec2(), intermediateTarget.pos());
    dynamicPath.set(localMap.getPath());

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
        Hpm2D h = vp.getWaypoint(u, dynamicPath.getVertices(), dt);
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
        Hpm2D h = vp.getWaypoint(u, staticPath.getVertices(), dt);
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
        Hpm2D h = vp.getWaypoint(u, worldPath.getVertices(), dt);
        carrot.setPos(h.pos());
        carrot.setHeading(h.heading());
    }

    //qDebug() << "extracted carrot:" << carrot << dynamicPathSuccess << staticPathSuccess << worldPathSuccess;

    pathTime = sw.elapsedTimeMs();
    state.pathTime = pathTime;


    //####################
    //# CONTROLLER LAYER #
    //####################
    // Action planning. High rate dynamic trajectory planning with bounded computation time.
    // The controller layer computes and sets the acceleration of the agent using Aborting A* or DWA or a PD controller.

    joystickCarrot = carrot.pos();
    if (command.joystick)
    {
        setVel(command.v, command.w);
    }
    else if (command.keyboard)
    {
        setVel(command.v, command.w);
        //qDebug() << "set vel:" << vel() << "acc:" << acc();
    }
    else if (command.trajectoryPlanningMethod == command.PD)
    {
        StopWatch sw;
        sw.start();

        setVel(pdControlTo(carrot.pos()));

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
        unicycleDWA.setGridModel(costmap);
        unicycleDWA.setStart(localUni);
        unicycleDWA.setCarrot(carrot.pos());
        Vec2 acc = unicycleDWA.search();
        setAcc(acc); // Acceleration control does not work in this project.

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
        shortTermAbortingAStar.setGridModel(costmap);
        shortTermAbortingAStar.setStartState(localUni);
        shortTermAbortingAStar.setTargetState(intermediateTarget);
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
}

// Clears a polygonal region from the map as described by pol.
void UnicycleRobot::clearMap(const Polygon &pol)
{
    worldMap.clearPolygon(pol);
}

// Fills a polygonal region in the map as described by pol.
void UnicycleRobot::fillMap(const Polygon &pol)
{
    worldMap.fillPolygon(pol);
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
    state.executionTime = state.senseTime+state.actTime;
    //qDebug() << state.senseTime << state.actTime;
}

// This is a simple PD controller that computes accelerations towards the target q.
// The PD controller can be used for acceleration (setAcc()) and for velocity control
// (setTxVel()) with different parameters. I found that for velocity control only the
// p parameters are needed.
Vec2 UnicycleRobot::pdControlTo(const Vec2 &q) const
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
    odomSensor.draw();
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
        costmap.draw(drawUtil.brushRed);
        glTranslated(0,0,0.001);
        costmap.drawBorder();
    }
}

// Draws the local geometric model.
void UnicycleRobot::drawGeometricModel() const
{
    if (command.showSensedPolygons)
    {
        localMap.draw(drawUtil.penThick, drawUtil.brushOrange, 0.5);
        glTranslated(0,0,0.001);
        costmap.drawBorder();
    }
}

// Map visualization.
void UnicycleRobot::drawWorldMap() const
{
    worldMap.draw();
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
    Vector<Polygon> pols = boundedVisPol.offseted(-config.gmPolygonDilation);
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
        worldPath.draw(drawUtil.penBlue, 0.2);
    }

    // Draw the dynamic path.
    if (dynamicPathSuccess)
        dynamicPath.draw(drawUtil.penRed);
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
    worldMap.draw(painter);

    if (command.showWorldVisibilityGraph)
        worldMap.drawVisibilityGraph(painter); // This is used for global path searches.

    // The main target.
    if (command.showTargets && mainTarget.norm() > EPSILON)
        drawUtil.drawCross(painter, mainTarget.pos(), drawUtil.pen, drawUtil.brushRed, 0.125);

    // The odometry history.
    if (command.showOdometry)
    {
        painter->save();
        //painter->translate(initialPose.pos());
        //painter->rotate(initialPose.heading()*RAD_TO_DEG);
        odomSensor.draw(painter);
        painter->restore();
    }

    // The pose history.
    if (command.showPose > 1)
        for (uint i = 0; i < poseHistory.size(); i++)
            drawUtil.drawNoseCircle(painter, poseHistory[i], drawUtil.penThin, drawUtil.brushYellow, 0.5*config.agentRadius);
    if (command.showPose > 0)
        drawUtil.drawNoseCircle(painter, pose(), drawUtil.penThin, drawUtil.brushYellow, 0.6*config.agentRadius);


    // Everything hereafter is drawn in local coordinates.
    painter->save();
    painter->translate(pos());
    painter->rotate(orientation()*RAD_TO_DEG);

    // The sensed grid.
    if (command.showSensedGrid)
    {
        costmap.draw(painter, drawUtil.brushOrange);
        costmap.drawBorder(painter);
    }

    // Sensed polygons.
    if (command.showSensedPolygons)
    {
        // The local geometric map.
        //localMap.draw(painter, drawUtil.penThick, drawUtil.brushRed, drawUtil.brushOrange);
        //costmap.drawBorder(painter);
        Vector<Polygon> dp = laserSensor.extractSensedPolygons();
        for (uint i = 0; i < dp.size(); i++)
        {
            dp[i].pruneOut(config.gmPolygonPruning);
            dp[i].draw(painter, drawUtil.pen, drawUtil.brushRed, 0.5);
        }
    }

    // The Visibility Polygon.
    if (command.showVisibilityPolygon)
    {
        visibilityPolygon.draw(painter, drawUtil.pen, drawUtil.brushLightGreen, 0.5);
        if (command.showLabels)
            visibilityPolygon.drawEdgeLabels(painter);

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
        Vector<Polygon> pols = boundedVisPol.offseted(-config.gmPolygonDilation);
        for (uint i = 0; i < pols.size(); i++)
        {
            pols[i].draw(painter, drawUtil.penDashed, drawUtil.brushLightGreen, 0.2);
            //pols[i].prune(config.gmPolygonPruning);
            //pols[i].draw(painter, drawUtil.penDashed, drawUtil.brushMagenta, 0.2);
        }

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
        drawUtil.drawArrow(painter, Vec2(), vv, drawUtil.penThick);

    // The stuckness impulse.
    if (isStuck && !stucknessImpulse.isNull())
        drawUtil.drawArrow(painter, Vec2(), stucknessImpulse, drawUtil.penGreenThick);

    // The intermediate target as a blue cross.
    if (command.showTargets && intermediateTarget.pos().norm() > EPSILON)
        drawUtil.drawCross(painter, intermediateTarget.pos(), drawUtil.pen, drawUtil.brushBlue, 0.08);

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
            worldPath.draw(painter, drawUtil.penBlueThick, 0.2);
        }

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

    // Controller visualization.
    if (command.trajectoryPlanningMethod == command.DWA)
        unicycleDWA.draw(painter);
    if (command.trajectoryPlanningMethod == command.STAA)
        shortTermAbortingAStar.draw(painter);

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
    out << worldMap;
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

    worldMap.clear();
    QDataStream in(&file);
    in >> worldMap;
    file.close();
}

QDebug operator<<(QDebug dbg, const UnicycleRobot &o)
{
    dbg << o.pose();
    return dbg;
}

#include "UnicycleRobot.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"
#include "blackboard/State.h"
#include "lib/util/GLlib.h"
#include "lib/util/ColorUtil.h"
#include "lib/util/RGBPixel.h"
#include "lib/util/Statistics.h"

QMutex UnicycleRobot::odomPoseMutex(QMutex::Recursive);
RGBDSensor UnicycleRobot::rgbdSensor;
GeometricModel UnicycleRobot::unifiedGeometricModel;
GeometricMap UnicycleRobot::geometricMap;

UnicycleRobot::UnicycleRobot() : UnicycleObstacle()
{
    rgbdUpdated = false;
    laserUpdated = false;
    staticWorldPathSuccess = true;
    trajectorySuccess = true;
    atTarget = false;
}

// Initialization after construction.
void UnicycleRobot::init()
{
    // Set up the polygon that describes the agent.
    setUnitOctogon();
    scale(config.agentRadius, config.agentRadius);

    // Init the main target to lie on the agent.
    mainTarget = pose();
    atTarget = true;

    // Init path and trajectory controllers.
    shortTermAbortingAStar.init();

    // The assumed first line of perception: a rectangular occupancy grid around the robot.
    // The area and resolution of the grid model is initialized based on config parameters.
    sensedGrid.setDim(2);
    sensedGrid.setN(Vec2u(config.gridHeight/config.gridCellSize+1, config.gridWidth/config.gridCellSize+1));
    sensedGrid.setMin(Vec2(-config.gridHeight/2+config.gridOffset, -config.gridWidth/2));
    sensedGrid.setMax(Vec2(config.gridHeight/2+config.gridOffset, config.gridWidth/2));
    sensedGrid.init();

    mapGrid = sensedGrid; // copy the structure
    mapAndSensedGrid = sensedGrid; // copy the structure
    dilatedMapAndSensedGrid = sensedGrid; // copy the structure

    // Initialize the sensor objects.
    rgbdSensor.init();
    laserSensor.init();
}

void UnicycleRobot::reset()
{
    setPose(Pose2D());
    geometricMap.clear();
}

// Is called by the robot interface when new laser data arrive.
void UnicycleRobot::writeLaserBuffer(const Vector<Vec2> &points)
{
    laserSensor.writePointBuffer(points);
    laserUpdated = true;
}

// Is called by the robot interface when a new point cloud arrives.
void UnicycleRobot::writePointBuffer(const Vector<Vec3> &points)
{
    rgbdSensor.writePointBuffer(points);
    rgbdUpdated = true;
}

// Is called by the robot interface when new rgb data arrive.
void UnicycleRobot::writeColorBuffer(const Vector<RGBPixel> &colors)
{
    rgbdSensor.writeColorBuffer(colors);
}

// Returns a reference to the point cloud buffer.
const Vector<Vec3> &UnicycleRobot::readPointBuffer() const
{
//    return Vector<Vec3>();
    return rgbdSensor.getPointBuffer();
}

// Returns a copy of the raw laser points.
Vector<Vec2> UnicycleRobot::readLaserBuffer() const
{
    Vector<Vec2> b;
    laserSensor.readPointBuffer(b);
    return b;
}

// Returns the target velocity to be sent to the real robot.
const Vec2& UnicycleRobot::getTxVel()
{
    txVel = predicted(config.rcIterationTime).vel(); // TODO: Might need to use the timeDiff.
    if (atTarget && !(command.keyboard || command.joystick))
        txVel.setNull();
    return txVel;
}

// Sets the sensed velocity from the robot.
void UnicycleRobot::setRxVel(double v, double w)
{
    rxVel.x = v;
    rxVel.y = w;
    setVel(rxVel);
}

// Is called by the robot interface to set the odometry pose.
void UnicycleRobot::setOdomPose(const Pose2D &p)
{
    QMutexLocker locker(&odomPoseMutex);
    odomPose = p;
}

Pose2D UnicycleRobot::getOdomPose() const
{
    QMutexLocker locker(&odomPoseMutex);
    return odomPose;
}

// Returns the camera info struct (width, height and such).
CameraInfo UnicycleRobot::getCameraInfo() const
{
//    return CameraInfo();
    return rgbdSensor.getCameraInfo();
}

// This is called by the robot interface to set the camera info.
void UnicycleRobot::setCameraInfo(const CameraInfo &info)
{
    rgbdSensor.setCameraInfo(info);
}

// Returns the laser info (angle range, max length and such).
const LaserInfo &UnicycleRobot::getLaserInfo() const
{
    return laserSensor.getLaserInfo();
}

// Called by the robot interface to set the laser info.
void UnicycleRobot::setLaserInfo(const LaserInfo &info)
{
    laserSensor.setLaserInfo(info);
}

void UnicycleRobot::setLaserToBasePose(const Pose2D &p)
{
    laserSensor.setLaserToBasePose(p);
}

// Sets the main target for the robot.
void UnicycleRobot::setTarget(const Pose2D &pose)
{
    mainTarget = pose;
    mainTarget.z = fpicut(mainTarget.z);
    atTarget = false;
}

// Is called by the robot interface to set the camera transform.
void UnicycleRobot::setCameraTransform(const Transform3D &T)
{
    rgbdSensor.setTransform(T);
}

// Return the transform of the rgbd sensor.
Transform3D UnicycleRobot::getCameraTransform() const
{
    return Transform3D();
    return pose().getMatrix() * rgbdSensor.getTransform();
}

// Returns a reference to the color buffer.
const Vector<RGBPixel> &UnicycleRobot::readColorBuffer() const
{
//    return Vector<RGBPixel>();
    return rgbdSensor.getColorBuffer();
}

// Sets the initial pose. The initial pose is in world coordinates
// and defines an offset that is added to laser localization.
void UnicycleRobot::setInitialPose(const Pose2D &p)
{
    this->initialPose = p;
    setPose(p);
    externalLocalizerPose = p;
}

// Receives a pose estimate from the external localizer such as AMCL in ROS.
void UnicycleRobot::setExternalLocalizerPose(const Pose2D &p)
{
    externalLocalizerPose = p;
    if (command.useExternalLocalizerPose)
        setPose(p);
}

// Returns true of the point p (given in robot cooordinates) lies inside the local grid.
bool UnicycleRobot::isPointInLocalGrid(const Vec2 &p) const
{
    return mapGrid.contains(p);
}

// Returns the index of the cell of the local grid that contains the point p.
// Out of bounds queries are mapped to the closest cell on the border.
Vec2u UnicycleRobot::getNodeIndex(const Vec2 &p) const
{
    return Vec2u(mapGrid.getNodeIndex(p));
}

// Returns the value of the local occupancy grid at point p.
double UnicycleRobot::getValueAt(const Vec2 &p) const
{
    return dilatedMapAndSensedGrid.getAt(p);
}

// In the sense method, world representation models are computed.
void UnicycleRobot::sense()
{
    StopWatch sw;
    sw.start();

    Pose2D localOdomPose = getOdomPose(); // This is mostly because of the mutex.

    if (initialPose.isNull())
    {
        initialPose = pose();
        initialOdomPose = localOdomPose;
        prevOdomPose = localOdomPose - initialOdomPose + initialPose; // Converts the odom pose into the frame of the initial pose.
    }

    // Update the pose of the agent based on the odometry sensor.
    localOdomPose = localOdomPose - initialOdomPose + initialPose; // Converts the odom pose into the frame of the initial pose.
    odomDiff = (localOdomPose - prevOdomPose);
    prevOdomPose = localOdomPose;

    if (command.keepPoseHistory)
    {
        odomHistory << localOdomPose; // Only for visualization.
        poseHistory << pose(); // Only for visualization.
    }

    if (command.useOdomAsPrior)
        setPose(odomDiff + pose());

    // Laser data smoothing.
    if (command.laserPointSmoothing)
        laserSensor.smoothPoints();

    // Retrieve the most recent visibility polygon from the laser sensor.
    visibilityPolygon = laserSensor.getVisibilityPolygon();

    // Geometric slam. Track the pose of the robot using laser data and build the map.
    if (laserUpdated)
    {
        Pose2D currentPose = geometricMap.slam(pose(), laserSensor.extractLines(), visibilityPolygon);
        setPose(currentPose);
    }

    //double time = sw.elapsedTimeMs();
    //qDebug() << state.frameId << "slam:" << time;

    // Sense the local occupancy map (the local grid).
    // The local grid is a local 8m x 8m occupancy grid, our assumed input. It is nearly centered
    // around the robot but is also pushed forward a bit so that the robot would see more to the
    // front than to the back. The world polygons are transformed into the robot frame and the
    // local grid is computed by drawing the polygons onto the grid like onto an image. The grid
    // is dilated and blurred after extraction to serve as a costmap.

    // Compute the mapGrid, a local grid of the map polygons.
    //mapGrid.computeOccupancyGrid((polygonMap-pose()).getPolygons(), 2);

    // Compute the sensed grid, either from the rgbd point cloud or from the laser data.
    if (laserUpdated)
    {
        sensedGrid.clear();
        sensedGrid.computeOccupancyGrid(visibilityPolygon, 2);
        //sensedGrid.computeOccupancyGrid(laserSensor.readPointBuffer());
    }
    else
    {
        //sensedGrid.computeOccupancyGrid(rgbdSensor.getTransformedPointBuffer(), config.floorHeight, config.ceilingHeight);
    }

    // Combine the map and sensed grids to a dilated and blurred "mapAndSensedGrid" to be used for motion planning.
    // This grid is heavily used as a lookup table for collision detection.
    mapAndSensedGrid = mapGrid + sensedGrid;
    dilatedMapAndSensedGrid = mapAndSensedGrid;
    dilatedMapAndSensedGrid.dilate(config.gridDilationRadius);

    // The mapAndSensedGrid is also used to extract a GeometricModel of a local set of polygons and also an expanded version
    // of it. It seems expensive to compute a grid from the world polygons and then convert back to polygons by contour
    // detection. But then again, the dilation of the grid is a really good expansion operator. Could this be done faster
    // and just as nice by using clipping and offsetting? The local polygons are used for path planning inside the Aborting A*.
    //mapAndSensedPolygons.setFromGrid(mapAndSensedGrid);
    expandedMapAndSensedPolygons.setFromGrid(dilatedMapAndSensedGrid);
    expandedMapAndSensedPolygons.renumber();

    // The blur of the map and sensed grid had to wait until after the polygon extraction.
    dilatedMapAndSensedGrid.blur(config.gridBlurRadius);

    // Combine a unified model that contains the expanded world polygons and the expanded local map and sensed polygons.
    // This unified model is used for global path planning. Dynamic polygons are also added to this model later.
    // The unified model is in local coordinates.
//    unifiedGeometricModel = polygonMap;
//    unifiedGeometricModel -= pose();
//    unifiedGeometricModel.transform();
    unifiedGeometricModel += expandedMapAndSensedPolygons;
    unifiedGeometricModel.renumber();

    // Compute the dynamic geometric model.
    dynamicPolygons.clear();
    expandedDynamicPolygons.clear();
    Vector<UnicycleObstacle> uo; // The dynamic obstacles will yet have to come from somewhere.
    for (uint i = 0; i < uo.size(); i++)
    {
        if (!command.predict)
            uo[i].setVel(0,0);
        expandedDynamicPolygons.addObstacle(uo[i]);
        dynamicPolygons.addObstacle(uo[i]);
    }
    dynamicPolygons.transform();
    expandedDynamicPolygons.grow(config.gmPolygonExpansionMargin);
    expandedDynamicPolygons.transform();

    //computeRayModel(); // Used for rule base control.
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
    // The main target is the (x,y) location of a drop-off point on the map
    // in world coordinates.

    // Target has been reached.
    //state.debug = (mainTarget-pose()).norm();
    if (!atTarget && (mainTarget-pose()).norm() < config.agentTargetReachedDistance)
    {
        //qDebug() << "Main target reached.";
        atTarget = true;
        setAcc(0,0);
        setVel(0,0);
        return;
    }

    if (atTarget && !(command.keyboard || command.joystick))
        return;


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

    StopWatch sw;
    sw.start();
    const Vector<Vec2>& pp = unifiedGeometricModel.computePath(Vec2(), (mainTarget-pose()).pos());
    staticWorldPathSuccess = !pp.isEmpty();
    if (staticWorldPathSuccess)
    {
        staticWorldPath = pp;

        // Determine the intermediate target.
        // The intermediate target is expressed in local coordinates.
        // The intermediate target is the intersection of the static world path and
        // the boundary of the visibility polygon.
//        Polygon visibilityPolygon = laserSensor.getVisibilityPolygon();
//        visibilityPolygon.scale(0.9, 0.9);
//        intermediateTarget.setPos(visibilityPolygon.pathIntersection(staticWorldPath).ip);
//        if (intermediateTarget.pos() == pp.last())
//            intermediateTarget.setHeading((mainTarget-pose()).heading());
//        else
//            intermediateTarget.setHeading((pp.last()-pp[pp.size()-2]).angle());

        // ..or a box.
        Box box = mapAndSensedGrid.boundingBox();
        box.grow(-0.1);
        Polygon bbox(box.bottomRight(), box.topRight(), box.topLeft(), box.bottomLeft());
        intermediateTarget.setPos(bbox.pathIntersection(staticWorldPath).ip);
        if (intermediateTarget.pos() == pp.last())
            intermediateTarget.setHeading((mainTarget-pose()).heading());
        else
            intermediateTarget.setHeading((pp.last()-pp[pp.size()-2]).angle());

        // Extract the carrot from the path.
        double carrotOffset = config.DWA_carrotOffset;
        if (command.trajectoryPlanningMethod == command.PD)
            carrotOffset = config.UPD_carrotOffset;
        uint i = 1;
        bool carrotFound = false;
        while (!carrotFound && i < staticWorldPath.size())
        {
            double segmentLength = (staticWorldPath[i]-staticWorldPath[i-1]).norm();
            if (segmentLength > carrotOffset)
            {
                carrot.setPos(staticWorldPath[i-1] + (carrotOffset/segmentLength) * (staticWorldPath[i]-staticWorldPath[i-1]));
                carrot.setHeading((staticWorldPath[i]-staticWorldPath[i-1]).angle());
                carrotFound = true;
            }
            else
            {
                carrotOffset -= segmentLength;
            }
            i++;
        }
        if (!carrotFound)
        {
            carrot = mainTarget-pose();
        }
    }
    else
    {
        //qDebug() << "frame" << state.frameId << "Static world path computation failed from" << pos() << "to:" << mainTarget;
        //state.stop = 1;
        //unifiedGeometricModel.setDebug(50);
        //unifiedGeometricModel.computePath(Vec2(), mainTarget-pose());
        //unifiedGeometricModel.setDebug(0);
    }

    state.pathTime = sw.elapsedTimeMs();


    //####################
    //# CONTROLLER LAYER #
    //####################
    // Action planning. High rate dynamic planning with bounded computation time.
    // The controller layer computes and sets the acceleration of the agent using Aborting A* or DWA or a PD controller.

    if (command.trajectoryPlanningMethod == command.DWA)
    {
        // This is a Unicycle DWA used to control a nonholonomic agent.

        StopWatch sw;
        sw.start();

        Unicycle localUni;
        localUni.setVel(vel());

        // Dynamic Window Approach based on nonholonomic trajectory types:
        // arc, B0 spline, and Fresnel integrals.
        //unicycleDWA.setDebug(config.debugLevel);
        unicycleDWA.setDynamicGeometricModel(expandedDynamicPolygons); // For collision checking.
        unicycleDWA.setGeometricModel(expandedMapAndSensedPolygons); // For path searches
        unicycleDWA.setGridModel(dilatedMapAndSensedGrid);
        unicycleDWA.setStart(localUni);
        unicycleDWA.setCarrot(carrot);
        Vec2 acc = unicycleDWA.search();
        double a = acc.x;
        double b = acc.y;
        setAcc(a, b);

        // Measure execution time.
        state.trajectoryTime = sw.elapsedTimeMs();
    }
    else if (command.trajectoryPlanningMethod == command.STAA)
    {
        StopWatch sw;
        sw.start();

        Unicycle localUni;
        localUni.setVel(rxVel);

        if (config.debugLevel > 5)
            shortTermAbortingAStar.setDebug(config.debugLevel);
        //shortTermAbortingAStar.setStaticGeometricModel(mapAndSensedPolygons);
        shortTermAbortingAStar.setExpandedStaticGeometricModel(expandedMapAndSensedPolygons);
        shortTermAbortingAStar.setDynamicGeometricModel(dynamicPolygons);
        shortTermAbortingAStar.setExpandedDynamicGeometricModel(expandedDynamicPolygons);
        shortTermAbortingAStar.setGridModel(mapAndSensedGrid);
        shortTermAbortingAStar.setDilatedGridModel(dilatedMapAndSensedGrid);
        shortTermAbortingAStar.setStartState(localUni);
        shortTermAbortingAStar.setTargetState(intermediateTarget);
        trajectorySuccess = shortTermAbortingAStar.aStarSearch();
        setAcc(shortTermAbortingAStar.getAction());
        //qDebug() << "ACTION:" << shortTermAbortingAStar.getAction() << "vel:" << vel() << "txVel:" << getTxVel();
        //qDebug() << "trace:" << shortTermAbortingAStar.getPlan();

        // Measure execution time.
        state.trajectoryTime = sw.elapsedTimeMs();
        state.aasExpansions = shortTermAbortingAStar.expansions;
        state.aasClosed = shortTermAbortingAStar.closed; // This actually meanvs how many expansions coming from closed cells were ignored.
        state.aasCollided = shortTermAbortingAStar.collided;
        state.aasProcessed = shortTermAbortingAStar.processed;
        //state.aasOpen = shortTermAbortingAStar.open.size();
        state.aasFinished = false;
    }
    else
    {
        StopWatch sw;
        sw.start();

        // Simple PD controller that steers towards the carrot.
        // When the dynamic path fails or deviates too strongly, a and b are set to zero.
        Vec2 acc = pdControlTo(carrot);
        setAcc(acc);

        // Measure execution time.
        state.trajectoryTime = sw.elapsedTimeMs();
    }

    // Keyboard control override.
    if (command.keyboard || command.joystick)
    {
        Vec2 comm(command.v, command.w); // between -1 and 1.
        double p = comm.norm();
        if (p > 1.0)
            comm /= p;
        Vec2 vv;
        vv.x = comm.x > 0 ? comm.x * config.agentLinearVelocityLimitForward : comm.x * -config.agentLinearVelocityLimitBackward;
        vv.y = comm.y * config.agentAngularVelocityLimit;

        //qDebug() << "command:" << command.v << command.w << "comm:" << p << comm << "vv:" << vv;

        setAcc(0, 0);
        setVel(vv);
    }
}

// Executes one robot control step without buffering.
void UnicycleRobot::step()
{
    StopWatch sw;
    sw.start();
    sense();
    state.senseTime = sw.elapsedTimeMs();
    sw.start();
    act();
    state.actTime = sw.elapsedTimeMs();
    state.executionTime = state.senseTime+state.actTime;
    //qDebug() << state.senseTime << state.actTime;
}

// Robot control step with data buffering.
void UnicycleRobot::bufferStep()
{
    QMutexLocker locker(&state.bigMutex);

    state.frameId++;
    state.timeDiff = stopWatch.elapsedTime();
    state.iterationTime = stopWatch.elapsedTimeMs();
    state.time = stopWatch.programTime();
    stopWatch.start();
    step();
    StopWatch sw;
    sw.start();
    state.buffer(config.bufferSize);
    state.bufferToFile();
    state.bufferTime = sw.elapsedTimeMs();
    rgbdUpdated = false;
    laserUpdated = false;
    //qDebug() << "sense:" << state.senseTime << "act:" << state.actTime << "buffer:" << state.bufferTime;
}

// Computes the ray sensing model.
void UnicycleRobot::computeRayModel()
{
    Vec2 base(1,0);
    base.normalize(config.raysLength);
    rayModel.clear();
    for (int i = 0; i < config.raysNumber; i++)
    {
        double angle = -config.raysAngleRange + i*2*config.raysAngleRange/(config.raysNumber-1);
        rayModel << unifiedGeometricModel.rayIntersection(Vec2(), base.rotated(angle));
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

// Draws the rgbd point cloud.
void UnicycleRobot::drawPointCloud() const
{
    if (command.showPointCloud == 1)
        rgbdSensor.drawPointCloud(config.floorHeight, config.ceilingHeight);
    if (command.showPointCloud == 2)
        rgbdSensor.drawPointCloud();
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
        GLlib::drawNoseCircle(odomHistory[i], colorUtil.ivory, 0.5*config.agentRadius);
    GLlib::drawNoseCircle(odomDiff+poseHistory.last(), colorUtil.transparent, 0.6*config.agentRadius);
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

    glColor4f(colorUtil.brushGreen.color().redF(), colorUtil.brushGreen.color().greenF(), colorUtil.brushGreen.color().blueF(), 0.9);
    GLlib::drawCylinder(config.agentRadius, config.agentHeight); // 90 cm high
}

// Draws the camera frame, the camera plane, and the frustum in an OpenGL context.
void UnicycleRobot::drawCameraPlane() const
{
    if (!command.showCameraFrame)
        return;
    rgbdSensor.drawCameraPlane();
}

// Draws the local grid model.
void UnicycleRobot::drawGridModel() const
{
    if (command.showGridModel == 4) // sensed only
    {
        sensedGrid.draw(colorUtil.brushMagenta);
    }
    if (command.showGridModel == 3) // map only
    {
        mapGrid.draw(colorUtil.brushRed);
    }
    if (command.showGridModel == 2) // all
    {
        dilatedMapAndSensedGrid.draw(colorUtil.brushOrange);
        glTranslated(0,0,0.001);
        mapAndSensedGrid.draw(colorUtil.brushRed);
        glTranslated(0,0,0.001);
        sensedGrid.draw(colorUtil.brushMagenta);
    }
    if (command.showGridModel > 0) // border
    {
        glTranslated(0,0,0.001);
        mapGrid.drawBorder();
    }
}

// Draws the local geometric model.
void UnicycleRobot::drawGeometricModel() const
{
    if (command.showGeometricModel == 1) // all
    {
        expandedMapAndSensedPolygons.draw(colorUtil.penThick, colorUtil.brushOrange, 0.5);
        //glTranslated(0,0,0.001);
        //mapAndSensedPolygons.draw(colorUtil.penThick, colorUtil.brushRed, 0.5);
        //glTranslated(0,0,0.001);
        //unifiedGeometricModel.draw(colorUtil.penThick, colorUtil.brushBlue, 0.5);
    }
    if (command.showGeometricModel > 0) // border
    {
        glTranslated(0,0,0.001);
        mapGrid.drawBorder();
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
void UnicycleRobot::drawWorldMap()
{
    geometricMap.draw();
}

// Draws a visualization of the visibility polygon extracted from the laser sensor.
void UnicycleRobot::drawVisibilityPolygon() const
{
    if (!command.showVisibilityPolygon)
        return;

    glPushMatrix();
    visibilityPolygon.draw(colorUtil.penThick, colorUtil.brushGreen, 0.2);
    //qDebug() << visibilityPolygon;

    glTranslated(0,0,0.01);

    Polygon boundedVisPol = visibilityPolygon;
    ListIterator<Line> ei = boundedVisPol.edgeIterator();
    while (ei.hasNext())
    {
        Line& edge = ei.next();
        Vec2& p1 = edge.p1();
        Vec2& p2 = edge.p2();
        if (p1.length() > config.slamVisibilityPolygonMaxDistance)
            p1.normalize(config.slamVisibilityPolygonMaxDistance);
        if (p2.length() > config.slamVisibilityPolygonMaxDistance)
            p2.normalize(config.slamVisibilityPolygonMaxDistance);
    }
    Vector<Polygon> pols = boundedVisPol.offseted(-config.slamVisibilityPolygonShrinking, config.laserDouglasPeuckerEpsilon);
    for (uint i = 0; i < pols.size(); i++)
        pols[i].draw(colorUtil.penThick, colorUtil.brushGreen, 0.2);

    glPopMatrix();
}

// Draws the static and the dynamic world path.
void UnicycleRobot::drawWorldPaths() const
{
    if (!command.showWorldPath)
        return;

    // Draw the static world path.
    if (staticWorldPathSuccess)
    {
        glColor3f(0.0,0.0,0.8);
        for (uint i = 1; i < staticWorldPath.size(); i++)
            GLlib::drawLine(staticWorldPath[i], staticWorldPath[i-1], 0.01);
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
    if (command.showVisibilityGraph)
        unifiedGeometricModel.drawVisibilityGraph();
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
    drawPointCloud(); // local

    drawBody(); // local
    drawCameraPlane(); // local

    glPopMatrix(); // end local coordinate frame
}

// QPainter drawing code.
void UnicycleRobot::draw(QPainter *painter) const
{
    // The body.
    if (command.showBody)
    {
        painter->setPen(colorUtil.penThick);
        painter->setBrush(colorUtil.brushYellow);
        Polygon::draw(painter);
    }

    // The pos dot.
    painter->save();
    painter->translate(pos());
    painter->rotate(orientation()*RAD_TO_DEG);
    painter->setPen(colorUtil.pen);
    painter->setBrush(colorUtil.brush);
    painter->scale(0.03, 0.03);
    painter->drawEllipse(QPointF(0, 0), 1, 1);
    painter->restore();

    // The main target.
    if (command.showTargets && mainTarget.norm() > EPSILON)
    {
        QPolygonF cross = colorUtil.getCrossPolygon();
        painter->save();
        painter->translate(mainTarget.pos());
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

    // The intermediate target.
    if (command.showTargets && intermediateTarget.pos().norm() > EPSILON)
    {
        QPolygonF cross = colorUtil.getCrossPolygon();
        painter->save();
        painter->translate(pos());
        painter->rotate(orientation()*RAD_TO_DEG);
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
        painter->translate(pos());
        painter->rotate(orientation()*RAD_TO_DEG);
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

    // The world paths.
    if (command.showWorldPath)
    {
        // Draw the static world path.
        if (staticWorldPathSuccess)
        {
            painter->save();
            painter->translate(pos());
            painter->rotate(orientation()*RAD_TO_DEG);
            painter->setPen(colorUtil.penBlueThick);
            painter->setBrush(colorUtil.brushBlue);
            painter->setOpacity(0.2);
            for (uint i = 1; i < staticWorldPath.size(); i++)
                painter->drawLine(QLineF(staticWorldPath[i].x, staticWorldPath[i].y, staticWorldPath[i-1].x, staticWorldPath[i-1].y));
            painter->restore();
        }

        // Draw the dynamic path.
        if (dynamicPathSuccess)
        {
            painter->save();
            painter->translate(pos());
            painter->rotate(orientation()*RAD_TO_DEG);
            painter->setPen(colorUtil.penRedThick);
            painter->setBrush(colorUtil.brushRed);
            painter->setOpacity(0.2);
            for (uint i = 1; i < dynamicPath.size(); i++)
                painter->drawLine(QLineF(dynamicPath[i].x, dynamicPath[i].y, dynamicPath[i-1].x, dynamicPath[i-1].y));
            painter->restore();
        }
    }

    // Visibility graph.
    if (command.showVisibilityGraph)
    {
        painter->save();
        painter->translate(pos());
        painter->rotate(orientation()*RAD_TO_DEG);
        unifiedGeometricModel.drawVisibilityGraph(painter); // This is used for global path searches.
        //shortTermAbortingAStar.drawVisibilityGraph(painter); // This is used for local path searches in the heuristic of AA*.
        painter->restore();
    }

    // Draw the ray model.
    if (command.showRayModel)
    {
        painter->save();
        Vec2 base(1,0);
        const double& maxRange = std::max(config.gridHeight, config.gridWidth);
        const Vector<Vec2>& rayEndpoints = rayModel;
        for (uint i = 0; i < rayEndpoints.size(); i++)
        {
            if (rayEndpoints[i] < maxRange-EPSILON)
            {
                painter->setBrush(colorUtil.brushRed);
                painter->setPen(colorUtil.penRed);
            }
            else
            {
                painter->setBrush(colorUtil.brush);
                painter->setPen(colorUtil.pen);
            }
            painter->drawLine(QPointF(), rayEndpoints[i]);
            painter->drawEllipse(rayEndpoints[i], 0.03, 0.03);
        }
        painter->restore();
    }

    // The sensed grid.
    if (command.showGridModel > 1)
    {
        painter->save();
        painter->translate(pos());
        painter->rotate(orientation()*RAD_TO_DEG);

        dilatedMapAndSensedGrid.draw(painter, colorUtil.brushOrange);
        //sensedGrid.draw(painter, colorUtil.brushOrange);
        mapGrid.draw(painter, colorUtil.brushRed);
        sensedGrid.draw(painter, colorUtil.brushYellow);
        painter->restore();
    }

    // ...and its border.
    if (command.showGridModel > 0)
    {
        painter->save();
        painter->translate(pos());
        painter->rotate(orientation()*RAD_TO_DEG);
        mapGrid.drawBorder(painter);
        painter->restore();
    }

    // The local geometric models (static and dynamic).
    if (command.showGeometricModel)
    {
        painter->save();
        painter->translate(pos());
        painter->rotate(orientation()*RAD_TO_DEG);
        expandedMapAndSensedPolygons.draw(painter, colorUtil.penThick, colorUtil.brushOrange);
        //mapAndSensedPolygons.draw(painter, colorUtil.penThick, colorUtil.brushRed);
        expandedDynamicPolygons.draw(painter, colorUtil.penThick, colorUtil.brushBlue);
        mapGrid.drawBorder(painter);
        painter->restore();
    }

    // Controller visualization.
    if (command.trajectoryPlanningMethod == command.DWA)
    {
        painter->save();
        painter->translate(pos());
        painter->rotate(orientation()*RAD_TO_DEG);
        unicycleDWA.draw(painter);
        painter->restore();
    }
    else if (command.trajectoryPlanningMethod == command.STAA)
    {
        painter->save();
        painter->translate(pos());
        painter->rotate(orientation()*RAD_TO_DEG);
        shortTermAbortingAStar.draw(painter);
        painter->restore();
    }
}

void UnicycleRobot::manualMapUpdate()
{
    qDebug() << state.frameId << "Manual map update.";
    const Vector<TrackedLine> &laserLines = laserSensor.extractLines();
    //lineMap.slam(pose(), laserLines, true);
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

void UnicycleRobot::streamOut(QDataStream& out) const
{
    //out << rgbdSensor;
    out << laserSensor;
    out << mainTarget.pos();
    out << pose();
    out << vel();
    out << rgbdUpdated;
    out << laserUpdated;
    out << initialPose;
    out << externalLocalizerPose;
    out << odomPose;
}

void UnicycleRobot::streamIn(QDataStream &in)
{
    Vec2 temp;
    //in >> rgbdSensor;
    in >> laserSensor;
    in >> temp;
    mainTarget.setPos(temp);
    Pose2D pose;
    in >> pose;
    setPose(pose);
    Vec2 vel;
    in >> vel;
    setVel(vel);
    in >> rgbdUpdated;
    in >> laserUpdated;
    in >> initialPose;
    in >> externalLocalizerPose;
    in >> odomPose;
}

QDataStream& operator<<(QDataStream& out, const UnicycleRobot &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, UnicycleRobot &o)
{
    o.streamIn(in);
    return in;
}

QDebug operator<<(QDebug dbg, const UnicycleRobot &o)
{
    dbg << o.pose();
    return dbg;
}

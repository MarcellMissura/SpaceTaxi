#include "Config.h"
#include "lib/globals.h"
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QDebug>

// The global config object contains application wide configuration variables.
// Basically, the config object is a globally accessible struct with public
// read and write access for everyone (not thread safe!). Yes, even if this
// violates common programming paradigms, it focuses on simplicity of use.
// Just include the config header anywhere and use config.blabla to access a
// configuration parameter. Typically, you will not want to write any parameters
// during runtime, only the config slider widget wants to do so.
// Similar to the State object, the Config object provides some basic reflection
// capabilities so that config sliders can be automatically generated for the gui.
// This is also used to automatically generate a config file that can be saved and
// loaded to preserve the config variables.
// All config variables are declared in this central place. Declare them in the
// config.h header, initialize them in the constructor, and optionally register
// them in the init() method if you want a slider to be created and the config
// variable to be saved in the file. Every registered config variable gets a name
// and a slider factor assigned that determines the sensitivity of the slider.
// The config object also supports save() and load() functions that serializes
// and unserializes the variable contents in a hand editable text file. The save
// and load functions take a robot name as an argument to support different config
// sets for different robots.


Config config;

Config::Config()
{
    // simulation
    simSpeedUp = 1.0;
    simLaserRays = 512; // How many sensor rays.
    simLaserAngleRange = 2.0; // The opening angle of the sensor field.
    simLaserLength = 10; // The maximum length of the simulated laser rays.
    simLinearDamping = 1.0;
    simAngularDamping = 1.0;
    simFriction = 0.3;

    // world and maps
    navGoalRadius = 0.5;
    unicycleAgents = 1;
    voidRadius = 0.4;
    voidDropOffPoints = 2;
    simpleRadius = 0.4;
    simpleDropOffPoints = 6;
    uRadius = 0.5;
    uDropOffPoints = 6;
    tunnelCorridorWidth = 2.0;
    wareHouseAisleWidth = 2.0;
    clutterWidth = 20;
    clutterHeight = 20;
    clutterObstacles = 100;
    clutterObstaclesSizeMin = 0.1;
    clutterObstaclesSizeMax = 1.0;
    clutterDropOffPoints = 4;
    officeRoomSize = 1.0; // width and height of one room in meters
    officeWallThickness = 0.05; // in percent of one unit
    officeDoorWidth = 0.3; // in percent of one unit
    officeRooms = 10; // number of rooms per unit

    // framework
    bufferSize = 100;
    debugLevel = 1;

    // agent
    agentRadius = 0.2;
    agentHeight = 0.5;
    agentWidth = 0.8;
    agentLinearAccelerationLimit = 5;
    agentLinearVelocityLimitForward = 2;
    agentLinearVelocityLimitBackward = -1;
    agentAngularAccelerationLimit = 4;
    agentAngularVelocityLimit = 1.5;
    agentTargetReachedDistance = 0.3;

    // laser sensor processing
    laserMaxRange = 18;
    laserSmoothingMinSegmentSize = 3;
    laserSmoothingSpatialPasses = 1;
    laserSegmentDistanceThreshold = 0.15;
    laserDouglasPeuckerEpsilon = 0.02;
    laserPolygonThickness = 0.2;
    laserLineMinLength = 0.3;
    laserLineMaxDistance = 6;
    laserLineMinAngle = 0.4;
    laserTriangleLegLength = 0.17; // The side length of the triangle.
    laserTriangleAngle = 2.3; // The angle at the tip of the triangle marker.
    laserTriangleSizeTolerance = 0.2;
    laserTriangleSmoothing = 0.5;

    // ray model
    raysAngleRange = -PI2;
    raysNumber = 11;
    raysLength = 5;

    // geometric model
    gmPolygonDilation = 0.5;
    gmAgentDilation = 0.5;
    gmDouglasPeuckerEpsilon = 0.05;
    gmPolygonPruning = 0.01;

    // grid model
    gridBlurRadius = 0.2;
    gridSensedDilationRadius = 0.6;
    gridWidth = 8;
    gridHeight = 8;
    gridOffset = 2;
    gridCellSize = 0.05; // The size of one cell in the grid model, e.g. 5 cm.
    gridClosedCellSize = 0.025;

    // Prediction
    predictFactor = 0.25; // How strongly does the prediction time increase with distance.
    predictMaxPredictTime = 1.0; // How much time to predict at most.
    predictIgnoreHorizon = 5.0; // From what ETA are obstacles ignored.

    // Emergency brake reflex.
    ebPredictionTime = 0.5;

    // Bezier control
    bezierDt = 0.6;
    bezierStretch = 1.0;

    // Reel control
    reelCarrotOffset = 0.2;

    // RuleBase controller
    rbRayDistanceThreshold = 0.02;
    rbTargetDistanceThreshold = 0.2;
    rbCarrotDistanceThreshold = 0.01;

    // ST Aborting A* config
    staaNotches = 7;
    staaExpansionLimit = 2000;
    staaDt = 0.1;
    staaFinishedThreshold = 0.4;
    staaGridCostWeight = 1.0;
    staaProximityCostWeight = 1.0;
    staaPolygonGrowth = 0.1;
    staaStucknessWeight = 1.0;
    staaInflationFactor = 1.0;

    // DWA
    DWA_carrotOffset = 1.0;
    DWA_notches = 7;
    DWA_time = 0.3;
    DWA_samples = 2;
    DWA_gridClearance = 0.8;
    DWA_geometricClearance = 1.0;
    DWA_carrotDistance = 0.5;
    DWA_backwardsPenalty = 0.0;

    // Unicycle PD controller.
    UPD_carrotOffset = 0.4;
    UPD_Plin = 10;
    UPD_Dlin = -1;
    UPD_Prot = 100;
    UPD_Drot = -10;

    // Line slam.
    slamSeenCornerMinAngle = 0.785;
    slamVisibilityPolygonBound = 7.0;
    slamMaxPoseDiff = 0.25;
    slamMergeMaxLineDist = 0.106;
    slamMergeMinOverlap = 0.15;
    slamSelectionBoxSize = 10.0;
    slamPairingMaxLengthDeviation = 0.26;
    slamPairingMaxPoseDist = 0.5;
    slamMinObservationCount = 4;
    slamClusteringAngleEps = 0.21;
    slamClusteringOrthoEps = 0.05;
    slamClusteringTransformEps = 0.1;
    slamHypMinOverlapPercent = 0.75;
    slamPoseGraphNodeDist = 0.5;
    slamPoseGraphNeighborhoodSize = 3;
    slamSnapAcceptanceThreshold = 0.9;
}

// The init() method should be called after construction.
// Here, all config variables are registered to build a descriptor meta
// structure that allows index and key based access to their values.
// If you don't want to see a certain member on the gui, there is no
// need to register it.
void Config::init()
{
    // framework
    registerMember("bufferSize", &bufferSize, 1000.0);
    registerMember("debugLevel", &debugLevel, 100.0);

    // simulation
    registerMember("simulation.speedUp", &simSpeedUp, 10.0);
    registerMember("simulation.linearDamping", &simLinearDamping, 10.0);
    registerMember("simulation.angularDamping", &simAngularDamping, 10.0);
    registerMember("simulation.friction", &simFriction, 10.0);
    registerMember("simulation.laser.rays", &simLaserRays, 1000);
    registerMember("simulation.laser.angleRange", &simLaserAngleRange, PI);
    registerMember("simulation.laser.length", &simLaserLength, 50.0);

    // world and maps
    registerMember("world.navGoalRadius", &navGoalRadius, 10.0);
    registerMember("world.unicycleAgents", &unicycleAgents, 100.0);
    registerMember("world.void.radius", &voidRadius, 0.5);
    registerMember("world.void.dropOffPoints", &voidDropOffPoints, 100.0);
    registerMember("world.simple.radius", &simpleRadius, 0.5);
    registerMember("world.simple.dropOffPoints", &simpleDropOffPoints, 100.0);
    registerMember("world.u.radius", &uRadius, 0.5);
    registerMember("world.u.dropOffPoints", &uDropOffPoints, 100.0);
    registerMember("world.tunnel.corridorWidth", &tunnelCorridorWidth, 20.0);
    registerMember("world.warehouse.aisleWidth", &wareHouseAisleWidth, 10.0);
    registerMember("world.clutter.width", &clutterWidth, 100.0);
    registerMember("world.clutter.height", &clutterHeight, 100.0);
    registerMember("world.clutter.obstacles", &clutterObstacles, 1000.0);
    registerMember("world.clutter.obstaclesSizeMin", &clutterObstaclesSizeMin, 1.0);
    registerMember("world.clutter.obstaclesSizeMax", &clutterObstaclesSizeMax, 10.0);
    registerMember("world.clutter.dropOffPoints", &clutterDropOffPoints, 100.0);
    registerMember("world.office.roomSize", &officeRoomSize, 10.0);
    registerMember("world.office.wallThickness", &officeWallThickness, 0.1);
    registerMember("world.office.doorWidth", &officeDoorWidth, 1.0);
    registerMember("world.office.rooms", &officeRooms, 100.0);

    // agent
    registerMember("agent.height", &agentHeight, 10.0);
    registerMember("agent.width", &agentWidth, 10.0);
    registerMember("agent.linearAccLimit", &agentLinearAccelerationLimit, 10.0);
    registerMember("agent.linearVelLimitForward", &agentLinearVelocityLimitForward, 5.0);
    registerMember("agent.linearVelLimitBackward", &agentLinearVelocityLimitBackward, 5.0);
    registerMember("agent.angularAccLimit", &agentAngularAccelerationLimit, 10.0);
    registerMember("agent.angularVelLimit", &agentAngularVelocityLimit, 5.0);
    registerMember("agent.targetReachedDistance", &agentTargetReachedDistance, 1.0);

    // laser sensor processing
    registerMember("laser.maxRange", &laserMaxRange, 1.0);
    registerMember("laser.segmentDistanceThreshold", &laserSegmentDistanceThreshold, 1.0);
    registerMember("laser.douglasPeuckerEpsilon", &laserDouglasPeuckerEpsilon, 0.1);
    registerMember("laser.smoothingPasses", &laserSmoothingSpatialPasses, 100);
    registerMember("laser.minSegmentSize", &laserSmoothingMinSegmentSize, 100);
    registerMember("laser.polygonThickness", &laserPolygonThickness, 1.0);
    registerMember("laser.lineMinLength", &laserLineMinLength, 1.0);
    registerMember("laser.lineMaxDistance", &laserLineMaxDistance, 20.0);
    registerMember("laser.lineMinAngle", &laserLineMinAngle, PI2);
    registerMember("laser.triangleLegLength", &laserTriangleLegLength, 1.0);
    registerMember("laser.triangleAngle", &laserTriangleAngle, PI);
    registerMember("laser.triangleSizeTolerance", &laserTriangleSizeTolerance, 1.0);
    registerMember("laser.triangleSmoothing", &laserTriangleSmoothing, 1.0);

    // ray model
    registerMember("rayModel.number", &raysNumber, 100.00);
    registerMember("rayModel.angle", &raysAngleRange, PI);
    registerMember("rayModel.length", &raysLength, 10.00);

    // grid model
    registerMember("gridModel.cellSize", &gridCellSize, 0.5);
    registerMember("gridModel.closedCellSize", &gridClosedCellSize, 0.5);
    registerMember("gridModel.sensedDilationRadius", &gridSensedDilationRadius, 1.0);
    registerMember("gridModel.blurRadius", &gridBlurRadius, 1.0);
    registerMember("gridModel.gridWidth", &gridWidth, 10);
    registerMember("gridModel.gridHeight", &gridHeight, 10);
    registerMember("gridModel.gridOffset", &gridOffset, 5);

    // geometric model
    registerMember("geometricModel.polygonDilation", &gmPolygonDilation, 1.0);
    registerMember("geometricModel.agentDilation", &gmAgentDilation, 1.0);
    registerMember("geometricModel.DouglasPeuckerEpsilon", &gmDouglasPeuckerEpsilon, 1.0);
    registerMember("geometricModel.polygonPruning", &gmPolygonPruning, 0.5);

    // Line Slam
    registerMember("lineSlam.seenCornerMinAngle", &slamSeenCornerMinAngle, PI2);
    registerMember("lineSlam.minObservationCount", &slamMinObservationCount, 100.0);
    registerMember("lineSlam.maxPoseDiff", &slamMaxPoseDiff, 0.5);
    registerMember("lineSlam.mergeMaxLineDist", &slamMergeMaxLineDist, 0.2);
    registerMember("lineSlam.mergeMinOverlap", &slamMergeMinOverlap, 0.25);
    registerMember("lineSlam.selectionBoxSize", &slamSelectionBoxSize, 20.0);
    registerMember("lineSlam.pairingMaxLengthDeviaton", &slamPairingMaxLengthDeviation, 1.0);
    registerMember("lineSlam.pairingMaxPoseDist", &slamPairingMaxPoseDist, 1.0);
    registerMember("lineSlam.hypMinOverlapPct", &slamHypMinOverlapPercent, 1.0);
    registerMember("lineSlam.clusteringAngleEps", &slamClusteringAngleEps, 0.5);
    registerMember("lineSlam.clusteringOrthoEps", &slamClusteringOrthoEps, 0.5);
    registerMember("lineSlam.clusteringTransformEps", &slamClusteringTransformEps, 0.5);
    registerMember("lineSlam.poseGraphNodeDist", &slamPoseGraphNodeDist, 2.0);
    registerMember("lineSlam.poseGraphNeighborhoodSize", &slamPoseGraphNeighborhoodSize, 100.0);
    registerMember("lineSlam.visibilityPolygonBound", &slamVisibilityPolygonBound, 20.0);
    registerMember("lineSlam.snapAcceptanceThreshold", &slamSnapAcceptanceThreshold, 1.0);



    // Path planning
    registerMember("prediction.predictFactor", &predictFactor, 1.0);
    registerMember("prediction.maxPredictTime", &predictMaxPredictTime, 3.0);
    registerMember("prediction.ignoreHorizon", &predictIgnoreHorizon, 5.0);

    registerMember("emergencyBrake.predictionTime", &ebPredictionTime, 2.0);

    // Bezier control
    registerMember("bezier.dt", &bezierDt, 1.0);
    registerMember("bezier.stretch", &bezierStretch, 10.0);

    // Reel control
    registerMember("reel.carrotOffset", &reelCarrotOffset, 1.0);

    // RuleBase controller
    registerMember("RuleBase.targetDistanceThreshold", &rbTargetDistanceThreshold, 1.0);
    registerMember("RuleBase.carrotDistanceThreshold", &rbCarrotDistanceThreshold, 1.0);
    registerMember("RuleBase.rayDistanceThreshold", &rbRayDistanceThreshold, 1.0);

    // ST Aborting A*
    registerMember("STAA.notches", &staaNotches, 100.0);
    registerMember("STAA.Dt", &staaDt, 1.0);
    registerMember("STAA.expansionLimit", &staaExpansionLimit, 10000.0);
    registerMember("STAA.finishedThresh", &staaFinishedThreshold, 0.1);
    registerMember("STAA.gridCostWeight", &staaGridCostWeight, 2.0);
    registerMember("STAA.stucknessWeight", &staaStucknessWeight, 2.0);
    registerMember("STAA.proximityCostWeight", &staaProximityCostWeight, 2.0);
    registerMember("STAA.polygonGrowth", &staaPolygonGrowth, 1.0);
    registerMember("STAA.inflationFactor", &staaInflationFactor, 10.0);

    // DWA
    registerMember("DWA.carrotOffset", &DWA_carrotOffset, 3.0);
    registerMember("DWA.notches", &DWA_notches, 100.0);
    registerMember("DWA.time", &DWA_time, 2.0);
    registerMember("DWA.samples", &DWA_samples, 100.0);
    registerMember("DWA.gridClearance", &DWA_gridClearance, 1.0);
    registerMember("DWA.geometricClearance", &DWA_geometricClearance, 1.0);
    registerMember("DWA.carrotDistance", &DWA_carrotDistance, 1.0);
    registerMember("DWA.backwardsPenalty", &DWA_backwardsPenalty, 1.0);

    // Unicycle PD controller.
    registerMember("UniPD.carrotOffset", &UPD_carrotOffset, 1.0);
    registerMember("UniPD.Plin", &UPD_Plin, 100.0);
    registerMember("UniPD.Dlin", &UPD_Dlin, 100.0);
    registerMember("UniPD.Prot", &UPD_Prot, 100.0);
    registerMember("UniPD.Drot", &UPD_Drot, 100.0);
}

// Loads the config variables from the .conf file.
// Unregistered variables are ignored.
void Config::load(QString robotName)
{
    QFile file("conf/" + robotName + ".conf");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Couldn't load config file" << file.fileName();
        return;
    }

    QTextStream in(&file);
    QString line;
    QStringList list;
    bool ok;
    while (!in.atEnd())
    {
        line = in.readLine().trimmed();
        list = line.split("=");
        if (list.length() == 2 && !line.startsWith("//") && !line.startsWith("#"))
        {
            QString key = list[0].trimmed();
            double value = list[1].trimmed().toDouble(&ok);
            if (memberNames.contains(key))
                this->operator[](key) = value;
        }
    }

    file.close();
}

// Saves the config variables to the .conf file.
void Config::save(QString robotName)
{
	QFile file("conf/" + robotName + ".conf");
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		qDebug() << "Couldn't open config file" << file.fileName();
		return;
	}

	QTextStream out(&file);
	foreach (QString key, memberNames)
		out << key << "=" << QString::number(this->operator[](key)) << endl;
	file.close();
}

// Returns a reference to the ith member of this object.
double& Config::operator()(int i)
{
	return this->operator[](memberNames[i]);
}

// Returns a reference to the ith member of this object.
double& Config::operator[](int i)
{
	return this->operator[](memberNames[i]);
}

// Returns a reference to the member that was registered with the given key.
double& Config::operator()(QString key)
{
	return this->operator[](key);
}

// Returns a reference to the member that was registered with the given key,
// e.g., "agent.width". If you try to access an unregistered member, you will
// get a useless reference to a sinkhole and a warning.
double& Config::operator[](QString key)
{
    if (!memberNames.contains(key))
    {
        qDebug() << "You are trying to access a non existent config member" << key;
        return sink;
    }

    double* ptr = (double*)((unsigned long int)this+memberOffsets[key]);
    double& rf = *ptr;
    return rf;
}

#include "Config.h"
#include "globals.h"
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QDebug>

// The global config object contains application wide configuration variables.
// Basically, the config object is a globally accessible struct with public
// read and write access for everyone (not thread safe!). Yes, even if this
// violates common programming paradigms, it focuses on simplicity of use.
// Just include the config header anywhere and use config.blabla to access a
// configuration parameter. Typically you will not want to write any parameters
// during runtime, only the config slider widget wants to do so.
// Similar to the State object, the Config object provides some basic reflection
// capabilities so that config sliders can be automatically generated for the gui.
// This is also used to automatically generate a config file that can be saved and
// loaded to preserve the config variables.
// All config variables are declared in this central place. Declare them in the
// config.h header, initialize them in the constructor and optionally register
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
    rcIterationTime = 0.01;
    speedUp = 1.0;
    bufferSize = 100;
    debugLevel = 1;

    // agent
    agentHeight = 0.5;
    agentWidth = 0.8;
    agentLinearAccelerationLimit = 5;
    agentLinearVelocityLimitForward = 2;
    agentLinearVelocityLimitBackward = -1;
    agentLinearJerkLimit = 20;
    agentLinearDamping = 1.0;
    agentAngularAccelerationLimit = 4;
    agentAngularVelocityLimit = 1.5;
    agentAngularDamping = 3.0;
    agentFriction = 0.3;

    // ray model
    raysAngleRange = -PI2;
    raysNumber = 11;
    raysLength = 5;

    // geometric model
    gmPolygonExpansionMargin = 0.7;

    // grid model
    gridDouglasPeuckerEpsilon = 1.0;
    gridMinimumSegmentSize = 0.0;
    gridBlurRadius = 0.7;
    gridWorldDilationRadius = 0.7;
    gridSensedDilationRadius = 0.5;
    gridWidth = 8;
    gridHeight = 8;
    gridOffset = 2;
    gridCellSize = 0.05; // The size of one cell in the grid model, e.g. 5 cm.
    gridClosedCellSize = 0.025;

    // world
    worldDropOffRadius = 3.5;
    unicycleAgents = 1;

    // maps
    voidRadius = 0.4;
    voidDropOffPoints = 2;
    simpleRadius = 0.4;
    simpleDropOffPoints = 6;
    uRadius = 0.5;
    uDropOffPoints = 6;

    outdoorWidth = 50;
    outdoorHeight = 35;
    outdoorObstaclesNr = 20;
    outdoorObstaclesMinWidth = 5;
    outdoorObstaclesMaxWidth = 20;
    outdoorObstaclesMinHeight = 0.1;
    outdoorObstaclesMaxHeight = 10;
    outdoorCarsNr = 10;

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

    // RuleBase controller
    rbRayDistanceThreshold = 0.02;
    rbTargetDistanceThreshold = 0.2;
    rbCarrotDistanceThreshold = 0.01;

    // ST Aborting A* config
    staaNotches = 7;
    staaExpansionLimit = 1000;
    staaTimeLimit = 19;
    staaDt = 0.1;
    staaFinishedThreshold = 0.4;
    staaGridCostWeight = 1.0;
    staaProximityCostWeight = 1.0;
    staaPolygonGrowth = 0.1;
    staaCollisionGridDilation = 0.1;
    staaSidePreference = 0;
    staaPredictionTimeLimit = 2.0;
    staaStucknessWeight = 1.0;

    // DWA
    DWA_carrotOffset = 1.0;
    DWA_notches = 7;
    DWA_time = 0.3;
    DWA_depth = 1;
    DWA_fraction = 0.5;
    DWA_samples = 2;
    DWA_gridClearance = 0.8;
    DWA_geometricClearance = 1.0;
    DWA_targetDistance = 0.5;
    DWA_targetAngle = 0.0;
    DWA_backwardsPenalty = 0.0;

    // Unicycle PD controller.
    UPD_carrotOffset = 0.4;
    UPD_Kp_lin = 10;
    UPD_Kd_lin = -1;
    UPD_Kp_rot = 100;
    UPD_Kd_rot = -10;

    // GUI
    sampleFactor = 1.0;
    colorContrast = 1.0;
    transparency = 0;
    scale = 1.0;
}

// The init() method should be called after construction.
// Here, all config variables are registered to build a descriptor meta
// structure that allows index and key based access to their values.
// If you don't want to see a certain member on the gui, there is no
// need to register it.
void Config::init()
{
    registerMember("rcIterationTime", &rcIterationTime, 0.1);
    registerMember("speedUp", &speedUp, 10.0);
    registerMember("bufferSize", &bufferSize, 1000.0);
    registerMember("debugLevel", &debugLevel, 100.0);

    // agent
    registerMember("agent.height", &agentHeight, 10.0);
    registerMember("agent.width", &agentWidth, 10.0);
    registerMember("agent.linearAccLimit", &agentLinearAccelerationLimit, 10.0);
    registerMember("agent.linearVelLimitForward", &agentLinearVelocityLimitForward, 10.0);
    registerMember("agent.linearVelLimitBackward", &agentLinearVelocityLimitBackward, 10.0);
    registerMember("agent.linearJerkLimit", &agentLinearJerkLimit, 500.0);
    registerMember("agent.linearDamping", &agentLinearDamping, 10.0);
    registerMember("agent.angularAccLimit", &agentAngularAccelerationLimit, 10.0);
    registerMember("agent.angularVelLimit", &agentAngularVelocityLimit, 10.0);
    registerMember("agent.angularDamping", &agentAngularDamping, 10.0);
    registerMember("agent.friction", &agentFriction, 10.0);

    // ST Aborting A*
    registerMember("RuleBase.targetDistanceThreshold", &rbTargetDistanceThreshold, 1.0);
    registerMember("RuleBase.carrotDistanceThreshold", &rbCarrotDistanceThreshold, 1.0);
    registerMember("RuleBase.rayDistanceThreshold", &rbRayDistanceThreshold, 1.0);

    // ST Aborting A*
    registerMember("STAA*.notches", &staaNotches, 100.0);
    registerMember("STAA*.Dt", &staaDt, 1.0);
    registerMember("STAA*.expansionLimit", &staaExpansionLimit, 10000.0);
    registerMember("STAA*.timeLimit", &staaTimeLimit, 100);
    registerMember("STAA*.finishedThresh", &staaFinishedThreshold, 1.0);
    registerMember("STAA*.gridCostWeight", &staaGridCostWeight, 2.0);
    registerMember("STAA*.stucknessWeight", &staaStucknessWeight, 2.0);
    registerMember("STAA*.proximityCostWeight", &staaProximityCostWeight, 2.0);
    registerMember("STAA*.polygonGrowth", &staaPolygonGrowth, 1.0);
    registerMember("STAA*.collisionGridDilation", &staaCollisionGridDilation, 0.5);
    registerMember("STAA*.predictionTimeLimit", &staaPredictionTimeLimit, 2.0);
    registerMember("STAA*.sidePreference", &staaSidePreference, 1.0);

    // DWA
    registerMember("DWA.carrotOffset", &DWA_carrotOffset, 3.0);
    registerMember("DWA.notches", &DWA_notches, 100.0);
    registerMember("DWA.depth", &DWA_depth, 3.0);
    registerMember("DWA.time", &DWA_time, 2.0);
    registerMember("DWA.fraction", &DWA_fraction, 1.0);
    registerMember("DWA.samples", &DWA_samples, 100.0);
    registerMember("DWA.gridClearance", &DWA_gridClearance, 1.0);
    registerMember("DWA.geometricClearance", &DWA_geometricClearance, 1.0);
    registerMember("DWA.targetDistance", &DWA_targetDistance, 1.0);
    registerMember("DWA.targetAngle", &DWA_targetAngle, 1.0);
    registerMember("DWA.backwardsPenalty", &DWA_backwardsPenalty, 1.0);

    // Unicycle PD controller.
    registerMember("UniPD.carrotOffset", &UPD_carrotOffset, 1.0);
    registerMember("UniPD.Kp_lin", &UPD_Kp_lin, 100.0);
    registerMember("UniPD.Kd_lin", &UPD_Kd_lin, 10.0);
    registerMember("UniPD.Kp_rot", &UPD_Kp_rot, 100.0);
    registerMember("UniPD.Kd_rot", &UPD_Kd_rot, 20.0);

    // rays
    registerMember("rayModel.number", &raysNumber, 100.00);
    registerMember("rayModel.angle", &raysAngleRange, PI);
    registerMember("rayModel.length", &raysLength, 10.00);

    // grid model
    registerMember("gridModel.douglasPeuckerEpsilon", &gridDouglasPeuckerEpsilon, 2.0);
    registerMember("gridModel.miniumSegmentSize", &gridMinimumSegmentSize, 10.0);
    registerMember("gridModel.cellSize", &gridCellSize, 0.5);
    registerMember("gridModel.closedCellSize", &gridClosedCellSize, 0.5);
    registerMember("gridModel.worldDilationRadius", &gridWorldDilationRadius, 2.0);
    registerMember("gridModel.sensedDilationRadius", &gridSensedDilationRadius, 2.0);
    registerMember("gridModel.blurRadius", &gridBlurRadius, 2.0);
    registerMember("gridModel.gridWidth", &gridWidth, 10);
    registerMember("gridModel.gridHeight", &gridHeight, 10);
    registerMember("gridModel.gridOffset", &gridOffset, 5);

    // geometric model
    registerMember("geometricModel.expansionMargin", &gmPolygonExpansionMargin, 2.0);

    // world
    registerMember("world.dropOffRadius", &worldDropOffRadius, 10.0);
    registerMember("world.unicycleAgents", &unicycleAgents, 100.0);

    // maps
    registerMember("maps.void.radius", &voidRadius, 0.5);
    registerMember("maps.void.dropOffPoints", &voidDropOffPoints, 100.0);
    registerMember("maps.simple.radius", &simpleRadius, 0.5);
    registerMember("maps.simple.dropOffPoints", &simpleDropOffPoints, 100.0);
    registerMember("maps.u.radius", &uRadius, 0.5);
    registerMember("maps.u.dropOffPoints", &uDropOffPoints, 100.0);
    registerMember("maps.outdoor.obstacles", &outdoorObstaclesNr, 100.0);
    registerMember("maps.outdoor.width", &outdoorWidth, 100.0);
    registerMember("maps.outdoor.height", &outdoorHeight, 100.0);
    registerMember("maps.outdoor.obstaclesMinWidth", &outdoorObstaclesMinWidth, 100.0);
    registerMember("maps.outdoor.obstaclesMaxWidth", &outdoorObstaclesMaxWidth, 100.0);
    registerMember("maps.outdoor.obstaclesMinHeight", &outdoorObstaclesMinHeight, 100.0);
    registerMember("maps.outdoor.obstaclesMaxHeight", &outdoorObstaclesMaxHeight, 100.0);
    registerMember("maps.clutter.width", &clutterWidth, 100.0);
    registerMember("maps.clutter.height", &clutterHeight, 100.0);
    registerMember("maps.clutter.obstacles", &clutterObstacles, 1000.0);
    registerMember("maps.clutter.obstaclesSizeMin", &clutterObstaclesSizeMin, 1.0);
    registerMember("maps.clutter.obstaclesSizeMax", &clutterObstaclesSizeMax, 10.0);
    registerMember("maps.clutter.dropOffPoints", &clutterDropOffPoints, 200.0);
    registerMember("maps.office.roomSize", &officeRoomSize, 10.0);
    registerMember("maps.office.wallThickness", &officeWallThickness, 0.1);
    registerMember("maps.office.doorWidth", &officeDoorWidth, 1.0);
    registerMember("maps.office.rooms", &officeRooms, 100.0);

    // GUI
    registerMember("gui.sampleFactor", &sampleFactor, 100.0);
    registerMember("gui.colorContrast", &colorContrast, 10.0);
    registerMember("gui.transparency", &transparency, 1.0);
    registerMember("gui.scale", &scale, 10.0);
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

// Returns a reference to the member that was registered with the given key.
// If you try to access an unregistered member, you will get a useless reference to a black hole and a warning.
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

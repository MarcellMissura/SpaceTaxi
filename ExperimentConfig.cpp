#include "ExperimentConfig.h"
#include "blackboard/Command.h"

QString ExperimentConfig::getMapName() const
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

QString ExperimentConfig::getControllerName() const
{
    QString controller;
    if (trajectoryPlanningMethod == command.PD)
        controller = "PD";
    if (trajectoryPlanningMethod == command.DWA)
        controller = "DWA";
    if (trajectoryPlanningMethod == command.STAA)
        controller = "STAA*";
    return controller;
}

QString ExperimentConfig::getTrajectoryType() const
{
    QString type;
    if (trajectoryType == command.Arc)
        type = "Arc";
    if (trajectoryType == command.B0)
        type = "B0";
    if (trajectoryType == command.Fresnel)
        type = "Fresnel";
    return type;
}

QString ExperimentConfig::getHeuristicName() const
{
    QString type;
    if (heuristic == command.MinimalConstruct)
        type = "MC";
    if (heuristic == command.GridDijkstra)
        type = "Dij";
    if (heuristic == command.PathEuklidean)
        type = "PEuk";
    if (heuristic == command.ReedShepp)
        type = "Ree";
    return type;
}

QString ExperimentConfig::getPredictionType() const
{
    QString type;
    if (predict == command.Holonomic)
        type = "Hol";
    if (predict == command.Unicycle)
        type = "Uni";
    if (predict == command.None)
        type = "None";
    return type;
}

QString ExperimentConfig::getFreqName() const
{
    QString fr;
    if (frequency == 10)
        fr = "10Hz";
    else if (frequency == 20)
        fr = "20Hz";
    else if (frequency == 30)
        fr = "30Hz";
    else
        fr = "unknown";
    return fr;
}

QDebug operator<<(QDebug dbg, const ExperimentConfig &ex)
{
    dbg << "id" << ex.id
        << "ghost" << command.ghostMode
        << "map" << ex.getMapName()
        << "agents" << ex.agents
        << "controller" << ex.getControllerName()
        << "heuristic" << ex.getHeuristicName()
        << "pred" << ex.getPredictionType()
        << "traj" << ex.getTrajectoryType()
        << "freq" << ex.getFreqName();
        //<< "score:" << ex.score
        //<< "cols:" << ex.collisions;
    return dbg;
}

QDebug operator<<(QDebug dbg, const ExperimentConfig* ex)
{
    dbg << "id:" << ex->id
        << "ghost:" << command.ghostMode
        << "score:" << ex->score
        << "collisions:" << ex->collisions
        << "pathTime:" << ex->pathTime
        << "trajTime:" << ex->trajectoryTime
        << "stucks:" << ex->stucks;
        //<< "closes:" << ex->closes
        //<< "staticpathfails:" << ex->staticPathFails
        //<< "dynamicpathfails:" << ex->dynamicPathFails
        //<< "trajectoryfails:" << ex->trajectoryFails;
    return dbg;
}

QDataStream& operator<<(QDataStream& out, const ExperimentConfig &ex)
{
    out << ex.id;
    out << ex.seed;
    out << ex.runs;
    out << ex.frames;
    out << ex.map;
    out << ex.agents;
    out << ex.trajectoryPlanningMethod;
    out << ex.trajectoryType;
    out << ex.heuristic;
    out << ex.predict;
    out << ex.frequency;

    out << ex.score;
    out << ex.collisions;
    out << ex.pathTime;
    out << ex.trajectoryTime;
    out << ex.v;
    out << ex.w;
    out << ex.jerk;
    out << ex.stucks;
    out << ex.closes;
    out << ex.staticPathFails;
    out << ex.dynamicPathFails;
    out << ex.trajectoryFails;
    return out;
}

QDataStream& operator>>(QDataStream& in, ExperimentConfig &ex)
{
    in >> ex.id;
    in >> ex.seed;
    in >> ex.runs;
    in >> ex.frames;
    in >> ex.map;
    in >> ex.agents;
    in >> ex.trajectoryPlanningMethod;
    in >> ex.trajectoryType;
    in >> ex.heuristic;
    in >> ex.predict;
    in >> ex.frequency;

    in >> ex.score;
    in >> ex.collisions;
    in >> ex.pathTime;
    in >> ex.trajectoryTime;
    in >> ex.v;
    in >> ex.w;
    in >> ex.jerk;
    in >> ex.stucks;
    in >> ex.closes;
    in >> ex.staticPathFails;
    in >> ex.dynamicPathFails;
    in >> ex.trajectoryFails;
    return in;
}

QTextStream& operator<<(QTextStream& out, const ExperimentConfig &ex)
{
    out << " id " << ex.id
        << " ghost " << command.ghostMode
        << " map " << ex.getMapName()
        << " agents " << ex.agents
        << " controller " << ex.getControllerName()
        << " heuristic " << ex.getHeuristicName()
        << " pred " << ex.getPredictionType()
        << " traj " << ex.getTrajectoryType()
        << " freq " << ex.frequency;
//        << " score: " << ex.score // 16
//        << " collisions: " << ex.collisions // 19
//        << " stucks: " << ex.stucks
//        << " pathTime: " << ex.pathTime
//        << " trajTime: " << ex.trajectoryTime;
        //<< " closes: " << ex.closes
        //<< " pathfails: " << ex.staticPathFails
        //<< " trajectoryfails: " << ex.trajectoryFails;
    return out;
}

QTextStream& operator<<(QTextStream& out, const ExperimentConfig* ex)
{
    out << " id: " << ex->id
        << " seed: " << ex->seed
        << " ghost:" << command.ghostMode
        << " map: " << ex->getMapName()
        << " agents: " << ex->agents
        << " controller: " << ex->getControllerName()
        << " heuristic: " << ex->getHeuristicName()
        << " pred: " << ex->getPredictionType()
        << " traj: " << ex->getTrajectoryType()
        << " freq: " << ex->frequency
        << " score: " << ex->score // 16
        << " collisions: " << ex->collisions // 19
        << " stucks: " << ex->stucks
        << " pathTime: " << ex->pathTime
        << " trajTime: " << ex->trajectoryTime;
        //<< " closes: " << ex.closes
        //<< " pathfails: " << ex.staticPathFails
        //<< " trajectoryfails: " << ex.trajectoryFails;
    return out;
}

// Streams the content of the Vector into the QTextStream.
QTextStream& operator<<(QTextStream& out, const QVector<ExperimentConfig> &o)
{
    for (uint i = 0; i < o.size(); i++)
        out << o[i] << "\n";
    return out;
}

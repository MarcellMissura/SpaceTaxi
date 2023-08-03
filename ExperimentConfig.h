#ifndef EXPERIMENTCONFIG_H_
#define EXPERIMENTCONFIG_H_
#include <QString>
#include "lib/util/VecN.h"

struct ExperimentConfig
{
    uint id;
    uint seed;
    uint runs;
    uint frames;
    uint map;
    uint agents;
    uint trajectoryPlanningMethod;
    uint trajectoryType;
    uint heuristic;
    uint predict; // 0 - no predict, 1 - linear, 2 - unicycle
    uint frequency; // 10, 20, 30 Hz.

    VecN<4> score; // min, mean, stddev, max
    VecN<4> collisions;
    VecN<4> pathTime;
    VecN<4> trajectoryTime;
    VecN<4> expansions;
    VecN<4> v;
    VecN<4> w;
    double jerk;
    uint stucks;
    uint closes;
    uint staticPathFails;
    uint dynamicPathFails;
    uint trajectoryFails;

    ExperimentConfig()
    {
        id = 0;
        seed = 0;
        frames = 0;
        map = 0;
        agents = 1;
        trajectoryPlanningMethod = 0;
        trajectoryType = 0;
        heuristic = 0;
        predict = 1;
        frequency = 30;

        jerk = 0;
        stucks = 0;
        closes = 0;
        staticPathFails = 0;
        dynamicPathFails = 0;
        trajectoryFails = 0;
    }
    ~ExperimentConfig(){}

    QString getMapName() const;
    QString getFileName() const;
    QString getControllerName() const;
    QString getTrajectoryType() const;
    QString getHeuristicName() const;
    QString getPredictionType() const;
    QString getFreqName() const;
};

QDebug operator<<(QDebug dbg, const ExperimentConfig &ex);
QDebug operator<<(QDebug dbg, const ExperimentConfig* ex);
QDataStream& operator<<(QDataStream& out, const ExperimentConfig &ex);
QDataStream& operator>>(QDataStream& in, ExperimentConfig &ex);
QTextStream& operator<<(QTextStream& out, const ExperimentConfig &ex);
QTextStream& operator<<(QTextStream& out, const ExperimentConfig* ex);
QTextStream& operator<<(QTextStream& out, const QVector<ExperimentConfig> &o);

#endif

#include "Experimenter.h"
#include "globals.h"
#include "blackboard/State.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"
#include "util/Logger.h"
#include "util/StopWatch.h"
#include "util/Statistics.h"
#include "geometry/Box.h"
#include "pml/poc.h"
#include "pml/lip.h"
#include <QtConcurrent/QtConcurrent>

// Fitness experiment
// ghost agents pd,dwa,staa* (top final 30Hz),
// smart agents pd,dwa,staa* (kitkat 90% 30Hz)
//
// The exepriment confirmed that STAA* is much better than
// PD and better than DWA. Cooperative DWA is almost able
// to avoid all collisions, perhaps can be tuned to do so.
// Cooperative STAA avoids all collisions solidly so that
// they become very rare. In ghost mode, STAA reduces more
// collisions than DWA.

// Heuristic experiment
// ghost agents patheuk, pathrtr, dijk (top final 30Hz)
// smart agents patheuk, pathrtr, dijk (laddoo 10% 30Hz)
//
// The MC Path RTR heuristic came out best in the heuristic
// experiment. The Dijkstra rtr should have been the same
// as good, except that computing the Dijkstra table is too
// time consuming and there is not enough time left to plan.
// I saw a clear increase of the Dijstra RTR fitness when
// switching from 30Hz to 10Hz. I am surprised how well the
// path euklidean heuristic works, even though it drives
// backwards quite a lot.

// Prediction experiment
// ghost agents uni,holo,none (final 10Hz)
// smart agents uni,holo,none (messed up at 10 Hz)
//
// The experiment confirmed that prediction does make sense
// and the number of collisions is reduced. I wasn't able to
// show a difference between holonomic and unicycle prediction
// though. Even without prediction, cooperation improves the
// performance by way more than the predictions alone.


// Trajectory experiment
// ghost agents arc,b0,fresnel (final 10Hz)
// smart agents arc,b0,fresnel (final 10Hz, bad at 10 Hz)
//
// I was not able to show a difference in quality between
// the Arc, B0, and Fresnel approximations neither at 30Hz
// nor at 10Hz. Arc is the fastest and simplest so let's use that.


// Frequency experiment
// ghost agents 10,20,30 (infcuda2 0% 10Hz)
// smart agents 10,20,30 (infcuda 0% 10Hz, messsed up)
//
// Even though at 10Hz the STAA has more time to compute,
// it seems to make no difference whether it is run at
// 10Hz, 20Hz or 30Hz.

// Assets:
// bignut (idle)
// raffaello (idle)
// rocher (idle)
// laddoo (idle)
// kestrel (idle)
// kitkat (idle)
// infcuda
// infcuda2

// Isolated:
// Prediction type - holo or uni makes no difference (just a tiny bit)
// Trajectory type - don't know yet
// Heuristic - don't know yet
// Lowest h or last opened?
// Dynamic path on and off (and predicted path?)

// COMBOS TO TRY:

// 0. all on: runs on kestrel with some errors in 12 hours.
// 1. minus stuckness: running on raffaello
// 2. minus prediction
// 3. minus dynamic path
// 4. minus force field and proximity
// 5. bestScoreNode / best heuristic node (STAA* only)
// 6. minus predictive path planning (STAA* only)
// 7. pre or post collision check (STAA* only): pre collision check works much better
// 8. proximity weight 1.0 or 0.5 (STAA* only)

// PD control: path -> dynamic path -> force field -> stuckness
// DWA: path -> dynamic path -> predicted cc -> force field -> stuckness
// STAA: path -> dynamic path -> predicted path -> predicted cc -> force field -> stuckness

// TODOS:

// hunt down worst cases
// evaluate the obstacle prediction model
// evaluate the motion prediction models (arc, b0, fresnel)
// evaluate the heuristic
// tetest

Experimenter::Experimenter() : QThread(NULL)
{
    running = false;
}

void Experimenter::init()
{
    state.init();
    config.init();
    config.load("config");
    state.world.init();
}

void Experimenter::run()
{
    Statistics::init(); // The random number generator has to be initialized in every thread.

    emit messageOut("Experimenter started.");
    qDebug() << "Experimenter started.";
    running = true;

    trajectoryExperiments();
    postProcessTrajectoryExperiments();
    //geometricModelTest();
    //geometryRuntimeTests();
    //arithmeticOperationsTest();
    //lineTests();
    //pocPredictionTest();

    running = false;
    emit messageOut("Experimenter finished.");
    qDebug() << "Experimenter finished.";
}

void Experimenter::trajectoryExperiments()
{
    // Set up the configuration of the experiment.
    // Number of frames.
    // Number of agents.
    // The map.
    // Controller: pd, dwa, staa*
    // Trajectory type.
    // Heuristic (grid or graph, path length or path rtr)
    // Prediction type: holonomic, unicycle or none.
    const static uint threads = 9;
    int runs = 20; // How many runs to run.
    int frames = 10000; // Number of frames per run to run a case on (at 30Hz)

    generateCases(runs, frames);
    //loadCases();

    progressLogger.open("data/progress.log");

    // Run all the cases.

    // Sequential.
//    for (uint i = 0; i < cases.size(); i++)
//        processCase(cases[i]);

    // Threaded.
    Vector<QFuture<void> > results(threads);
    uint caseIndex = 0;
    while (true)
    {
        int activeThreads = threads;
        for (uint i = 0; i < threads; i++)
        {
            if (results[i].isFinished())
            {
                if (caseIndex < cases.size())
                {
                    saveCases();

                    qDebug() << "Start thread" << i << "case" << cases[caseIndex];
                    progressLogger << "Start thread" << i << "on case" << cases[caseIndex]
                        << "seed:" << cases[caseIndex].seed;
                    progressLogger++;
                    results[i] = QtConcurrent::run(this, &Experimenter::processCase, cases[caseIndex]);
                    caseIndex++;
                }
                else
                {
                    activeThreads--;
                }
            }
        }

        if (activeThreads == 0)
            break;

        sleep(1);
    }

    progressLogger << "All cases finished. Final save of data/cases.dat.";
    progressLogger++;
    saveCases();
}

// Create a Vector of ExperimentConfig objects for all combinations of the parameters.
void Experimenter::generateCases(int runs, int frames)
{
    Vector<uint> agents; // Number of agents to try.
    agents << 1 << 2 << 3 << 4 << 5;
    Vector<uint> maps; // Maps to try.
    maps << 4 << 6 << 7; // 0 void, 1 simple, 2 U, 3 outdoor, 4 apartment, 5 warehouse, 6 office, 7 clutter
    Vector<uint> controllers; // Types of controllers to try.
    //controllers << command.PD << command.DWA << command.STAA;
    controllers << command.STAA;
    Vector<uint> heuristics; // Types of controllers to try.
    heuristics << command.MinimalConstruct;// << command.GridDijkstra << command.PathEuklidean;
    Vector<uint> trajectoryTypes; // Trajectory types to try.
    trajectoryTypes << command.Arc << command.B0 << command.Fresnel;
    Vector<uint> predictionTypes; // Prediction types to try.
    predictionTypes << command.Unicycle;// << command.Holonomic << command.None;
    Vector<uint> frequencies; // Frequencies to try.
    frequencies << 10;//30 << 20 << 10;

    cases.clear();
    uint counter = 0;
    for (uint i = 0; i < maps.size(); i++)
    {
        for (uint l = 0; l < agents.size(); l++)
        {
            uint seed = Statistics::randomInt();
            for (uint j = 0; j < controllers.size(); j++)
            {
                for (uint k = 0; k < heuristics.size(); k++)
                {
                    for (uint t = 0; t < trajectoryTypes.size(); t++)
                    {
                        for (uint p = 0; p < predictionTypes.size(); p++)
                        {
                            for (uint f = 0; f < frequencies.size(); f++)
                            {
                                ExperimentConfig ex;
                                ex.id = counter++;
                                ex.seed = seed;//Statistics::randomInt();
                                ex.runs = runs;
                                ex.map = maps[i];
                                ex.agents = agents[l];
                                ex.trajectoryPlanningMethod = controllers[j];
                                ex.trajectoryType = trajectoryTypes[t];
                                ex.heuristic = heuristics[k];
                                ex.predict = predictionTypes[p];
                                ex.frequency = frequencies[f];
                                ex.frames = ex.frequency*333;
                                cases << ex;
                            }
                        }
                    }
                }
            }
        }
    }

    qDebug() << cases.size() << "cases generated.";
}

void Experimenter::processCase(ExperimentConfig &ex)
{
    // Case filter.

//    if (ex.id != 38)
//    {
//        qDebug() << "Skipping case" << ex.id << ex.getMapName();
//        progressLogger << "Skipping case" << ex.id << ex.getMapName();
//        progressLogger++;
//        return;
//    }

//    // Skip void, simple, and warehouse.
//    if (ex.map == 0 || ex.map == 1 || ex.map == 5)
//    {
//        qDebug() << "Skipping case" << ex.id << ex.getMapName();
//        progressLogger << "Skipping case" << ex.id << ex.getMapName();
//        progressLogger++;
//        return;
//    }

//    // Skip non STAA
//    if (!ex.score.isNull() && ex.trajectoryPlanningMethod != command.STAA)
//    {
//        qDebug() << "Skipping non STAA case" << ex.id << ex.getMapName() << ex.agents << ex.getControllerName();
//        progressLogger << "Skipping non STAA case" << ex.id << ex.getMapName() << ex.agents << ex.getControllerName();
//        progressLogger++;
//        return;
//    }

    // Skip finished cases.
    if (!ex.score.isNull())
    {
        qDebug() << "Skipping finished case" << ex.id << ex.getMapName() << ex.agents << ex.getControllerName();
        progressLogger << "Skipping finished case" << ex.id << ex.getMapName() << ex.agents << ex.getControllerName();
        progressLogger++;
        return;
    }

    // Set up structures for the data collection.
    Vector<int> scores;
    Vector<int> collisions;
    Vector<double> pathTimes;
    Vector<double> trajectoryTimes;
    Vector<int> expansions;
    uint staticPathFails = 0;
    uint dynamicPathFails = 0;
    uint trajectoryFails = 0;
    uint stucks = 0;
    uint closes = 0;

    // Initialize the world.
    World world;
    world.init();

    Statistics::setSeed(ex.seed); // Set the seed so that the case can be reproduced.

    for (uint r = 1; r <= ex.runs; r++)
    {
        // Reset the map. Set up the environment parameters according to the case.
        //double check = Statistics::randomNumber();
        //qDebug() << " Starting run" << r << "case" << ex.id << "seed:" << ex.seed << "check:" << check;
        //progressLogger << " Starting run" << r << "case" << ex.id << "seed:" << ex.seed << "check:" << check;
        //progressLogger++;
        world.setMap(ex.map, ex.agents);
        world.setParams(ex.trajectoryPlanningMethod, ex.trajectoryType, ex.predict, ex.heuristic, ex.frequency);

        Vector<UnicycleAgent> agentsAtStart = world.unicycleAgents;
        //qDebug() << "Run" << r << "case" << ex.id << "seed:" << ex.seed << "unicycles at start:" << agentsAtStart;

        // Iterate through the frames.
        for (int k = 0; k < ex.frames; k++)
        {
            world.step(); // THE STEP
            //qDebug() << "frame" << k << "score:" << world.unicycleAgents[0].score;

            // Log collided frames.
//            if (ex.trajectoryPlanningMethod == command.STAA && world.unicycleAgents[0].inCollision)
//            {
//                qDebug() << "  collision in case" << ex << "run" << r << "frame" << k;
//                qDebug() << "    agents:" << agentsAtStart;
//                logger << "   collision in case" << ex.id << ex.getMapName() << ex.agents << ex.getControllerName() << "run" << r << "frame" << k;
//                logger++;
//                for (uint i = 0; i < agentsAtStart.size(); i++)
//                {
//                    logger << "    " << i << agentsAtStart[i].getName() << "pose:" << agentsAtStart[i].pose();
//                    logger++;
//                }
//            }

            // Log static path failed frames.
            if (!world.unicycleAgents[0].staticWorldPathSuccess)
            {
                qDebug() << "  Static path fail in case" << ex << "run" << r << "frame" << k << "seed:" << ex.seed;
                qDebug() << "    agents right now:" << world.unicycleAgents;
                progressLogger << "   static path fail case" << ex << "run" << r << "frame" << k << "seed:" << ex.seed;
                progressLogger++;
            }

            pathTimes << world.unicycleAgents[0].pathTime;
            trajectoryTimes << world.unicycleAgents[0].trajectoryTime;
            expansions << world.unicycleAgents[0].shortTermAbortingAStar.expansions;

            if (!world.unicycleAgents[0].staticWorldPathSuccess)
                staticPathFails++;

            if (!world.unicycleAgents[0].dynamicPathSuccess)
                dynamicPathFails++;

            if (!world.unicycleAgents[0].trajectorySuccess)
                trajectoryFails++;
        }

        // In smart mode, all agents do the same so we can use all their data.
        uint iMax = world.unicycleAgents.size();

        // In ghost mode, we only collect the data from the first agent.
        if (command.ghostMode)
            iMax = 1;

        for (uint i = 0; i < iMax; i++)
        {
            // Log and skip dried runs.
            int driedQLimit = 200;
            if (command.ghostMode)
                driedQLimit = 1000;
            if (world.unicycleAgents[i].shortTermAbortingAStar.dried > driedQLimit)
            {
                qDebug() << "  case" << ex.id << "run" << r << "agent" << i << "finished with dried Q:" << world.unicycleAgents[i].shortTermAbortingAStar.dried << ex
                         << "stucks:" <<  world.unicycleAgents[i].stucks << "score:" << world.unicycleAgents[i].score << "cols:" << world.unicycleAgents[i].collisions;
                progressLogger << "  case" << ex.id << "run" << r << "agent" << i << "finished with dried Q:" << world.unicycleAgents[i].shortTermAbortingAStar.dried << ex
                         << "stucks:" <<  world.unicycleAgents[i].stucks << "score:" << world.unicycleAgents[i].score << "cols:" << world.unicycleAgents[i].collisions;
                progressLogger++;
                continue;
            }

            // Log and skip messed up collision runs.
            if (world.unicycleAgents[i].collisions > 100)
            {
                qDebug() << "  case" << ex.id << "run" << r << "agent:" << i << "finished with too many collisions. " << ex
                         << "stucks:" <<  world.unicycleAgents[i].stucks << "score:" << world.unicycleAgents[i].score << "cols:" << world.unicycleAgents[i].collisions;
                progressLogger << "  case" << ex.id << "agent" << i << "run" << r << "finished with too many collisions:" << world.unicycleAgents[i].collisions << ex
                         << "stucks:" <<  world.unicycleAgents[i].stucks << "score:" << world.unicycleAgents[i].score << "cols:" << world.unicycleAgents[i].collisions;
                progressLogger++;
                continue;
            }

            // Log and skip messed up low score runs.
            if (world.unicycleAgents[i].score < 5)
            {
                qDebug() << "  case" << ex.id << "run" << r << "agent" << i << "finished with low score." << ex
                         << "stucks:" <<  world.unicycleAgents[i].stucks << "score:" << world.unicycleAgents[i].score << "cols:" << world.unicycleAgents[i].collisions;
                progressLogger << "  case" << ex.id << "run" << r << "agent" << i << "finished with low score." << ex
                         << "stucks:" <<  world.unicycleAgents[i].stucks << "score:" << world.unicycleAgents[i].score << "cols:" << world.unicycleAgents[i].collisions;
                progressLogger++;
                continue;
            }

            // Log and skip messed up stucked runs.
            if (world.unicycleAgents[i].stucks > 20)
            {
                qDebug() << "  case" << ex.id << "run" << r << "agent" << i << "finished with too many stucks." << ex
                         << "stucks:" <<  world.unicycleAgents[i].stucks << "score:" << world.unicycleAgents[i].score << "cols:" << world.unicycleAgents[i].collisions;
                progressLogger << "  case" << ex.id << "run" << r << "agent:" << i << "finished with too many stucks." << ex
                         << "stucks:" <<  world.unicycleAgents[i].stucks << "score:" << world.unicycleAgents[i].score << "cols:" << world.unicycleAgents[i].collisions;
                progressLogger++;
                continue;
            }

            qDebug() << "  case" << ex.id << "run" << r << "agent" << i << "finished." << ex
                        << "stucks:" <<  world.unicycleAgents[i].stucks << "score:" << world.unicycleAgents[i].score << "cols:" << world.unicycleAgents[i].collisions;
            progressLogger << "  case" << ex.id << "run" << r << "agent" << i << "finished." << ex
                     << "stucks:" <<  world.unicycleAgents[i].stucks << "score:" << world.unicycleAgents[i].score << "cols:" << world.unicycleAgents[i].collisions;
            progressLogger++;

            scores << world.unicycleAgents[i].score;
            collisions << world.unicycleAgents[i].collisions;
            stucks += world.unicycleAgents[i].stucks;
            closes += world.unicycleAgents[i].closes;
        }
    }

    if (scores.size() < 10)
    {
        qDebug() << "case" << ex << "has too few completed runs." << scores.size();
        progressLogger << "case" << ex.id << ex.getMapName() << ex.agents << ex.getControllerName() << "has too few completed runs." << scores.size();
        progressLogger++;
    }

    // Transfer / compute statistics into the case.
    ex.score = Statistics::minmeanstddevmax(scores);
    ex.collisions = Statistics::minmeanstddevmax(collisions);
    ex.expansions = Statistics::minmeanstddevmax(expansions);
    ex.pathTime = Statistics::minmeanstddevmax(pathTimes);
    ex.trajectoryTime = Statistics::minmeanstddevmax(trajectoryTimes);
    ex.stucks = stucks;
    ex.closes = closes;
    ex.staticPathFails = staticPathFails;
    ex.dynamicPathFails = dynamicPathFails;
    ex.trajectoryFails = trajectoryFails;

    // Save the case in memory.
    cases[ex.id] = ex;
    saveCases();

    // Human readable progress output:
    qDebug() << "Finished with case" << &cases[ex.id];
    progressLogger << "Finished with case" << &cases[ex.id];
    progressLogger++;
}

// Save all experimental data in a binary and a text file.
void Experimenter::saveCases()
{
    // Save all experimental data in a binary file.
    QFile binaryFile("data/cases.dat");
    binaryFile.open(QFile::WriteOnly);
    QDataStream binout(&binaryFile);
    binout << cases;
    binaryFile.close();

    // Save all experimental data in a text file.
    QFile textFile("data/experiments.log");
    textFile.open(QFile::WriteOnly | QFile::Text);
    QTextStream txtout(&textFile);
    txtout << cases;
    textFile.close();
}

// Generates formatted data files for plotting from the experimental results.
void Experimenter::loadCases()
{
    QFile file("data/cases.dat");
    file.open(QIODevice::ReadOnly);
    QDataStream in(&file);
    in >> cases;
    file.close();
    qDebug() << cases.size() << "cases loaded.";
}

void Experimenter::postProcessTrajectoryExperiments()
{
    // Load the cases from file.
    loadCases();

    // File format used for gnuplot:
    // agents, PD, DWA_Arc, MTAA*_Arc, STAA*_Arc

    Logger logger;

    // Score min, mean, stddev, max
    for (uint k = 0; k < cases.size(); k++)
    {
        ExperimentConfig &ex = cases[k];

        if (k == 0 || cases[k].map != cases[k-1].map)
        {
            //qDebug() << "opening" << QString("plots/" + cases[k].getMapName() + "_score.txt");
            logger.open("plots/" + cases[k].getMapName() + "_score.txt");
        }

        if (k == 0 || cases[k].agents != cases[k-1].agents)
        {
            logger++;
            logger << ex.agents;
        }
        logger << ex.score;
    }
    logger++;

    // Collisions min, mean, stddev, max
    for (uint k = 0; k < cases.size(); k++)
    {
        ExperimentConfig &ex = cases[k];

        if (k == 0 || cases[k].map != cases[k-1].map)
            logger.open("plots/" + cases[k].getMapName() + "_cols.txt");

        if (k == 0 || cases[k].agents != cases[k-1].agents)
        {
            logger++;
            logger << ex.agents;
        }
        logger << ex.collisions;
    }
    logger++;

    // Expansions min, mean, stddev, max
    for (uint k = 0; k < cases.size(); k++)
    {
        ExperimentConfig &ex = cases[k];

        if (k == 0 || cases[k].map != cases[k-1].map)
            logger.open("plots/" + cases[k].getMapName() + "_exps.txt");

        if (k == 0 || cases[k].agents != cases[k-1].agents)
        {
            logger++;
            logger << ex.agents;
        }
        logger << ex.expansions;
    }
    logger++;


    // Path time min, mean, stddev, max
    for (uint k = 0; k < cases.size(); k++)
    {
        ExperimentConfig &ex = cases[k];

        if (k == 0 || cases[k].map != cases[k-1].map)
            logger.open("plots/" + cases[k].getMapName() + "_pathtime.txt");

        if (k == 0 || cases[k].agents != cases[k-1].agents)
        {
            logger++;
            logger << ex.agents;
        }
        logger << ex.pathTime;
    }
    logger++;

    // Trajectory time min, mean, stddev, max
    for (uint k = 0; k < cases.size(); k++)
    {
        ExperimentConfig &ex = cases[k];

        if (k == 0 || cases[k].map != cases[k-1].map)
            logger.open("plots/" + cases[k].getMapName() + "_trajectorytime.txt");

        if (k == 0 || cases[k].agents != cases[k-1].agents)
        {
            logger++;
            logger << ex.agents;
        }
        logger << ex.trajectoryTime;
    }
    logger++;

    // v min, mean, stddev, max
    for (uint k = 0; k < cases.size(); k++)
    {
        ExperimentConfig &ex = cases[k];

        if (k == 0 || cases[k].map != cases[k-1].map)
            logger.open("plots/" + cases[k].getMapName() + "_v.txt");

        if (k == 0 || cases[k].agents != cases[k-1].agents)
        {
            logger++;
            logger << ex.agents;
        }
        logger << ex.v;
    }
    logger++;

    // w min, mean, stddev, max
    for (uint k = 0; k < cases.size(); k++)
    {
        ExperimentConfig &ex = cases[k];

        if (k == 0 || cases[k].map != cases[k-1].map)
            logger.open("plots/" + cases[k].getMapName() + "_w.txt");

        if (k == 0 || cases[k].agents != cases[k-1].agents)
        {
            logger++;
            logger << ex.agents;
        }
        logger << ex.w;
    }
    logger++;

    // Jerk
    for (uint k = 0; k < cases.size(); k++)
    {
        ExperimentConfig &ex = cases[k];

        if (k == 0 || cases[k].map != cases[k-1].map)
            logger.open("plots/" + cases[k].getMapName() + "_jerk.txt");

        if (k == 0 || cases[k].agents != cases[k-1].agents)
        {
            logger++;
            logger << ex.agents;
        }
        logger << ex.jerk;
    }
    logger++;

    // Stucks
    for (uint k = 0; k < cases.size(); k++)
    {
        ExperimentConfig &ex = cases[k];

        if (k == 0 || cases[k].map != cases[k-1].map)
            logger.open("plots/" + cases[k].getMapName() + "_stucks.txt");

        if (k == 0 || cases[k].agents != cases[k-1].agents)
        {
            logger++;
            logger << ex.agents;
        }
        logger << ex.stucks;
    }
    logger++;

    // Closes
    for (uint k = 0; k < cases.size(); k++)
    {
        ExperimentConfig &ex = cases[k];

        if (k == 0 || cases[k].map != cases[k-1].map)
            logger.open("plots/" + cases[k].getMapName() + "_closes.txt");

        if (k == 0 || cases[k].agents != cases[k-1].agents)
        {
            logger++;
            logger << ex.agents;
        }
        logger << ex.closes;
    }
    logger++;


    // Tell us about path fails.
    for (uint i = 0; i < cases.size(); i++)
        if (cases[i].staticPathFails > 0)
            qDebug() << "static path fail case" << i << "fails:" << cases[i].staticPathFails << cases[i];

    // Tell us about dynamic path fails.
//    for (uint i = 0; i < cases.size(); i++)
//        if (cases[i].dynamicPathFails > 0)
//            qDebug() << "dynamic path fail case " << i << "fails:" << cases[i].dynamicPathFails << cases[i];

    // Tell us about trajectory fails.
    for (uint i = 0; i < cases.size(); i++)
        if (cases[i].trajectoryFails > 0)
            qDebug() << "trajectory fail case" << i << "fails:" << cases[i].trajectoryFails << cases[i];

    return;
}


void Experimenter::geometricModelTest()
{
    const uint iterations = 100000;

    GridModel sensedGrid;
    GridModel dilatedSensedGrid;
    GeometricModel staticGeometricModel;
    GeometricModel expandedStaticGeometricModel;
    DynamicGeometricModel dynamicGeometricModel;
    DynamicGeometricModel expandedDynamicGeometricModel;
    GeometricModel unifiedGeometricModel;

    sensedGrid.setDim(2);
    sensedGrid.setN(Vec2u(config.gridHeight/config.gridCellSize+1, config.gridWidth/config.gridCellSize+1));
    sensedGrid.setMin(Vec2(-config.gridHeight/2+config.gridOffset, -config.gridWidth/2));
    sensedGrid.setMax(Vec2(config.gridHeight/2+config.gridOffset, config.gridWidth/2));
    sensedGrid.init();
    dilatedSensedGrid = sensedGrid;

    World world;
    world.init();
    world.setMap(4, 5);
    Pose2D pose = world.getUnicycleObstacles()[0].pose();

    StopWatch sw;
    sw.start();

    GeometricModel transformedWorldPolygons;
    transformedWorldPolygons.setObstacles(world.getStaticObstacles());
    transformedWorldPolygons -= pose; // Transform from world to local coordinates.
    transformedWorldPolygons.transform();
    sensedGrid.computeOccupancyGrid(transformedWorldPolygons.getObstacles());
    staticGeometricModel.setFromGrid(sensedGrid);
    dilatedSensedGrid = sensedGrid;
    dilatedSensedGrid.dilate(config.gridSensedDilationRadius);
    expandedStaticGeometricModel.setFromGrid(dilatedSensedGrid);
    dilatedSensedGrid.dilate(config.gridBlurRadius);
    dilatedSensedGrid.blur(config.gridBlurRadius);

    // Compute the dynamic geometric model from the unicycle agents in the world.
    // Only the ones located in the area of the sensed grid are added to the model.
    dynamicGeometricModel.clear();
    expandedDynamicGeometricModel.clear();
    Vector<UnicycleObstacle> uo = world.getUnicycleObstacles(world.getUnicycleObstacles()[0].getId());
    for (uint i = 0; i < uo.size(); i++)
    {
        uo[i] -= pose; // Transform from world to local coordinates.
        if (sensedGrid.contains(uo[i].pos()))
        {
            expandedDynamicGeometricModel.addObstacle(uo[i]);
            dynamicGeometricModel.addObstacle(uo[i]);
        }
    }
    dynamicGeometricModel.transform(); // why?
    expandedDynamicGeometricModel.grow(0.4); // less growth for STAA*
    expandedDynamicGeometricModel.transform(); // why?


    unifiedGeometricModel.clear();
    unifiedGeometricModel.setObstacles(world.getStaticObstacles());
    unifiedGeometricModel -= pose; // Convert to local coordinates.
    unifiedGeometricModel.transform();
    unifiedGeometricModel.addObstacles(expandedStaticGeometricModel.getObstacles());

    double time = sw.elapsedTimeMs();
    qDebug() << "Sensing:" << time;


    qDebug() << "Spawning" << iterations << "polygons.";

    // Generate random points.
    Vector<Vec2> points(iterations);
    for (int i = 0; i < iterations; i++)
    {
        double x = Statistics::uniformSample(0, world.width);
        double y = Statistics::uniformSample(0, world.height);
        points[i].set(x,y);
    }

    // Generate random agent polygons.
    Vector<Polygon> polygons;
    for (int i = 0; i < iterations; i++)
    {
        // Set up the polygon that describes the agent.
        double w = 0.5*config.agentWidth;
        double h = 0.5*config.agentHeight;
        Polygon p;
        p.addVertex(Vec2(-w, h));
        p.addVertex(Vec2(-w, -h));
        p.addVertex(Vec2(w, -0.8*h));
        p.addVertex(Vec2(w, 0.8*h));
        p.setConvex();
        p.setPos(points[i]);
        p.setRotation(Statistics::uniformSample(-PI, PI));
        //p.transform();
        polygons << p;
    }

    // Generate random unicycles.
    Vector<Unicycle> unicycles;
    for (int i = 0; i < iterations; i++)
    {
        Unicycle u;
        u.setPose(polygons[i].pose());
        u.a = 1;
        u.b = 1;
        u.dt = 0.5;
        unicycles << u;
    }

    uint collided = 0;

    // Point collision check in the occupancy grid.
    sw.start();
    for (uint i = 0; i < iterations; i++)
        if (sensedGrid.contains(points[i]) && sensedGrid.isOccupied(points[i]))
            collided++;
    time = sw.elapsedTimeMs();
    qDebug() << "Point in grid check:" << time/iterations << "ms";

    // Polygon collision check in the occupancy grid.
    sw.start();
    for (uint i = 0; i < iterations; i++)
        if (sensedGrid.polygonCollisionCheck(polygons[i]))
            collided++;
    time = sw.elapsedTimeMs();
    qDebug() << "Polygon in grid check:" << time/iterations << "ms";

    // Static point collision check with the unified model.
    sw.start();
    for (uint i = 0; i < iterations; i++)
        if (unifiedGeometricModel.pointCollisionCheck(points[i]) >= 0)
            collided++;
    time = sw.elapsedTimeMs();
    qDebug() << "Static point collision check in geometric model:" << time/iterations << "ms";

    // Static polygon collision check with the unified model.
    sw.start();
    for (uint i = 0; i < iterations; i++)
        if (unifiedGeometricModel.polygonCollisionCheck(polygons[i]) >= 0)
            collided++;
    time = sw.elapsedTimeMs();
    qDebug() << "Static polygon collision check in geometric model:" << time/iterations << "ms";

    // Dynamic point collision check with the dynamic model.
    sw.start();
    for (uint i = 0; i < iterations; i++)
        if (dynamicGeometricModel.dynamicPointCollisionCheck(points[i], 0.1) >= 0)
            collided++;
    time = sw.elapsedTimeMs();
    qDebug() << "Dynamic point collision check in geometric model:" << time/iterations << "ms";

    // Dynamic Polygon collision check with the dynamic model.
    sw.start();
    for (uint i = 0; i < iterations; i++)
        if (dynamicGeometricModel.dynamicPolygonCollisionCheck(polygons[i], 0.1) >= 0)
            collided++;
    time = sw.elapsedTimeMs();
    qDebug() << "Dynamic polygon check in geometric model:" << time/iterations << "ms";


    // Dynamic trajectory collision check.
    sw.start();
    for (uint i = 0; i < iterations; i++)
    {
        Collision col = dynamicGeometricModel.trajectoryCollisionCheck(unicycles[i]);
        if (col.dt >= 0)
            collided++;
    }
    time = sw.elapsedTimeMs();
    qDebug() << "Dynamic trajectory check in geometric model:" << time/iterations << "ms";

    // Path planning test.
    sw.start();
    for (uint i = 0; i < iterations-1; i++)
    {
        const Vector<Vec2>& pp = unifiedGeometricModel.computePath(points[i], points[i+1]);
        if (pp.isEmpty())
            collided++;
    }
    time = sw.elapsedTimeMs();
    qDebug() << "Path planning test in geometric model:" << time/(iterations-1) << "ms";

    sw.start();
    for (uint i = 0; i < iterations-1; i++)
    {
        const Vector<Vec2>& pp = dilatedSensedGrid.computePath(points[i], points[i+1]);
        if (pp.isEmpty())
            collided++;
    }
    time = sw.elapsedTimeMs();
    qDebug() << "Path planning test in grid model:" << time/(iterations-1) << "ms";
}


// Tests and prints the runtimes of geometric operations.
void Experimenter::geometryRuntimeTests()
{
    const int N = 10000;

    qDebug() << "Spawning" << N << "polygons.";

    // Generate N random points.
    QVector<Vec2> points(N);
    for (int i = 0; i < N; i++)
        points[i].randomize();

    // Genrate N random lines.
    QVector<Line> lines(N);
    for (int i = 0; i < N; i++)
    {
        Vec2 p1;
        Vec2 p2;
        p1.randomize();
        p2.randomize();
        lines[i].set(p1.x, p1.y, p2.x, p2.y);
    }

    // Generate N random boxes.
    QVector<Box> boxes(N);
    VecN<4> r;
    for (int i = 0; i < N; i++)
    {
        r.randomize();
        boxes[i].set(r[0], r[1], r[2], r[3]);
    }

    // Generate random polygons.
    QVector<Polygon> polygons;
    for (int i = 0; i < N; i++)
    {
        int points = Statistics::uniformSample(3, 16);
        Polygon p;
        p.setPos(Vec2::random());
        p.setRotation(Statistics::randomNumber());
        for (int j = 0; j < points; j++)
            p << Vec2::random();
        polygons << p;
    }

    StopWatch sw;

    // Transform computation test.
    sw.start();
    for (int i = 0; i < polygons.size(); i++)
    {
        polygons[i].transform();
    }
    double time = 1000.0*sw.elapsedTimeMs();
    qDebug() << "Transform computation time:" << time/polygons.size() << "microseconds.";

    // Bounding box computation test.
    sw.start();
    for (int i = 0; i < polygons.size(); i++)
    {
        Box box = polygons[i].boundingBox();
    }
    time = 1000.0*sw.elapsedTimeMs();
    qDebug() << "Bounding box computation time:" << time/polygons.size() << "microseconds.";

    // Point in Box test.
    sw.start();
    bool yes = false;
    for (int i = 0; i < points.size(); i++)
        for (int j = 0; j < boxes.size(); j++)
            yes = boxes[j].intersects(points[i]);
    time = 1000.0*sw.elapsedTimeMs();
    qDebug() << "Point in Box:" << time/(points.size()*boxes.size()) << "microseconds.";

    // Box vs Box test.
    sw.start();
    for (int i = 0; i < boxes.size(); i++)
        for (int j = 0; j < boxes.size(); j++)
            yes = boxes[j].intersects(boxes[i]);
    time = 1000.0*sw.elapsedTimeMs();
    qDebug() << "Box vs Box:" << time/(boxes.size()*boxes.size()) << "microseconds.";

    // Line vs Box test.
    sw.start();
    for (int i = 0; i < lines.size(); i++)
        for (int j = 0; j < lines.size(); j++)
            yes = lines[j].intersects(lines[i]);
    time = 1000.0*sw.elapsedTimeMs();
    qDebug() << "Line vs Line:" << time/(lines.size()*lines.size()) << "microseconds.";

    // Point in Polygon test.
    sw.start();
    for (int i = 0; i < points.size(); i++)
        for (int j = 0; j < polygons.size(); j++)
            yes = polygons[j].intersects(points[i]);
    time = 1000.0*sw.elapsedTimeMs();
    qDebug() << "Point vs Polygon test:" << time/(points.size()*polygons.size()) << "microseconds.";

    // Line with Polygon intersection test.
    sw.start();
    for (int i = 0; i < lines.size(); i++)
        for (int j = 0; j < polygons.size(); j++)
            yes = polygons[j].intersects(lines[i]);
    time = 1000.0*sw.elapsedTimeMs();
    qDebug() << "Line vs Polygon test:" << time/(lines.size()*polygons.size()) << "microseconds.";

    // Poylgon vs Polygon test.
    sw.start();
    for (int i = 0; i < polygons.size(); i++)
        for (int j = 0; j < polygons.size(); j++)
            yes = polygons[i].intersects(polygons[j]);
    time = 1000.0*sw.elapsedTimeMs();
    qDebug() << "Polygon vs Polygon:" << time/(polygons.size()*polygons.size()) << "microseconds.";
}

// This routine tests the efficiency of various container classes when filled with a large number of objects.
void Experimenter::containerTest()
{
    int l = 10000; // How many objects.
    Node s; // What object.
    StopWatch sw;


    qDebug() << "--- Payload:" << sizeof(Node)<< ", list size:" << l << "-------------";

    QVector<Node> qv;
    qv.reserve(l);
    sw.start();
    for (int k = 0; k < 1000; k++)
    {
        qv.clear();
        for (int i = 0; i < l; i++)
            qv << s;
    }
    qDebug() << "QVector:" << sw.elapsedTime() << "ms";

    Vector<Node> ve;
    ve.reserve(l);
    sw.start();
    for (int k = 0; k < 1000; k++)
    {
        ve.clear();
        for (int i = 0; i < l; i++)
            ve << s;
    }
    qDebug() << "Vector:" << sw.elapsedTime() << "ms";

    std::vector<Node> sv;
    sv.reserve(l);
    sw.start();
    for (int k = 0; k < 1000; k++)
    {
        sv.clear();
        for (int i = 0; i < l; i++)
            sv.push_back(s);
    }
    qDebug() << "std::vector:" << sw.elapsedTime() << "ms";

    QList<Node> ql;
    ql.reserve(l);
    sw.start();
    for (int k = 0; k < 1000; k++)
    {
        ql.clear();
        for (int i = 0; i < l; i++)
            ql << s;
    }
    qDebug() << "QList:" << sw.elapsedTime() << "ms";

    std::list<Node> sli;
    sli.resize(l);
    sw.start();
    for (int k = 0; k < 1000; k++)
    {
        sli.clear();
        for (int i = 0; i < l; i++)
            sli.push_back(s);
    }
    qDebug() << "std::list:" << sw.elapsedTime() << "ms";

    LinkedList<Node> lili;
    //lili.reserve(l);
    sw.start();
    for (int k = 0; k < 1000; k++)
    {
        lili.clear();
        for (int i = 0; i < l; i++)
            lili.push_back(s);
    }
    qDebug() << "LinkedList:" << sw.elapsedTime() << "ms";

    qDebug() << "Memset vs set zero:";
    double array[l];
    sw.start();
    for (int k = 0; k < 1000; k++)
        memset(array, 0, l*sizeof(double));
    qDebug() << "Memset:" << sw.elapsedTime() << "ms";

    sw.start();
    for (int k = 0; k < 1000; k++)
    {
        for (int i = 0; i < l; i++)
            array[i] = 0;
    }
    qDebug() << "Set zero:" << sw.elapsedTime() << "ms";
}

// Tortures the line intersection test with a number of mean cases like parallel lines
// and measure the runtimes of the line intersection test and left of test and tangential test.
void Experimenter::lineTests()
{
    // Easy cross.
    Line l1(0.0, 0.0, 1.0, 1.0);
    Line l2(0.0, 1.0, 1.0, 0.0);
    qDebug() << "Easy cross (yes)" << l1 << l2;
    qDebug() << "intersect:" << l1.intersects(l2) << l2.intersects(l1) << (l1.intersects(l2) ? "" : "FAIL!");
    //qDebug() << "point:" << l1.intersection(l2) << l2.intersection(l1);

    // Easy cross, no intersect.
    Line l3(0.0, 0.0, 1.0, 1.0);
    Line l4(0.0, 3.0, 1.0, 2.0);
    qDebug() << "Easy cross no intersect (no)" << l3 << l4;
    qDebug() << "intersect:" << l3.intersects(l4) << l4.intersects(l3) << (!l3.intersects(l4) ? "" : "FAIL!");
    //qDebug() << "point:" << l3.intersection(l4) << l4.intersection(l3);

    // Easy cross, end point touch.
    Line l5(0.0, 0.0, 1.0, 1.0);
    Line l6(0.0, 2.0, 1.0, 1.0);
    qDebug() << "Easy cross end point touch (no)" << l5 << l6;
    qDebug() << "intersect:" << l5.intersects(l6) << l6.intersects(l5) << (!l5.intersects(l6) ? "" : "FAIL!");
    //qDebug() << "point:" << l5.intersection(l6) << l6.intersection(l5);

    // Easy cross, middle touch.
    Line l51(0.0, 0.0, 1.0, 1.0);
    Line l61(0.0, 2.0, 2.0, 0.0);
    qDebug() << "Easy cross middle touch (yes)" << l51 << l61;
    qDebug() << "intersect:" << l51.intersects(l61) << l61.intersects(l51) << (l51.intersects(l61) ? "" : "FAIL!");
    //qDebug() << "point:" << l51.intersection(l61) << l61.intersection(l51);


    // Parallel horizontal disjunct lines example.
    Line l7(0.0, 0.0, 1.0, 0.0);
    Line l8(0.0, 1.0, 1.0, 1.0);
    qDebug() << "Parallel horizontal lines (no)" << l7 << l8;
    qDebug() << "intersect:" << l7.intersects(l8) << l8.intersects(l7) << (!l7.intersects(l8) ? "" : "FAIL!");
    //qDebug() << "point:" << l7.intersection(l8) << l8.intersection(l7);

    // Parallel vertical disjunct lines example.
    Line l9(0.0, 0.0, 0.0, 1.0);
    Line l10(1.0, 0.0, 1.0, 1.0);
    qDebug() << "Parallel vertical lines (no)" << l9 << l10;
    qDebug() << "intersect:" << l9.intersects(l10) << l10.intersects(l9) << (!l9.intersects(l10) ? "" : "FAIL!");
    //qDebug() << "point:" << l9.intersection(l10) << l10.intersection(l9);

    // Parallel diagonal disjunct lines example.
    Line l11(0.0, 0.0, 0.0, 1.0);
    Line l12(1.0, 0.0, 1.0, 1.0);
    l11.rotate(PI4);
    l12.rotate(PI4);
    qDebug() << "Parallel diagonal lines (no)" << l11 << l12;
    qDebug() << "intersect:" << l11.intersects(l12) << l12.intersects(l11) << (!l11.intersects(l12) ? "" : "FAIL!");
    //qDebug() << "point:" << l11.intersection(l12) << l12.intersection(l11);


    // Colinear horizontal disjunct lines example.
    Line l13(0.0, 0.0, 1.0, 0.0);
    Line l14(2.0, 0.0, 3.0, 0.0);
    qDebug() << "Colinear disjunct horizontal lines (no)" << l13 << l14;
    qDebug() << "intersect:" << l13.intersects(l14) << l14.intersects(l13) << (!l13.intersects(l14) ? "" : "FAIL!");
    //qDebug() << "point:" << l13.intersection(l14) << l14.intersection(l13);

    // Colinear vertical disjunct lines example.
    Line l15(0.0, 0.0, 0.0, 1.0);
    Line l16(0.0, 2.0, 0.0, 3.0);
    qDebug() << "Colinear disjunct vertical lines (no)" << l15 << l16;
    qDebug() << "intersect:" << l15.intersects(l16) << l16.intersects(l15) << (!l15.intersects(l16) ? "" : "FAIL!");
    //qDebug() << "point:" << l15.intersection(l16) << l16.intersection(l15);

    // Colinear diagonal disjunct lines example.
    Line l17(0.0, 0.0, 1.0, 0.0);
    Line l18(2.0, 0.0, 3.0, 0.0);
    l17.rotate(PI4);
    l18.rotate(PI4);
    qDebug() << "Colinear disjunct diagonal lines (no)" << l17 << l18;
    qDebug() << "intersect:" << l17.intersects(l18) << l18.intersects(l17) << (!l17.intersects(l18) ? "" : "FAIL!");
    //qDebug() << "point:" << l17.intersection(l18) << l18.intersection(l17);


    // Colinear horizontal end point touch lines example.
    Line l19(0.0, 0.0, 1.0, 0.0);
    Line l20(1.0, 0.0, 2.0, 0.0);
    qDebug() << "Colinear end point touch horizontal lines (no)" << l19 << l20;
    qDebug() << "intersect:" << l19.intersects(l20) << l20.intersects(l19) << (!l19.intersects(l20) ? "" : "FAIL!");
    //qDebug() << "point:" << l13.intersection(l14) << l14.intersection(l13);

    // Colinear vertical end point touch lines example.
    Line l21(0.0, 0.0, 0.0, 1.0);
    Line l22(0.0, 1.0, 0.0, 2.0);
    qDebug() << "Colinear end point touch vertical lines (no)" << l21 << l22;
    qDebug() << "intersect:" << l21.intersects(l22) << l22.intersects(l21) << (!l21.intersects(l22) ? "" : "FAIL!");
    //qDebug() << "point:" << l15.intersection(l16) << l16.intersection(l15);

    // Colinear diagonal end point touch lines example.
    Line l23(0.0, 0.0, 1.0, 0.0);
    Line l24(2.0, 0.0, 3.0, 0.0);
    l23.rotate(PI4);
    l24.rotate(PI4);
    qDebug() << "Colinear end point touch diagonal lines (no)" << l23 << l24;
    qDebug() << "intersect:" << l23.intersects(l24) << l24.intersects(l23) << (!l23.intersects(l24) ? "" : "FAIL!");
    //qDebug() << "point:" << l17.intersection(l18) << l18.intersection(l17);


    // Colinear horizontal overlapping lines example.
    Line l25(0.0, 0.0, 2.0, 0.0);
    Line l26(1.0, 0.0, 3.0, 0.0);
    qDebug() << "Colinear overlapping horizontal lines (yes)" << l25 << l26;
    qDebug() << "intersect:" << l25.intersects(l26) << l26.intersects(l25) << (l25.intersects(l26) ? "" : "FAIL!");
    //qDebug() << "point:" << l13.intersection(l14) << l14.intersection(l13);

    // Colinear vertical overlapping lines example.
    Line l27(0.0, 0.0, 0.0, 2.0);
    Line l28(0.0, 1.0, 0.0, 3.0);
    qDebug() << "Colinear overlapping vertical lines (yes)" << l27 << l28;
    qDebug() << "intersect:" << l27.intersects(l28) << l28.intersects(l27) << (l27.intersects(l28) ? "" : "FAIL!");
    //qDebug() << "point:" << l15.intersection(l16) << l16.intersection(l15);

    // Colinear diagonal overlapping lines example.
    Line l29(0.0, 0.0, 2.0, 0.0);
    Line l30(1.0, 0.0, 3.0, 0.0);
    l29.rotate(PI4);
    l30.rotate(PI4);
    qDebug() << "Colinear overlapping diagonal lines (yes)" << l29 << l30;
    qDebug() << "intersect:" << l29.intersects(l30) << l30.intersects(l29) << (l29.intersects(l30) ? "" : "FAIL!");
    //qDebug() << "point:" << l17.intersection(l18) << l18.intersection(l17);

    StopWatch sw;

    // Line intersection test.
    int howManyLines = 10000;
    LinkedList<Line> lines;
    for (int i = 0; i < howManyLines; i++)
    {
        Vec2 v1;
        Vec2 v2;
        v1.randomize();
        v2.randomize();
        lines << Line(v1, v2);
    }

    sw.start();
    ListIterator<Line> lit = lines.begin();
    while (lit.hasNext())
    {
        Line& l1 = lit.next();
        ListIterator<Line> lit2 = lines.begin();
        while (lit2.hasNext())
        {
            Line& l2 = lit2.next();
            l1.intersects(l2);
        }
    }
    double time = sw.elapsedTimeMs();

    qDebug() << "Line intersection:" << 1000*time/(howManyLines*howManyLines) << "microseconds.";


    // Sameside test.
    int howManyPoints = 300;
    LinkedList<Vec2> points;
    for (int i = 0; i < howManyPoints; i++)
    {
        Vec2 v1;
        v1.randomize();
        points << v1;
    }

    sw.start();
    lit = lines.begin();
    while (lit.hasNext())
    {
        Line& l = lit.next();

        ListIterator<Vec2> pit = points.begin();
        while (pit.hasNext())
        {
            Vec2& p1 = pit.next();

            ListIterator<Vec2> pit2 = points.begin();
            while (pit2.hasNext())
            {
                Vec2& p2 = pit2.next();
                l.sameSide(p1, p2);
            }
        }
    }
    time = sw.elapsedTimeMs();

    qDebug() << "Same side test:" << 1000*time/(howManyLines*howManyPoints*howManyPoints) << "microseconds.";


    // Left of test.
    sw.start();
    lit = lines.begin();
    while (lit.hasNext())
    {
        Line& l = lit.next();

        ListIterator<Vec2> pit = points.begin();
        while (pit.hasNext())
        {
            Vec2& p = pit.next();
            l.isLeftOf(p);
        }
    }
    time = sw.elapsedTimeMs();

    qDebug() << "Left of test:" << 1000*time/(howManyLines*howManyPoints) << "microseconds.";


    // Tangential test.
    int howManyNodes = 10000;
    LinkedList<Node> nodes;
    for (int i = 0; i < howManyNodes; i++)
    {
        Node n;
        n.randomize();
        n.v1.randomize();
        n.v2.randomize();
        nodes << n;
    }

    sw.start();
    ListIterator<Node> nit = nodes.begin();
    while (nit.hasNext())
    {
        Node& n1 = nit.next();

        ListIterator<Node> nit2 = nodes.begin();
        while (nit2.hasNext())
        {
            Node& n2 = nit2.next();
            n1.isTangentialTo(&n2);
        }
    }

    time = sw.elapsedTimeMs();

    qDebug() << "Tangential test:" << 1000*time/(howManyNodes*howManyNodes) << "microseconds.";
}

// Compares the accuracy of the analytic prediction with the simulation of holonomic and unicycle models.
void Experimenter::predictionAccuracyTest()
{
    int dataPoints = 100;
    double ax = -35;
    double ay = 30;
    double vx = 5;
    double vy = 5;
    double a = 40;
    double b = -18;


    Logger logger("data/test.txt");


    // I have run this test already and it turns out that, if the damping factors are removed and
    // the system iteration time is reduced by a factor of ten or hundred, both the holonomic and
    // unicycle prediction methods are right on par with the simulation.
    // This is especially important because the controls are accelerations, which are then converted
    // to forces for the simulation and this shows that there is no error in this conversion. Now,
    // The simulation is usually run with a 0.01 s update interval which is relatively coarse and
    // then the simulation starts being inaccurate (in this case actually the predictions are the
    // more accurate solutions). But then it also computes faster and so far there have been no
    // issues.

    // Holonomic test
    Hpm2D h;
    h.setVel(vx, vy);
    h.setAcc(ax, ay);

    Unicycle u = h;
    u.a = 0;
    u.b = 0;
    u.w = -6;

    for (int i = 0; i < dataPoints; i++)
    {
        h.predict(config.rcIterationTime);
        u.predict(config.rcIterationTime);

        logger << i*config.rcIterationTime;
        logger << u.pos() << h.pos();
        logger++;
    }
}

// Tests and prints the computation time of various predict functions.
void Experimenter::predictionPerformanceTest()
{
    Unicycle u;
    Hpm2D hpm;
    StopWatch sw;

    u.x = Statistics::randomNumber();
    u.y = Statistics::randomNumber();
    u.theta = Statistics::randomNumber();
    u.v = Statistics::randomNumber();
    u.w = Statistics::randomNumber();
    u.a = Statistics::randomNumber();
    u.b = Statistics::randomNumber();

    hpm.x = Statistics::randomNumber();
    hpm.y = Statistics::randomNumber();
    hpm.vx = Statistics::randomNumber();
    hpm.vy = Statistics::randomNumber();
    hpm.ax = Statistics::randomNumber();
    hpm.ay = Statistics::randomNumber();

    uint N = 10000000;

    sw.start();
    for (int k = 0; k < N; k++)
        u.predict();
    qDebug() << "Unicycle predict fresnel():" << sw.elapsedTimeMs()*1000/N << "micros" << u;

    u.b = 0;

    sw.start();
    for (int k = 0; k < N; k++)
        u.predict();
    qDebug() << "Unicycle predict b0():" << sw.elapsedTimeMs()*1000/N << "micros" << u;

    u.a = 0;
    u.b = 0;

    sw.start();
    for (int k = 0; k < N; k++)
        u.predict();
    qDebug() << "Unicycle predict arc():" << sw.elapsedTimeMs()*1000/N << "micros" << u;

    u.a = 0;
    u.b = 0;
    u.w = 0;

    sw.start();
    for (int k = 0; k < N; k++)
        u.predict();
    qDebug() << "Unicycle predict linear():" << sw.elapsedTimeMs()*1000/N << "micros" << u;

    sw.start();
    for (int k = 0; k < N; k++)
        hpm.predict();
    qDebug() << "Hpm2D predict():" << sw.elapsedTimeMs()*1000/N << "micros";
}

// Measures and prints the execution time of various arithmetic operations on the CPU.
void Experimenter::arithmeticOperationsTest()
{
    StopWatch sw;

    uint N = 100000000;

    double a = -9.2;
    double b = 3.6;
    double time = 0;


    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += k;
    }
    time = sw.elapsedTimeMs();
    qDebug() << "+ test:" << 1000*1000*time/N << "ns" << a;


    a = 1.0;
    b = 1.0 + 1.0E-6;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a *= b;
    }
    time = sw.elapsedTimeMs();
    qDebug() << "* test:" << 1000*1000*time/N << "ns" << a;

    a = 1.0E+100;
    b = 1.0 + 1.0E-6;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a /= b;
    }
    time = sw.elapsedTimeMs();
    qDebug() << "/ test:" << 1000*1000*time/N << "ns" << a;

    a = 0.1;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += sqrt(a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "sqrt test:" << 1000*1000*time/N << "ns" << a;

    a = 0.1;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += sin(a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "sin test:" << 1000*1000*time/N << "ns" << a;

    a = 0.1;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += fsin(a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "fsin test:" << 1000*1000*time/N << "ns" << a;

    a = 0.1;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += cos(a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "cos test:" << 1000*1000*time/N << "ns" << a;

    a = 0.1;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += fcos(a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "fcos test:" << 1000*1000*time/N << "ns" << a;

    a = 0.1;
    sw.start();
    double s,c;
    for (uint k = 0; k < N; k++)
    {
        sincos(a, &s, &c);
        a+=s;
    }
    time = sw.elapsedTimeMs();
    qDebug() << "sincos test:" << 1000*1000*time/N << "ns" << a;

    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += asin((double)k/N);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "asin test:" << 1000*1000*time/N << "ns" << a;

    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += acos((double)k/N);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "acos test:" << 1000*1000*time/N << "ns" << a;

    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += asin((double)k/N);
        a += acos((double)k/N);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "asincos test:" << 1000*1000*time/N << "ns" << a;

    a = 1.0;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += atan(a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "atan test:" << 1000*1000*time/N << "ns" << a;

    a = 0.1;
    b = 1.0 + 1.0E-6;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += atan2(b, a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "atan2 test:" << 1000*1000*time/N << "ns" << a;

    a = 0.1;
    b = 1.0 + 1.0E-6;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += fatan2(b, a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "fatan2 test:" << 1000*1000*time/N << "ns" << a;

    a = 100.0;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += picut(a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "picut test:" << 1000*1000*time/N << "ns" << a;

    a = 1.0;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += fpicut(a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "fpicut test:" << 1000*1000*time/N << "ns" << a;

    a = 1.0;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += ffpicut(a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "ffpicut test:" << 1000*1000*time/N << "ns" << a;

    a = 1.0;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += exp(a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "exp test:" << 1000*1000*time/N << "ns" << a;

    a = 1.0;
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += fexp(a);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "fexp test:" << 1000*1000*time/N << "ns" << a;


    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += 0.002;
        if (a*b < 0)
            b -= 0.001;
    }
    time = sw.elapsedTimeMs();
    qDebug() << "a*b < 0 sign test:" << 1000*1000*time/N << "ns" << a;

    sw.start();
    for (uint k = 0; k < N; k++)
    {
        a += 0.2;
        if (sgn(a) != sgn(b))
            b -= 0.001;
    }
    time = sw.elapsedTimeMs();
    qDebug() << "sgn(a) != sgn(b) sign test:" << 1000*1000*time/N << "ns" << a;


    static double table[100];
    sw.start();
    for (uint k = 0; k < N; k++)
    {
        double v = 100*(double)k/N;
        int idx = (int)v;
        double y = table[idx] + (v - idx) * (table[idx+1] - table[idx]);
        a += y;
    }
    time = sw.elapsedTimeMs()/N;
    qDebug() << "table lookup:" << 1000*1000*time/N << "ns" << a;
}


void Experimenter::pocPredictionTest()
{
    Poc poc, poc2, poc3, poc4;

    Poc start;
    start.phi = 0.2;
    start.vphi = 0;

    Logger logger("data/predict.txt");
    for(double dt = 0; dt <= 3.0+EPSILON; dt+=0.01)
    {
        poc.reset();
        poc = start;
        poc.simulate(dt);

        poc2.reset();
        poc2 = start;
        poc2.simulate2(dt);

        poc3.reset();
        poc3 = start;
        poc3.predictNaive(dt);

        poc4.reset();
        poc4 = start;
        poc4.predictTaylor(dt);

        logger << dt << poc.phi << poc2.phi << poc3.phi << poc4.phi;
        logger++;
    }

    StopWatch sw;
    sw.start();
    for(double dt = 0; dt <= 3.0+EPSILON; dt+=0.01)
    {
        poc.reset();
        poc = start;
        poc.simulate(dt);
    }
    double time = sw.elapsedTimeMs();
    qDebug() << "Euler:" << time;

    sw.start();
    for(double dt = 0; dt <= 3.0+EPSILON; dt+=0.01)
    {
        poc.reset();
        poc = start;
        poc.predictNaive(dt);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "Naive:" << time;

    sw.start();
    for(double dt = 0; dt <= 3.0+EPSILON; dt+=0.01)
    {
        poc.reset();
        poc = start;
        poc.predictTaylor(dt);
    }
    time = sw.elapsedTimeMs();
    qDebug() << "Taylor:" << time;
}

#ifndef EXPERIMENTER_H_
#define EXPERIMENTER_H_
#include <QThread>
#include <QString>
#include "util/Vec2.h"
#include "util/Vector.h"
#include "util/Logger.h"
#include "ExperimentConfig.h"

class Experimenter : public QThread
{
	Q_OBJECT

    QVector<ExperimentConfig> cases;
    Logger progressLogger;

public:
	bool running;

public:
    Experimenter();
    ~Experimenter(){running=false;wait();}

public:
	void init();
	void run();
    void stop(){running=false;}

    
public slots:
    void startstop(){running ? stop() : start();}

signals:
	void messageOut(QString m);
	void progress(int);
	void resetOut();
    void configChangedOut();

private:
    void trajectoryExperiments();
    void postProcessTrajectoryExperiments();
    void geometricModelTest();
    void geometryRuntimeTests();
    void containerTest();
    void lineTests();
    void predictionAccuracyTest();
    void predictionPerformanceTest();
    void arithmeticOperationsTest();
    void pocPredictionTest();
    void processCase(ExperimentConfig& ex);
    void saveCases();
    void loadCases();
    void generateCases(int runs, int frames);
    void loadCaseData();
};

#endif /* Experimenter_H_ */

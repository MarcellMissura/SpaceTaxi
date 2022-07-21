#ifndef OpenGLWidget_H
#define OpenGLWidget_H

#include "MessageQueue.h"
#include <QGLViewer/qglviewer.h>
#include "RobotModel/RobotModel.h"
#include "RobotModel/Copedo.h"
#include "RobotModel/Dynaped.h"
#include "RobotModel/Thor.h"
#include "RobotModel/Nop.h"
#include "blackboard/State.h"
#include "soccerfield/SoccerField.h"

class OpenGLWidget: public QGLViewer
{
	Q_OBJECT

public:
	double tscale;
	bool recording;

	bool autoCam;
    int showFloor;
    bool showGcv;
	bool showShadow;
    bool showTrajectories;
    bool showShortestPath;
    bool showRobotModel;
    bool showWorldPolygons;
    bool showWorldGrid;
    bool showWorldGeometry;
    bool showGlobalPath;
    bool showSensedGrid;
    bool showExperimentPaths;

private:
	int currentStateIndex;
    double radius;

    MessageQueue messageQueue;
    int pushArrowTimer;

    RobotModel* rxModel;
	RobotModel* txModel;
	Copedo rxCopedo;
	Copedo txCopedo;
	Dynaped rxDynaped;
	Dynaped txDynaped;
	Thor rxThor;
	Thor txThor;
	Nop rxNop;
	Nop txNop;
	RobotModel rxSimon;
	RobotModel txSimon;
    Vector<RobotModel> models;
    RobotModel trajectorySimon;
    SoccerField soccerField;

public:
    OpenGLWidget(QWidget* parent=0);
    ~OpenGLWidget();

	void changeRobot(QString robotName);

public:
	void keyPressEvent(QKeyEvent*);


public slots:
	void messageIn(QString m);
    void setFrameIndex(int cfi);
	void reset();

	void toggleAxis();
	void togglePerspective();
	void toggleFloor();
	void toggleShadow();
    void toggleTrajectories();
	void toggleGcv();
    void toggleRobotModel();
    void toggleShortestPath();
    void toggleWorldPolygons();
    void toggleWorldGrid();
    void toggleWorldGeometry();
    void toggleGlobalPath();
    void toggleSensedGrid();
    void toggleExperimentPaths();

protected:
	void init();
	void draw();

private:
    void drawFloor() const;
    void drawSoccerField() const;
    void drawGaitVector(Vec3 gaitVector, double r, double g, double b) const;
    void drawTrajectories() const;

};

#endif // OpenGLWidget_H

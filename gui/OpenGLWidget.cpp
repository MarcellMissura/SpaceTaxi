#include "OpenGLWidget.h"
#include <QKeyEvent>
#include "blackboard/Config.h"
#include "util/GLlib.h"
#include "soccerfield/Ball.h"
#include "soccerfield/Target.h"
#include "soccerfield/Obstacle.h"

// The OpenGLWidget offers a 3D view where the robot model and basically
// anything can be visualized. It's based on the QGLViewer library that offers
// great possibilities to create an OpenGL environment, move the camera in
// it with the mouse and it also comes with a library for 3D transformations.
// The landscape widget contains an own robot model for visualization purposes.
// The idea is that for the landscape to be an independent module, only joint
// angle data are transfered to it from simulation, from the real robot or
// over the network. The joint angles are applied to an internal model, which
// is then displayed on the screen. The drawback is that if the robot is
// changed, the landscape has to be notified to change its internal model for
// accurate visualization.

OpenGLWidget::OpenGLWidget(QWidget *parent) : QGLViewer(parent)
{
	radius = 10.0;
	recording = false;
	tscale = 1.0;
    autoCam = false;
    showFloor = 1;
	showGcv = true;
    showShadow = false;
    showTrajectories = false;
	currentStateIndex = 0;
	pushArrowTimer = 0;
    showRobotModel = false;
    showShortestPath = false;
    showWorldPolygons = true;
    showWorldGeometry = false;
    showWorldGrid = false;
    showGlobalPath = false;
    showSensedGrid = false;
    showExperimentPaths = false;

	rxModel = &rxSimon;
	txModel = &txSimon;

	rxSimon.name = "rxSimon";

	connect(&messageQueue, SIGNAL(updated()), this, SLOT(update()));

    setAnimationPeriod(0);
}

void OpenGLWidget::keyPressEvent(QKeyEvent* ev)
{
	QGLViewer::keyPressEvent(ev);
}

void OpenGLWidget::reset()
{
	rxModel->reset();
	txModel->reset();
}

void OpenGLWidget::init()
{
	restoreStateFromFile();

//	setBackgroundColor(QColor(245,245,245));
	setBackgroundColor(QColor(255,255,255));
	setForegroundColor(QColor(0,0,0));
	setFont(QFont("Helvetica", 18));

	//setAxisIsDrawn(true);
	setSceneRadius(radius);

	// Make camera the default manipulated frame.
    //setManipulatedFrame(camera()->frame());

	// Light setup
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glPointSize(2.0);
	glLineWidth(1.0);

    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
}

OpenGLWidget::~OpenGLWidget()
{
	saveStateToFile();
}

// Switch to a new robotModel.
void OpenGLWidget::changeRobot(QString robotName)
{
	if (robotName == "Nop")
	{
        rxModel = &rxSimon;
        txModel = &txSimon;
	}
	else if (robotName == "Thor")
	{
		rxModel = &rxThor;
		txModel = &txThor;
	}
	else if (robotName == "Copedo")
	{
		rxModel = &rxCopedo;
		txModel = &txCopedo;
	}
	else if (robotName == "Dynaped")
	{
		rxModel = &rxDynaped;
		txModel = &txDynaped;
	}
	else
	{
		rxModel = &rxSimon;
		txModel = &txSimon;
	}
}

void OpenGLWidget::messageIn(QString m)
{
	messageQueue.messageIn(m);
	update();
}

void OpenGLWidget::setFrameIndex(int cfi)
{
    currentStateIndex = cfi;
	update();
}

void OpenGLWidget::togglePerspective()
{
	if (camera()->type() == Camera::ORTHOGRAPHIC)
		camera()->setType(Camera::PERSPECTIVE);
	else
		camera()->setType(Camera::ORTHOGRAPHIC);
	update();
}

void OpenGLWidget::toggleAxis()
{
	toggleAxisIsDrawn();
	update();
}

void OpenGLWidget::toggleFloor()
{
    showFloor = (showFloor+1) % 3;
	update();
}

void OpenGLWidget::toggleShadow()
{
	showShadow = !showShadow;
	update();
}

void OpenGLWidget::toggleTrajectories()
{
    showTrajectories = !showTrajectories;
    update();
}

void OpenGLWidget::toggleGcv()
{
	showGcv = !showGcv;
	update();
}

void OpenGLWidget::toggleRobotModel()
{
    showRobotModel = !showRobotModel;
    update();
}

void OpenGLWidget::toggleShortestPath()
{
    showShortestPath = !showShortestPath;
    update();
}

void OpenGLWidget::toggleWorldPolygons()
{
    showWorldPolygons = !showWorldPolygons;
    update();
}

void OpenGLWidget::toggleWorldGrid()
{
    showWorldGrid = !showWorldGrid;
    update();
}

void OpenGLWidget::toggleWorldGeometry()
{
    showWorldGeometry = !showWorldGeometry;
    update();
}

void OpenGLWidget::toggleGlobalPath()
{
    showGlobalPath = !showGlobalPath;
    update();
}

void OpenGLWidget::toggleSensedGrid()
{
    showSensedGrid = !showSensedGrid;
    update();
}


void OpenGLWidget::toggleExperimentPaths()
{
    showExperimentPaths = !showExperimentPaths;
    update();
}

void OpenGLWidget::draw()
{
    // Mutex against the step of robot control loop.
    QMutexLocker locker(&state.gMutex);

    if (showFloor == 1)
        drawFloor();
    if (showFloor == 2)
        drawSoccerField();

    // Show recording state.
	if (recording)
	{
		glColor3f(0.8, 0.3, 0.3);
		drawText(10, 24, "Recording...", QFont("Helvetica", 18));
	}

	// On the top: show the message queue.
	for (int i = 0; i < messageQueue.messages.size(); i++)
	{
		glColor4f(0.3, 0.3, 0.0, messageQueue.fadeFactors[i]);
		drawText(10, 52 + i*28, messageQueue.messages[i], QFont("Helvetica", 18));
	}


    // On the bottom: show the frame id, step id, the current time and what not.
	glColor3f(0.3, 0.3, 0.8);
	drawText(this->width() - 52, this->height() - 10, "x" + QString().number(tscale, 'f', 1), QFont("Helvetica", 14));
    drawText(10, this->height() - 10, "frame: " + QString().number(state.frameId) +
            //"  step: " + QString().number(state[currentStateIndex].stepId) +
            //"  time: " + QString().number(state[currentStateIndex].time, 'f', 3) +
            "  expansions: " + QString().number(state[currentStateIndex].fsExpansions, 'f', 3) +
            "  steps: " + QString().number(state[currentStateIndex].fsSteps, 'f', 3) +
            "  exec: " + QString().number(state[currentStateIndex].lastExecutionTime, 'f', 3) +
			"  debug: " + QString().number(state[currentStateIndex].debug, 'f', 3),
			QFont("Helvetica", 14, QFont::Light));

    // Draw the world polygons.
    if (showWorldPolygons)
    {
        glPushMatrix();
        glTranslated(0, 0, 0.01);
        glLineWidth(30);
        glColor4d(0.6, 0.6, 0.6, 0.3);
        state.worldPolygons.draw();
        glPopMatrix();
    }

    // Draw the world grid.
    if (showWorldGrid)
        state.worldGridModel.draw(QColor("#AAFF5555"));

    // Draw the world geometric model.
    if (showWorldGeometry)
    {
        glLineWidth(1);
        state.worldGridModel.draw();
        glTranslated(0,0,0.01);
        glColor4d(0, 0.9, 0.9, 0.5);
        state.worldGeometricModel.draw();
    }

    // Draw the global path.
    if (showGlobalPath)
    {
        glColor3d(0, 0.9, 0.9);
        for (uint i = 1; i < state.globalPath.size(); i++)
            GLlib::drawLine(state.globalPath[i-1], state.globalPath[i], 0.04);
    }

    // Draw the sensed grid.
    if (showSensedGrid)
    {
        glPushMatrix();
        glTranslated(state.startLocation.x, state.startLocation.y, 0);
        glRotatef(state.startLocation.z*RAD_TO_DEG, 0, 0, 1);
        state.sensedGrid.draw();
        glPopMatrix();
    }

    // Draw the start and the goal.
    if (command.showStartAndGoal)
    {
        glPushMatrix();
        glTranslated(0, 0, 0.01);

        // Draw the start state.
        Vec3 stepTemp(0.0857143, 0.457143, -0.714286); // The fuck is this?
        stepTemp.frotate(state.startLocation.z);
        //Vec3 comLocation = state.startLocation + 0.5*stepTemp;
        //glColor3d(1.0, 0.2, 0.0);
        //glLineWidth(5);
        //GLlib::drawNoseCircle(comLocation, 6.0);

        // Draw a coordinate system to show the robot frame.
        if (axisIsDrawn())
        {
            glPushMatrix();
            glTranslated(state.startLocation.x, state.startLocation.y, 0);
            glRotatef(state.startLocation.z*RAD_TO_DEG, 0, 0 ,1);
            QGLViewer::drawAxis(1.0);
            glPopMatrix();
        }

        // Draw the target state.
        glColor3d(1.0, 1.0, 0.0);
        glLineWidth(5);
        GLlib::drawNoseCircle(state.targetLocation, 5.0);

        glPopMatrix();
    }

    // Draw the footstep plan.
    // Transform into the egocentric view of the robot.
    // Everything in the footstep plan is egocentric.
    glPushMatrix();
    glTranslated(0,0,0.004);
    glTranslated(state.startLocation.x, state.startLocation.y, 0);
    glRotatef(state.startLocation.z*RAD_TO_DEG, 0, 0 ,1);
    state.footStepPlan.draw();
    glPopMatrix();

    if (command.showLabels)
        state.footStepPlan.drawLabels(this);

    // The remaining path from intermediate goal to global goal.
    // A very annoying thing that's always broken.
    if (command.showFootsteps > 0)
    {
        Vec3 intermediateTarget = state.footStepPlan.getTargetState();

        // Transform the intermediate target back to world coordinates.
        intermediateTarget.rotate(state.startLocation.z);
        intermediateTarget.x += state.startLocation.x;
        intermediateTarget.y += state.startLocation.y;

        // Use MC to plan a path to the global target in the world geometric model.
        static VisibilityGraph vgg;
        vgg.setGeometricModel(state.worldGeometricModel);
        vgg.setTarget(state.targetLocation);
        if (vgg.minimalConstruct(intermediateTarget))
            vgg.drawPath();
    }

    // The experiment paths.
    if (showExperimentPaths)
    {
        // Lines connecting the com locations along the path.
        for (uint k = 0; k < state.experimentPaths.size(); k++)
        {
            glPushMatrix();
            glTranslated(0,0,0.004);
            const Vector<Vec3>& trace = state.experimentPaths[k];
            glLineWidth(3.0);
            glColor3d(0, 0, 0);
            glBegin(GL_LINE_STRIP);
            for (uint i = 0; i < trace.size()-1; i++)
                glVertex2dv(trace[i]);
            glEnd();
            glPopMatrix();
        }

        // Nose circles for the com locations at the start and the end of the path.
        for (uint k = 0; k < state.experimentPaths.size(); k++)
        {
            glPushMatrix();
            glTranslated(0,0,0.004);
            const Vector<Vec3>& trace = state.experimentPaths[k];
            Vec3 com1 = trace.first();
            glColor3d(1.0, 1.0, 1.0);
            GLlib::drawNoseCircle(com1);
            Vec3 com2 = trace.last();
            glColor3d(1.0, 1.0, 1.0);
            GLlib::drawNoseCircle(com2);
            glPopMatrix();
        }
    }


    // Draw the end effector and com trajectories.
    if (showTrajectories)
    {
        static const int frames = 200;
        trajectorySimon.walk(state[frames].rxAction.pose, Vec3(state[frames].fusedAngle.x, state[frames].fusedAngle.y, 0), state[frames].rxSupportLegSign);
        RobotModel rm = trajectorySimon;
        for (int i = frames; i > 0; i--)
        {
            rm.walk(state[i].rxAction.pose, Vec3(state[i].fusedAngle.x, state[i].fusedAngle.y, 0), state[i].rxSupportLegSign);
            rm.drawEndEffectors(0.5);
        }
        rm.draw(0.8, 0.6, 0.8, 0.95);
    }

    // The robot model.
    if (showRobotModel)
    {
        // Apply the pose to the robot model and draw.
        rxModel->walk(state[currentStateIndex].rxAction.pose, Vec3(state[currentStateIndex].fusedAngle.x, state[currentStateIndex].fusedAngle.y, 0), state[currentStateIndex].rxSupportLegSign);
        rxModel->draw(0.8, 0.6, 0.8, 0.95);
        if (showShadow)
        {
            if (rxModel->supportLegSign == txModel->supportLegSign)
                txModel->footStep = rxModel->footStep;
            txModel->walk(state[currentStateIndex].txAction.pose, Vec3(state[currentStateIndex].fusedAngle.x, state[currentStateIndex].fusedAngle.y, 0), state[currentStateIndex].txSupportLegSign);
            txModel->draw(0.2, 0.2, 0.2, 0.5);
        }

        if (autoCam)
            camera()->setPivotPoint(rxModel->base.position());

        // Draw the gait target and control vectors.
        if (showGcv)
        {
            glPushMatrix();
            glMultMatrixd(rxModel->base.worldMatrix());
            glColor4f(1.0, 0.6, 0.6, 0.8);
            glTranslatef(0, 0, 0.3);
            drawGaitVector(state[currentStateIndex].mgGcvUserTarget, 1.0, 0.5, 0.5);
            glTranslatef(0, 0, -0.2);
            drawGaitVector(state[currentStateIndex].mgGcv, 0.5, 0.5, 1.0);
            glPopMatrix();
        }
    }

	// Draw the push arrow.
	if (recording)
	{
		if (state[currentStateIndex].pushed != 0)
			pushArrowTimer = 20;
	}
	else
	{
		pushArrowTimer = state[currentStateIndex].pushed;
	}

	if (pushArrowTimer > 0)
	{
		double angle = state.pushImpulse.angle();
		double length = 0.1 + 0.1*state.pushImpulse.norm();

		glPushMatrix();
		glColor4f(0.8, 0.2, 0.2, qMin((double)pushArrowTimer/10, 1.0)*1.0);
		glMultMatrixd(rxModel->base.worldMatrix());
		glRotated(180*angle/PI, 0, 0, 1.0);
		glTranslated(-0.3 - length, 0, 0);
		GLlib::drawArrow(length, 0.05);
		glPopMatrix();

		pushArrowTimer--;
	}
}

void OpenGLWidget::drawGaitVector(Vec3 gaitVector, double r, double g, double b) const
{
	static GLUquadric* quadric = gluNewQuadric();

	float radius = 0.4;
	float thickness = 0.03;
	float minValue = 0.08;

	double direction = 0;
	if (Vec(gaitVector.x, gaitVector.y, 0).norm() >= minValue)
		direction = atan2(gaitVector.y, gaitVector.x);

	glColor4f(r, g, b, 0.9);

	if (Vec(gaitVector.x, gaitVector.y, 0).norm() >= minValue)
	{
		glPushMatrix();
		glRotatef(direction * 180/PI, 0,0,1);
		glTranslatef(radius, 0, 0);
		GLlib::drawArrow(Vec(gaitVector.x, gaitVector.y, 0.0).norm(), thickness);
		glPopMatrix();
	}

	if (fabs(gaitVector.z) >= minValue)
	{
		float angleStep = 0.01;
		int numt = 32;
		float angle = angleStep;
		double x,y,z;

		glPushMatrix();
		glRotatef(direction * 180/PI, 0,0,1);

		// Torus.
		for (int i = 0; i < numt; i++)
		{
			glBegin(GL_TRIANGLE_STRIP);
			angle = 0;
			while (angle < fabs(gaitVector.z) * PI)
			{
				x = (radius + thickness * cos(PII * i/numt) ) * cos(sgn(gaitVector.z) * angle);
				y = (radius + thickness * cos(PII * i/numt) ) * sin(sgn(gaitVector.z) * angle);
				z = thickness * sin(PII * i/numt);
				glVertex3f(x, y, z);

				x = (radius + thickness * cos(PII * (i+1)/numt) ) * cos(sgn(gaitVector.z) * angle);
				y = (radius + thickness * cos(PII * (i+1)/numt) ) * sin(sgn(gaitVector.z) * angle);
				z = thickness * sin(PII * (i+1)/numt);
				glVertex3f(x, y, z);

				angle += angleStep;
			}
			glEnd();
		}

		// Arrow head.
		glRotatef(gaitVector.z * 180, 0,0,1);
		glTranslatef(radius, 0, 0);
		glRotatef(-sgn(gaitVector.z)*90, 1,0,0);
		gluCylinder(quadric, 2*thickness, 0.0f, 0.1f, 32, 1);

		glPopMatrix();
    }
}

// Draws the soccer field.
void OpenGLWidget::drawSoccerField() const
{
    soccerField.draw();

    Ball ball;
    ball.setCartesian(2.2, 1.0);
    ball.draw();

    Target target;
    target.setCartesian(2.0, 1.0);
    target.draw();

    Obstacle obstacle;
    obstacle.setCartesian(3.0, 2.0);
    obstacle.draw();
}

// Draws the floor.
void OpenGLWidget::drawFloor() const
{
    double size = 1.4*radius;
    double stride = 0.5;
    glPushMatrix();
    glTranslated(0, 0, -0.002);
    glBegin( GL_QUADS );
    glColor3f(0.95, 0.95, 0.95);
    glVertex2d(-size, -size);
    glVertex2d(-size, size);
    glVertex2d(size, size);
    glVertex2d(size, -size);
    glEnd();

    glTranslated(0, 0, 0.001);

    glLineWidth(1);
    glBegin( GL_LINES );
    glColor3f(0.65, 0.65, 0.65);
    for (double i = -size; i <= size + 0.0001; i = i+stride)
    {
        glVertex2d(i, -size);
        glVertex2d(i, size);
        glVertex2d(-size, i);
        glVertex2d(size, i);
    }
    glEnd();
    glPopMatrix();
}

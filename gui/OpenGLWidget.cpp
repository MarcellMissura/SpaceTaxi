#include "OpenGLWidget.h"
#include <QTabWidget>
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "blackboard/Command.h"
#include "util/GLlib.h"

// The OpenGLWidget offers a 3D view where basically anything can be visualized.
// It's based on the QGLViewer library that offers great possibilities to create
// an OpenGL environment and to move the camera in it with the mouse. The draw()
// method is the most important one where OpenGL rendering takes place. The
// OpenGLWidget uses the OpenGLScene to issue draw commands.
OpenGLWidget::OpenGLWidget(QWidget *parent) : QGLViewer(parent)
{
    recording = false;
    inited = false;

    radius = 22.0;

    connect(&messageQueue, SIGNAL(updated()), this, SLOT(update()));
}

OpenGLWidget::~OpenGLWidget()
{
    if (inited)
        saveStateToFile();
}

// Initialization code executed once after construction.
void OpenGLWidget::init()
{
    //qDebug() << "OpenGLWidget::init()";
    bool succ = restoreStateFromFile();
    setSceneRadius(config.sceneRadius);
    setBackgroundColor(Qt::white);
    if (!succ)
    {
        camera()->setPosition(Vec(-4, -4, 4));
        camera()->setUpVector(Vec(0,0,1));
        camera()->lookAt(Vec(2, 0, 0));
        //showEntireScene();
    }

    // Light setup
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glEnable(GL_POINT_SMOOTH);
    //glEnable(GL_LINE_SMOOTH);

    // Using Alt-Mousewheel, you can zoom more precisely.
    camera()->setFlySpeed(0.0025);
    setWheelBinding(Qt::AltModifier, CAMERA, MOVE_FORWARD);

    inited = true;
}

// Reset code executed when the reset button is pressed.
void OpenGLWidget::reset()
{
    update();
}

// Main OpenGL drawing code.
void OpenGLWidget::draw()
{
    if (!command.draw)
        return;

    QMutexLocker locker(&state.bigMutex);

    stopWatch.start();

    glPushMatrix(); // Start vertical stacking.
    drawFloor();
    glTranslatef(0, 0, 0.001);
    drawRecordedPose();
    glTranslatef(0, 0, 0.001);

    // Draw the robot (which is almost everything).
    glTranslatef(0, 0, 0.001);
    state.robot.draw();

    // Draw all labels.
    // Because we are using the drawText() function of libqglviewer, we have to draw all labels
    // here in the draw() function. This has the ugly implicaton that man things in state.robot
    // need to be declared public so that we can access them from here. Otherwise the components
    // of state robot would need to have a reference to this qglviewer instance, which is also
    // very ugly.

    if  (command.showLabels)
    {
        // Line matching labels.
        if (command.showLineMap == 1)
        {
            glColor3f(0,0,0.8); // sensed line labels in blue
            for (uint i = 0; i < state.robot.geometricMap.inputLines.size(); i++)
            {
                const Line& l = state.robot.geometricMap.inputLines[i] + state.robot.geometricMap.inputPose;
                Vec2 c = l.center() + 0.06*l.normal();
                qglviewer::Vec screenPos = camera()->projectedCoordinatesOf(qglviewer::Vec(c.x, c.y, 0));
                drawText((int)screenPos[0], (int)screenPos[1], QString::number(l.id));
            }
            glColor3f(0.2,0,0.2); // map line labels in grey
            ListIterator<TrackedLine> it = state.robot.geometricMap.mapLines.begin();
            while (it.hasNext())
            {
                const TrackedLine& l = it.next();
                Vec2 c = l.center() + 0.06*l.normal();
                qglviewer::Vec screenPos = camera()->projectedCoordinatesOf(qglviewer::Vec(c.x, c.y, 0));
                drawText((int)screenPos[0], (int)screenPos[1], QString::number(l.id));
            }
        }

        // 2. Bare line map view.
        if (command.showLineMap == 2)
        {
            glColor3f(0,0,0); // All map lines in black.
            ListIterator<TrackedLine> it = state.robot.geometricMap.mapLines.begin();
            while (it.hasNext())
            {
                const TrackedLine& l = it.next();
                Vec2 c = l.center() + 0.06*l.normal();
                qglviewer::Vec screenPos = camera()->projectedCoordinatesOf(qglviewer::Vec(c.x, c.y, 0));
                drawText((int)screenPos[0], (int)screenPos[1], QString::number(l.id));
            }
        }

        // Pose graph labels.
        if (command.showPoseGraph)
        {
            ListIterator<PoseGraphNode> it = state.robot.geometricMap.poseGraphNodes.begin();
            while (it.hasNext())
            {
                const PoseGraphNode& gn = it.next();
                Vec2 c = gn.pose.pos();
                qglviewer::Vec screenPos = camera()->projectedCoordinatesOf(qglviewer::Vec(c.x, c.y, 0));
                drawText((int)screenPos[0], (int)screenPos[1], QString::number(gn.id));
            }
        }

        // Visibility polygon labels.
        if (command.showVisibilityPolygon)
        {
            glColor3f(0.1,0,1.0);
            ListIterator<Line> edges = state.robot.visibilityPolygon.edgeIterator();
            while (edges.hasNext())
            {
                const Line& l = edges.next();
                Vec2 c = l.center()+state.robot.pose();
                Vec2 n = 0.02*l.normal();
                c += n;
                qglviewer::Vec screenPos = camera()->projectedCoordinatesOf(qglviewer::Vec(c.x, c.y, 0));
                drawText((int)screenPos[0], (int)screenPos[1], QString::number(l.id));
            }
        }

        // Visibility Graph labels.
        if (command.showVisibilityGraph)
        {
            glColor3f(0,0,0);
            ListIterator<Node> it = state.robot.unifiedGeometricModel.visibilityGraph.nodes.begin();
            while (it.hasNext())
            {
                Node node = it.next();
                node += state.robot.pose();
                qglviewer::Vec screenPos = camera()->projectedCoordinatesOf(qglviewer::Vec(node.x+0.02, node.y, 0));
                drawText((int)screenPos[0], (int)screenPos[1], QString::number(node.id));
            }
        }

        // Laser labels.
        if (command.showLaser == 3)
        {
            Vector<Vec2> lb = state.robot.readLaserBuffer();
            lb += state.robot.pose();
            for (uint i = 0; i < lb.size(); i++)
            {
                qglviewer::Vec screenPos = camera()->projectedCoordinatesOf(qglviewer::Vec(lb[i].x+0.02, lb[i].y, 0));
                drawText((int)screenPos[0], (int)screenPos[1], QString::number(i));
            }
        }
    }

    glPopMatrix(); // Finish vertical stacking.

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

    // On the bottom: show the frame id, time, and debug.
    glColor3f(0.3, 0.3, 0.8);
    drawText(10, this->height() - 10, "frame: " + QString::number(state.frameId) +
            "  lines: " + QString::number(state.robot.geometricMap.mapLines.size()) +
            "  vertices: " + QString::number(state.robot.geometricMap.polygonMap.getVertexCount()) +
            "  nodes: " + QString::number(state.robot.geometricMap.poseGraphNodes.size()) +
            "  debug: " + QString::number(state.debug, 'f', 3),
             QFont("Helvetica", 14, QFont::Light));

    if (command.showRuler)
    {
        glColor3f(0.1,0.1,0.1);
        QGLViewer::drawText(mouseMovePos.x(), mouseMovePos.y(), worldCoordString, QFont("Helvetica", 12));
        QGLViewer::drawText(mouseMovePos.x(), mouseMovePos.y()+18, localCoordString, QFont("Helvetica", 12));
        QGLViewer::drawText(mouseMovePos.x(), mouseMovePos.y()+36, gridCoordString, QFont("Helvetica", 12));
    }

    state.drawTime = stopWatch.elapsedTimeMs();
}

// Handles incoming messages by appending them to the message queue.
void OpenGLWidget::messageIn(QString m)
{
    messageQueue.messageIn(m);
    update();
}

void OpenGLWidget::startRecording()
{
    recording = true;
    update();
}

void OpenGLWidget::stopRecording()
{
    recording = false;
    update();
}

// Jumps into the point of view of the robot camera.
void OpenGLWidget::cameraView()
{
    Transform3D T = state.robot.getCameraTransform();
    Vec3 forward = T * Vec3(1,0,0);
    Vec3 pos = T.getPosition();
    camera()->setPosition(Vec(pos.x,pos.y,pos.z));
    camera()->setUpVector(Vec(0,0,1));
    camera()->lookAt(Vec(forward.x,forward.y,forward.z));
    update();
}


// Toggles the drawing of the world coordinate system.
void OpenGLWidget::toggleAxis()
{
    toggleAxisIsDrawn();
    update();
}

void OpenGLWidget::drawRecordedPose()
{
    if (command.selectPose)
        GLlib::drawNoseCircle(recordedPose, Qt::green, config.agentRadius);
    else if (command.selectTarget)
        GLlib::drawNoseCircle(recordedPose, Qt::red, config.agentRadius);
}

void OpenGLWidget::mousePressEvent(QMouseEvent *qme)
{
    if (command.selectPose || command.selectTarget)
    {
        const QPoint &pos = qme->pos();
        bool found = false;
        qglviewer::Vec selectedPoint = camera()->pointUnderPixel(pos, found);
        if (found)
            recordedPose.setPos(selectedPoint.x, selectedPoint.y);
    }
    else
    {
        QGLViewer::mousePressEvent(qme);
    }
}

void OpenGLWidget::mouseReleaseEvent(QMouseEvent *qme)
{
    if (command.selectPose || command.selectTarget)
    {
        const QPoint &pos = qme->pos();
        bool found = false;
        qglviewer::Vec selectedPoint = camera()->pointUnderPixel(pos, found);
        if (found)
        {
            recordedPose.setHeading(ffpicut((recordedPose.pos() - Vec2(selectedPoint.x, selectedPoint.y)).angle()+PI)); // The heading is always w.r.t the x-axis in 2D.
            emit poseRecorded(recordedPose);
            update();
        }
    }
    else
    {
        QGLViewer::mouseReleaseEvent(qme);
    }
}

void OpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    mouseMovePos = event->pos();

    qglviewer::Vec orig, dir;
    camera()->convertClickToLine(mouseMovePos, orig, dir);
    dir.x = -dir.x/dir.z;
    dir.y = -dir.y/dir.z;
    dir.z = -1.0;

    Vec2 selectedPoint;
    selectedPoint.x = orig.x + dir.x*orig.z;
    selectedPoint.y = orig.y + dir.y*orig.z;
    //qDebug() << orig.x << orig.y << orig.z << " | " << dir.x << dir.y << dir.z << " | " << selectedPoint;

    if (command.showRuler)
    {
        Vec2 localPoint = Vec2(selectedPoint.x, selectedPoint.y) - state.robot.pose();
        worldCoordString = "world: [" + QString::number(selectedPoint.x,'f',3) + "," + QString::number(selectedPoint.y,'f',3) + "]\n";
        localCoordString = "loc:     [" + QString::number(localPoint.x,'f',3) + "," + QString::number(localPoint.y,'f',3) + "]";
        gridCoordString = "";
        if (state.robot.isPointInLocalGrid(localPoint))
        {
            Vec2u ni = state.robot.getNodeIndex(localPoint);
            gridCoordString = "grid:    [" + QString::number(ni.x) + "," + QString::number(ni.y) + "] " + QString::number(state.robot.getValueAt(localPoint));
        }

        update();
    }

    if (command.selectPose || command.selectTarget)
    {
        recordedPose.setHeading(ffpicut((recordedPose.pos() - Vec2(selectedPoint.x, selectedPoint.y)).angle()+PI)); // The heading is always w.r.t the x-axis in 2D.
    }
    else
    {
        QGLViewer::mouseMoveEvent(event);
    }
}

void OpenGLWidget::drawFloor()
{
    if (!command.showFloor)
        return;

    glPushMatrix();
    glTranslated(0,0,-0.02);
    float size = 2.0*radius;
    glBegin( GL_QUADS );
    glColor3f(0.8, 0.8, 0.8);
    glVertex2f(-size, -size);
    glVertex2f(-size, size);
    glVertex2f(size, size);
    glVertex2f(size, -size);
    glEnd();

    /*
    float stride = 1.0;
    glLineWidth(1);
    glBegin( GL_LINES );
    glColor3f(0.65, 0.65, 0.65);
    for (float i = -size; i <= size + 0.0001; i = i+stride)
    {
        glVertex2f(i, -size);
        glVertex2f(i, size);
        glVertex2f(-size, i);
        glVertex2f(size, i);
    }
    glEnd();
    */

    glPopMatrix();
}

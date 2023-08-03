#ifndef OPENGLWIDGET_H_
#define OPENGLWIDGET_H_

#include <QGLViewer/qglviewer.h>
#include "MessageQueue.h"
#include "lib/util/StopWatch.h"
#include "lib/util/Pose2D.h"

using namespace qglviewer;

class OpenGLWidget: public QGLViewer
{
    Q_OBJECT

    bool inited;

    double radius;

    MessageQueue messageQueue;
    StopWatch stopWatch;

    qglviewer::Vec poseArrowStart, poseArrowFinish;

    QString worldCoordString;
    QString localCoordString;
    QString gridCoordString;
    QPoint mouseMovePos;

    Pose2D recordedPose;

public:
    bool recording;

public:
    OpenGLWidget(QWidget* parent=0);
    ~OpenGLWidget();

public slots:
    void messageIn(QString m);
    void reset();
    void toggleAxis();
    void startRecording();
    void stopRecording();
    void cameraView();

protected:
    void init() override;
    void draw() override;

    void mousePressEvent(QMouseEvent *qme) override;
    void mouseReleaseEvent(QMouseEvent *qme) override;
    void mouseMoveEvent(QMouseEvent *qme) override;

private:
    void drawFloor();
    void drawRecordedPose();

signals:
    void poseRecorded(const Pose2D& p);

};

#endif

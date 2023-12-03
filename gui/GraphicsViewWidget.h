#ifndef GRAPHICSVIEWWIDGET_H_
#define GRAPHICSVIEWWIDGET_H_

#include <QtGui>
#include <QGraphicsView>
#include "MessageQueue.h"
#include "lib/util/StopWatch.h"
#include "lib/util/Pose2D.h"
#include "lib/geometry/Polygon.h"


class GraphicsViewWidget : public QGraphicsView
{
    Q_OBJECT

public:
    bool showAxis;
    bool showGrid;
    bool showRuler;
    bool showFrameInfo;
    bool recording;

public:
    GraphicsViewWidget(QWidget *parent = 0);
    ~GraphicsViewWidget();

private:
    MessageQueue messageQueue;
    StopWatch stopWatch;

    QPoint mouse; // current mouse location in pixel coordinates
    QPoint lastMouse; // last mouse location in pixel coordinates
    QPointF mouseVelocity; // estimated mouse velocity in pixels per second
    QPoint mouseClick; // mouse click location in pixel coordinates
    double mouseClickTimeStamp;
    double lastMouseUpdateTimeStamp;
    bool mouseDown;

    QTimer swipeFadeOutTimer;
    QPointF swipeFadeOutVelocity;
    double lastSwipeFadeOutTimeStamp;
    QPointF windowCenter;

    bool poseSelection;
    Pose2D selectedPose;

    bool boxSelection;
    Polygon selectedBox;
    Vec2 selectionP1;
    Vec2 selectionP2;

public slots:
    void init();
    void reset();
    void messageIn(QString m);
    void toggleAxis();
    void toggleGrid();
    void toggleRuler();
    void toggleFrameInfo();
    void startRecording();
    void stopRecording();
    void togglePoseSelection();
    void toggleBoxSelection();

signals:
    void poseSelected(const Pose2D& p);
    void boxSelected(const Polygon& b);

protected:
    void drawForeground(QPainter* painter, const QRectF& rect);
    void drawBackground(QPainter* painter, const QRectF& rect);
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);

private slots:
    void startSwipeFadeOut();
    void stopSwipeFadeOut();
    void swipeFadeOut();

private:
    void updateMouse(QPoint mousePos);
    void drawGrid(QPainter* painter, const QRectF& rect, double step = 1.0, QColor color = QColor(), Qt::PenStyle penStyle = Qt::SolidLine);
};

#endif // GRAPHICSVIEWWIDGET_H_

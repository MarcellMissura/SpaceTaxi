#ifndef GRAPHICSVIEWWIDGET_H_
#define GRAPHICSVIEWWIDGET_H_

#include <QtGui>
#include <QGraphicsView>
#include "MessageQueue.h"
#include "util/StopWatch.h"

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

protected:
    void drawForeground(QPainter* painter, const QRectF& rect);
    void drawBackground(QPainter* painter, const QRectF& rect);
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseDoubleClickEvent(QMouseEvent *event);

private:
    void updateMouse(QPoint mousePos);

private slots:
    void startSwipeFadeOut();
    void stopSwipeFadeOut();
    void swipeFadeOut();

private:
    void drawGrid(QPainter* painter, const QRectF& rect, double step = 1.0, QColor color = QColor(), Qt::PenStyle penStyle = Qt::SolidLine);
};

#endif // GRAPHICSVIEWWIDGET_H_

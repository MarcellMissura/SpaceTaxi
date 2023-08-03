#include "GraphicsViewWidget.h"
#include "blackboard/State.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"

GraphicsViewWidget::GraphicsViewWidget(QWidget *parent)
    : QGraphicsView(parent)
{
    setMouseTracking(true);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate); // Prevents tear when dragging an item.
    setRenderHint(QPainter::Antialiasing, true);

    scale(1, -1); // flip the y axis
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    showAxis = false;
    showGrid = false;
    showRuler = false;
    showFrameInfo = true;
    recording = false;

    mouseClickTimeStamp = 0;
    lastMouseUpdateTimeStamp = 0;
    mouseDown = false;

    swipeFadeOutTimer.setInterval(5);
    lastSwipeFadeOutTimeStamp = 0;
    connect(&swipeFadeOutTimer, SIGNAL(timeout()), this, SLOT(swipeFadeOut()));

    connect(&messageQueue, SIGNAL(updated()), this, SLOT(update()));
}

GraphicsViewWidget::~GraphicsViewWidget()
{

}

void GraphicsViewWidget::init()
{
    reset();
}

void GraphicsViewWidget::reset()
{
    stopSwipeFadeOut();
    fitInView(sceneRect(), Qt::KeepAspectRatio);
}

void GraphicsViewWidget::messageIn(QString m)
{
    messageQueue.messageIn(m);
    update();
}

void GraphicsViewWidget::startSwipeFadeOut()
{
    swipeFadeOutVelocity = mouseVelocity;
    lastSwipeFadeOutTimeStamp = stopWatch.programTime();
    windowCenter = mapToScene(width()/2, height()/2);
    swipeFadeOutTimer.start();
}

void GraphicsViewWidget::stopSwipeFadeOut()
{
    swipeFadeOutVelocity *= 0;
    swipeFadeOutTimer.stop();
}

void GraphicsViewWidget::startRecording()
{
    recording = true;
    update();
}

void GraphicsViewWidget::stopRecording()
{
    recording = false;
    update();
}

void GraphicsViewWidget::toggleAxis()
{
    showAxis = !showAxis;
    update();
}

void GraphicsViewWidget::toggleGrid()
{
    showGrid = !showGrid;
    update();
}

void GraphicsViewWidget::toggleRuler()
{
    showRuler = !showRuler;
    if (showRuler)
    {
        setMouseTracking(true);
        //setDragMode(QGraphicsView::NoDrag);
        setCursor(Qt::CrossCursor);
    }
    else
    {
        setMouseTracking(false);
        //setDragMode(QGraphicsView::ScrollHandDrag);
    }
    update();
}

// Toggles the frame info shown on the bottom.
void GraphicsViewWidget::toggleFrameInfo()
{
    showFrameInfo = !showFrameInfo;
    update();
}

// Handles the swipe fade out (inertial motion after the mouse button was released).
void GraphicsViewWidget::swipeFadeOut()
{
    if (swipeFadeOutVelocity.manhattanLength() < 0.85)
    {
        stopSwipeFadeOut();
        return;
    }

    double elapsedTime = (stopWatch.programTime() - lastSwipeFadeOutTimeStamp);
    QPointF mappedSwipeFadeOutVelocity = QPointF(swipeFadeOutVelocity.x()/transform().m11(), swipeFadeOutVelocity.y()/transform().m22());
    windowCenter -= mappedSwipeFadeOutVelocity*elapsedTime;
    centerOn(windowCenter);
    swipeFadeOutVelocity *= qMax(0.0, 1.0 - 5.0*elapsedTime);
    lastSwipeFadeOutTimeStamp = stopWatch.programTime();

    update();
}

// Updates the mouse state (position and velocity).
void GraphicsViewWidget::updateMouse(QPoint mousePos)
{
    mouse = mousePos;

    double timeDiff = (stopWatch.time()-lastMouseUpdateTimeStamp);
    if (timeDiff > 0.3)
    {
        mouseVelocity *= 0;
        lastMouse = mouse;
        lastMouseUpdateTimeStamp = stopWatch.time();
    }
    else if (timeDiff >= 0.003)
    {
        QPointF measuredMouseVelocity = (mouse - lastMouse)/timeDiff;
        mouseVelocity = 0.5*mouseVelocity + 0.5*measuredMouseVelocity;
        lastMouse = mouse;
        lastMouseUpdateTimeStamp = stopWatch.time();
    }
}

// Message queue, ruler, and the debug HUD.
// These are drawn on top of the graphics scene.
void GraphicsViewWidget::drawForeground(QPainter* painter, const QRectF& rect)
{
    StopWatch sw;
    sw.start();

    // Draw the world overlay.
    state.world.draw(painter);

    // The painter draws in scene coordinates by default, but now we want to draw in view coordinates.
    painter->resetTransform();

    // Show recording state.
    if (recording)
    {
        painter->save();
        painter->setFont(QFont("Helvetica", 18));
        painter->setPen(QColor::fromRgbF(0.8,0.3,0.3));
        painter->drawText(QPoint(18, 30), "Recording...");
        painter->restore();
    }

    // Draw the message queue.
    painter->save();
    painter->translate(18, 60);
    messageQueue.draw(painter, 85, 85, 0);
    painter->restore();

    // The ruler (coordinates shown next to the mouse pointer).
    if (showRuler)
    {
        QPointF mappedMouse = mapToScene(mouse);
        painter->drawText(mouse + QPoint(10, -10), "world: [" + QString::number(mappedMouse.x()) + ", " + QString::number(mappedMouse.y()) + "]");

        if (!state.world.unicycleAgents.isEmpty())
        {
            Vec2 locMouse = mapToScene(mouse);
            locMouse -= state.world.unicycleAgents.first().pose();
            painter->drawText(mouse + QPoint(10, 4), "local:   [" + QString::number(locMouse.x) + ", " + QString::number(locMouse.y) + "]");

            if (state.world.unicycleAgents.first().sensedGrid.contains(locMouse))
            {
                Vec2u mni = state.world.unicycleAgents.first().sensedGrid.getNodeIndex(locMouse);
                double v = state.world.unicycleAgents.first().sensedGrid.getAt(mni);
                painter->drawText(mouse + QPoint(10, 18), "grid:    [" + QString::number(mni.x) + ", " + QString::number(mni.y) + "] v:" + QString::number(v));
            }
        }
    }

    // Debug info like frame number, time, and a debug value.
    if (showFrameInfo)
    {
        painter->setFont(QFont("Helvetica", 14, QFont::Light));
        painter->setPen(QColor::fromRgbF(0.3,0.3,0.7));
        painter->drawText(QPoint(18, height()-10), "frame: " + QString::number(state.frameId) +
                    "  score: " + QString::number(state.uniTaxi.score) +
                          "  cols: " + QString::number(state.uniTaxi.collisions) +
                          "  stucks: " + QString::number(state.uniTaxi.stucks) +
                          "  closes: " + QString::number(state.uniTaxi.closes));
        if (state.iterationTime > 0)
            painter->drawText(QPoint(width()-55, height()-10), "x" + QString::number((1000.0/command.frequency)/state.iterationTime).left(4));
    }

    state.drawTime = sw.elapsedTimeMs();
}

// The axes and the grid are drawn on the background.
void GraphicsViewWidget::drawBackground(QPainter *painter, const QRectF &rect)
{
    // Draw the world underlay.
    state.world.drawBackground(painter);

    painter->setRenderHint(QPainter::Antialiasing, false);
    QPen pen(QColor(100, 10, 10));
    pen.setWidth(2);
    pen.setCosmetic(true);
    painter->setPen(pen);

    // Debug draw of window center and scene center.
    if (false)
    {
        QPointF windowCenter = mapToScene(width()/2, height()/2);
        painter->drawEllipse(windowCenter, 0.03, 0.03);
        QPointF sceneCenter = scene()->itemsBoundingRect().center();
        painter->drawEllipse(sceneCenter, 0.03, 0.03);
    }

    // draw grid
    if (showGrid)
    {
        if (transform().m11() > 150)
            drawGrid(painter, rect, 0.05, QColor(150, 150, 255, 100), Qt::DotLine);
        if (transform().m11() > 80)
            drawGrid(painter, rect, 0.1, QColor(150, 150, 255, 100), Qt::DashLine);
        if (transform().m11() > 4)
            drawGrid(painter, rect, 0.5, QColor(150, 150, 255, 100));
    }

    // draw axes
    if (showAxis)
    {
        painter->drawLine(QPointF(0, rect.top()), QPointF(0, rect.bottom()));
        painter->drawLine(QPointF(rect.left(), 0), QPointF(rect.right(), 0));
    }
}

void GraphicsViewWidget::drawGrid(QPainter *painter, const QRectF &rect, double step, QColor color, Qt::PenStyle penStyle)
{
    painter->save();

    QPen pen;
    pen.setColor(color);
    pen.setStyle(penStyle);
    pen.setCosmetic(true);
    painter->setPen(pen);

    double x = qRound(rect.left() / step) * step;
    double y = qRound(rect.top() / step) * step;
    while (x < rect.right())
    {
        painter->drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
        x += step;
    }
    while (y < rect.bottom())
    {
        painter->drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
        y += step;
    }

    painter->restore();
}

void GraphicsViewWidget::wheelEvent(QWheelEvent *event)
{
    if (event->delta() < 0)
        scale(0.83, 0.83);
    else
        scale(1.2, 1.2);
    update();
}

void GraphicsViewWidget::mousePressEvent(QMouseEvent *event)
{
//    QGraphicsScene* sc = scene();
//    qDebug() << "mouse click widget";
//    qDebug() << "There are" << items(event->pos()).size() << sc->items(mapToScene(event->pos())).size() << "items at";
//    qDebug() << "screen position" << event->pos();
//    qDebug() << "scene position" << mapToScene(event->pos());

    setDragMode(QGraphicsView::ScrollHandDrag);

    mouseClick = event->pos();
    mouseClickTimeStamp = stopWatch.programTime();
    mouseDown = (event->buttons() & (Qt::LeftButton | Qt::RightButton | Qt::MiddleButton));
    stopSwipeFadeOut();
    QGraphicsView::mousePressEvent(event);
}

void GraphicsViewWidget::mouseMoveEvent(QMouseEvent *event)
{
    updateMouse(event->pos());
    mouseDown = (event->buttons() & (Qt::LeftButton | Qt::RightButton | Qt::MiddleButton));
    QGraphicsView::mouseMoveEvent(event); // use the default drag implementation
    update();
}

void GraphicsViewWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    reset();
    update();
}

void GraphicsViewWidget::mouseReleaseEvent(QMouseEvent *event)
{
    // Start swipe fade out if the screen was dragged and released.
    if (mouse != mouseClick and event->button() == Qt::LeftButton and items(event->pos()).size() == 0)
    {
        updateMouse(event->pos());
        startSwipeFadeOut();
    }

    mouseDown = (event->buttons() & (Qt::LeftButton | Qt::RightButton | Qt::MiddleButton));

    setDragMode(QGraphicsView::NoDrag);
    QGraphicsView::mouseReleaseEvent(event);
}

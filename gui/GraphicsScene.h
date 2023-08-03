#ifndef GRAPHICSSCENE_H_
#define GRAPHICSSCENE_H_

#include <QtGui>
#include <QGraphicsRectItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsScene>
#include "lib/util/StopWatch.h"

class GraphicsScene : public QGraphicsScene
{
    Q_OBJECT

    QFont font;

public:
    GraphicsScene(QObject *parent = 0);
    ~GraphicsScene();

public slots:
    void init();
};

#endif // GRAPHICSSCENE_H_

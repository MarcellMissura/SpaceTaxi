#include "GraphicsScene.h"
#include "board/Config.h"
#include "board/State.h"
#include "board/Command.h"
#include "lib/util/DrawUtil.h"

// The GraphicsScene is a two dimensional scene where drawable objects are located and
// painted on a widget. The scene is always a part of a QGraphicsWidget widget.
// Currently, whenever it is time to redraw the scene, a call to init() is issued where
// the entire scene is cleared and rebuilt. After the scene is built, the drawing of the
// scene happens automatically by the graphics view framework.

// Currently, the GraphicsScene is not really used. All of the drawing happens in
// GraphicsViewWidget::drawForeground(). The GraphicsScene exists only to to keep the
// graphics view framework in the loop in case it will be needed again in the future.

GraphicsScene::GraphicsScene(QObject *parent)
    : QGraphicsScene(parent)
{
    font.setFamily("Arial");
    font.setPointSize(1);
}

GraphicsScene::~GraphicsScene()
{

}

// This is the place where the world objects are added as QGraphicsItems to the scene.
// This method clears the scene and rebuilds it entirely.
void GraphicsScene::init()
{
    clear();

    // Set the scene rect in order to have the view zoomed to the right distance that shows
    // all included obstacles. The world width and height only need to be approximate.
    setSceneRect(-0.2*state.world.width, -0.2*state.world.height, 1.3*state.world.width, 1.3*state.world.height);
    //addRect(sceneRect(), drawUtil.penLightGrayThinDashed, Qt::NoBrush);
}

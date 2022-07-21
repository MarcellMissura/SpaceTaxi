#include "GraphicsScene.h"
#include "globals.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "blackboard/Command.h"
#include "util/ColorUtil.h"

// The GraphicsScene is a two dimensional scene where drawable objects are located and
// painted on a widget. The scene is always a part of a QGraphicsWidget widget.
// Currently, whenever it is time to redraw the scene, this is indicated by a call to
// init(), the entire scene is cleared and rebuilt from the objects found
// in state.world. After the scene is built, the drawing of the scene happens
// automatically by the graphics view framework. The world obstacles, the agents,
// and the drop off points are drawn here.

// There is also some world stuff being drawn in world.draw(). There is no good reason
// to have a part of the world drawn in the graphics scene and another part using
// QPainter in world.draw(). It is now more or less just to keep the graphics view
// framework in the loop in case it is needed in the future.

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
    setSceneRect(-0.3*state.world.width, -0.3*state.world.height, 1.6*state.world.width, 1.6*state.world.height);
    //addRect(0, 0, state.world.width, state.world.height, colorUtil.pen, colorUtil.brushLightGray);
}

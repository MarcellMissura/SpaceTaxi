#include "GridSearchNode.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"

// The GridSearchNode class is used by A* searches in grid models.

// Default constructor used basically only to instantiate the root node.
GridSearchNode::GridSearchNode()
{
    n = 0;
    parent = 0;
    successor = 0;
    pidx = 0;
    closed = false;
    blocked = false;
    g = -1;
    f = 0;
    h = 0;
}

// Resets the node to a default state, where it can be used as a root node.
void GridSearchNode::reset()
{
    closed = false;
    blocked = false;
    parent = 0;
    successor = 0;
    h = 0;
    f = 0;
    g = -1;
}

// QPainter drawing code.
void GridSearchNode::draw(QPainter *painter) const
{
    if (parent == 0)
        return;

    double s = 0.03;
    painter->save();
    painter->setOpacity(0.7);
    painter->setPen(drawUtil.penGrayThin);
    painter->setBrush(drawUtil.brush);
    painter->drawLine(QLineF(parent->stateIdx.x, parent->stateIdx.y, stateIdx.x, stateIdx.y));
    painter->drawEllipse(QPointF(stateIdx.x, stateIdx.y), s, s);
    painter->restore();
}

// OpenGL drawing code.
void GridSearchNode::draw() const
{
    if (parent == 0)
        return;

    glBegin(GL_LINE);
    glVertex2d(parent->stateIdx.x, parent->stateIdx.y);
    glVertex2d(stateIdx.x, stateIdx.y);
    glEnd();

    glColor3d(0.1, 0.1, 0.1);
    glPushMatrix();
    glTranslated(stateIdx.x, stateIdx.y, 0);
    GLlib::drawFilledCircle(0.4);
    glPopMatrix();
}

QDebug operator<<(QDebug dbg, const GridSearchNode &n)
{
    dbg << "state:" << n.stateIdx
        << "g:" << n.g
        << "h:" << n.h
        << "f:" << n.f;

    return dbg;
}

QDebug operator<<(QDebug dbg, const GridSearchNode* n)
{
    dbg	<< "state:" << n->stateIdx
        << "g:" << n->g
        << "h:" << n->h
        << "f:" << n->f;
    return dbg;
}

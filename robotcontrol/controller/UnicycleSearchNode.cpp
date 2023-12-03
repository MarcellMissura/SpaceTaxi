#include "UnicycleSearchNode.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"
#include "board/Command.h"
#include "board/Config.h"

// The UnicycleSearchNode class is used by A* search.

// Default constructor used basically only to instantiate the root node.
UnicycleSearchNode::UnicycleSearchNode() : Unicycle()
{
    id = 0;
    depth = 0;
    parent = 0;
    pidx = 0;
    closed = false;
    collided = 0;
    g = 0;
    f = 0;
    h = 0;
    type = command.trajectoryType;
}

// Resets the node to a default state, where it can be used as a root node.
void UnicycleSearchNode::reset()
{
    id = 0;
    depth = 0;
    closed = false;
    collided = 0;
    parent = 0;
    h = 0;
    f = 0;
    g = -1;
}


// Sets the depth of the footstep in the search tree.
void UnicycleSearchNode::setDepth(uint d)
{
    depth = d;
}

// Returns the depth of the search node in the search tree.
uint UnicycleSearchNode::getDepth()
{
    return depth;
}

// Populates this search node from the given node u using the action acc applied for time dt.
void UnicycleSearchNode::propagate(const UnicycleSearchNode* u, const Vec2& acc, double dt)
{
    *this = *u;
    parent = u;
    depth = parent->depth+1;
    closed = false;
    collided = 0;
    action = acc;

    // Now generate the trajectory using the acceleration sample.
    if (type == command.Arc)
    {
        // Compute the velocity that results from integrating the acceleration for (half) dt.
        double vv = bound(config.agentLinearVelocityLimitBackward, v + acc.x*dt*0.5, config.agentLinearVelocityLimitForward);
        double ww = bound(-config.agentAngularVelocityLimit, w + acc.y*dt*0.5, config.agentAngularVelocityLimit);
        double v1 = bound(config.agentLinearVelocityLimitBackward, v + acc.x*dt, config.agentLinearVelocityLimitForward);
        double w1 = bound(-config.agentAngularVelocityLimit, w + acc.y*dt, config.agentAngularVelocityLimit);

        setVel(vv, ww); // Set approximate velocity.
        setAcc(0, 0); // Zero acceleration! This will make arcs.
        predict(dt); // Predict the new state.
        setVel(v1, w1); // Set exact velocity.
    }
    else if (type == command.B0)
    {
        // Compute the velocity that results from integrating the acceleration for (half) dt.
        double ww = bound(-config.agentAngularVelocityLimit, w + acc.y*dt*0.5, config.agentAngularVelocityLimit);
        double w1 = bound(-config.agentAngularVelocityLimit, w + acc.y*dt, config.agentAngularVelocityLimit);

        setVel(v, ww); // Set approximate velocity.
        setAcc(acc.x, 0); // Zero b, this will make b0 spirals.
        predict(dt); // Predict the new state.
        setVel(v, w1); // Set exact velocity.
    }
    else if (type == command.Fresnel)
    {
        setAcc(acc); // This will make Fresnel spirals.
        predict(dt); // Predict the new state precisely.
    }
}

// Flips the UnicycleSearchNode to a "head first" representation.
// The UnicycleSearchNodes represent a trajectory with the final
// state. But when drawing them, usually the first state of the
// trajectory is needed. This function will do that.
UnicycleSearchNode UnicycleSearchNode::flipped() const
{
    double dt = config.staaDt;
    double vv = bound(config.agentLinearVelocityLimitBackward, v + action.x*dt*0.5, config.agentLinearVelocityLimitForward);
    double ww = bound(-config.agentAngularVelocityLimit, w + action.y*dt*0.5, config.agentAngularVelocityLimit);
    //double v1 = bound(config.agentLinearVelocityLimitBackward, v + action.x*dt, config.agentLinearVelocityLimitForward);
    //double w1 = bound(-config.agentAngularVelocityLimit, w + action.y*dt, config.agentAngularVelocityLimit);

    UnicycleSearchNode u = *this;
    u.setPose(parent->pose());
    if (type == command.Arc)
        u.setVel(vv, ww);
    else if (type == command.B0)
        u.setVel(parent->v, ww);
    else if (type == command.Fresnel)
        u.setVel(parent->vel());
    qDebug() << "flipping this:" << *this;
    qDebug() << "to this:" << u;
    qDebug() << "to pred:" << u.predicted(dt);
    return u;
}

// Returns the trace of nodes back to the root as a Vector of node objects.
// The Vector is in reversed order, i.e., this node comes first, then its
// parent and so on. The first action from the root is in last position
// and the root itself is not contained.
Vector<UnicycleSearchNode> UnicycleSearchNode::trace() const
{
    Vector<UnicycleSearchNode> v;
    v << *this;
    if (depth > 0)
        v << parent->trace();
    return v;
}

// Returns the first action in the search tree that leads to this node.
Vec2 UnicycleSearchNode::rootAction() const
{
    UnicycleSearchNode const * node = this;
    while (node->depth > 1)
        node = node->parent;
    return node->action;
}

// Draws the UnicycleSearchNode in an OpenGL context.
void UnicycleSearchNode::draw() const
{
    // Flip the UnicycleSearchNode to a "head first" representation.
    // The UnicycleSearchNodes represent a trajectory with the final
    // state. But when drawing them, the first state of the trajectory
    // is needed.
    Unicycle u = *this;
    u.setPose(parent->pose());
    double v05 = bound(config.agentLinearVelocityLimitBackward, parent->v + action.x*dt*0.5, config.agentLinearVelocityLimitForward);
    double w05 = bound(-config.agentAngularVelocityLimit, parent->w + action.y*dt*0.5, config.agentAngularVelocityLimit);
    u.setVel(v05, w05);
    u.setAcc(0,0);
    u.draw();
}

// Draws the UnicycleSearchNode on a QPainter.
void UnicycleSearchNode::draw(QPainter* painter) const
{
    UnicycleSearchNode u = *this;
    u.setPose(parent->pose());
    double v05 = bound(config.agentLinearVelocityLimitBackward, parent->v + action.x*dt*0.5, config.agentLinearVelocityLimitForward);
    double w05 = bound(-config.agentAngularVelocityLimit, parent->w + action.y*dt*0.5, config.agentAngularVelocityLimit);
    u.setVel(v05, w05);
    u.setAcc(0,0);
    u.draw(painter);
}

void UnicycleSearchNode::drawNoseCircle(QPainter *painter, double r, QBrush brush) const
{
    painter->save();
    painter->translate(pos());
    painter->rotate(heading()*RAD_TO_DEG);
    painter->setPen(drawUtil.penThin);
    painter->setBrush(brush);
    painter->drawEllipse(QPointF(), r, r);
    painter->drawLine(QPointF(), QPointF(2*r, 0));

    // Node label.
    if (command.showLabels > 0)
    {
        QFont font;
        font.setFamily("Arial");
        font.setPointSize(1);
        painter->scale(-0.01, 0.01);
        painter->setFont(font);
        painter->setOpacity(0.8);
        painter->drawText(QPointF(), QString::number(id));
    }

    painter->restore();
}

void UnicycleSearchNode::drawTrajectory(QPainter *painter) const
{
    painter->save();
    painter->setBrush(Qt::NoBrush);

    double dt = config.staaDt;
    double vv = bound(config.agentLinearVelocityLimitBackward, parent->v + action.x*dt*0.5, config.agentLinearVelocityLimitForward);
    double ww = bound(-config.agentAngularVelocityLimit, parent->w + action.y*dt*0.5, config.agentAngularVelocityLimit);

    UnicycleSearchNode pu = *this;
    pu.setPose(parent->pose());
    if (type == command.Arc)
    {
        pu.setVel(vv, ww);
        pu.setAcc(0, 0); // Zero acceleration! This will make arcs.
    }
    else if (type == command.B0)
        pu.setVel(parent->v, ww);
    else if (type == command.Fresnel)
        pu.setVel(parent->vel());

    //qDebug() << "this pos:" << pos() << "parent" << parent->pos() << "u pose:" << u.pos();
    //qDebug() << pose() << "should be equal to" << u.predicted(dt).pose();

    // Draw the trajectory backwards. It gets rid of an unwanted visual effect.
    double ddt = 0.05;
    double ct = dt;
    QPainterPath pp;
    pp.moveTo(pu.predicted(ct).pos()); // same as this pose
    while (ct > ddt)
    {
        ct -= ddt;
        pp.lineTo(pu.predicted(ct).pos());
    }
    pp.lineTo(pu.pos());
    painter->drawPath(pp);


    // The switching points.
    double s = 0.01;
    painter->drawEllipse(pos(), s, s);
    painter->drawEllipse(pu.predicted(dt).pos(), s, s);

    painter->restore();
}

QDebug operator<<(QDebug dbg, const UnicycleSearchNode &n)
{
    dbg << "id:" << n.id
        << "depth:" << n.depth
        //<< "collided:" << int(n.collided)
        << "action:" << n.action
        << "pose:" << n.pose();
        //<< "g:" << n.g
        //<< "h:" << n.h
        //<< "f:" << n.f;

    return dbg;
}

QDebug operator<<(QDebug dbg, const UnicycleSearchNode* n)
{
    dbg	<< "id:" << n->id
        << "depth:" << n->depth
        << "action:" << n->action
        << "pose:" << n->pose();
        //<< "g:" << n->g
        //<< "h:" << n->h
        //<< "f:" << n->f;
    return dbg;
}

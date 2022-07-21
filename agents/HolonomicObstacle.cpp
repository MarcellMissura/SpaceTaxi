#include "HolonomicObstacle.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "util/ColorUtil.h"

// The HolonomicObstacle class represents a shape that moves like
// the Synchronized Holonomic Model. It has velocity and acceleration
// components in the x and y directions. It implements the
// Synchronized Holonomic Model for state prediction and collision
// checking. The physics in and out interface is also specific to
// this holonomic motion and has been reimplemented for the
// HolonomicObstacle.

HolonomicObstacle::HolonomicObstacle() : Obstacle()
{
    vx = 0;
    vy = 0;
    ax = 0;
    ay = 0;
}

HolonomicObstacle::HolonomicObstacle(const Hpm2D& v) : Obstacle()
{
    *this = v;
}

HolonomicObstacle& HolonomicObstacle::operator=(const Hpm2D& v)
{
    setPos(v.pos());
    setVel(v.vel());
    setAcc(v.acc());
    return *this;
}

// Converts this HolonomicObstacle to a Hpm2D.
HolonomicObstacle::operator Hpm2D() const
{
    Hpm2D h;
    h.setPos(x,y);
    h.setVel(vx,vy);
    h.setAcc(ax,ay);
    return h;
}

// Generates a human readable name.
const QString HolonomicObstacle::getName() const
{
    QString name = "HolonomicObstacle" + QString::number(getId());
    return name;
}

// Updates the internal state from the physical body after a simulation step.
void HolonomicObstacle::physicsTransformIn()
{
    Obstacle::physicsTransformIn();
    setVel(body->GetLinearVelocity().x, body->GetLinearVelocity().y);
}

// Applies the controls to the physical body before a simulation step.
void HolonomicObstacle::physicsControl()
{
    // Apply control force to taxi.
    // The mass of the body has to be 1.0 so that acceleration = force.
    body->ApplyForceToCenter(b2Vec2(ax, ay), true);
}

// Updates the state of the body in the physical simulation from the
// state of this object through the body pointer. This is used to load
// historical states.
void HolonomicObstacle::physicsTransformOut()
{
    Obstacle::physicsTransformOut();
    b2Vec2 v(vx, vy);
    body->SetLinearVelocity(v);
}

// Returns the velocity of the HolonomicObstacle.
Vec2 HolonomicObstacle::vel() const
{
    return Vec2(vx,vy);
}

// Sets the velocity vector of the HolonomicObstacle.
void HolonomicObstacle::setVel(const Vec2 &v)
{
    vx = v.x;
    vy = v.y;
}

// Sets the velocity vector of the HolonomicObstacle.
void HolonomicObstacle::setVel(double vx, double vy)
{
    this->vx = vx;
    this->vy = vy;
}

// Returns the acceleration of the HolonomicObstacle.
Vec2 HolonomicObstacle::acc() const
{
    return Vec2(ax,ay);
}

// Sets the acceleration of the HolonomicObstacle.
void HolonomicObstacle::setAcc(const Vec2 &c)
{
    ax = c.x;
    ay = c.y;
}

// Sets the acceleration of the HolonomicObstacle.
void HolonomicObstacle::setAcc(double ax, double ay)
{
    this->ax = ax;
    this->ay = ay;
}

// The predict method forwards this object in time by time dt.
void HolonomicObstacle::predict(double dt)
{
    Hpm2D h = *this;
    h.predict(dt);
    *this = h;
}

// Returns a forwarded object in time by time dt.
HolonomicObstacle HolonomicObstacle::predicted(double dt) const
{
    HolonomicObstacle v = *this;
    v.predict(dt);
    return v;
}

// Performs a collision check between this holonomic (moving) obstacle and
// the holonomic bang described by the kf. If no collision occurs between
// the bang and this obstacle, -1 is returned. Otherwise the relative time
// is returned to indicate the future time of collision relative to the
// start of the bang. If the collision time is later than the dt in the
// keyframe, -1 is returned.
double HolonomicObstacle::intersects(const Hpm2D &kf) const
{
    //qDebug() << "  HolonomicObstacle::intersects(Hpm2D):" << kf;

    // Trick: we substract the acceleration and the velocity of
    // the obstacle from the keyframe so that we can regard the
    // obstacle as static. This way we can reuse the intersect
    // method in the deepest layer including the bounding box
    // rejection test.
    Hpm2D kf2 = kf;
    kf2.ax -= ax;
    kf2.ay -= ay;
    kf2.vx -= vx;
    kf2.vy -= vy;
    return Obstacle::intersects(kf2);
}

// Performs a collision check between this holonomic (moving) obstacle
// and the unicycle bang described by u. If no collision occurs between
// the bang and this obstacle, -1 is returned. Otherwise the time dt is
// returned to indicate the future time of collision relative to the
// start of this bang. If the collision time is later than the dt in u,
// -1 is returned.
double HolonomicObstacle::intersects(const Unicycle &u) const
{
    //if (config.debugLevel > 0)
    //qDebug() << "        HolonomicObstacle::intersects(Unicycle) id:" << getId() << "u:" << u;

    // Swept volume test.
    //Box sw = sweptVolume(u.dt);
    //if (!sw.intersects(u.boundingBox()))
    //    return -1;

    // Intersect every edge with the unicycle trajectory.
    double ct = -1;
    ListIterator<Line> it = edgeIterator();
    while (it.hasNext())
    {
        const Line& edge = it.next();
        double cct = u.intersects(edge, vel());
        if (cct >= 0 && (ct < 0 || cct < ct))
            ct = cct;
    }

    return ct;
}

// Returns true if the future location of this obstacle at time dt contains the point v.
bool HolonomicObstacle::dynamicPointIntersection(const Vec2 &v, double dt) const
{
    HolonomicObstacle h;
    h = *this;
    h.predict(dt);
    return ((Obstacle*)&h)->intersects(v);
}

// Returns true if the future location of this obstacle at time dt collides with polygon p.
bool HolonomicObstacle::dynamicPolygonIntersection(const Polygon &p, double dt) const
{
    HolonomicObstacle h;
    h = *this;
    h.predict(dt);
    h.transform();
    return ((Obstacle*)&h)->intersects(p);
}

// Returns the bounding box of the volume swept by the holonomic obstacle until time dt.
Box HolonomicObstacle::sweptVolume(double dt) const
{
    // The technique being used here is to compute the Minkowski sum of the
    // bounding box of the polygon and the bounding box of the holonomic trajectory.
    Box bb = Obstacle::boundingBox();
    Hpm2D h = *this;
    h.dt = dt;
    const Box& sw = h.boundingBox();
    bb += sw;
    return bb;
}

QDebug operator<<(QDebug dbg, const HolonomicObstacle &o)
{
    dbg << o.getId() << o.getName() << "pos:" << o.pos() << "vel:" << o.vel();
    return dbg;
}

QDataStream& operator<<(QDataStream& out, const HolonomicObstacle &o)
{
    out << ((Obstacle&)(o));
    out << o.vx;
    out << o.vy;
    out << o.ax;
    out << o.ay;
    return out;
}

QDataStream& operator>>(QDataStream& in, HolonomicObstacle &o)
{
    in >> ((Obstacle&)(o));
    in >> o.vx;
    in >> o.vy;
    in >> o.ax;
    in >> o.ay;
    return in;
}

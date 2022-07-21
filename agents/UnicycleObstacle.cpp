#include "UnicycleObstacle.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"

// The UnicycleObstacle class represents objects that move in a car-like manner.
// A car's lateral velocity is always zero. A car's state is the position (x,y),
// the orientation theta, the forward velocity v, and an angular velocity w (omega).
// The controls of the car are a, which is the acceleration of the forward velocity,
// and b, which is the acceleration of the angular velocity. A car cannot change
// its angular velocity instantly. A car is similar to the unicycle, except that
// it has a minimal turning radius, and the forward velocity v and the angular
// velocity w are connected in a way that the car does not turn unless it drives
// forward. The UnicycleObstacle inherits from Obstacle and overrides some of its
// methods to specifiy the handling of a unicycle in the physical simulation.

UnicycleObstacle::UnicycleObstacle() : Obstacle()
{
    v = 0;
    w = 0;
    a = 0;
    b = 0;
    timeStep = config.rcIterationTime;
}

// Conversion from UnicycleObstale to HolonomicObstacle.
UnicycleObstacle::operator HolonomicObstacle() const
{
    HolonomicObstacle o;
    o.setId(getId());
    o.setPos(pos());
    o.setRotation(rotation());
    o.setVel(v*cos(theta), v*sin(theta));
    ListIterator<Vec2> li = vertexIterator();
    while (li.hasNext())
        o.addVertex(li.next());
    return o;
}

// Copy constructor from Unicycle (pml) object.
UnicycleObstacle::UnicycleObstacle(const Unicycle& o) : Obstacle()
{
    *this = o;
}

// Assignment operator for Unicycle (pml) object.
UnicycleObstacle& UnicycleObstacle::operator=(const Unicycle& o)
{
    x = o.x;
    y = o.y;
    theta = o.theta;
    v = o.v;
    w = o.w;
    a = o.a;
    b = o.b;
    boundingBoxValid = false;

    return *this;
}

// Conversion to Unicycle object (pml).
UnicycleObstacle::operator Unicycle() const
{
    Unicycle u;
    u.x = x;
    u.y = y;
    u.theta = theta;
    u.v = v;
    u.w = w;
    u.a = a;
    u.b = b;
    return u;
}

// Initializes the unicycle obstacle.
void UnicycleObstacle::init()
{
    double w = config.agentWidth;
    double h = config.agentHeight;

    vertices << Vec2(-w, h);
    vertices << Vec2(-w, -h);
    vertices << Vec2(w, -h);
    vertices << Vec2(w, h);
    boundingBoxValid = false;
}

// Generates a human readable name.
const QString UnicycleObstacle::getName() const
{
    QString name = "UnicycleObstacle" + QString::number(getId());
    return name;
}

// Updates the internal state from the physical body after a simulation step.
void UnicycleObstacle::physicsTransformIn()
{
    // Car physics with Box2D according to http://www.iforce2d.net/b2dtut/top-down-car

    // Transform the world velocity vector into the body frame of the car.
    b2Vec2 forwardNormal = body->GetWorldVector(b2Vec2(1,0));
    b2Vec2 rightNormal = body->GetWorldVector(b2Vec2(0,1));
    double forwardVelocity = b2Dot(forwardNormal, body->GetLinearVelocity());
    b2Vec2 lateralVelocity = b2Dot(rightNormal, body->GetLinearVelocity())*rightNormal;

    // Cancels the lateral motion with the opposite impulse.
    body->ApplyLinearImpulseToCenter(-body->GetMass()*lateralVelocity, true);

    Obstacle::physicsTransformIn(); // pos and rotation
    setVel(forwardVelocity, body->GetAngularVelocity());

    //qDebug() << "pos:" << pos() << "vel:" << vel();
}

// Applies the controls to the physical body before a simulation step.
void UnicycleObstacle::physicsControl()
{
    // Car physics with Box2D according to http://www.iforce2d.net/b2dtut/top-down-car
    // All lateral forces are cancelled out and only the forward acceleration is applied.
    // Also a torque is applied to rotate the body.
    // The mass of the body has to be 1.0 so that acceleration = force.
    b2Vec2 forwardNormal = body->GetWorldVector(b2Vec2(1,0));
    body->ApplyForceToCenter(a*forwardNormal, true);
    body->ApplyTorque(b, true);
}

// Updates the state of the body in the physical simulation from the
// state of this object through the body pointer. This is used to load
// historical states.
void UnicycleObstacle::physicsTransformOut()
{
    Obstacle::physicsTransformOut();
    b2Vec2 vv(v*cos(theta), v*sin(theta));
    body->SetLinearVelocity(vv);
    body->SetAngularVelocity(w);
}


// Returns the velocity of the UnicycleObstacle. The velocity of the car obstacle
// is a vector of the linear and angular velocities v and omega.
Vec2 UnicycleObstacle::vel() const
{
    return Vec2(v, w);
}

// Sets the velocity of the UnicycleObstacle. The velocity of the car obstacle
// is a vector of the linear and angular velocities v and omega.
void UnicycleObstacle::setVel(const Vec2 &v)
{
    this->v = v.x;
    this->w = v.y;
    boundingBoxValid = false;
}

// Sets the velocity of the UnicycleObstacle. The velocity of the car obstacle
// is a vector of the linear and angular velocities v and omega.
void UnicycleObstacle::setVel(double v, double omega)
{
    this->v = v;
    this->w = omega;
    boundingBoxValid = false;
}

// Returns the acceleration of the UnicycleObstacle. The acceleration of the car
// obstacle are the forward acceleration a and the angular acceleration b.
Vec2 UnicycleObstacle::acc() const
{
    return Vec2(a, b);
}

// Returns the acceleration of the UnicycleObstacle. The acceleration of the car
// obstacle are the forward acceleration a and the angular acceleration b.
void UnicycleObstacle::setAcc(const Vec2 &c)
{
    setAcc(c.x, c.y);
}

// Sets the acceleration of the UnicycleObstacle.
void UnicycleObstacle::setAcc(double a, double b)
{
    // Smooth bound checks on a and b.
    a = qBound(-config.agentLinearAccelerationLimit, a, config.agentLinearAccelerationLimit);
    if (v + a*timeStep > config.agentLinearVelocityLimitForward)
        a = (config.agentLinearVelocityLimitForward-v)/timeStep;
    else if (v + a*timeStep < config.agentLinearVelocityLimitBackward)
        a = (config.agentLinearVelocityLimitBackward-v)/timeStep;
    b = qBound(-config.agentAngularAccelerationLimit, b, config.agentAngularAccelerationLimit);
    if (fabs(w + b*timeStep) > config.agentAngularVelocityLimit)
        b = sgn(b)*(config.agentAngularVelocityLimit-fabs(w))/timeStep;

    this->a = a;
    this->b = b;
    boundingBoxValid = false;
}

// Returns a forwarded object in time by dt.
UnicycleObstacle UnicycleObstacle::predicted(double dt) const
{
    UnicycleObstacle v = *this;
    v.predict(dt);
    return v;
}

// Predicts and updates the state of the unicycle to a future state at time dt.
void UnicycleObstacle::predict(double dt)
{
    Unicycle u = *this;
    u.predict(dt);
    *this = u;
}

// Performs a collision check against the holonomic bang described by the kf.
// If no collision occurs between the bang and this obstacle, -1 is returned.
// Otherwise the relative time dt is returned to indicate the future time of
// collision relative to this bang. If the collision time is later than the dt
// in the Hpm2D keyframe, -1 is returned.
double UnicycleObstacle::intersects(const Hpm2D &kf) const
{
    // We treat the intersection between a unicycle obstacle and a holonomic
    // bang such that we transfer the polygon from the unicycle to the
    // holonomic motion and then use the holo-uni intersection method.
    HolonomicObstacle hol = *this;
    hol.setPos(kf.pos());
    hol.setVel(kf.vel());
    return hol.intersects(*this);
}

// Unicycle-unicycle collision checks are not implemented.
double UnicycleObstacle::intersects(const Unicycle &u) const
{
    return -1;
}

// Returns true if the future location of this obstacle at time dt contains the point v.
bool UnicycleObstacle::dynamicPointIntersection(const Vec2 &v, double dt) const
{
    UnicycleObstacle u;
    u = *this;
    u.predict(dt);
    return ((Obstacle*)&u)->intersects(v);
}

// Returns true if the future location of this obstacle at time dt collides with polygon p.
bool UnicycleObstacle::dynamicPolygonIntersection(const Polygon &p, double dt) const
{
    UnicycleObstacle u;
    u = *this;
    u.predict(dt);
    u.transform();
    return ((Obstacle*)&u)->intersects(p);
}

// Returns the bounding box of this unicycle obstacle.
// The bounding box of an obstacle in motion is the bounding box of the swept volume.
const Box &UnicycleObstacle::boundingBox() const
{
    // The technique being used here is to compute the Minkowski sum of the
    // bounding box of the polygon and the bounding box of the trajectory.
    if (boundingBoxValid)
        return aabb;

    Obstacle::boundingBox();
    Unicycle u = *this;
    u.dt = config.DWA_time;
    const Box& sub = u.boundingBox();
    aabb += sub;
    return aabb;
}

QDebug operator<<(QDebug dbg, const UnicycleObstacle &o)
{
    dbg << o.getId() << o.getName() << "pose:" << o.pose() << "vel:" << o.vel();
    return dbg;
}

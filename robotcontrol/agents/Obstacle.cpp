#include "Obstacle.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "lib/util/ColorUtil.h"

// The Obstacle class represents a static obstacle that is a Polygon and
// can be added to the physical simulation. It extends the Polygon class
// with a readable name and a body pointer to the corresponding body in
// the Box2D simulation.
// The Obstacle is intended to define structure and to be a base class
// for objects with different types of motion, e.g. the synchronized
// holonomic model (see HolonomicObstacle) or a unicycle motion model
// (the UnicycleObstacle). It defines an interface for these subclasses
// for predicting a future state, collision checking against various forms
// of motion, collision response, and data exchange with the physical
// simulation. The physicsIn() and physicsOut() methods handle the
// communication with the physical simulation. Overriding the
// collisionResponse() callback function allows customizted collision response.
// The collision response function can be used to modify physical properties
// (velocity, position), but also to drive the game logic when implemented in
// agents on a higher level of the Obstacle class hierarchy. For example, the
// passenger pickup is triggered by a collision between the passenger and the
// taxi. The predict() function computes the future state of an obstacle
// according to its specific motion model. For the Obstacle class that is
// considered to be static, the motion model does nothing. And of course,
// there is always draw() for visualization.

Obstacle::Obstacle() : Polygon()
{
    active = true;
}

// Construct as box constructor for convenience.
Obstacle::Obstacle(double x, double y, double w, double h) : Polygon(x, y, w, h)
{
    active = true;
}

// Init function called once after initialization.
void Obstacle::init()
{

}

// Reset function for resets between iterations.
void Obstacle::reset()
{

}

// Generates a human readable name.
const QString Obstacle::getName() const
{
    QString name = "Obstacle" + QString::number(getId());
    return name;
}

// Activates (unhides) this object.
void Obstacle::activate()
{
    active = true;
    body->SetActive(true);
}

// Deactivates (hides) this object. It will no longer participate in
// simulation until it is activated again.
void Obstacle::deactivate()
{
    active = false;
    if (body > 0)
    {
        body->SetLinearVelocity(b2Vec2(0, 0));
        body->SetActive(false);
    }
}

// Tells you if the object is active or not.
bool Obstacle::isActive() const
{
    return active;
}

// Updates the internal state after a simulation step. The new state is
// read from the body pointer, which provides access to the physical
// body in the Box2D simulation.
void Obstacle::physicsTransformIn()
{
    // Basically, we read the position and the orientation of the body
    // in the simulation and write them into the internal representation
    // (x, y, theta).
    setPose(body->GetPosition().x, body->GetPosition().y, picut(body->GetAngle()));
}

// Applies the controls to the physical body before a simulation step.
// Typically this will be applying forces to the body that realize the
// acceleration computed by the motion controller.
void Obstacle::physicsControl()
{
    // The Obstacle is static and does nothing. Subclasses override this
    // function and do motion type specific things to control the body in
    // the simulation.
}

// Updates the state of the body in the physical simulation from the
// state of this object through the body pointer. This is used to load
// historical states.
void Obstacle::physicsTransformOut()
{
    b2Vec2 p(x, y);
    body->SetTransform(p, theta);
}

// This method is called when a collision involving this obstacle occured
// in the physical simulation. The parameter is a const pointer to the
// other obstacle that was involved in the collision.
void Obstacle::collisionResponse(const Obstacle *o)
{

}

// The predict function updates the motion state (position and velocity)
// of the Obstacle according to the implemented motion model. The Obstacle
// is static and, thus, does nothing, but subclasses would overwrite this
// method and implement a specific motion model.
void Obstacle::predict(double dt)
{
    //qDebug() << "Obstacle::predict():" << dt;
}

// Sames as predict, but it returns a copy of the future state.
Obstacle Obstacle::predicted(double dt) const
{
    Obstacle v = *this;
    v.predict(dt);
    return v;
}

QDebug operator<<(QDebug dbg, const Obstacle &o)
{
    if (dbg.autoInsertSpaces())
        dbg << o.getId() << o.getName() << (Polygon&)o;
    else
        dbg << o.getId() << " " << o.getName() << " " << (Polygon&)o;
    return dbg;
}

QDebug operator<<(QDebug dbg, const Obstacle* o)
{
    if (dbg.autoInsertSpaces())
        dbg << o->getId() << o->getName() << (Polygon&)(*o);
    else
        dbg << o->getId() << " " << o->getName() << " " << (Polygon&)(*o);
    return dbg;
}

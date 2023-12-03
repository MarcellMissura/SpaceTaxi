#include "Bezier2D.h"
#include "lib/util/DrawUtil.h"

// This is a Bezier curve-based non-linear keyframe interpolator.
// https://en.wikipedia.org/wiki/B%C3%A9zier_curve

Bezier2D::Bezier2D()
{

}

// Clears the keyframe player. It deletes all keyframes.
void Bezier2D::clear()
{
    keyframes.clear();
    P1.setNull();
    P2.setNull();
}

// Clears the keyframe player. It deletes all keyframes.
void Bezier2D::reset()
{
    clear();
}

// Sets the control point P1.
void Bezier2D::setP1(const Vec2& p1)
{
    P1 = p1;
}

// Sets the control point P2. P2 is not required. If it
// is not set, it is automatically ignored.
void Bezier2D::setP2(const Vec2& p2)
{
    P2 = p2;
}

// Appends a set of keyframes to the motion sequence.
void Bezier2D::addKeyframes(const Vector<Keyframe2D> &keyframes)
{
    for (int i = 0; i < keyframes.size(); i++)
        addKeyframe(keyframes[i]);
}

// Appends a keyframe to the motion sequence.
void Bezier2D::addKeyframe(const Keyframe2D &kf)
{
    keyframes << kf;
}

// Adds a keyframe to the motion sequence. dt is the relative time with respect to
// the previous frame, x and vx are the position and the velocity in the x direction,
// and y and vy are the position and the velocity in the y dimension.
void Bezier2D::addKeyframe(double dt, double x, double vx, double y, double vy)
{
    Keyframe2D kf(dt, x, y, vx, vy);
    addKeyframe(kf);
}

// Evaluates the Bezier curve at time dt in [0,1] and returns the calculated motion state.
Keyframe2D Bezier2D::evaluateAt(double dt) const
{
    if (keyframes.size() < 2)
        return Keyframe2D();

    if (!P2.isNull())
    {
        Vec2 P0 = keyframes[0].pos();
        Vec2 P3 = keyframes.last().pos();
        Vec2 P0P1 = P0 + dt*(P1-P0);
        Vec2 P1P2 = P1 + dt*(P2-P1);
        Vec2 P2P3 = P2 + dt*(P3-P2);
        Vec2 P0P2 = P0P1 + dt*(P1P2-P0P1);
        Vec2 P1P3 = P1P2 + dt*(P2P3-P1P2);
        Vec2 P0P3 = P0P2 + dt*(P1P3-P0P2);
        Keyframe2D f;
        f.setPos(P0P3);
        return f;
    }
    else
    {
        Vec2 P0 = keyframes[0].pos();
        Vec2 P3 = keyframes.last().pos();
        Vec2 P0P1 = P0 + dt*(P1-P0);
        Vec2 P1P3 = P1 + dt*(P3-P1);
        Vec2 P0P3 = P0P1 + dt*(P1P3-P0P1);
        Keyframe2D f;
        f.setPos(P0P3);
        return f;
    }
}

// Generic QPainter paint code.
void Bezier2D::draw(QPainter *painter, const QPen &pen) const
{
    if (keyframes.size() < 2)
        return;

    painter->save();
    painter->setPen(pen);

    // Draw the motion trajectory.
    double stepSize = 0.01;
    Keyframe2D oldState = keyframes[0];
    for (double t = stepSize; t <= 1.0; t=t+stepSize)
    {
        Keyframe2D newState = evaluateAt(t);
        if (oldState.pos() != newState.pos())
            painter->drawLine(oldState.pos(), newState.pos());
        oldState = newState;
    }

    // Draw the Bezier polygon.
    painter->setPen(drawUtil.penGrayThinDashed);
    painter->drawLine(keyframes[0].pos(), P1);
    if (!P2.isNull() && P2 != P1)
    {
        painter->drawLine(P1, P2);
        painter->drawLine(P2, keyframes[1].pos());
    }
    else
        painter->drawLine(P1, keyframes[1].pos());

    // Draw circles where needed.
    double s = 0.03;
    painter->setBrush(pen.color());
    painter->drawEllipse(keyframes[0].pos(), s, s);
    painter->drawEllipse(P1, s, s);
    if (!P2.isNull())
        painter->drawEllipse(P2, s, s);
    painter->drawEllipse(keyframes[1].pos(), s, s);

    painter->restore();
}

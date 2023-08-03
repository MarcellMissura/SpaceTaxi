#include "VelocityProfile.h"
#include "blackboard/Config.h"

// Instantiates a blank keyframe player with acc and vel bounds set to 0.
VelocityProfile::VelocityProfile()
{
}

// Using a line velocity model, returns a waypoint that is "time" time away
// along the path from the current state. If the path is less than time long,
// the last point of the path is returned.
Hpm2D VelocityProfile::getWaypoint(const Hpm2D& start, const Vector<Vec2>& path, double time) const
{
//    qDebug() << "Planning for path:" << path;
//    qDebug() << "Start:" << start;
//    qDebug() << "Time:" << time;

    if (path.isEmpty())
        return Hpm2D();

    // Data structure for the velocity profile.
    Vector<double> V(path.length());

    // Project the current velocity onto the path. This will be the velocity in the first vertex.
    Vec2 section = path[1]-path[0]; // current section vector
    Vec2 startVel = start.vel();
    Vec2 projectedStartVel = startVel;
    projectedStartVel.projectOnVector(section);
    double v0 = sgn(section*startVel)*projectedStartVel.norm(); // current velocity projected onto the section vector
    V[0] = v0;

    V[0] = max(V[0], 0.0); // Hack needed to help dumb controllers to turn around.

    // Compute angle based velocity bounds for all in-between vertices of the path.
    // For now this is a simple linear model such that the maximum velocity is allowed at an angle of 0,
    // and it linearly decreases to a 0 velocity at an angle of PI half (90 degrees).
    for (int i = 1; i < path.length()-1; i++)
    {
        Vec2 v1 = path[i]-path[i-1];
        Vec2 v2 = path[i+1]-path[i];
        double alpha = v1.angleTo(v2);
        V[i] = config.agentLinearVelocityLimitForward*max(1.0-fabs(alpha)/PI2, 0.0); // The velocity limit is enforced here as well.
    }

    // And finally, the velocity in the last vertex is zero.
    V[path.length()-1] = 0;

    // Propagate the velocity bounds to the neighbours of each path node to ensure dynamic feasibility
    // in terms of the acceleration limit.
    for (int i = 0; i < path.size()-1; i++)
    {
        double dx = (path[i+1]-path[i]).norm();
        double adx = 2*dx*config.agentLinearAccelerationLimit;
        double dv = V[i+1]*V[i+1]-V[i]*V[i];

        // Forward propagation (easy).
        // If there is not enough space to accelerate from vi up to vi+1, vi+1 has to be reduced.
        if (dv > adx)
            V[i+1] = sqrt(V[i]*V[i] + adx);

        // Backward propagation (difficult).
        // If there is not enough space to decelerate from vi down to vi+1, vi has to be reduced.
        // But this has an influence on the path node before, and that on the one before that.
        // The propagation might have to touch all nodes visited so far.
        else if (dv < -adx)
        {
            V[i] = sqrt(V[i+1]*V[i+1] + adx);

            for (int j = i-1; j >= 0; j--)
            {
                double dx = (path[j+1]-path[j]).norm();
                double adx = 2*dx*config.agentLinearAccelerationLimit;
                double dv = V[j+1]*V[j+1]-V[j]*V[j];
                if (dv < -adx)
                {
                    V[j] = sqrt(V[j+1]*V[j+1] + adx);
                }
                else
                {
                    break;
                }
            }
        }
    }

    // Determine the residual.
    if (v0 > V[0])
    {
        double residual = v0 - V[0];
        //qDebug() << "Residual in v0!" << v0 << V[0];
    }

    // Determine the section that the desired waypoint is in and compute the actual waypoint.
    BangBang b;
    b.setA(config.agentLinearAccelerationLimit);
    b.setV(config.agentLinearVelocityLimitForward);
    double waypointTime = time;
    for (int i = 0; i < path.size()-1; i++)
    {
        double l = (path[i+1]-path[i]).norm();

        b.clear();
        b.addKeyframe(0, 0, V[i]);
        b.addKeyframe(0, l, V[i+1]);
        Vector<Keyframe> ctrl = b.getTimeOptimalControlSequence();
        double sectionTime = ctrl.last().t;
        //qDebug() << "ctrl:" << ctrl;

        if (sectionTime > waypointTime)
        {
            // Compute the waypoint.
            Keyframe kf = b.evaluateAt(ctrl, waypointTime);
            Vec2 dd = (kf.x/l)*(path[i+1]-path[i]);
            Hpm2D waypoint;
            waypoint.setPos(path[i]+dd);
            waypoint.setVel(dd.normalized(kf.v));
            return waypoint;
        }

        waypointTime -= sectionTime;
    }

    Hpm2D waypoint;
    waypoint.setPos(path.last());
    return waypoint;
}

// Returns the time to travel the path when using a line velocity model.
double VelocityProfile::getTotalTime(const Hpm2D& start, const Vector<Vec2>& path) const
{
    if (path.isEmpty())
        return 0;

    // Data structure for the velocity profile.
    Vector<double> V(path.length());

    // Project the current velocity onto the path. This will be the velocity in the first vertex.
    Vec2 section = path[1]-path[0]; // current section vector
    Vec2 startVel = start.vel();
    Vec2 projectedStartVel = startVel;
    projectedStartVel.projectOnVector(section);
    double v0 = sgn(section*startVel)*projectedStartVel.norm(); // current velocity projected onto the section vector
    V[0] = v0;

    // Compute angle based velocity bounds for all in-between vertices of the path.
    // For now this is a simple linear model such that the maximum velocity is allowed at an angle of 0,
    // and it linearly decreases to a 0 velocity at an angle of PI half (90 degrees).
    for (int i = 1; i < path.length()-1; i++)
    {
        Vec2 v1 = path[i]-path[i-1];
        Vec2 v2 = path[i+1]-path[i];
        double alpha = v1.angleTo(v2);
        V[i] = config.agentLinearVelocityLimitForward*max(1.0-fabs(alpha)/PI2, 0.0); // The velocity limit is enforced here as well.
    }

    // And finally, the velocity in the last vertex is zero.
    V[path.length()-1] = 0;

    // Propagate the velocity bounds to the neighbours of each path node to ensure dynamic feasibility
    // in terms of the acceleration limit.
    for (int i = 0; i < path.size()-1; i++)
    {
        double dx = (path[i+1]-path[i]).norm();
        double adx = 2*dx*config.agentLinearAccelerationLimit;
        double dv = V[i+1]*V[i+1]-V[i]*V[i];

        // Forward propagation (easy).
        // If there is not enough space to accelerate from vi up to vi+1, vi+1 has to be reduced.
        if (dv > adx)
            V[i+1] = sqrt(V[i]*V[i] + adx);

        // Backward propagation (difficult).
        // If there is not enough space to decelerate from vi down to vi+1, vi has to be reduced.
        // But this has an influence on the path node before, and that on the one before that.
        // The propagation might have to touch all nodes visited so far.
        else if (dv < -adx)
        {
            V[i] = sqrt(V[i+1]*V[i+1] + adx);

            for (int j = i-1; j >= 0; j--)
            {
                double dx = (path[j+1]-path[j]).norm();
                double adx = 2*dx*config.agentLinearAccelerationLimit;
                double dv = V[j+1]*V[j+1]-V[j]*V[j];
                if (dv < -adx)
                {
                    V[j] = sqrt(V[j+1]*V[j+1] + adx);
                }
                else
                {
                    break;
                }
            }
        }
    }

    // Determine the residual.
    if (v0 > V[0])
    {
        double residual = v0 - V[0];
        //qDebug() << "Residual in v0!" << v0 << V[0];
    }

    // Determine the section that the desired waypoint is in and compute the actual waypoint.
    BangBang b;
    b.setA(config.agentLinearAccelerationLimit);
    b.setV(config.agentLinearVelocityLimitForward);
    double totalTime = 0;
    for (int i = 0; i < path.size()-1; i++)
    {
        double l = (path[i+1]-path[i]).norm();

        b.clear();
        b.addKeyframe(0, 0, V[i]);
        b.addKeyframe(0, l, V[i+1]);
        Vector<Keyframe> ctrl = b.getTimeOptimalControlSequence();
        double sectionTime = ctrl.last().t;
        //qDebug() << "ctrl:" << ctrl;
        totalTime += sectionTime;
    }

    return totalTime;
}

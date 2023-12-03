#include "HolonomicDWA.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "blackboard/Command.h"
#include "lib/util/DrawUtil.h"
#include "lib/geometry/Collision.h"

// This is a custom implementation of the Dynamic Window Approach (DWA)
// for controlling a physical body that moves like the Synchronized
// Holonomic Model. The DWA works in a way that it performs a grid search
// over a set of rasterized controls, approximates the expected trajectory
// for each control input for a fixed amount of preview time, and evaluates
// the trajectory in terms of progress towards the target, velocity and
// heading at the end of the trajectory, and clearance. This implementation
// differs from the originally proposed version in the way that
// - It samples controls in acceleration space instead of velocity space.
// - It predicts the trajectories using the Synchronized Holonomic Model
//   instead of arcs.
// - It performs analytic collision checks with the Synchronized Holonomic Model.
// - It uses a smoothed costmap to sample proximity information from the grid.

HolonomicDWA::HolonomicDWA()
{
    debug = 0;
    bestTrajectoryId = 0;
    trajectoryType = 0;
}

HolonomicDWA::~HolonomicDWA()
{

}

void HolonomicDWA::setDebug(int d)
{
    debug = d;
}

void HolonomicDWA::setGeometricModel(const DynamicGeometricModel &gm)
{
    geometricModel = &gm;
}

void HolonomicDWA::setGridModel(const GridModel &gm)
{
    gridModel = &gm;
}

void HolonomicDWA::setTarget(const Vec2& wp)
{
    target = wp;
}

void HolonomicDWA::setStart(const Hpm2D &wp)
{
    start = wp;
}

void HolonomicDWA::setTrajectoryType(int type)
{
    trajectoryType = type;
}

// Performs the DWA search and returns the controls of the best trajectory.
Vec2 HolonomicDWA::search()
{
    // Dynamically adjust the size of the trajectorySamples vector to the number of notches.
    // This is so that the notches can be changed during runtime.
    if (trajectories.size() != config.DWA_notches*config.DWA_notches)
        trajectories.resize(config.DWA_notches*config.DWA_notches);

    // Sample the 2D acceleration space on a regular grid.
    int counter = 0;
    double maxTargetProgress = 0;
    double notch = 2*config.agentLinearAccelerationLimit/(config.DWA_notches-1);
    for (int i = 0; i < config.DWA_notches; i++)
    {
        for (int j = 0; j < config.DWA_notches; j++)
        {
            HDWA_Trajectory& traj = trajectories[counter++];
            traj.acc.x = config.agentLinearAccelerationLimit-i*notch;
            traj.acc.y = config.agentLinearAccelerationLimit-j*notch;

            // Compute the holonomic trajectory that results from the
            // acceleration sample and the constant DWA lookahead time.
            traj.ctrl[0] = start; // Begin with the start state.
            traj.ctrl[0].dt = config.DWA_time; // The time to forward by is the DWA_time parameter.
            traj.ctrl[0].setAcc(traj.acc); // The sampled acceleration.
            traj.ctrl[1] = traj.ctrl[0]; // Make a copy of the first keyframe.
            traj.ctrl[1].predict(); // Forward it.
            traj.ctrl[1].dt = 0;

            // Grid clearance (proximity).
            // Proximity to obstacles is punished. The proximity value is gained from the blurred grid
            // model by evaluating it at sample points along the trajectory and taking the maximum of these.
            // The grid model has been blurred specifically for this purpose.
            traj.gridClearance = gridModel->getAt(traj.ctrl[1].pos());
            for (int i = 1; i <= config.DWA_samples; i++)
            {
                double dt = i*config.DWA_time/(config.DWA_samples+1);
                Hpm2D kf = traj.ctrl[0].predicted(dt);
                traj.gridClearance = fmax(gridModel->getAt(kf.pos()), traj.gridClearance);
            }
            traj.gridClearance = -traj.gridClearance;

            // Geometric clearance, expressed in time to collision.
            // Collision check the trajectory in the geometric model.
            // The clearance is a value between 0 and 1 that says what portion of the trajectory the
            // taxi can drive before colliding with something.
            Collision col = geometricModel->trajectoryCollisionCheck(traj.ctrl[0]);
            traj.geometricClearance = 1.0;
            if (col.dt >= 0)
                traj.geometricClearance = col.dt/config.DWA_time; // normalized
            traj.colt = col.dt;

            // Target progress.
            // The target progress indicates how well the trajectory approaches the target.
            // It will also be normalized to [0,1] in the end.
            traj.targetDistance = (target - traj.ctrl.last().pos()).norm();
            maxTargetProgress = qMax(maxTargetProgress, traj.targetDistance);

            // If there is a collision, replace the trajectory with one that ends at the collision point.
            // For now this is just for a visual effect.
            if (col.dt >= 0)
            {
                traj.ctrl[0].dt = col.dt;
                traj.ctrl[1] = traj.ctrl[0]; // Make a copy of this keyframe.
                traj.ctrl[1].predict(); // Forward the keyframe.
                traj.ctrl[1].dt = 0;
            }

            //qDebug() << "HoloDWA:" << traj.acc << traj.ctrl[0] << traj.ctrl[1];
        }
    }


    // Normalize the progress part, compute the fitness function, and find its maximum.
    bestTrajectoryId = 0;
    int bestAdmissibleTrajectory = -1;
    int bestInadmissibleTrajectory = -1;
    for (int i = 0; i < trajectories.size(); i++)
    {
        // Normalize the target progress.
        if (maxTargetProgress > EPSILON)
            trajectories[i].targetDistance =  1.0 - trajectories[i].targetDistance/maxTargetProgress;

        // If the trajectory is admissible.
        if (trajectories[i].colt == -1)
        {
            // Compute the fitness function.
            double f1 = config.DWA_gridClearance * trajectories[i].gridClearance
                      + config.DWA_carrotDistance * trajectories[i].targetDistance;
                      //+ config.DWA_backwardsPenalty * (1.0-qMax(trajectories[i].ctrl[1].v/config.agentLinearVelocityLimitBackward, 0.0));

            trajectories[i].f = f1; // For visual

            // Keep track of the best admissible trajectory.
            if (bestAdmissibleTrajectory == -1 || f1 > trajectories[bestAdmissibleTrajectory].f1)
            {
                bestAdmissibleTrajectory = i;
                trajectories[bestAdmissibleTrajectory].f1 = f1;
            }
        }
        else
        {
            // Compute the fitness function using only clearance components.
            double f2 = config.DWA_gridClearance * trajectories[i].gridClearance
                    + config.DWA_geometricClearance * trajectories[i].geometricClearance;

            trajectories[i].f = 0; // For visual

            // Keep track of the best trajectory.
            if (bestInadmissibleTrajectory == -1 || f2 > trajectories[bestInadmissibleTrajectory].f2)
            {
                bestInadmissibleTrajectory = i;
                trajectories[bestInadmissibleTrajectory].f2 = f2;
            }
        }

        if (debug > 1)
        {
            qDebug() << i << trajectories[i].acc
                     << "Grid:" << trajectories[i].gridClearance
                     << "Geom:" << trajectories[i].geometricClearance
                     << "Progress:" << trajectories[i].targetDistance
                     << "Fitness:" << trajectories[i].f
                     << (trajectories[i].colt > -1 ? "COL" : "");
        }
    }

    // Elect the best trajectory.
    if (bestAdmissibleTrajectory > -1)
    {
        bestTrajectoryId = bestAdmissibleTrajectory;
        trajectories[bestTrajectoryId].f = trajectories[bestTrajectoryId].f1;
    }
    else
    {
        bestTrajectoryId = bestInadmissibleTrajectory;
        trajectories[bestTrajectoryId].f = trajectories[bestTrajectoryId].f2;
    }


    if (debug > 0)
    {
        qDebug() << "Best:" << bestTrajectoryId << trajectories[bestTrajectoryId].acc
                 << "Grid:" << trajectories[bestTrajectoryId].gridClearance
                 << "Geom:" << trajectories[bestTrajectoryId].geometricClearance
                 << "Progress:" << trajectories[bestTrajectoryId].targetDistance
                 << "Fitness:" << trajectories[bestTrajectoryId].f1 << trajectories[bestTrajectoryId].f2 << trajectories[bestTrajectoryId].f;
    }

    return trajectories[bestTrajectoryId].acc;
}

// Draws the tentacles (trajectory samples).
void HolonomicDWA::draw(QPainter *painter) const
{
    painter->save();

    for (int i = 0; i < trajectories.size(); i++)
    {
        const HDWA_Trajectory& traj = trajectories[i];

        QPen pen;
        pen.setWidth(1);
        pen.setCosmetic(true);
        pen.setColor(drawUtil.getHeightMapColor(traj.f, 0, trajectories.at(bestTrajectoryId).f));
        painter->setPen(pen);
        painter->setOpacity(0.5);

        // The bounding boxes.
        if (config.debugLevel > 3)
        {
            Box bb = traj.ctrl[0].boundingBox();
            bb.draw(painter);
        }

        // The trajectories.
        for (int j = 0; j < traj.ctrl.size(); j++)
        {
            traj.ctrl[j].draw(painter);

            // The points.
            double s = 0.01;
            painter->drawEllipse(traj.ctrl.last().pos(), s, s);
        }

        // Debug labels.
        if (config.debugLevel > 2)
        {
            QFont font;
            font.setFamily("Arial");
            font.setPointSize(2);
            painter->setFont(font);

            painter->save();
            painter->translate(traj.ctrl.last().pos());
            painter->scale(0.01, -0.01);
            painter->setOpacity(0.8);
            painter->drawText(QPointF(), "[" + QString::number(traj.acc.x, 'f', 2) + "," + QString::number(trajectories.at(i).acc.y, 'f', 2) + "]");
            painter->restore();
        }
    }

    if (trajectories.size() > 0)
    {
        QPen pen;
        pen.setWidth(3);
        pen.setCosmetic(true);
        pen.setColor(Qt::red);
        painter->setPen(pen);
        painter->setBrush(pen.color());
        painter->setOpacity(1.0);

        for (int j = 0; j < trajectories.at(bestTrajectoryId).ctrl.size(); j++)
            trajectories.at(bestTrajectoryId).ctrl[j].draw(painter);
    }

    painter->restore();
}

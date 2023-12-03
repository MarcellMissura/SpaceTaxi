#include "UnicycleDWA.h"
#include "board/Config.h"
#include "board/State.h"
#include "board/Command.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"
#include "lib/geometry/Collision.h"
#include "lib/geometry/VisibilityGraph.h"

// This is a custom implementation of the Dynamic Window Approach (DWA).

UnicycleDWA::UnicycleDWA()
{
    bestTrajectoryId = 0;
    trajectoryType = 0;
    geometricModel = 0;
}

UnicycleDWA::~UnicycleDWA()
{

}

void UnicycleDWA::setGeometricModel(const GeometricModel &gm)
{
    geometricModel = &gm;
}

void UnicycleDWA::setGridModel(const GridModel &gm)
{
    gridModel = &gm;
}

void UnicycleDWA::setCarrot(const Vec2 &wp)
{
    carrot = wp;
}

void UnicycleDWA::setStart(const Unicycle &wp)
{
    start = wp;
}

void UnicycleDWA::setTrajectoryType(int type)
{
    trajectoryType = type;
}

// Performs the DWA search with respect to a carrot and returns the next incremental acceleration of the best trajectory.
Vec2 UnicycleDWA::search(bool debug)
{
    // Dynamically adjust the size of the trajectories vector to the number of notches.
    // This is so that the notches can be changed during runtime.
    int numberOfTrajectories = config.DWA_notches*config.DWA_notches;
    if (trajectories.size() != numberOfTrajectories)
        trajectories.resize(numberOfTrajectories);

    if (debug)
        qDebug() << "UnicycleDWA::search() start:" << start << "target:" << carrot << "trajectories:" << trajectories.size();

    // Sample the 2D acceleration space on a regular grid and generate unicycle trajectories.
    int counter = 0;
    double maxCarrotDistance = 0;
    double notch_a = 2*config.agentLinearAccelerationLimit/(config.DWA_notches-1);
    double notch_b = 2*config.agentAngularAccelerationLimit/(config.DWA_notches-1);
    for (int i = 0; i < config.DWA_notches; i++)
    {
        for (int j = 0; j < config.DWA_notches; j++)
        {
            UDWA_Trajectory& traj = trajectories[counter];
            counter++;

            traj.acc.x = -config.agentLinearAccelerationLimit + i*notch_a;
            traj.acc.y = -config.agentAngularAccelerationLimit + j*notch_b;

            // Now generate the trajectory using the acceleration sample we just computed.
            traj.ctrl[0] = start; // Begin with the start state.
            traj.ctrl[0].dt = config.DWA_time; // The time to predict by is the DWA_time parameter.
            if (trajectoryType == command.Arc)
            {
                // Compute the velocity that results from integrating the acceleration sample times the DWA time.
                double v = start.v + traj.acc.x*config.DWA_time*0.5;
                double w = start.w + traj.acc.y*config.DWA_time*0.5;

                traj.ctrl[0].setVel(v, w);
                traj.ctrl[0].setAcc(0, 0); // Zero acceleration! This will make arcs.
            }
            else if (trajectoryType == command.B0)
            {
                // Compute the velocity that results from integrating the acceleration sample times the DWA time.
                double w = start.w + traj.acc.y*config.DWA_time*0.5;

                traj.ctrl[0].setVel(start.v, w);
                traj.ctrl[0].setAcc(traj.acc.x, 0); // Zero b, this will make b0 spirals.
            }
            else if (trajectoryType == command.Fresnel)
            {
                traj.ctrl[0].setAcc(traj.acc); // This will make Fresnel spirals.
            }
            traj.ctrl[1] = traj.ctrl[0]; // Make a copy of this keyframe.
            traj.ctrl[1].predict(); // Predict the keyframe.
            traj.ctrl[1].dt = 0;


            // Grid clearance.
            // Proximity to obstacles is punished. The proximity value is gained from the grid model by
            // taking the maximum of several sample points along the trajectory. The grid model has been blurred
            // specifically for this purpose. The cells of the grid contain [0,1] values so they are normalized.
            traj.gridClearance = gridModel->getAt(traj.ctrl[1].pos());
            for (int i = 1; i <= config.DWA_samples; i++)
            {
                double dt = i*config.DWA_time/(config.DWA_samples+1);
                Unicycle u = traj.ctrl[0].predicted(dt);
                traj.gridClearance = fmax(gridModel->getAt(u.pos()), traj.gridClearance); // already normalized
            }
            traj.gridClearance = -traj.gridClearance;

            // Geometric clearance (predictive).
            // Collision check the trajectory in the dynamic geometric model.
            // The clearance is a value between 0 and 1 that indicates at what portion
            // of the trajectory the collision occurs. A value of 1.0 means no collision.
            Collision col = geometricModel->trajectoryCollisionCheck(traj.ctrl[0], debug);
            traj.geometricClearance = 1.0;
            if (col.dt >= 0)
                traj.geometricClearance = col.dt/config.DWA_time; // normalized

            // Carrot distance.
            // The carrot distance indicates how well the trajectory approaches the carrot.
            // It will be normalized to [0,1] in the end.
            traj.carrotDistance = (carrot - traj.ctrl.last().pos()).norm(); // not normalized!
            maxCarrotDistance = qMax(maxCarrotDistance, traj.carrotDistance);

            // Reset the fitness value just because.
            traj.f = 0;

            // If there is a collision, replace the trajectory with one that ends at the collision point.
            // This is just for a visual effect and does not play a role in the objective function.
            if (col.dt >= 0)
            {
                traj.ctrl[0].dt = col.dt;
                traj.ctrl[1] = traj.ctrl[0]; // Make a copy of this keyframe.
                traj.ctrl[1].predict(); // Forward the keyframe.
                traj.ctrl[1].dt = 0;
            }
        }
    }

    // Normalize the components, compute the fitness function, and find its maximum. The larger the better.
    bestTrajectoryId = 0;
    int bestAdmissibleTrajectory = -1;
    int bestInadmissibleTrajectory = -1;
    for (int i = 0; i < trajectories.size(); i++)
    {
        // Normalize the carrot distance.
        if (maxCarrotDistance > EPSILON)
            trajectories[i].carrotDistance =  1.0 - trajectories[i].carrotDistance/maxCarrotDistance;

        // If the trajectory is admissible, i.e. non-colliding.
        if (trajectories[i].geometricClearance == 1.0)
        {
            // Compute the fitness function.
            double f1 = config.DWA_gridClearance * trajectories[i].gridClearance
                      + config.DWA_carrotDistance * trajectories[i].carrotDistance
                      + config.DWA_backwardsPenalty * (1.0-qMax(trajectories[i].ctrl[1].v/config.agentLinearVelocityLimitBackward, 0.0));

            trajectories[i].f = f1; // For visual

            // Keep track of the best admissible trajectory.
            if (bestAdmissibleTrajectory == -1 || f1 > trajectories[bestAdmissibleTrajectory].f1)
            {
                bestAdmissibleTrajectory = i;
                trajectories[bestAdmissibleTrajectory].f1 = f1;
            }
        }
        else // Otherwise inadmissible...
        {
            // Compute the fitness function using only clearance components.
            double f2 = config.DWA_gridClearance * trajectories[i].gridClearance
                    + config.DWA_geometricClearance * trajectories[i].geometricClearance;

            trajectories[i].f = 0; // For visual

            // Keep track of the best inadmissible trajectory.
            if (bestInadmissibleTrajectory == -1 || f2 > trajectories[bestInadmissibleTrajectory].f2)
            {
                bestInadmissibleTrajectory = i;
                trajectories[bestInadmissibleTrajectory].f2 = f2;
            }
        }

        if (debug)
            qDebug() << i << trajectories[i].acc
//                     << "from:" << trajectories[i].ctrl.first()
                     << "to:" << trajectories[i].ctrl.last().pose()
//                     << "Grid:" << trajectories[i].gridClearance
//                     << "Geom:" << trajectories[i].geometricClearance
//                     << "Distance:" << trajectories[i].carrotDistance
//                     << "Backwards:" << 1.0-qMax(trajectories[i].ctrl[1].v/config.agentLinearVelocityLimitBackward, 0.0)
                     << "Fitness:" << trajectories[i].f
                     << (trajectories[i].geometricClearance < 1.0 ? "COL" : "");
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


    if (debug)
        qDebug() << "Best:" << bestTrajectoryId << trajectories[bestTrajectoryId].acc
                 << "Grid:" << trajectories[bestTrajectoryId].gridClearance
                 << "Geom:" << trajectories[bestTrajectoryId].geometricClearance
                 << "Distance:" << trajectories[bestTrajectoryId].carrotDistance
//                 << "Backwards:" << 1.0-qMax(trajectories[bestTrajectoryId].ctrl[1].v/config.agentLinearVelocityLimitBackward, 0.0)
                 << "Fitness:" << trajectories[bestTrajectoryId].f1 << trajectories[bestTrajectoryId].f2 << trajectories[bestTrajectoryId].f;

    return trajectories[bestTrajectoryId].acc;
}

// Draws the tentacles (trajectory samples).
void UnicycleDWA::draw(QPainter *painter) const
{
    painter->save();

    for (int i = 0; i < trajectories.size(); i++)
    {
        const UDWA_Trajectory& traj = trajectories[i];

        QPen pen;
        pen.setWidth(2);
        pen.setCosmetic(true);
        pen.setColor(drawUtil.getHeightMapColor(traj.f, 0, trajectories.at(bestTrajectoryId).f));
        painter->setPen(pen);
        painter->setBrush(pen.color());
        painter->setOpacity(0.5);

        // The bounding boxes of the trajectories.
        if (config.debugLevel > 30)
        {
            painter->save();
            painter->setBrush(Qt::NoBrush);
            Box bb = traj.ctrl[0].boundingBox();
            bb.draw(painter);
            painter->restore();
        }

        // The trajectories.
        painter->save();
        for (int j = 0; j < traj.ctrl.size(); j++)
        {
            if (traj.geometricClearance < 1.0)
                painter->setPen(drawUtil.penThin);
            else
                painter->setPen(pen);
            traj.ctrl[j].draw(painter);
        }
        painter->restore();

        // Debug labels.
        if (config.debugLevel > 0)
        {
            QFont font;
            font.setFamily("Arial");
            font.setPointSize(1);
            painter->setFont(font);

            painter->save();
            painter->translate(traj.ctrl.last().pos());
            painter->rotate(-90);
            painter->scale(0.02, -0.02);
            painter->setOpacity(0.8);
            painter->drawText(QPointF(), "acc:[" + QString::number(trajectories.at(i).acc.x, 'g', 2) + "," + QString::number(trajectories.at(i).acc.y, 'g', 2) + "]"
                              + " gc:" + QString::number(trajectories.at(i).gridClearance, 'g', 2) + " f:" + QString::number(trajectories.at(i).f, 'g', 2));

            painter->restore();
        }
    }

    // The best trajectory.
    if (trajectories.size() > 0)
    {
        QPen pen;
        pen.setWidth(4);
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

// OpenGL visualization.
void UnicycleDWA::draw() const
{
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1);
    for (int i = 0; i < trajectories.size(); i++)
    {
        const UDWA_Trajectory& traj = trajectories[i];

        // The trajectories.
        for (int j = 0; j < traj.ctrl.size(); j++)
        {
            if (traj.geometricClearance < 1.0)
                GLlib::setColor(Qt::black);
            else
                GLlib::setColor(drawUtil.getHeightMapColor(traj.f, 0, trajectories.at(bestTrajectoryId).f));
            traj.ctrl[j].draw();
        }
    }

    // The best trajectory.
    if (trajectories.size() > 0)
    {
        glLineWidth(2);
        GLlib::setColor(Qt::red);
        for (int j = 0; j < trajectories.at(bestTrajectoryId).ctrl.size(); j++)
            trajectories.at(bestTrajectoryId).ctrl[j].draw();
    }
}

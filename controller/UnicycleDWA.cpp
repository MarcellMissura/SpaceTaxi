#include "UnicycleDWA.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "blackboard/Command.h"
#include "util/ColorUtil.h"
#include "geometry/Collision.h"
#include "geometry/VisibilityGraph.h"

// This is a custom implementation of the Dynamic Window Approach (DWA)

UnicycleDWA::UnicycleDWA()
{
    debug = 0;
    bestTrajectoryId = 0;
    trajectoryType = 0;
    dynamicGeometricModel = 0;
}

UnicycleDWA::~UnicycleDWA()
{

}

void UnicycleDWA::setDebug(int d)
{
    debug = d;
}

void UnicycleDWA::setDynamicGeometricModel(const DynamicGeometricModel &gm)
{
    dynamicGeometricModel = &gm;
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

void UnicycleDWA::setTarget(const Pose2D &wp)
{
    target = wp;
}

void UnicycleDWA::setStart(const Unicycle &wp)
{
    start = wp;
}

void UnicycleDWA::setTrajectoryType(int type)
{
    trajectoryType = type;
}

// Performs the DWA search with respect to a target state and returns the next incremental acceleration of the best trajectory.
Vec2 UnicycleDWA::search()
{
    int depth = qMax(1, (int)config.DWA_depth);

    // Dynamically adjust the size of the trajectories vector to the number of notches.
    // This is so that the notches can be changed during runtime.
    int numberOfTrajectories = pow(config.DWA_notches*config.DWA_notches, depth);
    if (trajectories.size() != numberOfTrajectories)
        trajectories.resize(numberOfTrajectories);

    if (debug > 1)
        qDebug() << "UnicycleDWA::search() start:" << start << "target:" << carrot << "trajectories:" << trajectories.size();


    VisibilityGraph visibilityGraph;
    visibilityGraph.setGeometricModel(*geometricModel);
    visibilityGraph.setBounds(config.gridWidth/2,
                              -config.gridHeight/2+config.gridOffset,
                              -config.gridWidth/2,
                              config.gridHeight/2+config.gridOffset);
    visibilityGraph.setTarget(target.pos());

    // Sample the 2D acceleration space on a regular grid and generate unicycle trajectories.
    int counter = 0;
    double maxCarrotDistance = 0;
    double maxCarrotAngle = 0;
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
                double v = start.v + traj.acc.x*config.DWA_time*config.DWA_fraction;
                double w = start.w + traj.acc.y*config.DWA_time*config.DWA_fraction;

                traj.ctrl[0].setVel(v, w);
                traj.ctrl[0].setAcc(0, 0); // Zero acceleration! This will make arcs.
            }
            else if (trajectoryType == command.B0)
            {
                // Compute the velocity that results from integrating the acceleration sample times the DWA time.
                double w = start.w + traj.acc.y*config.DWA_time*config.DWA_fraction;

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
            // evaluating it at each sample point and taking the maximum. The grid model has been blurred
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
            Collision col = dynamicGeometricModel->trajectoryCollisionCheck(traj.ctrl[0]);
            traj.geometricClearance = 1.0;
            if (col.dt >= 0)
                traj.geometricClearance = col.dt/config.DWA_time; // normalized

            // Target distance.
            // The target distance indicates how well the trajectory approaches the target.
            // It will be normalized to [0,1] in the end.
            traj.carrotDistance = (carrot - traj.ctrl.last().pos()).norm(); // not normalized!
            maxCarrotDistance = qMax(maxCarrotDistance, traj.carrotDistance);

            // Target angle.
            // Orienting towards the carrot. It will be normalized to [0,1] in the end.
            traj.carrotAngle = fabs(Vec2(carrot-traj.ctrl.last().pos()).angle() - traj.ctrl.last().heading()); // not normalized yet!
            maxCarrotAngle = qMax(maxCarrotAngle, traj.carrotAngle);

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

        // Normalize the carrot angle.
        if (maxCarrotAngle > EPSILON)
            trajectories[i].carrotAngle =  1.0 - trajectories[i].carrotAngle/maxCarrotAngle;

        // If the trajectory is admissible, i.e. non-colliding.
        if (trajectories[i].geometricClearance == 1.0)
        {
            // Compute the fitness function.
            double f1 = config.DWA_gridClearance * trajectories[i].gridClearance
                      + config.DWA_targetDistance * trajectories[i].carrotDistance
                      + config.DWA_targetAngle * trajectories[i].carrotAngle
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

        if (debug > 10)
            qDebug() << i << trajectories[i].acc
//                     << "from:" << trajectories[i].ctrl.first()
                     << "to:" << trajectories[i].ctrl.last().pose()
//                     << "Grid:" << trajectories[i].gridClearance
//                     << "Geom:" << trajectories[i].geometricClearance
//                     << "Distance:" << trajectories[i].carrotDistance
//                     << "Angle:" << trajectories[i].carrotAngle
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


    if (debug > 10)
        qDebug() << "Best:" << bestTrajectoryId << trajectories[bestTrajectoryId].acc
                 << "Grid:" << trajectories[bestTrajectoryId].gridClearance
                 << "Geom:" << trajectories[bestTrajectoryId].geometricClearance
                 << "Distance:" << trajectories[bestTrajectoryId].carrotDistance
//                 << "Angle:" << trajectories[bestTrajectoryId].carrotAngle
//                 << "Backwards:" << 1.0-qMax(trajectories[bestTrajectoryId].ctrl[1].v/config.agentLinearVelocityLimitBackward, 0.0)
                 << "Fitness:" << trajectories[bestTrajectoryId].f1 << trajectories[bestTrajectoryId].f2 << trajectories[bestTrajectoryId].f;

    return trajectories[bestTrajectoryId].acc;
}

// Evaluates the path rtr heuristic function for a given path.
// Even though the path starts with from and ends with to, from and to are
// needed to encode the start and goal orientations.
double UnicycleDWA::heuristic_pathrtr(const Vector<Vec2> &path, const Pose2D &from, const Pose2D &to) const
{
    // Remember the needed heading at the beginning of the path.
    double pathOrientation = Vec2(path[1]-path[0]).fangle();
    double frontAngleToPathOrientation = fabs(ffpicut(pathOrientation-from.z));
    double backAngleToPathOrientation = PI - frontAngleToPathOrientation;

    double currentHeading = pathOrientation;
    double pathTurningDistance = 0;
    double pathDrivingDistance = 0;
    for (uint i = 1; i < path.size(); i++)
    {
        Vec2 vecToNextPathNode = path[i]-path[i-1];
        double angleToNextPathNode = ffpicut(vecToNextPathNode.fangle()-currentHeading);
        currentHeading = ffpicut(currentHeading+angleToNextPathNode);
        pathTurningDistance += fabs(angleToNextPathNode);
        pathDrivingDistance += vecToNextPathNode.norm();
    }

    double frontAngleToGoalOrientation = fabs(ffpicut(to.z-currentHeading));
    double backAngleToGoalOrientation = PI - frontAngleToGoalOrientation;

    double frontTimeForRotation = (frontAngleToPathOrientation+pathTurningDistance+frontAngleToGoalOrientation)/config.agentAngularVelocityLimit;
    double backTimeForRotation = (backAngleToPathOrientation+pathTurningDistance+backAngleToGoalOrientation)/config.agentAngularVelocityLimit;
    double frontTimeForDriving = pathDrivingDistance/config.agentLinearVelocityLimitForward;
    double backTimeForDriving = pathDrivingDistance/-config.agentLinearVelocityLimitBackward;

    double hfront = frontTimeForRotation + frontTimeForDriving;
    double hback = backTimeForRotation + backTimeForDriving;

    return min(hfront, hback);
}

// Draws the tentacles (trajectory samples).
void UnicycleDWA::draw(QPainter *painter) const
{
    painter->save();

    // Draw the predictions of the dynamic geometric model.
    if (config.debugLevel > 0 && dynamicGeometricModel != 0)
    {
        for (int i = 1; i <= 3; i++)
        {
            DynamicGeometricModel gm = *dynamicGeometricModel;
            gm.predict(i*config.staaDt);
            gm.setId(i);
            gm.draw(painter, colorUtil.penThin, colorUtil.brushRed, 0.1);
        }
    }

    for (int i = 0; i < trajectories.size(); i++)
    {
        const UDWA_Trajectory& traj = trajectories[i];

        QPen pen;
        pen.setWidth(2);
        pen.setCosmetic(true);
        pen.setColor(colorUtil.getHeightMapColor(traj.f, 0, trajectories.at(bestTrajectoryId).f));
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
                painter->setPen(colorUtil.penThin);
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

}

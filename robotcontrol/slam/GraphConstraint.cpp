#include "GraphConstraint.h"
#include "lib/globals.h"
#include <cmath>

Eigen::Matrix3d GraphConstraint::getAij() const
{
    return Aij;
}

Eigen::Matrix3d GraphConstraint::getBij() const
{
    return Bij;
}

Eigen::Vector3d GraphConstraint::getErrorVector() const
{
    return errorVector;
}

void GraphConstraint::addOdomConstraint(PoseGraphNode &node1, PoseGraphNode &node2)
{
    // We add the constraint from node1 to node2, so i<j.
    i = node1.id;
    j = node2.id;
    ni = &node1;
    nj = &node2;
    posei = node1.pose;
    posej = node2.pose;
    measurement = posej - posei;
    observation = measurement;
}

void GraphConstraint::addLoopClosingConstraint(
        PoseGraphNode &node1,
        PoseGraphNode &node2,
        const Pose2D &observation)
{
    // We add the constraint from node1 to node2, so i<j.
    i = node1.id;
    j = node2.id;
    ni = &node1;
    nj = &node2;
    posei = node1.pose;
    posej = node2.pose;
    measurement = posej - posei;
    this->observation = observation;
    //qDebug() << "Pose i=" << i << "---" << posei;
    //qDebug() << "Pose j=" << j << "---"  << posej << "\n";
}

void GraphConstraint::updateConstraint(const PoseGraphNode &node1, const PoseGraphNode &node2)
{
    posei = node1.pose;
    posej = node2.pose;
    measurement = posej - posei;
    observation = measurement;
}

void GraphConstraint::linearizeConstraint()
{
    // Construct error Jacobian A
    Aij = Eigen::Matrix3d::Zero();
    double theta = posei.z;
    Aij.block<2, 2>(0, 0) = -getRotationMatrix(theta).transpose();
    Eigen::Vector2d t_ji;
    t_ji(0) = posej.x - posei.x;
    t_ji(1) = posej.y - posei.y;
    Aij.block<2, 1>(0, 2) = getRotationMatrixDerivative(theta).transpose() * t_ji;
    Aij(2, 2) = -1;

    // Construct error Jacobian B
    Bij = Eigen::Matrix3d::Zero();
    theta = posei.z;
    Bij.block<2, 2>(0, 0) = getRotationMatrix(theta).transpose();
    Bij(2, 2) = 1;

    // Compute error vector.
    Pose2D error = observation.diff(measurement);
    error.z = picut(error.z);

    errorVector(0) = error.x;
    errorVector(1) = error.y;
    errorVector(2) = error.z;
}

Eigen::Matrix2d GraphConstraint::getRotationMatrix(double theta) const
{
    double c = fcos(theta);
    double s = fsin(theta);

    Eigen::Matrix2d rotMatrix;
    rotMatrix(0, 0) = c;
    rotMatrix(0, 1) = -s;
    rotMatrix(1, 0) = s;
    rotMatrix(1, 1) = c;
    return rotMatrix;
}

Eigen::Matrix2d GraphConstraint::getRotationMatrixDerivative(double theta) const
{
    double c = fcos(theta);
    double s = fsin(theta);

    Eigen::Matrix2d rotMatrix;
    rotMatrix(0, 0) = -s;
    rotMatrix(0, 1) = -c;
    rotMatrix(1, 0) = c;
    rotMatrix(1, 1) = -s;
    return rotMatrix;
}

QDebug operator<< (QDebug dbg, const GraphConstraint &constraint)
{
    dbg << "Constraint" << constraint.i << constraint.j << "Observation" << constraint.observation;
    return dbg;
}

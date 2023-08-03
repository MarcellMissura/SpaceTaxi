#ifndef GRAPHCONSTRAINT_H
#define GRAPHCONSTRAINT_H

#include "PoseGraphNode.h"
#include "TrackedLine.h"
#include "lib/util/Pose2D.h"
#include <eigen3/Eigen/Dense>

class GraphConstraint
{
public:
    // Id's of connected nodes by this constraint.
    uint i, j;

    PoseGraphNode* ni;
    PoseGraphNode* nj;

    // Poses of node i and node j.
    Pose2D posei;
    Pose2D posej;

    Pose2D measurement;
    Pose2D observation;

    /** @brief Jacobian matrices and error vector for Linearization. */
    Eigen::Matrix3d Aij = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Bij = Eigen::Matrix3d::Zero();
    Eigen::Vector3d errorVector = Eigen::Vector3d::Zero();

public:
    GraphConstraint(){}
    ~GraphConstraint(){}

    /** @brief Getter for jacobians and error vector. */
    Eigen::Matrix3d getAij() const;
    Eigen::Matrix3d getBij() const;
    Eigen::Vector3d getErrorVector() const;

    /** @brief Adds the corresponding constraint from odometry between two consecutive nodes. */
    void addOdomConstraint(PoseGraphNode &node1, PoseGraphNode &node2);

    /** @brief Adds the corresponding constraint from loop closing between two nodes. */
    void addLoopClosingConstraint(
            PoseGraphNode &node1,
            PoseGraphNode &node2,
            const Pose2D &observation);

    /** @brief Computes the jacobians and error vector over the constraint between two nodes. */
    void linearizeConstraint();

    /** @brief Updates the graph constraint (posei, posej and measurement), from new node configuration. */
    void updateConstraint(const PoseGraphNode &node1, const PoseGraphNode &node2);

private:

    /** @brief Returns the rotation matrix SO(2).*/
    Eigen::Matrix2d getRotationMatrix(double theta) const;

    /** @brief Returns the derivative of rotation matrix SO(2).*/
    Eigen::Matrix2d getRotationMatrixDerivative(double theta) const;
};

QDebug operator<< (QDebug dbg, const GraphConstraint &constraint);

#endif // GRAPHCONSTRAINT_H

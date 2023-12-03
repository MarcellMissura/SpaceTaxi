#include "ShortTermAbortingAStar.h"
#include "board/Config.h"
#include "board/Command.h"
#include "board/State.h"
#include "lib/kfi/BangBang2D.h"
#include "lib/util/Vec3u.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"

// This is an A*-based motion planner that plans short plans with a high frequency.
// An action tree expressed in (x,y,theta) coordinates is mapped onto a support foot
// centeted grid where states can be closed which helps greatly with overcoming
// obsacles. Collision checks between planned footsteps and obstalces are made
// with a geometric model. The heurisitc is an RTR function. The A* is terminated
// early and then the state with the lowest heuristic is returned rather than the
// best score state the one with the lowest heuristic is more robustly close to
// the target. Footsteps are evaluated for balance, collision, and progress with
// respect to a target. The root of the tree is the current state of the robot,
// i.e. the current location of the support foot (refered to as loc) and the
// measured lip state. For balance, the initial lip state determines where the
// center of the step actions is and the zmp bounds are used to determine a
// feasible set of steps, propagating from end-of-step state to end-of-step
// state through the steps. The timing of the steps is computed automatically.

ShortTermAbortingAStar::ShortTermAbortingAStar()
{
    bestScoreNode = 0;
    bestHeuristicNode = 0;
    bestSolutionNode = 0;
    deepestNode = 0;
    expansions = 0;
    opened = 0;
    depth = 0;
    processed = 0;
    heuristicPathsComputed = 0;
    collided = 0;
    closed = 0;
    dried = 0;
    finished = false;
    score = 0;
    executionTime = 0;

    sensedGrid = 0;
    localMap = 0;

    stuck = false;
    trajectoryType = command.trajectoryType;
    heuristicType = command.heuristic;
    timeLimit = 84;
}

void ShortTermAbortingAStar::init()
{
    // A grid for closing states.
    closedMap.setDim(3);
    closedMap.setN(Vec3u(config.gridHeight/config.gridClosedCellSize+1, config.gridWidth/config.gridClosedCellSize+1, 31));
    closedMap.setMin(Vec3(-config.gridHeight/2+config.gridOffset, -config.gridWidth/2, -PI));
    closedMap.setMax(Vec3(config.gridHeight/2+config.gridOffset, config.gridWidth/2, PI));
    closedMap.init();

    // The hull polygon that is used for collision checking.
    double w = 0.5*config.agentWidth;
    double h = 0.5*config.agentHeight;
    hullPolygon.clear();
    hullPolygon.appendVertex(Vec2(-w, h));
    hullPolygon.appendVertex(Vec2(-w, -h));
    hullPolygon.appendVertex(Vec2(-0.9*w, -h));
    hullPolygon.appendVertex(Vec2(0.9*w, -0.8*h));
    hullPolygon.appendVertex(Vec2(w, -0.8*h));
    hullPolygon.appendVertex(Vec2(w, 0.8*h));
    hullPolygon.appendVertex(Vec2(0.9*w, 0.8*h));
    hullPolygon.appendVertex(Vec2(-0.9*w, h));
    hullPolygon.grow(config.staaPolygonGrowth);
    hullPolygon.reverseOrder();
    hullPolygon.setConvex();
    hullPolygon.setCW();

    // Precompute the action set.
    computeActionSet();
}

// Resets the data structure for the A* search.
void ShortTermAbortingAStar::reset()
{
    // It's faster to reset only the cells that have been closed
    // than to clear the whole closed grid.
    for (uint i = 0; i < closedList.size(); i++)
        closedMap.setAt(closedList[i], 0);
    closedList.clear();
    open.clear();
    Q.clear();
    bestScoreNode = 0;
    bestHeuristicNode = 0;
    deepestNode = 0;
    bestSolutionNode = 0;
    expansions = 0;
    opened = 0;
    depth = 0;
    processed = 0;
    heuristicPathsComputed = 0;
    collided = 0;
    closed = 0;
    executionTime = 0;
    finished = false;
    score = 0;
}

// Sets the grid model for the search.
void ShortTermAbortingAStar::setGridModel(GridModel &gm)
{
    sensedGrid = &gm;
}

// Sets the geometric model for the search.
void ShortTermAbortingAStar::setGeometricModel(GeometricModel &gm)
{
    localMap = &gm;
}

// Generates a discrete set of actions by sampling accelerations a and b.
// The a and b components are divided into a configurable amount of notches
// within configurable min and max ranges.
void ShortTermAbortingAStar::computeActionSet()
{
    // Sample the acceleration space on a regular grid.
    actionSet.clear();
    double notch_a = 2*config.agentLinearAccelerationLimit/(config.staaNotches-1);
    double notch_b = 2*config.agentAngularAccelerationLimit/(config.staaNotches-1);
    for (int i = 0; i < config.staaNotches; i++)
        for (int j = 0; j < config.staaNotches; j++)
            actionSet << Vec2(-config.agentLinearAccelerationLimit + i*notch_a,
                              -config.agentAngularAccelerationLimit + j*notch_b);
}

// Sets the start state for the search.
void ShortTermAbortingAStar::setStartState(const Unicycle &uni)
{
    *((Unicycle*)(&startState)) = uni;
    startState.type = trajectoryType;
}

// Sets the target state for the search. The target is given in local coordinates.
void ShortTermAbortingAStar::setTargetState(const Pose2D &target)
{
    targetState = target;
}

// Returns the target state.
const Pose2D &ShortTermAbortingAStar::getTargetState() const
{
    return targetState;
}

// Sets the time limit for the search in milliseconds.
void ShortTermAbortingAStar::setTimeLimit(double tl)
{
    timeLimit = tl;
}

// Performs the Polygons + Minimal Construct + PathRTR search using Aborting A*.
// Preconditions: setGridModel() must have been called to provide the input perception grid,
// The start and target states have to be set using setStartState() and setTargetState(),
// and reset() must have been called to reset the data structures after the last search.
bool ShortTermAbortingAStar::aStarSearch(int debug)
{
    if (debug > 0)
    {
        qDebug() << "ShortTermAbortingAStar::aStarSearch() started.";
        qDebug() << "Start State:" << startState;
        qDebug() << "Target state:" << targetState << "n:" << (startState.pose()-targetState).norm();
        qDebug() << "Stuck:" << stuck;
    }

    computeActionSet(); // Just to have it configurable.

    stopWatch.start();

    reset(); // Resets the data structures needed for the A* search.

    // If the Dijkstra heuristic is turned on, prepare the Dijkstra map.
    if (heuristicType == command.GridDijkstra)
        sensedGrid->initDijkstraMap(targetState.pos());

    // Push the start node into the priority queue.
    open << startState;
    Q.push(&open.first());
    bestHeuristicNode = &open.first();
    bestSolutionNode = &open.first();
    deepestNode = &open.first();

    // A*: Keep expanding the top of the queue until the goal is found or the queue is empty.
    while (!Q.isEmpty())
    {
        // Pop the best score node from the queue.
        bestScoreNode = Q.pop();
        //bestSolutionNode = bestScoreNode; // This line determines whether the bestScoreNode or the bestHeuristicNode is taken as the solution. Currently we are using the bestHeuristicNode.
        depth = bestSolutionNode->depth;
        expansions++;
        opened = Q.size();

        // Keep track of the deepest node.
        if (bestScoreNode->depth > deepestNode->depth)
            deepestNode = bestScoreNode;

        // Debug output.
        if (debug > 1)
        {
            qDebug() << "Popped:" << bestScoreNode
                     //<< "vel: " << bestScoreNode->vel()
                     << "\tg:" << bestScoreNode->g
                     << "h:" << bestScoreNode->h
                     << "f:" << bestScoreNode->f
                     << "n:" << (targetState-bestScoreNode->pose()).norm() << (targetState-bestScoreNode->pose()).max();
        }

        // Abort the search when the target has been found.
        // This is the best possible case and we can use the best score node as solution.
        if (isFinished())
        {
            finished = true;
            bestSolutionNode = bestScoreNode;
            depth = bestSolutionNode->depth;

            if (debug > 0)
            {
                qDebug() << "Aborting A* finished!"
                         << "expansions:" << expansions
                         << "processed:" << processed
                         << "open:" << open.size()
                         << "closed:" << closed;
                qDebug() << "Best action:" << getAction()
                         << "Final state:" << bestSolutionNode;
            }

            executionTime = stopWatch.elapsedTimeMs();
            return true;
        }

        // Abort the search when the expansion limit is reached, or the computation time is depleted,
        // or the robot is stuck and the search makes no progress.
        if ((config.staaExpansionLimit > 0 && expansions >= config.staaExpansionLimit)
                || (command.useTimeAbort && stopWatch.elapsedTimeMs() > timeLimit )
                || (stuck && bestScoreNode->depth > 7))
        {
            executionTime = stopWatch.elapsedTimeMs();
            if (debug > 0)
            {
                qDebug() << "Aborting A* aborted!"
                         << "Q:" << Q.size()
                         << "expansions:" << expansions
                         << "processed:" << processed
                         << "open:" << open.size()
                         << "closed:" << closed
                         << "time:" << executionTime;
                qDebug() << "bestSolutionNode:" << bestSolutionNode->trace();
            }

            return true;
        }

        // Skip closed states.
        if (command.useClosing && closedMap.getAt(bestScoreNode->pose()) > 0)
        {
            if (debug > 3)
                qDebug() << "    Closed, skipping.";
            closed++;
            continue;
        }

        // Close the node.
        bestScoreNode->closed = true; // This is to mark a node as closed for drawing.
        if (bestScoreNode->getDepth() > 0)
        {
            closedMap.setAt(bestScoreNode->pose(), 1); // This closes a cell in the closed map.
            closedList << closedMap.getNodeFlatIndex(bestScoreNode->pose());
        }

        // Keep track of the best heuristic node.
        if (bestScoreNode->collided == 0 && (bestHeuristicNode->depth == 0 || bestScoreNode->h < bestHeuristicNode->h))
        {
            bestHeuristicNode = bestScoreNode;
            bestSolutionNode = bestHeuristicNode;
            depth = bestSolutionNode->depth;
            //qDebug() << "best heuristic node switch to" << bestHeuristicNode->h << bestScoreNode->depth;
        }

        // Expand the children using the action set and sort them into the priority queue.
        for (uint i = 0; i < actionSet.size(); i++)
        {
            // Propagate from parent.
            UnicycleSearchNode currentChild;
            currentChild.type = trajectoryType;
            currentChild.propagate(bestScoreNode, actionSet[i], config.staaDt);
            currentChild.id = ++processed;

            // Skip out of bound states.
            if (!closedMap.contains(currentChild.pos()))
            {
                if (debug > 3)
                    qDebug() << "    Out of bounds, skipping.";
                continue;
            }

            // Skip closed states.
            if (command.useClosing && closedMap.getAt(currentChild.pose()) > 0)
            {
                if (debug > 3)
                    qDebug() << "    Closed" << currentChild << ". Skipping.";

                closed++;
                continue;
            }

            // SAFETY ZONE
            currentChild.sz = getSafetyPolygon(currentChild.v);
            if (!localMap->polygonCollisionCheck(currentChild.sz).isEmpty())
            {
                currentChild.collided = 4;
                collided++;
                if (debug > 3)
                    qDebug() << "    Collided with safety zone" << currentChild << ". Skipping.";
                open << currentChild;
                continue;
            }

            // COLLISION CHECK

            // It turns out that the following collision check routine costs much
            // less to compute than the heuristic, so we do the collision check
            // before the heuristic computation and skip the child if it collides.
            // Skipping collided children has the downside of the q possibly running
            // dry when all actions collided and the robot not knowing what to do.
            // The collision check is performed in a way that first, a polygon vs
            // grid check is performed in the sensed grid. If there is no collision,
            // a dynamic polygon check is performed vs the geometricModel where
            // moving obstacles are regarded only up to a certain depth. It misses
            // cases where two polygons move through each other in one time step
            // (tunneling), but as long as the time step is chosen small enough,
            // tunneling is no problem. Otherwise a swept volume concept needs to
            // be implemented.

            planPolygon = hullPolygon;
            planPolygon.setPose(currentChild.pose());
            planPolygon.transform();

            // Quick polygon collision check in the occupancy grid.
            // This is a very fast operation and hopefully enough to cover the static obstacles.
            if (sensedGrid->polygonCollisionCheck(planPolygon))
            {
                currentChild.collided = 2;
                collided++;
                if (debug > 3)
                    qDebug() << "    Collided with grid" << currentChild << ". Skipping.";
                open << currentChild;
                continue;
            }

            // Polygon collision check with the geometric model.
            if (!localMap->polygonCollisionCheck(planPolygon).isEmpty())
            {
                currentChild.collided = 3;
                collided++;
                if (debug > 3)
                    qDebug() << "    Collided with polygon. Skipping.";
                open << currentChild;
                continue;
            }


            // Cost so far (g = g + c). Costs are expressed as time, even though a penalty
            // is added for collided states and for obstacle proximity.
            double time = bestScoreNode->g + config.staaDt;
            double grid = config.staaGridCostWeight*sensedGrid->getAt(planPolygon); // Grid proximity. The costmap is used here.
            double geom = config.staaProximityCostWeight*max(1.0-localMap->dynamicDistance(currentChild.pos()), 0.0); // Geometric proximity.
            double tentative_g = time + grid + geom;

            // Add a weight to forward actions when in stuckness mode.
            if (stuck)
                tentative_g += config.staaStucknessWeight*currentChild.v;

            // HEURISTIC
            // Compute the heuristic and the f value. The inflation factor has a positive effect on the convergence.
            currentChild.g = tentative_g;
            currentChild.h = config.staaInflationFactor*heuristic(currentChild, targetState);
            currentChild.f = currentChild.h + currentChild.g;

            // We use the heuristic as a collision check as well.
            // If no path has been found, the node must be inside an obstacle.
            if (currentChild.h < 0)
            {
                currentChild.collided = 1;
                collided++;
                if (debug > 3)
                    qDebug() << "    Collided according to heuristic" << currentChild << ". Skipping.";
                open << currentChild;
                continue;
            }

            // A hack just for visualization.
            if (heuristicType == command.MinimalConstruct)
                currentChild.path = localMap->getPath();


            // Open the child node, push it into the open set and the priority queue.
            open << currentChild;
            Q.push(&open.last());

            if (debug > 2)
            {
                qDebug() << "    Pushed" << currentChild
                         << "\t g:" << currentChild.g << "(" << time << grid << geom << ")"
                         << "h:" << currentChild.h << "f:" << currentChild.f;
            }
        }
    }

    if (debug > 0)
    {
        qDebug() << "STAA* finished with empty queue! expansions:" << expansions << "processed:" << processed << "open:" << open.size() << "closed:" << closed << "collided:" << collided;
        qDebug() << "trace:" << bestSolutionNode->trace();
    }

    dried++;

    // Here, we are in the situation where the queue ran dry but the target was not found.
    // This typically happens when all options have collided or were closed.
    // We set an emergency brake action.
    bestSolutionNode->action = Vec2(-sgn0(startState.v)*config.agentLinearAccelerationLimit, -sgn0(startState.w)*config.agentAngularAccelerationLimit);

    executionTime = stopWatch.elapsedTimeMs();

    return false;
}

// Evaluates the Euklidean heuristic between two states from and to.
// The Euklidean heuristic is simply the distance between the locations.
// The orientation is ignored.
double ShortTermAbortingAStar::heuristic_euklidean(const Pose2D &from, const Pose2D &to) const
{
    return (from.pos()-to.pos()).norm()/config.agentLinearVelocityLimitForward;
}

// Evaluates the Euklidean heuristic along a path, i.e. sums up the length of the
// path segments and divides by the maximum velocity.
double ShortTermAbortingAStar::heuristic_patheuklidean(const Path &path, const Pose2D &from, const Pose2D &to) const
{
    // Remember the needed heading at the beginning of the path.
    double pathDrivingDistance = path.length();
    return pathDrivingDistance/config.agentLinearVelocityLimitForward;
}

// Evaluates the rtr heuristic between two states from and to.
// The rtr heuristic is the sum of the times that are needed to turn to the target,
// drive to the target, and turn into the target direction. Maximum velocities
// are used to hopefully underestimate the true costs.
double ShortTermAbortingAStar::heuristic_rtr(const Pose2D &from, const Pose2D &to, bool debug) const
{
    // The RTR heuristic is expressed in time needed to turn
    // to the target, drive to the target, and turn into the target direction.

    if (debug)
        qDebug() << "         heuristic_rtr from:" << from << "to:" << to;

    Vec2 vecToTarget(to.x-from.x, to.y-from.y);
    double distanceToTarget = vecToTarget.norm();
    double angleToTarget = vecToTarget.fangle();
    double frontAngleToTarget = distanceToTarget > EPSILON ? fabs(ffpicut(angleToTarget-from.z)) : 0;
    double frontAngleToGoalOrientation = distanceToTarget > EPSILON ? fabs(ffpicut(to.z-angleToTarget)) : 0;
    double frontTimeToRotateToTarget = frontAngleToTarget/config.agentAngularVelocityLimit;
    double frontTimeToDriveToTarget = distanceToTarget/config.agentLinearVelocityLimitForward;
    double frontTimeToRotateToGoalOrientation = frontAngleToGoalOrientation/config.agentAngularVelocityLimit;
    double backAngleToTarget = PI - frontAngleToTarget;
    double backAngleToGoalOrientation = PI - frontAngleToGoalOrientation;
    double backTimeToRotateToTarget = backAngleToTarget/config.agentAngularVelocityLimit;
    double backTimeToDriveToTarget = distanceToTarget/-config.agentLinearVelocityLimitBackward;
    double backTimeToRotateToGoalOrientation = backAngleToGoalOrientation/config.agentAngularVelocityLimit;

    double hfront = frontTimeToRotateToTarget + frontTimeToDriveToTarget + frontTimeToRotateToGoalOrientation;
    double hback = backTimeToRotateToTarget + backTimeToDriveToTarget + backTimeToRotateToGoalOrientation;

    if (debug)
    {
        qDebug() << "            vec:" << vecToTarget << vecToTarget.norm() << vecToTarget.fangle() << "angleToTarget" << angleToTarget;
        qDebug() << "            angs:" << frontAngleToTarget << distanceToTarget << frontAngleToGoalOrientation;
        qDebug() << "            times:" << frontTimeToRotateToTarget << frontTimeToDriveToTarget << frontTimeToRotateToGoalOrientation;
        qDebug() << "            fb:" << hfront << hback;
    }

    return min(hfront, hback);

    // Taking the min of the front and back version has greatly helped to eliminate
    // a weak spot in the RTR function close behind and left and right of the robot
    // where it vastly overestimates the costs.
}

// Evaluates the rtr heuristic between two states from and to.
// The rtr heuristic is the sum of the times that are needed to turn to the target,
// drive to the target, and turn into the target direction. Maximum velocities
// are used to hopefully underestimate the true costs.
double ShortTermAbortingAStar::heuristic_dock_rtr(const Pose2D &from, const Pose2D &to, bool debug) const
{
    Pose2D localTarget = to - from;
    double angle = localTarget.pos().fangle();
    //localTarget.rotate(-angle); // rotate
    double af = fabs(ffpicut(localTarget.heading()-angle));
    double lf = max(0.0, min(1.0, 2.0*(af-0.2))); // lf is 0 when af is 0.2 or less, lf is 1.0 when af is 0.5 or more

    Pose2D dock = to;
    dock.translate(-lf * Vec2(1.0, 0).rotated(to.heading()));
    if (debug)
    {
        qDebug() << "      heuristic_dock_rtr:" << from << dock << to;
        qDebug() << "         af lf:" << af << lf << "localtarget:" << localTarget << "angle:" << angle;
        qDebug() << "         h:" << heuristic_rtr(from, dock, true) << heuristic_rtr(dock, to, true);
    }
    return heuristic_rtr(from, dock) + heuristic_rtr(dock, to);
}

// Evaluates the rtr min heuristic between two states from and to.
// The rtr min heuristic returns the minimum of the time needed to drive
// to the target and the time needed to turn into the target orientation.
// This is a very minimal heuristic that is sure to underestimate the costs.
double ShortTermAbortingAStar::heuristic_rtr_min(const Pose2D &from, const Pose2D &to) const
{
    Vec2 vecToTarget;
    vecToTarget.x = to.x-from.x;
    vecToTarget.y = to.y-from.y;
    double angleToTarget = vecToTarget.fangle();
    //double frontAngleToTarget = fabs(ffpicut(angleToTarget-from.z));
    double distanceToTarget = vecToTarget.norm();
    double frontAngleToGoalOrientation = fabs(ffpicut(to.z-angleToTarget));
    //double frontTimeToRotateToTarget = frontAngleToTarget/config.agentAngularVelocityLimit;
    double frontTimeToDriveToTarget = distanceToTarget/config.agentLinearVelocityLimitForward;
    double frontTimeToRotateToGoalOrientation = fabs(frontAngleToGoalOrientation/config.agentAngularVelocityLimit);
    //double backAngleToTarget = PI - frontAngleToTarget;
    double backAngleToGoalOrientation = PI - frontAngleToGoalOrientation;
    //double backTimeToRotateToTarget = backAngleToTarget/config.agentAngularVelocityLimit;
    double backTimeToDriveToTarget = distanceToTarget/-config.agentLinearVelocityLimitBackward;
    double backTimeToRotateToGoalOrientation = backAngleToGoalOrientation/config.agentAngularVelocityLimit;

    // Note that only the angle to the goal orientation is used and not the whole turning task.
    // The whole turning task can overestimate situations that can be solved with less turning,
    // e.g. when the target is close to the agent on the left or the right side.

    double hfront = min(frontTimeToRotateToGoalOrientation, frontTimeToDriveToTarget);
    double hback = min(backTimeToRotateToGoalOrientation, backTimeToDriveToTarget);

    return min(hfront, hback);
}

// Evaluates the rtr max heuristic between two states from and to.
// The rtr max heuristic returns the maximum of the times needed to drive
// to the target and the time needed to turn into the target orientation.
// This heuristic underestimates the costs but less than rtr min.
double ShortTermAbortingAStar::heuristic_rtr_max(const Pose2D &from, const Pose2D &to) const
{
    Vec2 vecToTarget;
    vecToTarget.x = to.x-from.x;
    vecToTarget.y = to.y-from.y;
    double angleToTarget = vecToTarget.fangle();
    //double frontAngleToTarget = fabs(ffpicut(angleToTarget-from.z));
    double distanceToTarget = vecToTarget.norm();
    double frontAngleToGoalOrientation = fabs(ffpicut(to.z-angleToTarget));
    //double frontTimeToRotateToTarget = frontAngleToTarget/config.agentAngularVelocityLimit;
    double frontTimeToDriveToTarget = distanceToTarget/config.agentLinearVelocityLimitForward;
    double frontTimeToRotateToGoalOrientation = fabs(frontAngleToGoalOrientation/config.agentAngularVelocityLimit);
    //double backAngleToTarget = PI - frontAngleToTarget;
    double backAngleToGoalOrientation = PI - frontAngleToGoalOrientation;
    //double backTimeToRotateToTarget = backAngleToTarget/config.agentAngularVelocityLimit;
    double backTimeToDriveToTarget = distanceToTarget/-config.agentLinearVelocityLimitBackward;
    double backTimeToRotateToGoalOrientation = backAngleToGoalOrientation/config.agentAngularVelocityLimit;

    // Note that only the angle to the goal orientation is used and not the whole turning task.
    // The whole turning task can overestimate situations that can be solved with less turning,
    // e.g. when the target is close to the agent on the left or the right side.

    double hfront = max(frontTimeToRotateToGoalOrientation, frontTimeToDriveToTarget);
    double hback = max(backTimeToRotateToGoalOrientation, backTimeToDriveToTarget);

    return min(hfront, hback);
}

// Evaluates the path rtr heuristic function for a given path.
// Even though the path starts with from and ends with to, from and to are
// needed to encode the start and goal orientations.
double ShortTermAbortingAStar::heuristic_pathrtr(const Path &path, const Pose2D &from, const Pose2D &to) const
{
    //qDebug() << "ShortTermAbortingAStar::heuristic_pathrtr(const Path &path, const Pose2D &from, const Pose2D &to)";
    //qDebug() << "from:" << from << "to:" << to << "path:" << path << path.last() << path.last().isNull();

    // Remember the needed heading at the beginning of the path.
    double pathOrientation = path.last().isNull() ? from.z : Vec2(path[1]-path[0]).fangle();
    double frontAngleToPathOrientation = fabs(ffpicut(pathOrientation-from.z));
    double backAngleToPathOrientation = PI - frontAngleToPathOrientation;

    double currentHeading = pathOrientation;
    double pathTurningDistance = 0;
    double pathDrivingDistance = 0;
    for (uint i = 1; i < path.size(); i++)
    {
        Vec2 vecToNextPathNode = path[i]-path[i-1];
        double angleToNextPathNode = (path[i] == path[i-1]) ? 0 : ffpicut(vecToNextPathNode.fangle()-currentHeading);
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

// OpenGL visualiztion.
void ShortTermAbortingAStar::draw() const
{
    // The closed map.
    if (config.debugLevel > 3)
        closedMap.drawCumulated();

    // Draw the pose of all open states.
    if (config.debugLevel > 2)
    {
        ListIterator<UnicycleSearchNode> it = open.begin();
        it.next(); // skip the root
        while (it.hasNext())
        {
            const UnicycleSearchNode& u = it.next();

            QColor c = drawUtil.brushWhite.color();
            if (u.collided == 3)
                c = drawUtil.brushRed.color(); // dynamic collision
            else if (u.collided == 2)
                c = drawUtil.brushOrange.color(); // static collision
            else if (u.collided == 1)
                c = drawUtil.brushDarkGray.color(); // path collision
            glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF());
            GLlib::drawNoseCircle(u.pose(), 0.02); // dynamic collision
        }
    }

    // Only the closed nodes with trajectories.
    if (config.debugLevel > 0)
    {
        // Draw the trajectories and nose circles.
        ListIterator<UnicycleSearchNode> it = open.begin();
        it.next(); // skip the root
        while (it.hasNext())
        {
            const UnicycleSearchNode& u = it.next();
            if (!u.closed)
                continue;
            u.flipped().draw();

            QColor c = drawUtil.brushBlue.color();
            if (u.collided == 3)
                c = drawUtil.brushRed.color(); // dynamic collision
            else if (u.collided == 2)
                c = drawUtil.brushOrange.color(); // static collision
            else if (u.collided == 1)
                c = drawUtil.brushDarkGray.color(); // path collision
            glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF());
            GLlib::drawNoseCircle(u.pose(), 0.02); // dynamic collision
        }
    }

    // The shortest paths of the closed nodes.
    if (config.debugLevel == 2)
    {
        ListIterator<UnicycleSearchNode> it = open.begin();
        it.next(); // skip the root
        while (it.hasNext())
        {
            const UnicycleSearchNode& u = it.next();
            if (!u.closed)
                continue;

            u.path.draw(drawUtil.penGray);
        }
    }

    // Trace the best solution node.
    Polygon drawPolygon;
    if (bestSolutionNode != 0)
    {
        Vector<UnicycleSearchNode> trace = bestSolutionNode->trace();
        for (uint i = 0; i < trace.size()-1; i++)
        {
            drawPolygon = hullPolygon;
            drawPolygon.setPose(trace[i].pose());
            QColor col = drawUtil.brushMagenta.color();
            col.setAlphaF(0.1);
            drawPolygon.draw(drawUtil.penThick, drawUtil.brushMagenta, 0.1);
            glColor3f(0, 0, 0);
            trace[i].flipped().draw();
            col.setAlphaF(1.0);
            glColor4f(col.redF(), col.greenF(), col.redF(), col.alphaF());
            GLlib::drawNoseCircle(trace[i].pose(), 0.04);
        }
    }
}

// Visualizes the state of the search on a QPainter.
void ShortTermAbortingAStar::draw(QPainter* painter) const
{
    Polygon drawPolygon;

    // Debug level
    // 0 - draw the best solution trace
    // 1 - closed nodes
    // 2 - shortest paths
    // 3 - safety zones
    // 4 - closed map

    // The closed map.
    if (config.debugLevel > 3)
        closedMap.drawCumulated(painter);

    // The safety zones of the closed nodes.
    if (config.debugLevel >= 2)
    {
        ListIterator<UnicycleSearchNode> it = open.begin();
        it.next(); // skip the root
        while (it.hasNext())
        {
            UnicycleSearchNode& u = it.next();
            if (!u.closed)
                continue;

            u.sz.setPose(u.pose());
            if (u.collided == 4)
                u.sz.draw(painter, drawUtil.penThin, drawUtil.brushRed, 0.3);
            else
                u.sz.draw(painter, drawUtil.penThin, Qt::NoBrush, 0.1);
        }
    }


    // Draw the pose of all open states.
    if (config.debugLevel > 2)
    {
        ListIterator<UnicycleSearchNode> it = open.begin();
        it.next(); // skip the root
        while (it.hasNext())
        {
            const UnicycleSearchNode& u = it.next();
            if (u.collided == 4)
                u.drawNoseCircle(painter, 0.02, drawUtil.brushMagenta); // safety zone collision
            if (u.collided == 3)
                u.drawNoseCircle(painter, 0.02, drawUtil.brushRed); // dynamic collision
            else if (u.collided == 2)
                u.drawNoseCircle(painter, 0.02, drawUtil.brushOrange); // static collision
            else if (u.collided == 1)
                u.drawNoseCircle(painter, 0.02, drawUtil.brushDarkGray); // path collision
            else
                u.drawNoseCircle(painter, 0.02, drawUtil.brushWhite); // no collision
        }
    }

    // Only the closed nodes with trajectories.
    if (config.debugLevel > 0)
    {
        painter->save();

        // Draw the collided polygons.
        if (false)
        {
            painter->setOpacity(0.05);
            ListIterator<UnicycleSearchNode> it = open.begin();
            it.next(); // skip the root
            while (it.hasNext())
            {
                const UnicycleSearchNode& u = it.next();
                if (!u.closed || u.collided < 1)
                    continue;
                drawPolygon = hullPolygon;
                drawPolygon.setPose(u.pose());

                painter->setPen(drawUtil.penThin);
                if (u.collided == 3)
                    drawPolygon.draw(painter, drawUtil.penThin, drawUtil.brushRed, 0.05); // dynamic collision
                else if (u.collided == 2)
                    drawPolygon.draw(painter, drawUtil.penThin, drawUtil.brushOrange, 0.05); // static collision
                else if (u.collided == 1)
                    drawPolygon.draw(painter, drawUtil.penThin, drawUtil.brushDarkGray, 0.05); // path collision
                else
                    drawPolygon.draw(painter, drawUtil.penThin, drawUtil.brushBlue, 0.05);
            }
        }

        // Draw the trajectories of the closed nodes.
        painter->setOpacity(1.0);
        painter->setPen(drawUtil.penThin);
        ListIterator<UnicycleSearchNode> it = open.begin();
        it.next(); // skip the root
        while (it.hasNext())
        {
            const UnicycleSearchNode& u = it.next();
            if (!u.closed)
                continue;
            u.drawTrajectory(painter); // The trajectory.
        }

        // Draw the nose circles of the closed nodes.
        it = open.begin();
        it.next(); // skip the root
        while (it.hasNext())
        {
            const UnicycleSearchNode& u = it.next();
            if (!u.closed)
                continue;

            // The nose circle is colored in different colors depending on the type of the collision.
            if (u.collided == 3)
                u.drawNoseCircle(painter, 0.02, drawUtil.brushRed); // dynamic collision
            else if (u.collided == 2)
                u.drawNoseCircle(painter, 0.02, drawUtil.brushOrange); // static collision
            else if (u.collided == 1)
                u.drawNoseCircle(painter, 0.02, drawUtil.brushDarkGray); // path collision
            else
                u.drawNoseCircle(painter, 0.02, drawUtil.brushBlue); // no collision
        }

        painter->restore();
    }

    // The shortest paths of the closed nodes.
    if (config.debugLevel >= 2)
    {
        ListIterator<UnicycleSearchNode> it = open.begin();
        it.next(); // skip the root
        while (it.hasNext())
        {
            const UnicycleSearchNode& u = it.next();
            if (!u.closed)
                continue;

            u.path.draw(painter, drawUtil.penRedThin, 0.2);
        }
    }

    // Trace the best solution node.
    if (bestSolutionNode != 0)
    {
        painter->save();
        painter->setPen(drawUtil.penThick);
        Vector<UnicycleSearchNode> trace = bestSolutionNode->trace();
        for (uint i = 0; i < trace.size()-1; i++)
        {
            drawPolygon = hullPolygon;
            drawPolygon.setPose(trace[i].pose());
            drawPolygon.draw(painter, drawUtil.pen, drawUtil.brushMagenta, 0.1);
            painter->setOpacity(1.0);
            trace[i].drawTrajectory(painter);
            trace[i].drawNoseCircle(painter, 0.04, drawUtil.brushMagenta);
        }
        painter->restore();
    }
}

// The heuristic function called by A*.
double ShortTermAbortingAStar::heuristic(const UnicycleSearchNode &from, const Pose2D &to)
{
    if (heuristicType == command.Euklidean)
    {
        return heuristic_euklidean(from.pose(), to);
    }
    else if (heuristicType == command.RTR) // rtr
    {
        return heuristic_rtr(from.pose(), to);
    }
    else if (heuristicType == command.DOCK_RTR) // dock rtr
    {
        return heuristic_dock_rtr(from.pose(), to);
    }
    else if (heuristicType == command.RTR_MAX) // rtr max
    {
        return heuristic_rtr_max(from.pose(), to);
    }
    else if (heuristicType == command.RTR_MIN) // rtr min
    {
        return heuristic_rtr_min(from.pose(), to);
    }
    else if (heuristicType == command.PathEuklidean) // path euklidean
    {
        bool success = localMap->computeDynamicPath(from.pos(), to.pos());
        if (success)
            return heuristic_patheuklidean(localMap->getPath(), from.pose(), to);
    }
    else if (heuristicType == command.MinimalConstruct) // path rtr with mc
    {
        bool success = localMap->computeDynamicPath(from.pos(), to.pos());
        if (success)
            return heuristic_pathrtr(localMap->getPath(), from.pose(), to);
    }
    else if (heuristicType == command.GridDijkstra) // path rtr with dijkstra
    {
        const Vector<Vec2>& pp = sensedGrid->computeDijkstraPath(from.pos(), to.pos());
        //qDebug() << "dijkstra path:" << pp;
        Path ppp;
        ppp.set(pp);
        bool success = !pp.isEmpty();
        if (success)
            return heuristic_pathrtr(ppp, from.pose(), to);
    }

    return -1;
}

// Returns a polygon describing the safety zone for velocity vel.
Polygon ShortTermAbortingAStar::getSafetyPolygon(double vel) const
{
    Polygon sz;
    if (vel > 0.16)
    {
        double x = max(0.0, vel * 2.6/1.5);
        double y = vel <= 0.5 ? 0.5 : (0.5 + 0.5/0.6 * (vel-0.5));
        sz << Vec2(x, y) << Vec2(0,y) << Vec2(0, -y) << Vec2(x, -y);
    }
    return sz;
}

// Activates or deactivates the stuckness mode.
// When the stuckness mode is activated, forward actions are weighted higher to induce a
// collision-checked forward prodding behavior in order to resolve stuck situations.
void ShortTermAbortingAStar::setStuck(bool value)
{
    stuck = value;
}

// Returns true if the search has reached the target.
// The targe condition is expressed as a threshold over the RTR max function
// of the bestScore pose and the target state.
bool ShortTermAbortingAStar::isFinished()
{
    if (bestScoreNode == 0)
        return false;
    //score = heuristic_rtr_max(bestScoreNode->pose(), targetState);
    //return (score < config.staaFinishedThreshold);
    return ((bestScoreNode->pose()-targetState).max() < config.agentTargetReachedDistance);
}

// Returns the computed plan.
Vector<UnicycleSearchNode> ShortTermAbortingAStar::getPlan() const
{
    return bestSolutionNode->trace();
}

// Returns the next action to take according to the last computed plan.
Vec2 ShortTermAbortingAStar::getAction() const
{
    if (bestSolutionNode != 0)
        return bestSolutionNode->rootAction();
    return Vec2();
}


// Returns the last measured execution time.
double ShortTermAbortingAStar::getExecutionTime() const
{
    return executionTime;
}

// Returns the number of iterations the A* algorithm needed.
int ShortTermAbortingAStar::getExpansions() const
{
    return expansions;
}

// Returns the size of the open list.
int ShortTermAbortingAStar::getOpenListSize() const
{
    return open.size();
}

// Tells you the size of the q in the end.
int ShortTermAbortingAStar::getQSize() const
{
    return Q.size();
}

// How many shortest paths have been computed?
int ShortTermAbortingAStar::getPathsComputed() const
{
    return heuristicPathsComputed;
}

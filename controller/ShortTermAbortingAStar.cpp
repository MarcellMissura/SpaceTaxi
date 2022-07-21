#include "ShortTermAbortingAStar.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"
#include "blackboard/State.h"
#include "util/Vec3u.h"
#include "util/ColorUtil.h"
#include "util/GLlib.h"

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
    debug = 0;
    expansions = 0;
    processed = 0;
    heuristicPathsComputed = 0;
    collided = 0;
    closed = 0;
    dried = 0;
    finished = false;
    executionTime = 0;

    staticGeometricModel = 0;
    dynamicGeometricModel = 0;
    sensedGrid = 0;
    dilatedSensedGrid = 0;

    stuck = false;
    trajectoryType = command.trajectoryType;
    heuristicType = command.heuristic;
    timeLimit = config.staaTimeLimit;
}

void ShortTermAbortingAStar::init()
{
    // A grid for closing states.
    closedMap.setDim(3);
    closedMap.setN(Vec3u(config.gridHeight/config.gridClosedCellSize+1, config.gridWidth/config.gridClosedCellSize+1, 31));
    closedMap.setMin(Vec3(-config.gridHeight/2+config.gridOffset, -config.gridWidth/2, -PI));
    closedMap.setMax(Vec3(config.gridHeight/2+config.gridOffset, config.gridWidth/2, PI));
    closedMap.init();

    // Compute the hull polygon that is used for collision checking.
    double w = 0.5*config.agentWidth;
    double h = 0.5*config.agentHeight;
    int subDivs = 3;
    hullPolygon.clear();
    for (int i = 0; i < subDivs; i++)
    {
        double frac = (double)i/subDivs;
        hullPolygon.addVertex(Vec2(-w, h + frac*(-h-h)));
    }
    for (int i = 0; i < subDivs; i++)
    {
        double frac = (double)i/subDivs;
        hullPolygon.addVertex(Vec2(-w + frac*(w+w), -h + frac*(-0.8*h+h)));
    }
    for (int i = 0; i < subDivs; i++)
    {
        double frac = (double)i/subDivs;
        hullPolygon.addVertex(Vec2(w, -0.8*h + frac*(0.8*h+0.8*h)));
    }
    for (int i = 0; i < subDivs; i++)
    {
        double frac = (double)i/subDivs;
        hullPolygon.addVertex(Vec2(w + frac*(-w-w), 0.8*h + frac*(h-0.8*h)));
    }
    hullPolygon.grow(config.staaPolygonGrowth);
    hullPolygon.setConvex();


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
    processed = 0;
    heuristicPathsComputed = 0;
    collided = 0;
    closed = 0;
    executionTime = 0;
    finished = false;
}

// Sets the sensed grid.
void ShortTermAbortingAStar::setSensedGrid(GridModel &gm)
{
    sensedGrid = &gm;
}

// Sets the dilated sensed grid.
void ShortTermAbortingAStar::setDilatedSensedGrid(GridModel &gm)
{
    dilatedSensedGrid = &gm;
}

// Sets the static geometric model for the search.
void ShortTermAbortingAStar::setStaticGeometricModel(const GeometricModel &gm)
{
    staticGeometricModel = &gm;
}

// Sets the dynamic geometric model for the search.
void ShortTermAbortingAStar::setDynamicGeometricModel(const DynamicGeometricModel &gm)
{
    dynamicGeometricModel = &gm;
}

// Sets the expanded static geometric model for the search.
void ShortTermAbortingAStar::setExpandedStaticGeometricModel(const GeometricModel &gm)
{
    expandedStaticGeometricModel = &gm;
}

// Sets the expanded dynamic geometric model for the search.
void ShortTermAbortingAStar::setExpandedDynamicGeometricModel(const DynamicGeometricModel &gm)
{
    expandedDynamicGeometricModel = &gm;
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

// Performs the Polygons + Minimal Construct + PathRTR search using Aborting A*.
// Preconditions: setGridModel() must have been called to provide the input perception grid,
// The start and target states have to be set using setStartState() and setTargetState(),
// and reset() must have been called to reset the data structures after the last search.
bool ShortTermAbortingAStar::aStarSearch()
{
    if (debug > 0)
    {
        qDebug() << "ShortTermAbortingAStar::aStarSearch() started.";
        qDebug() << "Start State:" << startState;
        qDebug() << "Target state:" << targetState;
        qDebug() << "Stuck:" << stuck;
    }

    computeActionSet(); // Just to have it configurable.

    stopWatch.start();

    reset(); // Resets the data structures needed for the A* search.

    // If the Dijkstra heuristic is turned on, prepare the Dijkstra map.
    if (heuristicType == command.GridDijkstra)
    {
        dilatedSensedGrid->initDijkstraMap(targetState.pos());
        //double tt = stopWatch.elapsedTimeMs();
        //qDebug() << tt;
    }

    // Set up the predicted unified models for the path searches in every depth.
    uint maxPredictionDepth = config.staaPredictionTimeLimit/config.staaDt;
    predictedUnifiedModels.resize(maxPredictionDepth+1);
    predictedDynamicModels.resize(maxPredictionDepth+1);
    for (uint i = 0; i < predictedUnifiedModels.size(); i++)
    {
        predictedUnifiedModels[i].clear();
        predictedUnifiedModels[i].setObstacles(expandedStaticGeometricModel->getObstacles());
        predictedUnifiedModels[i].setBounds();
        if (i*config.staaDt <= config.staaPredictionTimeLimit)
        {
            erasedDynamicModel = *expandedDynamicGeometricModel;
            erasedDynamicModel.predict(i*config.staaDt); // This line controls predicted path planning.
            erasedDynamicModel.transform();
            erasedDynamicModel.eraseObstaclesAt(targetState.pos());
            predictedUnifiedModels[i].addObstacles(erasedDynamicModel.getObstacles());
            const Vector<Vec2>& pp = predictedUnifiedModels[i].computePath(startState.pos(), targetState.pos());
            if (pp.isEmpty() || !command.useDynamicPath)
            {
                predictedUnifiedModels[i].clear();
                predictedUnifiedModels[i].setObstacles(expandedStaticGeometricModel->getObstacles());
            }
        }

        predictedDynamicModels[i].clear();
        if (i*config.staaDt <= config.staaPredictionTimeLimit)
        {
            predictedDynamicModels[i] = *dynamicGeometricModel;
            predictedDynamicModels[i].predict(i*config.staaDt);
            predictedDynamicModels[i].transform();
        }
    }

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
        //bestSolutionNode = bestScoreNode; // This line determines whether the bestScoreNode or the bestHeuristicNode is used.
        expansions++;

        // Keep track of the deepest node.
        if (bestScoreNode->depth > deepestNode->depth)
            deepestNode = bestScoreNode;

        // Debug output.
        if (debug > 1)
        {
            qDebug() << "Popped:" << bestScoreNode
                     << "vel: " << bestScoreNode->vel()
                     << "\tg:" << bestScoreNode->g
                     << "h:" << bestScoreNode->h
                     << "f:" << bestScoreNode->f;
        }

        // Abort the search when the target has been found.
        // This is the best possible case and we can use the best score node as solution.
        if (isFinished())
        {
            finished = true;
            bestSolutionNode = bestScoreNode;

            if (debug > 0)
            {
                qDebug() << "Aborting A* finished!"
                         << "expansions:" << expansions
                         << "processed:" << processed
                         << "open:" << open.size()
                         << "closed:" << closed;
                qDebug() << "Best action:" << getAction();
            }

            executionTime = stopWatch.elapsedTimeMs();
            return true;
        }

        // Abort the search when the expansion limit is reached or the computation time is depleted.
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
            //qDebug() << "best heuristic node switch to" << bestHeuristicNode->h << bestScoreNode->depth;
        }

        // Expand the children using the action set and sort them into the priority queue.
        for (uint i = 0; i < actionSet.size(); i++)
        {
            // Propagate from parent.
            UnicycleSearchNode currentChild;
            currentChild.type = trajectoryType;
            currentChild.propagate(bestScoreNode, actionSet[i]);
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


            // COLLISION CHECK

            // It turns out that the following collision check routine costs much
            // less to compute than the heuristic, so we do the collision check
            // before the heuristic computation and skip the child if it collides.
            // The collision check is performed in a way that first, a polygon vs
            // grid check is performed in the sensed grid. If there is no collision,
            // a dynamic polygon check is performed vs the dynamicGeometricModel up
            // to a certain depth. It misses cases where two polygons move through
            // each other in one time step (tunneling), but what are the odds of
            // that happening, right?

            planPolygon = hullPolygon;
            planPolygon.setPose(currentChild.pose());
            planPolygon.transform();

            // Quick polygon collision check in the occupancy grid.
            // This is very fast and hopefully enough to cover the static obstacles.
            if (sensedGrid->polygonCollisionCheck(planPolygon))
            {
                currentChild.collided = 2;
                collided++;
                if (debug > 3)
                    qDebug() << "    Collided with grid" << currentChild << ". Skipping.";
                open << currentChild;
                continue;
            }

            // Check only the convex polygons of the map.
            // This is because the grid collision check misses small obstacles that fit
            // into the agent polygon. Small obstacles are almost always convex.
            int obstIdx = staticGeometricModel->convexPolygonCollisionCheck(planPolygon);
            if (obstIdx >= 0)
            {
                currentChild.collided = 2;
                collided++;
                if (debug > 3)
                    qDebug() << "    Collided with convex polygon" << obstIdx << staticGeometricModel->getObstacle(obstIdx).getId() << ". Skipping.";
                open << currentChild;
                continue;
            }

            // Polygon collision check with the dynamic model.
            // This is also not that expensive since there are so few dynamic obstacles.
            if (currentChild.depth*currentChild.dt <= config.staaPredictionTimeLimit)
            {
                int obstIdx = predictedDynamicModels[currentChild.depth].staticPolygonCollisionCheck(planPolygon);
                if (obstIdx >= 0)
                {
                    currentChild.collided = 3;
                    collided++;
                    if (debug > 3)
                        qDebug() << "    Collided with dynamic polygon" << obstIdx << dynamicGeometricModel->getObstacle(obstIdx).getId() << ". Skipping.";
                    open << currentChild;
                    continue;
                }
            }


            // Cost so far (g = g + c). Costs are expressed as time, even though a penalty
            // is added for collided states and for obstacle proximity.
            double tentative_g = bestScoreNode->g + config.staaDt;
            tentative_g += config.staaGridCostWeight*dilatedSensedGrid->getAt(planPolygon); // grid proximity
            //tentative_g += config.staaGridCostWeight*dilatedSensedGrid->getAt(currentChild.pos()); // grid proximity
            tentative_g += config.staaProximityCostWeight*max(1.0-predictedDynamicModels[min(currentChild.depth, maxPredictionDepth)].distance(currentChild.pos()), 0.0); // polygon proximity

            // Add a weight to forward actions when in stuckness mode.
            if (stuck)
                tentative_g += config.staaStucknessWeight*currentChild.v;

            // Compute the heuristic and the f value.
            currentChild.g = tentative_g;
            currentChild.h = heuristic(currentChild.pose(), targetState, min(currentChild.depth, maxPredictionDepth));
            currentChild.f = currentChild.h + currentChild.g;

            // We use the heuristic as a collision check as well.
            // If no path has been found, the node must be inside an obstacle.
            if (currentChild.h < 0)
            {
                //qDebug() << "path failed for child" << currentChild;
                currentChild.collided = 1;
                collided++;
                if (debug > 3)
                    qDebug() << "    Collided according to heuristic" << currentChild << ". Skipping.";
                open << currentChild;
                continue;
            }

            // A hack just for visualization.
            if (command.heuristic == command.MinimalConstruct)
            {
                currentChild.path = predictedUnifiedModels[min(currentChild.depth, maxPredictionDepth)].getLastPath();
            }
            else
            {
                currentChild.path = dilatedSensedGrid->getLastPath();
            }

            // Open the child node, push it into the open set and the priority queue.
            open << currentChild;
            Q.push(&open.last());

            if (debug > 2)
            {
                qDebug() << "    Pushed" << currentChild
                         << "\t g:" << currentChild.g << (config.staaStucknessWeight*currentChild.v)
                         << "h:" << currentChild.h << "f:" << currentChild.f;
            }
        }
    }

    if (debug > 0)
    {
        qDebug() << "STAA* finished with empty queue! expansions:" << expansions << "processed:" << processed << "open:" << open.size() << "closed:" << closed;
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
double ShortTermAbortingAStar::heuristic_patheuklidean(const Vector<Vec2> &path, const Pose2D &from, const Pose2D &to) const
{
    // Remember the needed heading at the beginning of the path.
    double pathDrivingDistance = 0;
    for (uint i = 1; i < path.size(); i++)
        pathDrivingDistance += (path[i]-path[i-1]).norm();
    return pathDrivingDistance/config.agentLinearVelocityLimitForward;
}

// Evaluates the rtr heuristic between two states from and to.
// The rtr heuristic is the sum of the times that are needed to turn to the target,
// drive to the target, and turn into the target direction. Maximum velocities
// are used to hopefully underestimate the true costs.
double ShortTermAbortingAStar::heuristic_rtr(const Pose2D &from, const Pose2D &to) const
{
    // The RTR heuristic is expressed in time needed to turn
    // to the target, drive to the target, and turn into the target direction.

    Vec2 vecToTarget;
    vecToTarget.x = to.x-from.x;
    vecToTarget.y = to.y-from.y;
    double angleToTarget = vecToTarget.fangle();
    double frontAngleToTarget = fabs(ffpicut(angleToTarget-from.z));
    double distanceToTarget = vecToTarget.norm();
    double frontAngleToGoalOrientation = fabs(ffpicut(to.z-angleToTarget));
    double frontTimeToRotateToTarget = frontAngleToTarget/config.agentAngularVelocityLimit;
    double frontTimeToDriveToTarget = distanceToTarget/config.agentLinearVelocityLimitForward;
    double frontTimeToRotateToGoalOrientation = fabs(frontAngleToGoalOrientation/config.agentAngularVelocityLimit);
    double backAngleToTarget = PI - frontAngleToTarget;
    double backAngleToGoalOrientation = PI - frontAngleToGoalOrientation;
    double backTimeToRotateToTarget = backAngleToTarget/config.agentAngularVelocityLimit;
    double backTimeToDriveToTarget = distanceToTarget/-config.agentLinearVelocityLimitBackward;
    double backTimeToRotateToGoalOrientation = backAngleToGoalOrientation/config.agentAngularVelocityLimit;

    double hfront = frontTimeToRotateToTarget + frontTimeToDriveToTarget + frontTimeToRotateToGoalOrientation;
    double hback = backTimeToRotateToTarget + backTimeToDriveToTarget + backTimeToRotateToGoalOrientation;

    return min(hfront, hback);

    // Taking the min of the front and back version has greatly helped to eliminate
    // a weak spot in the RTR function close behind and left and right of the robot
    // where it vastly overestimates the costs.
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
double ShortTermAbortingAStar::heuristic_pathrtr(const Vector<Vec2> &path, const Pose2D &from, const Pose2D &to) const
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

// OpenGL visualiztion.
void ShortTermAbortingAStar::draw() const
{
    // The closed map.
    if (config.debugLevel > 3)
        closedMap.drawCumulated();

    // Draw the predictions of the dynamic geometric model.
    if (config.debugLevel > 0 && bestSolutionNode != 0)
    {
        for (int i = 1; i <= deepestNode->depth; i++)
        {
            if (i*bestScoreNode->dt <= config.staaPredictionTimeLimit)
            {
                DynamicGeometricModel gm = *dynamicGeometricModel;
                gm.predict(i*bestSolutionNode->dt);
                gm.setId(i);
                gm.draw(colorUtil.penThin, colorUtil.brushRed, 0.1);
            }
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

            QColor c = colorUtil.brushWhite.color();
            if (u.collided == 3)
                c = colorUtil.brushRed.color(); // dynamic collision
            else if (u.collided == 2)
                c = colorUtil.brushOrange.color(); // static collision
            else if (u.collided == 1)
                c = colorUtil.brushDarkGray.color(); // path collision
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

            QColor c = colorUtil.brushBlue.color();
            if (u.collided == 3)
                c = colorUtil.brushRed.color(); // dynamic collision
            else if (u.collided == 2)
                c = colorUtil.brushOrange.color(); // static collision
            else if (u.collided == 1)
                c = colorUtil.brushDarkGray.color(); // path collision
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

            glColor3f(0.3,0.3,0.3);
            for (uint i = 1; i < u.path.size(); i++)
                GLlib::drawLine(u.path[i], u.path[i-1], 0.025);
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
            QColor col = colorUtil.brushMagenta.color();
            col.setAlphaF(0.1);
            drawPolygon.draw(col);
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
    painter->save();

    Polygon drawPolygon;

    // The closed map.
    if (config.debugLevel > 3)
        closedMap.drawCumulated(painter);

    // Draw the predictions of the dynamic geometric model.
    if (config.debugLevel > 0 && bestSolutionNode != 0)
    {
        for (int i = 1; i <= deepestNode->depth; i++)
        {
            if (i*bestScoreNode->dt <= config.staaPredictionTimeLimit)
            {
                DynamicGeometricModel gm = *dynamicGeometricModel;
                gm.predict(i*bestSolutionNode->dt);
                gm.setId(i);
                gm.draw(painter, colorUtil.penThin, colorUtil.brushRed);
            }
        }
    }

    // The stack of predicted unified models.
//    for (uint i = 0; i < predictedUnifiedModels.size(); i++)
//        predictedUnifiedModels[i].draw(painter, colorUtil.pen, colorUtil.brushGreen);

    // Draw the pose of all open states.
    if (config.debugLevel > 2)
    {
        ListIterator<UnicycleSearchNode> it = open.begin();
        it.next(); // skip the root
        while (it.hasNext())
        {
            const UnicycleSearchNode& u = it.next();
            if (u.collided == 3)
                u.drawNoseCircle(painter, 0.02, colorUtil.brushRed); // dynamic collision
            else if (u.collided == 2)
                u.drawNoseCircle(painter, 0.02, colorUtil.brushOrange); // static collision
            else if (u.collided == 1)
                u.drawNoseCircle(painter, 0.02, colorUtil.brushDarkGray); // path collision
            else
                u.drawNoseCircle(painter, 0.02, colorUtil.brushWhite);
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

                painter->setPen(colorUtil.penThin);
                if (u.collided == 3)
                    painter->setBrush(colorUtil.brushRed); // dynamic collision
                else if (u.collided == 2)
                    painter->setBrush(colorUtil.brushOrange); // static collision
                else if (u.collided == 1)
                    painter->setBrush(colorUtil.brushDarkGray); // path collision
                else
                    painter->setBrush(colorUtil.brushBlue);
                drawPolygon.draw(painter);
            }
        }

        // Draw the trajectories of the closed nodes.
        painter->setOpacity(1.0);
        painter->setPen(colorUtil.penThin);
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
                u.drawNoseCircle(painter, 0.02, colorUtil.brushRed); // dynamic collision
            else if (u.collided == 2)
                u.drawNoseCircle(painter, 0.02, colorUtil.brushOrange); // static collision
            else if (u.collided == 1)
                u.drawNoseCircle(painter, 0.02, colorUtil.brushDarkGray); // path collision
            else
                u.drawNoseCircle(painter, 0.02, colorUtil.brushBlue); // no collision
        }

        painter->restore();
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

            painter->save();
            painter->setOpacity(0.2);
            painter->setPen(colorUtil.penThin);
            for (uint i = 1; i < u.path.size(); i++)
                painter->drawLine(QLineF(u.path[i], u.path[i-1]));
            painter->restore();
        }
    }

    // Trace the best solution node.
    if (bestSolutionNode != 0)
    {
        painter->save();
        Vector<UnicycleSearchNode> trace = bestSolutionNode->trace();
        for (uint i = 0; i < trace.size()-1; i++)
        {
            drawPolygon = hullPolygon;
            drawPolygon.setPose(trace[i].pose());
            painter->setOpacity(0.1);
            painter->setPen(colorUtil.pen);
            painter->setBrush(colorUtil.brushMagenta);
            drawPolygon.draw(painter);
            painter->setOpacity(1.0);
            trace[i].drawTrajectory(painter);
            trace[i].drawNoseCircle(painter, 0.04, colorUtil.brushMagenta);
        }
        painter->restore();
    }

    painter->restore();
}

// Draws the bounding boxes of all contained objects.
void ShortTermAbortingAStar::drawVisibilityGraph(QPainter *painter) const
{
    if (!predictedUnifiedModels.isEmpty())
        predictedUnifiedModels[0].drawVisibilityGraph(painter);
}

// The heuristic function called by A*.
double ShortTermAbortingAStar::heuristic(const Pose2D &from, const Pose2D &to, int depth)
{
    if (heuristicType == command.Euklidean)
    {
        return heuristic_euklidean(from, to);
    }
    else if (heuristicType == command.PathEuklidean)
    {
        const Vector<Vec2>& pp = predictedUnifiedModels[depth].computePath(from.pos(), to.pos());
        bool success = !pp.isEmpty();
        if (success)
            return heuristic_patheuklidean(pp, from, to);
    }
    else if (heuristicType == command.RTR) // rtr
    {
        return heuristic_rtr(from, to);
    }
    else if (heuristicType == command.RTR_MAX) // rtr max
    {
        return heuristic_rtr_max(from, to);
    }
    else if (heuristicType == command.RTR_MIN) // rtr min
    {
        return heuristic_rtr_min(from, to);
    }
    else if (heuristicType == command.MinimalConstruct)
    {
        const Vector<Vec2>& pp = predictedUnifiedModels[depth].computePath(from.pos(), to.pos());
        bool success = !pp.isEmpty();
        if (success)
            return heuristic_pathrtr(pp, from, to);
    }
    else if (heuristicType == command.GridDijkstra)
    {
        const Vector<Vec2>& pp = dilatedSensedGrid->computeDijkstraPath(from.pos(), to.pos());
        //qDebug() << "dijkstra path:" << pp;
        bool success = !pp.isEmpty();
        if (success)
            return heuristic_pathrtr(pp, from, to);
    }

    return -1;
}

bool ShortTermAbortingAStar::getStuck() const
{
    return stuck;
}

void ShortTermAbortingAStar::setStuck(bool value)
{
    stuck = value;
}

// Sets the debug flag.
void ShortTermAbortingAStar::setDebug(int d)
{
    debug = d;
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

int ShortTermAbortingAStar::getPathsComputed() const
{
    return heuristicPathsComputed;
}

// Returns true if the search has come to completion and false if it has been aborted.
bool ShortTermAbortingAStar::isFinished() const
{
    return (bestScoreNode != 0 && heuristic_rtr_max(bestScoreNode->pose(), targetState) < config.staaFinishedThreshold);
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

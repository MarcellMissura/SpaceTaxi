#include "PathAStar.h"
#include "board/State.h"
#include "lib/util/StopWatch.h"
#include "lib/util/DrawUtil.h"

// The PathAStar class implements a plain vanilla 8-neighbour
// A* search in a grid. The grid structure must be given through
// a GridModel object. Usage:
// PathAStar pathAStar;
// pathAStar.init();
// pathAstar.setGridModel(someGrid);
// pathAstar.setStartState(someStart);
// pathAstar.setTargetState(someTarget);
// pathAstar.search();
// Vector<Vec2> path = pathAStar.getPath();
// pathAStar.draw(painter);

PathAStar::PathAStar()
{
    debug = 0;
    expansions = 0;
    bestScoreNode = 0;
}

// Initializes the needed data structures.
void PathAStar::init()
{
    // Prepare the action set.
    actions[0].set(1, 0);
    actions[1].set(-1, 0);
    actions[2].set(0, 1);
    actions[3].set(0, -1);
    actions[4].set(1, 1);
    actions[5].set(-1, 1);
    actions[6].set(1, -1);
    actions[7].set(-1, -1);

    // Cost table for the neighborhood.
    for (int i = 0; i < 8; i++)
        cost[i] = actions[i].norm();
}

// Resets the search for the next search.
void PathAStar::reset()
{
    for (int i = 0; i < nodes.size(); i++)
        nodes[i].reset();
    q.clear();
    expansions = 0;
}

void PathAStar::setDebug(int d)
{
    debug = d;
}

// Provides the grid model for the A* search.
// The grid model is used for isOccupied() checks during the search.
void PathAStar::setGridModel(const GridModel *gm)
{
    gridModel = gm;
    if (nodes.size() < gridModel->getNodeCount())
    {
        nodes.resize(gridModel->getNodeCount());
        for (uint n = 0; n < gridModel->getNodeCount(); n++)
        {
            nodes[n].stateIdx = gridModel->convertIndex(n);
            nodes[n].n = n;
        }
    }
}

// Sets the start state for the A* search.
// The start state comes in world coordinates and is converted to index space.
void PathAStar::setStartState(const Vec2 &p)
{
    startState = gridModel->getNodeIndex(p);
}

// Sets the target state for the A* search.
// The target state comes in world coordinates and is converted to index space.
void PathAStar::setTargetState(const Vec2& p)
{
    targetState = gridModel->getNodeIndex(p);
}

// Performs the A* search and returns true on succes.
// Returns false if the target could not be found.
bool PathAStar::aStarSearch()
{
    if (debug > 0)
    {
        qDebug() << "PathSearch::search() started.";
        qDebug() << "Start State:" << startState << Vec2(gridModel->getNodeCoordinates(startState));
        qDebug() << "Target state:" << targetState << Vec2(gridModel->getNodeCoordinates(targetState));
    }

    // If the start state is in an occupied cell, we
    // switch into an invincible mode where occupied cells are
    // not discarded, but pushed with a large penalty. That way,
    // a path is guaranteed to be found.
    bool invincibleMode = gridModel->isOccupied(startState);

    reset();

    // Prepare the start node.
    uint startStateIdx = gridModel->convertIndex(startState);
    nodes[startStateIdx].parent = 0;
    nodes[startStateIdx].g = 0;
    nodes[startStateIdx].h = (targetState-startState).norm();

    // Push it into the priority queue.
    q.push(&(nodes[startStateIdx]));

    // Keep expanding the top of the queue until the goal is found or the queue is empty.
    while (!q.isEmpty())
    {
        // Pop the best score node from the queue.
        bestScoreNode = q.top();
        q.pop();
        expansions++;

        // Debug output.
        if (debug > 1)
        {
            qDebug() << expansions << "Popped node" << bestScoreNode->stateIdx
                     << Vec2(gridModel->getNodeCoordinates(bestScoreNode->stateIdx))
                     << "h:" << bestScoreNode->h
                     << "g:" << bestScoreNode->g
                     << "f:" << bestScoreNode->f;
        }

        // Check if the goal state has been reached.
        if (bestScoreNode->h <= EPSILON) // Fastest, but requires admissible heuristic.
        {
            if (debug > 0)
            {
                qDebug() << "A* finished!" << "BestScoreNode:" << bestScoreNode
                         //<< "Solution:" << bestScoreNode->stateHistory
                         << "iterations:" << expansions;
            }

            // Set the successor pointers.
            GridSearchNode* node = bestScoreNode;
            while (node->parent != 0)
            {
                node->parent->successor = node;
                node = node->parent;
            }

            return true;
        }

        // Close the node.
        nodes[bestScoreNode->n].closed = true;

        // Expand children and sort them into the priority queue.
        for (int i = 0; i < 8; i++)
        {
            Vec2u childStateIdx = bestScoreNode->stateIdx + actions[i];
            uint childn = gridModel->convertIndex(childStateIdx);

            // Out of bounds check.
            if (!gridModel->contains(childStateIdx))
                continue;

            // Closed check.
            if (nodes[childn].closed)
            {
                if (debug > 3)
                    qDebug() << "    Closed" << actions[i] << "to state" << childStateIdx << ". Skipping.";
                continue;
            }

            // Tentative cost so far (g = g + c).
            double tentative_g = bestScoreNode->g + cost[i] + gridModel->getAt(childStateIdx);

            // Collision check.
            if (gridModel->isOccupied(childStateIdx))
            {
                if (debug > 3)
                    qDebug() << "    Collided" << actions[i] << "to state" << childStateIdx << ". Skipping.";

                if (invincibleMode)
                    tentative_g += 1000;
                else
                  continue;
            }

            // Open check (needs tentative cost).
            if (nodes[childn].g > 0 && nodes[childn].g < tentative_g+EPSILON)
            {
                if (debug > 3)
                    qDebug() << "    Open" << actions[i] << "to state" << childStateIdx << ". Skipping.";
                continue;
            }

            // Populate the child node.
            nodes[childn].parent = bestScoreNode;
            nodes[childn].g = tentative_g;
            nodes[childn].h = (targetState-childStateIdx).norm();
            nodes[childn].f = nodes[childn].h + nodes[childn].g;

            // Push the child.
            q.push(&nodes[childn]);

            if (debug > 2)
            {
                qDebug() << "    Pushed" << actions[i] << "x:" << nodes[childn].stateIdx
                         << "h:" << nodes[childn].h << "g:" << nodes[childn].g << "f:" << nodes[childn].f;
            }
        }
    }

    if (debug > 0)
        qDebug() << "Path A* finished with empty queue!";

    if (debug > 0)
    {
        qDebug() << "A* aborted."
                 << "iterations:" << expansions
                 << "Best score node:" << bestScoreNode;
    }

    return false;
}

// Performs the LazyTheta* search.
bool PathAStar::lazyThetaStarSearch()
{
    if (debug > 0)
    {
        qDebug() << "LazyThetaStar::search() started.";
        qDebug() << "Start State:" << startState;
        qDebug() << "Target state:" << targetState;
    }

    // If the start state is in an occupied cell, we
    // switch into an invincible mode where occupied cells are
    // not discarded, but pushed with a large penalty. That way,
    // a path is guaranteed to be found.
    bool invincibleMode = gridModel->isOccupied(startState);

    reset();

    // Prepare the start node.
    uint startStateIdx = gridModel->convertIndex(startState);
    nodes[startStateIdx].parent = 0;
    nodes[startStateIdx].g = 0;
    nodes[startStateIdx].h = (targetState-startState).norm();

    // Push it into the priority queue.
    q.push(&(nodes[startStateIdx]));

    // Keep expanding the top of the queue until the goal is found or the queue is empty.
    while (!q.isEmpty())
    {
        // Pop the best score node from the queue.
        bestScoreNode = q.top();
        q.pop();
        expansions++;

        // Debug output.
        if (debug > 1)
        {
            qDebug() << expansions << "Popped node" << bestScoreNode->stateIdx
                     << "h:" << bestScoreNode->h
                     << "g:" << bestScoreNode->g
                     << "f:" << bestScoreNode->f;
            if (bestScoreNode->parent != 0)
                qDebug() << "parent:" << bestScoreNode->parent->stateIdx << "g:" << bestScoreNode->parent->g << (void*)bestScoreNode->parent;
        }

        // Lazy Theta* test.
        // Test if there is an actual line of sight to the parent of this node.
        // If not, replace the parent with the cell from the direct neighborhood
        // of the node that is closed and has the lowest g.
        if (bestScoreNode->parent != 0 && !gridModel->hasLineOfSight(bestScoreNode->stateIdx, bestScoreNode->parent->stateIdx))
        {
            if (debug > 1)
                qDebug() << "   No LOS to parent.";

            double min_g = std::numeric_limits<double>::max();
            int minParentStateIdx = -1;
            for (int i = 0; i < 8; i++)
            {
                Vec2u state = bestScoreNode->stateIdx + actions[i];
                uint stateIdx = gridModel->convertIndex(state);

                // Out of bounds check.
                if (!gridModel->contains(state))
                    continue;

                // Closed check.
                if (!nodes[stateIdx].closed)
                    continue;

                double tentative_g = nodes[stateIdx].g + cost[i];
                if (tentative_g < min_g)
                {
                    min_g = tentative_g;
                    minParentStateIdx = stateIdx;
                }

                if (debug > 1)
                    qDebug() << "   No LOS to parent. New parent:" << nodes[minParentStateIdx].stateIdx;
            }

            // Repopulate the child node.
            uint bestScoreStateIdx = gridModel->convertIndex(bestScoreNode->stateIdx);
            nodes[bestScoreStateIdx].parent = &nodes[minParentStateIdx];
            nodes[bestScoreStateIdx].g = min_g;
            nodes[bestScoreStateIdx].f = nodes[bestScoreStateIdx].h + nodes[bestScoreStateIdx].g;
        }

        // Check if the goal state has been reached.
        if (bestScoreNode->h <= EPSILON) // Fastest, but requires admissible heuristic.
        {
            if (debug > 0)
            {
                qDebug() << "LazyTheta* finished!" << "BestScoreNode:" << bestScoreNode << (void*)(bestScoreNode->parent)
                         << "iterations:" << expansions;
            }

            // Set the successor pointers.
            GridSearchNode* node = bestScoreNode;
            while (node->parent != 0)
            {
                node->parent->successor = node;
                node = node->parent;
            }

            return true;
        }

        // Close the node.
        nodes[bestScoreNode->n].closed = true;

        // Expand children and sort them into the priority queue.
        for (int i = 0; i < 8; i++)
        {
            Vec2u childStateIdx = bestScoreNode->stateIdx + actions[i];
            uint childn = gridModel->convertIndex(childStateIdx);

            // Out of bounds check.
            if (!gridModel->contains(childStateIdx))
                continue;

            // Closed check.
            if (nodes[childn].closed)
            {
                if (debug > 3)
                    qDebug() << "    Closed" << childStateIdx << ". Skipping.";
                continue;
            }

            // Tentative cost so far (g = g + c).
            GridSearchNode* parent = 0;
            double tentative_g = 0;
            if (bestScoreNode->parent != 0)
            {
                tentative_g = bestScoreNode->parent->g;
                tentative_g += (bestScoreNode->parent->stateIdx - childStateIdx).norm();
                //qDebug() << "g from parent:" << bestScoreNode->parent->g << "+" << (bestScoreNode->parent->state - state).norm();
                parent = bestScoreNode->parent;
            }
            else
            {
                tentative_g = bestScoreNode->g + cost[i];
                parent = bestScoreNode;
            }

            tentative_g += gridModel->getAt(childStateIdx);

            // Collision check.
            if (gridModel->isOccupied(childStateIdx))
            {
                if (debug > 3)
                    qDebug() << "    Collided" << actions[i] << "to state" << childStateIdx << ". Skipping.";

                if (invincibleMode)
                    tentative_g += 1000;
                else
                  continue;
            }

            // Open check with tentative cost.
            if (nodes[childn].g > 0 && nodes[childn].g < tentative_g+EPSILON)
            {
                if (debug > 3)
                    qDebug() << "    Open" << childStateIdx << "g:" << nodes[childn].g << "(tentative_g:" << tentative_g << "). Skipping.";
                continue;
            }

            // Populate the child node.
            nodes[childn].parent = parent;
            nodes[childn].g = tentative_g;
            nodes[childn].h = (targetState-childStateIdx).norm();
            nodes[childn].f = nodes[childn].h + nodes[childn].g;

            // Push the child.
            q.push(&nodes[childn]);

            if (debug > 2)
            {
                qDebug() << "    Pushed" << nodes[childn].stateIdx
                         << "h:" << nodes[childn].h << "g:" << nodes[childn].g << "f:" << nodes[childn].f;
            }
        }
    }

    if (debug > 0)
        qDebug() << "LazyTheta* finished with empty queue!";

    if (debug > 0)
    {
        qDebug() << "LazyTheta* aborted."
                 << "iterations:" << expansions
                 << "Best score node:" << bestScoreNode;
    }

    return false;
}

// Returns the point d meters along the path.
Vec2 PathAStar::getWaypoint(double d)
{
    GridSearchNode* node = bestScoreNode;
    while(node->parent != 0)
        node = node->parent;
    double dist = d;
    Vec2 pos = gridModel->getNodeCoordinates(node->stateIdx);
    while(node->successor != 0 && dist > 0)
    {
        node = node->successor;
        Vec2 pos2 = gridModel->getNodeCoordinates(node->stateIdx);
        if ((pos2-pos).norm() < d - dist)
        {
            dist -= (pos2-pos).norm();
        }
        else
        {
            return pos + (pos2-pos).normalized(dist);
        }
        pos = pos2;
    }
    return pos;
}

// Returns the found path.
Vector<Vec2> PathAStar::getPath() const
{
    Vector<Vec2> path;
    GridSearchNode* node = bestScoreNode;
    while (node != 0)
    {
        path << gridModel->getNodeCoordinates(node->stateIdx);
        node = node->parent;
    }
    path.reverse();
    return path;
}

// QPainter drawing code.
void PathAStar::draw(QPainter *painter) const
{
    if (bestScoreNode == 0)
        return;

    Vec2 stride = gridModel->getStride();
    painter->save();
    painter->scale(stride.x, stride.y);

    // Draw the whole tree.
    for (int i = 0; i < nodes.size(); i++)
        if (nodes[i].closed)
            nodes[i].draw(painter);

    // Draw the path.
    if (bestScoreNode != 0)
    {
        GridSearchNode* node = bestScoreNode;
        QPainterPath p;
        p.moveTo(node->stateIdx.x, node->stateIdx.y);
        while (node != 0)
        {
            p.lineTo(node->stateIdx.x, node->stateIdx.y);
            node = node->parent;
        }

        double s = 0.03;
        painter->strokePath(p, drawUtil.penRedThick);
        painter->setBrush(drawUtil.brushOrange);
        painter->setPen(drawUtil.penOrange);
        for (int i = 0; i < p.elementCount(); i++)
            painter->drawEllipse(p.elementAt(i), s, s);
    }

    double s = 0.5;

    // Draw the start state.
    painter->setPen(drawUtil.pen);
    painter->setBrush(drawUtil.brushRed);
    painter->drawEllipse(startState, s, s);

    // Draw the goal state.
    painter->setPen(drawUtil.pen);
    painter->setBrush(drawUtil.brushGreen);
    painter->drawEllipse(targetState, s, s);

    painter->restore();
}

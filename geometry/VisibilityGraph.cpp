#include "VisibilityGraph.h"
#include "globals.h"
#include "blackboard/Config.h"
#include "util/ColorUtil.h"
#include "util/StopWatch.h"
#include <GL/glu.h>
#include "util/GLlib.h"
#include "GeometricModel.h"

// The VisibilityGraph is a connectivity graph in a polygonal environment.
// The corners of the polygons are connected with each other if their connection
// does not intersect with any polygon in the scene. The set of these connections
// is the visibility graph. The visibility graph can be searched for the shortest
// path from a start to a target location. This class harbours the Minimal Construct
// algorithm, which is particularly fast in finding the shortest path, because it
// doesn't compute the entire visibility graph, only a minimal portion of it.
//
// Usage:
// VisibilityGraph visibilityGraph;
// visibilityGraph.setGeometricModel(gm); // The geometric model has to come from somewhere.
// visibilityGraph.setStartState(s);
// visibilityGraph.setTargetState(t);
// bool success = visibilityGraph.minimalConstruct();
// Vector<Vec2> path = visibilityGraph.getPath();
//
// Internally, a list of Nodes and an AdjacencyMatrix represent the graph.
// Currently there are three different algorithms implemented to construct and
// search the graph. fullConstruct() is an O(n²) Overmars Welzl algorithm that
// performs a polar sweep with the help of a rotation tree. naiveConstruct() is
// the naive O(n³) algorithm that intersects every edge of the graph with every
// line (side of polygon). minimalConstruct() is my one shot path finder that
// does not construct the entire graph, but only what is needed to find the
// shortest path.

// Decides if this Node (p) can be tangentially connected with Node q.
bool Node::isTangentialTo(Node *q) const
{
    // Polygon edges are always considered to be tangential.
    if (lineId1 == q->lineId2 || lineId2 == q->lineId1)
        return true;

    // Otherwise the actual tangential test needs to be computed.
    // Remember that v1 is the vector from this vertex to the next vertex in the polygon,
    // v2 is the vector from this vertex to the previous vertex, and v3 is the vector
    // from this vertex to the point q.
    Vec2 v3 = *q-*this;
    double detv3pv1 = v3.x*v1.y-v3.y*v1.x;
    double detv3pv2 = v3.x*v2.y-v3.y*v2.x;
    double detv3qv1 = v3.x*q->v1.y-v3.y*q->v1.x;
    double detv3qv2 = v3.x*q->v2.y-v3.y*q->v2.x;
    bool res = ( (obstId == -1 || detv3pv1 > -EPSILON && detv3pv2 > -EPSILON || detv3pv1 < EPSILON && detv3pv2 < EPSILON)
                 &&
                 (q->obstId == -1 || detv3qv1 > -EPSILON && detv3qv2 > -EPSILON || detv3qv1 < EPSILON && detv3qv2 < EPSILON)
               );
    return res;
}

bool Node::isClosed() const
{
    return closed;
}

VisibilityGraph::VisibilityGraph()
{
    tangentialTests = 0;
    sameSideTests = 0;
    lineIntersectionTests = 0;
    debug = 0;
    searchIterations = 0;
    startNode = 0;
}

// Resets the graph to an initial state where only the start and the goal
// vertices are present and connected. The nodes and the adjacency matrix
// are cleared except for the start node and the target node and the
// connection from the start node to the target node.
void VisibilityGraph::reset()
{
    nodes.clear();
    adjacencyMatrix.clear();
    polygonClosed.clear();
    lineIntersectionTests = 0;
    tangentialTests = 0;
    sameSideTests = 0;
    searchIterations = 0;

    // There is this convention that the first node in the nodes list
    // is the start node with the id 0 and the second node in the list
    // is the target node with the id 1.
    Node sn;
    sn.id = 0;
    sn.closed = true; // The start node should always be closed for parenting.
    nodes << sn;
    startNode = &nodes.last();

    Node tn(target);
    tn.id = 1;
    nodes << tn;
    targetNode = &nodes.last();

    adjacencyMatrix.set(1, 0); // This dodges a bunch of memory allocation issues.
}

// Sets the geometric model that the graph construction algorithms needs to check line intersections.
void VisibilityGraph::setGeometricModel(const GeometricModel &gm)
{
    this->gm = &gm;
}

// Sets the bounding box of the visibility graph.
// This box is used to confine the search within.
void VisibilityGraph::setBounds(double t, double l, double b, double r)
{
    boundingBox.set(t, l, b, r);
}

// Returns the bounding box.
const Box &VisibilityGraph::getBounds() const
{
    return boundingBox;
}

void VisibilityGraph::setDebug(int d)
{
    debug = d;
}

// The start state the search will originate from.
void VisibilityGraph::setStart(const Vec2 &p)
{
    start = p;
}

// Sets the target state. It automatically resets the graph, too.
void VisibilityGraph::setTarget(const Vec2 &p)
{
    if (target != p)
    {
        target = p;
        reset();
    }
}

// A private method to initialize the minimal construct search.
// It exists mostly so that it would show in the profiler. This
// method is called by minimalConstruct() before every search to
// clear the queue and to link the start node into the graph.
void VisibilityGraph::initMinimalConstruct()
{
    // Clear the priority queue and the computed path.
    q.clear();
    path.clear();

    // Every known neighbor of the start node needs to be pushed to initialize the search.
    // When MinimalConstruct is started with a blank graph, only the connection to the target
    // node is known and so only one node will be pushed into the q, but when the search is
    // restarted and the graph is reused, all connections involving the start need to be pushed.
    // This includes basically every vertex of the graph minus nontangential connections, since
    // the start node has changed and no line intersection tests have yet been made. I am not yet
    // sure how well this behaves in large graphs. Reusing the graph saves on line intersection
    // checks that don't need to be repeated though, so it makes sense to try this.
    ListIterator<Node> it = nodes.begin();
    it.next(); // Skip start node.
    while (it.hasNext())
    {
        Node& child = it.next();
        child.closed = false; // Reset the closed state after the last search.
        //child.parent = 0; // Reset the parent after the last search.

        // We can skip children that have a nontangential connection with the start node.
        if (!child.isTangentialTo(startNode))
        {
            if (debug > 5)
                qDebug() << "   " << child.id << "is not tangential with start node.";
            adjacencyMatrix.unset(child.id, startNode->id);
            continue; // next child
        }

        // Connect with the start node in the adjacency matrix.
        adjacencyMatrix.set(child.id, startNode->id);

        // Update parent, f, g, and h.
        child.parent = startNode;
        child.g = (*startNode-child).norm();
        child.h = (*targetNode-child).norm();
        child.f = child.h+child.g;

        // Push the child into the priority queue.
        q.push(&(child));

        if (debug > 2)
            qDebug() << "   Pushed" << child.id << "p:" << child.parent->id << "g:" << child.g << "h:" << child.h << "f:" << child.f;
    }
}

// This algorithm is a repairing A* search that incrementally constructs the
// visibility graph while it searches for the shortest path. It begins with the direct
// line from start to target. If this line intersects a Polygon, the corners
// of the Polygon are fully connected to the graph without computing line intersection
// tests. Intersection checks are only performed on the lines that A* opens to a new
// waypoint. During the search, when the opened edge fails the intersection test, the
// A* needs to be repaired by removing the collided edge, reparenting the opened node,
// and adding the waypoints of the intersected Polygon to the graph.
// The resulting path is returned and also written into the path. You can query it with
// getPath(). resetGraph() has to be called once before a search. Then, repeated calls
// to minimalConstruct() will reuse the graph until resetGraph() is called again.
bool VisibilityGraph::minimalConstruct(const Vec2& start)
{
    if (debug > 0)
    {
        qDebug() << "VisibilityGraph::minimalConstruct() started.";
        qDebug() << "Start:" << start << "Target:" << target;
    }

    // Change the location of the start node.
    startNode->x = start.x;
    startNode->y = start.y;

    // The new start node will be reconnected with every known node, especially
    // with the target node, in the initMinimalConstruct() function.
    initMinimalConstruct();

    // And let's go.
    while (!q.isEmpty())
    {
        // Pop the best score node from the queue.
        Node* bestScoreNode = q.pop();

        if (debug > 1)
            qDebug() << "Popped id:" << bestScoreNode->id
                     << "p:" << bestScoreNode->parent->id
                     << "g:" << bestScoreNode->g
                     << "h:" << bestScoreNode->h
                     << "f:" << bestScoreNode->f;

        // 1. Line intersection test.
        // Now the line between the popped node and its parent is checked for intersection with the polygons.
        // This is the most expensive step of the algorithm. This is why it's only performed inside the A* search
        // when a node is popped from the queue. Additional savings are drawn from caching the performed tests in
        // the adjacency matrix.
        if (!adjacencyMatrix.isChecked(bestScoreNode->id, bestScoreNode->parent->id)) // edge already checked check :)
        {
            Line line(*(bestScoreNode->parent), *bestScoreNode);
            int collidedPolygonIdx = gm->lineCollisionCheck(line);
            adjacencyMatrix.check(bestScoreNode->id, bestScoreNode->parent->id);
            lineIntersectionTests++;
            if (collidedPolygonIdx >= 0)
            {
                if (debug > 3)
                    qDebug() << "   Line from " << *bestScoreNode->parent << "to" << *bestScoreNode << "collides with Polygon" << gm->getObstacle(collidedPolygonIdx).getId();

                // There was a collision. Repair the graph and the A* search.

                // Remove the failed edge from the graph.
                adjacencyMatrix.unset(bestScoreNode->id, bestScoreNode->parent->id);

                // Unparent the node.
                bestScoreNode->parent = 0;

                // Attempt to reparent the node.
                // Parenting a node means finding the closed node among the neighbours of the node
                // with the lowest g+c value and setting it as the parent, or assigning no parent
                // if no such candidate was found. The reason for this is that the last popped
                // node was open through the edge we just removed and this might have prevented the
                // node to be opened and pushed through an another edge. Assigning the popped node
                // as a child to the lowest cost neighbor that has the open status and pushing the
                // node back into the queue fixes this error.
                findParent(bestScoreNode);

                // If the collided polygon is not closed yet, fully connect the waypoints of the polygon with the graph.
                if (!polygonClosed.contains(collidedPolygonIdx))
                {
                    if (debug > 3)
                        qDebug() << "   Adding Polygon" << gm->getObstacle(collidedPolygonIdx).getId() << "to the graph.";

                    // Prepare an iterator for the new nodes only (that are yet to be added).
                    ListIterator<Node> newNodesIterator = nodes.end();

                    const Obstacle& collidedPolygon = gm->getObstacle(collidedPolygonIdx);
                    connectPolygonFully(collidedPolygon);

                    // Advance the new nodes iterator by one to point at the first new node.
                    newNodesIterator.next();

                    // Parent the new nodes, if possible. In the process of parenting, the new nodes may already be pushed.
                    while (newNodesIterator.hasNext())
                    {
                        Node& node = newNodesIterator.next();
                        findParent(&node);
                    }

                    polygonClosed.insert(collidedPolygonIdx, true);
                }

                continue; // back to pop
            }
        }

        // Check if the goal state has been reached.
        if (bestScoreNode->h < EPSILON)
        {
            if (debug > 0)
            {
                qDebug() << "Target node reached." << *bestScoreNode;
            }

            // Follow the parent pointers back to create the path in reverse.
            path.clear();
            Node* node = bestScoreNode;
            int counter = 0;
            while (node != 0 && counter < 100)
            {
                counter++;
                if (counter == 100)
                {
                    qDebug() << "endless loop in MinimalConstruct.";
                    qDebug() << *node << node << *node->parent;
                }

                path.push_back(*node);
                if (node->parent != 0)
                    node->parent->successor = node;
                node = node->parent;
            }

            // Reverse the reverse. :)
            path.reverse();

            if (debug > 0)
                qDebug() << "minimalConstruct() finished! Path:" << path;

            return true;
        }

        // Close the node.
        bestScoreNode->closed = true;


        // 2. Check if a state with a successor has been reached.
        // Successors are known to be the next node along the shortest
        // path to the goal, so nothing else needs to be expanded.
        if (bestScoreNode->successor != 0)
        {
            if (debug > 3)
            {
                qDebug() << "Path connected at:" << *bestScoreNode;
            }

            Node& child = *bestScoreNode->successor;

            // Closed check. Does this exist?
            if (child.isClosed())
            {
                if (debug > 4)
                    qDebug() << "   Closed successor" << child.id << child << "g:" << child.g << "f:" << child.f;
                continue; // next child
            }

            // Open check.
            // Compute the tentative cost for the child, but update the child only if the cost would improve.
            double tentative_g = bestScoreNode->g + (*bestScoreNode-child).norm(); // g + c
            if (child.getPidx() > 0 && child.g < tentative_g + EPSILON)// child is open but already has lower cost
            {
                if (debug > 4)
                    qDebug() << "   Open successor" << child.id << child << "p:" << child.parent->id << "g:" << child.g << "(" << tentative_g << ") f:" << child.f;
                continue; // next child
            }

            // Update parent, f, g, and h.
            child.parent = bestScoreNode;
            child.g = tentative_g;
            //child.h = (*targetNode-child).norm(); // Should already know.
            child.f = child.h+child.g;

            // Sort the new node into the priority queue.
            q.push(&(child));

            if (debug > 3)
                qDebug() << "   Pushed successor" << child.id << child << "p:" << child.parent->id << "g:" << child.g << "h:" << child.h << "f:" << child.f;

            continue; // back to pop
        }


        // Otherwise expand children and sort them into the priority queue.
        ListIterator<Node> it = nodes.begin();
        for (uint i = 0; i < nodes.size(); i++)
        {
            Node& child = it.next();

            //qDebug() << "Expanding" << child.id << adjacencyMatrix.isSet(child.id, bestScoreNode->id);

            // Adjacency matrix-based looping over children.
            if (!adjacencyMatrix.isSet(child.id, bestScoreNode->id))
                continue;

            // Closed check.
            if (child.closed)
            {
                if (debug > 3)
                    qDebug() << "   Closed" << child.id << child << "g:" << child.g << "f:" << child.f;
                continue; // next child
            }

            // Open check.
            // Compute the tentative cost for the child, but update the child only, if the cost would improve.
            double tentative_g = bestScoreNode->g + (*bestScoreNode-child).norm(); // g + c
            if (child.getPidx() > 0 && child.g < tentative_g + EPSILON)// child is open but already has lower cost
            {
                if (debug > 3)
                    qDebug() << "   Open" << child.id << child << "p:" << child.parent->id << "g:" << child.g << "(" << tentative_g << ") f:" << child.f;
                continue; // next child
            }

            // We can skip children that have a nontangential connection with the parent.
            // Delaying the tangential check to before the push has the advantage of not having
            // to compute the tangential check for the lines that never get pushed.
            if (!child.isTangentialTo(bestScoreNode))
            {
                if (debug > 4)
                    qDebug() << "   Not tangential" << child.id << child << "g:" << child.g << "(" << tentative_g << ") f:" << child.f;
                adjacencyMatrix.unset(child.id, bestScoreNode->id);
                continue; // next child
            }

            if (child.getPidx() > 0) // child is open (a debug message only)
            {
                if (debug > 3)
                    qDebug() << "   Upgrade" << child.id << "p:" << child.parent->id << "g:" << child.g << "tentative g:" << tentative_g << "h:" << child.h << "f:" << child.f;
            }

            // Update parent, f, g, and h.
            child.parent = bestScoreNode;
            child.g = tentative_g;
            child.h = (*targetNode-child).norm();
            child.f = child.h+child.g;

            // Sort the new node into the priority queue.
            q.push(&(child));

            if (debug > 2)
                qDebug() << "   Pushed" << child.id << child << "p:" << child.parent->id << "g:" << child.g << "h:" << child.h << "f:" << child.f;
        }
    }

    if (debug > 0)
        qDebug() << "   Empty queue. minimalConstruct() failed to find a solution.";

    return false;
}

// Fully connects a Polygon into the visiblity graph without performing any line intersection checks.
void VisibilityGraph::connectPolygonFully(const Obstacle &obst)
{
    // This function "fully" connects a possibly nonconvex Polygon with the visibility
    // graph in a way that no line intersection tests are made. The edges between the
    // corners of the polygon and all known nodes so far are set in the adjacency matrix.
    // Concave corners are skipped, but tangential checks are not yet performed. They
    // are delayed until before a push operation occurs.

    // Set up an iterator for the new nodes only.
    ListIterator<Node> newNodesIterator = nodes.end();

    uint firstId = 2;
    if (!nodes.isEmpty())
        firstId = nodes.last().id+1;
    uint lastId = firstId;

    // Create nodes from the vertices of the polygon.
    ListIterator<Vec2> vertices = obst.vertexIterator();
    for (int i = 0; i < obst.size(); i++)
    {
        // Compute the in and out edges of the corner.
        // These will be used for concavity and tangential testing.
        Vec2 v = vertices.peekCur();
        Vec2 v1 = vertices.peekNext() - v;
        Vec2 v2 = vertices.peekPrev() - v;
        vertices.next();

        // Concave corners can be skipped right away.
        if (v2.isRightOf(v1))
            continue;

        // Out of bounds polygon corners are skipped.
        if (!boundingBox.isEmpty() && !boundingBox.intersects(v, -0.01))
            continue;

        // Add the Polygon corner to the nodes.
        // The line ids are computed from the id of the polygon.
        nodes << Node(v);
        Node& node = nodes.last();
        node.id = lastId++;
        node.obstId = obst.getId();
        node.lineId1 = obst.getBaseLineId()+i;
        node.lineId2 = i==0?obst.getBaseLineId()+obst.size()-1:node.lineId1-1;
        node.v1 = v1;
        node.v2 = v2;
    }

    // Advance the new nodes iterator by one to have it point at the first new node.
    newNodesIterator.next();
    //ListIterator<Node> newNodesIterator2 = newNodesIterator;

    // Now connect all nodes so far with the new nodes, and also connect the new nodes among each other.
    // No checks are made whatsoever.
    ListIterator<Node> it = nodes.begin();
    while (it.hasNext())
    {
        Node& outerNode = it.next();
        if (outerNode.id == newNodesIterator.peekCur().id && newNodesIterator.hasNext())
            newNodesIterator.next();

        ListIterator<Node> it2 = newNodesIterator;
        while (it2.hasNext())
        {
            Node& innerNode = it2.next();
            adjacencyMatrix.set(innerNode.id, outerNode.id);
            if (debug > 5)
                qDebug() << "   Edge" << innerNode.id << outerNode.id << innerNode << "to" << outerNode << "added to graph.";
        }
    }

    // I experimented with marking all polygon edges as checked, and it does make things
    // faster, but then polygons must not overlap, so no.

/*
    Node& firstNode = newNodesIterator2.next();
    Node lastNode = firstNode;
    while (newNodesIterator2.hasNext())
    {
        Node& thisNode = newNodesIterator2.next();
        if (thisNode.isAPolygonEdge(&lastNode))
            adjacencyMatrix.check(thisNode.id, lastNode.id);
        lastNode = thisNode;
    }
    if (firstNode.isAPolygonEdge(&lastNode))
        adjacencyMatrix.check(lastNode.id, firstNode.id);
*/
}

// Finds a parent for a dangling node.
// This is a part of the minimalConstruct() algorithm.
// Parenting a node means finding a closed node among the neighbours of the node
// with the lowest g+c value and setting it as the parent, or assigning no parent
// if no such candidate was found.
void VisibilityGraph::findParent(Node* orphan)
{
    // The neighborhood relation is encoded as an Adjacency Matrix (AM).
    // Iterating through the right column of the AM and checking each edge if
    // it's "set" is a fast way of retrieving the neighbours. Of course, it would
    // be even faster to iterate through a "children" list of the node, but that
    // has disadvantages when removing nodes from the graph. I should consider
    // keeping a closed list though.

    if (debug > 5)
        qDebug() << "   Reparenting" << orphan->id << *orphan;

    Node* parent = 0;
    double minCost = std::numeric_limits<double>::max();
    ListIterator<Node> it = nodes.begin();
    for (uint i = 0; i < nodes.size(); i++)
    {
        Node& node = it.next();

        if (!node.isClosed()) // A parent has to be closed.
            continue;

        if (!adjacencyMatrix.isSet(node.id, orphan->id)) // It has to be a neighbour of v.
            continue;

        if (!node.isTangentialTo(orphan)) // It has to be tangential even.
        {
            if (debug > 4)
                qDebug() << "   Not tangential" << orphan->id << "to" << node.id << node;
            adjacencyMatrix.unset(node.id, orphan->id);
            continue;
        }

        // Set the parent if better than the parent so far.
        double cost = node.g + (*orphan-node).norm();
        if (cost < minCost)
        {
            minCost = cost;
            parent = &(node);
        }
    }

    if (parent == 0)
        return;

    orphan->parent = parent;
    orphan->g = minCost;
    orphan->h = (target-*orphan).norm();
    orphan->f = orphan->g + orphan->h;
    q.push(orphan);
    if (debug > 4)
    {
        qDebug() << "   Parented" <<  orphan->id << "to" << orphan->parent->id;
        qDebug() << "   Pushed" << orphan->id << *orphan << "p:" << orphan->parent->id << "g:" << orphan->g << "h:" << orphan->h << "f:" << orphan->f;
    }
}

// Returns the computed path.
const Vector<Node>& VisibilityGraph::getPath() const
{
    return path;
}

// Draws the visibility graph on a QPainter.
void VisibilityGraph::draw(QPainter *painter) const
{
    painter->save();

    // The visibility graph edges taken from the adjacency matrix.
    painter->setPen(colorUtil.penThin);
    painter->setBrush(colorUtil.brushDarkGray);
    painter->setOpacity(0.2);
    ListIterator<Node> it = nodes.begin();
    while (it.hasNext())
    {
        Node node = it.next();
        ListIterator<Node> it2 = it;
        while (it2.hasNext())
        {
            Node node2 = it2.next();
            if (adjacencyMatrix.isChecked(node.id, node2.id))
                painter->drawLine(node, node2);
        }
    }

    // The visibility graph nodes.
    painter->setPen(Qt::NoPen);
    painter->setBrush(colorUtil.brush);
    painter->setOpacity(1.0);
    double s = 0.03;
    it = nodes.begin();
    while (it.hasNext())
    {
        Node node = it.next();
        painter->save();
        painter->translate(node.x, node.y);
        painter->drawEllipse(QPointF(), s, s);
        painter->restore();
    }

    // Node labels.
    QFont font;
    font.setFamily("Arial");
    font.setPointSize(1);
    painter->setFont(font);
    painter->setPen(colorUtil.pen);
    painter->setOpacity(0.8);

    it = nodes.begin();
    while (it.hasNext())
    {
        Node node = it.next();

        painter->save();
        painter->translate(node.x + 0.05, node.y + 0.05);
        painter->scale(0.2, -0.2);
        painter->drawText(QPointF(), QString::number(node.id));
        painter->restore();
    }

    painter->restore();
}

// Draws the path computed by the visibility graph on a QPainter.
void VisibilityGraph::drawPath(QPainter *painter) const
{
    painter->save();
    painter->setPen(colorUtil.penBlueThick);
    painter->setBrush(colorUtil.brushBlue);
    painter->setOpacity(0.2);
    for (int i = 1; i < path.size(); i++)
        painter->drawLine(QLineF(path[i].x, path[i].y, path[i-1].x, path[i-1].y));
    painter->restore();
}

QDebug operator<<(QDebug dbg, const VisibilityGraph &w)
{
    dbg << "Path:" << w.path << "\n";
    return dbg;
}

QDebug operator<<(QDebug dbg, const Node &o)
{
    dbg << o.id << "["  << o.x << o.y << "]";
    return dbg;
}

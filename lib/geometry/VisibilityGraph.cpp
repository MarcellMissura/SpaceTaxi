#include "VisibilityGraph.h"
#include "GeometricModel.h"
#include "blackboard/Config.h"
#include "blackboard/State.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/StopWatch.h"
#include "lib/util/GLlib.h"
#include <GL/glu.h>

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
// The Minimal Construct algorithm is programmed in a way that it reuses the nodes and
// the adjacency matrix from earlier searches until resetGraph() is called. This way,
// consecutive searches in the same graph towards the same target can be executed much
// faster, which is for example the case with the Aborting A* controller where heaps of
// shortest paths are computed in the same graph.

VisibilityGraph::VisibilityGraph()
{
    tangentialTests = 0;
    lineIntersectionTests = 0;
    startNode = 0;
    targetNode = 0;
    gm = 0;
    reset();
}

// Copy constructor.
VisibilityGraph::VisibilityGraph(const VisibilityGraph &o)
{
    *this = o;
}

// Assignment operator.
VisibilityGraph& VisibilityGraph::operator=(const VisibilityGraph &o)
{
    gm = o.gm; // The geometric model.
    nodes = o.nodes; // These are the nodes of the visibility graph.
    startNode = &nodes.first();
    targetNode = &(nodes.begin().peekNext());
    adjacencyMatrix = o.adjacencyMatrix; // Adjacency matrix of the graph.
    polygonClosed = o.polygonClosed; // A closed list for the polygons that have been added to the graph.
    boundingBox = o.boundingBox; // A bounding box that confines the search to remain within.
    path = o.getPath(); // The result of a path search.
    lineIntersectionTests = o.lineIntersectionTests;
    tangentialTests = o.tangentialTests;

    return *this;
}

// Resets the graph to an initial state where only the start and the goal
// vertices are present in the nodes list. The nodes and the adjacency matrix
// are cleared except for the start node and the target node and the connection
// from the start node to the target node.
void VisibilityGraph::reset()
{
    nodes.clear();
    adjacencyMatrix.clear();
    polygonClosed.clear();
    lineIntersectionTests = 0;
    tangentialTests = 0;

    // There is this convention that the first node in the nodes list
    // is the start node with the id 0 and the second node in the list
    // is the target node with the id 1.
    Node sn;
    sn.id = 0;
    sn.closed = true; // The start node should always be closed for parenting.
    nodes << sn;
    startNode = &nodes.last();

    Node tn;
    tn.id = 1;
    nodes << tn;
    targetNode = &nodes.last();

    adjacencyMatrix.set(1, 0);
    adjacencyMatrix.set(0, 1);
}

// Sets the geometric model that is needed for the construction of the visibility graph.
void VisibilityGraph::setGeometricModel(const GeometricModel* gm)
{
    this->gm = gm;
    reset();
}

// Sets the scene bounding box of the visibility graph. This box is used to confine the
// search for the shortest path within the rectangular area of a local map.
void VisibilityGraph::setBounds(double t, double l, double b, double r)
{
    boundingBox.set(t, l, b, r);
}

// Sets the scene bounding box of the visibility graph. This box is used to confine the
// search for the shortest path within the rectangular area of a local map.
void VisibilityGraph::setBounds(const Box& bb)
{
    boundingBox = bb;
}

// Returns the bounding box.
const Box &VisibilityGraph::getBounds() const
{
    return boundingBox;
}

// Searches the polygonal scene for the shortest path from "from" to "to".
// In order to guarantee a path even in situations where "from" or "to" are
// occluded, both from and to are moved out of obstacles (and into free space).
// A GeometricModel must have been provided beforehand using setGeometricModel().
// The starting point and the target of the search are provided as an argument to
// this function. The returned boolean indicates whether the path was successfully
// found. A path search can be unsuccessful when there really is no way from
// start to target in the map. After a successful search, you can retrieve the
// found path using the getPath() function.
bool VisibilityGraph::computePath(const Vec2 &from, const Vec2 &to, int debug)
{
    if (debug > 0)
        qDebug() << state.frameId << "computePath(const Vec2 &from, const Vec2 &to):" << from << to;

    if (gm == 0)
    {
        qDebug() << "VisibilityGraph::computePath(): no geometric model has been set.";
        return false;
    }

    Vec2 movedStart = gm->moveIntoFreeSpace(from, debug > 0);
    Vec2 movedTarget = gm->moveIntoFreeSpace(to, debug > 0);

    // Use the visibility graph to compute the path.
    return minimalConstruct(movedStart, movedTarget, debug);
}

// A private method to initialize the minimal construct search.
// It exists mostly so that it would show in the profiler. This
// method is called by minimalConstruct() before every search to
// clear the queue and to link a new start node into the graph.
// The adjacency matrix is preserved though to speed up multiple
// searches in the same graph.
void VisibilityGraph::startNodeChanged(int debug)
{
    // Every known neighbor of the start node needs to be pushed to initialize the search.
    // When MinimalConstruct is started with a blank graph, only the connection to the target
    // node is known and so only one node will be pushed into the q, but when the search is
    // restarted and the graph is reused, all connections involving the start need to be pushed.
    // This includes basically every vertex of the graph minus nontangential connections. I am
    // not yet sure how well this behaves in large graphs. On the one hand, pushing a lot of
    // connections that might not be used costs some effort, but reusing the graph saves on line
    // intersection checks that don't need to be repeated. To relate the costs and the savings,
    // the initial pushing of the start node in a known graph is in O(N log N) and it increases
    // the size of the q for all subseqent push operations. On the other hand, every line collision
    // test is in O(N) and by saving the whole graph, O(N²) line collision tests do not need to be
    // spent. But this is all just theory and I am not entire certain how a complete reset approach
    // play out against a graph-preserving approach in real applications. In cases where saving the
    // graph actually turns out to be harmful, the graph can be reset to a blank state by calling
    // resetGraph().
    ListIterator<Node> it = nodes.begin();
    it.next(); // Skip start node.
    while (it.hasNext())
    {
        Node& child = it.next();
        child.closed = false; // Reset the closed state after the last search.
        //child.parent = 0; // Reset the parent after the last search.

        // We can skip children that have a nontangential connection with the start node.
        tangentialTests++;
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
            qDebug() << "   Pushed" << *child.parent << "->" << child << "g:" << child.g << "h:" << child.h << "f:" << child.f;
    }
}

// Reconnects the target node with the graph when the traget location changed.
void VisibilityGraph::targetNodeChanged()
{
    // When the target location changes, the target node needs to be fully reconnected to the
    // graph and the successor pointers need to be reset.

    ListIterator<Node> it = nodes.begin();
    it.next(); // Skip start node.
    while (it.hasNext())
    {
        Node& child = it.next();
        child.closed = false; // Reset the closed state after the last search.
        child.successor = 0; // Clear the successor.

        // Connect with the target node in the adjacency matrix.
        adjacencyMatrix.set(child.id, targetNode->id);
    }

    return;
}

// This algorithm is a repairing A* search that incrementally constructs the
// visibility graph while it searches for the shortest path. It begins with the direct
// line from start to target. If the direct line is collision free, the search is already
// finished. If the direct line intersects a Polygon, the corners of the Polygon are fully
// connected to the graph without computing any line intersection tests and the A* search
// is continued on this modified graph. Line intersection checks are only performed for
// edges that the A* search opens to a new waypoint. During the search, when an opened
// edge fails the intersection test, the graph needs to be repaired by removing the
// collided edge, reparenting the opened node, and adding the corners of the intersected
// Polygon to the graph. When the shortest path is finally found, the function returns
// true and the resulting path can be queried with getPath(). Repeated calls to
// minimalConstruct() will reuse the explored graph until resetGraph() is called.
// The starting point and the target of the search are provided as an argument to this
// function.
bool VisibilityGraph::minimalConstruct(const Vec2& start, const Vec2 &target, int debug)
{
    if (debug > 0)
    {
        qDebug() << "VisibilityGraph::minimalConstruct() started.";
        qDebug() << "Start:" << start << "Target:" << target;
    }

    if (*startNode == start && *targetNode == target || start == target)
    {
        //qDebug() << "  Nothing to do." << (*startNode == start) << (*targetNode == target) << (start == target);
        return true;
    }

    // Clear the priority queue and the computed path.
    q.clear();
    path.clear();

    if (target != *targetNode)
    {
        // Change the location of the target node.
        targetNode->x = target.x;
        targetNode->y = target.y;

        // Update the graph connections with the target node.
        targetNodeChanged();
    }

    if (start != *startNode || start.isNull())
    {
        // Change the location of the start node.
        startNode->x = start.x;
        startNode->y = start.y;

        // The start node needs to be connected with every known node, especially
        // with the target node. This is what the initMinimalConstruct() function does.
        startNodeChanged(debug);
    }

    // And let's go.
    while (!q.isEmpty())
    {
        // 1. Pop the best score node from the queue.
        // The first pop is typically the target node with a direct line from start to target.
        Node* bestScoreNode = q.pop();

        if (debug > 1)
            qDebug() << "Popped" << *bestScoreNode->parent << "->" << *bestScoreNode
                     << "g:" << bestScoreNode->g
                     << "h:" << bestScoreNode->h
                     << "f:" << bestScoreNode->f << (bestScoreNode->parent->id == 8);


        // 2. Line intersection test.
        // Now the line between the popped node and its parent is checked for intersection
        // with the polygons. This is the most expensive step of the algorithm. This is why
        // it's only performed when a node is popped from the queue. Additional savings are
        // drawn from caching the performed tests in the adjacency matrix.
        if (!adjacencyMatrix.isChecked(bestScoreNode->id, bestScoreNode->parent->id)) // edge already checked check :)
        {
            Line line(*(bestScoreNode->parent), *bestScoreNode);
            const Polygon& pit = gm->dilatedLineCollisionCheck(line);
            adjacencyMatrix.check(bestScoreNode->id, bestScoreNode->parent->id);
            lineIntersectionTests++;
            if (!pit.isEmpty())
            {
                if (debug > 2)
                {
                    qDebug() << "   Line from " << *bestScoreNode->parent << "to" << *bestScoreNode << "collides with Polygon" << pit.getId();
                    //qDebug() << "      end point intersect tests:" << pit.intersects(*bestScoreNode->parent) << pit.intersects(*bestScoreNode);
                }

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
                // node back into the queue prevents this error from happening.
                findParent(bestScoreNode, debug);

                // If the collided polygon is not closed yet, fully connect the corners of the polygon with the graph.
                if (!polygonClosed.contains(pit.getId()))
                {
                    if (debug > 3)
                        qDebug() << "   Adding Polygon" << pit.getId() << "to the graph.";

                    // Prepare an iterator for the new nodes only (that are yet to be added).
                    ListIterator<Node> newNodesIterator = nodes.end();

                    connectPolygonFully(pit, debug);

                    // Advance the new nodes iterator by one to point at the first new node.
                    newNodesIterator.next();

                    // Parent the new nodes, if possible. In the process of parenting, the new nodes may already be pushed.
                    while (newNodesIterator.hasNext())
                    {
                        Node& node = newNodesIterator.next();
                        findParent(&node, debug);
                    }

                    // Close the polygon so that it's added to the graph only once.
                    polygonClosed.insert(pit.getId(), true);
                }

                continue; // back to pop
            }
        }
        else if (debug > 3)
        {
            qDebug() << "   Line from " << *bestScoreNode->parent << "to" << *bestScoreNode << "is already checked.";
        }


        // 3. Check if the goal state has been reached.
        if (bestScoreNode->h < EPSILON)
        {
            if (debug > 0)
                qDebug() << "Target node reached." << *bestScoreNode;

            // Follow the parent pointers back to create the path in reverse.
            path.clear();
            Node* node = bestScoreNode;
            int counter = 0;
            while (node != 0 && counter < 100)
            {
                counter++;
                if (counter == 100)
                {
                    qDebug() << "Endless loop in MinimalConstruct detected.";
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
                qDebug() << "minimalConstruct() finished! Path size:" << path.size();

            return true;
        }


        // 4. Close the node.
        bestScoreNode->closed = true;


        // 5. Check if a state with a successor has been reached.
        // Successors are known from an earlier search to be the next node along
        // the shortest path to the goal, so nothing else needs to be expanded.
        if (bestScoreNode->successor != 0)
        {
            if (debug > 3)
                qDebug() << "Path connected at:" << *bestScoreNode;

            Node& child = *bestScoreNode->successor;

            // Closed check. Does this case even exist?
            if (child.closed)
            {
                if (debug > 4)
                    qDebug() << "   Closed successor" << child << "g:" << child.g << "f:" << child.f;
                continue; // next child
            }

            // Open check.
            // Compute the tentative cost for the child, but update the child only if the cost would improve.
            double tentative_g = bestScoreNode->g + (*bestScoreNode-child).norm(); // g + c
            if (child.getPidx() > 0 && child.g < tentative_g + EPSILON)// child is open but already has lower cost
            {
                if (debug > 4)
                    qDebug() << "   Open successor" << child << "p:" << child.parent->id << "g:" << child.g << "(" << tentative_g << ") f:" << child.f;
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
                qDebug() << "   Pushed successor" << child << "p:" << child.parent->id << "g:" << child.g << "h:" << child.h << "f:" << child.f;

            continue; // back to pop
        }


        // 6. Otherwise expand children and sort them into the priority queue.
        ListIterator<Node> it = nodes.begin();
        while (it.hasNext())
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
                    qDebug() << "   Closed" << child << "g:" << child.g << "f:" << child.f;
                continue; // next child
            }

            // Open check.
            // Compute the tentative cost for the child, but update the child only, if the cost would improve.
            double tentative_g = bestScoreNode->g + (*bestScoreNode-child).norm(); // g + c
            if (child.getPidx() > 0 && child.g < tentative_g + EPSILON)// child is open but already has lower cost
            {
                if (debug > 3)
                    qDebug() << "   Open" << child << "p:" << child.parent->id << "g:" << child.g << "(" << tentative_g << ") f:" << child.f;
                continue; // next child
            }

            // We can skip children that have a nontangential connection with the parent.
            // Delaying the tangential check to before the push has the advantage of not having
            // to compute the tangential check for the lines that never get pushed.
            tangentialTests++;
            if (!child.isTangentialTo(bestScoreNode))
            {
                if (debug > 4)
                    qDebug() << "   Not tangential" << child << "g:" << child.g << "(" << tentative_g << ") f:" << child.f;
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
                qDebug() << "   Pushed" << child.parent->id << "->" << child << "g:" << child.g << "h:" << child.h << "f:" << child.f;
        }
    }

    if (debug > 0)
        qDebug() << "   Empty queue. minimalConstruct() failed to find a solution.";

    return false;
}

// Fully connects a Polygon into the visiblity graph without performing any line intersection checks.
void VisibilityGraph::connectPolygonFully(const Polygon &pol, int debug)
{
    // This function "fully" connects a possibly nonconvex polygon with the visibility
    // graph in a way that no line intersection tests are made. Concave corners are skipped.
    // If a scene boundary is set (setBounds()), out of bounds corners are skipped as well.
    // Convex corners become Nodes in the nodes list and the AdjacencyMatrix (AM) is extended
    // to accomodate the new nodes. The edges between the convex corners of the polygon and
    // all known nodes so far are set to 1 (unknown connection state) in the adjacency matrix.
    // Tangential checks are not yet performed. They are delayed until before a push operation
    // occurs. This way, some of the tangential checks never need to be performed. Line
    // intersection tests are not performed either. They are delayed until after the pop
    // operation in the main algorithm. When the tangential tests and the line intersection
    // tests are performed, the values of the adjacency matrix are either upgraded to checked
    // (known connection state) or unset (no connection state).

    // Set up an iterator for the new nodes only.
    ListIterator<Node> newNodesIterator = nodes.end();

    uint nodeId = 2;
    if (!nodes.isEmpty())
        nodeId = nodes.last().id+1;

    // Create nodes from the vertices of the polygon.
    ListIterator<Line> edges = pol.edgeIterator();
    while (edges.hasNext())
    {
        // Compute the in and out edges of the corner.
        // These will be used for concavity and tangential testing.
        Vec2 v = edges.peekCur().p1();
        Vec2 v1 = edges.peekNext().p1() - v;
        Vec2 v2 = edges.peekPrev().p1() - v;
        uint curEdgeId = edges.peekCur().id;
        uint prevEdgeId = edges.peekPrev().id;
        edges.next();

        // Concave corners can be skipped right away.
        if (v2.isLeftOf(v1)) // This only works because we are inside of CCW and outside of CW.
        {
            if (debug > 5)
                qDebug() << "   Concave corner" << v << "skipped.";
            continue;
        }

        // Out of bounds polygon corners are also skipped.
        if (!boundingBox.isEmpty() && !boundingBox.intersects(v, -0.01))
            continue;

        // Add the Polygon corner to the nodes.
        Node node(v);
        node.id = nodeId++;
        node.polyId = pol.getId();
        node.edgeId1 = curEdgeId;
        node.edgeId2 = prevEdgeId;
        node.v1 = v1;
        node.v2 = v2;
        nodes << node;
    }

    // Advance the new nodes iterator by one to have it point at the first new node.
    newNodesIterator.next();

    // Now connect all nodes so far with the new nodes.
    ListIterator<Node> it = nodes.begin();
    while (it.hasNext() && it != newNodesIterator)
    {
        Node& outerNode = it.next();

        ListIterator<Node> it2 = newNodesIterator;
        while (it2.hasNext())
        {
            Node& innerNode = it2.next();
            adjacencyMatrix.set(innerNode.id, outerNode.id);
            if (debug > 5)
                qDebug() << "   Edge" << innerNode << "to" << outerNode << "added to graph.";
        }
    }

    // If the polygon is not convex, connect the new nodes among each other.
    if (!pol.isConvex())
    {
        ListIterator<Node> it = newNodesIterator;
        while (it.hasNext())
        {
            Node& outerNode = it.next();
            newNodesIterator.next();
            ListIterator<Node> it2 = newNodesIterator;
            while (it2.hasNext())
            {
                Node& innerNode = it2.next();
                adjacencyMatrix.set(innerNode.id, outerNode.id);
                if (debug > 5)
                    qDebug() << "   Edge" << innerNode << "to" << outerNode << "added to graph.";
            }
        }
    }

    // For convex polygons, we only need to connect the neighboring edges with each other.
    else
    {
        ListIterator<Node> it = newNodesIterator;
        while (!it.atEnd())
        {
            Node& outerNode = it.next();
            Node& innerNode = it.cur();
            adjacencyMatrix.set(innerNode.id, outerNode.id);
            if (debug > 5)
                qDebug() << "   Edge" << innerNode << "to" << outerNode << "added to graph.";

        }

        adjacencyMatrix.set(nodes.last().id, newNodesIterator.cur().id);
        if (debug > 5)
            qDebug() << "   Edge" << nodes.last() << "to" << newNodesIterator.cur() << "added to graph.";
    }

    // I experimented with marking all polygon edges as checked, and it does make things
    // faster, but then polygons must not overlap, so no.
}

// Finds a parent for a dangling node.
// This is a part of the minimalConstruct() algorithm.
// Parenting a node means finding a closed node among the neighbours of the node
// with the lowest g+c value and setting it as the parent, or assigning no parent
// if no such candidate was found.
void VisibilityGraph::findParent(Node* orphan, int debug)
{
    // The neighborhood relation is encoded as an Adjacency Matrix (AM).
    // Iterating through the right column of the AM and checking each edge if
    // it's "set" is a fast way of retrieving the neighbours. Of course, it would
    // be even faster to iterate through a "children" list of the node, but that
    // has disadvantages when removing nodes from the graph. I should consider
    // keeping a closed list though.

    if (debug > 5)
        qDebug() << "   Reparenting" << *orphan;

    Node* parent = 0;
    double minCost = std::numeric_limits<double>::max();
    ListIterator<Node> it = nodes.begin();
    while (it.hasNext())
    {
        Node& node = it.next();

        if (!node.closed) // A parent has to be closed.
            continue;

        if (!adjacencyMatrix.isSet(node.id, orphan->id)) // It has to be a neighbour of v.
            continue;

        tangentialTests++;
        if (!node.isTangentialTo(orphan)) // It has to be tangential even.
        {
            if (debug > 5)
                qDebug() << "   Not tangential" << orphan->id << "to" << node;
            adjacencyMatrix.unset(node.id, orphan->id);
            continue;
        }

        // Set the parent if it is better than the best parent so far.
        double cost = node.g + (*orphan-node).norm();
        if (cost < minCost)
        {
            minCost = cost;
            parent = &(node);
        }
    }

    if (parent == 0)
        return;

    // If a new parent was successfully found, update the orphan node and push it into the q.
    orphan->parent = parent;
    orphan->g = minCost;
    orphan->h = (*targetNode - *orphan).norm();
    orphan->f = orphan->g + orphan->h;
    q.push(orphan);
    if (debug > 4)
    {
        qDebug() << "   Parented" <<  orphan->id << "to" << orphan->parent->id;
        qDebug() << "   Pushed" << parent->id << *orphan << "p:" << orphan->parent->id << "g:" << orphan->g << "h:" << orphan->h << "f:" << orphan->f;
    }
}

// Returns the last computed path.
const Vector<Vec2> &VisibilityGraph::getPath() const
{
    return path;
}

// Draws the visibility graph on a QPainter.
void VisibilityGraph::draw(QPainter *painter) const
{
    painter->save();

    // The visibility graph edges taken from the adjacency matrix.
    painter->setPen(drawUtil.penThin);
    painter->setBrush(drawUtil.brushDarkGray);
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
    painter->setBrush(drawUtil.brush);
    painter->setOpacity(1.0);
    double s = 0.025;
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
    painter->setPen(drawUtil.pen);
    painter->setOpacity(0.8);
    it = nodes.begin();
    while (it.hasNext())
    {
        Node node = it.next();

        painter->save();
        painter->translate(node.x + 0.03, node.y + 0.04);
        painter->scale(0.1, -0.1);
        painter->drawText(QPointF(), QString::number(node.id));
        painter->restore();
    }

    painter->restore();
}

// Draws the visibility graph in an OpenGL context.
void VisibilityGraph::draw() const
{
    // The visibility graph edges taken from the adjacency matrix.
    GLlib::setColor(drawUtil.brushDarkGray.color());
    ListIterator<Node> it = nodes.begin();
    while (it.hasNext())
    {
        Node node = it.next();
        ListIterator<Node> it2 = it;
        while (it2.hasNext())
        {
            Node node2 = it2.next();
            if (adjacencyMatrix.isChecked(node.id, node2.id))
                GLlib::drawLine(node, node2, 0.002);
        }
    }

    // The visibility graph nodes.
    GLlib::setColor(drawUtil.brush.color());
    double s = 0.02;
    it = nodes.begin();
    while (it.hasNext())
    {
        Node node = it.next();
        glPushMatrix();
        glTranslated(node.x, node.y, 0);
        GLlib::drawEllipse(QPointF(), s, s);
        glPopMatrix();
    }

    // The node labels are drawn in the opengl widget.
}

// Draws the path computed by the visibility graph on a QPainter.
void VisibilityGraph::drawPath(QPainter *painter) const
{
    painter->save();
    painter->setPen(drawUtil.penBlueThick);
    painter->setBrush(drawUtil.brushBlue);
    painter->setOpacity(0.2);
    for (int i = 1; i < path.size(); i++)
        painter->drawLine(QLineF(path[i].x, path[i].y, path[i-1].x, path[i-1].y));
    painter->restore();
}

// Draws the path computed by the visibility graph on a QPainter.
void VisibilityGraph::drawPath() const
{
    glColor3f(0.0,0.0,0.8);
    for (int i = 1; i < path.size(); i++)
        GLlib::drawLine(path[i], path[i-1], 0.01);
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

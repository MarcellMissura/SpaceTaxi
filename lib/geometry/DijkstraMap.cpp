#include "DijkstraMap.h"
#include "GridModel.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"
#include "lib/util/Vec3u.h"

// The DijkstraMap is a grid structure of search nodes that hold an accurate
// heuristic value to a target and a successor pointer towards the next cell
// along the path to the target. The Dijkstra map is computed using a least
// cost first Dijkstra search that starts at the target and propagates the
// successor pointers and heuristic values of cells neighbour by neighbour.
// Obstacles block the flow and so in the end, the distances in the Dijkstra
// map implicitly reflect the shortest path from any cell to the target while
// avoiding obstales. Once computed with computeDijkstraMap(target), the
// Dijkstra Map can be efficiently queried with dijkstraHeuristic(start) or
// with dijkstraPath(start) for the heuristic value or the entire shortest
// path from a start point to the target.
//
// The DijkstraMap extends the Grid and needs to be init()-ed like so:
//
// DijkstraMap dm;
// dm.setDim(2); // Set the number of dimensions. Usually 2.
// dm.setN(Vec2u(101, 201); // Set the numer of nodes per dimension.
// dm.setMin(Vec2(config.xMin, config.yMin)); // Set the minimum values per dimension.
// dm.setMax(Vec2(config.xMax, config.yMax)); // Set the maximum values per dimension.
// dm.init(); // Initialize the grid representation.
//
// After initialization, use the computeDijkstraMap() function to compute
// the actual Dijkstra map. After this computation, you can query the Dijkstra
// map for the heuristic value of a cell or the entire path from a start cell
// to a target cell.
//
// dm.computeDijkstraMap(gridModel, target);
// double h = dm.dijkstraHeuristic(start);
// bool success = dm.dijkstraPath(start);
// if (success)
//  Vector<Vec2> path = dm.getPath();

DijkstraMap::DijkstraMap()
{
    debug = 0;
    expansions = 0;
}

// Initializes the grid search data structures.
// Provide the grid definition parameters first and then call this method.
// DijkstraMap dm;
// dm.setDim(2); // Set the number of dimensions. Usually 2.
// dm.setN(Vec2u(101, 201); // Set the numer of nodes per dimension.
// dm.setMin(Vec2(config.xMin, config.yMin)); // Set the minimum values per dimension.
// dm.setMax(Vec2(config.xMax, config.yMax)); // Set the maximum values per dimension.
// dm.init(); // Initialize the grid representation.

// Computes the Dijkstra heuristic map with LazyTheta* checks.
// For the computation, a GridModel and a target have to be provided.
// The GridModel expresses the occupancy, the target is a double valued
// point within the grid boundaries. Using the LazyTheta* checks results
// in much better successors. The smoothness of the paths approaches
// Minimal Construct quality.
void DijkstraMap::computeDijkstraMap(const GridModel& gridModel, const Vec2& target)
{
    if (debug > 0)
        qDebug() << "DijkstraMap::computeDijkstraMap() target:" << target;

    setDim(2);
    setN(gridModel.getN());
    setMin(gridModel.getMin());
    setMax(gridModel.getMax());
    init();

    // Allocate and initialize a vector of nodes.
    Vec2u N = getN();
    nodes.resize(N.x);
    for (uint i = 0; i < N.x; i++)
        nodes[i].resize(N.y);
    for (uint i = 0; i < N.x; i++)
    {
        for (uint j = 0; j < N.y; j++)
        {
            nodes[i][j].reset();
            nodes[i][j].stateIdx.x = i;
            nodes[i][j].stateIdx.y = j;
            nodes[i][j].pos = getNodeCoordinates(nodes[i][j].stateIdx);
        }
    }

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
    Vec2 stride = getStride();
    cost[0] = Vec2(stride.x, 0).norm();
    cost[1] = Vec2(-stride.x, 0).norm();
    cost[2] = Vec2(0, stride.y).norm();
    cost[3] = Vec2(0, -stride.y).norm();
    cost[4] = stride.norm();
    cost[5] = stride.norm();
    cost[6] = stride.norm();
    cost[7] = stride.norm();


    // Write the occupied cells from the grid model into the Dijkstra map.
    //Vec2u N = getN();
    for (uint i = 0; i < N.x; i++)
        for (uint j = 0; j < N.y; j++)
            if (gridModel.isOccupied(i,j))
                nodes[i][j].blocked = true;

    // Initialize the q with the target node and start pumping.
    Vec2u targetIdx = getNodeIndex(target);
    GridSearchNode* startNode = &nodes[targetIdx.x][targetIdx.y];
    q.clear();
    q.push(startNode);
    int counter = 0;
    while (!q.isEmpty())
    {
        // Pop from the q and mark it as closed.
        GridSearchNode* parent = q.pop();
        parent->closed = true;
        counter++;

        if (debug > 0)
            qDebug() << "  Popped" << *parent;

        // Iterate over all neighbours.
        for (uint i = 0; i < 8; i++)
        {
            Vec2u childIdx = parent->stateIdx + actions[i];

            // Out of bounds check.
            if (childIdx.x > N.x-1 || childIdx.y > N.y-1)
            {
                if (debug > 2)
                    qDebug() << "   Out of bounds. Idx:" << childIdx << "N:" << N;
                continue; // next child
            }

            GridSearchNode& childNode = nodes[childIdx.x][childIdx.y];

            // Skip closed neighbours.
            if (childNode.closed)
            {
                if (debug > 2)
                    qDebug() << "   Closed. Idx:" << childIdx;
                continue; // next child
            }

            // Collision check.
            if (childNode.blocked)
            {
                if (debug > 2)
                    qDebug() << "   Blocked. Idx:" << childIdx;
                continue; // next child
            }

            // Compute a tentative heuristic.
            double h = parent->h + cost[i]; // Euklidean heuristic

            // Skip neighbours that are already open with a lower score.
            if (childNode.successor != 0 && childNode.h < h+EPSILON)
            {
                if (debug > 10)
                    qDebug() << "   " << childNode.stateIdx << "is already open.";
                continue; // next child
            }

            // Decrease key (hijack) message.
            if (debug > 0 && childNode.parent != 0 && childNode.h > h)
                qDebug() << "   Hijacking" << childNode.stateIdx << "who is already open to" << childNode.parent->stateIdx;

            // Perform the line-of-sight check.
            GridSearchNode* successor = parent;
            if (parent->successor != 0 && hasLineOfSight(childNode.stateIdx, parent->successor->stateIdx))
            {
                h = parent->successor->h + (childNode.pos-parent->successor->pos).norm();
                successor = parent->successor;
            }

            // All tests passed. Update the child and push.
            childNode.h = h;
            childNode.f = h;
            childNode.successor = successor;
            q.push(&childNode);
            if (debug > 0)
                qDebug() << "    Pushed" << childNode << "parent:" << parent->stateIdx;
        }
    }
}

// Returns the precomputed heuristic value at the given start location. Lightning fast.
// The heuristic value is the length of the shortest path to the target.
double DijkstraMap::dijkstraHeuristic(const Vec2 &start) const
{
    Vec2u idx = getNodeIndex(start);
    return nodes[idx.x][idx.y].h;
}

// Computes the shortest path from start to target by following the successor
// pointer cell by cell until the target is reached. Returns true on success,
// false otherwise. The start location is given as a parameter.
// The target has been provided for the precomputation of the Dijkstra map.
// After this, you can call getPath() to get your path.
Vector<Vec2> DijkstraMap::dijkstraPath(const Vec2 &start) const
{
    if (debug > 0)
        qDebug() << "DijkstraMap::dijkstraPath(const Vec2 &start):" << start << Vec2u(getNodeIndex(start));

    path.clear();
    Vec2u idx = getNodeIndex(start);
    const GridSearchNode* cur = &nodes[idx.x][idx.y];
    if (cur->successor == 0)
    {
        if (debug > 1)
            qDebug() << "no successor" << cur << "start:" << start << idx;
        return path;
    }

    int counter = 0; // infinite loop guard
    while (cur != 0 && counter < 200)
    {
        counter++;
        path << cur->pos;
        if (debug > 0)
            qDebug() << cur->stateIdx << cur->pos;
        cur = cur->successor;
    }

    if (debug > 0)
        qDebug() << "path:" << path;
    return path;
}

// Returns true if the line between cell A and cell B does not come across
// an occupied cell. The implementation is based on the Bresenham algorithm.
// https://de.wikipedia.org/wiki/Bresenham-Algorithmus
bool DijkstraMap::hasLineOfSight(const Vec2u& cellIdxA, const Vec2u& cellIdxB) const
{
    Vec2u p;
    int t, dx, dy, incx, incy, pdx, pdy, ddx, ddy, deltaslowdirection, deltafastdirection, err;

    int xstart = cellIdxA.x;
    int xend = cellIdxB.x;
    int ystart = cellIdxA.y;
    int yend = cellIdxB.y;

    /* Entfernung in beiden Dimensionen berechnen */
    dx = xend - xstart;
    dy = yend - ystart;

    /* Vorzeichen des Inkrements bestimmen */
    incx = sgn0(dx);
    incy = sgn0(dy);
    if(dx<0) dx = -dx;
    if(dy<0) dy = -dy;

    /* feststellen, welche Entfernung größer ist */
    if (dx>dy)
    {
        /* x ist schnelle Richtung */
        pdx=incx; pdy=0;    /* pd. ist Parallelschritt */
        ddx=incx; ddy=incy; /* dd. ist Diagonalschritt */
        deltaslowdirection =dy;   deltafastdirection =dx;   /* Delta in langsamer Richtung, Delta in schneller Richtung */
    }
    else
    {
        /* y ist schnelle Richtung */
        pdx=0;    pdy=incy; /* pd. ist Parallelschritt */
        ddx=incx; ddy=incy; /* dd. ist Diagonalschritt */
        deltaslowdirection =dx;   deltafastdirection =dy;   /* Delta in langsamer Richtung, Delta in schneller Richtung */
    }

    /* Initialisierungen vor Schleifenbeginn */
    p.x = xstart;
    p.y = ystart;
    err = deltafastdirection/2;

    if (isBlocked(p))
        return false;

    /* Pixel berechnen */
    for(t=0; t<deltafastdirection; ++t) /* t zaehlt die Pixel, deltafastdirection ist Anzahl der Schritte */
    {
        /* Aktualisierung Fehlerterm */
        err -= deltaslowdirection;
        if(err<0)
        {
            /* Fehlerterm wieder positiv (>=0) machen */
            err += deltafastdirection;
            /* Schritt in langsame Richtung, Diagonalschritt */
            p.x += ddx;
            p.y += ddy;
        }
        else
        {
            /* Schritt in schnelle Richtung, Parallelschritt */
            p.x += pdx;
            p.y += pdy;
        }

        if (isBlocked(p))
            return false;
    }

    return true;
}

// Sets all border cells to "blocked".
// This is the fastest way to protect A* from trying to access the grid out of bounds.
void DijkstraMap::blockBorder()
{
    Vec2u N = getN();
    for (uint i = 0; i < N[0]; i++)
    {
        setBlocked(Vec2u(i, 0));
        setBlocked(Vec2u(i, N[1]-1));
    }
    for (uint j = 0; j < N[1]; j++)
    {
        setBlocked(Vec2u(0, j));
        setBlocked(Vec2u(N[0]-1, j));
    }
}

// Sets the cell with the index dx to blocked state.
void DijkstraMap::setBlocked(const Vec2u& idx)
{
    nodes[idx.x][idx.y].blocked = true;
}

// Returns true if the cell with the dim index n is in a blocked state.
bool DijkstraMap::isBlocked(const Vec2u& idx) const
{
    return nodes[idx.x][idx.y].blocked;
}

// Draws the standard colored-cell grid representation.
void DijkstraMap::draw() const
{
    double max = 10;
    Vec3 stride = getStride();
    Vec2u N = getN();
    glBegin(GL_QUADS);
    for (uint i = 0; i < N.x; i++)
    {
        for (uint j = 0; j < N.y; j++)
        {
            if (nodes[i][j].h > 0)
            {
                QColor c = drawUtil.getHeightMapColor(nodes[i][j].h, 0, max);
                glColor3f(c.redF(), c.greenF(), c.blueF());
            }
            else
            {
                glColor4f(0.5, 0.5, 0.5, 0.9);
            }

            double x, y, w, h;
            Vec2 p = nodes[i][j].pos;
            x = p.x-0.5*stride.x;
            y = p.y-0.5*stride.y;
            w = stride.x;
            h = stride.y;

            glVertex2d(x,y);
            glVertex2d(x+w,y);
            glVertex2d(x+w,y+h);
            glVertex2d(x,y+h);
        }
    }

    glEnd();
}

// Draws the standard colored-cell grid representation.
void DijkstraMap::draw(QPainter* painter) const
{
    if (nodes.isEmpty())
        return;

    double max = 10;
    double opacity = 1.0;

    Vec2u N = getN();
    Vec2 stride = getStride();

    //painter->save();
    painter->setOpacity(opacity);
    painter->setPen(Qt::NoPen);

    for (uint i = 0; i < N.x; i++)
    {
        for (uint j = 0; j < N.y; j++)
        {
            if (nodes[i][j].h > 0)
            {
                QColor c = drawUtil.getHeightMapColor(max-nodes[i][j].h, 0, max);
                painter->setBrush(QBrush(c));

                // The occupancy square.
                QRectF q(0, 0, stride.x, stride.y);
                q.translate(nodes[i][j].pos.x-0.5*stride.x, nodes[i][j].pos.y-0.5*stride.y);
                painter->drawRect(q);
            }
        }
    }

    glEnd();
}

// Draws the found path in an OpenGL context.
void DijkstraMap::drawPath() const
{
    // Draw the shortest path.
    if (!path.isEmpty())
    {
        glColor3d(0,0,0.3);
        for (uint i = 1; i < path.size(); i++)
            GLlib::drawLine(path[i-1], path[i], 0.04);
    }
}

// Draws the found path on a QPainter.
void DijkstraMap::drawPath(QPainter* painter) const
{
    if (!path.isEmpty())
    {
        QPainterPath p;
        p.moveTo(path[0].x, path[0].y);
        for (uint i = 1; i < path.size(); i++)
            p.lineTo(path[i].x, path[i].y);

        double s = 0.03;
        painter->strokePath(p, drawUtil.penRedThick);
        painter->setBrush(drawUtil.brushOrange);
        painter->setPen(drawUtil.penOrange);
        for (int i = 0; i < p.elementCount(); i++)
            painter->drawEllipse(p.elementAt(i), s, s);
    }
}

// Sets the debug value which controls how much debug information is printed.
void DijkstraMap::setDebug(int d)
{
    debug = d;
}

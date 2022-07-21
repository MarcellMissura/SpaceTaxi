#include "GridModel.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"
#include "util/Logger.h"
#include <iostream>
#include "util/ColorUtil.h"
#include "util/GLlib.h"
#include "util/Vec2i.h"
#include "util/Transform3D.h"
#include "PathAStar.h"

// The grid model is a representation as a reactangular grid of cells.
// Each cell contains a real value between 0 and 1 that indicates the occupancy
// (or height) of the cell. The GridModel extends Grid and needs to be init()-ed like so:
//
// GridModel gm;
// gm.setDim(2); // Set the number of dimensions. This is always 2.
// gm.setN(Vec2u(101, 201); // Set the numer of nodes per dimension.
// gm.setMin(Vec2(config.xMin, config.yMin)); // Set the minimum values per dimension.
// gm.setMax(Vec2(config.xMax, config.yMax)); // Set the maximum values per dimension.
// gm.init(); // Initialize the grid representation.
//
// In the heart of the GridModel is an opencv matrix M of type uchar (1 channel).
// Each number of the matrix (each cell in the grid) can have one of 256 values
// that are mapped to the double valued [0,1] range. The methods of the GridModel
// class revolve around this matrix such as init()-ializing it, setting and
// reading values of cells with setAt() and getAt(), and checking for occupancy
// with the isOccupied() method.

// Furthermore, we can use image filtering techniques to for example erode() and
// dilate(), which is a great way of erasing or expanding obstacles. You can also
// blur() the map, which smoothens the edges with a Gaussian blur.

// Last but not least, the GridModel offers a way to computeOccupancyGrid() from
// a set of polygons. The GridModel can also compute polygonal obstacles from the
// grid with extractPolygons() using a contour detection algorithm.

// The GridModel provides draw() methods for QPainter and OpenGL.

// Now there are also path planning methods integrated into the GridModel.
// After setting up the occupancy grid, call Vector<Vec2> path = computePath(from, to).
// Or, call initDijkstraMap(const Vec2 &to) to prepare a Dijkstra map for a fixed
// target and then call Vector<Vec2> path = computeDijkstraPath(from, to) to compute
// a path using the prepared Dijkstra map.

GridModel::GridModel()
{

}

// Copy constructor.
GridModel::GridModel(const GridModel &o) : Grid(o)
{
    *this = o;
}

// Assignment operator.
GridModel& GridModel::operator=(const GridModel &o)
{
    if (this == &o)
        return *this;

    // Set up the grid parameters.
    setDim(o.getDim());
    setN(o.getN());
    setMin(o.getMin());
    setMax(o.getMax());
    init();

    M = o.M.clone(); // The reason why we need the assignment operator.

    return *this;
}

// cv::Mat Copy constructor.
GridModel::GridModel(const cv::Mat &o)
{
    *this = o;
}

// cv::Mat Assignment operator.
GridModel& GridModel::operator=(const cv::Mat &o)
{
    // Set up the grid parameters.
    setDim(2);
    setN(Vec2u(o.cols, o.rows));
    setMin(Vec2(0, -config.gridCellSize*o.rows));
    setMax(Vec2(config.gridCellSize*o.cols, 0));
    init();

    M = o.clone();

    return *this;
}

// The init method sets up the grid structure (min, max, number of cells) and computes the
// raster of the grid coordinates. This is where the data matrix M is initialized.
void GridModel::init()
{
    Grid::init();

    // Set up the openCV matrix M (the data structure for the grid).
    Vec2u N = getN();
    M = cv::Mat::zeros(cv::Size(N.x,N.y), cv::DataType<uchar>::type);
}

// Clears the data (resets to 0), but leaves grid the structure.
void GridModel::clear()
{
    M = cv::Scalar(0);
}

// Returns the width of the grid (number of cells).
uint GridModel::getWidth() const
{
    return getN()[0];
}

// Returns the height of the grid (number of cells).
uint GridModel::getHeight() const
{
    return getN()[1];
}

// Computes an occupancy grid using the polygons passed as a parameter.
// After this method, all cells that intersect with any of the given polygons
// will have a value of 1. All other cells a value of 0.
// The polygons can be in a transformed or untransformed state.
void GridModel::computeOccupancyGrid(const Vector<Obstacle>& polygons)
{
    //qDebug() << "GridModel::computeOccupancyGrid():";
    //qDebug() << polygons;

    // The drawContours() function from the OpenCV library is used
    // to draw the polygons directly onto the matrix M. Unfortunately,
    // the polygons have to be mapped to pixel coordinates and converted
    // to a nested vector of vectors data structure first.

    std::vector<std::vector<cv::Point>> segmentsAsContour;
    std::vector<int> segmentTypes;
    for (uint i = 0; i < polygons.size(); i++)
    {
        segmentTypes.push_back(polygons[i].type);
        std::vector<cv::Point> vec;
        LinkedList<Vec2> tv = polygons[i].getTransformedVertices();
        ListIterator<Vec2> it = tv.begin();
        while (it.hasNext())
        {
            Vec2& v = it.next();
            Vec2i idx = getUnboundedNodeIndex(v);
            vec.push_back(cv::Point(idx.x, idx.y));
        }
        segmentsAsContour.push_back(vec);
    }
    cv::Scalar colorBlocked(255,255,255);
    cv::Scalar colorFree(0,0,0);
    if (!polygons.isEmpty() && polygons[0].type == Obstacle::FreeSpace)
        M = cv::Scalar(255);
    else
        M = cv::Scalar(0);
    for (uint i = 0; i < segmentsAsContour.size(); i++)
    {
        if (segmentTypes[i] == Obstacle::FreeSpace)
            cv::drawContours(M, segmentsAsContour, i, colorFree, cv::FILLED);
        else
            cv::drawContours(M, segmentsAsContour, i, colorBlocked, cv::FILLED);
    }

    return;
}

// Computes an occupancy grid from the given point cloud.
// All cells that at least one point projects into will be marked as 1.
// All other cells remain untouched.
void GridModel::computeOccupancyGrid(const Vector<Vec3>& pointCloud, double floorHeight, double ceilingHeight)
{
    for (uint i = 0; i < pointCloud.size(); i++)
    {
        const Vec3& pt = pointCloud[i];
        if (pt.isNan())
            continue;

        if (pt.z < floorHeight || pt.z > ceilingHeight)
            continue;

        if (!contains(pt))
            continue;

        setAt(getNodeIndex(pt), 1);
    }
}

// Sets all border cells to a value of 1 (occupied).
// This can be used to keep A* from accessing the grid out of bounds.
void GridModel::blockBorder()
{
    Vec2u N = getN();
    for (uint i = 0; i < N[0]; i++)
    {
        setAt(Vec2u(i, 0), 1);
        setAt(Vec2u(i, N[1]-1), 1);
    }
    for (uint j = 0; j < N[1]; j++)
    {
        setAt(Vec2u(0, j), 1);
        setAt(Vec2u(N[0]-1, j), 1);
    }
}

// Returns true if the cell that the point x is in is occupied.
// All values > 1 - EPSILON are considered to be occupied.
bool GridModel::isOccupied(const Vec2 &x) const
{
    return isOccupied(getNodeIndex(x));
}

// Returns true if the cell with the two dimensional index idx is occupied.
// All values > 0.99 are considered to be occupied.
bool GridModel::isOccupied(const Vec2u &idx) const
{
    return (getAt(idx) > 0.99);
}

// Returns true if the cell with the index i,j is occupied.
// All values > 0.99 are considered to be occupied.
bool GridModel::isOccupied(uint i, uint j) const
{
    return (getAt(i, j) > 0.99);
}

// Returns true if the cell with the flat index n is occupied.
// All values > 1 - EPSILON are considered to be occupied.
bool GridModel::isOccupied(uint n) const
{
    return isOccupied(convertIndex(n));
}

// Returns true if the line between cell A and cell B does not come across an occupied cell.
// The implementation is based on the Bresenham algorithm.
// https://de.wikipedia.org/wiki/Bresenham-Algorithmus
bool GridModel::hasLineOfSight(const Vec2u& cellIdxA, const Vec2u& cellIdxB) const
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

    if (isOccupied(p))
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

        if (isOccupied(p))
            return false;
    }

    return true;
}

// Performs a quick but unprecise check if the polygon collides with the grid.
// Every vertex of the polygon is mapped to the grid and checked if the cell
// is occupied. This is done with the centroid of the polygon as well. If one
// of the touched cells is occupied, the polygon is considered to be colliding.
// The polygon is required to be in a transformed state.
bool GridModel::polygonCollisionCheck(const Polygon &p) const
{
    ListIterator<Vec2> it = p.vertexIterator();
    while (it.hasNext())
        if (isOccupied(it.next()))
            return true;
//    if (isOccupied(p.centroid()))
//        return true;
    return false;
}

// Computes a path from "from" to "to" using the LazyTheta* algorithm.
const Vector<Vec2> &GridModel::computePath(const Vec2 &from, const Vec2 &to)
{
//    qDebug() << "GridModel::computePath(const Vec2 &from, const Vec2 &to):" << from << to;

    PathAStar pathAStar;
    pathAStar.init();
    //pathAStar.setDebug(config.debugLevel);
    pathAStar.setGridModel(this);
    pathAStar.setStartState(from);
    pathAStar.setTargetState(to);
    pathAStar.lazyThetaStarSearch();
    path = pathAStar.getPath();
    //qDebug() << "A*" << from << to << path.size() << path;

    return path;
}

// Computes a path from "from" using a Dijkstra table lookup.
// The target "to" must have been already provided for the
// precomputation of the Dijkstra map with initDijkstraMap(to).
const Vector<Vec2> &GridModel::computeDijkstraPath(const Vec2 &from, const Vec2 &to)
{
    path = dij.dijkstraPath(from);
    return path;
}

// Precomputes a Dijkstra map using the occupancy grid and the target "to".
void GridModel::initDijkstraMap(const Vec2 &to)
{
    //qDebug() << "  GridModel::initDijkstraMap(to)" << to;
    dij.computeDijkstraMap(*this, to);
}

// Returns the last computed path.
const Vector<Vec2> &GridModel::getLastPath()
{
    return path;
}

// Returns the maximum grid value taken at the corners of the polygon.
double GridModel::getAt(const Polygon &p) const
{
    double max = 0;
    ListIterator<Vec2> it = p.vertexIterator();
    while (it.hasNext())
        max = qMax(max, getAt(it.next()));
    //max = qMax(max, getAt(p.centroid()));
    return max;
}

// Returns the bounding box of the grid area.
Box GridModel::boundingBox() const
{
    return Box(getMax()[1], getMin()[0], getMin()[1], getMax()[0]);
}

// Adds GridModel o to this one by executing the opencv mat += operation.
void GridModel::operator+=(const GridModel &o)
{
    M += o.M;
}

// Applies an erode operation by radius to the occupancy grid.
// The radius is given in meters.
void GridModel::erode(double radius)
{
    Vec2 stride = getStride();
    if (!((2*radius/stride.x) > 1) || !((2*radius/stride.y) > 1))
        return;

    // Execute the OpenCV erode routine.
    cv::Mat mask = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*radius/stride.x, 2*radius/stride.y));
    cv::erode(M, M, mask);
}

// Applies a dilate operation by radius (in meters) to the occupancy grid.
// This is a great way of expanding the obstacles by the roboter size.
void GridModel::dilate(double radius)
{
    Vec2 stride = getStride();
    if (!((2*radius/stride.x) > 1) || !((2*radius/stride.y) > 1))
        return;

    // Execute the OpenCV dilate routine.
    cv::Mat mask = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*radius/stride.x, 2*radius/stride.y));
    cv::dilate(M, M, mask);
}

// Applies a blur operation by radius (in meters) to the occupancy grid.
// This is useful to smoothen an occupancy map for DWA.
void GridModel::blur(double radius)
{
    Vec2 stride = getStride();
    if (!((2*radius/stride.x) > 1) || !((2*radius/stride.y) > 1))
        return;

    // Execute the OpenCV blur filter.
    cv::blur(M, M, cv::Size(qMax(2*radius/stride.x, 1.0), qMax(2*radius/stride.y, 1.0)));
}

// Applies a Canny edge filter to the occupancy grid.
// This has never been used so far.
void GridModel::canny()
{
    // Execute the OpenCV canny edge detector.
    cv::Canny(M, M, 0, 1);
}

// Returns a vector of static obstacles (polygons) extracted from the grid.
// The algorithm segments the grid by means of contour detection.
// The segments are converted to polygons and simplified with the Douglas Peucker algorithm.
// The polygons are non-convex and disjunct. The returned polygons are transformed with
// respect to the grid, i.e. their transform is zero and the vertices are in grid coordinates.
Vector<Obstacle> GridModel::extractPolygons(Vector<VecN<4> >& hierarchyOut) const
{
    Vec2 stride = getStride();

    // Contour detection.
    std::vector<std::vector<cv::Point>> segmentsAsContour;
    std::vector<cv::Vec4i> hierachy;
    // RETR_EXTERNAL: retrieve only the most external (top-level) contours. (fastest)
    // RETR_LIST, retrieve all contours without any hierarchical information
    // RETR_TREE, retrieve all contours with hierarchical information
    // CHAIN_APPROX_SIMPLE: compresses horizontal, vertical, and diagonal segments and leaves only their end points
    cv::findContours(M, segmentsAsContour, hierachy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Douglas Peucker
    Vector<std::vector<cv::Point>> segmentsAsPolygonDP;
    segmentsAsPolygonDP.clear();
    for (uint i = 0; i < segmentsAsContour.size(); i++)
    {
        std::vector<cv::Point> segmentPoints;
        cv::approxPolyDP(segmentsAsContour[i], segmentPoints, config.gridDouglasPeuckerEpsilon, true);
        segmentsAsPolygonDP.push_back(segmentPoints);
        // Note: we can't discard anything here, otherwise the hierarchy structure will be faulty.
    }

    // Copy the hierarchy structure.
    hierarchyOut.resize(hierachy.size());
    for (uint i = 0; i < hierachy.size(); i++)
    {
        hierarchyOut[i][0] = hierachy[i][0];
        hierarchyOut[i][1] = hierachy[i][1];
        hierarchyOut[i][2] = hierachy[i][2];
        hierarchyOut[i][3] = hierachy[i][3];
    }

    // Convert the Douglas Peucker polygons to static obstacles.
    Vector<Obstacle> staticObstacles;
    staticObstacles.clear();
    for (uint i = 0; i < segmentsAsPolygonDP.size(); i++)
    {
        Obstacle pol;
        pol.setId(i);
        pol.clear();
        for (uint j = segmentsAsPolygonDP[i].size(); j > 0; j--)
            pol << Vec2(segmentsAsPolygonDP[i][j-1].x, segmentsAsPolygonDP[i][j-1].y);
        pol.scale(stride.x, stride.y);
        pol.translate(getMin());
        pol.transform();
        staticObstacles << pol;
    }

    // The obstacles sensed from the grid ARE already transformed.
    return staticObstacles;
}

// Returns a random collision-free pose in the map according to isOccupied().
// Since the position is obtained by random sampling, it can theoretically happen that a collision-free
// pose is never found. After 1000 tries, the method gives up trying and returns a random colliding position.
Vec2 GridModel::getRandomFreePosition() const
{
    bool good = false;
    int counter = 0;
    Vec2 rp;
    while (!good && counter < 1000)
    {
        rp = uniformSample();
        if (!isOccupied(rp))
            good = true;
        counter++;
        if (counter >= 1000)
            qDebug() << "Warning! GridModel::getRandomFreePosition() failed to find a free position.";
    }

    return rp;
}

// Returns the output value of the grid cell that contains point x.
double GridModel::getAt(Vec2 x) const
{
    Vec2u idx = getNodeIndex(x);
    return (double)M.at<uchar>(idx.y, idx.x)/255;
}

// Returns the output value of the grid cell specified by the index idx.
double GridModel::getAt(Vec2u idx) const
{
    return (double)M.at<uchar>(idx.y, idx.x)/255;
}

// Returns the output value of the grid cell specified by the index i,j.
double GridModel::getAt(uint i, uint j) const
{
    return (double)M.at<uchar>(j, i)/255;
}

// Returns the output value of the grid cell specified by the flat index n.
double GridModel::getAt(uint n) const
{
    Vec2u idx = convertIndex(n);
    return (double)M.at<uchar>(idx.y, idx.x)/255;
}

// Sets the grid cell that contains the point x to value v.
// v is expected to be in [0,1] otherwise the behavior is undefined.
void GridModel::setAt(Vec2 x, double v)
{
    Vec2u idx = getNodeIndex(x);
    M.at<uchar>(idx.y, idx.x) = v*255;
}

// Sets the grid cell specified by the index idx to value v.
// v is expected to be in [0,1] otherwise the behavior is undefined.
void GridModel::setAt(Vec2u idx, double v)
{
    M.at<uchar>(idx.y, idx.x) = v*255;
}

// Sets the grid cell specified by the flat index n to value v.
// v is expected to be in [0,1] otherwise the behavior is undefined.
void GridModel::setAt(uint n, double v)
{
    Vec2u idx = convertIndex(n);
    M.at<uchar>(idx.y, idx.x) = v*255;
}

// Sets the grid cell specified by the index i,j to value v.
// v is expected to be in [0,1] otherwise the behavior is undefined.
void GridModel::setAt(uint i, uint j, double v)
{
    M.at<uchar>(j, i) = v*255;
}


// Draws the occupancy grid on a QPainter.
void GridModel::draw(QPainter *painter, const QBrush& brush, double opacity, double min, double max) const
{
    colorUtil.setHeatMapColor(brush.color());

    Vec2u n = getN();
    Vec2 stride = getStride();

    painter->save();
    painter->setOpacity(opacity);
    painter->setPen(Qt::NoPen);

    // Draw the grid nodes and the occupancy squares.
    for (uint j = 0; j < n.y; j++)
    {
        for (uint i = 0; i < n.x; i++)
        {
            Vec2u idx(i,j);
            Vec2 c = getNodeCoordinates(idx);

            if (getAt(idx) <= 0)
                continue;

            // The occupancy square.
            QRectF q(c.x, c.y, stride.x, stride.y);
            q.translate(-0.5*stride.x, -0.5*stride.y);
            painter->setBrush(colorUtil.getHeatMapColor(getAt(idx), min, max));
            painter->drawRect(q);
            //qDebug() << i << j << idx << getAt(idx) << colorUtil.getHeatMapColor(getAt(idx), min, max);

            // The grid node.
            double circleRadius = 0.002;
            painter->setBrush(colorUtil.brush);
            painter->drawEllipse(c, circleRadius, circleRadius);
        }
    }

    colorUtil.resetHeatMapColor();
    painter->restore();
}

// Draws the occupancy grid in OpenGL.
void GridModel::draw(const QBrush& brush, double opacity, double min, double max) const
{
    //qDebug() << "GridModel::draw():" << c;

    Vec2u n = getN();
    Vec2 stride = getStride();

    colorUtil.setHeatMapColor(brush.color());

    glPushMatrix();
    glTranslated(-0.5*stride.x, 0.5*stride.y, 0);

    QColor color;
    for (uint j = 0; j < n.y; j++)
    {
        for (uint i = 0; i < n.x; i++)
        {
            Vec2u idx(i,j);
            if (!getAt(idx))
                continue;

            Vec2 c = getNodeCoordinates(idx);
            color = colorUtil.getHeatMapColor(getAt(idx), min, max);
            glColor4f(color.redF(), color.greenF(), color.blueF(), opacity*color.alphaF());
            //GLlib::drawQuad(c.x-0.5*stride.x, c.y+0.5*stride.y, stride.x, stride.y);
            GLlib::drawQuad(c.x, c.y, stride.x, stride.y);

            //qDebug() << "drawing" << i << j << c << "value:" << valueAt(idx) << colorUtil.getHeatMapColor(valueAt(idx), min, max) << color;
            //qDebug() << "Empty cells:" << emptyCount << "occupied:" << (n.x * n.y - emptyCount);
        }
    }

    glPopMatrix();

    colorUtil.resetHeatMapColor();
}

// Draws the border of the occupancy grid on a QPainter.
void GridModel::drawBorder(QPainter *painter) const
{
    double circleRadius = 0.02;

    painter->save();
    painter->setOpacity(1.0);
    painter->setPen(Qt::NoPen);
    painter->setBrush(colorUtil.brush);

    Vec2u n = getN();
    for (uint j = 0; j < n.y; j++)
    {
        painter->drawEllipse(Vec2(getNodeCoordinates(Vec2u(0,j))), circleRadius, circleRadius);
        painter->drawEllipse(Vec2(getNodeCoordinates(Vec2u(n.x-1,j))), circleRadius, circleRadius);
    }
    for (uint i = 0; i < n.x; i++)
    {
        painter->drawEllipse(Vec2(getNodeCoordinates(Vec2u(i,0))), circleRadius, circleRadius);
        painter->drawEllipse(Vec2(getNodeCoordinates(Vec2u(i,n.y-1))), circleRadius, circleRadius);
    }

    painter->restore();
}

void GridModel::drawBorder() const
{
    static const double circleRadius = 0.02;
    Transform3D T;

    // Black.
    glColor3f(0, 0, 0);

    Vec2u n = getN();
    for (uint j = 0; j < n.y; j++)
    {
        const Vec2& pt1 = getNodeCoordinates(Vec2u(0,j));
        T.setFromParams(pt1.x, pt1.y, 0, 0, 0, 0);
        glPushMatrix();
        glMultMatrixd(T);
        GLlib::drawFilledCircle(circleRadius);
        glPopMatrix();

        const Vec2& pt2 = getNodeCoordinates(Vec2u(n.x-1,j));
        T.setFromParams(pt2.x, pt2.y, 0, 0, 0, 0);
        glPushMatrix();
        glMultMatrixd(T);
        GLlib::drawFilledCircle(circleRadius);
        glPopMatrix();
    }
    for (uint i = 0; i < n.x; i++)
    {
        const Vec2& pt1 = getNodeCoordinates(Vec2u(i,0));
        T.setFromParams(pt1.x, pt1.y, 0, 0, 0, 0);
        glPushMatrix();
        glMultMatrixd(T);
        GLlib::drawFilledCircle(circleRadius);
        glPopMatrix();

        const Vec2& pt2 = getNodeCoordinates(Vec2u(i,n.y-1));
        T.setFromParams(pt2.x, pt2.y, 0, 0, 0, 0);
        glPushMatrix();
        glMultMatrixd(T);
        GLlib::drawFilledCircle(circleRadius);
        glPopMatrix();
    }
}

void GridModel::drawDijkstraMap(QPainter *painter) const
{
    dij.draw(painter);
}

// Prints a text output to the console.
void GridModel::printOutputs() const
{
    for (uint i = 0; i < getNodeCount(); i++)
        qDebug() << i << Vec2u(convertIndex(i)) << getAt(i);
}

QDebug operator<<(QDebug dbg, const GridModel &w)
{
    dbg  << "dim:" << w.getDim() << "N:" << Vec2u(w.getN()) << "min:" << Vec2(w.getMin()) << "max:" << Vec2(w.getMax());
    return dbg;
}

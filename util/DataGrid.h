#ifndef DATAGRID_H_
#define DATAGRID_H_
#include "Grid.h"
#include <QFile>
#include "util/ColorUtil.h"
#include "util/Vec3u.h"
#include "util/Vec3.h"
#include <QFile>
#include <GL/glu.h>

// The DataGrid extends the Grid with output values at the grid nodes. The type of the
// output is determined by a template parameter. For example: DataGrid<double> dg;

// To use the DataGrid, first set up the grid by providing the grid construction parameters
// such as the number of dimesions, the extents of the grid (min and max) in each dimension,
// and the number of nodes per dimension N. Then, call init() to initialize the data table.
// This will compute a grid that has its first node at min and last node at max and the
// remaining nodes are evenly distributed among them in every dimension. Each grid node is
// associated with an output value of the type that you provided as a template parameter.
// You can set and read this value using the available setAt() and getAt() methods. Nodes
// are addressed either with a dim index of type VecNu, or a flat index n of type uint.
// DataGrid inherits the coordinate conversion methods from Grid.

//Example:
//
// DataGrid<double> grid;
// grid.setDim(3); // Set the number of dimensions.
// grid.setN(Vec3u(101,201,301)); // Set the numer of nodes per dimension.
// grid.setMin(Vec3(config.xMin, config.yMin, config.zMin)); // Set the minimum values per dimension.
// grid.setMax(Vec3(config.xMax, config.yMax, config.zMax)); // Set the maximum values per dimension.
// grid.init(); // Compute the grid representation.
//
// Then, populate the grid by setting the node outputs.
//
// for (int n = 0; n < grid.getNodeCount(); n++)
//      grid.setAt(n, some value);
//
// You can query the grid at any continuous point within the grid boundaries.
// Vec3 v = grid.getAt(x);
//
// There save(), load(), and draw() function available.
// Please review the function documentations for more information.

template <typename T>
class DataGrid : public Grid
{
    Vector<T> Y; // Ouput values of all DataGrid nodes in a flat list (size: pow(N, DIM)-ish).

public:

    DataGrid() {}
    ~DataGrid() {}

    bool isEmpty() const {return Y.empty();}

    // Initialize the DataGrid. The grid parameters have to have been provided beforehand.
    // Calculates the raster of the grid coordinates. The grid nodes are distributed between the
    // respective min and max values of each dimension such that the first node is located at the
    // min and the last node is located at the max. Dim, N, min, and max must be set before
    // computing the raster. Make sure to set the grid parameters first and then call this method
    // to prepare the data grid before using it.
    void init()
    {
        // DataGrid overrides Grid::init() so that we can resize Y and p here.
        Grid::init();
        Y.resize(getNodeCount());
    }

    // Resets the data grid to zero output, but does not change the layout of the grid.
    virtual void clear()
    {
        //Y.fill(0);
        memset((void*)Y.data(), 0, sizeof(T)*Y.size()); // Faster with memset.
    }

    // Loads the DataGrid from a binary file that has previously been save()-d.
    void load(QString name = "DataGrid.dgr")
    {
        Grid::load(name);

        QString fileName = name.section(".", 0, 0);
        if (fileName.isEmpty())
        {
            qDebug() << "DataGrid::load(): invalid file name" << name;
            return;
        }
        fileName += ".dgr";

        QFile file(fileName);
        if (!file.open(QIODevice::ReadOnly))
        {
            qDebug() << "DataGrid::load(): Could not open file" << file.fileName();
            return;
        }

        QDataStream in(&file);
        for (uint i = 0; i < getNodeCount(); i++)
            in >> Y[i];
        file.close();
    }

    // Saves the DataGrid in a binary file that can later be load()-ed.
    void save(QString name = "DataGrid.dgr") const
    {
        Grid::save(name);

        QString fileName = name.section(".", 0, 0);
        if (fileName.isEmpty())
        {
            qDebug() << "DataGrid::save(): invalid file name" << name;
            return;
        }
        fileName += ".dgr";

        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly))
        {
            qDebug() << "DataGrid::save(): could not write to file" << file.fileName();
            return;
        }

        QDataStream out(&file);
        for (uint i = 0; i < getNodeCount(); i++)
            out << Y[i];
        file.close();
    }

    // Sets the output value of the DataGrid node identified by flat index n to y.
    void setAt(uint n, T y)
    {
        Y[n] = y;
    }

    // Sets the output value of the DataGrid node identified by DIM index idx to y.
    void setAt(const uint *idx, T y)
    {
        uint n = convertIndex(idx);
        Y[n] = y;
    }

    // Sets the output value of the grid node nearest to the point x.
    // x is truncated to lie inside the boundaries of the DataGrid.
    void setAt(const double *x, T y)
    {
        uint n = getNodeFlatIndex(x);
        Y[n] = y;
    }


    // Returns the output value of the node identified by flat index n.
    T getAt(uint n) const
    {
        return Y[n];
    }

    // Returns the output value of the node identified by DIM index idx.
    T getAt(const uint *idx) const
    {
        uint n = convertIndex(idx);
        return Y[n];
    }

    // Evaluates the DataGrid at point x using the output value of the nearest
    // grid node. x is truncated to lie inside the boundaries of the DataGrid.
    // If no data is loaded, this method returns 0.
    T getAt(const double *x) const
    {
        uint n = getNodeFlatIndex(x);
        return Y[n];
    }

    // Draws a 2D color matrix in OpenGL context where the color of each cell indicates
    // a value between min and max. The color is computed according to the blue-red-yellow
    // height map palette in ColorUtil. If the DataGrid is 3D, the sliceIdx can be used
    // to draw a 2D slice of the grid taken in the x-y plane. Higher than 3D is not supported.
    void draw(double min=0, double max=1.0, double opacity=1.0, uint sliceIdx=0) const
    {
        if (Y.empty())
            return;

        Vec3u N = getN();
        Vec3 stride = getStride();
        glBegin(GL_QUADS);
        for (uint j = 0; j < N[1]; j++)
        {
            for (uint i = 0; i < N[0]; i++)
            {
                uint n = convertIndex(Vec3u(i,j,qMin(sliceIdx, N[2]-1)));

                if (Y[n] == 0)
                    continue;

                QColor c = colorUtil.getHeightMapColor(Y[n], min, max);
                glColor4f(c.redF(), c.greenF(), c.blueF(), opacity);

                double x, y, w, h;
                Vec3 p = getNodeCoordinates(n);
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

    // QPainter drawing code that draws a 2D projection of the DataGrid using the ColorUtil height map.
    void draw(QPainter *painter) const
    {
        if (Y.empty())
            return;

        painter->save();
        painter->setPen(Qt::NoPen);
        Vec3u N = getN();
        Vec3 stride = getStride();
        for (uint i = 0; i < N[0]; i++)
        {
            for (uint j = 0; j < N[1]; j++)
            {
                if (getDim() > 2)
                {
                    uint counter = 0;
                    for (uint k = 0; k < N[2]; k++) // Huh?
                        if (Y[convertIndex(Vec3u(i,j,k))])
                            counter++;

                    if (counter > 0)
                    {
                        painter->setBrush(colorUtil.getHeatMapColor(counter, 0, N[2]));
                        painter->drawRect(QRectF(this->raster[0][i]-0.5*this->stride[0], this->raster[1][j]-0.5*this->stride[1], this->stride[0], this->stride[1]));
                    }
                }
                else
                {
                    painter->drawRect(QRectF(this->raster[0][i]-0.5*stride[0], this->raster[1][j]-0.5*stride[1], stride[0], stride[1]));
                }
            }
        }

        painter->restore();
    }


    // Draws the cumulative view of the data grid.
    void drawCumulated() const
    {
        if (isEmpty())
            return;

        glPushMatrix();
        glBegin(GL_QUADS);

        colorUtil.setHeatMapColor(QColor("AA00AA"));

        Vec3u N = getN();
        Vec3 stride = getStride();
        for (uint i = 0; i < N[0]; i++)
        {
            for (uint j = 0; j < N[1]; j++)
            {
                uint counter = 0;
                for (uint k = 0; k < N[2]; k++)
                    if (getAt(Vec3u(i,j,k)) > 0)
                        counter++;

                if (counter > 0)
                {
                    QColor c = colorUtil.getHeatMapColor(counter, 0, N[2]/10);
                    glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF());

                    double x, y, w, h;
                    //uint n = convertIndex(Vec3u(i,j,0));
                    Vec3 p = getNodeCoordinates(Vec3u(i,j,0));
                    x = p.x-0.5*stride.x;
                    y = p.y-0.5*stride.y;
                    w = stride.x;
                    h = stride.y;

                    glVertex2f(x,y);
                    glVertex2f(x+w,y);
                    glVertex2f(x+w,y+h);
                    glVertex2f(x,y+h);
                }
            }
        }

        colorUtil.setHeatMapColor(QColor("#11FF0000"));

        glEnd();
        glPopMatrix();
    }

    // Draws the cumulative view of the data grid.
    void drawCumulated(QPainter* painter) const
    {
        if (isEmpty())
            return;

        colorUtil.setHeatMapColor(QColor("AA00AA"));

        painter->save();
        painter->setPen(Qt::NoPen);
        Vec3u N = getN();
        Vec3 stride = getStride();
        for (uint i = 0; i < N[0]; i++)
        {
            for (uint j = 0; j < N[1]; j++)
            {
                uint counter = 0;
                for (uint k = 0; k < N[2]; k++)
                    if (getAt(Vec3u(i,j,k)) > 0)
                        counter++;

                Vec3 p = getNodeCoordinates(Vec3u(i,j,0));

                if (counter > 0)
                {
                    //qDebug() << i << j << p << counter << colorUtil.getHeatMapColor(counter, 0, N[2]/2);
                    //painter->setBrush(colorUtil.getHeatMapColor(counter, 0, N[2]/2));
                    painter->setBrush(colorUtil.brushOrange);
                    painter->drawRect(QRectF(p.x-0.5*stride.x, p.y-0.5*stride.y, stride.x, stride.y));
                }
            }
        }

        painter->restore();

        colorUtil.setHeatMapColor(QColor("#11FF0000"));
    }

    // Prints a text output of the nonzero nodes into the console.
    // Don't call this if your grid is huge.
    void printOutputs() const
    {
        for (uint i = 0; i < getNodeCount(); i++)
            qDebug() << Vec3u(convertIndex(i)) << getAt(i);
    }
};

#endif

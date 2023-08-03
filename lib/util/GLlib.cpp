#include "GLlib.h"
#include <math.h>
#include <QColor>

namespace GLlib
{
    // 1: top left front
    // 2: bottom left front
    // 3: top right front
    // 4: bottom right front
    // 5: top right back
    // 6: bottom right back
    // 7: top left back
    // 8: bottom left back
    void drawBox(
            double x1, double y1, double z1,
            double x2, double y2, double z2,
            double x3, double y3, double z3,
            double x4, double y4, double z4,
            double x5, double y5, double z5,
            double x6, double y6, double z6,
            double x7, double y7, double z7,
            double x8, double y8, double z8
            )
    {
        glBegin(GL_QUAD_STRIP);
        glVertex3d(x1, y1, z1);
        glVertex3d(x2, y2, z2);
        glVertex3d(x3, y3, z3);
        glVertex3d(x4, y4, z4);
        glVertex3d(x5, y5, z5);
        glVertex3d(x6, y6, z6);
        glVertex3d(x7, y7, z7);
        glVertex3d(x8, y8, z8);
        glVertex3d(x1, y1, z1);
        glVertex3d(x2, y2, z2);
        glEnd();

        glBegin(GL_QUADS);
        glVertex3d(x1, y1, z1);
        glVertex3d(x3, y3, z3);
        glVertex3d(x5, y5, z5);
        glVertex3d(x7, y7, z7);
        glVertex3d(x2, y2, z2);
        glVertex3d(x4, y4, z4);
        glVertex3d(x6, y6, z6);
        glVertex3d(x8, y8, z8);
        glEnd();

        glColor3f(0.3, 0.3, 0.3);
        glLineWidth(2);
        glBegin( GL_LINE_STRIP );
        glVertex3d(x1, y1, z1);
        glVertex3d(x2, y2, z2);
        glVertex3d(x4, y4, z4);
        glVertex3d(x3, y3, z3);
        glVertex3d(x1, y1, z1);
        glVertex3d(x7, y7, z7);
        glVertex3d(x8, y8, z8);
        glVertex3d(x6, y6, z6);
        glVertex3d(x5, y5, z5);
        glVertex3d(x7, y7, z7);
        glEnd();

        glBegin(GL_LINES);
        glVertex3d(x3, y3, z3);
        glVertex3d(x5, y5, z5);
        glVertex3d(x4, y4, z4);
        glVertex3d(x6, y6, z6);
        glVertex3d(x2, y2, z2);
        glVertex3d(x8, y8, z8);
        glEnd();
    }

    // Draws a cuboid with half extents x, y and z.
    void drawBox(double x, double y, double z)
    {
        glBegin( GL_QUAD_STRIP );
        glVertex3d(x, y, z);
        glVertex3d(x, y, -z);
        glVertex3d(x, -y, z);
        glVertex3d(x, -y, -z);
        glVertex3d(-x, -y, z);
        glVertex3d(-x, -y, -z);
        glVertex3d(-x, y, z);
        glVertex3d(-x, y, -z);
        glVertex3d(x, y, z);
        glVertex3d(x, y, -z);
        glEnd();

        glBegin( GL_QUADS );
        glVertex3d(x, y, z);
        glVertex3d(x, -y, z);
        glVertex3d(-x, -y, z);
        glVertex3d(-x, y, z);
        glVertex3d(x, y, -z);
        glVertex3d(x, -y, -z);
        glVertex3d(-x, -y, -z);
        glVertex3d(-x, y, -z);
        glEnd();

        glColor3f(0, 0, 0);
        glLineWidth(2);
        glBegin( GL_LINE_STRIP );
        glVertex3d(x, y, z);
        glVertex3d(x, y, -z);
        glVertex3d(x, -y, -z);
        glVertex3d(x, -y, z);
        glVertex3d(x, y, z);
        glVertex3d(-x, y, z);
        glVertex3d(-x, y, -z);
        glVertex3d(-x, -y, -z);
        glVertex3d(-x, -y, z);
        glVertex3d(-x, y, z);
        glEnd();

        glBegin( GL_LINES );
        glVertex3d(x, -y, z);
        glVertex3d(-x, -y, z);
        glVertex3d(x, -y, -z);
        glVertex3d(-x, -y, -z);
        glVertex3d(x, y, -z);
        glVertex3d(-x, y, -z);
        glEnd();
    }


    // Draws a cuboid with half extents dx, dy and dz and set color.
    void drawWireFrame(double dx, double dy, double dz)
    {
        glBegin( GL_LINE_STRIP );
        glVertex3d(dx, dy, dz);
        glVertex3d(dx, dy, -dz);
        glVertex3d(dx, -dy, -dz);
        glVertex3d(dx, -dy, dz);
        glVertex3d(dx, dy, dz);
        glVertex3d(-dx, dy, dz);
        glVertex3d(-dx, dy, -dz);
        glVertex3d(-dx, -dy, -dz);
        glVertex3d(-dx, -dy, dz);
        glVertex3d(-dx, dy, dz);
        glEnd();

        glBegin( GL_LINES );
        glVertex3d(dx, -dy, dz);
        glVertex3d(-dx, -dy, dz);
        glVertex3d(dx, -dy, -dz);
        glVertex3d(-dx, -dy, -dz);
        glVertex3d(dx, dy, -dz);
        glVertex3d(-dx, dy, -dz);
        glEnd();
    }

    // Draws a quad with x,y top left corner and width and height.
    void drawQuad(double x, double y, double dx, double dy)
    {
        glBegin( GL_QUADS );
        glVertex3d(x, y, 0.001);
        glVertex3d(x+dx, y, 0.001);
        glVertex3d(x+dx, y-dy, 0.001);
        glVertex3d(x, y-dy, 0.001);
        glEnd();
    }

    // Draws a bordered quad with x,y top left corner and width and height.
    void drawBorderedQuad(double x, double y, double dx, double dy)
    {
        glBegin( GL_QUADS );
        glVertex3d(x, y, 0.001);
        glVertex3d(x+dx, y, 0.001);
        glVertex3d(x+dx, y-dy, 0.001);
        glVertex3d(x, y-dy, 0.001);
        glEnd();

        glColor3f(0.1, 0.1, 0.1);
        glBegin( GL_LINE_STRIP );
        glVertex3d(x, y, 0.001);
        glVertex3d(x+dx, y, 0.001);
        glVertex3d(x+dx, y-dy, 0.001);
        glVertex3d(x, y-dy, 0.001);
        glVertex3d(x, y, 0.001);
        glEnd();
    }

    // Draws a sphere.
    void drawSphere(double radius)
    {
        static GLUquadric* quadric = gluNewQuadric();
        gluSphere(quadric, radius, 32, 32);
    }

    // Draws a sphere with a camera plane aligned border around it (you wish).
    void drawBorderedSphere(double radius)
    {
        static GLUquadric* quadric = gluNewQuadric();
        gluSphere(quadric, radius, 32, 32);
    }

    // Draws a circle in the xy plane.
    // http://slabode.exofire.net/circle_draw.shtml
    void drawCircle(double radius, double thickness)
    {
        int slices = 32;

        double theta = 2 * 3.1415926 / double(slices);
        double c = fcos(theta);
        double s = fsin(theta);
        double t;

        double x = radius; //we start at angle = 0
        double y = 0;
        double x_ = radius-thickness;
        double y_ = 0;

        glBegin(GL_TRIANGLE_STRIP);
        for(int ii = 0; ii <= slices; ii++)
        {
            glVertex2d(x, y);
            glVertex2d(x_, y_);

            //apply the rotation matrix
            t = x;
            x = c * x - s * y;
            y = s * t + c * y;

            t = x_;
            x_ = c * x_ - s * y_;
            y_ = s * t + c * y_;
        }
        glEnd();
    }

    // Draws the outline of a circle at Pose p.
    void drawCircle(const Pose2D &p, const QColor &color, double radius, double thickness)
    {
        glPushMatrix();
        glTranslated(p.x, p.y, 0.0001);
        glRotated(p.z*RAD_TO_DEG, 0, 0, 1);
        setColor(color);
        drawCircle(radius, thickness);
        glPopMatrix();
    }

    // Draws a filled circle in the xy plane with a TRIANGLE_FAN.
    // http://slabode.exofire.net/circle_draw.shtml
    void drawFilledCircle(double radius)
    {
        int slices = 32;

        double theta = 2 * 3.1415926 / double(slices);
        double c = fcos(theta);
        double s = fsin(theta);
        double t;

        double x = radius; // start at angle = 0
        double y = 0;

        glBegin(GL_TRIANGLE_FAN);
        glVertex2d(0, 0);
        for(int ii = 0; ii <= slices; ii++)
        {
            glVertex2d(x, y);
            t = x;
            x = c*x-s*y; // rotation matrix
            y = s*t+c*y; // rotation matrix
        }
        glEnd();
    }

    void drawFilledCircle(const Pose2D &p, const QColor &color, double radius)
    {
        glPushMatrix();
        glTranslated(p.x, p.y, 0.0001);
        glRotated(p.z*RAD_TO_DEG, 0, 0, 1);
        setColor(color);
        drawFilledCircle(radius);
        glPopMatrix();
    }

    // Draws a circle stack.
    void drawFilledCircleStack(double radius)
    {
        drawFilledCircle(radius);

        glPushMatrix();
        glRotated(90, 1, 0, 0);
        drawFilledCircle(radius);
        glPopMatrix();

        glPushMatrix();
        glRotated(90, 0, 1, 0);
        drawFilledCircle(radius);
        glPopMatrix();
    }

    // Draws a nose circle at Pose p.
    void drawNoseCircle(const Pose2D &p, const QColor &color, double radius)
    {
        double lineThickness = 0.15*radius;
        glPushMatrix();
        glTranslated(p.x, p.y, 0.0001);
        glRotated(p.z*RAD_TO_DEG, 0, 0, 1);
        glColor3d(0, 0, 0); // black
        GLlib::drawFilledCircle(radius);
        glTranslated(0, 0, 0.0001);
        setColor(color);
        GLlib::drawFilledCircle(radius-lineThickness);
        glTranslated(0, 0, 0.0001);
        glColor3d(0, 0, 0); // black
        GLlib::drawLine(Vec2(), Vec2(radius*1.5, 0), 0.5*lineThickness);
        glPopMatrix();
    }

    // Draws a coordinate frame.
    void drawFrame(double size, int lw)
    {
        glLineWidth(lw);
        glBegin( GL_LINES );
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(size, 0.0, 0.0);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, size, 0.0);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, 0.0, size);
        glEnd();
    }

    // Draws a coordinate frame.
    void drawFrame(double sx, double sy, double sz)
    {
        glLineWidth(2);
        glBegin( GL_LINES );
        glColor3f(1.0, 0.0, 0.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(sx, 0.0, 0.0);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, sy, 0.0);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, 0.0, sz);
        glEnd();
    }

    // Draws a 3D arrow that points along the x axis.
    void drawArrow(double length, double radius)
    {
        static GLUquadric* quadric = gluNewQuadric();

        if (radius < 0.0)
            radius = 0.05 * length;

        double headLength = 0.4*length < 0.15 ? 0.4*length : 0.15;
        double arrowLength = length - headLength;

        glPushMatrix();
        glRotated(90, 0, 1.0, 0);
        gluCylinder(quadric, radius, radius, arrowLength, 32, 1);
        glTranslatef(0.0, 0.0, arrowLength);
        gluCylinder(quadric, 1.5*radius, 0.0, headLength, 32, 1);
        glPopMatrix();
    }

    // Draws a cross.
    void drawCross(double size)
    {
        double thickness = 0.35;
        double t = thickness*size;

        glPushMatrix();

        glBegin( GL_QUADS );
        glVertex2d(size, t);
        glVertex2d(-size, t);
        glVertex2d(-size, -t);
        glVertex2d(size, -t);
        glEnd();
        glTranslatef(0, 0, 0.001);
        glBegin( GL_QUADS );
        glVertex2d(t, size);
        glVertex2d(-t, size);
        glVertex2d(-t, -size);
        glVertex2d(t, -size);
        glEnd();

        glTranslatef(0, 0, 0.001);
        glLineWidth(3);
        glColor3d(0, 0, 0);
        glBegin(GL_LINE_LOOP);
        glVertex2d(size, t);
        glVertex2d(size, -t);
        glVertex2d(t, -t);
        glVertex2d(t, -size);
        glVertex2d(-t, -size);
        glVertex2d(-t, -t);
        glVertex2d(-size, -t);
        glVertex2d(-size, t);
        glVertex2d(-t, t);
        glVertex2d(-t, size);
        glVertex2d(t, size);
        glVertex2d(t, t);
        glEnd();

        glPopMatrix();
    }

    void drawCone(double bottomRadius, double topRadius, double height)
    {
        static GLUquadric* quadric = gluNewQuadric();
        gluCylinder(quadric, bottomRadius, topRadius, height, 32, 32);
        drawFilledCircle(bottomRadius);
        glTranslated(0, 0, height);
        drawFilledCircle(topRadius);
        drawCircle(topRadius);
        glTranslated(0, 0, -height);
        drawCircle(bottomRadius);
    }

    void drawCylinder(double radius, double height)
    {
        static GLUquadric* quadric = gluNewQuadric();
        gluCylinder(quadric, radius, radius, height, 32, 1);
        drawFilledCircle(radius);
        glTranslated(0, 0, height);
        drawFilledCircle(radius);
        glColor3f(0, 0, 0);
        drawCircle(radius);
        glTranslated(0, 0, -height);
        drawCircle(radius);
    }

    // Draws a line with variable thickness. You set the thickness and the color.
    void drawLine(const Vec2 &from, const Vec2 &to, double thickness)
    {
        Vec2 d = to-from;
        d.flip();
        d.normalize(thickness);
        //glColor3b(0,0,0);
        glBegin(GL_POLYGON);
        glVertex2dv(from+d);
        glVertex2dv(from-d);
        glVertex2dv(to-d);
        glVertex2dv(to+d);
        glEnd();
    }

    // From: https://stackoverflow.com/a/34735255
    void drawEllipse(const Vec2 &p, double rx, double ry, uint16_t segments)
    {
        float theta = PII / float(segments);
        float c = fcos(theta);//precalculate the sine and cosine
        float s = fsin(theta);
        float t;

        float x = 1;//we start at angle = 0
        float y = 0;

        glBegin(GL_LINE_LOOP);
        for(int ii = 0; ii < segments; ii++)
        {
            //apply radius and offset
            glVertex2f(x * rx + p.x, y * ry + p.y);//output vertex

            //apply the rotation matrix
            t = x;
            x = c * x - s * y;
            y = s * t + c * y;
        }
        glEnd();
    }

    // Sets the openGL color to the given QColor.
    void setColor(const QColor &c)
    {
        glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF());
    }

    // Draws an arrow onto the x-y plane with the tip located at p.pos()
    // and oriented as p.heading().
    void drawLineArrow(const Pose2D &p, const QColor &color, double size)
    {
        //qDebug() << "drawing line arrow at" << p << "col:" << color << "size:" << size;
        glPushMatrix();
        glMultMatrixd(p.getMatrix());
        glTranslated(0, 0, 0.00001);
        //glScaled(size, size, 1.0);
        setColor(color);

        glBegin(GL_LINES);
        glVertex2d(-size, 0);
        glVertex2d(0, 0);
        glEnd();

        glRotated(-20, 0, 0, 1);

        glBegin(GL_LINES);
        glVertex2d(qMax(-0.2, -size*0.4), 0);
        glVertex2d(0, 0);
        glEnd();

        glRotated(40, 0, 0, 1);

        glBegin(GL_LINES);
        glVertex2d(qMax(-0.3, -size*0.4), 0);
        glVertex2d(0, 0);
        glEnd();

        glPopMatrix();
    }
}

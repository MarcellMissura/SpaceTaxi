#ifndef GLLIB
#define GLLIB
#include "Vec3.h"
#include "Vec2.h"
#include "Pose2D.h"
#include <GL/glu.h>

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
		);

    // Draws a cuboid with half extents x, y, and z.
    void drawBox(double x=0.5, double y=0.5, double z=0.5);

    // Draws a wire frame of half extents dx, dy, and dz and set color
    void drawWireFrame(double dx, double dy, double dz);

    // Draws a quad with x,y top left corner and width and height.
    void drawQuad(double x, double y, double dx, double dy);
    void drawBorderedQuad(double x, double y, double dx, double dy);

    // Draws a cone.
    void drawCone(double bottomRadius, double topRadius, double height);

    // Draws a cylinder.
    void drawCylinder(double radius, double height);

    // Draws a sphere.
    void drawSphere(double radius = 1.0);

    // Draws a sphere with a camera plane aligned border around it.
    void drawBorderedSphere(double radius = 1.0);

    // Draws a circle in the xy plane.
    void drawCircle(double radius = 0.1, double thickness=0.01);
    void drawCircle(const Pose2D &p, const QColor &color, double radius=1.0, double thickness=0.01);
    void drawFilledCircle(double radius = 1.0);
    void drawFilledCircle(const Pose2D &p, const QColor &color, double radius=1.0);
    void drawFilledCircleStack(double radius = 0.1);
    void drawNoseCircle(const Pose2D& p, const QColor& color, double radius=0.1);
    void drawLineArrow(const Pose2D& p, const QColor& color, double size=0.1);

    // Draws a coordinate frame.
    void drawFrame(double size = 1.0, int lw = 2);
    void drawFrame(double sx, double sy, double sz);

    // Draws an arrow.
    void drawArrow(double length, double radius);

    // Draws a cross.
    void drawCross(double size = 0.1);

    void drawLine(const Vec2& from, const Vec2& to, double thickness = 0.01);

    // Draw an ellipse.
    void drawEllipse(const Vec2& p, double rx, double ry, uint16_t segments=10);

    void setColor(const QColor& c);
}

#endif //GLLIB

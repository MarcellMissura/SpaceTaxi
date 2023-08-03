#include "Curve.h"
#include <math.h>
#include "blackboard/State.h"
#include "blackboard/Config.h"
#include "blackboard/Command.h"


Curve::Curve()
{
	stateMemberId = 0;
	color = QColor("red");
	highlight = false;
}


// Returns the raw curve value at time mark t. The time t is given in seconds relative to the
// first data point which has the time mark 0.
// The horizontal translation has to be taken into account.
double Curve::valueAt(double t)
{
	int nearestIndex = findIndex(t - transform.dx());
	return state[nearestIndex](stateMemberId);
}

// Returns the transformed curve value at time mark t. The time t is given in seconds relative
// to the first data point which has the time mark 0.
// The value is determined by interpolating between the two nearest neighbors in the data set.
// In the transformed version, the vertical offset and the scaling factor are also taken into
// account.
double Curve::transformedValueAt(double t)
{
	double y = valueAt(t);
	return y*transform.m22() + transform.dy();
}

// Returns the raw curve value at time mark t. The time t is given in seconds relative to the
// first data point which has the time mark 0.
// The value is determined by interpolating between the two nearest neighbors in the data set.
// The horizontal translation has to be taken into account.
double Curve::interpolatedValueAt(double t)
{
	int nearestIndex = findIndex(t - transform.dx());

	double x1 = state[nearestIndex].time;
	double x2 = state[nearestIndex-1].time;
	double y1 = state[nearestIndex](stateMemberId);
	double y2 = state[nearestIndex-1](stateMemberId);
	double x = qBound(x1, t, x2);
	double y = y1 + (y2-y1)*(x-x1)/(x2-x1);

	// Filter NaN cases.
	if (x1 == x2)
		y = y1;

	return y;
}

// Returns the transformed curve value at time mark t. The time t is given in seconds relative
// to the first data point which has the time mark 0.
// The value is determined by interpolating between the two nearest neighbors in the data set.
// In the transformed version, the vertical offset and the scaling factor are also taken into
// account.
double Curve::interpolatedTransformedValueAt(double t)
{
	double y = interpolatedValueAt(t);
	return y*transform.m22() + transform.dy();
}

// Returns the floor state index for the given time t.
int Curve::findIndex(double t)
{
	return state.findIndex(t);
}

// Applies a translation to the entire curve.
void Curve::translate(double x, double y)
{
	transform.translate(x, y/transform.m22());
}

// Applies a scale operation to the entire curve.
void Curve::scale(double x, double y)
{
	transform.scale(x, y);
}

// "Stretches" the curve such that the transformed value at x grows or shrinks by y.
// I know, this is a weird function, but it's needed to implement scaling on mouse drag.
void Curve::stretchBy(double x, double y)
{
	if (qAbs(interpolatedTransformedValueAt(x) - transform.dy()) > EPSILON)
		transform.scale( 1.0, (interpolatedTransformedValueAt(x) - transform.dy() + y) / (interpolatedTransformedValueAt(x) - transform.dy()) );
}

double Curve::dx()
{
	return transform.dx();
}

double Curve::dy()
{
	return transform.dy();
}

double Curve::scalex()
{
	return transform.m11();
}

double Curve::scaley()
{
	return transform.m22();
}

// Resets the scaling factor and the vertical offset of the curve, but not the horizontal offset.
void Curve::reset()
{
	double dx = transform.dx();
	transform.reset();
	transform.translate(dx, 0);
}

// Sets a marker at time t.
void Curve::setMarkerAt(double t)
{
	if (!markers.contains(t))
		markers << t;
}

// Draws the curve with the QPainter.
void Curve::draw(QPainter* painter)
{
	painter->save();

	// Set pen and brush.
	QPen pen(color);
    	pen.setWidth(1 + highlight);
	pen.setCosmetic(true);
	painter->setPen(pen);
	painter->setBrush(QColor(0, 0, 0, 0));

	// Combine the coordinate system transformation with the individual transform of this curve.
	painter->save();
	painter->setTransform(transform, true);
	QTransform currentTransform = painter->transform();

	// Determine the start and end index of the visible data contained in the bounding box.
	// The bounding box needs to be mapped with the individual transform of this curve in case
	// a horizontal offset was defined for this curve.
	QPointF topLeft = currentTransform.inverted().map(QPointF(0,0));
	QPointF bottomRight = currentTransform.inverted().map(QPointF(painter->window().bottomRight()));
	QRectF boundingBox = QRectF(topLeft, bottomRight);
	int startIndex = findIndex(boundingBox.left());
	int endIndex = findIndex(boundingBox.right());

	// When the view is zoomed out, multiple states map to the same pixel and there is no point drawing all of them.
	// To avoid excessive plotting, determine the stride to step through the state history.
	int stride = 1;
    stride = qMax(1, qFloor(1.0/(painter->transform().m11()*(1.0/command.frequency))));

    // Collect the indices of the state history that need to be plotted.
	indices.clear();
    for (int i = startIndex; i < endIndex; i=i+stride)
        indices << i;
    indices << endIndex;

	// Draw the actual curve.
	double x1,y1,x2,y2;
	x2 = state[indices[0]].time;
	y2 = state[indices[0]](stateMemberId);
	for (int i = 1; i < indices.size(); i++)
	{
		x1 = x2;
		y1 = y2;
		x2 = state[indices[i]].time;
		y2 = state[indices[i]](stateMemberId);
        	if (x1 == x1 && x2 == x2 && y1 == y1 && y2 == y2) // check for nan
			painter->drawLine(QPointF(x1, y1), QPointF(x2, y2));
	}

	// The curve transform is disabled again for drawing the markers so that the circles are not transformed to ellipses.
	painter->restore();

	// Draw the markers on the curve.
	double screenScaleX = 1.0/painter->transform().m11();
	double screenScaleY = 1.0/painter->transform().m22();
	pen.setWidth(2);
	painter->setPen(pen);
	foreach (double marker, markers)
		painter->drawEllipse(QPointF(marker, interpolatedTransformedValueAt(marker)), 4.0*screenScaleX, 4.0*screenScaleY);

	// Draw the marker values. This requires a standard screen coordinate system so that the font is not upside down.
	painter->resetTransform();
	foreach (double marker, markers)
		painter->drawText(currentTransform.map(QPointF(marker, interpolatedValueAt(marker))) + QPointF(10,-4), QString::number(interpolatedValueAt(marker), 'f', 3) );

	painter->restore();
}



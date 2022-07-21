#include "GraphWidget.h"
#include <math.h>
#include "blackboard/State.h"
#include "util/ColorUtil.h"

// The GraphWidget shows plotted data of the members that have been
// registered in state.

GraphWidget::GraphWidget(QWidget *parent)
    : QWidget(parent)
{
	setMouseTracking(true);

	recording = false;
	showAxes = true;
	screenScale.rx() = 0.01; // 1 px is worth 0.01 value
	screenScale.ry() = 0.01; // 1 px is worth 0.01 value
	screenOffset = QPointF(0,0);
	mouseCatchRadius = 6.0;
	mouseClickTimeStamp = 0;
	dragMoved = false;
	mousePresent = false;
	draggedCurveId = -1;
	nearestCurveId = -1;
	lastSwipeFadeOutTimeStamp = 0;
	lastMouseUpdateTimeStamp = 0;
	swipeFadeOutTimer.setInterval(20);
	connect(&swipeFadeOutTimer, SIGNAL(timeout()), this, SLOT(swipeFadeOut()));
}

GraphWidget::~GraphWidget()
{
	// Save the curve translation and scale factors.
	QSettings settings;
	settings.beginGroup("curvesettings");
	settings.remove(""); // clear group
	for (int i = 0; i < curves.size(); i++)
		settings.setValue(state.memberNames[curves[i].stateMemberId], curves[i].transform);
	settings.endGroup();

	//settings.setValue("screenScale", screenScale);
}

// Generates curve objects for each registered state member and restores on/off status.
// state.init() must have been called first.
void GraphWidget::init()
{
	QSettings settings;

	//screenScale = settings.value("screenScale", QPointF(0.01, 0.01)).toPointF();
	//screenScale = QPointF(0.01, 0.01);

	// Generate curve objects with random colors based on the state object meta info.
	int idx = 0;
	foreach (QString key, state.memberNames)
	{
		Curve curve;
		curve.stateMemberId = idx++;

		// Assign randomly distributed colors.
		// Bias blue for "left" data and red for "right" data.
		if (key.contains("eft"))
			curve.color = colorUtil.sampleBlueColor();
		else if (key.contains("ight"))
			curve.color = colorUtil.sampleRedColor();
		else
			curve.color = colorUtil.sampleUniformColor();

		// Restore scaling and translation from the QSettings.
		if (settings.contains("curvesettings/" + key))
			curve.transform = settings.value("curvesettings/" + key).value<QTransform>();

		curves << curve;

		// Restore on/off state from QSettings. These are saved in the checkbox widget.
		if (settings.contains("checkboxstates/" + key) && settings.value("checkboxstates/" + key).toBool())
			idsOfTheCurvesToShow << curve.stateMemberId;
	}

}

// Resamples all colors (in case you didn't like the old ones).
void GraphWidget::resampleColors()
{
	for (int i = 0; i < curves.size(); i++)
        curves[i].color = ColorUtil::sampleUniformColor();
	update();
}

// Returns the state index for the left and right border of the graph widget.
Vec2i GraphWidget::getLeftRightStateIndex()
{
	// Determine the start and end index of the visible data contained in the bounding box.
	// The bounding box needs to be mapped with the individual transform of this curve in case
	// a horizontal offset was defined for this curve.
	QTransform tf = screenTransform;
    tf.translate(-state.time, 0);
	QPointF topLeft = tf.inverted().map(QPointF(0,0));
	QPointF bottomRight = tf.inverted().map(QPointF(width(), height()));
	QRectF boundingBox = QRectF(topLeft, bottomRight);
    int startIndex = state.findIndex(boundingBox.left());
    int endIndex = state.findIndex(boundingBox.right());

    return Vec2i(startIndex, endIndex);
}

// Writes a gnuplot ready data export to data/export.txt from the data that is currently displayed in the graph widget.
void GraphWidget::exportData()
{
	// Determine the start and end index of the visible data contained in the bounding box.
	// The bounding box needs to be mapped with the individual transform of this curve in case
	// a horizontal offset was defined for this curve.
	QTransform tf = screenTransform;
    tf.translate(-state.time, 0);
	QPointF topLeft = tf.inverted().map(QPointF(0,0));
	QPointF bottomRight = tf.inverted().map(QPointF(width(), height()));
	QRectF boundingBox = QRectF(topLeft, bottomRight);
	int startIndex = state.findIndex(boundingBox.left());
	int endIndex = state.findIndex(boundingBox.right());

	QFile datafile("data/export.txt");
	datafile.open(QFile::WriteOnly);
	QTextStream out(&datafile);

	// Print the header.
	out << "# ";
	foreach (int id, idsOfTheCurvesToShow)
		out << state.memberNames[id] << " ";
	out << "\n";

	// Print the data.
	for (int t = startIndex; t > endIndex; t--)
	{
		foreach (int id, idsOfTheCurvesToShow)
			out << state[t](id) << " ";
		out << "\n";
	}

	datafile.close();

	emit messageOut("Data exported to data/export.txt");
}

// Switches a specific curve on and off.
void GraphWidget::setShowCurve(int id, bool on)
{
	if (on && !idsOfTheCurvesToShow.contains(id))
		idsOfTheCurvesToShow << id;
	else if (!on && idsOfTheCurvesToShow.contains(id))
		idsOfTheCurvesToShow.removeAll(id);

	update();
}

// Handles the swipe fade out (inertial motion after the mouse button was released).
void GraphWidget::swipeFadeOut()
{
	if (swipeFadeOutVelocity.manhattanLength() < 0.8)
	{
		swipeFadeOutVelocity *= 0;
		swipeFadeOutTimer.stop();
	}
	else
	{
		double elapsedTime = (stopWatch.programTime() - lastSwipeFadeOutTimeStamp);
		QPointF mappedSwipeFadeOutVelocity = QPointF(swipeFadeOutVelocity.x()*screenScale.x(), -swipeFadeOutVelocity.y()*screenScale.y());
		screenOffset -= mappedSwipeFadeOutVelocity*elapsedTime;
		swipeFadeOutVelocity *= qMax(0.0, 1.0 - 4.0*elapsedTime);
		updateScreenTransform();
	}

	lastSwipeFadeOutTimeStamp = stopWatch.programTime();
	update();
}


void GraphWidget::keyReleaseEvent(QKeyEvent *event)
{
	if (event->isAutoRepeat())
		return;

	update();
}

void GraphWidget::mousePressEvent(QMouseEvent *event)
{
    mouseClick = event->localPos();
	mouseClickTimeStamp = stopWatch.programTime();

	// If the mouseMoveEvent reports a curve closer than the mouse catch radius, then it's the start of a drag.
	if (nearestCurveId > -1)
	{
		draggedCurveId = nearestCurveId;
		draggedCurveX = mappedMouse.x();
	}

	swipeFadeOutTimer.stop();
	swipeFadeOutVelocity *= 0;
	setCursor(Qt::ClosedHandCursor);
	dragMoved = false;
}

void GraphWidget::mouseMoveEvent(QMouseEvent *event)
{
    updateMouse(event->localPos());

	// Curve highlighting.
	// If we are not dragging anything...
	// Find the closest curve to the mouse and check if it's inside the mouse catch radius.
	if (draggedCurveId == -1)
	{
		// Go through the curves and check if the mouse is close enough to one of them.
		double nearestDist = mouseCatchRadius*screenScale.y();
		nearestCurveId = -1;
		foreach (int id, idsOfTheCurvesToShow)
		{
			Curve curve = curves[id];

            double y = curve.interpolatedTransformedValueAt(mappedMouse.x() + state.time);
			double dist = qAbs(y - mappedMouse.y());
			if (dist < nearestDist)
			{
				nearestDist = dist;
				nearestCurveId = id;
			}

			curves[id].highlight = false;
		}

		// Highlight the curve if mouse is near.
		if (nearestCurveId > -1)
			curves[nearestCurveId].highlight = true;
	}


	// Detect drag motion and scale or translate accordingly.
	if (event->buttons() & (Qt::LeftButton | Qt::RightButton | Qt::MiddleButton))
	{
		dragMoved = true;

		// If we are dragging a curve...
		if (draggedCurveId > -1)
		{
			// Scale and translate functions.
			if (event->modifiers() & Qt::ControlModifier || event->buttons() & (Qt::RightButton | Qt::MiddleButton))
			{
                curves[draggedCurveId].stretchBy(draggedCurveX + state.time, mappedMouseDiff.y()); // + t because the curves are offset by -t
			}
			else
			{
				curves[draggedCurveId].translate(0, mappedMouseDiff.y());
			}
		}

		// ...otherwise we could be scaling the screen
		else if (event->modifiers() & Qt::ControlModifier || event->buttons() & (Qt::RightButton | Qt::MiddleButton))
		{
			if (mappedMouse.x() < 0)
				screenScale.rx() *= (mappedMouse.x()-mappedMouseDiff.x()) / mappedMouse.x();
			screenScale.ry() *= (mappedMouse.y()-mappedMouseDiff.y()) / mappedMouse.y();
			updateScreenTransform();
		}

		// ...or just translating the screen.
		else
		{
			screenOffset -= mappedMouseDiff;
			updateScreenTransform();
		}
	}

	update();
}

void GraphWidget::mouseReleaseEvent(QMouseEvent *event)
{
	// Add a new marker if didn't drag and a curve is near.
	if (!dragMoved && nearestCurveId > -1)
	{
        curves[nearestCurveId].setMarkerAt(mappedMouse.x()+state.time);
	}

	// Start swipe fade out if the screen was dragged.
	else if (draggedCurveId == -1 && mouse != mouseClick && event->button() == Qt::LeftButton)
	{
		updateMouse(event->localPos());
		swipeFadeOutVelocity = mouseVelocity;
		lastSwipeFadeOutTimeStamp = stopWatch.programTime();
		swipeFadeOutTimer.start();
	}

	dragMoved = false;
	draggedCurveId = -1;
	unsetCursor();
	update();
}

// Updates the mouse state (position and velocity).
void GraphWidget::updateMouse(QPointF mousePos)
{
	mouse = mousePos;
	mousePresent = true;
	mappedMouse = screenTransform.inverted().map(mouse);
	mouseDiff = (mouse - lastMouse);
	mappedMouseDiff = QPointF(mouseDiff.x()*screenScale.x(), mouseDiff.y()*screenScale.y());
	mappedMouseDiff.ry() = -mappedMouseDiff.y();

	double timeDiff = (stopWatch.time()-lastMouseUpdateTimeStamp);
	if (timeDiff > 0.3)
	{
		mouseVelocity *= 0;
		mappedMouseVelocity = QPointF(mouseVelocity.x()*screenScale.x(), mouseVelocity.y()*screenScale.y());
		mappedMouseVelocity.ry() = -mappedMouseVelocity.y();
		lastMouse = mouse;
		lastMouseUpdateTimeStamp = stopWatch.time();
	}
	else if (timeDiff >= 0.003)
	{
		QPointF measuredMouseVelocity = (mouse - lastMouse)/timeDiff;
		mouseVelocity = 0.5*mouseVelocity + 0.5*measuredMouseVelocity;
		mappedMouseVelocity = QPointF(mouseVelocity.x()*screenScale.x(), mouseVelocity.y()*screenScale.y());
		mappedMouseVelocity.ry() = -mappedMouseVelocity.y();
		lastMouse = mouse;
		lastMouseUpdateTimeStamp = stopWatch.time();
	}

	//qDebug() << mouseDiff << mappedMouseDiff << lastMouse << timeDiff;
}

void GraphWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
	// If the mouseMoveEvent reports a curve closer than the mouse catch radius, call reset() on it.
	if (nearestCurveId > -1)
		curves[nearestCurveId].reset();
	else if (event->buttons() & (Qt::RightButton | Qt::MiddleButton))
		screenScale = QPointF(0.01, 0.01);
	else
		screenOffset = QPointF();

	updateScreenTransform();
	update();
}

void GraphWidget::wheelEvent(QWheelEvent *event)
{
	if (event->delta() > 0)
	{
		screenScale /= 1.2;
//		screenOffset = screenOffset/1.2;
	}
	else
	{
		screenScale *= 1.2;
//		screenOffset = screenOffset*1.2;
	}

	updateScreenTransform();
	updateMouse(QPointF(event->pos()));
	update();
}

void GraphWidget::leaveEvent(QEvent* event)
{
    Q_UNUSED(event);
	mousePresent = false;
}

void GraphWidget::resizeEvent(QResizeEvent* event)
{
    Q_UNUSED(event);
	updateScreenTransform();
}

// Updates the transformation from screen to logical coordinates.
// This should be called each time when the scaling factor or the
// screen translation changes.
void GraphWidget::updateScreenTransform()
{
	screenTransform = QTransform();
	screenTransform.translate(width()/2, height()/2);
	screenTransform.scale(1.0/screenScale.x(), -1.0/screenScale.y());
	screenTransform.translate(-screenOffset.x(), -screenOffset.y());

	mappedMouse = screenTransform.inverted().map(mouse);

	//qDebug() << screenScale.x() << screenTransform.m11() << 1.0/(screenTransform.m11()*config.systemIterationTime);
}

void GraphWidget::paintEvent(QPaintEvent*)
{
	double x, y;
	QPointF mp;

	// Start the widget painter.
	QPainter painter(this);

	// Prepare the basic painter colors and pen.
	QPen pen = QPen(QColor(0, 0, 0));
	pen.setCosmetic(true);
	painter.setPen(pen);
	painter.setFont(QFont("Arial", 8));
	QFontMetrics fm(painter.font());
	int fontHeight = fm.height();

	// Calculate the bounding box in the transformed coordinate system.
	QPointF topLeft = screenTransform.inverted().map(QPointF(0,0));
	QPointF bottomRight = screenTransform.inverted().map(QPointF(width(),height()));
	QRectF boundingBox = QRectF(topLeft, bottomRight);

	// Apply the coordinate transformation from device coordinates ((0,0) is in the top left corner)
	// to logical coordinates, where the origin of the coordinate system is in the center
	// and y grows upwards. A scaling factor converts from pixel values to logical units
	// (e.g. 100 px = 1 second).
	painter.setTransform(screenTransform);

	// Draw the axes.
	if (showAxes)
	{
		painter.drawLine(QPointF(boundingBox.left(), 0), QPointF(boundingBox.right(), 0));
		painter.drawLine(QPointF(0, boundingBox.bottom()), QPointF(0, boundingBox.top()));
		if (screenScale.y() < 0.3)
		{
			for (double i = floor(boundingBox.left()); i < boundingBox.right(); i=i+1.0)
				painter.drawLine(QPointF(i, 0), QPointF(i, -4.0*screenScale.y()));
			for (double i = floor(boundingBox.bottom()); i < boundingBox.top(); i=i+1.0)
				painter.drawLine(QPointF(0, i), QPointF(4.0*screenScale.x(), i));
		}
	}

	// No data guard.
	if (state.size() == 0)
		return;

	// Offset by t before drawing the curves.
	// This has the effect that the y-axis marks the current frame.
    painter.translate(-state.time, 0);

	// Draw the selected curves.
	painter.setRenderHint(QPainter::Antialiasing, true);
	foreach (int id, idsOfTheCurvesToShow)
		curves[id].draw(&painter);

	// Draw the mouse line on top of the curves.
	if (mousePresent)
	{
		pen.setColor(QColor("grey"));
		painter.setPen(pen);
        x = qBound(state[state.size()-1].time, mappedMouse.x()+state.time, state.time);
		painter.drawLine(QPointF(x, boundingBox.bottom()), QPointF(x, boundingBox.top()));
	}

	// Draw the mouse marker and the zero marker.
	foreach (int id, idsOfTheCurvesToShow)
	{
		pen.setColor(curves[id].color);
		pen.setWidth(2+2*curves[id].highlight);
		painter.setPen(pen);

		// Draw the mouse marker on the curve.
		if (mousePresent)
		{
            x = qBound(state[state.size()-1].time, mappedMouse.x()+state.time, state.time);
			y = qBound(boundingBox.bottom(), curves[id].transformedValueAt(x), boundingBox.top());
			painter.drawEllipse(QPointF(x,y), (4.0+2.0*curves[id].highlight)*screenScale.x(), (4.0+2.0*curves[id].highlight)*screenScale.y());
		}

		// Draw the zero marker on the curve.
        x = state.time;
		y = qBound(boundingBox.bottom(), curves[id].interpolatedTransformedValueAt(x), boundingBox.top());
		painter.drawEllipse(QPointF(x,y), (4.0+2.0*curves[id].highlight)*screenScale.x(), (4.0+2.0*curves[id].highlight)*screenScale.y());
	}

	// To write the marker values, the screen transformation has to be reset so that the font is not upside down.
	// Instead, each text coordinate is transformed individually.
	painter.resetTransform();
	foreach (int id, idsOfTheCurvesToShow)
	{
		pen.setColor(curves[id].color);
		pen.setWidth(1);
		painter.setPen(pen);
		painter.setFont(QFont("Arial", 8, curves[id].highlight ? QFont::DemiBold : -1));

		// Draw the mouse marker value.
		if (mousePresent)
		{
            x = qBound(state[state.size()-1].time-state.time, mappedMouse.x(), state.time-state.time);
            y = curves[id].transformedValueAt(mappedMouse.x()+state.time);
			mp = screenTransform.map(QPointF(x, y)) + QPointF(10,-4);
			mp.ry() = qBound((double)(fontHeight), mp.y(), (double)(height()-1));
            painter.drawText(mp, QString::number(curves[id].valueAt(mappedMouse.x()+state.time), 'f', 3) );
		}

		// Draw the zero marker value.
		x = 0;
        y = curves[id].interpolatedTransformedValueAt(state.time);
		mp = screenTransform.map(QPointF(x, y)) + QPointF(10,-4);
		mp.ry() = qBound((double)(fontHeight), mp.y(), (double)(height()-1));
        painter.drawText(mp, QString::number(curves[id].interpolatedValueAt(state.time), 'f', 3) );
	}

	// Show the current time on the bottom.
	if (mousePresent)
	{
        x = qBound(state[state.size()-1].time-state.time, mappedMouse.x(), state.time-state.time);
		y = -height();
		mp = screenTransform.map(QPointF(x, y)) + QPointF(10,0);
		mp.ry() = qBound((double)(fontHeight), mp.y(), (double)(height()-4));
		painter.setPen(QColor("grey"));
		painter.drawText(mp, QString::number(x, 'f', 3));
	}
}

#ifndef CURVE_H_
#define CURVE_H_
#include <QtGui>

class Curve
{
public:

	int stateMemberId;
	QList<double> markers;
	QTransform transform;
	QColor color;
	bool highlight;
	QList<int> indices;

public:

	Curve();
	~Curve(){};

	void draw(QPainter* painter);
	int findIndex(double t);
	double interpolatedValueAt(double t);
	double interpolatedTransformedValueAt(double t);
	double valueAt(double t);
	double transformedValueAt(double t);
	void translate(double x, double y=0);
	void scale(double x, double y);
	void stretchBy(double x, double y);
	void reset();
	void setMarkerAt(double t);
	double dx();
	double dy();
	double scalex();
	double scaley();
};

#endif // CURVE_H_

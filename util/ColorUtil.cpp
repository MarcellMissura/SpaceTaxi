#include "ColorUtil.h"
#include <QtCore/qmath.h>
#include "util/Statistics.h"

// The ColorUtil is a useful helper when used in the context
// of rendering on a QPainter. Mostly, it is a collection of
// named QPen and QBrush objects such as penRedThin that are
// already set up ready for use, but the ColorUtil also offers
// a color interpolation interface for height maps and heat
// maps and random color sampling functions.

ColorUtil colorUtil;

ColorUtil::ColorUtil()
{
    // A blue-red-yellow palette for mapping colors for height maps.
    heightMapPalette << QColor("#000077");
	heightMapPalette << QColor("#0000FF");
	heightMapPalette << QColor("#FF0000");
	heightMapPalette << QColor("#FFFF00");

    // A transparent white to opaque red palette good for heat maps.
    heatMapPalette << QColor("#00FFFFFF");
    heatMapPalette << QColor("#FFFF0000");

    for (uint i = 0; i < 128; i++)
        randomPalette << sampleUniformColor();

    pen.setCosmetic(true);
    pen.setWidth(2);

    penThick.setCosmetic(true);
    penThick.setWidth(3);

    penThin.setCosmetic(true);
    penThin.setWidth(1);

    penDashed.setCosmetic(true);
    penDashed.setStyle(Qt::DashLine);
    penDashed.setWidth(2);

    penThinDashed.setCosmetic(true);
    penThinDashed.setStyle(Qt::DashLine);
    penThinDashed.setWidth(1);

    penWhite.setCosmetic(true);
    penWhite.setWidth(2);
    penWhite.setColor(Qt::white);

    penWhiteDashed.setCosmetic(true);
    penWhiteDashed.setStyle(Qt::DashLine);
    penWhiteDashed.setWidth(3);
    penWhiteDashed.setColor(Qt::white);

    penWhiteThin.setCosmetic(true);
    penWhiteThin.setWidth(1);
    penWhiteThin.setColor(Qt::white);

    penWhiteThick.setCosmetic(true);
    penWhiteThick.setWidth(3);
    penWhiteThick.setColor(Qt::white);

    penGray.setCosmetic(true);
    penGray.setWidth(2);
    penGray.setColor(QColor::fromRgb(100,100,100));

    penGrayThin.setCosmetic(true);
    penGrayThin.setWidth(1);
    penGrayThin.setColor(QColor::fromRgb(175,175,175));

    penGrayDashed.setCosmetic(true);
    penGrayDashed.setStyle(Qt::DashLine);
    penGrayDashed.setWidth(2);
    penGrayDashed.setColor(QColor::fromRgb(175,175,175));

    penGrayThinDashed.setCosmetic(true);
    penGrayThinDashed.setStyle(Qt::DashLine);
    penGrayThinDashed.setWidth(1);
    penGrayThinDashed.setColor(QColor::fromRgb(175,175,175));

    penGrayThick.setCosmetic(true);
    penGrayThick.setWidth(3);
    penGrayThick.setColor(QColor::fromRgb(175,175,175));

    penLightGray.setCosmetic(true);
    penLightGray.setWidth(2);
    penLightGray.setColor(QColor::fromRgb(200,200,200));

    penLightGrayThin.setCosmetic(true);
    penLightGrayThin.setWidth(1);
    penLightGrayThin.setColor(QColor::fromRgb(200,200,200));

    penLightGrayDashed.setCosmetic(true);
    penLightGrayDashed.setStyle(Qt::DashLine);
    penLightGrayDashed.setWidth(2);
    penLightGrayDashed.setColor(QColor::fromRgb(200,200,200));


    penDarkGray.setCosmetic(true);
    penDarkGray.setWidth(2);
    penDarkGray.setColor(QColor::fromRgb(150,150,150));

    penDarkGrayThin.setCosmetic(true);
    penDarkGrayThin.setWidth(1);
    penDarkGrayThin.setColor(QColor::fromRgb(150,150,150));

    penDarkGrayDashed.setCosmetic(true);
    penDarkGrayDashed.setStyle(Qt::DashLine);
    penDarkGrayDashed.setWidth(2);
    penDarkGrayDashed.setColor(QColor::fromRgb(150,150,150));

    penDarkGrayThinDashed.setCosmetic(true);
    penDarkGrayThinDashed.setStyle(Qt::DashLine);
    penDarkGrayThinDashed.setWidth(1);
    penDarkGrayThinDashed.setColor(QColor::fromRgb(150,150,150));


    penRed.setCosmetic(true);
    penRed.setWidth(2);
    penRed.setColor(Qt::red);

    penRedDashed.setCosmetic(true);
    penRedDashed.setWidth(2);
    penRedDashed.setStyle(Qt::DashLine);
    penRedDashed.setColor(Qt::red);

    penRedThin.setCosmetic(true);
    penRedThin.setWidth(1);
    penRedThin.setColor(Qt::red);

    penRedThinDashed.setCosmetic(true);
    penRedThinDashed.setWidth(1);
    penRedThinDashed.setStyle(Qt::DashLine);
    penRedThinDashed.setColor(Qt::red);

    penRedThick.setCosmetic(true);
    penRedThick.setWidth(3);
    penRedThick.setColor(Qt::red);

    penYellow.setCosmetic(true);
    penYellow.setWidth(2);
    penYellow.setColor(Qt::yellow);

    penYellowThick.setCosmetic(true);
    penYellowThick.setWidth(5);
    penYellowThick.setColor(Qt::yellow);

    penOrange.setCosmetic(true);
    penOrange.setWidth(2);
    penOrange.setColor(QColor::fromRgb(255,165,0));

    penOrangeThick.setCosmetic(true);
    penOrangeThick.setWidth(7);
    penOrangeThick.setColor(QColor::fromRgb(255,165,0));

    penBlue.setCosmetic(true);
    penBlue.setWidth(2);
    penBlue.setColor(Qt::blue);

    penBlueThin.setCosmetic(true);
    penBlueThin.setWidth(1);
    penBlueThin.setColor(Qt::blue);

    penBlueThick.setCosmetic(true);
    penBlueThick.setWidth(3);
    penBlueThick.setColor(Qt::blue);

    penGreen.setCosmetic(true);
    penGreen.setWidth(2);
    penGreen.setColor(Qt::green);

    penGreenThin.setCosmetic(true);
    penGreenThin.setWidth(1);
    penGreenThin.setColor(Qt::green);

    penGreenThick.setCosmetic(true);
    penGreenThick.setWidth(3);
    penGreenThick.setColor(Qt::green);


    brush.setColor(QColor::fromRgb(0,0,0));
    brush.setStyle(Qt::SolidPattern);

    brushLightGray.setColor(QColor::fromRgb(200,200,200));
    brushLightGray.setStyle(Qt::SolidPattern);

    brushGray.setColor(QColor::fromRgb(175,175,175));
    brushGray.setStyle(Qt::SolidPattern);

    brushDarkGray.setColor(QColor::fromRgb(150,150,150));
    brushDarkGray.setStyle(Qt::SolidPattern);

    brushWhite.setColor(QColor::fromRgb(255,255,255));
    brushWhite.setStyle(Qt::SolidPattern);

    brushIvory.setColor(QColor::fromRgb(255,255,240));
    brushIvory.setStyle(Qt::SolidPattern);

    brushYellow.setColor(QColor::fromRgb(255,255,0));
    brushYellow.setStyle(Qt::SolidPattern);

    brushRed.setColor(QColor::fromRgb(255,0,0));
    brushRed.setStyle(Qt::SolidPattern);

    brushOrange.setColor(QColor::fromRgb(255,100,0));
    brushOrange.setStyle(Qt::SolidPattern);

    brushBlue.setColor(QColor::fromRgb(0,0,255));
    brushBlue.setStyle(Qt::SolidPattern);

    brushLightBlue.setColor(QColor::fromRgb(125,125,255));
    brushLightBlue.setStyle(Qt::SolidPattern);

    brushMagenta.setColor(QColor::fromRgb(255,0,255));
    brushMagenta.setStyle(Qt::SolidPattern);

    brushGreen.setColor(QColor::fromRgb(0,255,0));
    brushGreen.setStyle(Qt::SolidPattern);

    brushLightGreen.setColor(QColor::fromRgb(100,255,100));
    brushLightGreen.setStyle(Qt::SolidPattern);
}

// Maps a value v between min and max to a color in the blue-red-yellow height map palette.
QColor ColorUtil::getHeightMapColor(double v, double min, double max)
{
	if (min >= max - 0.000001)
		return heightMapPalette[0];

	v = qBound(min, v, max - 0.000001); // We subtract an epsilon to avoid hitting exactly the max value.
	double dblIndx = (heightMapPalette.size()-1) * (v-min)/(max-min);
	int paletteIndex = qFloor(dblIndx);
	double factor = dblIndx - paletteIndex;
	return QColor((1.0-factor)*heightMapPalette[paletteIndex].red() + factor*heightMapPalette[paletteIndex+1].red(),
                  (1.0-factor)*heightMapPalette[paletteIndex].green() + factor*heightMapPalette[paletteIndex+1].green(),
                  (1.0-factor)*heightMapPalette[paletteIndex].blue() + factor*heightMapPalette[paletteIndex+1].blue());
}

// Maps a value v between min and max to a color in the heat map palette.
// The heat map is a transparent-white to opaque-red space where low values near the minimum will be transparent
// and high values will be opaque and red. A custom color (instead of red) can be specified with the
// setHeatMapColor() function.
QColor ColorUtil::getHeatMapColor(double v, double min, double max)
{
    if (min >= max - 0.000001)
        return heatMapPalette[0];

    v = qBound(min, v, max - 0.000001); // We subtract an epsilon to avoid hitting exactly the max value.
    double dblIndx = (heatMapPalette.size()-1) * (v-min)/(max-min);
    int paletteIndex = qFloor(dblIndx);
    double factor = dblIndx - paletteIndex;
    //qDebug() << factor << paletteIndex << heatMapPalette[paletteIndex] << heatMapPalette[paletteIndex].alpha();
    return QColor((1.0-factor)*heatMapPalette[paletteIndex].red() + factor*heatMapPalette[paletteIndex+1].red(),
                  (1.0-factor)*heatMapPalette[paletteIndex].green() + factor*heatMapPalette[paletteIndex+1].green(),
                  (1.0-factor)*heatMapPalette[paletteIndex].blue() + factor*heatMapPalette[paletteIndex+1].blue(),
                  (1.0-factor)*heatMapPalette[paletteIndex].alpha() + factor*heatMapPalette[paletteIndex+1].alpha());
}

// The getHeatMapColor() function maps a value to a range from transparent
// white to opaque red. Using this function, you can overwrite the red with
// a different color.
void ColorUtil::setHeatMapColor(QColor c)
{
    heatMapPalette.last() = c;
}

// Resets the heat map color to default red.
void ColorUtil::resetHeatMapColor()
{
    heatMapPalette.last() = QColor("#FFFF0000");
}

// Returns a color sample with a high blue component.
QColor ColorUtil::sampleBlueColor()
{
    double randh = Statistics::uniformSample();
//	double rands = Statistics::uniformSample();
    double randv = Statistics::uniformSample();

	int h = 210 + randh*60;
	int s = 255;
	int v = 128 + randv*127;
	return QColor::fromHsv(h,s,v);
}

// Returns a color sample with a high red component.
QColor ColorUtil::sampleRedColor()
{
    double randh = Statistics::uniformSample();
//	double rands = Statistics::uniformSample();
    double randv = Statistics::uniformSample();

	int h = int(330 + randh*60) % 360;
	int s = 255;
	int v = 128 + randv*127;
	return QColor::fromHsv(h,s,v);
}

// Returns a uniformly sampled color.
QColor ColorUtil::sampleUniformColor()
{
    double randh = Statistics::uniformSample();
//	double rands = Statistics::uniformSample();
    double randv = Statistics::uniformSample();

	int h = randh*359;
	int s = 255;
	int v = 128 + randv*127;

	return QColor::fromHsv(h,s,v);
}

// Returns a uniformly sampled color.
QColor ColorUtil::randomColor(uint idx)
{
    return randomPalette[min(idx, randomPalette.size()-1)];
}

// Returns a color sample that avoids the red and blue corners.
QColor ColorUtil::sampleNonRBColor()
{
    double randh = Statistics::uniformSample();
//	double rands = Statistics::uniformSample();
    double randv = Statistics::uniformSample();

	int h = 30 + randh*180;
	int s = 255;
	int v = 64 + randv*(255-64);

    return QColor::fromHsv(h,s,v);
}

// Returns a cross expressed as QPolygonF for to be drawn.
QPolygonF ColorUtil::getCrossPolygon()
{
    QPolygonF cross;
    cross.append(QPointF(-3, 1));
    cross.append(QPointF(-1, 1));
    cross.append(QPointF(-1, 3));
    cross.append(QPointF(1, 3));
    cross.append(QPointF(1, 1));
    cross.append(QPointF(3, 1));
    cross.append(QPointF(3, -1));
    cross.append(QPointF(1, -1));
    cross.append(QPointF(1, -3));
    cross.append(QPointF(-1, -3));
    cross.append(QPointF(-1, -1));
    cross.append(QPointF(-3, -1));
    return cross;
}

QDataStream& operator<<(QDataStream& out, const RGBPixel &o)
{
    out << o.r;
    out << o.g;
    out << o.b;
    return out;
}

QDataStream& operator>>(QDataStream& in, RGBPixel &o)
{
    in >> o.r;
    in >> o.g;
    in >> o.b;
    return in;
}

#ifndef COLORUTIL_H_
#define COLORUTIL_H_
#include <QPen>
#include <QBrush>
#include <QColor>
#include "util/Vector.h"

// The ColorUtil is a useful helper when used in the context
// of rendering on a QPainter. Mostly, it is a collection of
// named QPen and QBrush objects such as penRedThin that are
// already set up ready for use, but the ColorUtil also offers
// a color interpolation interface for height maps and heat
// maps and random color sampling functions.

// Holds the value of a single color image pixel in 24-bit RGB format.
struct RGBPixel
{
    uchar r;
    uchar g;
    uchar b;

    // OpenGL support.
    operator const uchar*() const {return (const uchar*)this;}
};

struct ColorUtil
{
    ColorUtil();
    ~ColorUtil(){}

    Vector<QColor> heightMapPalette;
    Vector<QColor> heatMapPalette;
    Vector<QColor> randomPalette;

    QPen pen;
    QPen penDashed;
    QPen penThick;
    QPen penThin;
    QPen penThinDashed;
    QPen penWhite;
    QPen penWhiteDashed;
    QPen penWhiteThin;
    QPen penWhiteThick;

    QPen penGray;
    QPen penGrayDashed;
    QPen penGrayThin;
    QPen penGrayThinDashed;
    QPen penGrayThick;

    QPen penLightGray;
    QPen penLightGrayDashed;
    QPen penLightGrayThin;
    QPen penDarkGray;
    QPen penDarkGrayDashed;
    QPen penDarkGrayThin;
    QPen penDarkGrayThinDashed;
    QPen penRed;
    QPen penRedDashed;
    QPen penRedThin;
    QPen penRedThinDashed;
    QPen penRedThick;
    QPen penYellow;
    QPen penYellowThick;
    QPen penOrange;
    QPen penOrangeThick;
    QPen penBlue;
    QPen penBlueThin;
    QPen penBlueThick;
    QPen penGreen;
    QPen penGreenThick;
    QPen penGreenThin;

    QBrush brush;
    QBrush brushGray;
    QBrush brushLightGray;
    QBrush brushDarkGray;
    QBrush brushWhite;
    QBrush brushIvory;
    QBrush brushYellow;
    QBrush brushOrange;
    QBrush brushRed;
    QBrush brushBlue;
    QBrush brushLightBlue;
    QBrush brushMagenta;
    QBrush brushGreen;
    QBrush brushLightGreen;

    QColor getHeightMapColor(double v, double min, double max);
    QColor getHeatMapColor(double v, double min, double max);
    QColor randomColor(uint idx=0);
    void setHeatMapColor(QColor c);
    void resetHeatMapColor();

    static QColor sampleRedColor();
    static QColor sampleBlueColor();
    static QColor sampleUniformColor();
    static QColor sampleNonRBColor();

    static QPolygonF getCrossPolygon();
};

extern ColorUtil colorUtil;


QDataStream& operator<<(QDataStream& out, const RGBPixel &o);
QDataStream& operator>>(QDataStream& in, RGBPixel &o);

#endif /* COLORUTIL_H_ */


#ifndef COLORUTIL_H_
#define COLORUTIL_H_
#include <QPen>
#include <QBrush>
#include <QColor>
#include "lib/util/Vector.h"
#include "lib/util/Vec2.h"

// The ColorUtil is a useful helper when used in the context
// of rendering on a QPainter. Mostly, it is a collection of
// named QPen and QBrush objects, for example penRedThin, that
// are already set up ready for use, but the ColorUtil also
// offers a color interpolation interface for height maps and
// heat maps, random color sampling functions, and a cross
// polygon.

struct DrawUtil
{
    DrawUtil();
    ~DrawUtil(){}

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
    QPen penRedThicker;
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
    QBrush brushTransparent;
    QBrush brushIvory;
    QBrush brushYellow;
    QBrush brushOrange;
    QBrush brushRed;
    QBrush brushBlue;
    QBrush brushLightBlue;
    QBrush brushMagenta;
    QBrush brushGreen;
    QBrush brushLightGreen;

    QColor white;
    QColor black;
    QColor grey;
    QColor lightGrey;
    QColor darkGrey;
    QColor yellow;
    QColor red;
    QColor green;
    QColor lightGreen;
    QColor blue;
    QColor ivory;
    QColor transparent;

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
    static QPainterPath getArrow(const Vec2 &from, const Vec2 &to);
};

extern DrawUtil drawUtil;

#endif /* COLORUTIL_H_ */


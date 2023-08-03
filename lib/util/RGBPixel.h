#ifndef RGBPIXEL_H
#define RGBPIXEL_H
#include <QDataStream>
#include <QDebug>

struct RGBPixel
{
    uchar r;
    uchar g;
    uchar b;

    // OpenGL support.
    operator const uchar*() const {return (const uchar*)this;}

    bool isNan() const;
};

QDataStream& operator<<(QDataStream& out, const RGBPixel &o);
QDataStream& operator>>(QDataStream& in, RGBPixel &o);
QDebug operator<<(QDebug dbg, const RGBPixel &o);

#endif


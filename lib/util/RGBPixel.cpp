#include "RGBPixel.h"


#include <cmath>

bool RGBPixel::isNan() const
{
    return std::isnan(r) || std::isnan(g) || std::isnan(b);
}

QDebug operator<<(QDebug dbg, const RGBPixel &o)
{
    dbg << "r:" << o.r << "g:" << o.g << "b:" << o.b;
    return dbg;
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




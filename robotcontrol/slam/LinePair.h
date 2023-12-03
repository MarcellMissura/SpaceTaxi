#ifndef LINEPAIR_H
#define LINEPAIR_H

#include "lib/geometry/Line.h"
#include "lib/util/Pose2D.h"
#include "TrackedLine.h"

class LinePair
{

public:

    uint id;
    Pose2D inputPose; // The world pose the inputLine is seen in.
    TrackedLine* inputLine; // The percieved line relative to inputPose.
    TrackedLine* mapLine; // Pointer to the map line paired with the input line.
    double sortField; // A field used for sorting the line pairs by an arbitrary value.
    bool confirmed; // A flag indicating whether a line pair is confirmed.

public:

    LinePair();
    ~LinePair(){}

    double angleDiff() const;
    double lengthDiff() const;
    double ortho() const;
    double overlap() const;
    double lineLineDist() const;
    double linePoseDist() const;
    double projectionDistance() const;
    double percentualOverlap() const;
    double weight() const;
    TrackedLine prospectedLine() const;
    TrackedLine prospectedLineRotated() const;
    Pose2D tr() const;

    bool operator <(const LinePair& v) const {return sortField < v.sortField;}
    bool operator <=(const LinePair& v) const {return sortField <= v.sortField;}
    bool operator >(const LinePair& v) const {return sortField > v.sortField;}
    bool operator >=(const LinePair& v) const {return sortField >= v.sortField;}

    bool operator ==(const LinePair& v) const {return inputLine == v.inputLine && mapLine == v.mapLine;}
    bool operator !=(const LinePair& v) const {return inputLine != v.inputLine || mapLine != v.mapLine;}

    void draw() const;
    void drawQuick() const;
};

QDebug operator<<(QDebug dbg, const LinePair &n);
QDebug operator<<(QDebug dbg, const LinePair *n);

#endif

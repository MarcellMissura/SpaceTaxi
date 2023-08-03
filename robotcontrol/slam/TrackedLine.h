#ifndef TRACKEDLINE_H
#define TRACKEDLINE_H

#include "lib/util/Vector.h"
#include "lib/util/Pose2D.h"
#include "lib/geometry/Line.h"
class PoseGraphNode;

class TrackedLine : public Line
{
public:

    Vector<TrackedLine> lineObservations; // Storage for line observations that went into computing an "average".
    LinkedList<PoseGraphNode*> observerNodes;

    bool seenP1; // Has p1 been seen?
    bool seenP2; // Has p2 been seen?
    double totalWeight; // The accumulated weight of all obversations.
    uint observationCount; // How many times has this line been observed?
    uint firstSeen; // The frameId of the first observation.
    uint lastSeen; // The frameId of the last observation.
    bool active; // Flag that indicates whether the line is in the active set or not.


    TrackedLine();
    TrackedLine(const Line& l, uint frameId=0);
    TrackedLine(const Line& l, bool seenP1, bool seenP2, uint frameId=0);
    ~TrackedLine(){}

    void setTo(const TrackedLine &l);

    void flip();
    void sort();

    bool isActive() const;

    bool addLineObservation(const TrackedLine& l, double weight = 1.0, bool debug=false);
    bool testLineObservation(const TrackedLine& l, double weight = 1.0, bool debug=false);

    double lineLineDist(const TrackedLine& l, bool debug=false) const;
    double linePoseDist(const TrackedLine& l, const Pose2D& inputPose, bool debug=false) const;

    void streamOut(QDataStream& out) const;
    void streamIn(QDataStream& in);
};

QDebug operator<<(QDebug dbg, const TrackedLine &n);
QDebug operator<<(QDebug dbg, const TrackedLine* n);
QDataStream& operator<<(QDataStream& out, const TrackedLine &o);
QDataStream& operator>>(QDataStream& in, TrackedLine &o);

extern TrackedLine operator+(const TrackedLine& l, const Pose2D& p);
extern TrackedLine operator-(const TrackedLine& l, const Pose2D& p);
extern void operator+=(TrackedLine& l, const Pose2D& p);
extern void operator-=(TrackedLine& l, const Pose2D& p);
extern Vector<TrackedLine> operator+(const Vector<TrackedLine>& v, const Pose2D& p);
extern Vector<TrackedLine> operator-(const Vector<TrackedLine>& v, const Pose2D& p);
extern void operator+=(Vector<TrackedLine>& v, const Pose2D& p);
extern void operator-=(Vector<TrackedLine>& v, const Pose2D& p);
extern LinkedList<TrackedLine> operator+(const LinkedList<TrackedLine>& v, const Pose2D& p);
extern LinkedList<TrackedLine> operator-(const LinkedList<TrackedLine>& v, const Pose2D& p);
extern void operator+=(LinkedList<TrackedLine>& v, const Pose2D& p);
extern void operator-=(LinkedList<TrackedLine>& v, const Pose2D& p);

#endif

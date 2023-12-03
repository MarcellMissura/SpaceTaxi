#include "TrackedLine.h"
#include "board/Config.h"
#include "board/State.h"
#include "board/Command.h"
#include "lib/util/DrawUtil.h"
#include "lib/util/GLlib.h"

TrackedLine::TrackedLine() : Line()
{
    observationCount = 0;
    totalWeight = 1;
    lastSeen = 0;
    active = false;
    seenP1 = false;
    seenP2 = false;
    seenP1Count = 0;
    seenP2Count = 0;
}

TrackedLine::TrackedLine(const Line& l, uint frameId) : Line(l)
{
    type = l.getType();
    observationCount = 0;
    totalWeight = 1;
    lastSeen = frameId;
    active = false;
    seenP1 = false;
    seenP2 = false;
    seenP1Count = 0;
    seenP2Count = 0;
    sort(); // Makes sure that x1 < x2.
}

TrackedLine::TrackedLine(const Line& l, bool seenP1, bool seenP2, uint frameId) : Line(l)
{
    type = l.getType();
    observationCount = 0;
    totalWeight = 1;
    lastSeen = frameId;
    active = false;
    this->seenP1 = seenP1;
    this->seenP2 = seenP2;
    sort(); // Makes sure that x1 < x2.
}

// (Re)sets the TrackedLine to be the specified TrackedLine l.
void TrackedLine::setTo(const TrackedLine &l)
{
    set(l.p1(), l.p2());
    totalWeight = l.totalWeight;
    lineObservations = l.lineObservations;
    sort(); // Makes sure that x1 < x2.
}

// Flips the TrackedLine so that p1 becomes p2 and p2 becomes p1.
void TrackedLine::flip()
{
    Line::flip();
    bool bt = seenP1;
    seenP1 = seenP2;
    seenP2 = bt;
    //qDebug() << "Trackline flip" << *this << seenP1 << seenP2;
}

// Sorts the TrackedLine such that x1 < x2 and if x1 == x2 then y1 < y2;
void TrackedLine::sort()
{
    if (x1() > x2() || (x1() == x2() && y1() > y2()))
        flip();
}

// Returns true if this line is active, i.e. is a member of the active map lines set.
bool TrackedLine::isActive() const
{
    return active;
}

// Returns a reference to the observer nodes of this line. Only pointers are stored
// to the nodes, which are in saved in the GeometricMap, and pointers are evil.
const LinkedList<PoseGraphNode *> &TrackedLine::getObserverNodes() const
{
    return observerNodes;
}

// Sets (overwrites) the observer nodes of this line.
void TrackedLine::setObserverNodes(const LinkedList<PoseGraphNode *> &on)
{
    observerNodes = on;
    return;
}

// Records a line observation and updates the tracked line with a lightweight
// projection union algorithm. Line observations with too much projection distance
// or too little overlap are ignored. Returns true if the observation has been accepted.
bool TrackedLine::addLineObservation(const TrackedLine &l, double weight, bool debug)
{
    // We merge line observations with the tracked line by their line-line cost.
    // If the cost is below a theshold, the observed line is integrated into the weighted average
    // of all observed lines so far. The new length of the map line is determined by a projected
    // union of all involved lines projected onto the map line.
    if (weight == 0)
        return false;

    // Test the line-line distance.
    if (lineLineDist(l) > config.slamMergeMaxLineDist)
    {
        if (debug)
            qDebug() << state.frameId << "Line observation dropped due to high line-line-dist:" << lineLineDist(l, debug) << "lines:" << l.id << id;
        return false;
    }

    // This is only used for visualization and can be removed.
    if (command.keepLineObservations)
    {
        lineObservations << l;
        lineObservations << l.lineObservations;
    }

    // Flip for correct alignment.
    if (l.lineVector()*lineVector() < 0)
        flip();

    // Compute the weighted average interpolation line.
    Line oldLine = *this;
    double f1 = totalWeight/(totalWeight+weight);
    double f2 = weight/(totalWeight+weight);
    if (length2() > l.length2())
    {
        Vec2 n = normal();
        Vec2 v1 = orthogonalDistance(l.p1())*n;
        Vec2 v2 = orthogonalDistance(l.p2())*n;
        Vec2 pp1 = (f1+f2)*p1()+f2*v1;
        Vec2 pp2 = (f1+f2)*p2()+f2*v2;
        set(pp1, pp2);
        totalWeight += weight;

        // Apply the projection-union to determine the length.
        projectionUnion(l);
    }
    else
    {
        Vec2 n = l.normal();
        Vec2 v1 = l.orthogonalDistance(p1())*n;
        Vec2 v2 = l.orthogonalDistance(p2())*n;
        Vec2 pp1 = (f1+f2)*l.p1()+f1*v1;
        Vec2 pp2 = (f1+f2)*l.p2()+f1*v2;
        set(pp1, pp2);
        totalWeight += weight;

        // Apply the projection-union to determine the length.
        projectionUnion(oldLine);
    }

    // Overwrite seen vertices with the gliding weighted mean.
    if (seenP1)
    {
        Vec2 ppp1 = projection(oldLine.p1());
        if (l.seenP1)
            ppp1 = f1*p1()+f2*projection(l.p1());
        setP1(ppp1);
    }
    else if (l.seenP1)
    {
        setP1(projection(l.p1()));
        seenP1 = true;
    }

    if (seenP2)
    {
        Vec2 ppp2 = projection(oldLine.p2());
        if (l.seenP2)
            ppp2 = f1*p2()+f2*projection(l.p2());
        setP2(ppp2);
    }
    else if (l.seenP2)
    {
        setP2(projection(l.p2()));
        seenP2 = true;
    }

//    if (debug)
//    {
//        qDebug() << "Merging this line" << *this << "with" << l;
//        qDebug() << "v1 v2:" << v1 << v2 << "f1 f2:" << f1 << f2 << "pp1 pp2:" << pp1 << pp2;
//    }

    sort();
    lastSeen = state.frameId;
    observationCount++;

    //qDebug() << "map line" << line.id << "updated to" << line;

    return true;
}

// Merges the TrackedLine l with this one.
bool TrackedLine::mergeLine(const TrackedLine &l, bool debug)
{
    if (addLineObservation(l, l.totalWeight))
    {
        if (debug)
            qDebug() << state.frameId << "mergeLine: Merging map lines" << id << "and" << l.getId();

        observerNodes.unify(l.getObserverNodes());

        ListIterator<PoseGraphNode*> nodeIt = l.getObserverNodes().begin();
        while (nodeIt.hasNext())
        {
            ListIterator<TrackedLine*> lineIt = nodeIt.next()->seenMapLines.begin();
            while (lineIt.hasNext())
            {
                if (lineIt.cur()->getId() == l.getId())
                {
                    nodeIt.cur()->seenMapLines.remove(lineIt);
                    break;
                }
                else
                {
                    lineIt.next();
                }
            }
        }
    }

    return false;
}

// Computes the line-line-distance metric from TrackedLine l to this line. TrackedLine l is
// expected in the same coordinate frame as this line (world coordinates). The returned cost
// expresses by how much line l needs to move in order to be covered by this line, i.e. by how
// much it needs to rotate, translate orthogonally to this line, and translate parallel to this line.
double TrackedLine::lineLineDist(const TrackedLine &l, bool debug) const
{
    // The overlap is the length of the segment where both lines overlap.
    // The overlap is zero if the lines touch in only one point. The overlap is negative if the lines
    // do not overlap. It is then the (negative) distance between their closest points.
    double overlap = projectionOverlap(l);

    // The projection distance of line l onto this line is the sum of the absolute orthogonal
    // distances of p1 and p2 of line l to this line. In experiments this proved to be a suitable
    // line to line distance superior to angle and ortho.
    double proj = projectionDistance(l);

    // The cost of the total transformation is simply the sum of the components.
    // The overlap cost is not very precise and expresses by how much line l would need to move
    // horizontally to this line for the shorter of the two lines to be completely covered by the other.
    double cost = proj - min(overlap-config.slamMergeMinOverlap, 0.0);

    if (debug)
    {
        qDebug() << "lineLineDist" << l.id << id
                 //<< "lines:" << l << *this
                 << "proj:" << proj
                 << "overlap:" << overlap << min(overlap-config.slamMergeMinOverlap, 0.0)
                 << "cost:" << cost;
    }

    return cost;
}

// Computes the line-pose-distance metric from TrackedLine l to this line while taking the input
// pose into account the line l is seen from. TrackedLine l is expected in local coordinates
// relative to inputPose while this line has to be in the same coordinate frame as the inputPose
// (world coordinates). The returned cost expresses by how much line l needs to move in order to
// be covered by this line, i.e. by how much it needs to rotate around the inputPose, translate
// orthogonally to this line, and translate parallel to this line.
double TrackedLine::linePoseDist(const TrackedLine &l, const Pose2D &inputPose, bool debug) const
{
    // The smaller angle is always used. The maximum angle diff between two lines is pi half.
    double angleDiff = pihalfcut(angle()-(l.angle()+inputPose.z));

    // Rotate the input line to the angle of this line around the input pose. It is important to rotate
    // around the input pose so that we compute the correct orthogonal distance after the rotation.
    Line rotatedInputLine = l + inputPose.turned(angleDiff);

    // Orthogonal distance *after* the rotation. Note that the ortho of the line-pose-distance is
    // measured after the rotation of the mapped input line to the angle of the map line.
    double ortho = orthogonalDistance(rotatedInputLine.p1());

    // Overlap after the rotation. The overlap is the length of the segment where both lines overlap.
    // The overlap is zero if the lines touch in only one point. The overlap is negative if the lines
    // do not overlap. It is then the (negative) distance between their closest points.
    double overlap = projectionOverlap(rotatedInputLine);

    // The cost of the total transformation is simply the sum of the components.
    // The overlap cost is not very precise and expresses by how much a line l would need to move
    // horizontally to this line for the shorter of the two lines to be completely covered by the other.
    // I found that lines that overlap by at least 0.5 meters are always a good match regardless of
    // their lengths, so I decided to saturate the lengths at 1.0 meters.
    double cost = fabs(angleDiff) + fabs(ortho) + max(min(l.length(), length(), 1.0) - overlap, 0.0);

    if (debug)
        qDebug() << "linePoseDist" << l.id << id << "angleDiff:" << angleDiff << "ortho:" << ortho
                 << "lengths:" << l.length() << length()
                 //<< "line:" << *this
                 //<< "rot line:" << rotatedInputLine
                 //<< "input pose:" << inputPose
                 << "overlap:" << overlap << max(min(l.length(), length(), 1.0) - overlap, 0.0) << "cost:" << cost;

    return cost;
}

// Draws the tracked line in OpenGL including the seen corners.
void TrackedLine::draw() const
{
    glBegin(GL_LINES);
    glVertex2d(p1().x, p1().y);
    glVertex2d(p2().x, p2().y);
    glEnd();
    if (seenP1)
        GLlib::drawFilledCircle(p1(), drawUtil.brush.color(), 0.03);
    if (seenP2)
        GLlib::drawFilledCircle(p2(), drawUtil.brush.color(), 0.03);

}

// Draws the tracked line on a QPainter including the seen corners.
// Pen, brush, and opacity already need to be set.
void TrackedLine::draw(QPainter *painter, const QPen& pen, double opacity) const
{
    if (p1() == p2())
        return;
    double r = pen.width()*0.00375;
    painter->save();
    painter->setPen(pen);
    painter->setBrush(QBrush(pen.color()));
    painter->setOpacity(opacity);
    painter->drawLine(QLineF(p1().x, p1().y, p2().x, p2().y));
    if (seenP1)
        painter->drawEllipse(p1(), r, r);
    if (seenP2)
        painter->drawEllipse(p2(), r, r);
    painter->restore();
}

// Maps the TrackedLine l from the coordinate frame of Pose2D p to world coordinates.
TrackedLine operator+(const TrackedLine& l, const Pose2D& p)
{
    TrackedLine c = l;
    c += p;
    return c;
}

// Maps the TrackedLine l into the coordinate frame of the Pose2D p.
TrackedLine operator-(const TrackedLine& l, const Pose2D& p)
{
    TrackedLine c = l;
    c -= p;
    return c;
}

// Maps the TrackedLine l from the coordinate frame of Pose2D p to world coordinates.
void operator+=(TrackedLine& l, const Pose2D& p)
{
    l.rotate(p.z);
    l.translate(p.x, p.y);
    l.sort();
}

// Maps the TrackedLine l into the coordinate frame of the Pose2D p.
void operator-=(TrackedLine& l, const Pose2D& p)
{
    l.translate(-p.x, -p.y);
    l.rotate(-p.z);
    l.sort();
}

Vector<TrackedLine> operator+(const Vector<TrackedLine> &v, const Pose2D &p)
{
    thread_local Vector<TrackedLine> tmp;
    tmp.resize(v.size());
    for (uint i = 0; i < v.size(); ++i)
        tmp[i] = v[i] + p;
    return tmp;
}

Vector<TrackedLine> operator-(const Vector<TrackedLine> &v, const Pose2D &p)
{
    thread_local Vector<TrackedLine> tmp;
    tmp.resize(v.size());
    for (uint i = 0; i < v.size(); ++i)
        tmp[i] = v[i] - p;
    return tmp;
}

void operator+=(Vector<TrackedLine> &v, const Pose2D &p)
{
    for (uint i = 0; i < v.size(); ++i)
        v[i] += p;
}

void operator-=(Vector<TrackedLine> &v, const Pose2D &p)
{
    for (uint i = 0; i < v.size(); ++i)
        v[i] -= p;
}

LinkedList<TrackedLine> operator+(const LinkedList<TrackedLine> &v, const Pose2D &p)
{
    thread_local LinkedList<TrackedLine> tmp;
    tmp.clear();
    ListIterator<TrackedLine> it = v.begin();
    while (it.hasNext())
        tmp << it.next()+p;
    return tmp;
}

LinkedList<TrackedLine> operator-(const LinkedList<TrackedLine> &v, const Pose2D &p)
{
    thread_local LinkedList<TrackedLine> tmp;
    tmp.clear();
    ListIterator<TrackedLine> it = v.begin();
    while (it.hasNext())
        tmp << it.next()-p;
    return tmp;
}

void operator+=(LinkedList<TrackedLine> &v, const Pose2D &p)
{
    ListIterator<TrackedLine> it = v.begin();
    while (it.hasNext())
    {
        TrackedLine& l = it.next();
        l += p;
    }
}

void operator-=(LinkedList<TrackedLine> &v, const Pose2D &p)
{
    ListIterator<TrackedLine> it = v.begin();
    while (it.hasNext())
    {
        TrackedLine& l = it.next();
        l -= p;
    }
}

// Writes the line into a data stream.
void TrackedLine::streamOut(QDataStream &out) const
{
    Line::streamOut(out);
    out << seenP1;
    out << seenP2;
    out << totalWeight;
    out << observationCount;
}

// Reads the line from a data stream.
void TrackedLine::streamIn(QDataStream &in)
{
    Line::streamIn(in);
    in >> seenP1;
    in >> seenP2;
    in >> totalWeight;
    in >> observationCount;
}

QDebug operator<<(QDebug dbg, const TrackedLine &n)
{
    dbg.setAutoInsertSpaces(false);
    dbg << n.id << (n.isBlockingLine()?"b":"s")
        << " age: (" << n.observationCount << ", " << n.lastSeen << ")"
        << " [" << n.x1() << ", " << n.y1() << "] to [" << n.x2() << ", " << n.y2() <<"] length: " << n.length() << " observers: " << n.getObserverNodes().size() << " active: " << n.isActive() << " ";
    dbg.setAutoInsertSpaces(true);
    return dbg;
}

QDebug operator<<(QDebug dbg, const TrackedLine* n)
{
    dbg.setAutoInsertSpaces(false);
    dbg << n->id << (n->isBlockingLine()?"b":"s")
        << " age: (" << n->observationCount << ", " << n->lastSeen << ")"
        << " [" << n->x1() << ", " << n->y1() << "] to [" << n->x2() << ", " << n->y2() <<"] length: " << n->length() << " observers: " << n->getObserverNodes().size() << " active: " << n->isActive() << " ";
    dbg.setAutoInsertSpaces(true);
    return dbg;
}

QDataStream& operator<<(QDataStream& out, const TrackedLine &o)
{
    o.streamOut(out);
    return out;
}

QDataStream& operator>>(QDataStream& in, TrackedLine &o)
{
    o.streamIn(in);
    return in;
}

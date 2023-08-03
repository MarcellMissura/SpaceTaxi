#include "LinePair.h"
#include "lib/util/GLlib.h"
#include "lib/util/ColorUtil.h"

LinePair::LinePair()
{
    id = 0;
    sortField = 0;
    confirmed = false;
}

// Returns the prospected line, i.e. the input line mapped to world coordinates using the inputPose.
TrackedLine LinePair::prospectedLine() const
{
    return *inputLine+inputPose;
}

// Returns the prospected line that has been rotated to the angle of the map line.
TrackedLine LinePair::prospectedLineRotated() const
{
    return *inputLine+inputPose.turned(angleDiff());
}

// Returns the angle from the input line to the map line.
// The returned angle is always between -PI2 and PI2.
double LinePair::angleDiff() const
{
    return pihalfcut(mapLine->angle()-(inputLine->angle()+inputPose.z));
}

// Returns a percentual overlap measure between the prospected line and the map line.
// How much percent of the input line (prospected line) is covered by the map line?
double LinePair::percentualOverlap() const
{
    double overlap = mapLine->projectionOverlap(prospectedLine());
    return overlap/inputLine->length();
}

// Returns a Pose2D transform relative to inputPose that matches the input line with the map line.
// The transform is computed by first rotating around the input pose by the angle from the input line to the map line,
// and then translating along the normal of the map line by the orthogonal distance. Note that this is not
// necessarily an exact transformation as the translation parallel to the map line remains unknown.
Pose2D LinePair::tr() const
{
    double rot = angleDiff();
    Vec2 trans = -mapLine->normal()*mapLine->orthogonalDistance((*inputLine+inputPose.turned(rot)).p1());
    return Pose2D(trans, rot);
}

// Returns the weight of the line pair.
// The length of the shorter of the input line and the paired map line determines how much a pair is trusted.
double LinePair::weight() const
{
    return min(inputLine->length(), mapLine->length());
}

// Returns the orthogonal distance between the center point of the prospected line
// and the map line. The sign of the orthogonalDistance is positive when the point
// is on the left of the map line line and negative when the point is on the right
// of the map line with respect to its p1 to p2 direction.
double LinePair::ortho() const
{
    return mapLine->orthogonalDistance(inputLine->center()+inputPose);
}

// Returns the overlap between the map line and the prospected input line.
double LinePair::overlap() const
{
    return mapLine->projectionOverlap(prospectedLine());
}

// Returns the projection distance of the prospected line onto the map line.
// The projection distance is the sum of the absolute orthogonal
// distances of the end points of the prospected line to the map line.
double LinePair::projectionDistance() const
{
    return mapLine->projectionDistance(prospectedLine());
}

// Returns the line-line-distance metric from the prospected line to the mapLine.
double LinePair::lineLineDist() const
{
    return mapLine->lineLineDist(prospectedLine());
}

// Returns the line-pose-distance metric from the prospected line to the map line.
double LinePair::linePoseDist() const
{
    return mapLine->linePoseDist(*inputLine, inputPose);
}

// Draws the line pair in OpenGL context.
void LinePair::draw() const
{
    // The line pairs in blue and black.
    // The input lines are shown in blue.
    // The map lines are shown in black.
    // Draws also the rotatedInputLine and lines to the target.
    //glTranslated(0,0,0.001);

    glLineWidth(5);

    // The input line in blue with a circle on the seen corners.
    glColor3f(0,0,0.8); // blue
    glPushMatrix();
    glMultMatrixd(inputPose.getMatrix());
    inputLine->draw();
    if (inputLine->seenP1)
        GLlib::drawFilledCircle(inputLine->p1(), drawUtil.brushBlue.color(), 0.015);
    if (inputLine->seenP2)
        GLlib::drawFilledCircle(inputLine->p2(), drawUtil.brushBlue.color(), 0.015);
    glPopMatrix();

    // The map line in black with a circle on the seen corners.
    glColor3f(0,0,0); // black
    mapLine->draw();
    if (mapLine->seenP1)
        GLlib::drawFilledCircle(mapLine->p1(), drawUtil.brush.color(), 0.015);
    if (mapLine->seenP2)
        GLlib::drawFilledCircle(mapLine->p2(), drawUtil.brush.color(), 0.015);

    // Stippled rotated input line.
    glLineWidth(2);
    glDisable(GL_LINE_SMOOTH);
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(2, 0xAAAA);
    glColor3f(0.4,0.4,0.6); // blue grey
    prospectedLineRotated().draw();
    glEnable(GL_LINE_SMOOTH);
    glDisable(GL_LINE_STIPPLE);

    // Connect the corresponding end points of the rotated input line and map lines in thin grey.
    // Seen vertex pairs are additionally visualized by arrows.
    glLineWidth(1);
    glColor3f(0.4,0.4,0.4);
    Line drawLine;
    const Line& l1 = prospectedLineRotated();
    const TrackedLine& l2 = *mapLine;
    drawLine.set(l1.p1(), l2.p1());
    drawLine.draw();
    drawLine.set(l1.p2(), l2.p2());
    drawLine.draw();
    glLineWidth(3);
    glColor3f(0.1, 0.1, 0.1);
    if (inputLine->seenP1 && l2.seenP1)
        GLlib::drawLineArrow(Pose2D(l2.p1(), (l2.p1()-l1.p1()).angle()), drawUtil.brush.color(), (l2.p1()-l1.p1()).norm());
    if (inputLine->seenP2 && l2.seenP2)
        GLlib::drawLineArrow(Pose2D(l2.p2(), (l2.p2()-l1.p2()).angle()), drawUtil.brush.color(), (l2.p2()-l1.p2()).norm());
}

void LinePair::drawQuick() const
{
    // Connect the corresponding end points of the input line and the map line in thin grey.
    glLineWidth(1);
    glColor3f(0.4,0.4,0.4);
    Line drawLine;
    Line l1 = *inputLine+inputPose;
    const TrackedLine& l2 = *mapLine;
    drawLine.set(l1.p1(), l2.p1());
    drawLine.draw();
    drawLine.set(l1.p2(), l2.p2());
    drawLine.draw();
}

QDebug operator<<(QDebug dbg, const LinePair &n)
{
    dbg //<< "id:" << n.id
        << n.inputLine->id
        << n.mapLine->id
        << "weight:" << n.weight()
        << "angleDiff:" << n.angleDiff()
        << "ortho:" << n.ortho()
        << "overlap:" << n.percentualOverlap()
        << "lpd:" << n.linePoseDist()
        << "lld:" << n.lineLineDist();
        //<< "tr:" << n.tr();
    return dbg;
}

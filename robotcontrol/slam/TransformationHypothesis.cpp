#include "TransformationHypothesis.h"
#include "blackboard/Config.h"

TransformationHypothesis::TransformationHypothesis()
{
    id = 0;
    linePair1 = 0;
    linePair2 = 0;
    weight = 0;
    votes = 0;
    isParallel = false;
}

// Given two input line - map line pairs i and j, this function computes the uniquely defined transformation
// that maps the input lines of both pairs onto the respective map lines. The function returns false if
// an invalid pair of line pairs was provided, i.e., if the pairs do not agree on the rotation, or if
// after the transformation the overlap is not at least 75%. If all goes well, the function returns
// true and the computed transformation is written into this TransformationHypothesis object.
bool TransformationHypothesis::computeTransform(LinePair& lpi_, LinePair &lpj_, bool debug)
{
    LinePair lpi = lpi_;
    LinePair lpj = lpj_;
    isParallel = false;
    tr_.setZero();

    // Input lines must be unique.
    if  (lpi_.inputLine->id == lpj_.inputLine->id)
    {
        if (debug)
        qDebug() << "Pairs" << lpi_.inputLine->id << lpi_.mapLine->id << "and" << lpj_.inputLine->id << lpj_.mapLine->id
                 << "discarded due to ambiguity.";
        return false;
    }

    double adi = lpi_.angleDiff();
    double adj = lpj_.angleDiff();

    // Pairs of line pairs must agree on their angle diff, otherwise they cannot create a reasonable transformation hypothesis.
    if  (fabs(pihalfcut(adi - adj)) > config.slamClusteringAngleEps)
    {
        if (debug)
        qDebug() << "Pairs" << lpi_.inputLine->id << lpi_.mapLine->id << "and" << lpj_.inputLine->id << lpj_.mapLine->id
                 << "discarded due to angleDiff" << lpi_.angleDiff() << lpj_.angleDiff() << fabs(pihalfcut(lpi.angleDiff() - lpj_.angleDiff()));
        return false;
    }

    double wi = lpi_.weight();
    double wj = lpj_.weight();

    // For the rotation hypothesis, take the weighted average of the two angleDiffs.
    // Except sometimes that leads to a wrong result and a rotation of PI has to be added.
    //double rot = (wi*lpi_.angleDiff() + wj*lpj_.angleDiff()) / (wi+wj);
    double frac = wj/(wi+wj);
    double rot = pihalfcut(adi + frac*pihalfcut(adj-adi));
    lpi.inputPose.turn(rot);
    lpj.inputPose.turn(rot);

    // Detect and correct false rotation.
    Vec2 v1 = lpj_.mapLine->center()-lpi_.mapLine->center();
    Vec2 v2 = (lpj_.inputLine->center()-lpi_.inputLine->center()).frotated(lpi.inputPose.orientation());
    if (v1 * v2 < 0)
    {
        rot += PI;
        lpi.inputPose.turn(PI);
        lpj.inputPose.turn(PI);
    }

    // Now compute the translation needed to match the rotated input lines with their respective map lines.
    Vec2 trans;

    // If the map lines of both pairs are almost parallel, the pairs can only create a half decent transformation hypothesis.
    if (fabs(pihalfcut(lpi_.mapLine->angle() - lpj_.mapLine->angle())) < config.slamClusteringAngleEps)
    {
        // In the parallel map line case, the pairs have to agree on their ortho, or we can already discard them.
        if (fabs(lpi.ortho() - lpj.ortho()) > config.slamClusteringOrthoEps)
        {
            if (debug)
            qDebug() << "Pairs" << lpi_.inputLine->id << lpi_.mapLine->id << "and" << lpj_.inputLine->id << lpj_.mapLine->id
                     << "have parallel map lines but different orthos. Discarded.";
            return false;
        }

        isParallel = true;
        trans = (wi*lpi.tr().pos() + wj*lpj.tr().pos()) / (wi+wj);
    }
    else
    {
        // If there is a significant enough angle between the map lines, and the rotation of the input lines
        // has already been figured out, the optimal translation can be computed with some vector math.

        Vec2 ni = -lpi_.mapLine->normal()*lpi_.mapLine->orthogonalDistance(lpi_.inputLine->center()+lpi.inputPose);
        Vec2 cc = (lpj_.inputLine->center()+lpj.inputPose) + ni;
        double ai = lpi_.mapLine->a();
        double aj = lpj_.mapLine->a();
        Vec2 p1 = lpj_.mapLine->p1();
        Vec2 z;
        if (lpi_.mapLine->isVertical())
        {
            z.x = cc.x;
            z.y = p1.y + aj*(z.x-p1.x);
        }
        else if (lpj_.mapLine->isVertical())
        {
            z.x = p1.x;
            z.y = cc.y + ai*(z.x-cc.x);
        }
        else
        {
            z.x = (cc.y - ai*cc.x - p1.y + aj*p1.x) / (aj-ai);
            z.y = cc.y + ai*(z.x-cc.x);
        }
        trans = ni+z-cc;

        // Both map lines lines cannot be vertical because that would be a case of parallel map lines.
    }

    // Apply the translation and update the input pose in the line pairs.
    lpi.inputPose.translate(trans);
    lpj.inputPose.translate(trans);

    // Finally, validate the hypothesis based on the overlap after the transformation.
    // A significant portion (ca. 75%) of the input line must be covered. The map line
    // does not need to be covered significantly. There are cases where the robot would
    // look through a door and see only a small portion of a long line in the corridor.
    if (lpi.percentualOverlap() < config.slamPairingMinOverlapPercent || lpj.percentualOverlap() < config.slamPairingMinOverlapPercent)
    {
        if (debug)
        {
            qDebug() << "Pairs" << lpi_.inputLine->id << lpi_.mapLine->id << "and" << lpj_.inputLine->id << lpj_.mapLine->id
                     << "discarded due to insufficient overlap." << lpi.percentualOverlap() << lpj.percentualOverlap();
        }
        return false;
    }

    // Complete the transformation hypothesis.
    weight = wi+wj;
    tr_.set(trans.rotated(-lpi_.inputPose.heading()), rot);
    return true;
}

// Given two input line - map line pairs i and j, this function computes the uniquely defined transformation
// that maps the input lines of both pairs onto the respective map lines. The function returns false if
// an invalid pair of line pairs was provided, i.e., if the pairs do not agree on the rotation, or if
// after the transformation the overlap is not at least 75%. If all goes well, the function returns
// true and the computed transformation is written into this TransformationHypothesis object.
bool TransformationHypothesis::computeTranslation(LinePair& lpi_, LinePair &lpj_, bool debug)
{
    LinePair lpi = lpi_;
    LinePair lpj = lpj_;
    isParallel = false;
    tr_.setZero();

    // Input lines must be unique.
    if  (lpi_.inputLine->id == lpj_.inputLine->id)
    {
        if (debug)
        qDebug() << "Pairs" << lpi_.inputLine->id << lpi_.mapLine->id << "and" << lpj_.inputLine->id << lpj_.mapLine->id
                 << "discarded due to ambiguity.";
        return false;
    }

    double adi = lpi_.angleDiff();
    double adj = lpj_.angleDiff();

    // Pairs of line pairs must agree on their angle diff, otherwise they cannot create a reasonable transformation hypothesis.
    if  (fabs(pihalfcut(adi - adj)) > config.slamClusteringAngleEps)
    {
        if (debug)
        qDebug() << "Pairs" << lpi_.inputLine->id << lpi_.mapLine->id << "and" << lpj_.inputLine->id << lpj_.mapLine->id
                 << "discarded due to angleDiff" << lpi_.angleDiff() << lpj_.angleDiff() << fabs(pihalfcut(lpi.angleDiff() - lpj_.angleDiff()));
        return false;
    }

    double wi = lpi_.weight();
    double wj = lpj_.weight();

    // Now compute the translation needed to match the rotated input lines with their respective map lines.
    Vec2 trans;

    // If the map lines of both pairs are almost parallel, the pairs can only create a half decent transformation hypothesis.
    if (fabs(pihalfcut(lpi_.mapLine->angle() - lpj_.mapLine->angle())) < config.slamClusteringAngleEps)
    {
        // In the parallel map line case, the pairs have to agree on their ortho, or we can already discard them.
        if (fabs(lpi.ortho() - lpj.ortho()) > config.slamClusteringOrthoEps)
        {
            if (debug)
            qDebug() << "Pairs" << lpi_.inputLine->id << lpi_.mapLine->id << "and" << lpj_.inputLine->id << lpj_.mapLine->id
                     << "have parallel map lines but different orthos. Discarded.";
            return false;
        }

        isParallel = true;
        trans = (wi*lpi.tr().pos() + wj*lpj.tr().pos()) / (wi+wj);
    }
    else
    {
        // If there is a significant enough angle between the map lines, and the rotation of the input lines
        // has already been figured out, the optimal translation can be computed with some vector math.

        Vec2 ni = -lpi_.mapLine->normal()*lpi_.mapLine->orthogonalDistance(lpi_.inputLine->center()+lpi.inputPose);
        Vec2 cc = (lpj_.inputLine->center()+lpj.inputPose) + ni;
        double ai = lpi_.mapLine->a();
        double aj = lpj_.mapLine->a();
        Vec2 p1 = lpj_.mapLine->p1();
        Vec2 z;
        if (lpi_.mapLine->isVertical())
        {
            z.x = cc.x;
            z.y = p1.y + aj*(z.x-p1.x);
        }
        else if (lpj_.mapLine->isVertical())
        {
            z.x = p1.x;
            z.y = cc.y + ai*(z.x-cc.x);
        }
        else
        {
            z.x = (cc.y - ai*cc.x - p1.y + aj*p1.x) / (aj-ai);
            z.y = cc.y + ai*(z.x-cc.x);
        }
        trans = ni+z-cc;

        // Both map lines lines cannot be vertical because that would be a case of parallel map lines.
    }

    // Apply the translation and update the input pose in the line pairs.
    lpi.inputPose.translate(trans);
    lpj.inputPose.translate(trans);

    // Finally, validate the hypothesis based on the overlap after the transformation.
    // A significant portion (ca. 75%) of the input line must be covered. The map line
    // does not need to be covered significantly. There are cases where the robot would
    // look through a door and see only a small portion of a long line in the corridor.
    if (lpi.percentualOverlap() < config.slamPairingMinOverlapPercent || lpj.percentualOverlap() < config.slamPairingMinOverlapPercent)
    {
        if (debug)
        {
            qDebug() << "Pairs" << lpi_.inputLine->id << lpi_.mapLine->id << "and" << lpj_.inputLine->id << lpj_.mapLine->id
                     << "discarded due to insufficient overlap." << lpi.percentualOverlap() << lpj.percentualOverlap();
        }
        return false;
    }

    // Complete the transformation hypothesis.
    weight = wi+wj;
    tr_.set(trans.rotated(-lpi_.inputPose.heading()), 0);
    return true;
}

// Returns the distance between two TransformationHypotheses, ie., |x1-x2|+|y2-y2|+|z1-z2|.
double TransformationHypothesis::dist(const TransformationHypothesis& other) const
{
    return tr_.dist(other.tr_);
}

// Returns the transform that this hypothesis suggests as a Pose2D relative to inputPose.
const Pose2D &TransformationHypothesis::tr() const
{
    return tr_;
}

QDebug operator<<(QDebug dbg, const TransformationHypothesis &n)
{
    dbg << n.id << "votes:" << n.votes
        << "pairs:" << n.linePair1->inputLine->id << n.linePair1->mapLine->id << "and"
        << n.linePair2->inputLine->id << n.linePair2->mapLine->id
        << "tr:" << n.tr();
    return dbg;
}

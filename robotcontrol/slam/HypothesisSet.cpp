#include "HypothesisSet.h"

HypothesisSet::HypothesisSet()
{
    _snapQuality = 0;
}

// Clears the hypothesis set to a blank state.
void HypothesisSet::clear()
{
    hyps.clear();
    _avgTransform.setNull();
    _snapQuality = 0;
}

// Returns true when there are no hypotheses in the set.
bool HypothesisSet::isEmpty() const
{
    return hyps.isEmpty();
}

// Returns the size of the hypothesis set.
uint HypothesisSet::size() const
{
    return hyps.size();
}

// Sets the transform hypothesis set to be the one provided by v.
void HypothesisSet::set(const Vector<TransformationHypothesis> v)
{
    hyps = v;
    _avgTransform.setNull();
    _snapQuality = 0;
}

// Returns a refence to the internal structure of the stored hypotheses.
const Vector<TransformationHypothesis> &HypothesisSet::getHyps() const
{
    return hyps;
}

// Evaluates the snap quality of the hypothesis set using the ratio of the covered input length
// and the total input length.
double HypothesisSet::snapQuality(const Vector<TrackedLine> &inputLines, bool debug)
{
    if (_snapQuality != 0)
        return _snapQuality;

    // Build a set of confirmed line pairs that appear in the consensus set.
    // and forge the average tansform of the hyp set into them.
    Vector<LinePair> confirmedLinePairs;
    for (uint i = 0; i < hyps.size(); i++)
    {
        TransformationHypothesis& t = hyps[i];

        if (!t.linePair1->confirmed)
        {
            confirmedLinePairs << *t.linePair1;
            confirmedLinePairs.last().inputPose = avgTransform();
        }
        if (!t.linePair2->confirmed)
        {
            confirmedLinePairs << *t.linePair2;
            confirmedLinePairs.last().inputPose = avgTransform();
        }
        t.linePair1->confirmed = true;
        t.linePair2->confirmed = true;
    }

    // The confirmed flags need to be reset.
    for (uint i = 0; i < hyps.size(); i++)
    {
        TransformationHypothesis& t = hyps[i];
        t.linePair1->confirmed = false;
        t.linePair2->confirmed = false;
    }

    // Evaluate the quality of the snap.
    // The snap quality is expressed as successfully overlapped input length divided by the total input length.
    // The snap quality approaches 1 if most input lines were part of a hypothesis in the consensus set.
    double totalInputLength = 0;
    for (uint i = 0; i < inputLines.size(); i++)
        totalInputLength += inputLines[i].length();
    double confirmedLength = 0;
    for (uint i = 0; i < confirmedLinePairs.size(); i++)
    {
        if (debug)
            qDebug() << confirmedLinePairs[i] << "contributes" << confirmedLinePairs[i].overlap();
        confirmedLength += confirmedLinePairs[i].overlap();
    }

    _snapQuality = confirmedLength/totalInputLength;

    if (debug)
    {
        qDebug() << "Input lines:" << &inputLines;
        qDebug() << "Confirmed global line pairs:" << &confirmedLinePairs;
        qDebug() << "Avg pose:" << avgTransform();
        qDebug() << "Snap quality:" << _snapQuality
                 << "total length:" << totalInputLength
                 << "confirmed length:" << confirmedLength;
    }

    return _snapQuality;
}

// Evaluates the snap quality of the hypothesis set using the ratio of the covered input length
// and the total input length.
double HypothesisSet::getSnapQuality() const
{
    return _snapQuality;
}

// Computes the weighted average Pose2D transform of the hypothesis set.
const Pose2D& HypothesisSet::avgTransform(bool recompute) const
{
    if (!recompute && !_avgTransform.isNull())
        return _avgTransform;

    // The average is computed using a sliding average algorithm in order to accomodate
    // the picut issue with the angles.
    // a0 = w0*x0 / w0
    // a1 = w0*a0 + w1*(a0+(x1-a0)) / (w0+w1)
    // a2 = (w0+w1)*a1 + w2*(a1+(x1-a1)) / (w0+w1+w2)
    // ai = (s_i-1 * a_i-1 + wi*(a_i-1 + (xi-a_i-1))) / si

    double totalWeight = 0;
    _avgTransform.setNull();
    for (uint i = 0; i < hyps.size(); i++)
    {
        const TransformationHypothesis& t = hyps[i];
        _avgTransform.x = totalWeight*_avgTransform.x + t.weight*(_avgTransform.x + (t.tr().x - _avgTransform.x));
        _avgTransform.y = totalWeight*_avgTransform.y + t.weight*(_avgTransform.y + (t.tr().y - _avgTransform.y));
        _avgTransform.z = totalWeight*_avgTransform.z + t.weight*(_avgTransform.z + ffpicut(t.tr().z - _avgTransform.z));
        totalWeight += t.weight;
        _avgTransform /= totalWeight;
    }

    return _avgTransform;
}

// Returns the confirmed line pairs in this hypothesis set.
Vector<LinePair> HypothesisSet::getConfirmedLinePairs()
{
    Vector<LinePair> confirmedLinePairs;
    for (uint i = 0; i < hyps.size(); i++)
    {
        TransformationHypothesis& t = hyps[i];

        if (!t.linePair1->confirmed)
        {
            confirmedLinePairs << *t.linePair1;
            confirmedLinePairs.last().inputPose = avgTransform();
        }
        if (!t.linePair2->confirmed)
        {
            confirmedLinePairs << *t.linePair2;
            confirmedLinePairs.last().inputPose = avgTransform();
        }
        t.linePair1->confirmed = true;
        t.linePair2->confirmed = true;
    }

    // The confirmed flags need to be reset.
    for (uint i = 0; i < hyps.size(); i++)
    {
        TransformationHypothesis& t = hyps[i];
        t.linePair1->confirmed = false;
        t.linePair2->confirmed = false;
    }

    return confirmedLinePairs;
}

// Returns true if the hypothesis tr is in conflict with any hypothesis in the set.
bool HypothesisSet::isInConflictWith(const TransformationHypothesis &tr) const
{
    for (uint k = 0; k < hyps.size(); k++)
        if (hyps[k].isInConflictWith(tr))
            return true;
    return false;
}

// Reduces the hypotheses to the largest set of transformations that are in consens with each other with
// respect to the dist between the transformations and the neighbourhood threshold eps. In other words,
// outliers are discarded that are far away from the agreeing majority of the hypotheses set.
void HypothesisSet::discardOutliers(double eps, bool debug)
{
    // The consensus set is computed with a voting algorithm. For every hypothesis in the set,
    // we count all voters that are less than eps distance away. Finally, we determine the hyp(s)
    // that have the most voters and gather their voters to form the consensus set.
    uint mostVotes = 0;
    thread_local Vector<uint> mostVotesIdx;
    mostVotesIdx.clear();

    // Reset all votes to zero.
    for (uint i = 0; i < hyps.size(); i++)
        hyps[i].votes = 0;

    // Perform the voting.
    for (uint i = 0; i < hyps.size(); i++)
    {
        for (uint j = i+1; j < hyps.size(); j++)
        {
            if (false && debug)
                qDebug() << "hyps" << i << j << "dist:" << hyps[i].dist(hyps[j]);

            if (hyps[i].dist(hyps[j]) < eps)
            {
                if (false && debug)
                    qDebug() << "hyps" << i << j << "vote for each other. dist:" << hyps[i].dist(hyps[j]);

                hyps[i].votes++;
                hyps[j].votes++;
            }
        }
    }

    // Extract the most votes.
    for (uint i = 0; i < hyps.size(); i++)
    {
        if (hyps[i].votes > mostVotes)
        {
            mostVotes = hyps[i].votes;
            mostVotesIdx.clear();
        }

        if (hyps[i].votes >= mostVotes)
        {
            mostVotesIdx << i;
        }
    }

    if (debug)
    {
        qDebug() << "Hyp consensus:";
        for (uint i = 0; i < hyps.size(); i++)
            qDebug() << i << hyps[i].votes << "hyp:" << hyps[i];
        qDebug() << "Most popular:" << mostVotesIdx;
    }

    // Collect the indices of all the voters.
    uint ms = mostVotesIdx.size();
    char closed[hyps.size()];
    memset(closed, 0, hyps.size());
    for (uint k = 0; k < ms; k++)
        closed[mostVotesIdx[k]] = 1;
    for (uint k = 0; k < ms; k++)
    {
        uint i = mostVotesIdx[k];
        for (uint j = 0; j < hyps.size(); j++)
        {
            if (closed[j] == 1)
                continue;

            if (hyps[i].dist(hyps[j]) < eps)
            {
                mostVotesIdx << j;
                closed[j] = 1;
            }
        }
    }

    thread_local Vector<TransformationHypothesis> consensusSet;
    consensusSet.clear();
    for (uint i = 0; i < mostVotesIdx.size(); i++)
        consensusSet << hyps[mostVotesIdx[i]];

    // Overwrite the hypothesis set with the consensus set.
    hyps = consensusSet;

    return;
}

// Clusters the hypothesis set with the DBScan algorithm using eps as the distance threshold.
// A Vector of HypothesisSets is returned, one set for each cluster.
const Vector<HypothesisSet>& HypothesisSet::cluster(double eps, bool debug) const
{
    bool clustered[hyps.size()];
    for (uint i = 0; i < hyps.size(); i++)
        clustered[i] = false;

    static Vector<uint> N; // neighborhood queue
    static HypothesisSet cluster; // current cluster in the making
    static Vector<HypothesisSet> clusters; // resulting clusters
    N.clear();
    cluster.clear();
    clusters.clear();
    for (uint i = 0; i < hyps.size(); i++)
    {
        if (clustered[i])
            continue;

        // Begin a cluster with item i.
        cluster.clear();
        cluster << hyps[i];
        clustered[i] = true;
        if (debug)
            qDebug() << "Begin cluster" << clusters.size() << "with" << i << hyps[i];

        // Queue all item i's eps neighbors (except itself) into the neighborhood queue.
        N.clear();
        for (uint j = i+1; j < hyps.size(); j++)
        {
            if (!clustered[j] && hyps[j].dist(hyps[i]) < eps)
            {
                N << j;
                if (debug)
                    qDebug() << " Queueing direct neighbor" << j << hyps[j] << "dist:" << hyps[j].dist(hyps[i]);
            }
            else if (debug)
                qDebug() << " Skipping direct neighbor" << j << hyps[j] << "dist:" << hyps[j].dist(hyps[i]);
        }

        // Work through the neighborhood queue and keep queueing all epsilon-reachable neighbors of neighbors.
        for (uint k = 0; k < N.size(); k++)
        {
            if (clustered[N[k]])
                continue;

            clustered[N[k]] = true;
            cluster << hyps[N[k]];
            if (debug)
                qDebug() << "  Adding neighbor" << N[k] << hyps[N[k]] << "dist:" << hyps[N[k]].dist(hyps[i]);
            for (uint j = 0; j < hyps.size(); j++)
            {
                if (!clustered[j] && hyps[j].dist(hyps[N[k]]) < eps)
                {
                    N << j;
                    if (debug)
                        qDebug() << "   Neighbor" << j << hyps[j] << "is reachable from" << N[k] << hyps[N[k]] << "dist:" << hyps[j].dist(hyps[N[k]]);
                }
                else if (debug)
                    qDebug() << "  Skipping neighbor" << j << hyps[j] << "dist:" << hyps[j].dist(hyps[N[k]]);
            }
        }

        clusters << cluster;
    }

    return clusters;
}

// Remove hypothesis conflicts from the cluster by splitting it into conflict-free subclusters.
// Two hypotheses are in conflict when the same input line is assigned to different map lines.
const Vector<HypothesisSet> &HypothesisSet::splitConflictFreeSubClusters(bool debug) const
{
    thread_local Vector<HypothesisSet> splitHypsClusters;
    splitHypsClusters.clear();
    thread_local HypothesisSet resultCluster;

    // Extract sub clusters.
    // This algorithm is based on an adjacency matrix that never really needs to be computed.
    for (uint i = 0; i < hyps.size(); i++)
    {
        resultCluster.clear();
        for (uint j = 0; j < hyps.size(); j++)
        {
            if (debug)
                qDebug() << "Testing" << i << j << "hyps" << hyps[i].id << hyps[j].id;
            if (hyps[j].isInConflictWith(hyps[i]))
                continue;

            if (!resultCluster.isInConflictWith(hyps[j]))
            {
                if (debug)
                    qDebug() << "Pushing" << hyps[j].id;
                resultCluster << hyps[j];
            }
        }

        splitHypsClusters << resultCluster;
    }

    // Prune identical clusters.
    if (debug)
        qDebug() << "Pruning identical clusters." << splitHypsClusters.size();
    for (int i = splitHypsClusters.size()-1; i >= 0; i--)
    {
        for (int j = i-1; j >= 0; j--)
        {
            if (splitHypsClusters[i] == splitHypsClusters[j])
            {
                if (debug)
                    qDebug() << i << j << "clusters are equal" << splitHypsClusters[i].size();
                splitHypsClusters.remove(j);
                break;
            }
        }
    }

    return splitHypsClusters;
}

// Draws the LineMap on a QPainter.
void HypothesisSet::draw(QPainter *painter) const
{

}

// Appends a transform hypothesis to the set.
HypothesisSet& HypothesisSet::operator<<(const TransformationHypothesis &th)
{
    hyps << th;
    return *this;
}

// Appends a transform hypothesis to the set.
HypothesisSet& HypothesisSet::operator<<(const Vector<TransformationHypothesis> &th)
{
    hyps << th;
    return *this;
}

// Draws the HypothesisSet in OpenGL context.
void HypothesisSet::draw() const
{

}

QDebug operator<<(QDebug dbg, const HypothesisSet &o)
{
    dbg << "size:" << o.size()
             << "transform:" << o.avgTransform()
             << "quality:" << o.getSnapQuality();
    dbg << &o.getHyps();
    return dbg;
}

QDebug operator<<(QDebug dbg, const HypothesisSet *o)
{
    dbg << "size:" << o->size()
             << "transform:" << o->avgTransform()
             << "quality:" << o->getSnapQuality();
    return dbg;
}

#ifndef AGENT_H_
#define AGENT_H_
#include "geometry/GeometricModel.h"
#include "geometry/GridModel.h"

class Agent
{
public:

    int agentId;
    Hpm2D mainTarget;
    Hpm2D intermediateTarget;

    GeometricModel geometricModel;
    GridModel gridModel;

    Agent();
    ~Agent(){}

public:

    virtual void step();
    virtual void sense();
    virtual void act();

    void setAgentId(int agentId);
    int getAgentId() const;

    void staticSense();
    void cheatSense();
};

#endif

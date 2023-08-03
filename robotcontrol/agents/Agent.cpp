#include "Agent.h"
#include "util/StopWatch.h"
#include "blackboard/State.h"
#include "blackboard/Command.h"

// The Agent is the base class for HolonomicAgents and UnicycleAgents.
// By inheriting Agent, the same sense() method is used by all agents
// while they all implement their motoion specific act() method.

Agent::Agent()
{
    agentId = 0;
}

// Robot control step.
void Agent::step()
{
    StopWatch sw;
    sw.start();
    sense();
    state.senseTime = sw.elapsedTimeMs();
    sw.start();
    act();
    state.actTime = sw.elapsedTimeMs();
}

// Update the world model.
void Agent::sense()
{
    // The sensing pipleline prepares the grid model and the geometic model by (partially) simulating
    // the perception algorithm the real robot would execute. You have a choice between a
    // staticSense() method, which is the most conservative in terms of using operations that are
    // available on a real robot, but also gives the least information, and a cheatSense() method
    // that cuts corners to provide most information by copying them out of the world.

    // In each sense, a grid model is computed, which is an occupancy map of the present obstacles,
    // and a geometric model, which is a polygonal representation of the world.
    // The polygons in the world are used as source. First, an occupancy map is computed from the
    // world polygons. Then, by means of contour detection, segments of occupied cells are converted
    // to polygons.

    // Growing is done in a way that the occupancy grid is dilated before segmentation. This is much
    // easier, than implementing a safe grow operation for non-convex obstacles that after the growth
    // could overlap and self-intersect. I have made good experiences with blurring the occupancy map
    // to create a costmap to be used by DWA as an obstacle proximity function.

    if (command.unpredict)
        staticSense();
    else
        cheatSense();

    // A couple of things that need to be done, otherwise nothing works.
    geometricModel.transform();
    geometricModel.generateLineSegments();
}

// The static sense method is most basic. It computes an occupancy grid from all obstacles in
// the world, dilates by the expansion radius, and extracts polygons using contour detection.
// The grid is then blurred for the DWA.
void Agent::staticSense()
{
    gridModel.computeOccupancyGrid(state.world.getAllObstacles(agentId)); // 43
    gridModel.dilate(config.gridDilationRadius); // 32
    geometricModel.setFromGrid(gridModel); // 18
    gridModel.blur(config.gridBlurRadius);
    gridModel.blockBorder();
}

// The cheat sense processes static and dynamic obstacles in different ways.
// The static obstacles are occupancy mapped, dilated, and extracted using countour detection.
// The dynamic obstacles are copied out of the world (with velocity) and grown.
void Agent::cheatSense()
{
    // Grid model computed only using the static obstacles in the world.
    gridModel.computeOccupancyGrid(state.world.getStaticObstacles());
    gridModel.dilate(config.gridDilationRadius);

    // At least the polygons of the static obstacles are extracted from
    // the grid model using contour detection.
    Vector<Obstacle> staticObstacles = gridModel.extractPolygons();

    gridModel.blur(config.gridBlurRadius);
    gridModel.blockBorder();

    // Add the static obstacles gained from the grid to the geometric model.
    geometricModel.clear();
    for (int i = 0; i < staticObstacles.size(); i++)
        geometricModel.addStaticObstacle(staticObstacles[i]);

    // The cheat: import the dynamic obstacles from the world by grow()-ing
    // their polygons and copying their velocities.
    Vector<const UnicycleAgent*> unicycleAgents = state.world.getUnicycleAgents(agentId);
    if (command.ghostMode && agentId > 0)
    {
        for (int i = 0; i < unicycleAgents.size(); i++)
        {
            if (unicycleAgents[i]->getAgentId() == 0)
            {
                HolonomicObstacle o = *unicycleAgents[i];
                if (command.unpredict)
                    o.setVel(0,0);
                o.grow(config.gmPolygonExpansionMargin);
                geometricModel.addHolonomicObstacle(o);
            }
        }
    }
    else
    {
        Vector<const UnicycleAgent*> unicycleAgents = state.world.getUnicycleAgents(agentId);
        for (int i = 0; i < unicycleAgents.size(); i++)
        {
            HolonomicObstacle o = *unicycleAgents[i];
            if (command.unpredict)
                o.setVel(0,0);
            o.grow(config.gmPolygonExpansionMargin);
            geometricModel.addHolonomicObstacle(o);
        }
    }
}

// Compute an action. This method results in the acceleration of the taxi
// (which are the controls) being set in the x and y directions.
void Agent::act()
{

}

void Agent::setAgentId(int agentId)
{
    this->agentId = agentId;
}

int Agent::getAgentId() const
{
    return agentId;
}

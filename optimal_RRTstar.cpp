#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/geometric/planners/rrt/RRTstar.h>
// #include <ompl/geometric/planners/rrt/RRT.h>
#include "myRRT.h"
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <iostream>
#include <fstream>

// g++ -I/home/tsoyarty/Desktop/Bc_work/omplapp-1.6.0-Source/ompl/src -I/usr/include/eigen3 optimal_RRTstar.cpp -o optimal_RRTstar -lompl

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
    return (const void *)rot != (const void *)pos;
}


// Our collision checker. For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {
        return this->clearance(state) > 0.0;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ob::State* state) const
    {
        const ob::RealVectorStateSpace::StateType* state2D = state->as<ob::RealVectorStateSpace::StateType>();
        double x = state2D->values[0];
        double y = state2D->values[1];
        return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25;
    }
};

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

int main()
{
    // Construct the robot state space in which we're planning. We're
    // planning in [0,1]x[0,1], a subset of R^2.
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

    // Set the bounds of space to be in [0,1].
    space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 1.0);

    // Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));

    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0;

    // Create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Set the optimization objective
    pdef->setOptimizationObjective(getPathLengthObjective(si));

    // Construct our optimizing planner using the RRTstar algorithm.
    // ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));
    // ob::PlannerPtr optimizingPlanner(new og::RRT(si));
    // ob::PlannerPtr optimizingPlanner(new og::RRTConnect(si));
    ob::PlannerPtr optimizingPlanner(new og::RRT(si));

    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // Attempt to solve the planning problem within one second of
    // planning time
    ob::PlannerStatus solved = optimizingPlanner->solve(1.0);
    
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        og::PathGeometric& path = dynamic_cast<og::PathGeometric&>(*pdef->getSolutionPath());
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path.print(std::cout);

        // save the path to a file
        std::ofstream outFile("opt_rrt.txt");
        path.printAsMatrix(outFile);
        outFile.close();
    }
    else    
        std::cout << "Govno" << std::endl;
    
    return 0;
}

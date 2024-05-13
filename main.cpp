#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// #include "myRRT.h"
// #include "RRTXstatic.h"
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>

#include <ompl/geometric/planners/prm/PRM.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#include <iostream>
#include <fstream>

#include <vector>
// g++ -I/home/tsoyarty/Desktop/Bc_work/omplapp-1.6.0-Source/ompl/src -I/usr/include/eigen3 optimal_RRTstar.cpp -o optimal_RRTstar -lompl

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std; 


bool isStateValid(const ob::State *state) //DEMO
{
    // cast the abstract state type to the type we expect
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
 
    // extract the first component of the state and cast it to what we expect
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
 
    // extract the second component of the state and cast it to what we expect
    const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
 
    // check validity of state defined by pos & rot
    
 
    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return (const void*)rot != (const void*)pos;
}


// Our collision checker. For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".

// Define a class for polygonal obstacle representation
class PolygonObstacle {
public:
    PolygonObstacle(const std::vector<std::pair<double, double>>& vertices) : vertices_(vertices) {}

    // Method to check if a point is inside the polygonal obstacle
    bool isInside(double x, double y) const {
        bool inside = false;
        int n = vertices_.size();
        for (int i = 0, j = n - 1; i < n; j = i++) {
            if (((vertices_[i].second > y) != (vertices_[j].second > y)) &&
                (x < (vertices_[j].first - vertices_[i].first) * (y - vertices_[i].second) / (vertices_[j].second - vertices_[i].second) + vertices_[i].first)) {
                inside = !inside;
            }
        }
        return inside;
    }

private:
    std::vector<std::pair<double, double>> vertices_;
};

// Define a validity checker class that handles both circular and polygonal obstacles
class ValidityChecker : public ob::StateValidityChecker {
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si) {}

    // Method to add polygonal obstacles
    void addPolygonObstacle(const std::vector<std::pair<double, double>>& vertices) {
        polygonObstacles_.emplace_back(vertices);
    }

    // Method to add circular obstacles
    void addCircularObstacle(double centerX, double centerY, double radius) {
        circularObstacles_.emplace_back(centerX, centerY, radius);
    }

     // Method to get the number of added obstacles
    size_t getNumObstacles() const {
        return circularObstacles_.size() + polygonObstacles_.size();
    }

    // Method to check if a state is valid (not colliding with any obstacle)
    bool isValid(const ob::State* state) const override {
        return !collidesWithObstacle(state);
    }

private:
    // Method to check if a state collides with any obstacle
    bool collidesWithObstacle(const ob::State* state) const {
        const auto* state2D = state->as<ob::RealVectorStateSpace::StateType>();
        double x = state2D->values[0];
        double y = state2D->values[1];
        // Check collision with circular obstacles
        for (const auto& obstacle : circularObstacles_) {
            double distSq = (x - obstacle.centerX) * (x - obstacle.centerX) +
                            (y - obstacle.centerY) * (y - obstacle.centerY);
            if (distSq <= obstacle.radius * obstacle.radius) { 
                return true;
            }
        }

        // Check collision with polygonal obstacles
        for (const auto& obstacle : polygonObstacles_) {
            if (obstacle.isInside(x, y)) { 
                return true;
            }
        } 
        return false;
    }

    // Structure to represent circular obstacles
    struct CircularObstacle {
        double centerX;
        double centerY;
        double radius;
        CircularObstacle(double x, double y, double r) : centerX(x), centerY(y), radius(r) {}
    };

    std::vector<CircularObstacle> circularObstacles_;
    std::vector<PolygonObstacle> polygonObstacles_;
}; 


ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}



struct Point {
    double x;
    double y;
};

void readFromFile(const std::string& filename, Point& startPoint, Point& endPoint, std::vector<std::vector<Point>>& polygons) {
    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;
        std::getline(file, line); // Read start point
        std::istringstream startStream(line.substr(line.find(":") + 1));
        startStream >> startPoint.x;
        char comma; // to read comma separator
        startStream >> comma >> startPoint.y;

        std::getline(file, line); // Read end point
        std::istringstream endStream(line.substr(line.find(":") + 1));
        endStream >> endPoint.x;
        endStream >> comma >> endPoint.y;

        while (std::getline(file, line)) { // Read polygons
            std::vector<Point> polygon;
            std::istringstream iss(line);
            std::string point;
            while (iss >> point) {
                std::istringstream pointStream(point);
                char comma; // to read comma separator
                Point p;
                pointStream >> p.x;
                pointStream >> comma >> p.y; // Read x and y coordinates
                polygon.push_back(p);
            }
            polygons.push_back(polygon);
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}


int main()
{ 
    // Construct the robot state space in which we're planning. We're
    // planning in [0,1]x[0,1], a subset of R^2.
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));

    // Set the bounds of space to be in [0,1].
    space->as<ob::RealVectorStateSpace>()->setBounds(0.0, 149.0);

    // Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));

    // Create the validity checker
    ValidityChecker validityChecker(si);
    

    // ============================================================================== 
    // validityChecker.addCircularObstacle(75.0, 75.0, 30.0);  
    
    // std::vector<std::pair<double, double>> polygonVertices1 = {{35,50},{35,70},{115,70},{115,50}}; 
    // validityChecker.addPolygonObstacle(polygonVertices1);
    // std::vector<std::pair<double, double>> polygonVertices2 = {{35,50},{35,130},{55,130},{55,50}}; 
    // validityChecker.addPolygonObstacle(polygonVertices2);
    // std::vector<std::pair<double, double>> polygonVertices3 = {{105,50},{105,130},{125,130},{125,50}}; 
    // validityChecker.addPolygonObstacle(polygonVertices3);
    Point startPoint, goalPoint;
    std::vector<std::vector<Point>> polygons;
    readFromFile("Maze_obs", startPoint, goalPoint, polygons);

    // Add polygons to the validity checker and grid
    for (size_t i = 0; i < polygons.size(); ++i) {
        std::vector<std::pair<double, double>> polygonVertices;
        for (const auto& point : polygons[i]) {
            polygonVertices.push_back({point.x, point.y});
        }
        std::cout << "std::vector<std::pair<double, double>> polygonVertices" << i + 1 << " = {";
        for (size_t j = 0; j < polygonVertices.size(); ++j) {
            std::cout << "{" << polygonVertices[j].first << "," << polygonVertices[j].second << "}";
            if (j < polygonVertices.size() - 1) {
                std::cout << ",";
            }
        }
        std::cout << "};\n";

        // Add polygon to validity checker
        validityChecker.addPolygonObstacle(polygonVertices);

        // Add polygon to grid
        // for each point in polygonVertices, set corresponding grid cell to obstacle
    }
    // ==============================================================================

    std::cout << "Number of obstacles: " << validityChecker.getNumObstacles() << std::endl;
    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(&validityChecker));
    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ob::ScopedState<> start(space);
    
    std::cout<<"========================"<<endl;
    std::cout<<startPoint.x<<endl;
    std::cout<<startPoint.y<<endl;

    std::cout<<goalPoint.x<<endl;    
    std::cout<<goalPoint.y<<endl;
    std::cout<<"========================"<<endl;

    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = startPoint.x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = startPoint.y;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goalPoint.x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goalPoint.y;

    // Create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Set the optimization objective
    pdef->setOptimizationObjective(getPathLengthObjective(si));
 

    //==================================================================================================================
    //==================================================================================================================
    //==================================================================================================================

    ob::PlannerPtr optimizingPlanner(new og::RRTXstatic(si));
    
    //==================================================================================================================
    //==================================================================================================================
    //==================================================================================================================

    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // Attempt to solve the planning problem within one second of
    // planning time 
    ob::PlannerStatus solved = optimizingPlanner->solve(1); 
    
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        og::PathGeometric& path = dynamic_cast<og::PathGeometric&>(*pdef->getSolutionPath());
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        // path.print(std::cout);

        // save the path to a file
        std::ofstream outFile("main.txt");
        path.printAsMatrix(outFile);
        outFile.close();
    }
    else    
        std::cout << "Not Found" << std::endl;
    
    return 0;
}

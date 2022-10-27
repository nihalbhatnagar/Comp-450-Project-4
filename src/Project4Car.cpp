///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Parv Shrivastava and Nihal Bhatnagar
//////////////////////////////////////

#include <iostream>
#include <fstream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/base/State.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/extensions/ode/OpenDESimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpacePtr &space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2;
    }

    void project(const ompl::base::State * state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the car
        auto r2 = state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(0)->as<ompl::base::RealVectorStateSpace::StateType>(0);
        projection[0] = r2->values[0];
        projection[1] = r2->values[1];
    }
};

void carODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control * control,
            ompl::control::ODESolver::StateType & qdot)
{
    // TODO: Fill in the ODE for the car's dynamics
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double angularVel = u[0];
    const double forwardAccel = u[1];

    const double heading = q[2];
    const double vel = q[3];
 
    // Ensure qdot is the same size as q.  Zero out all values.
    qdot.resize(q.size(), 0);
 
    // x-dot
    qdot[0] = vel * cos(heading);
    // y-dot
    qdot[1] = vel * sin(heading);

    qdot[2] = angularVel;
    qdot[3] = forwardAccel;
}

void makeStreet(std::vector<Rectangle> & obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.
    Rectangle rect1;
    rect1.x = 2.0;
    rect1.y = 6.0;
    rect1.width = 6.0;
    rect1.height = 2.0;
    obstacles.push_back(rect1);

    Rectangle rect2;
    rect2.x = 5.0;
    rect2.y = 8.0;
    rect2.width = 5.0;
    rect2.height = 5.0;
    obstacles.push_back(rect2);

    Rectangle rect3;
    rect3.x = 3.0;
    rect3.y = 4.0;
    rect3.width = 2.0;
    rect3.height = 3.0;
    obstacles.push_back(rect3);
}

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state, const std::vector<Rectangle> &obstacles){
    return isValidStateSquare(state, 0.25, obstacles) && si->satisfiesBounds(state);
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    namespace ob = ompl::base;
    namespace oc = ompl::control;

    auto se2 = std::make_shared<ob::SE2StateSpace>();

    // setting bounds for heading
    ob::RealVectorBounds se2bounds(2);
    se2bounds.setLow(0);
    se2bounds.setHigh(10);
    se2->setBounds(se2bounds);

    // setting bounds for velocity
    auto r1 = std::make_shared<ob::RealVectorStateSpace>(1);
    ob::RealVectorBounds r1bounds(1);
    r1bounds.setLow(-6);
    r1bounds.setHigh(6);
    r1->setBounds(r1bounds);

    // combining the spaces together and registering as default projection
    ob::StateSpacePtr space;
    space = se2 + r1;
    space->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new CarProjection(space)));

    // make the obstacles
    makeStreet(obstacles);

    // init the control space with angular velocity and forward acceleration
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // setting bounds for acceleration
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-3);
    cbounds.setHigh(3);
    cspace->setBounds(cbounds);

    oc::SimpleSetupPtr ss(new oc::SimpleSetup(cspace));

    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));

    oc::SpaceInformation *si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker([si, obstacles](const ob::State *state) { return isStateValid(si, state, obstacles); });

    // define start state
    ob::ScopedState<> start(space);
    start[0] = 5;
    start[1] = 3;
    start[2] = M_PI / 4;
    start[3] = 2;

    // define goal state
    ob::ScopedState<> goal(space);
    goal[0] = 9;
    goal[1] = 9;
    goal[2] = M_PI / 2;
    goal[3] = 5;

    ss->setStartAndGoalStates(start, goal, 0.05);

    return ss;
}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.
    std::string filePath;
    if (choice == 1) {
        ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
        filePath = "carPathRRT.txt";
    } else if (choice == 2) {
        ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
        filePath = "carPathKPIECE1.txt";
    }

    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(1.0);
    
    if(solved)
    {
        std::cout << "Found solution:" << std::endl;
        ss->getSolutionPath().print(std::cout);
        std::ofstream file(filePath);
        ss->getSolutionPath().asGeometric().printAsMatrix(file);
        file.close();
    }
}

void benchmarkCar(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the car
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}

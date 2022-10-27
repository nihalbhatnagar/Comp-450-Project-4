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
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Converted from state to r2
        auto compound_state = state->as<ompl::base::CompoundState>();

        const ompl::base::RealVectorStateSpace::StateType* r2;
        r2 = compound_state->as<ompl::base::RealVectorStateSpace::StateType>(0);

        projection[0] = r2->values[0];
        projection[1] = r2->values[1];
    }
};

void carODE(const ompl::control::ODESolver::StateType & q, const ompl::control::Control * control,
            ompl::control::ODESolver::StateType & qdot)
{
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

void makeStreet(std::vector<Rectangle> &obstacles)
{
    Rectangle rect1;
    rect1.x = 2.0;
    rect1.y = 6.0;
    rect1.width = 2.0;
    rect1.height = 1.0;
    obstacles.push_back(rect1);
}

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state, const std::vector<Rectangle> &obstacles){
    // Cast the state to a real vector state
    auto compound_state = state->as<ompl::base::CompoundState>();

    const ompl::base::RealVectorStateSpace::StateType* r2;
    r2 = compound_state->as<ompl::base::RealVectorStateSpace::StateType>(0);

    return isValidStatePoint(r2, obstacles) && si->satisfiesBounds(state);
}

void carPostIntegration(const ompl::base::State* /*state*/, const ompl::control::Control * /*control*/, const double /*duration*/, ompl::base::State *result)
{
    ompl::base::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(1));
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> & obstacles)
{
    namespace ob = ompl::base;
    namespace oc = ompl::control;

    // setting bounds
    auto se2 = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds se2bounds(2);
    se2bounds.setLow(0);
    se2bounds.setHigh(11);
    se2->setBounds(se2bounds);

    // setting bounds
    auto r1 = std::make_shared<ob::RealVectorStateSpace>(1);
    ob::RealVectorBounds r1bounds(1);
    r1bounds.setLow(-6);
    r1bounds.setHigh(6);
    r1->setBounds(r1bounds);

    // combining the spaces together and registering as default projection
    ob::StateSpacePtr space;
    space = se2 + r1;
    space->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new CarProjection(space)));

    // init the control space with angular velocity and forward acceleration
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-3);
    cbounds.setHigh(3);
    cspace->setBounds(cbounds);

    oc::SimpleSetupPtr ss(new oc::SimpleSetup(cspace));

    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE));
    ss->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver, &carPostIntegration));
    ss->getSpaceInformation()->setPropagationStepSize(0.05);

    oc::SpaceInformation *si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker([si, &obstacles](const ob::State *state) { return isStateValid(si, state, obstacles); });

    // start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start[0] = 5;
    start[1] = 3;
    start[2] = 0;
    start[3] = 3;

    // goal state
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal[0] = 5;
    goal[1] = 9;
    goal[2] = 0;
    goal[3] = 6;

    ss->setStartAndGoalStates(start, goal, 0.1);

    return ss;
}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
{
    std::string filePath;
    if (choice == 1) {
        ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
        filePath = "carPathRRT.txt";
    } else if (choice == 2) {
        ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
        filePath = "carPathKPIECE1.txt";
    }

    ss->setup();
    ompl::base::PlannerStatus solved = ss->solve(10.0);
    
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

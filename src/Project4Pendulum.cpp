///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Parv Shrivastava and Nihal Bhatnagar
//////////////////////////////////////

#include <iostream>
#include <fstream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/base/State.h>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/extensions/ode/OpenDESimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/spaces/SE2StateSpace.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

#include <cmath>

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpacePtr &space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the pendulum
        auto compound_state = state->as<ompl::base::CompoundState>();

        const ompl::base::RealVectorStateSpace::StateType* r2;
        r2 = compound_state->as<ompl::base::RealVectorStateSpace::StateType>(0);

        projection[0] = r2->values[0];
        projection[1] = r2->values[1];
    }
};

bool isStateValid(const ompl::control::SpaceInformation * /* si */, const ompl::base::State *state){
    const ompl::base::RealVectorStateSpace::StateType* R2State = state->as<ompl::base::RealVectorStateSpace::StateType>();
    // double theta = R2State->values[0];
    double angular = R2State->values[1];
    return abs(angular) <= 10;
}

void pendulumODE(const ompl::control::ODESolver::StateType & q , const ompl::control::Control * control ,
                 ompl::control::ODESolver::StateType & qdot )
{
    // TODO: Fill in the ODE for the pendulum's dynamics
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double torque = u[0];

    const double theta = q[0];
    const double angular = q[1];
 
    // Ensure qdot is the same size as q.  Zero out all values.
    qdot.resize(q.size(), 0);
 
    qdot[0] = angular;          // theta
    qdot[1] = -9.81 * cos(theta) + torque;        // angular
}


ompl::control::SimpleSetupPtr createPendulum(double torque )
{
    namespace ob = ompl::base;
    namespace oc = ompl::control;
    
    // construct the state space we are planning in
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-4);
    bounds.setHigh(4);
    space->setBounds(bounds);
    space->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new PendulumProjection(space)));

    auto cm = std::make_shared<oc::RealVectorControlSpace>(space,1);
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-torque);
    cbounds.setHigh(torque);

    cm->setBounds(cbounds);

    oc::SimpleSetupPtr ss(new oc::SimpleSetup(cm));
    oc::SpaceInformation *si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker(
         [si](const ob::State *state) { return isStateValid(si, state); });
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver));

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
    start[0] = -1.5707;
    start[1] = 0;

    // define goal state
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);
    goal[0] = 1.5707;
    goal[1] = 0;

    ss->setStartAndGoalStates(start, goal,.01);

    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    std::string filePath;
    if (choice == 1) {
        ss->setPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
        filePath = "pendulumPathRRT.txt";
    } else if (choice == 2) {
        ss->setPlanner(std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation()));
        filePath = "pendulumPathKPIECE1.txt";
    } else if (choice == 3) {
        ss->setPlanner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
        filePath = "pendulumPathRGRRT.txt";
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

void benchmarkPendulum(ompl::control::SimpleSetupPtr &ss )
{
    // // // setup benchmarking
    std::string benchmark_name = "Pendulum Benchmark";
    double runtime_limit = 60.0;
    double memory_limit = 1000.0;
    int run_count = 30;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark b(*ss, benchmark_name);

    auto rrt = std::make_shared<ompl::control::RRT>(ss->getSpaceInformation());
    rrt->setName("RRT");

    auto kpiece = std::make_shared<ompl::control::KPIECE1>(ss->getSpaceInformation());
    kpiece->setName("KPIECE");

    auto rgrrt = std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation());
    rgrrt->setName("RG-RRT");

    b.addPlanner(rrt);
    b.addPlanner(kpiece);
    b.addPlanner(rgrrt);

    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

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

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}

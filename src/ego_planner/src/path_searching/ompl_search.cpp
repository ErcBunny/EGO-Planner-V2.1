#include <path_searching/ompl_search.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

OMPLSearch::OMPLSearch(GridMap::Ptr &map) {
    grid_map = map;
    grid_map_res = grid_map->getResolution();
    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
}

ompl::base::PlannerStatus OMPLSearch::searchPath(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos,
                                                 double xy_range, double z_lb, double z_ub, double timeout,
                                                 std::vector<Eigen::Vector3d> &path) {

    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(0, start_pos(0) - xy_range);
    bounds.setHigh(0, start_pos(0) + xy_range);
    bounds.setLow(1, start_pos(1) - xy_range);
    bounds.setHigh(1, start_pos(1) + xy_range);
    bounds.setLow(2, z_lb);
    bounds.setHigh(2, z_ub);
    space->setBounds(bounds);

    auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    si->setStateValidityChecker([this](const ompl::base::State *state) { return isStateValid(state); });
    si->setStateValidityCheckingResolution(0.5 * grid_map_res / xy_range);
    si->setMotionValidator(std::make_shared<ompl::base::DiscreteMotionValidator>(si));
    si->setup();

    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    ompl::base::ScopedState<> start(space), goal(space);
    start[0] = start_pos(0);
    start[1] = start_pos(1);
    start[2] = start_pos(2);
    goal[0] = goal_pos(0);
    goal[1] = goal_pos(1);
    goal[2] = goal_pos(2);
    pdef->setStartAndGoalStates(start, goal);
    pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));

    // TODO: planner algorithm settings?
    auto planner(std::make_shared<ompl::geometric::BFMT>(si));
    planner->setProblemDefinition(pdef);
    planner->setup();

    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(timeout);
    if (solved) {
        path.clear();
        const ompl::geometric::PathGeometric sol = ompl::geometric::PathGeometric(
                dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
        for (size_t i = 0; i < sol.getStateCount(); i++) {
            const auto state = sol.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            path.emplace_back(state[0], state[1], state[2]);
        }
    }

    return solved;
}

bool OMPLSearch::isStateValid(const ompl::base::State *state) {
    const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
    const Eigen::Vector3d position((*pos)[0], (*pos)[1], (*pos)[2]);
    return grid_map->getInflateOccupancy(position) <= 0;
}

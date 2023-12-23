#ifndef SRC_OMPL_SEARCH_H
#define SRC_OMPL_SEARCH_H

#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/util/Console.h>

#include <plan_env/grid_map.h>
#include <traj_utils/planning_visualization.h>

class OMPLSearch {
public:
    explicit OMPLSearch(GridMap::Ptr &map);

    ~OMPLSearch() = default;

    ompl::base::PlannerStatus searchPath(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &goal_pos,
                                         double xy_range, double z_lb, double z_ub, double timeout,
                                         std::vector<Eigen::Vector3d> &path);

private:
    bool isStateValid(const ompl::base::State *state);

    GridMap::Ptr grid_map;
    double grid_map_res;
};

#endif //SRC_OMPL_SEARCH_H

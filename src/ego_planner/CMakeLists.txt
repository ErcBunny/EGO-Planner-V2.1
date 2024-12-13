cmake_minimum_required(VERSION 3.0.2)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        # ros framework
        roscpp
        # messages
        std_msgs
        geometry_msgs
        quadrotor_msgs
        trajectory_msgs
        # handy tools
        cv_bridge
        message_filters
        message_generation
        roslint
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED) # need to find PCL before Boost to avoid ld issues
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES ego_planner
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/../devel/include
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIRS}
        $ENV{CONDA_PREFIX}/include/blasfeo/include
        $ENV{CONDA_PREFIX}/include/hpipm/include
        /usr/local/include/blasfeo/include
        /usr/local/include/hpipm/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ego_planner.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ego_planner_node.cpp)
add_executable(bridge_node_udp src/bridge_node_udp.cpp)
add_executable(drone_detect_node src/drone_detect_node.cpp src/drone_detector/drone_detector.cpp)
add_executable(ego_planner_node
        src/ego_planner_node.cpp
        src/optimizer/poly_traj_optimizer.cpp
        src/path_searching/ompl_search.cpp
        src/path_searching/dyn_a_star.cpp
        src/plan_env/grid_map.cpp
        src/plan_env/raycast.cpp
        src/plan_manage/ego_replan_fsm.cpp
        src/plan_manage/planner_manager.cpp
        src/traj_utils/planning_visualization.cpp
)
add_executable(moving_obstacles src/moving_obstacles.cpp src/traj_utils/planning_visualization.cpp)
add_executable(traj2odom_node src/traj2odom_node.cpp src/optimizer/poly_traj_optimizer.cpp src/path_searching/dyn_a_star.cpp)
add_executable(traj_server
        src/traj_server.cpp
        src/optimizer/poly_traj_optimizer.cpp
        src/path_searching/dyn_a_star.cpp
        include/acados_export_diffdrive/acados_solver_diffdrive.c
        include/acados_export_diffdrive/acados_sim_solver_diffdrive.c
        include/acados_export_diffdrive/diffdrive_constraints/diffdrive_constr_h_e_fun.c
        include/acados_export_diffdrive/diffdrive_constraints/diffdrive_constr_h_e_fun_jac_uxt_zt.c
        include/acados_export_diffdrive/diffdrive_constraints/diffdrive_constr_h_fun.c
        include/acados_export_diffdrive/diffdrive_constraints/diffdrive_constr_h_fun_jac_uxt_zt.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_0_fun.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_0_fun_jac_ut_xt.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_0_hess.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_e_fun.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_e_fun_jac_ut_xt.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_e_hess.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_fun.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_fun_jac_ut_xt.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_hess.c
        include/acados_export_diffdrive/diffdrive_model/diffdrive_expl_ode_fun.c
        include/acados_export_diffdrive/diffdrive_model/diffdrive_expl_vde_adj.c
        include/acados_export_diffdrive/diffdrive_model/diffdrive_expl_vde_forw.c
)
add_executable(main_diffdrive
        include/acados_export_diffdrive/main_diffdrive.c
        include/acados_export_diffdrive/acados_solver_diffdrive.c
        include/acados_export_diffdrive/diffdrive_constraints/diffdrive_constr_h_e_fun.c
        include/acados_export_diffdrive/diffdrive_constraints/diffdrive_constr_h_e_fun_jac_uxt_zt.c
        include/acados_export_diffdrive/diffdrive_constraints/diffdrive_constr_h_fun.c
        include/acados_export_diffdrive/diffdrive_constraints/diffdrive_constr_h_fun_jac_uxt_zt.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_0_fun.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_0_fun_jac_ut_xt.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_0_hess.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_e_fun.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_e_fun_jac_ut_xt.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_e_hess.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_fun.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_fun_jac_ut_xt.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_hess.c
        include/acados_export_diffdrive/diffdrive_model/diffdrive_expl_ode_fun.c
        include/acados_export_diffdrive/diffdrive_model/diffdrive_expl_vde_adj.c
        include/acados_export_diffdrive/diffdrive_model/diffdrive_expl_vde_forw.c
)
add_executable(main_sim_diffdrive
        include/acados_export_diffdrive/main_sim_diffdrive.c
        include/acados_export_diffdrive/acados_sim_solver_diffdrive.c
        include/acados_export_diffdrive/diffdrive_constraints/diffdrive_constr_h_e_fun.c
        include/acados_export_diffdrive/diffdrive_constraints/diffdrive_constr_h_e_fun_jac_uxt_zt.c
        include/acados_export_diffdrive/diffdrive_constraints/diffdrive_constr_h_fun.c
        include/acados_export_diffdrive/diffdrive_constraints/diffdrive_constr_h_fun_jac_uxt_zt.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_0_fun.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_0_fun_jac_ut_xt.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_0_hess.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_e_fun.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_e_fun_jac_ut_xt.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_e_hess.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_fun.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_fun_jac_ut_xt.c
        include/acados_export_diffdrive/diffdrive_cost/diffdrive_cost_y_hess.c
        include/acados_export_diffdrive/diffdrive_model/diffdrive_expl_ode_fun.c
        include/acados_export_diffdrive/diffdrive_model/diffdrive_expl_vde_adj.c
        include/acados_export_diffdrive/diffdrive_model/diffdrive_expl_vde_forw.c
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(bridge_node_udp ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(drone_detect_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(ego_planner_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OMPL_LIBRARIES})
target_link_libraries(moving_obstacles ${catkin_LIBRARIES})
target_link_libraries(traj2odom_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(traj_server ${catkin_LIBRARIES} acados hpipm blasfeo m)
target_link_libraries(main_diffdrive acados hpipm blasfeo m)
target_link_libraries(main_sim_diffdrive acados hpipm blasfeo m)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ego_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

cmake_minimum_required(VERSION 2.8.3)
project(tiago_bartender_mtc)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	actionlib
	actionlib_msgs
	genmsg
	moveit_planners_ompl
	moveit_ros_planning_interface
	moveit_task_constructor_core
	moveit_task_constructor_msgs
	mtc_pour
	roscpp
	pal_interaction_msgs
	tiago_bartender_msgs
)

add_action_files(DIRECTORY action FILES
	PickBottle.action
)

generate_messages(DEPENDENCIES
                  actionlib_msgs
                  geometry_msgs)

catkin_package()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# action server

add_executable(tiago_bartender_mtc_server src/tiago_bartender_mtc_server.cpp)
add_dependencies(tiago_bartender_mtc_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(tiago_bartender_mtc_server ${catkin_LIBRARIES})

add_executable(tiago_bottle_pick_server src/tiago_bottle_pick_server.cpp)
add_dependencies(tiago_bottle_pick_server ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tiago_bottle_pick_server ${catkin_LIBRARIES})

# helpers

# create constraint approximation db
add_executable(tiago_bartender_construct_constraint_database src/construct_constraint_database.cpp)
add_dependencies(tiago_bartender_construct_constraint_database ${catkin_EXPORTED_TARGETS})
target_link_libraries(tiago_bartender_construct_constraint_database ${catkin_LIBRARIES})

# wrapper for MoveIt's FollowJointTrajectory action

add_executable(tiago_gripper_controller src/tiago_gripper_controller.cpp)
add_dependencies(tiago_gripper_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(tiago_gripper_controller ${catkin_LIBRARIES})

# sample executables for individual testing

add_executable(tiago_test_path_constraint src/tiago_test_path_constraint.cpp)
add_dependencies(tiago_test_path_constraint ${catkin_EXPORTED_TARGETS})
target_link_libraries(tiago_test_path_constraint ${catkin_LIBRARIES})

add_executable(tiago_bottle_pick src/tiago_bottle_pick.cpp)
add_dependencies(tiago_bottle_pick ${catkin_EXPORTED_TARGETS})
target_link_libraries(tiago_bottle_pick ${catkin_LIBRARIES})

add_executable(tiago_bottle_pour src/tiago_bottle_pour.cpp)
add_dependencies(tiago_bottle_pour ${catkin_EXPORTED_TARGETS})
target_link_libraries(tiago_bottle_pour ${catkin_LIBRARIES})


cmake_minimum_required(VERSION 2.8.3)

project(robotic_gazebo)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
	CATKIN_DEPENDS
		roslaunch
		rospy
		gazebo_ros
		edy_dualarm_description
		warehouse
)

foreach(dir launch src)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

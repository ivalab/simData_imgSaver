cmake_minimum_required(VERSION 2.8.3)

project(robotic_gazebo)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_package()

find_package(roslaunch)

foreach(dir launch src)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

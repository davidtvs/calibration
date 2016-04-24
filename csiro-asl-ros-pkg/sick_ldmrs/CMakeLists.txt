cmake_minimum_required(VERSION 2.8.3)
project(sick_ldmrs)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  std_msgs
  rospy
  roslib
  rostest
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  driver_base 
)

catkin_python_setup()

catkin_add_nosetests(test/test_xport.py)

add_rostest(test/ldmrs.test)

generate_dynamic_reconfigure_options(cfg/ldmrs.cfg)

# make sure configure headers are built before any node using them
# add_dependencies(example_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

catkin_package(
    DEPENDS std_msgs rospy roslib rostest sensor_msgs std_msgs dynamic_reconfigure driver_base python-numpy
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(PROGRAMS nodes/sickldmrs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

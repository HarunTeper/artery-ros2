set(SOURCES
    BasicRobotLifecycleController.cc
    Core.cc
    RobotObject.cc
    UniformRobotCreationPolicy.cc
    Ros2Node.cc
)

add_library(ros2 SHARED ${SOURCES})
target_include_directories(ros2 PUBLIC $<TARGET_PROPERTY:core,INCLUDE_DIRECTORIES>)
set_property(TARGET ros2 PROPERTY NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})
set_property(TARGET ros2 PROPERTY OMNETPP_LIBRARY ON)

macro(find_ros)
    set(ROS_INCLUDE_DIRS "")
    set(ROS_LIBRARIES "")
    foreach(ros_pkg ${ARGV})
        find_package("${ros_pkg}" REQUIRED)
        list(APPEND ROS_INCLUDE_DIRS "${${ros_pkg}_INCLUDE_DIRS}")
        list(APPEND ROS_LIBRARIES "${${ros_pkg}_LIBRARIES}")
    endforeach()
endmacro()

find_ros(rclcpp gazebo_msgs auna_its_msgs auna_msgs etsi_its_msgs)
target_include_directories(ros2 PUBLIC ${ROS_INCLUDE_DIRS})
target_link_libraries(ros2 PUBLIC ${ROS_LIBRARIES})

find_package(PkgConfig MODULE REQUIRED)
pkg_check_modules(ZEROMQ REQUIRED libzmq)
target_include_directories(ros2 PUBLIC "${ZEROMQ_INCLUDE_DIRS}")
target_link_libraries(ros2 PUBLIC "${ZEROMQ_LIBRARIES}")

install(TARGETS ros2 LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION ${CMAKE_INSTALL_DATADIR}/ned/ros2 FILES_MATCHING PATTERN "*.ned")
set_property(TARGET ros2 APPEND PROPERTY INSTALL_NED_FOLDERS ${CMAKE_INSTALL_DATADIR}/ned/ros2)

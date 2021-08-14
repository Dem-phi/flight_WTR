cmake_minimum_required(VERSION 2.8.3)
project(main_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -03)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3") #c++11
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -march=native --fast-math -ffast-math -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)

find_package(catkin REQUIRED COMPONENTS
        #recognition
        # serial
        roscpp
        rospy
        std_msgs
        # image_transport
        cv_bridge
        geometry_msgs
        nav_msgs
        mavros_msgs
        tf
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(jsoncpp REQUIRED)


set(THIRD_PARTY_LIBS
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include/StateWorkerInclude
        ${CMAKE_CURRENT_SOURCE_DIR}/include/MathTools
        ${CMAKE_CURRENT_SOURCE_DIR}/include/MsgJar
        ${CMAKE_CURRENT_SOURCE_DIR}/include/Reference
)
catkin_package(

)

add_executable(rs_remap
        App/realsenseRemap.cpp)
target_link_libraries(rs_remap
        ${THIRD_PARTY_LIBS}
        )


add_executable(show_status
        App/showStatus.cpp)
target_link_libraries(show_status
        ${THIRD_PARTY_LIBS}
        )

add_executable(circle
        App/test_circle.cpp)
target_link_libraries(circle
        ${THIRD_PARTY_LIBS}
        )

add_executable(demo
        src/demo.cpp)
target_link_libraries(demo
        ${THIRD_PARTY_LIBS}
        ${catkin_LIBRARIES}
)


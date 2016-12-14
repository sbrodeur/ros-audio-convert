cmake_minimum_required(VERSION 2.8.3)

project(audio_convert)

find_package(catkin REQUIRED COMPONENTS roscpp rosbag audio_common_msgs)

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 gstreamer-plugins-base-1.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread signals program_options)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS})

catkin_package()

add_executable(bag2wav src/bag2wav.cpp)
target_link_libraries(bag2wav ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} gstapp-1.0 ${Boost_LIBRARIES}) 
add_dependencies(bag2wav ${catkin_EXPORTED_TARGETS})

install(TARGETS bag2wav 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


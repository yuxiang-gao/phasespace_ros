cmake_minimum_required(VERSION 2.8.3)
project(phasespace_ros)

add_compile_options(-std=c++11)
#if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
#  set(CMAKE_BUILD_TYPE Release)
#endif()

find_package(catkin_simple REQUIRED)

catkin_simple()

# cs_add_library(my_lib src/my_lib.cpp)
link_directories(${catkin_LIBRARY_DIRS} bin)

add_library(owlsock SHARED IMPORTED)
set_target_properties(owlsock PROPERTIES IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/bin/libowlsock.so)

cs_add_executable(${PROJECT_NAME}_node src/phasespace_node.cpp)
target_link_libraries(${PROJECT_NAME}_node owlsock)

cs_install()

# catkin_python_setup()

# cs_install_scripts(scripts/)

cs_export()

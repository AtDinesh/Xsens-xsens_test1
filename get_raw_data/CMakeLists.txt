cmake_minimum_required(VERSION 2.8.3)
project(xsens_test1)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

#file(
#        GLOB
#        source_files
#        ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/*.cpp
#)

#file(
#        GLOB
#        header_files
#        ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/src/*.h
#)


include_directories(
  /home/datchuth/local_libs/xsens/include/
  /home/datchuth/local_libs/xsens/public/include/
  /usr/include/eigen3/
)



## Declare a C++ library
add_library(deviceClass
  /home/datchuth/local_libs/xsens/examples/mtsdk/src_cpp/deviceclass.cpp
  /home/datchuth/local_libs/xsens/examples/mtsdk/src_cpp/deviceclass.h
)

add_library(conio
  /home/datchuth/local_libs/xsens/examples/mtsdk/src_cpp/conio.c
)


SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -std=gnu++11")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(mti ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(xsens_test1
main.cpp
deviceclass.h
deviceclass.cpp
conio.h)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(mti_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(xsens_test1
   deviceClass
   conio
   /home/datchuth/local_libs/xsens/lib64/libxsensdeviceapi.so
   /home/datchuth/local_libs/xsens/lib64/libxstypes.so
   /home/datchuth/local_libs/xsens/public/xcommunication/libxcommunication.a
   /home/datchuth/local_libs/xsens/public/xstypes/libxstypes.a
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS mti mti_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mti.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

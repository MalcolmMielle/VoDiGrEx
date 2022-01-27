#for debug : cmake -DCMAKE_BUILD_TYPE=Debug /path/to/source


cmake_minimum_required(VERSION 2.6)
project(vodigrex)


add_definitions(-std=c++11)

SET(CMAKE_CXX_FLAGS "-Wall")

# Version for all the file in debug have TIMED defined in debug mode
SET(CMAKE_CXX_FLAGS_DEBUG  "-DDEBUG -g")
#Version for the same thing but for one target
# target_compile_definitions(Probabilitieslib PUBLIC $<$<CONFIG:Debug>:TIMED>)

find_package( OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package( bettergraph REQUIRED)

find_package( Boost REQUIRED )
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()  
endif()

include_directories("${PROJECT_BINARY_DIR}" ${CMAKE_CURRENT_BINARY_DIR}  
	include include/voronoidiagram/ include/linefollower/
  )

add_library(EVGthin SHARED
			include/Evg_thin/evg-thin.cc
			)

add_library(Voronoi SHARED
			include/voronoidiagram/ThinkerVoronoi.cpp
			include/voronoidiagram/ThinkerEVG.cpp
			)
target_link_libraries(Voronoi ${OpenCV_LIBS})

add_library(LineFollower SHARED
			include/linefollower/LineFollower.cpp
			)
target_link_libraries(Voronoi ${OpenCV_LIBS})

add_executable(vodigrex main.cpp)

add_executable(test_line_follower Test/test_line_follower.cpp)
target_link_libraries(test_line_follower ${OpenCV_LIBS} Voronoi LineFollower)

add_executable(test_line_graph Test/test_line_graph.cpp)
target_link_libraries(test_line_graph ${OpenCV_LIBS} Voronoi LineFollower)

add_executable(test_graph_corners Test/test_graph_corner.cpp)
target_link_libraries(test_graph_corners ${OpenCV_LIBS} Voronoi LineFollower)

add_executable(test_line_doors Test/test_line_doors.cpp)
target_link_libraries(test_line_doors ${OpenCV_LIBS} Voronoi LineFollower)

add_executable(test_multi Test/test_multi.cpp)
target_link_libraries(test_multi ${OpenCV_LIBS} Voronoi LineFollower)

add_executable(test_multi_doors Test/test_multi_doors.cpp)
target_link_libraries(test_multi_doors ${OpenCV_LIBS} Voronoi LineFollower)

add_executable(partial_image Test/partial_image_voronoi.cpp)
target_link_libraries(partial_image ${OpenCV_LIBS} Voronoi LineFollower)

add_executable(test_evacuation Test/test_evacuation.cpp)
target_link_libraries(test_evacuation ${OpenCV_LIBS} Voronoi LineFollower)

add_executable(test_building Test/test_building.cpp)
target_link_libraries(test_building ${OpenCV_LIBS} Voronoi LineFollower)

add_subdirectory(include)
add_subdirectory(Test)


# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

## Installing the files

## Installing the files

FILE(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")
INSTALL(FILES ${files} DESTINATION include/vodigrex)

FILE(GLOB files_linef "${CMAKE_CURRENT_SOURCE_DIR}/include/linefollower/*.hpp" "${CMAKE_CURRENT_SOURCE_DIR}/include/linefollower/*.tpl")
INSTALL(FILES ${files_linef} DESTINATION include/vodigrex/linefollower)

FILE(GLOB files_voro "${CMAKE_CURRENT_SOURCE_DIR}/include/voronoidiagram/*.hpp")
INSTALL(FILES ${files_voro} DESTINATION include/vodigrex/voronoidiagram)

FILE(GLOB files_utils "${CMAKE_CURRENT_SOURCE_DIR}/include/utils/*.hpp")
INSTALL(FILES ${files_utils} DESTINATION include/vodigrex/utils)

FILE(GLOB files_evg "${CMAKE_CURRENT_SOURCE_DIR}/include/Evg_thin/*.hh")
INSTALL(FILES ${files_evg} DESTINATION include/vodigrex/Evg_thin)

INSTALL(FILES vodigrex-config.cmake DESTINATION lib/vodigrex)
INSTALL(TARGETS Voronoi DESTINATION lib/vodigrex)
INSTALL(TARGETS LineFollower DESTINATION lib/vodigrex)
INSTALL(TARGETS EVGthin DESTINATION lib/vodigrex)

install(TARGETS vodigrex RUNTIME DESTINATION bin)
install(TARGETS test_line_follower RUNTIME DESTINATION bin)


## If rpm wanted

SET(CPACK_GENERATOR "RPM")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Malcolm Mielle") #required
 
INCLUDE(CPack)


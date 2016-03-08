cmake_minimum_required(VERSION 2.6)
project(vodigrex)

SET(CMAKE_CXX_FLAGS "-Wall")

# Version for all the file in debug have TIMED defined in debug mode
SET(CMAKE_CXX_FLAGS_DEBUG  "-DDEBUG -g")
#Version for the same thing but for one target
# target_compile_definitions(Probabilitieslib PUBLIC $<$<CONFIG:Debug>:TIMED>)

find_package( OpenCV REQUIRED)

find_package( Boost REQUIRED )
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()  
endif()

include_directories("${PROJECT_BINARY_DIR}" ${CMAKE_CURRENT_BINARY_DIR}  
	includes includes/VoronoiDiagram/ includes/LineFollower/
)

add_library(VoronoiLib 
			includes/VoronoiDiagram/Thinker_Voronoi.cpp
# 			includes/VoronoiDiagram/Thinker_EVG.cpp
			)
target_link_libraries(VoronoiLib ${OpenCV_LIBS})

add_library(LineFollowerLib 
			includes/LineFollower/LineFollower.cpp
			)
target_link_libraries(VoronoiLib ${OpenCV_LIBS})

add_executable(vodigrex main.cpp)

add_executable(test_line_follower Test/test_line_follower.cpp)
target_link_libraries(test_line_follower ${OpenCV_LIBS} VoronoiLib LineFollowerLib)

add_executable(test_line_graph Test/test_line_graph.cpp)
target_link_libraries(test_line_graph ${OpenCV_LIBS} VoronoiLib LineFollowerLib)

add_executable(test_line_doors Test/test_line_doors.cpp)
target_link_libraries(test_line_doors ${OpenCV_LIBS} VoronoiLib LineFollowerLib)

install(TARGETS vodigrex RUNTIME DESTINATION bin)
install(TARGETS test_line_follower RUNTIME DESTINATION bin)

add_subdirectory(includes)
add_subdirectory(Test)
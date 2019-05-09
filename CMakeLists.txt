cmake_minimum_required(VERSION 3.5)

# Options. Turn on with 'cmake -Druntests=ON'.
option(runtests "Build all tests." OFF) # Makes boolean 'test' available.

project(dynamic_obstacle_avoidance)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(
	Eigen3 REQUIRED
	armadillo REQUIRED
)

include_directories(
  	include
  	${Eigen3_INCLUDE_DIRS}
  	${armadillo_INCLUDE_DIRS}
)

set(CORE_SOURCES
	src/Obstacle/Obstacle.cpp
 	src/Obstacle/Ellipsoid2D.cpp
 	src/Obstacle/LaserScan2D.cpp
 	src/State/Pose.cpp
 	src/State/State.cpp
	src/Modulation.cpp
  	src/Agent.cpp
)

add_library(${PROJECT_NAME} SHARED ${CORE_SOURCES})

target_link_libraries(${PROJECT_NAME} armadillo)

install(DIRECTORY include/
  	DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  	ARCHIVE DESTINATION lib
  	LIBRARY DESTINATION lib
  	RUNTIME DESTINATION bin)

if (runtests)
	if (APPLE)
    	add_definitions(-DGTEST_USE_OWN_TR1_TUPLE)
    	add_definitions(-D__GLIBCXX__)
  	endif (APPLE)

  	add_subdirectory(lib/googletest)

    enable_testing()

  	# Include the gtest library. gtest_SOURCE_DIR is available due to
  	# 'project(gtest)' above.
  	include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
	
	add_executable(runTestModulation tests/testModulation.cpp)
	target_link_libraries(runTestModulation
		gtest 
		gtest_main
		${PROJECT_NAME}
	)

	add_executable(runTestObstacle tests/testEllipsoid2DObstacle.cpp)
	target_link_libraries(runTestObstacle
		gtest 
		gtest_main
		${PROJECT_NAME}
	)

	add_executable(runTestState tests/testState.cpp)
	target_link_libraries(runTestState
		gtest 
		gtest_main
		${PROJECT_NAME}
	)

	add_test(NAME runTestModulation COMMAND runTestModulation)
	add_test(NAME runTestObstacle COMMAND runTestObstacle)
	add_test(NAME runTestState COMMAND runTestState)
endif()
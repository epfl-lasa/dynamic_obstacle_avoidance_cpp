cmake_minimum_required(VERSION 3.9)

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

add_subdirectory(lib/dlib dlib_build)
add_subdirectory(lib/libsvm libsvm_build)
add_subdirectory(lib/SVMGrad SVMGrad_build)
add_subdirectory(lib/limbo limbo_build)

find_package(
	Eigen3 REQUIRED
	armadillo REQUIRED
	mlpack REQUIRED
)

find_package(PythonLibs 3.6)
find_package(OpenMP)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.45.0 REQUIRED COMPONENTS system filesystem thread unit_test_framework)

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

include_directories(
  	include
  	lib/SVMGrad/include
  	lib/libsvm/include
  	lib/limbo/src
  	${EIGEN3_INCLUDE_DIR}
  	${armadillo_INCLUDE_DIRS}
  	${mlpack_INCLUDE_DIRS}
  	${PYTHON_INCLUDE_DIRS}
)

set(CORE_SOURCES
	src/Obstacle/Obstacle.cpp
 	src/Obstacle/Ellipsoid.cpp
 	src/Obstacle/Aggregate.cpp
 	src/Obstacle/PointCloudToObstacle.cpp
 	src/Obstacle/StarShapeHull.cpp
 	src/State/Pose.cpp
 	src/State/State.cpp
	src/Modulation.cpp
  	src/Agent.cpp
  	src/Utils/MathTools.cpp
  	src/Utils/Plotting/PlottingTools.cpp
  	src/Utils/ObstacleGeneration/Aggregation.cpp
)

add_library(${PROJECT_NAME} SHARED ${CORE_SOURCES})

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX armadillo mlpack ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} limbo libsvm svm_grad dlib::dlib)
else()
	target_link_libraries(${PROJECT_NAME} armadillo mlpack ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} limbo libsvm svm_grad dlib::dlib)
endif()


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

	add_executable(runTestObstacle tests/testEllipsoidObstacle.cpp)
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

	add_executable(runTestPointCloudToObstacle tests/testPointCloudToObstacle.cpp)
	target_link_libraries(runTestPointCloudToObstacle
		gtest 
		gtest_main
		${PROJECT_NAME}
	)

	add_test(NAME runTestModulation COMMAND runTestModulation)
	add_test(NAME runTestObstacle COMMAND runTestObstacle)
	add_test(NAME runTestState COMMAND runTestState)
	add_test(NAME runTestPointCloudToObstacle COMMAND runTestPointCloudToObstacle)
endif()

add_executable(runTestOptimization tests/testOptimization.cpp)
target_link_libraries(runTestOptimization
	${PROJECT_NAME}
)

add_executable(runTestConfiguration tests/testConfiguration.cpp)
target_link_libraries(runTestConfiguration
	${PROJECT_NAME}
)

add_executable(runTestSingleConfiguration tests/testSingleConfiguration.cpp)
target_link_libraries(runTestSingleConfiguration
	${PROJECT_NAME}
)

add_executable(runTestWall tests/testWall.cpp)
target_link_libraries(runTestWall
	${PROJECT_NAME}
)

add_executable(runTest3D tests/test3D.cpp)
target_link_libraries(runTest3D
	${PROJECT_NAME}
)

add_executable(runTestKIT tests/testKIT.cpp)
target_link_libraries(runTestKIT
	${PROJECT_NAME}
)

add_executable(runTestMovingObstacle tests/testMovingObstacle.cpp)
target_link_libraries(runTestMovingObstacle
	${PROJECT_NAME}
)
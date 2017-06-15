project(StereoscopicPi C CXX)
cmake_minimum_required(VERSION 2.8)

SET(EXECNAME "stereoPi")
SET(BUILD_GPIO_WRAPPER TRUE)
SET(BUILD_STEREOPI TRUE)

IF(WIN32) # Windows
	
ELSE() # Linux
	#Include Directories

	#Link Libraries

	#Package check

	#Package Search

#	SET(Eigen_INCLUDE_DIRS "/usr/include/eigen3")
ENDIF()

# # Mandatory Required Libraries

# #Include Header File Directory


# #Linker LIB Directory

# #Compile Definition
ADD_DEFINITIONS(-lwiringPi)

IF(BUILD_GPIO_WRAPPER)
	#Find Source and Header Files
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/GPIOWrapper/include)
	SET(GPIO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/GPIOWrapper/src")
	FILE(GLOB_RECURSE GPIO_SRCS "${GPIO_SRC_DIR}/*.cpp")
ENDIF()

IF(BUILD_STEREOPI)
	#Find Source and Header Files
	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
	SET(STEREOPI_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
	FILE(GLOB_RECURSE STEREOPI_SRCS "${STEREOPI_SRC_DIR}/*.cpp")

	#Build 
	ADD_EXECUTABLE("${EXECNAME}" ${GPIO_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/test_camera.cpp)
	set_property(TARGET ${EXECNAME} PROPERTY OUTPUT_NAME ${EXECNAME})

	# Add the required libraries for linking:
	TARGET_LINK_LIBRARIES(${EXECNAME}
		wiringPi
	)

ENDIF()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3" ) # Release Perf mode

cmake_minimum_required(VERSION 2.8)
project(grabcut_3d)

find_package(catkin REQUIRED roscpp pcl_ros)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp pcl_ros
               DEPENDS OpenCV PCL
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)

#catkin_python_setup()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
			   ${PCL_INCLUDE_DIRS}
)
include_directories(include)

add_executable(grabcut_3d_exe src/grabcut_3d.cpp src/grabcut_3d_app.cpp)
target_link_libraries(grabcut_3d_exe ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_library(${PROJECT_NAME} src/grabcut_3d.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
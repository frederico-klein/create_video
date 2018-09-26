cmake_minimum_required(VERSION 2.8.3)
project(create_video)

catkin_package()

find_package(catkin REQUIRED COMPONENTS
    roslint
    cv_bridge
    image_transport
    message_filters
    message_generation
    rosconsole
    roscpp

    )

find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)

include_directories(${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
)

roslint_cpp()

add_executable(video_recorder src/video_recorder.cpp)
target_link_libraries(video_recorder ${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)

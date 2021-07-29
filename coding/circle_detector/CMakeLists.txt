cmake_minimum_required(VERSION 3.0.2)

# Create ${PROJECT_NAME} ${PROJECT_SOURCE_DIR}
project(PROJECT_NAME)

# use this if you wanna use debug
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -g)

# Set executable output path
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE})
# Add child_dir if you need
#add_subdirectory(./src)
#add_subdirectory(./include)

# Set include path ( contains lib include path)
include_directories(
    ./include
)

# Set lib path
#link_directories({lib_path})

add_executable(${PROJECT_NAME}
    ./src/main.cpp
)

# Set link libs if you need
target_link_libraries(${PROJECT_NAME}
   ${OpenCV_LIBRARIES}
)


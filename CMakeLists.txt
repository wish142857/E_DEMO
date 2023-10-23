cmake_minimum_required(VERSION 2.8.3)
project(e_demo)

# --- Set ---
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3")

# --- Find ---
find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)

# --- 1. CS Init ---
catkin_simple(ALL_DEPS_REQUIRED)


# --- 2. CS Build ---
cs_add_executable(hello_node
    src/HelloNode.cpp
)

target_link_libraries(hello_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cs_add_executable(e_demo_time_surface
    src/TimeSurfaceNode.cpp
)

target_link_libraries(e_demo_time_surface
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

# --- 3. CS Install ---
cs_install()

# --- 4. CS export ---
cs_export()

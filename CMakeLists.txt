cmake_minimum_required(VERSION 3.0.0)
project(PhantomOmni VERSION 0.1.0 LANGUAGES C CXX)

include_directories(
    /usr/include
    /usr/local/include
    /usr/include/HDU
    /usr/include/HD
    ${PROJECT_SOURCE_DIR}/include
)
link_directories(
    /usr/lib
    /usr/local/lib
)

add_executable(omni src/main.cpp)

target_link_libraries(omni HD HDU rt ncurses pthread)
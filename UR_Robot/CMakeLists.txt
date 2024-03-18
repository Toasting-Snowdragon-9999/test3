cmake_minimum_required(VERSION 3.5)

project(UR_Robot LANGUAGES CXX)

find_package(ur_rtde REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(UR_Robot main.cpp)

target_link_libraries(UR_Robot PRIVATE ur_rtde::rtde)

include(GNUInstallDirs)
install(TARGETS UR_Robot
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

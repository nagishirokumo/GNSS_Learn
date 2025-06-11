cmake_minimum_required(VERSION 3.30)
project(POS_NAV_Project)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static -static-libgcc -static-libstdc++")

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(D:/Learning/202409-202506/GNSS_Code/Dependence/eigen-3.4.0)


file(GLOB SRC_LIST ./src/*.cpp)
file(GLOB INCLUDE_LIST ./include/*.h)

add_executable(POS_NAV_Project ${SRC_LIST}
        ${INCLUDE_LIST}
)
target_link_libraries(POS_NAV_Project PRIVATE Ws2_32)
add_custom_command(TARGET POS_NAV_Project POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/resources $<TARGET_FILE_DIR:POS_NAV_Project>/resources
)


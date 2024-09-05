find_package(Eigen3 CONFIG REQUIRED)

add_executable(assignment-2 main.cpp
        ../math/include/math/math.h)

target_link_libraries(assignment-2 PUBLIC math PRIVATE Eigen3::Eigen)

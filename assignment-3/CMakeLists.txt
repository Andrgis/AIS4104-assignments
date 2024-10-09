find_package(Eigen3 CONFIG REQUIRED)

add_executable(assignment-3 main.cpp
        ../math/include/math/math.h)

target_link_libraries(assignment-3 PUBLIC math PRIVATE Eigen3::Eigen)

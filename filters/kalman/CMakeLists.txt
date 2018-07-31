add_library(kalman SHARED kalman.h kalman.cpp)

target_link_libraries(kalman ${EIGEN3_INCLUDE_DIR})

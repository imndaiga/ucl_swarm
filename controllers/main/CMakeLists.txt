add_library(main SHARED main.h main.cpp)

target_link_libraries(main
  pso
  aco
  kalman
  ${EIGEN3_INCLUDE_DIR}
  argos3core_simulator
  argos3plugin_simulator_eyebot
  argos3plugin_simulator_spiri
  argos3plugin_simulator_genericrobot)

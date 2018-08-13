add_library(lawn MODULE lawn.h lawn.cpp)
target_link_libraries(lawn
  kalman
  ${EIGEN3_INCLUDE_DIR}
  argos3core_simulator
  argos3plugin_simulator_eyebot
  argos3plugin_simulator_spiri
  argos3plugin_simulator_genericrobot)

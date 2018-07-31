add_library(eyebot_pso MODULE eyebot_pso.h eyebot_pso.cpp)

target_link_libraries(eyebot_pso
  pso
  kalman
  ${EIGEN3_INCLUDE_DIR}
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_eyebot
  argos3plugin_simulator_spiri)

add_library(eyebot_pso MODULE eyebot_pso.h eyebot_pso.cpp)

target_link_libraries(eyebot_pso
  argos3core_simulator
  argos3plugin_simulator_eyebot
  argos3plugin_simulator_spiri
  argos3plugin_simulator_genericrobot)

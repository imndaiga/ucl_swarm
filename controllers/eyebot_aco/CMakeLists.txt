add_library(eyebot_aco MODULE eyebot_aco.h eyebot_aco.cpp)

target_link_libraries(eyebot_aco
  aco
  argos3core_simulator
  argos3plugin_simulator_eyebot
  argos3plugin_simulator_spiri
  argos3plugin_simulator_genericrobot)

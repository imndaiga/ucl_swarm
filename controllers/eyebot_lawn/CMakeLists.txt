add_library(eyebot_lawn MODULE eyebot_lawn.h eyebot_lawn.cpp)
target_link_libraries(eyebot_lawn
  argos3core_simulator
  argos3plugin_simulator_eyebot
  argos3plugin_simulator_spiri
  argos3plugin_simulator_genericrobot)

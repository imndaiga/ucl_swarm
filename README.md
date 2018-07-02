# An Evaluation of Swarm Algorithms

This code accompanies my thesis project on Swarm Algorithms.

## Requirements
Tested on both Linux (Ubuntu 16.04) and Windows (Windows Linux Subsystem). The latter requires an X-windows server installed on the windows host (e.g. [XMing](https://sourceforge.net/projects/xming/)).
To successfully run, you need the following software installed:
- [Argos3](https://www.argos-sim.info)
- [Buzz](https://the.swarming.buzz)

Installation steps can be found [here](https://github.com/MISTLab/Buzz).

Ensure that you've set the ARGOS_PLUGIN_PATH environment to point to your Argos Library build directory as well as BUZZ_INCLUDE_PATH to the installed Buzz include parent directory. Compile the buzz script with the ```bzzc swarm.bzz``` command.

|Platform| Compilation command |
|:---|:---|
|TelosB 
  | make TARGET=sky |
|DPP-cc430 
  | make TARGET=dpp |

This directory contains an re-implementation of the LWB protocol, meant to be as similar as possible as the descritption from 
> C. Sarkar, R. V. Prasad, R. T. Rajan, and K. Langendoen  
*Sleeping Beauty: Efficient Communication for Node Scheduling*,
2016 IEEE 13th International Conference on Mobile Ad Hoc and Sensor Systems (MASS)
[Direct link](https://ieeexplore.ieee.org/abstract/document/7815012/) description.

The round period with a round period of 10s. All source nodes send a request to get a slot assigned to send their data.

After 3 rounds without any new request, the host signals the end of the bootstrapping phase (of Sleeping Beauty).
The host then select a set of nodes that should remain active and send their data at every round. All other nodes go to sleep until the end of the scheduling period, which is 10-round long (ie 100s).

All others configuration parameters of Sleeping Beauty have been set according to [the original paper](https://ieeexplore.ieee.org/abstract/document/7815012/) description.
All values are in `project-conf.h` (`#define SB_xxx`).
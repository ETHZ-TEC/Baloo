|Platform| Compilation command |
|:---|:---|
|TelosB   | make TARGET=sky |
|DPP-cc430   | make TARGET=dpp |

This directory contains an re-implementation of the Crystal protocol, meant to be as similar as possible as the descritption from 
> *Interference-resilient Ultra-low Power Aperiodic Data Collection*,  
T. Istomin, M. Trobinger, A. L. Murphy, and G. P. Picco,
Proceedings of the 17th ACM/IEEE International Conference on Information Processing in Sensor Networks (IPSN).  
[Direct Link](https://doi.org/10.1109/IPSN.2018.00015)


The epoch length is set to 2s. Source nodes start generating packets from the 11th epoch onwards (controlled by the `CRYSTAL_START_EPOCH` parameter).

The protocol uses channel hopping by default. The channel is selected from the `channel_array[]` following a predifined sequence based on the epoch and TA pair counters. See sequence definition in the `get_channel_epoch()` and `get_channel_epoch_ta()` functions.

The protocol uses the interference detection feature
  - Controlled by the `GMW_CONF_USE_NOISE_DETECTION` parameter.
  - Settings: -60 dBm threshold, 80 threshold crossing in one slot to return *high noise*

At each epoch, a fixed number of nodes generate and try to send a packet.
  - Supported values: 0, 1 and 20
  - Controlled by the `CRYSTAL_NB_CONCURRENT_SENDER` parameter
  - The actual nodes sending in one epoch is defined by the `sndtbl` array, staticly defined are used for all Crystal test (see `sndtbl.c`).

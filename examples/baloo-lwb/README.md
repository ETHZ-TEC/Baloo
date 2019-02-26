|Platform| Compilation command |
|:---|:---|
|TelosB 
  | make TARGET=sky |
|DPP-cc430 
  | make TARGET=dpp |

This directory contains an re-implementation of the LWB protocol, meant to be as similar as possible as the descritption from 
> *Low-power Wireless Bus*,  
F. Ferrari, M. Zimmerling, L. Mottola, and L. Thiele  
Proceedings of the 10th ACM Conference on Embedded Network Sensor Systems (SenSys)
[Direct Link](doi.acm.org/10.1145/2426656.2426658)

All source nodes request one stream to the host at bootstrap. The stream inter-packet interval (IPI) is controlled by the `SOURCE_IPI` parameter in `project-conf.h`
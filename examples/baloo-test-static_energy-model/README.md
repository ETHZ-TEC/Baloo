|Platform| Compilation command |
|:---|:---|
|TelosB 
  | make TARGET=sky |
|DPP-cc430 
  | make TARGET=dpp |

This project evaluate the benefits of having communication rounds (vs no rounds), in a static framework (like TTW). The goal is to experimentally validate the latency and energy models for a Glossy-based NET layer protocol.

Written with the dpp in mind, but should also work on for the sky mote (the timing or some operations might be different though).
